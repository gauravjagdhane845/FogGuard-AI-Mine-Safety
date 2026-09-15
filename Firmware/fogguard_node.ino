/*
 * ==========================================================
 *  FogGuard — ESP32 Vehicle Node Firmware
 *  SIH 2026 | PS 26007
 * ==========================================================
 *
 *  ROLE OF THIS BOARD:
 *  Mounted on each dumper / light vehicle / pedestrian tag.
 *  Responsibilities:
 *    1. Read own GPS position (NEO-6M / NEO-M8N)
 *    2. Read local visibility sensor (analog dust/fog sensor)
 *    3. Compute a local Fog Severity Index (0-5)
 *    4. Broadcast own position + severity over LoRa mesh (V2V)
 *    5. Receive peers' broadcasts, compute proximity risk
 *    6. Drive a speed-limit relay + buzzer + status LEDs
 *    7. Forward telemetry to the Raspberry Pi gateway over UART
 *
 *  WIRING (ESP32 DevKit v1):
 *  ----------------------------------------------------------
 *  LoRa SX1276 (RA-02) :  SCK=18  MISO=19  MOSI=23  NSS=5
 *                         RST=14  DIO0=26   VCC=3V3  GND=GND
 *  GPS NEO-6M          :  TX->GPIO16(RX2)   RX->GPIO17(TX2)
 *                         VCC=3V3  GND=GND
 *  Visibility sensor   :  AOUT->GPIO34 (ADC1_CH6, input only)
 *  Speed-limit relay   :  IN->GPIO25
 *  Buzzer              :  +->GPIO27
 *  LED green (safe)    :  GPIO32  (via 220R)
 *  LED amber (caution) :  GPIO33  (via 220R)
 *  LED red (danger)    :  GPIO4   (via 220R)
 *  UART to Raspberry Pi:  TX=GPIO1 / RX=GPIO3 (Serial0, USB)
 *
 *  LIBRARIES REQUIRED (Arduino Library Manager):
 *    - LoRa            by Sandeep Mistry
 *    - TinyGPSPlus     by Mikal Hart
 *    - ArduinoJson     by Benoit Blanchon  (v6.x)
 * ==========================================================
 */

#include <SPI.h>
#include <LoRa.h>
#include <TinyGPSPlus.h>
#include <ArduinoJson.h>

// ---------------- Vehicle identity ----------------
// CHANGE THIS PER BOARD:  "D-07", "D-12", "LV-01", "PED-01"
#define VEHICLE_ID    "D-07"
#define VEHICLE_TYPE  "DUMPER"     // DUMPER | LIGHT | PEDESTRIAN

// ---------------- Pin map ----------------
#define LORA_SCK      18
#define LORA_MISO     19
#define LORA_MOSI     23
#define LORA_NSS       5
#define LORA_RST      14
#define LORA_DIO0     26

#define GPS_RX_PIN    16    // ESP32 receives GPS TX here
#define GPS_TX_PIN    17

#define PIN_VISIBILITY 34   // ADC input-only pin
#define PIN_RELAY      25
#define PIN_BUZZER     27
#define PIN_LED_GREEN  32
#define PIN_LED_AMBER  33
#define PIN_LED_RED     4

// ---------------- Radio config ----------------
// Use 433E6 in India (ISM band). 865-867MHz also permitted.
#define LORA_FREQ       433E6
#define LORA_TX_POWER   17      // dBm (2..20)
#define LORA_SF         9       // spreading factor 7..12
#define LORA_BW         125E3
#define LORA_SYNC_WORD  0xF3    // network id, must match all nodes

// ---------------- Timing ----------------
#define BROADCAST_INTERVAL_MS   1000   // V2V position broadcast rate
#define UART_REPORT_INTERVAL_MS 2000   // telemetry to Raspberry Pi
#define PEER_TIMEOUT_MS         8000   // drop peers not heard from
#define MAX_PEERS               12

// ---------------- Safety thresholds ----------------
// DGMS: available visibility must be >= 3x stopping distance.
// Stopping distance = reaction distance + braking distance:
//    d(v_ms) = v_ms * T_REACTION + v_ms^2 / (2 * DECEL)
#define T_REACTION_S         1.5f    // operator reaction time
#define DECEL_MS2            2.0f    // loaded haul truck on dirt road
#define VISIBILITY_MARGIN    3.0f    // DGMS 3x rule
#define ABS_MAX_SPEED_KMH    40.0f   // mine speed limit ceiling
#define PROXIMITY_WARN_M     80.0f   // warn if peer closer than this
#define PROXIMITY_CRIT_M     40.0f   // critical if peer closer than this

// ==========================================================
//  Globals
// ==========================================================
TinyGPSPlus gps;
HardwareSerial GPSSerial(2);   // UART2 for GPS

struct Peer {
  char     id[10];
  double   lat;
  double   lon;
  uint8_t  severity;
  float    speed;
  uint32_t lastSeen;
  bool     active;
};

Peer peers[MAX_PEERS];

double   myLat = 0.0, myLon = 0.0;
float    mySpeedKmh = 0.0;
bool     gpsValid = false;

float    visibilityM   = 200.0;   // metres
uint8_t  fogSeverity   = 0;       // 0..5
float    safeSpeedCap  = ABS_MAX_SPEED_KMH;
float    nearestPeerM  = 9999.0;
char     nearestPeerId[10] = "-";
bool     convoyMode    = false;

uint32_t lastBroadcast = 0;
uint32_t lastUartReport = 0;

// ==========================================================
//  Helpers
// ==========================================================

/* Haversine distance in metres between two lat/lon points */
double haversineM(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(radians(lat1)) * cos(radians(lat2)) *
             sin(dLon / 2) * sin(dLon / 2);
  return R * 2 * atan2(sqrt(a), sqrt(1 - a));
}

/*
 * Read the visibility sensor and convert to metres.
 * A GP2Y1010 / SHARP dust sensor or a simple IR transmissometer
 * gives an analog voltage that rises as particulate density rises.
 * Calibrate MIN/MAX on-site against a known reference.
 */
float readVisibilityMetres() {
  const int SAMPLES = 12;
  uint32_t acc = 0;
  for (int i = 0; i < SAMPLES; i++) {
    acc += analogRead(PIN_VISIBILITY);   // 0..4095 on ESP32
    delayMicroseconds(400);
  }
  float raw = (float)acc / SAMPLES;

  // Site calibration constants — MEASURE THESE AT THE MINE.
  const float RAW_CLEAR = 300.0;    // clear air reading
  const float RAW_DENSE = 3200.0;   // dense fog reading

  float t = (raw - RAW_CLEAR) / (RAW_DENSE - RAW_CLEAR);
  t = constrain(t, 0.0f, 1.0f);

  // Map linearly onto 250m (clear) .. 5m (dense fog)
  float vis = 250.0f - t * (250.0f - 5.0f);

  // Exponential smoothing to reject sensor jitter
  static float smoothed = 200.0f;
  smoothed = 0.85f * smoothed + 0.15f * vis;
  return smoothed;
}

/* Map visibility (m) to the FogGuard 0-5 Severity Index */
uint8_t computeSeverity(float visM) {
  if (visM >= 150.0) return 0;
  if (visM >= 100.0) return 1;
  if (visM >=  60.0) return 2;
  if (visM >=  30.0) return 3;
  if (visM >=  15.0) return 4;
  return 5;
}

/*
 * Derive max safe speed by inverting the DGMS visibility rule.
 *
 *   visibility >= MARGIN * ( v_ms*T_REACTION + v_ms^2/(2*DECEL) )
 *
 * With v_ms = v_kmh/3.6 this becomes a quadratic in v_kmh:
 *   A*v^2 + B*v - visibility = 0
 *   A = MARGIN / (3.6^2 * 2 * DECEL)
 *   B = MARGIN * T_REACTION / 3.6
 * Take the positive root.
 */
float computeSafeSpeed(float visM) {
  if (visM <= 0.0f) return 0.0f;

  const float A = VISIBILITY_MARGIN / (3.6f * 3.6f * 2.0f * DECEL_MS2);
  const float B = VISIBILITY_MARGIN * T_REACTION_S / 3.6f;

  float disc = B * B + 4.0f * A * visM;
  float v = (-B + sqrt(disc)) / (2.0f * A);
  return constrain(v, 0.0f, ABS_MAX_SPEED_KMH);
}

/* Clear stale peers we have not heard from recently */
void expirePeers() {
  uint32_t now = millis();
  for (int i = 0; i < MAX_PEERS; i++) {
    if (peers[i].active && (now - peers[i].lastSeen > PEER_TIMEOUT_MS)) {
      peers[i].active = false;
    }
  }
}

/* Insert or update a peer record */
void upsertPeer(const char* id, double lat, double lon, uint8_t sev, float spd) {
  int freeSlot = -1;
  for (int i = 0; i < MAX_PEERS; i++) {
    if (peers[i].active && strcmp(peers[i].id, id) == 0) {
      peers[i].lat = lat; peers[i].lon = lon;
      peers[i].severity = sev; peers[i].speed = spd;
      peers[i].lastSeen = millis();
      return;
    }
    if (!peers[i].active && freeSlot < 0) freeSlot = i;
  }
  if (freeSlot >= 0) {
    strncpy(peers[freeSlot].id, id, sizeof(peers[freeSlot].id) - 1);
    peers[freeSlot].id[sizeof(peers[freeSlot].id) - 1] = '\0';
    peers[freeSlot].lat = lat; peers[freeSlot].lon = lon;
    peers[freeSlot].severity = sev; peers[freeSlot].speed = spd;
    peers[freeSlot].lastSeen = millis();
    peers[freeSlot].active = true;
  }
}

/* Find the closest active peer; updates nearestPeerM / nearestPeerId */
void evaluateProximity() {
  nearestPeerM = 9999.0;
  strcpy(nearestPeerId, "-");
  if (!gpsValid) return;

  for (int i = 0; i < MAX_PEERS; i++) {
    if (!peers[i].active) continue;
    double d = haversineM(myLat, myLon, peers[i].lat, peers[i].lon);
    if (d < nearestPeerM) {
      nearestPeerM = d;
      strncpy(nearestPeerId, peers[i].id, sizeof(nearestPeerId) - 1);
      nearestPeerId[sizeof(nearestPeerId) - 1] = '\0';
    }
  }
}

// ==========================================================
//  LoRa V2V messaging
// ==========================================================

/* Broadcast our own state to all nearby nodes */
void broadcastState() {
  StaticJsonDocument<192> doc;
  doc["id"]  = VEHICLE_ID;
  doc["t"]   = VEHICLE_TYPE;
  doc["lat"] = myLat;
  doc["lon"] = myLon;
  doc["sev"] = fogSeverity;
  doc["spd"] = mySpeedKmh;
  doc["vis"] = (int)visibilityM;

  char buf[192];
  size_t n = serializeJson(doc, buf, sizeof(buf));

  LoRa.beginPacket();
  LoRa.write((uint8_t*)buf, n);
  LoRa.endPacket();

  LoRa.receive();   // return to continuous receive mode
}

/* Handle an inbound LoRa packet from a peer */
void onLoRaPacket(int packetSize) {
  if (packetSize <= 0 || packetSize > 250) return;

  char buf[256];
  int i = 0;
  while (LoRa.available() && i < (int)sizeof(buf) - 1) {
    buf[i++] = (char)LoRa.read();
  }
  buf[i] = '\0';

  StaticJsonDocument<192> doc;
  if (deserializeJson(doc, buf)) return;   // malformed, ignore

  const char* pid = doc["id"] | "";
  if (strlen(pid) == 0) return;
  if (strcmp(pid, VEHICLE_ID) == 0) return;   // ignore our own echo

  upsertPeer(pid,
             doc["lat"] | 0.0,
             doc["lon"] | 0.0,
             doc["sev"] | 0,
             doc["spd"] | 0.0f);
}

// ==========================================================
//  Actuation — relay, buzzer, LEDs
// ==========================================================
void applyActuation() {
  // Convoy mode engages at severity 4+
  convoyMode = (fogSeverity >= 4);

  // Relay is energised (speed limiter engaged) whenever we are
  // above severity 2, or a peer is critically close.
  bool limiterOn = (fogSeverity >= 2) || (nearestPeerM < PROXIMITY_CRIT_M);
  digitalWrite(PIN_RELAY, limiterOn ? HIGH : LOW);

  // Status LEDs
  digitalWrite(PIN_LED_GREEN, fogSeverity <= 1);
  digitalWrite(PIN_LED_AMBER, fogSeverity == 2 || fogSeverity == 3);
  digitalWrite(PIN_LED_RED,   fogSeverity >= 4);

  // Buzzer: short chirp on proximity warning, continuous on critical
  static uint32_t lastBeep = 0;
  uint32_t now = millis();

  if (nearestPeerM < PROXIMITY_CRIT_M) {
    digitalWrite(PIN_BUZZER, HIGH);               // continuous
  } else if (nearestPeerM < PROXIMITY_WARN_M) {
    if (now - lastBeep > 900) {                   // chirp every 900ms
      digitalWrite(PIN_BUZZER, HIGH);
      delay(60);
      digitalWrite(PIN_BUZZER, LOW);
      lastBeep = now;
    }
  } else {
    digitalWrite(PIN_BUZZER, LOW);
  }
}

// ==========================================================
//  Telemetry to Raspberry Pi (over USB serial / UART0)
// ==========================================================
void reportToGateway() {
  StaticJsonDocument<384> doc;
  doc["id"]        = VEHICLE_ID;
  doc["type"]      = VEHICLE_TYPE;
  doc["lat"]       = myLat;
  doc["lon"]       = myLon;
  doc["gps_ok"]    = gpsValid;
  doc["speed"]     = mySpeedKmh;
  doc["visibility"]= (int)visibilityM;
  doc["severity"]  = fogSeverity;
  doc["safe_speed"]= safeSpeedCap;
  doc["convoy"]    = convoyMode;
  doc["near_id"]   = nearestPeerId;
  doc["near_m"]    = (nearestPeerM > 9000) ? -1 : (int)nearestPeerM;

  JsonArray arr = doc.createNestedArray("peers");
  for (int i = 0; i < MAX_PEERS; i++) {
    if (!peers[i].active) continue;
    JsonObject p = arr.createNestedObject();
    p["id"]  = peers[i].id;
    p["lat"] = peers[i].lat;
    p["lon"] = peers[i].lon;
    p["sev"] = peers[i].severity;
  }

  serializeJson(doc, Serial);
  Serial.println();          // newline delimits records for the Pi
}

// ==========================================================
//  Setup
// ==========================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println(F("# FogGuard node booting..."));

  pinMode(PIN_RELAY,     OUTPUT);
  pinMode(PIN_BUZZER,    OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_AMBER, OUTPUT);
  pinMode(PIN_LED_RED,   OUTPUT);

  digitalWrite(PIN_RELAY,  LOW);
  digitalWrite(PIN_BUZZER, LOW);

  // ADC range 0-3.3V
  analogSetPinAttenuation(PIN_VISIBILITY, ADC_11db);

  // GPS on UART2
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // LoRa on VSPI
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println(F("# ERROR: LoRa init failed. Halting."));
    while (true) {
      digitalWrite(PIN_LED_RED, !digitalRead(PIN_LED_RED));
      delay(250);
    }
  }
  LoRa.setTxPower(LORA_TX_POWER);
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.enableCrc();
  LoRa.onReceive(onLoRaPacket);
  LoRa.receive();

  for (int i = 0; i < MAX_PEERS; i++) peers[i].active = false;

  Serial.print(F("# FogGuard node ready: "));
  Serial.println(VEHICLE_ID);
}

// ==========================================================
//  Main loop
// ==========================================================
void loop() {
  // --- 1. Feed the GPS parser ---
  while (GPSSerial.available()) {
    gps.encode(GPSSerial.read());
  }
  if (gps.location.isValid()) {
    myLat = gps.location.lat();
    myLon = gps.location.lng();
    gpsValid = true;
  }
  if (gps.speed.isValid()) {
    mySpeedKmh = gps.speed.kmph();
  }

  // --- 2. Sense local fog conditions ---
  visibilityM  = readVisibilityMetres();
  fogSeverity  = computeSeverity(visibilityM);
  safeSpeedCap = computeSafeSpeed(visibilityM);

  // --- 3. Maintain peer table + proximity ---
  expirePeers();
  evaluateProximity();

  // --- 4. Drive outputs ---
  applyActuation();

  // --- 5. Broadcast our state over the mesh ---
  uint32_t now = millis();
  if (now - lastBroadcast >= BROADCAST_INTERVAL_MS) {
    lastBroadcast = now;
    broadcastState();
  }

  // --- 6. Report telemetry up to the Raspberry Pi ---
  if (now - lastUartReport >= UART_REPORT_INTERVAL_MS) {
    lastUartReport = now;
    reportToGateway();
  }

  delay(20);
}
