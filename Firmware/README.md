# FogGuard — Hardware Firmware Package

**SIH 2026 | Problem Statement 26007**
Safe and Efficient Operation of Mine Vehicles in Fog and Low-Visibility Conditions

---

## System Overview

```
 PER VEHICLE
 ┌──────────────────────────────┐        ┌───────────────────────┐
 │  ESP32 Node                  │  UART  │  Raspberry Pi Gateway │
 │  • GPS position              │───────▶│  • Camera + AI vision │
 │  • Visibility sensor         │  JSON  │  • Severity fusion    │
 │  • Fog Severity 0–5          │        │  • DGMS speed model   │
 │  • LoRa V2V broadcast   ◀────┼── mesh │  • Audit log          │
 │  • Relay / buzzer / LEDs     │        │  • MQTT uplink        │
 └──────────────────────────────┘        └───────────┬───────────┘
                                                     │ MQTT
                                         ┌───────────▼───────────┐
                                         │  Control Room Server  │
                                         │  • Fleet picture      │
                                         │  • Zone severity      │
                                         │  • Reroute advice     │
                                         │  • Dashboard API/WS   │
                                         └───────────────────────┘
```

---

## Files

| File | Runs on | Purpose |
|---|---|---|
| `esp32_node/fogguard_node.ino` | ESP32 | GPS, visibility sensing, LoRa V2V mesh, relay/buzzer/LED actuation |
| `rpi_gateway/fog_detector.py` | Raspberry Pi | Camera fog/dust detection, dehazing, object detection |
| `rpi_gateway/fogguard_gateway.py` | Raspberry Pi | Fusion, DGMS speed model, MQTT, audit log, in-cab API |
| `rpi_gateway/control_room_server.py` | Control room PC | Fleet aggregation, zone severity, reroute advice, dashboard API |

---

## 1. ESP32 Wiring

**Board:** ESP32 DevKit v1 (30-pin)

### LoRa SX1276 / RA-02
| LoRa pin | ESP32 pin |
|---|---|
| VCC | 3V3 (**not 5V**) |
| GND | GND |
| SCK | GPIO18 |
| MISO | GPIO19 |
| MOSI | GPIO23 |
| NSS / CS | GPIO5 |
| RST | GPIO14 |
| DIO0 | GPIO26 |

> Always attach the antenna before powering up. Transmitting without an antenna can destroy the module.

### GPS NEO-6M / NEO-M8N
| GPS pin | ESP32 pin |
|---|---|
| VCC | 3V3 |
| GND | GND |
| TX | GPIO16 (RX2) |
| RX | GPIO17 (TX2) |

### Visibility / dust sensor (GP2Y1010AU0F or IR transmissometer)
| Sensor | ESP32 |
|---|---|
| AOUT | GPIO34 (ADC1, input-only) |
| VCC | 5V (via VIN) |
| GND | GND |

### Outputs
| Device | ESP32 pin | Notes |
|---|---|---|
| Speed-limit relay IN | GPIO25 | Opto-isolated relay module, separate 5V supply |
| Buzzer + | GPIO27 | Active buzzer |
| LED green | GPIO32 | via 220 Ω |
| LED amber | GPIO33 | via 220 Ω |
| LED red | GPIO4 | via 220 Ω |

**Power:** 24 V vehicle supply → DC-DC buck to 5 V → ESP32 VIN. Add a 1000 µF capacitor across 5 V and a TVS diode on the vehicle input; haul-truck electrical systems are electrically noisy.

### Arduino IDE setup
1. Boards Manager → install **esp32 by Espressif**
2. Library Manager → install:
   - `LoRa` by Sandeep Mistry
   - `TinyGPSPlus` by Mikal Hart
   - `ArduinoJson` (v6.x) by Benoit Blanchon
3. Board: *ESP32 Dev Module*, Upload speed: 115200
4. **Change `VEHICLE_ID` per board** (`D-07`, `D-12`, `LV-01`, `PED-01`…)
5. All nodes must share the same `LORA_FREQ` and `LORA_SYNC_WORD`

---

## 2. Raspberry Pi Setup

```bash
sudo apt update
sudo apt install -y python3-pip python3-opencv mosquitto mosquitto-clients
pip3 install pyserial paho-mqtt fastapi uvicorn numpy

# optional: object detection
pip3 install ultralytics
```

Connect the ESP32 by USB (appears as `/dev/ttyUSB0`), then:

```bash
sudo usermod -a -G dialout $USER      # log out/in after this
cd rpi_gateway
python3 fog_detector.py               # test the camera alone
python3 fogguard_gateway.py           # run the full gateway
```

In-cab API: `http://<pi-ip>:8000/api/state`

### Run at boot
```bash
sudo nano /etc/systemd/system/fogguard.service
```
```ini
[Unit]
Description=FogGuard Vehicle Gateway
After=network.target

[Service]
ExecStart=/usr/bin/python3 /home/pi/fogguard/fogguard_gateway.py
WorkingDirectory=/home/pi/fogguard
Restart=always
User=pi

[Install]
WantedBy=multi-user.target
```
```bash
sudo systemctl enable --now fogguard
```

---

## 3. Control Room

```bash
sudo systemctl start mosquitto
python3 control_room_server.py
```
API: `http://<server-ip>:9000/api/fleet` · WebSocket: `ws://<server-ip>:9000/ws`

Set `MQTT_HOST` in `fogguard_gateway.py` to the control room IP.

---

## 4. Calibration — do this before any field trial

The defaults will run, but they are **not** site-accurate until calibrated.

### a) Visibility sensor (ESP32)
In `readVisibilityMetres()`, set `RAW_CLEAR` and `RAW_DENSE` by printing raw ADC values in clear air and in dense fog.

### b) Camera visibility curve (Pi)
In `fog_detector.py`, fit `VIS_CLEAR_M` and `HAZE_EXPONENT`:

1. Place markers at 25 / 50 / 100 / 200 m along a haul road
2. At several fog levels, log `estimate_haze_index()` and record the furthest visible marker
3. Fit `visibility = VIS_CLEAR_M * (1 - h) ** HAZE_EXPONENT` to those pairs

### c) Braking model (both files — keep them identical)
`T_REACTION_S` (default 1.5 s) and `DECEL_MS2` (default 2.0 m/s²) should come from the actual loaded-truck braking tests for your fleet.

### d) Zones
Survey real GPS coordinates for each haul-road zone in `control_room_server.py` → `ZONES`.

---

## 5. How the safety logic works

**Severity bands** (identical in ESP32 and Pi):

| Visibility | Severity | Response |
|---|---|---|
| ≥ 150 m | 0 | Normal |
| 100–150 m | 1 | Monitoring |
| 60–100 m | 2 | Advisory limit |
| 30–60 m | 3 | Speed capped |
| 15–30 m | 4 | Convoy mode |
| < 15 m | 5 | Hold position |

**Safe speed** inverts the DGMS rule that available visibility must be at least 3× stopping distance:

```
visibility ≥ 3 × ( v·T_reaction + v² / (2·decel) )
```
solved as a quadratic for the maximum permissible speed. Verified self-consistent:

| Visibility | Safe speed | Stopping dist | 3× |
|---|---|---|---|
| 144 m | 40.0 km/h | 47.5 m | 142.6 m |
| 120 m | 36.0 km/h | 40.0 m | 120.0 m |
| 80 m | 27.9 km/h | 26.7 m | 80.0 m |
| 45 m | 19.1 km/h | 15.0 m | 45.0 m |
| 20 m | 10.7 km/h | 6.7 m | 20.0 m |

**Sensor fusion** never lets the fused severity fall *below* the physical sensor reading — the sensor is the fail-safe floor, and the camera can only ever raise severity, weighted by its own confidence.

**Dust vs fog** is discriminated by the red:blue channel ratio, since iron-ore dust is strongly red-shifted while water fog is near-neutral grey.

**Explainability** — every intervention carries a plain-language reason (e.g. *"Visibility 45 m — speed capped at 19 km/h"*). Opaque alarms get disabled by operators, which is a documented failure mode in mining collision-avoidance deployments.

**Audit log** is SHA-256 hash-chained: each row hashes `(previous_hash + payload)`, so editing any earlier row invalidates every subsequent hash. Verify with `GET /api/health` → `audit_chain_valid`.

---

## 6. Safety scope

This is a **driver-assistance** system, not autonomous control.

- Low severity → advisory only
- High severity → speed **cap**, never an automatic brake command
- Power loss or mesh dropout → reverts to normal manual operation (fail-safe, never leaves a truck locked or braked)
- Full autonomous braking would require OEM certification and DGMS approval, which is outside this project's scope

---

## 7. Bench test without a vehicle

1. Flash two ESP32s with different `VEHICLE_ID`s
2. Open both serial monitors at 115200 — each should list the other under `peers`
3. Cover the visibility sensor to force severity up; LEDs should progress green → amber → red and the relay should click at severity 2
4. Run `control_room_server.py` and confirm both vehicles appear at `/api/fleet`
