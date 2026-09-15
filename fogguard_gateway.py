#!/usr/bin/env python3
"""
==============================================================
 FogGuard — Raspberry Pi Vehicle Gateway
 SIH 2026 | PS 26007
==============================================================

ROLE OF THIS BOARD:
Mounted in the dumper cabin alongside the ESP32 node.
Responsibilities:
  1. Read telemetry from the ESP32 over serial (USB/UART)
  2. Run the camera + AI fog/obstacle detection pipeline
  3. Fuse camera severity with the ESP32's sensor severity
  4. Publish fused state to the control room over MQTT
  5. Expose a local REST API for the in-cab dashboard
  6. Keep an offline store-and-forward buffer when the network drops
  7. Write a tamper-evident (hash-chained) DGMS audit log

HARDWARE:
  - Raspberry Pi 4 / 5 (4GB+)
  - Pi Camera Module 3 or USB camera (thermal preferred)
  - ESP32 node connected via USB  -> /dev/ttyUSB0
  - Optional: 4G dongle for uplink

INSTALL:
  sudo apt update
  sudo apt install -y python3-opencv python3-pip
  pip3 install pyserial paho-mqtt fastapi uvicorn numpy

RUN:
  python3 fogguard_gateway.py
==============================================================
"""

import json
import time
import queue
import hashlib
import logging
import sqlite3
import threading
from pathlib import Path
from datetime import datetime, timezone

import serial
import paho.mqtt.client as mqtt

from fog_detector import FogDetector   # local module (see fog_detector.py)

# ==============================================================
# Configuration
# ==============================================================
VEHICLE_ID = "D-07"

SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200

MQTT_HOST = "192.168.1.100"     # control room broker IP
MQTT_PORT = 1883
MQTT_TOPIC_STATE = f"fogguard/vehicle/{VEHICLE_ID}/state"
MQTT_TOPIC_ALERT = f"fogguard/vehicle/{VEHICLE_ID}/alert"
MQTT_KEEPALIVE = 30

API_HOST = "0.0.0.0"
API_PORT = 8000

CAMERA_INDEX = 0
DETECT_INTERVAL_S = 1.0          # run vision this often
PUBLISH_INTERVAL_S = 2.0         # publish to control room this often

DB_PATH = Path("/home/pi/fogguard/audit.db")
BUFFER_MAX = 5000                # offline records to retain

# ---- DGMS-derived speed model (must match ESP32 constants) ----
# Stopping distance = reaction distance + braking distance
#   d(v) = v_ms * T_REACTION  +  v_ms^2 / (2 * DECEL)
# DGMS requires available visibility >= 3 x stopping distance.
T_REACTION_S = 1.5        # operator reaction time
DECEL_MS2 = 2.0           # loaded haul truck on dirt haul road
VISIBILITY_MARGIN = 3.0   # DGMS 3x rule
ABS_MAX_SPEED_KMH = 40.0  # mine speed-limit ceiling

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger("fogguard")


# ==============================================================
# Shared state
# ==============================================================
class VehicleState:
    """Thread-safe snapshot of everything we know right now."""

    def __init__(self):
        self._lock = threading.Lock()
        self.data = {
            "vehicle_id": VEHICLE_ID,
            "ts": None,
            "lat": 0.0,
            "lon": 0.0,
            "gps_ok": False,
            "speed": 0.0,
            # severity sources
            "sensor_severity": 0,
            "vision_severity": 0,
            "fused_severity": 0,
            "visibility_m": 200,
            "vision_confidence": 0.0,
            # derived control
            "safe_speed": ABS_MAX_SPEED_KMH,
            "convoy": False,
            "reason": "System nominal",
            # proximity
            "near_id": "-",
            "near_m": -1,
            "peers": [],
            # detections from camera
            "detections": [],
            "link_ok": False,
        }

    def update(self, **kwargs):
        with self._lock:
            self.data.update(kwargs)
            self.data["ts"] = datetime.now(timezone.utc).isoformat()

    def snapshot(self):
        with self._lock:
            return dict(self.data)


STATE = VehicleState()


# ==============================================================
# Fog severity fusion + DGMS speed model
# ==============================================================
def compute_safe_speed(visibility_m: float) -> float:
    """
    Invert the DGMS visibility rule to get a maximum safe speed.

    Requirement:  visibility >= MARGIN * stopping_distance(v)
    Stopping distance, with v in m/s:
        d = v*T_REACTION + v^2 / (2*DECEL)

    Substituting v_ms = v_kmh / 3.6 and setting MARGIN*d = visibility
    gives a quadratic in v_kmh:
        A*v^2 + B*v - visibility = 0
    where
        A = MARGIN / (3.6^2 * 2 * DECEL)
        B = MARGIN * T_REACTION / 3.6

    Solved with the positive root.
    """
    if visibility_m <= 0:
        return 0.0

    A = VISIBILITY_MARGIN / ((3.6 ** 2) * 2.0 * DECEL_MS2)
    B = VISIBILITY_MARGIN * T_REACTION_S / 3.6

    disc = B * B + 4.0 * A * visibility_m
    v = (-B + disc ** 0.5) / (2.0 * A)
    return max(0.0, min(v, ABS_MAX_SPEED_KMH))


def fuse_severity(sensor_sev: int, vision_sev: int, vision_conf: float) -> int:
    """
    Fuse the ESP32's physical sensor reading with the camera's
    vision estimate.

    Rationale: the dust sensor is reliable but local and slow;
    the camera sees further ahead but can be fooled by ore dust.
    We weight the camera only as much as its own confidence allows,
    and we never let fusion drop BELOW the physical sensor reading
    (fail-safe: the sensor is the floor).
    """
    w = max(0.0, min(vision_conf, 1.0))
    fused = (1.0 - w) * sensor_sev + w * vision_sev
    fused_i = int(round(fused))
    return max(sensor_sev, min(5, fused_i))


def explain(severity: int, safe_speed: float, near_id: str, near_m: int,
            visibility_m: int) -> str:
    """
    Build the human-readable reason shown to the driver.
    Explainability is deliberate: opaque alarms get disabled by
    operators, which is a documented failure mode in mining CAS.
    """
    if near_m is not None and 0 <= near_m < 40:
        return f"{near_id} at {near_m} m ahead — hold distance"
    if severity == 0:
        return "Clear conditions — normal operation"
    if severity == 1:
        return f"Light haze, visibility {visibility_m} m — monitoring"
    if severity == 2:
        return f"Visibility {visibility_m} m — advisory limit {safe_speed:.0f} km/h"
    if severity == 3:
        return f"Visibility {visibility_m} m — speed capped at {safe_speed:.0f} km/h"
    if severity == 4:
        return f"Visibility {visibility_m} m — convoy mode, follow lead vehicle"
    return f"Visibility {visibility_m} m — hold position, await control room"


# ==============================================================
# Tamper-evident audit log (DGMS-ready)
# ==============================================================
class AuditLog:
    """
    Hash-chained log. Each row stores the SHA-256 of
    (previous_hash + payload), so any retro-edit of an earlier row
    breaks every subsequent hash — making tampering detectable.
    """

    def __init__(self, path: Path):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.conn = sqlite3.connect(str(path), check_same_thread=False)
        self.lock = threading.Lock()
        self._init_schema()

    def _init_schema(self):
        with self.conn:
            self.conn.execute("""
                CREATE TABLE IF NOT EXISTS audit (
                    id        INTEGER PRIMARY KEY AUTOINCREMENT,
                    ts        TEXT NOT NULL,
                    vehicle   TEXT NOT NULL,
                    severity  INTEGER,
                    speed     REAL,
                    safe_speed REAL,
                    visibility INTEGER,
                    reason    TEXT,
                    lat       REAL,
                    lon       REAL,
                    prev_hash TEXT,
                    row_hash  TEXT
                )
            """)
            self.conn.execute("""
                CREATE TABLE IF NOT EXISTS outbox (
                    id      INTEGER PRIMARY KEY AUTOINCREMENT,
                    topic   TEXT NOT NULL,
                    payload TEXT NOT NULL
                )
            """)

    def _last_hash(self) -> str:
        cur = self.conn.execute(
            "SELECT row_hash FROM audit ORDER BY id DESC LIMIT 1")
        row = cur.fetchone()
        return row[0] if row else "GENESIS"

    def record(self, s: dict):
        with self.lock:
            prev = self._last_hash()
            payload = json.dumps({
                "ts": s["ts"], "vehicle": s["vehicle_id"],
                "severity": s["fused_severity"], "speed": s["speed"],
                "safe_speed": s["safe_speed"], "visibility": s["visibility_m"],
                "reason": s["reason"], "lat": s["lat"], "lon": s["lon"],
            }, sort_keys=True)
            row_hash = hashlib.sha256((prev + payload).encode()).hexdigest()
            with self.conn:
                self.conn.execute("""
                    INSERT INTO audit (ts, vehicle, severity, speed, safe_speed,
                                       visibility, reason, lat, lon,
                                       prev_hash, row_hash)
                    VALUES (?,?,?,?,?,?,?,?,?,?,?)
                """, (s["ts"], s["vehicle_id"], s["fused_severity"], s["speed"],
                      s["safe_speed"], s["visibility_m"], s["reason"],
                      s["lat"], s["lon"], prev, row_hash))

    def verify(self) -> bool:
        """Re-walk the chain and confirm no row was altered."""
        with self.lock:
            cur = self.conn.execute("""
                SELECT ts, vehicle, severity, speed, safe_speed, visibility,
                       reason, lat, lon, prev_hash, row_hash
                FROM audit ORDER BY id ASC
            """)
            prev = "GENESIS"
            for r in cur.fetchall():
                payload = json.dumps({
                    "ts": r[0], "vehicle": r[1], "severity": r[2],
                    "speed": r[3], "safe_speed": r[4], "visibility": r[5],
                    "reason": r[6], "lat": r[7], "lon": r[8],
                }, sort_keys=True)
                expect = hashlib.sha256((prev + payload).encode()).hexdigest()
                if expect != r[10] or r[9] != prev:
                    return False
                prev = r[10]
            return True

    # ---- offline store-and-forward ----
    def buffer_push(self, topic: str, payload: str):
        with self.lock, self.conn:
            self.conn.execute(
                "INSERT INTO outbox (topic, payload) VALUES (?,?)",
                (topic, payload))
            self.conn.execute("""
                DELETE FROM outbox WHERE id NOT IN (
                    SELECT id FROM outbox ORDER BY id DESC LIMIT ?
                )
            """, (BUFFER_MAX,))

    def buffer_drain(self, limit=200):
        with self.lock:
            cur = self.conn.execute(
                "SELECT id, topic, payload FROM outbox ORDER BY id ASC LIMIT ?",
                (limit,))
            return cur.fetchall()

    def buffer_delete(self, ids):
        if not ids:
            return
        with self.lock, self.conn:
            self.conn.executemany(
                "DELETE FROM outbox WHERE id = ?", [(i,) for i in ids])


AUDIT = AuditLog(DB_PATH)


# ==============================================================
# Thread 1 — ESP32 serial reader
# ==============================================================
def serial_reader_thread():
    """Continuously read newline-delimited JSON from the ESP32."""
    while True:
        try:
            with serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=2) as ser:
                log.info("Serial link open on %s", SERIAL_PORT)
                STATE.update(link_ok=True)
                buf = b""
                while True:
                    chunk = ser.readline()
                    if not chunk:
                        continue
                    line = chunk.decode("utf-8", errors="ignore").strip()
                    if not line or line.startswith("#"):
                        continue          # comment/debug line from ESP32
                    try:
                        msg = json.loads(line)
                    except json.JSONDecodeError:
                        continue

                    STATE.update(
                        lat=msg.get("lat", 0.0),
                        lon=msg.get("lon", 0.0),
                        gps_ok=msg.get("gps_ok", False),
                        speed=msg.get("speed", 0.0),
                        sensor_severity=msg.get("severity", 0),
                        visibility_m=msg.get("visibility", 200),
                        near_id=msg.get("near_id", "-"),
                        near_m=msg.get("near_m", -1),
                        peers=msg.get("peers", []),
                        link_ok=True,
                    )
        except serial.SerialException as e:
            log.warning("Serial error: %s — retrying in 3s", e)
            STATE.update(link_ok=False)
            time.sleep(3)


# ==============================================================
# Thread 2 — Camera / AI fog detection
# ==============================================================
def vision_thread():
    detector = FogDetector(camera_index=CAMERA_INDEX)
    if not detector.open():
        log.error("Camera unavailable — running in sensor-only mode")
        return

    log.info("Vision pipeline started")
    while True:
        t0 = time.time()
        try:
            result = detector.process_frame()
            if result is not None:
                STATE.update(
                    vision_severity=result["severity"],
                    vision_confidence=result["confidence"],
                    detections=result["detections"],
                )
        except Exception as e:
            log.exception("Vision error: %s", e)

        dt = time.time() - t0
        time.sleep(max(0.0, DETECT_INTERVAL_S - dt))


# ==============================================================
# Thread 3 — Fusion + MQTT publisher
# ==============================================================
class Publisher:
    def __init__(self):
        self.client = mqtt.Client(client_id=f"fogguard-{VEHICLE_ID}")
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.connected = False

    def _on_connect(self, client, userdata, flags, rc):
        self.connected = (rc == 0)
        log.info("MQTT connected (rc=%s)", rc)

    def _on_disconnect(self, client, userdata, rc):
        self.connected = False
        log.warning("MQTT disconnected (rc=%s)", rc)

    def start(self):
        try:
            self.client.connect_async(MQTT_HOST, MQTT_PORT, MQTT_KEEPALIVE)
            self.client.loop_start()
        except Exception as e:
            log.warning("MQTT start failed: %s", e)

    def send(self, topic: str, payload: str):
        """Publish if online; otherwise buffer to disk for later."""
        if self.connected:
            try:
                self.client.publish(topic, payload, qos=1)
                return True
            except Exception as e:
                log.warning("Publish failed: %s", e)
        AUDIT.buffer_push(topic, payload)
        return False

    def flush_buffer(self):
        """Drain the offline outbox once the link returns."""
        if not self.connected:
            return
        rows = AUDIT.buffer_drain()
        sent = []
        for rid, topic, payload in rows:
            try:
                self.client.publish(topic, payload, qos=1)
                sent.append(rid)
            except Exception:
                break
        if sent:
            AUDIT.buffer_delete(sent)
            log.info("Flushed %d buffered records", len(sent))


PUB = Publisher()


def fusion_publish_thread():
    last_severity = -1
    while True:
        s = STATE.snapshot()

        fused = fuse_severity(
            s["sensor_severity"], s["vision_severity"], s["vision_confidence"])
        safe_speed = compute_safe_speed(s["visibility_m"])
        reason = explain(fused, safe_speed, s["near_id"],
                         s["near_m"], s["visibility_m"])

        STATE.update(
            fused_severity=fused,
            safe_speed=round(safe_speed, 1),
            convoy=(fused >= 4),
            reason=reason,
        )

        s = STATE.snapshot()
        PUB.send(MQTT_TOPIC_STATE, json.dumps(s))
        PUB.flush_buffer()

        # Log every severity transition to the tamper-evident audit trail
        if fused != last_severity:
            AUDIT.record(s)
            PUB.send(MQTT_TOPIC_ALERT, json.dumps({
                "vehicle_id": VEHICLE_ID,
                "ts": s["ts"],
                "severity": fused,
                "reason": reason,
                "safe_speed": s["safe_speed"],
            }))
            log.info("Severity %s -> %s | %s", last_severity, fused, reason)
            last_severity = fused

        time.sleep(PUBLISH_INTERVAL_S)


# ==============================================================
# Thread 4 — Local REST API for the in-cab dashboard
# ==============================================================
def api_thread():
    from fastapi import FastAPI
    from fastapi.middleware.cors import CORSMiddleware
    import uvicorn

    app = FastAPI(title="FogGuard Vehicle API", version="1.0")
    app.add_middleware(
        CORSMiddleware, allow_origins=["*"],
        allow_methods=["*"], allow_headers=["*"],
    )

    @app.get("/api/state")
    def get_state():
        """Live state consumed by the in-cab dashboard UI."""
        return STATE.snapshot()

    @app.get("/api/health")
    def health():
        s = STATE.snapshot()
        return {
            "vehicle_id": VEHICLE_ID,
            "serial_link": s["link_ok"],
            "gps_fix": s["gps_ok"],
            "mqtt": PUB.connected,
            "audit_chain_valid": AUDIT.verify(),
        }

    @app.get("/api/peers")
    def peers():
        return {"peers": STATE.snapshot()["peers"]}

    uvicorn.run(app, host=API_HOST, port=API_PORT, log_level="warning")


# ==============================================================
# Entrypoint
# ==============================================================
def main():
    log.info("FogGuard gateway starting — vehicle %s", VEHICLE_ID)

    PUB.start()

    threads = [
        threading.Thread(target=serial_reader_thread, daemon=True),
        threading.Thread(target=vision_thread, daemon=True),
        threading.Thread(target=fusion_publish_thread, daemon=True),
        threading.Thread(target=api_thread, daemon=True),
    ]
    for t in threads:
        t.start()

    log.info("All subsystems running. API on :%d", API_PORT)
    try:
        while True:
            time.sleep(10)
    except KeyboardInterrupt:
        log.info("Shutting down.")


if __name__ == "__main__":
    main()
