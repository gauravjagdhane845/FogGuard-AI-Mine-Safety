#!/usr/bin/env python3
"""
==============================================================
 FogGuard — Control Room Server
 SIH 2026 | PS 26007
==============================================================

Runs on the control room machine (any PC / server / Pi).
  1. Subscribes to every vehicle's MQTT state topic
  2. Maintains a live fleet picture
  3. Computes per-zone fog severity from vehicle reports
  4. Suggests reroutes around high-severity zones
  5. Serves a REST + WebSocket API for the dashboard UI

INSTALL
  pip3 install paho-mqtt fastapi uvicorn websockets

RUN
  # start a broker first, e.g.:
  #   sudo apt install mosquitto && sudo systemctl start mosquitto
  python3 control_room_server.py
==============================================================
"""

import json
import time
import math
import asyncio
import logging
import threading
from datetime import datetime, timezone

import paho.mqtt.client as mqtt
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("controlroom")

MQTT_HOST = "localhost"
MQTT_PORT = 1883
TOPIC_STATE = "fogguard/vehicle/+/state"
TOPIC_ALERT = "fogguard/vehicle/+/alert"

API_PORT = 9000
VEHICLE_TIMEOUT_S = 15


# ==============================================================
# Haul-road zone definitions (survey these at the mine)
# ==============================================================
ZONES = [
    {"id": "A", "name": "Pit Approach", "lat": 18.6600, "lon": 81.2500, "radius_m": 400},
    {"id": "B", "name": "Junction",     "lat": 18.6650, "lon": 81.2560, "radius_m": 400},
    {"id": "C", "name": "Dump Route",   "lat": 18.6700, "lon": 81.2620, "radius_m": 400},
]

ROUTES = {
    "A": {"name": "Pit -> Crusher",    "zones": ["A", "B"]},
    "B": {"name": "Pit -> Waste Dump", "zones": ["A", "C"]},
}


def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


# ==============================================================
# Fleet state
# ==============================================================
class Fleet:
    def __init__(self):
        self._lock = threading.Lock()
        self.vehicles = {}     # id -> last state dict
        self.alerts = []       # newest first

    def update_vehicle(self, vid, state):
        with self._lock:
            state["_rx"] = time.time()
            self.vehicles[vid] = state

    def add_alert(self, alert):
        with self._lock:
            alert["_rx"] = time.time()
            self.alerts.insert(0, alert)
            del self.alerts[200:]

    def active_vehicles(self):
        now = time.time()
        with self._lock:
            return {k: v for k, v in self.vehicles.items()
                    if now - v.get("_rx", 0) < VEHICLE_TIMEOUT_S}

    def zone_severities(self):
        """
        Each zone's severity = the highest severity reported by any
        vehicle currently inside it. Highest-wins is deliberate:
        under-reporting fog is far more dangerous than over-reporting.
        """
        out = {}
        vehicles = self.active_vehicles()
        for z in ZONES:
            sev = 0
            count = 0
            for v in vehicles.values():
                if not v.get("gps_ok"):
                    continue
                d = haversine_m(v.get("lat", 0), v.get("lon", 0),
                                z["lat"], z["lon"])
                if d <= z["radius_m"]:
                    count += 1
                    sev = max(sev, int(v.get("fused_severity", 0)))
            out[z["id"]] = {
                "id": z["id"], "name": z["name"],
                "severity": sev, "vehicles_inside": count,
            }
        return out

    def reroute_advice(self):
        """
        Flag any route whose worst zone is at severity 4+, and suggest
        an alternative route whose worst zone is lower.
        """
        zs = self.zone_severities()
        scored = {}
        for rid, r in ROUTES.items():
            worst = max((zs[z]["severity"] for z in r["zones"] if z in zs),
                        default=0)
            scored[rid] = worst

        advice = []
        for rid, worst in scored.items():
            if worst >= 4:
                alts = [(a, s) for a, s in scored.items()
                        if a != rid and s < worst]
                if alts:
                    best = min(alts, key=lambda x: x[1])
                    advice.append({
                        "avoid_route": rid,
                        "avoid_name": ROUTES[rid]["name"],
                        "severity": worst,
                        "use_route": best[0],
                        "use_name": ROUTES[best[0]]["name"],
                        "use_severity": best[1],
                        "message": (f"{ROUTES[rid]['name']} at severity {worst} "
                                    f"— divert to {ROUTES[best[0]]['name']}"),
                    })
                else:
                    advice.append({
                        "avoid_route": rid,
                        "avoid_name": ROUTES[rid]["name"],
                        "severity": worst,
                        "use_route": None,
                        "message": (f"{ROUTES[rid]['name']} at severity {worst} "
                                    f"— no clear alternative, hold dispatch"),
                    })
        return advice

    def summary(self):
        vehicles = self.active_vehicles()
        zones = self.zone_severities()
        sevs = [int(v.get("fused_severity", 0)) for v in vehicles.values()]
        return {
            "ts": datetime.now(timezone.utc).isoformat(),
            "vehicle_count": len(vehicles),
            "peak_severity": max(sevs) if sevs else 0,
            "convoy_count": sum(1 for v in vehicles.values() if v.get("convoy")),
            "vehicles": list(vehicles.values()),
            "zones": list(zones.values()),
            "reroutes": self.reroute_advice(),
            "alerts": self.alerts[:20],
        }


FLEET = Fleet()


# ==============================================================
# MQTT ingest
# ==============================================================
def on_connect(client, userdata, flags, rc):
    log.info("MQTT connected rc=%s", rc)
    client.subscribe(TOPIC_STATE, qos=1)
    client.subscribe(TOPIC_ALERT, qos=1)


def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())
    except Exception:
        return
    parts = msg.topic.split("/")
    if len(parts) < 4:
        return
    vid, kind = parts[2], parts[3]

    if kind == "state":
        FLEET.update_vehicle(vid, payload)
    elif kind == "alert":
        FLEET.add_alert(payload)
        log.info("ALERT %s sev=%s %s", vid,
                 payload.get("severity"), payload.get("reason"))


def mqtt_thread():
    c = mqtt.Client(client_id="fogguard-controlroom")
    c.on_connect = on_connect
    c.on_message = on_message
    while True:
        try:
            c.connect(MQTT_HOST, MQTT_PORT, 30)
            c.loop_forever()
        except Exception as e:
            log.warning("MQTT error: %s — retry in 5s", e)
            time.sleep(5)


# ==============================================================
# API
# ==============================================================
app = FastAPI(title="FogGuard Control Room", version="1.0")
app.add_middleware(CORSMiddleware, allow_origins=["*"],
                   allow_methods=["*"], allow_headers=["*"])


@app.get("/api/fleet")
def fleet():
    return FLEET.summary()


@app.get("/api/zones")
def zones():
    return {"zones": list(FLEET.zone_severities().values())}


@app.get("/api/reroutes")
def reroutes():
    return {"reroutes": FLEET.reroute_advice()}


@app.websocket("/ws")
async def ws(websocket: WebSocket):
    """Pushes the full fleet picture to the dashboard twice a second."""
    await websocket.accept()
    try:
        while True:
            await websocket.send_json(FLEET.summary())
            await asyncio.sleep(2)
    except WebSocketDisconnect:
        pass


if __name__ == "__main__":
    threading.Thread(target=mqtt_thread, daemon=True).start()
    log.info("Control room API on :%d", API_PORT)
    uvicorn.run(app, host="0.0.0.0", port=API_PORT, log_level="warning")
