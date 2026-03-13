"""
MQTT Traffic Light Service  –  Port 8009
=========================================
Connects to service.ifak.eu:1883, subscribes to iot_board/lsa/leitstelle,
and exposes the received LSA (Lichtsignalanlage / traffic-light) data via
a REST API consumed by the Streamlit dashboard.

All raw MQTT messages are stored as-is.  A best-effort parser tries to
extract well-known traffic-light fields (id, state, lat, lon, …) so the
existing dashboard endpoints keep working even when the payload schema
differs from the built-in simulator format.

This service is subscribe-only: it never publishes to the broker.
"""

import json
import random
import threading
import time
from datetime import datetime
from typing import Optional

import paho.mqtt.client as mqtt
from fastapi import FastAPI
from pydantic import BaseModel

# ─── Configuration ─────────────────────────────────────────────────────────────
MQTT_BROKER   = "service.ifak.eu"
MQTT_PORT     = 1883
MQTT_TOPIC    = "iot_board/lsa/leitstelle"
SERVICE_PORT  = 8009

app = FastAPI(title="MQTT Traffic Light Service", version="1.0.0")

# ─── In-memory store ────────────────────────────────────────────────────────────
_lights: dict[str, dict] = {}       # id → latest state dict
_events: list[dict]       = []      # rolling history (last 200)
_MAX_EVENTS = 200

_mqtt_status = {
    "broker":    MQTT_BROKER,
    "port":      MQTT_PORT,
    "topic":     MQTT_TOPIC,
    "connected": False,
    "messages_received": 0,
}

# ─── "Red triggered" sequence state ────────────────────────────────────────────
_TRIGGER_PHRASE   = "red triggered"
_TRIGGER_LIGHT_ID = "TRIGGERED_LIGHT"

_trigger_stats = {
    "total_triggers":   0,
    "last_triggered":   None,
    "cycles_completed": 0,
}

# Shared mutable state for the triggered light (also written into _lights).
_triggered_state: dict = {
    "id":              _TRIGGER_LIGHT_ID,
    "intersection":    "Triggered Intersection",
    "lat":             52.1205,
    "lon":             11.6276,
    "state":           "GREEN",
    "phase":           "normal",
    "time_in_state_s": 0,
    "cycle_s":         24,          # 2+20+2 = 24 s fixed cycle
    "topic":           MQTT_TOPIC,
    "raw":             {},
    "timestamp":       datetime.utcnow().isoformat() + "Z",
    "sequence_active": False,
}
_lights[_TRIGGER_LIGHT_ID] = _triggered_state   # always present in /traffic/lights

_trigger_lock   = threading.Lock()
_cancel_event   = threading.Event()
_sequence_thread: Optional[threading.Thread] = None


def _set_triggered_state(state: str, duration_s: float) -> bool:
    """
    Transition the triggered light to *state*, increment time_in_state_s every
    0.1 s, and return True when *duration_s* has elapsed.
    Returns False early if _cancel_event is set (new trigger arrived).
    """
    now = datetime.utcnow().isoformat() + "Z"
    with _trigger_lock:
        _triggered_state["state"]           = state
        _triggered_state["time_in_state_s"] = 0
        _triggered_state["sequence_active"] = True
        _triggered_state["timestamp"]       = now
        _lights[_TRIGGER_LIGHT_ID]          = dict(_triggered_state)

    start = time.monotonic()
    while True:
        elapsed = time.monotonic() - start
        if elapsed >= duration_s:
            with _trigger_lock:
                _triggered_state["time_in_state_s"] = int(duration_s)
                _lights[_TRIGGER_LIGHT_ID] = dict(_triggered_state)
            return True
        if _cancel_event.is_set():
            return False
        with _trigger_lock:
            _triggered_state["time_in_state_s"] = int(elapsed)
            _lights[_TRIGGER_LIGHT_ID] = dict(_triggered_state)
        time.sleep(0.1)


def _run_sequence():
    """
    Sequence on "red triggered":
      YELLOW  2 s  → RED  20 s  → YELLOW  2 s  → GREEN  (until next trigger)
    """
    # Phase 1 – yellow pre-red
    if not _set_triggered_state("YELLOW", 2):
        return
    # Phase 2 – red
    if not _set_triggered_state("RED", 20):
        return
    # Phase 3 – yellow post-red
    if not _set_triggered_state("YELLOW", 2):
        return

    # Phase 4 – green (open-ended; keep counting time_in_state_s)
    now = datetime.utcnow().isoformat() + "Z"
    with _trigger_lock:
        _triggered_state["state"]           = "GREEN"
        _triggered_state["time_in_state_s"] = 0
        _triggered_state["sequence_active"] = False
        _triggered_state["timestamp"]       = now
        _lights[_TRIGGER_LIGHT_ID]          = dict(_triggered_state)
        _trigger_stats["cycles_completed"] += 1

    # Keep counting while waiting for the next trigger
    start = time.monotonic()
    while not _cancel_event.is_set():
        with _trigger_lock:
            _triggered_state["time_in_state_s"] = int(time.monotonic() - start)
            _lights[_TRIGGER_LIGHT_ID] = dict(_triggered_state)
        time.sleep(0.5)


def _handle_red_triggered():
    """Cancel any in-progress sequence and start a fresh one."""
    global _sequence_thread
    _cancel_event.set()
    if _sequence_thread and _sequence_thread.is_alive():
        _sequence_thread.join(timeout=1.0)
    _cancel_event.clear()

    now = datetime.utcnow().isoformat() + "Z"
    with _trigger_lock:
        _trigger_stats["total_triggers"] += 1
        _trigger_stats["last_triggered"]  = now

    _sequence_thread = threading.Thread(target=_run_sequence, daemon=True)
    _sequence_thread.start()

# ─── MQTT client setup ──────────────────────────────────────────────────────────
_mqttc = mqtt.Client(client_id=f"traffic_service_{random.randint(1000,9999)}")


def _on_connect(client, userdata, flags, rc):
    if rc == 0:
        _mqtt_status["connected"] = True
        client.subscribe(MQTT_TOPIC)
    else:
        _mqtt_status["connected"] = False


def _on_disconnect(client, userdata, rc):
    _mqtt_status["connected"] = False


def _on_message(client, userdata, msg):
    raw_text = msg.payload.decode(errors="replace")
    _mqtt_status["messages_received"] += 1

    # ── "red triggered" command ──────────────────────────────────────────────
    if _TRIGGER_PHRASE in raw_text.lower():
        _handle_red_triggered()
        # Record an event for history (include all columns the dashboard expects)
        now = datetime.utcnow().isoformat() + "Z"
        event = {
            "id":              _TRIGGER_LIGHT_ID,
            "intersection":    "Triggered Intersection",
            "state":           "TRIGGERED",
            "phase":           "normal",
            "time_in_state_s": 0,
            "cycle_s":         24,
            "lat":             52.1205,
            "lon":             11.6276,
            "raw":             {"raw": raw_text},
            "topic":           msg.topic,
            "timestamp":       now,
            "event_time":      now,
        }
        _events.append(event)
        if len(_events) > _MAX_EVENTS:
            _events.pop(0)
        return
    # ── end trigger handling ─────────────────────────────────────────────────

    # Try to parse as JSON; fall back to storing the raw string.
    try:
        payload = json.loads(raw_text)
        if not isinstance(payload, dict):
            payload = {"raw": payload}
    except Exception:
        payload = {"raw": raw_text}

    # Best-effort extraction of common traffic-light fields.
    # The real iot_board/lsa/leitstelle feed may use different key names,
    # so we check several aliases and fall back to sensible defaults.
    light_id = (
        payload.get("id")
        or payload.get("lsa_id")
        or payload.get("signal_id")
        or payload.get("name")
        or "LSA_UNKNOWN"
    )
    state_raw = (
        payload.get("state")
        or payload.get("zustand")        # German field name
        or payload.get("signal_state")
        or "UNKNOWN"
    )
    # Normalise common German → English state names
    _STATE_MAP = {
        "ROT": "RED", "GELB": "YELLOW", "GRUEN": "GREEN", "GRÜN": "GREEN",
        "RED": "RED", "YELLOW": "YELLOW", "GREEN": "GREEN",
    }
    state = _STATE_MAP.get(str(state_raw).upper(), str(state_raw).upper())

    record = {
        "id":              light_id,
        "intersection":    payload.get("intersection") or payload.get("kreuzung") or payload.get("location") or "Unknown",
        "lat":             payload.get("lat") or payload.get("latitude")  or 52.1205,
        "lon":             payload.get("lon") or payload.get("longitude") or 11.6276,
        "state":           state,
        "phase":           payload.get("phase") or payload.get("betriebsart") or "normal",
        "time_in_state_s": payload.get("time_in_state_s") or payload.get("verweildauer") or 0,
        "cycle_s":         payload.get("cycle_s") or payload.get("umlaufzeit") or 60,
        "topic":           msg.topic,
        "raw":             payload,          # full original payload always stored
        "timestamp":       datetime.utcnow().isoformat() + "Z",
    }
    _lights[light_id] = record

    event = {**record, "event_time": record["timestamp"]}
    _events.append(event)
    if len(_events) > _MAX_EVENTS:
        _events.pop(0)


_mqttc.on_connect    = _on_connect
_mqttc.on_disconnect = _on_disconnect
_mqttc.on_message    = _on_message


def _start_mqtt():
    """Connect and run the MQTT loop in a background thread (subscribe-only)."""
    try:
        _mqttc.connect(_mqtt_status["broker"], _mqtt_status["port"], keepalive=60)
        _mqttc.loop_forever()
    except Exception:
        _mqtt_status["connected"] = False


_mqtt_thread = threading.Thread(target=_start_mqtt, daemon=True)
_mqtt_thread.start()


# ─── REST endpoints ─────────────────────────────────────────────────────────────

@app.get("/")
def health():
    return {
        "service": "MQTT Traffic Light Service",
        "status":  "ok",
        "mqtt":    _mqtt_status,
        "lights_tracked": len(_lights),
    }


@app.get("/traffic/lights")
def get_all_lights():
    return {"lights": list(_lights.values()), "total": len(_lights)}


@app.get("/traffic/lights/{light_id}")
def get_light(light_id: str):
    if light_id not in _lights:
        from fastapi import HTTPException
        raise HTTPException(status_code=404, detail=f"Light {light_id!r} not found")
    return _lights[light_id]


@app.get("/traffic/summary")
def get_summary():
    lights = list(_lights.values())
    total   = len(lights)
    by_state = {"RED": 0, "YELLOW": 0, "GREEN": 0}
    faults  = 0
    for lt in lights:
        s = lt.get("state", "RED")
        by_state[s] = by_state.get(s, 0) + 1
        if lt.get("phase") == "fault":
            faults += 1
    return {
        "total":    total,
        "by_state": by_state,
        "faults":   faults,
        "healthy":  total - faults,
    }


@app.get("/traffic/history")
def get_history(limit: int = 100):
    return {"events": _events[-limit:], "total": len(_events)}


@app.get("/traffic/triggered")
def get_triggered():
    """Current state of the 'red triggered' light plus trigger statistics."""
    with _trigger_lock:
        light = dict(_lights.get(_TRIGGER_LIGHT_ID, _triggered_state))
    return {
        "light":  light,
        "stats":  dict(_trigger_stats),
        "sequence": {
            "yellow_pre_s":  2,
            "red_s":         20,
            "yellow_post_s": 2,
            "green_s":       "until next trigger",
        },
    }


@app.get("/traffic/raw")
def get_raw(limit: int = 20):
    """Return the raw payloads of the last N messages as received from the broker."""
    return {
        "broker": _mqtt_status["broker"],
        "topic":  _mqtt_status["topic"],
        "messages": [e.get("raw") for e in _events[-limit:]],
        "total": len(_events),
    }


class BrokerConfig(BaseModel):
    broker: str
    port:   int   = 1883
    topic:  str   = "iot_board/lsa/leitstelle"


@app.post("/config")
def update_config(cfg: BrokerConfig):
    """Reconnect to a different MQTT broker/topic (subscribe-only)."""
    _mqttc.disconnect()
    _mqtt_status.update(broker=cfg.broker, port=cfg.port, topic=cfg.topic,
                        connected=False, messages_received=0)
    _lights.clear()
    _events.clear()

    def _reconnect():
        try:
            _mqttc.connect(cfg.broker, cfg.port, keepalive=60)
            _mqttc.subscribe(cfg.topic)
        except Exception:
            pass

    threading.Thread(target=_reconnect, daemon=True).start()
    return {"status": "reconnecting", "config": cfg.model_dump()}


# ─── Entry point ────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import uvicorn
    uvicorn.run("mqtt_traffic_service:app", host="0.0.0.0", port=SERVICE_PORT, reload=False)
