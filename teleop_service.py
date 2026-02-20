"""
Teleoperation Service — simulates remote control of a differential-drive robot.

Accepts velocity commands (linear m/s + angular °/s), updates the robot pose
using dead-reckoning kinematics, tracks a path trail, and exposes a REST API
consumed by the Streamlit dashboard.

Robot coordinate convention
    • X axis = East  (heading 0°)
    • Y axis = North (heading 90°)
    • Positive angular = counter-clockwise (left turn)

Port: 8007
"""

import math
from datetime import datetime

import uvicorn
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

# ── Robot state ────────────────────────────────────────────────────────────────

_robot: dict = {
    "x":              0.0,    # metres (local frame, origin = connect point)
    "y":              0.0,
    "heading":        0.0,    # degrees (0 = East, 90 = North)
    "speed":          0.0,    # m/s
    "angular":        0.0,    # °/s
    "battery":        100.0,  # %
    "status":         "Idle",
    "last_cmd_time":  None,
    "total_distance": 0.0,
}

# Connected fleet vehicle
_connection: dict = {
    "vehicle_id":   None,    # e.g. "AV-001"
    "vehicle_name": None,
    "home_lat":     None,    # GPS origin when connect() was called
    "home_lon":     None,
}

_cmd_history:  list[dict] = []
_path_history: list[dict] = []

_DRAIN_PER_METRE  = 0.05
_DRAIN_PER_DEGREE = 0.005

# Magdeburg lat/lon constants for position sync
_LAT_PER_METRE = 1.0 / 111_320          # degrees per metre northward
_LON_PER_METRE = 1.0 / (111_320 * 0.616)  # degrees per metre eastward @ 52°N

# ── Input models ───────────────────────────────────────────────────────────────

class CmdVel(BaseModel):
    linear:   float = 0.0
    angular:  float = 0.0
    duration: float = 0.5


class ConnectRequest(BaseModel):
    vehicle_id:   str
    vehicle_name: str  = ""
    battery:      float = 100.0
    home_lat:     float = 52.1205
    home_lon:     float = 11.6276


# ── Internal helpers ───────────────────────────────────────────────────────────

def _now() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def _snapshot_path() -> None:
    lat = _connection["home_lat"]
    lon = _connection["home_lon"]
    entry = {
        "x":         round(_robot["x"],       3),
        "y":         round(_robot["y"],       3),
        "heading":   round(_robot["heading"], 1),
        "timestamp": datetime.now().strftime("%H:%M:%S.%f")[:-3],
    }
    if lat is not None:
        entry["latitude"]  = round(lat + _robot["y"] * _LAT_PER_METRE, 6)
        entry["longitude"] = round(lon + _robot["x"] * _LON_PER_METRE, 6)
    _path_history.append(entry)
    if len(_path_history) > 200:
        _path_history.pop(0)


def _log_cmd(label: str, linear: float, angular: float) -> None:
    _cmd_history.append({
        "timestamp":    _now(),
        "command":      label,
        "linear_ms":    round(linear,           2),
        "angular_degs": round(angular,          1),
        "x":            round(_robot["x"],      3),
        "y":            round(_robot["y"],      3),
        "heading":      round(_robot["heading"],1),
        "battery":      round(_robot["battery"],1),
    })
    if len(_cmd_history) > 50:
        _cmd_history.pop(0)


def _cmd_label(linear: float, angular: float) -> str:
    moving  = abs(linear)  > 0.01
    turning = abs(angular) > 1.0
    if moving  and not turning: return "Forward"      if linear  > 0 else "Backward"
    if turning and not moving:  return "Rotate Left"  if angular > 0 else "Rotate Right"
    if moving  and turning:
        d = "Left" if angular > 0 else "Right"
        return f"Forward-{d}" if linear > 0 else f"Backward-{d}"
    return "Custom"


def _apply_cmd(linear: float, angular: float, duration: float) -> None:
    dt      = max(0.1, min(duration, 5.0))
    linear  = max(-2.0, min(2.0,  linear))
    angular = max(-90.0, min(90.0, angular))

    hr = math.radians(_robot["heading"])
    _robot["x"]       += linear * math.cos(hr) * dt
    _robot["y"]       += linear * math.sin(hr) * dt
    _robot["heading"]  = (_robot["heading"] + angular * dt) % 360
    _robot["speed"]    = linear
    _robot["angular"]  = angular

    dist = abs(linear)  * dt
    rot  = abs(angular) * dt
    _robot["total_distance"] += dist
    _robot["battery"] = max(0.0, _robot["battery"]
                            - dist * _DRAIN_PER_METRE
                            - rot  * _DRAIN_PER_DEGREE)

    _robot["status"] = (
        "Moving"   if abs(linear)  > 0.01 else
        "Rotating" if abs(angular) > 1.0  else
        "Idle"
    )
    _robot["last_cmd_time"] = _now()


def _current_gps() -> dict:
    lat = _connection["home_lat"]
    lon = _connection["home_lon"]
    if lat is None:
        return {}
    return {
        "latitude":  round(lat + _robot["y"] * _LAT_PER_METRE, 6),
        "longitude": round(lon + _robot["x"] * _LON_PER_METRE, 6),
    }


# ── FastAPI app ────────────────────────────────────────────────────────────────

app = FastAPI()


@app.get("/")
def root():
    return {
        "service":        "Teleoperation Service",
        "status":         "running",
        "robot_status":   _robot["status"],
        "battery":        round(_robot["battery"], 1),
        "connected_to":   _connection["vehicle_id"],
    }


@app.get("/robot/state")
def robot_state():
    state = {
        "x":              round(_robot["x"],              3),
        "y":              round(_robot["y"],              3),
        "heading":        round(_robot["heading"],        1),
        "speed":          round(_robot["speed"],          2),
        "angular":        round(_robot["angular"],        1),
        "battery":        round(_robot["battery"],        1),
        "status":         _robot["status"],
        "total_distance": round(_robot["total_distance"], 3),
        "last_cmd_time":  _robot["last_cmd_time"],
        "connected_to":   _connection["vehicle_id"],
        "vehicle_name":   _connection["vehicle_name"],
    }
    state.update(_current_gps())
    return state


@app.post("/robot/connect")
def connect(req: ConnectRequest):
    """Connect the teleop robot to a fleet vehicle, loading its position and battery."""
    _connection["vehicle_id"]   = req.vehicle_id
    _connection["vehicle_name"] = req.vehicle_name
    _connection["home_lat"]     = req.home_lat
    _connection["home_lon"]     = req.home_lon
    # Reset local pose to origin (vehicle's GPS position is the new home)
    _robot.update({
        "x": 0.0, "y": 0.0, "heading": 0.0,
        "speed": 0.0, "angular": 0.0,
        "battery": req.battery,
        "status": "Idle",
        "last_cmd_time": _now(),
        "total_distance": 0.0,
    })
    _path_history.clear()
    _cmd_history.clear()
    _snapshot_path()
    return {
        "status":     "connected",
        "vehicle_id": req.vehicle_id,
        "home_lat":   req.home_lat,
        "home_lon":   req.home_lon,
    }


@app.post("/robot/disconnect")
def disconnect():
    """Disconnect from the current fleet vehicle."""
    vid = _connection["vehicle_id"]
    _connection.update({"vehicle_id": None, "vehicle_name": None,
                         "home_lat": None, "home_lon": None})
    return {"status": "disconnected", "released_vehicle": vid}


@app.get("/robot/connection")
def get_connection():
    """Return the currently connected fleet vehicle info."""
    return {
        "vehicle_id":   _connection["vehicle_id"],
        "vehicle_name": _connection["vehicle_name"],
        "connected":    _connection["vehicle_id"] is not None,
        "home_lat":     _connection["home_lat"],
        "home_lon":     _connection["home_lon"],
    }


@app.post("/robot/cmd_vel")
def cmd_vel(cmd: CmdVel):
    if _robot["status"] == "E-Stop":
        return {"status": "rejected", "reason": "Emergency stop active — call /robot/reset"}
    if _robot["battery"] <= 0:
        return {"status": "rejected", "reason": "Battery depleted"}

    label = _cmd_label(cmd.linear, cmd.angular)
    _apply_cmd(cmd.linear, cmd.angular, cmd.duration)
    _snapshot_path()
    _log_cmd(label, cmd.linear, cmd.angular)

    result = {
        "status":  "ok",
        "command": label,
        "x":       round(_robot["x"],       3),
        "y":       round(_robot["y"],       3),
        "heading": round(_robot["heading"], 1),
        "battery": round(_robot["battery"], 1),
    }
    result.update(_current_gps())
    return result


@app.post("/robot/stop")
def stop():
    _robot.update({"speed": 0.0, "angular": 0.0, "status": "Idle",
                   "last_cmd_time": _now()})
    _snapshot_path()
    _log_cmd("Stop", 0.0, 0.0)
    return {"status": "stopped"}


@app.post("/robot/estop")
def estop():
    _robot.update({"speed": 0.0, "angular": 0.0, "status": "E-Stop",
                   "last_cmd_time": _now()})
    _log_cmd("E-STOP", 0.0, 0.0)
    return {"status": "emergency_stopped"}


@app.post("/robot/reset")
def reset():
    _robot.update({
        "x": 0.0, "y": 0.0, "heading": 0.0,
        "speed": 0.0, "angular": 0.0,
        "battery": 100.0, "status": "Idle",
        "last_cmd_time": None, "total_distance": 0.0,
    })
    _path_history.clear()
    _cmd_history.clear()
    return {"status": "reset"}


@app.get("/robot/history")
def robot_history():
    return {"history": _cmd_history, "count": len(_cmd_history)}


@app.get("/robot/path")
def robot_path():
    return {"path": _path_history, "count": len(_path_history)}


if __name__ == "__main__":
    print("Starting Teleoperation Service on port 8007...")
    print("  Robot starts at (0, 0)  heading 0°  battery 100%")
    uvicorn.run(app, host="0.0.0.0", port=8007)
