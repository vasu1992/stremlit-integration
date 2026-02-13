"""
LiDAR Service — bridges rosbridge_websocket (ROS) to a REST API.

The service maintains a persistent async WebSocket connection to a rosbridge
server running on the robot, subscribes to a sensor_msgs/LaserScan topic,
converts polar scan data to Cartesian coordinates, and exposes the result
through FastAPI endpoints consumed by the Streamlit dashboard.

Default rosbridge address: ws://192.168.1.100:9090
Default scan topic:        /scan
"""

import asyncio
import json
import math
import time
from contextlib import asynccontextmanager
from datetime import datetime
from typing import Optional

import uvicorn
import websockets
from fastapi import FastAPI
from pydantic import BaseModel

# ── Shared state ──────────────────────────────────────────────────────────────

state: dict = {
    "robot_ip": "192.168.1.100",
    "robot_port": 9090,
    "ros_topic": "/scan",
    "connected": False,
    "scan_count": 0,
    "scan_rate_hz": 0.0,
    "last_scan_time": None,
    "latest_scan": None,
}

scan_history: list[dict] = []          # rolling last-30 scan summaries
_listener_task: Optional[asyncio.Task] = None
_prev_scan_epoch: float = 0.0          # used for Hz calculation


# ── Scan processing ───────────────────────────────────────────────────────────

def _process_scan(msg: dict) -> None:
    """Convert a raw rosbridge LaserScan message into Cartesian points."""
    global _prev_scan_epoch

    angle_min: float = msg.get("angle_min", -math.pi)
    angle_inc: float = msg.get("angle_increment", 0.0)
    range_min: float = msg.get("range_min", 0.0)
    range_max: float = msg.get("range_max", 30.0)
    raw_ranges: list = msg.get("ranges", [])

    points: list[dict] = []
    for i, r in enumerate(raw_ranges):
        # Skip invalid readings
        if r != r or math.isinf(r):          # NaN or Inf
            continue
        if r < range_min or r > range_max:
            continue
        angle = angle_min + i * angle_inc
        points.append({
            "x": round(r * math.cos(angle), 4),
            "y": round(r * math.sin(angle), 4),
            "range": round(r, 4),
            "angle_deg": round(math.degrees(angle), 2),
        })

    now_epoch = time.time()
    now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

    # Scan rate
    if _prev_scan_epoch > 0:
        delta = now_epoch - _prev_scan_epoch
        state["scan_rate_hz"] = round(1.0 / delta, 1) if delta > 0 else 0.0
    _prev_scan_epoch = now_epoch

    state["scan_count"] += 1
    state["last_scan_time"] = now_str

    ranges_valid = [p["range"] for p in points]
    state["latest_scan"] = {
        "timestamp": now_str,
        "point_count": len(points),
        "points": points,
        "range_min_measured": round(min(ranges_valid), 3) if ranges_valid else 0.0,
        "range_max_measured": round(max(ranges_valid), 3) if ranges_valid else 0.0,
        "sensor_range_min": range_min,
        "sensor_range_max": range_max,
    }

    # Append to rolling history (keep last 30)
    scan_history.append({
        "timestamp": now_str,
        "min_range": round(min(ranges_valid), 3) if ranges_valid else 0.0,
        "max_range": round(max(ranges_valid), 3) if ranges_valid else 0.0,
        "mean_range": round(sum(ranges_valid) / len(ranges_valid), 3) if ranges_valid else 0.0,
        "point_count": len(points),
    })
    if len(scan_history) > 30:
        scan_history.pop(0)


# ── rosbridge listener ────────────────────────────────────────────────────────

async def _rosbridge_listener() -> None:
    """Continuously connect to rosbridge and stream LaserScan messages."""
    while True:
        url = f"ws://{state['robot_ip']}:{state['robot_port']}"
        try:
            async with websockets.connect(
                url, open_timeout=5, ping_interval=20, ping_timeout=10
            ) as ws:
                state["connected"] = True
                # Subscribe to the LaserScan topic
                await ws.send(json.dumps({
                    "op": "subscribe",
                    "topic": state["ros_topic"],
                    "type": "sensor_msgs/LaserScan",
                }))
                async for raw in ws:
                    try:
                        msg = json.loads(raw)
                    except json.JSONDecodeError:
                        continue
                    if msg.get("op") == "publish" and msg.get("topic") == state["ros_topic"]:
                        _process_scan(msg["msg"])
        except asyncio.CancelledError:
            state["connected"] = False
            return
        except Exception:
            state["connected"] = False
        # Wait before reconnecting
        await asyncio.sleep(3)


def _start_listener() -> asyncio.Task:
    return asyncio.create_task(_rosbridge_listener())


# ── FastAPI lifespan ──────────────────────────────────────────────────────────

@asynccontextmanager
async def lifespan(application: FastAPI):
    global _listener_task
    _listener_task = _start_listener()
    yield
    _listener_task.cancel()
    try:
        await _listener_task
    except asyncio.CancelledError:
        pass


app = FastAPI(lifespan=lifespan)


# ── Config model ──────────────────────────────────────────────────────────────

class LidarConfig(BaseModel):
    robot_ip: str
    robot_port: int = 9090
    ros_topic: str = "/scan"


# ── Endpoints ─────────────────────────────────────────────────────────────────

@app.get("/")
def root():
    return {
        "service": "LiDAR Bridge Service",
        "status": "running",
        "connected": state["connected"],
        "robot": f"{state['robot_ip']}:{state['robot_port']}",
        "topic": state["ros_topic"],
    }


@app.get("/scan/status")
def scan_status():
    """Connection + scan statistics."""
    return {
        "connected": state["connected"],
        "robot_ip": state["robot_ip"],
        "robot_port": state["robot_port"],
        "ros_topic": state["ros_topic"],
        "scan_count": state["scan_count"],
        "scan_rate_hz": state["scan_rate_hz"],
        "last_scan_time": state["last_scan_time"],
    }


@app.get("/scan/latest")
def scan_latest():
    """Latest processed LiDAR scan with Cartesian points."""
    return state["latest_scan"]


@app.get("/scan/history")
def scan_history_endpoint():
    """Last 30 scan summaries (min/max/mean range, point count)."""
    return {"history": scan_history, "count": len(scan_history)}


@app.post("/config")
async def update_config(config: LidarConfig):
    """Update robot connection settings and restart the listener."""
    global _listener_task

    state["robot_ip"] = config.robot_ip
    state["robot_port"] = config.robot_port
    state["ros_topic"] = config.ros_topic
    state["connected"] = False

    # Restart listener with new config
    if _listener_task and not _listener_task.done():
        _listener_task.cancel()
        try:
            await _listener_task
        except asyncio.CancelledError:
            pass
    _listener_task = _start_listener()

    return {
        "status": "reconnecting",
        "robot_ip": state["robot_ip"],
        "robot_port": state["robot_port"],
        "ros_topic": state["ros_topic"],
    }


if __name__ == "__main__":
    print("Starting LiDAR Bridge Service on port 8005...")
    print(f"Connecting to rosbridge at ws://{state['robot_ip']}:{state['robot_port']}")
    uvicorn.run(app, host="0.0.0.0", port=8005)
