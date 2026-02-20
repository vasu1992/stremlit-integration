"""
Dummy LiDAR Service — generates synthetic 2D laser-scan data for visualization.

Simulates a robot placed at the origin inside a rectangular room with a
configurable number of moving circular obstacles.  The scan data mimics what
a real sensor_msgs/LaserScan would look like in RViz:
  • 360 rays, one per degree
  • Range values clipped to [range_min, range_max]
  • Gaussian noise added to each reading
  • Cartesian (x, y) coordinates returned alongside polar (range, angle_deg)

The simulation loop runs at ~10 Hz via asyncio so the REST endpoints always
serve fresh data without any blocking.

Port: 8006
"""

import asyncio
import math
import random
import time
from contextlib import asynccontextmanager
from datetime import datetime
from typing import Optional

import uvicorn
from fastapi import FastAPI
from pydantic import BaseModel

# ── Default simulation parameters (mutable via /config) ───────────────────────

_cfg: dict = {
    "room_width":     10.0,   # metres – walls at ± room_width/2 on X
    "room_height":    8.0,    # metres – walls at ± room_height/2 on Y
    "num_obstacles":  3,
    "obstacle_radius": 0.4,   # metres
    "noise_std":      0.02,   # Gaussian noise std-dev (metres)
    "num_rays":       360,
    "range_min":      0.12,
    "range_max":      12.0,
    "scan_hz_target": 10.0,
}

# ── Shared mutable state ───────────────────────────────────────────────────────

_state: dict = {
    "scan_count":     0,
    "scan_rate_hz":   0.0,
    "last_scan_time": None,
    "latest_scan":    None,
}

_obstacles: list[dict] = []          # list of {x, y, vx, vy}
_scan_history: list[dict] = []       # rolling last-30 scan summaries
_prev_epoch: float = 0.0
_sim_task: Optional[asyncio.Task] = None


# ── Obstacle initialisation ────────────────────────────────────────────────────

def _init_obstacles(n: int) -> None:
    global _obstacles
    hw = _cfg["room_width"]  / 2 - _cfg["obstacle_radius"] - 0.2
    hh = _cfg["room_height"] / 2 - _cfg["obstacle_radius"] - 0.2
    _obstacles = [
        {
            "x":  random.uniform(-hw * 0.6, hw * 0.6),
            "y":  random.uniform(-hh * 0.6, hh * 0.6),
            "vx": random.choice([-1, 1]) * random.uniform(0.02, 0.06),
            "vy": random.choice([-1, 1]) * random.uniform(0.02, 0.06),
        }
        for _ in range(n)
    ]


# ── Ray-casting helpers ────────────────────────────────────────────────────────

def _wall_distance(angle: float) -> float:
    """Smallest positive t along (cos θ, sin θ) that hits the room boundary."""
    c, s = math.cos(angle), math.sin(angle)
    hw = _cfg["room_width"]  / 2
    hh = _cfg["room_height"] / 2
    dists: list[float] = []
    if c >  1e-9:
        dists.append( hw / c)
    elif c < -1e-9:
        dists.append(-hw / c)
    if s >  1e-9:
        dists.append( hh / s)
    elif s < -1e-9:
        dists.append(-hh / s)
    return min(dists)


def _circle_distance(angle: float, cx: float, cy: float, r: float) -> Optional[float]:
    """Distance from origin to a circle obstacle along angle θ. None = miss."""
    dx, dy = math.cos(angle), math.sin(angle)
    # solve: ||(dx,dy)*t - (cx,cy)||² = r²
    b = cx * dx + cy * dy
    disc = b * b - (cx * cx + cy * cy - r * r)
    if disc < 0:
        return None
    t = b - math.sqrt(disc)
    return t if t > 0 else None


# ── Scan generation ────────────────────────────────────────────────────────────

def _generate_scan() -> None:
    global _prev_epoch

    # 1. Move obstacles, bounce off inner walls
    hw = _cfg["room_width"]  / 2 - _cfg["obstacle_radius"]
    hh = _cfg["room_height"] / 2 - _cfg["obstacle_radius"]
    for obs in _obstacles:
        obs["x"] += obs["vx"]
        obs["y"] += obs["vy"]
        if not (-hw <= obs["x"] <= hw):
            obs["vx"] *= -1
            obs["x"] = max(-hw, min(hw, obs["x"]))
        if not (-hh <= obs["y"] <= hh):
            obs["vy"] *= -1
            obs["y"] = max(-hh, min(hh, obs["y"]))

    # 2. Cast rays
    n_rays = _cfg["num_rays"]
    r_min  = _cfg["range_min"]
    r_max  = _cfg["range_max"]
    noise  = _cfg["noise_std"]
    obs_r  = _cfg["obstacle_radius"]

    points: list[dict] = []
    for i in range(n_rays):
        angle = -math.pi + i * (2 * math.pi / n_rays)
        r = _wall_distance(angle)
        for obs in _obstacles:
            d = _circle_distance(angle, obs["x"], obs["y"], obs_r)
            if d is not None:
                r = min(r, d)
        r = max(r_min, min(r_max, r + random.gauss(0, noise)))
        points.append({
            "x":         round(r * math.cos(angle), 4),
            "y":         round(r * math.sin(angle), 4),
            "range":     round(r, 4),
            "angle_deg": round(math.degrees(angle), 2),
        })

    # 3. Compute scan rate
    now_epoch = time.time()
    now_str   = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    if _prev_epoch > 0:
        delta = now_epoch - _prev_epoch
        _state["scan_rate_hz"] = round(1.0 / delta, 1) if delta > 0 else 0.0
    _prev_epoch = now_epoch
    _state["scan_count"] += 1
    _state["last_scan_time"] = now_str

    # 4. Store latest scan
    ranges = [p["range"] for p in points]
    _state["latest_scan"] = {
        "timestamp":          now_str,
        "point_count":        len(points),
        "points":             points,
        "range_min_measured": round(min(ranges), 3),
        "range_max_measured": round(max(ranges), 3),
        "sensor_range_min":   r_min,
        "sensor_range_max":   r_max,
        # obstacle centres for overlay in the dashboard
        "obstacles": [
            {"x": round(o["x"], 3), "y": round(o["y"], 3)}
            for o in _obstacles
        ],
        # room corners for boundary drawing in the dashboard
        "room": {
            "width":  _cfg["room_width"],
            "height": _cfg["room_height"],
        },
    }

    # 5. Rolling history (last 30)
    _scan_history.append({
        "timestamp":   now_str,
        "min_range":   round(min(ranges), 3),
        "max_range":   round(max(ranges), 3),
        "mean_range":  round(sum(ranges) / len(ranges), 3),
        "point_count": len(points),
    })
    if len(_scan_history) > 30:
        _scan_history.pop(0)


# ── Background simulation loop ─────────────────────────────────────────────────

async def _simulation_loop() -> None:
    interval = 1.0 / _cfg["scan_hz_target"]
    while True:
        _generate_scan()
        await asyncio.sleep(interval)


# ── FastAPI lifespan ───────────────────────────────────────────────────────────

@asynccontextmanager
async def lifespan(application: FastAPI):
    global _sim_task
    _init_obstacles(_cfg["num_obstacles"])
    _sim_task = asyncio.create_task(_simulation_loop())
    yield
    _sim_task.cancel()
    try:
        await _sim_task
    except asyncio.CancelledError:
        pass


app = FastAPI(lifespan=lifespan)


# ── Config model ───────────────────────────────────────────────────────────────

class SimConfig(BaseModel):
    room_width:    float = 10.0
    room_height:   float = 8.0
    num_obstacles: int   = 3
    noise_std:     float = 0.02


# ── Endpoints ──────────────────────────────────────────────────────────────────

@app.get("/")
def root():
    return {
        "service":       "Dummy LiDAR Simulator",
        "status":        "running",
        "room_width":    _cfg["room_width"],
        "room_height":   _cfg["room_height"],
        "num_obstacles": len(_obstacles),
        "scan_hz":       _state["scan_rate_hz"],
    }


@app.get("/scan/status")
def scan_status():
    """Simulation status — always 'connected' because data is generated locally."""
    return {
        "connected":     True,
        "simulation":    True,
        "room_width":    _cfg["room_width"],
        "room_height":   _cfg["room_height"],
        "num_obstacles": len(_obstacles),
        "scan_count":    _state["scan_count"],
        "scan_rate_hz":  _state["scan_rate_hz"],
        "last_scan_time":_state["last_scan_time"],
    }


@app.get("/scan/latest")
def scan_latest():
    """Latest generated scan: Cartesian points, obstacle centres, room dims."""
    return _state["latest_scan"]


@app.get("/scan/history")
def scan_history_endpoint():
    """Last 30 scan summaries (min/max/mean range, point count)."""
    return {"history": _scan_history, "count": len(_scan_history)}


@app.post("/config")
def update_config(config: SimConfig):
    """Update room size / obstacle count / noise and restart obstacles."""
    _cfg["room_width"]    = max(2.0, config.room_width)
    _cfg["room_height"]   = max(2.0, config.room_height)
    _cfg["noise_std"]     = max(0.0, config.noise_std)
    _cfg["num_obstacles"] = max(0, config.num_obstacles)
    _init_obstacles(_cfg["num_obstacles"])
    return {
        "status":        "updated",
        "room_width":    _cfg["room_width"],
        "room_height":   _cfg["room_height"],
        "num_obstacles": len(_obstacles),
        "noise_std":     _cfg["noise_std"],
    }


if __name__ == "__main__":
    print("Starting Dummy LiDAR Simulator on port 8006…")
    print(f"  Room: {_cfg['room_width']} × {_cfg['room_height']} m  |  "
          f"Obstacles: {_cfg['num_obstacles']}  |  "
          f"Scan rate: {_cfg['scan_hz_target']} Hz")
    uvicorn.run(app, host="0.0.0.0", port=8006)
