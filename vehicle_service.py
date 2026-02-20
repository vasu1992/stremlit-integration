import random
from datetime import datetime
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import uvicorn

app = FastAPI()

# ── Initial fleet state ──────────────────────────────────────────────────────
# 5 vehicles seeded around Magdeburg, Germany (52.12°N, 11.63°E)
_BASE_LAT = 52.1205
_BASE_LON = 11.6276

_INITIAL_OFFSETS = [
    ( 0.010,  0.005),
    (-0.008,  0.012),
    ( 0.003, -0.010),
    (-0.015, -0.007),
    ( 0.012, -0.015),
]

vehicles_db: list[dict] = []
history_db:  list[dict] = []
_next_id_counter = [6]   # mutable counter for new registrations


# ── Input models ──────────────────────────────────────────────────────────────

class VehicleRegistration(BaseModel):
    name:         str
    vehicle_type: str   = "Autonomous Vehicle"
    latitude:     float = _BASE_LAT
    longitude:    float = _BASE_LON


class PositionUpdate(BaseModel):
    latitude:    float
    longitude:   float
    heading_deg: float = 0.0
    speed:       float = 0.0


# ── Internal helpers ──────────────────────────────────────────────────────────

def _make_vehicle(index: int) -> dict:
    lat_off, lon_off = _INITIAL_OFFSETS[index]
    vid = f"AV-{index + 1:03d}"
    return {
        "id":           vid,
        "name":         f"Autonomous Vehicle {index + 1}",
        "status":       random.choices(
            ["Active", "Idle", "Charging", "Error"],
            weights=[0.65, 0.20, 0.10, 0.05],
        )[0],
        "speed":        round(random.uniform(20, 80), 1),
        "battery":      round(random.uniform(40, 95), 1),
        "latitude":     round(_BASE_LAT + lat_off, 6),
        "longitude":    round(_BASE_LON + lon_off, 6),
        "distance_km":  round(random.uniform(0, 200), 1),
        "cpu_usage":    round(random.uniform(20, 60), 1),
        "sensor_health":"OK",
        "teleop_active":False,
        "last_updated": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
    }


def _simulate_step(vehicle: dict) -> dict:
    """Advance one vehicle by one simulation step (skip teleop-controlled vehicles)."""
    if vehicle.get("teleop_active"):
        vehicle["last_updated"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        return vehicle

    if random.random() < 0.05:
        vehicle["status"] = random.choices(
            ["Active", "Idle", "Charging", "Error"],
            weights=[0.65, 0.20, 0.10, 0.05],
        )[0]

    status = vehicle["status"]

    if status == "Active":
        vehicle["speed"]       = round(random.uniform(20, 120), 1)
        vehicle["battery"]     = round(max(5.0, vehicle["battery"] - random.uniform(0.1, 0.5)), 1)
        vehicle["distance_km"] = round(vehicle["distance_km"] + vehicle["speed"] / 360, 2)
        vehicle["latitude"]    = round(vehicle["latitude"]  + random.uniform(-0.001, 0.001), 6)
        vehicle["longitude"]   = round(vehicle["longitude"] + random.uniform(-0.001, 0.001), 6)
        vehicle["cpu_usage"]   = round(random.uniform(20, 95), 1)
    elif status == "Charging":
        vehicle["speed"]   = 0
        vehicle["battery"] = round(min(100.0, vehicle["battery"] + random.uniform(1.0, 3.0)), 1)
        vehicle["cpu_usage"] = round(random.uniform(5, 20), 1)
    elif status == "Idle":
        vehicle["speed"]   = 0
        vehicle["battery"] = round(max(5.0, vehicle["battery"] - random.uniform(0.0, 0.1)), 1)
        vehicle["cpu_usage"] = round(random.uniform(5, 30), 1)
    else:  # Error
        vehicle["speed"]   = 0
        vehicle["cpu_usage"] = round(random.uniform(5, 30), 1)

    vehicle["sensor_health"] = (
        "Error"   if status == "Error"           else
        "Warning" if vehicle["battery"] < 20     else
        "OK"
    )
    vehicle["last_updated"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    return vehicle


# Initialise fleet on startup
for i in range(5):
    vehicles_db.append(_make_vehicle(i))


# ── Endpoints ─────────────────────────────────────────────────────────────────

@app.get("/")
def root():
    return {"service": "Vehicle Telemetry Service", "status": "running",
            "fleet_size": len(vehicles_db)}


@app.post("/fleet/simulate")
def simulate():
    """Advance all non-teleop vehicles one step and record telemetry history."""
    global history_db
    for vehicle in vehicles_db:
        _simulate_step(vehicle)
        history_db.append({
            "vehicle_id": vehicle["id"],
            "timestamp":  vehicle["last_updated"],
            "speed":      vehicle["speed"],
            "battery":    vehicle["battery"],
            "latitude":   vehicle["latitude"],
            "longitude":  vehicle["longitude"],
        })
    history_db = history_db[-100:]
    return {"status": "simulated", "vehicles": len(vehicles_db)}


@app.get("/fleet/vehicles")
def get_vehicles():
    """Current state of all vehicles."""
    return {"vehicles": vehicles_db, "count": len(vehicles_db)}


@app.get("/fleet/vehicles/{vehicle_id}")
def get_vehicle(vehicle_id: str):
    """Get a single vehicle by ID."""
    v = next((v for v in vehicles_db if v["id"] == vehicle_id), None)
    if not v:
        raise HTTPException(status_code=404, detail=f"Vehicle {vehicle_id!r} not found")
    return v


@app.post("/fleet/register")
def register_vehicle(body: VehicleRegistration):
    """Register a new vehicle and add it to the active fleet."""
    vid = f"AV-{_next_id_counter[0]:03d}"
    _next_id_counter[0] += 1
    vehicle = {
        "id":            vid,
        "name":          body.name,
        "status":        "Idle",
        "speed":         0.0,
        "battery":       100.0,
        "latitude":      round(body.latitude,  6),
        "longitude":     round(body.longitude, 6),
        "distance_km":   0.0,
        "cpu_usage":     0.0,
        "sensor_health": "OK",
        "teleop_active": False,
        "vehicle_type":  body.vehicle_type,
        "last_updated":  datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
    }
    vehicles_db.append(vehicle)
    return {"status": "registered", "vehicle": vehicle}


@app.delete("/fleet/vehicles/{vehicle_id}")
def deregister_vehicle(vehicle_id: str):
    """Remove a vehicle from the fleet."""
    global vehicles_db
    before = len(vehicles_db)
    vehicles_db = [v for v in vehicles_db if v["id"] != vehicle_id]
    if len(vehicles_db) == before:
        raise HTTPException(status_code=404, detail=f"Vehicle {vehicle_id!r} not found")
    return {"status": "deregistered", "vehicle_id": vehicle_id}


@app.patch("/fleet/vehicles/{vehicle_id}/position")
def update_position(vehicle_id: str, body: PositionUpdate):
    """Update a vehicle's GPS position and speed (called by teleop sync)."""
    v = next((v for v in vehicles_db if v["id"] == vehicle_id), None)
    if not v:
        raise HTTPException(status_code=404, detail=f"Vehicle {vehicle_id!r} not found")
    v["latitude"]     = round(body.latitude,    6)
    v["longitude"]    = round(body.longitude,   6)
    v["speed"]        = round(body.speed,       1)
    v["status"]       = "Active" if body.speed > 0.5 else "Idle"
    v["teleop_active"]= True
    v["last_updated"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    return {"status": "updated", "vehicle": v}


@app.post("/fleet/vehicles/{vehicle_id}/release")
def release_vehicle(vehicle_id: str):
    """Release a vehicle from teleop control back to autonomous simulation."""
    v = next((v for v in vehicles_db if v["id"] == vehicle_id), None)
    if not v:
        raise HTTPException(status_code=404, detail=f"Vehicle {vehicle_id!r} not found")
    v["teleop_active"] = False
    v["status"]        = "Idle"
    v["speed"]         = 0.0
    v["last_updated"]  = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    return {"status": "released", "vehicle": v}


@app.get("/fleet/kpis")
def get_kpis():
    """Aggregated fleet KPIs."""
    statuses    = [v["status"]  for v in vehicles_db]
    batteries   = [v["battery"] for v in vehicles_db]
    speeds      = [v["speed"]   for v in vehicles_db if v["status"] == "Active"]
    error_count = statuses.count("Error")

    fleet_health = (
        "Critical" if error_count > 0          else
        "Warning"  if any(b < 20 for b in batteries) else
        "Good"
    )
    return {
        "total_vehicles":    len(vehicles_db),
        "active":            statuses.count("Active"),
        "idle":              statuses.count("Idle"),
        "charging":          statuses.count("Charging"),
        "error":             error_count,
        "avg_speed_kmh":     round(sum(speeds) / len(speeds), 1) if speeds else 0.0,
        "avg_battery_pct":   round(sum(batteries) / len(batteries), 1),
        "total_distance_km": round(sum(v["distance_km"] for v in vehicles_db), 1),
        "fleet_health":      fleet_health,
    }


@app.get("/fleet/history")
def get_history():
    """Last 100 telemetry readings across all vehicles."""
    return {"history": history_db, "count": len(history_db)}


if __name__ == "__main__":
    print("Starting Vehicle Telemetry Service on port 8004...")
    uvicorn.run(app, host="0.0.0.0", port=8004)
