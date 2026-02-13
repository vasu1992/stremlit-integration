import random
from datetime import datetime
from fastapi import FastAPI
import uvicorn

app = FastAPI()

# ── Initial fleet state ──────────────────────────────────────────────────────
# 5 vehicles seeded around San Francisco (37.77°N, 122.41°W)
_BASE_LAT = 37.7749
_BASE_LON = -122.4194

_INITIAL_OFFSETS = [
    ( 0.010,  0.005),
    (-0.008,  0.012),
    ( 0.003, -0.010),
    (-0.015, -0.007),
    ( 0.012, -0.015),
]

vehicles_db: list[dict] = []
history_db: list[dict] = []


def _make_vehicle(index: int) -> dict:
    lat_off, lon_off = _INITIAL_OFFSETS[index]
    vid = f"AV-{index + 1:03d}"
    return {
        "id": vid,
        "name": f"Autonomous Vehicle {index + 1}",
        "status": random.choices(
            ["Active", "Idle", "Charging", "Error"],
            weights=[0.65, 0.20, 0.10, 0.05],
        )[0],
        "speed": round(random.uniform(20, 80), 1),
        "battery": round(random.uniform(40, 95), 1),
        "latitude": round(_BASE_LAT + lat_off, 6),
        "longitude": round(_BASE_LON + lon_off, 6),
        "distance_km": round(random.uniform(0, 200), 1),
        "cpu_usage": round(random.uniform(20, 60), 1),
        "sensor_health": "OK",
        "last_updated": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
    }


def _simulate_step(vehicle: dict) -> dict:
    """Advance one vehicle by one simulation step."""
    # 5 % chance to change status
    if random.random() < 0.05:
        vehicle["status"] = random.choices(
            ["Active", "Idle", "Charging", "Error"],
            weights=[0.65, 0.20, 0.10, 0.05],
        )[0]

    status = vehicle["status"]

    if status == "Active":
        vehicle["speed"] = round(random.uniform(20, 120), 1)
        vehicle["battery"] = round(
            max(5.0, vehicle["battery"] - random.uniform(0.1, 0.5)), 1
        )
        vehicle["distance_km"] = round(
            vehicle["distance_km"] + vehicle["speed"] / 360, 2
        )
        vehicle["latitude"] = round(
            vehicle["latitude"] + random.uniform(-0.001, 0.001), 6
        )
        vehicle["longitude"] = round(
            vehicle["longitude"] + random.uniform(-0.001, 0.001), 6
        )
        vehicle["cpu_usage"] = round(random.uniform(20, 95), 1)
    elif status == "Charging":
        vehicle["speed"] = 0
        vehicle["battery"] = round(
            min(100.0, vehicle["battery"] + random.uniform(1.0, 3.0)), 1
        )
        vehicle["cpu_usage"] = round(random.uniform(5, 20), 1)
    elif status == "Idle":
        vehicle["speed"] = 0
        vehicle["battery"] = round(
            max(5.0, vehicle["battery"] - random.uniform(0.0, 0.1)), 1
        )
        vehicle["cpu_usage"] = round(random.uniform(5, 30), 1)
    else:  # Error
        vehicle["speed"] = 0
        vehicle["cpu_usage"] = round(random.uniform(5, 30), 1)

    # Sensor health
    if status == "Error":
        vehicle["sensor_health"] = "Error"
    elif vehicle["battery"] < 20:
        vehicle["sensor_health"] = "Warning"
    else:
        vehicle["sensor_health"] = "OK"

    vehicle["last_updated"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    return vehicle


# Initialise fleet on startup
for i in range(5):
    vehicles_db.append(_make_vehicle(i))


# ── Endpoints ─────────────────────────────────────────────────────────────────

@app.get("/")
def root():
    return {"service": "Vehicle Telemetry Service", "status": "running"}


@app.post("/fleet/simulate")
def simulate():
    """Advance all vehicles one step and record telemetry history."""
    global history_db
    for vehicle in vehicles_db:
        _simulate_step(vehicle)
        history_db.append({
            "vehicle_id": vehicle["id"],
            "timestamp": vehicle["last_updated"],
            "speed": vehicle["speed"],
            "battery": vehicle["battery"],
            "latitude": vehicle["latitude"],
            "longitude": vehicle["longitude"],
        })
    # Keep only the last 100 history records
    history_db = history_db[-100:]
    return {"status": "simulated", "vehicles": len(vehicles_db)}


@app.get("/fleet/vehicles")
def get_vehicles():
    """Current state of all vehicles."""
    return {"vehicles": vehicles_db, "count": len(vehicles_db)}


@app.get("/fleet/kpis")
def get_kpis():
    """Aggregated fleet KPIs."""
    statuses = [v["status"] for v in vehicles_db]
    batteries = [v["battery"] for v in vehicles_db]
    speeds = [v["speed"] for v in vehicles_db if v["status"] == "Active"]
    error_count = statuses.count("Error")

    if error_count > 0:
        fleet_health = "Critical"
    elif any(b < 20 for b in batteries):
        fleet_health = "Warning"
    else:
        fleet_health = "Good"

    return {
        "total_vehicles": len(vehicles_db),
        "active": statuses.count("Active"),
        "idle": statuses.count("Idle"),
        "charging": statuses.count("Charging"),
        "error": error_count,
        "avg_speed_kmh": round(sum(speeds) / len(speeds), 1) if speeds else 0.0,
        "avg_battery_pct": round(sum(batteries) / len(batteries), 1),
        "total_distance_km": round(sum(v["distance_km"] for v in vehicles_db), 1),
        "fleet_health": fleet_health,
    }


@app.get("/fleet/history")
def get_history():
    """Last 100 telemetry readings across all vehicles."""
    return {"history": history_db, "count": len(history_db)}


if __name__ == "__main__":
    print("Starting Vehicle Telemetry Service on port 8004...")
    uvicorn.run(app, host="0.0.0.0", port=8004)
