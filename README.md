# Microservices + Streamlit Dashboard

A microservices architecture with a Streamlit dashboard covering user management, analytics, interactive maps, autonomous vehicle fleet telemetry, and live LiDAR visualization via ROS/rosbridge.

---

## Project Structure

```
.
├── streamlit_app.py        # Unified dashboard frontend  (Port 8501)
├── user_service.py         # User management service     (Port 8001)
├── analytics_service.py    # Event analytics service     (Port 8002)
├── location_service.py     # Location / map service      (Port 8003)
├── vehicle_service.py      # AV fleet telemetry service  (Port 8004)
├── lidar_service.py        # RViz LiDAR bridge service   (Port 8005)
├── requirements.txt        # Python dependencies
└── README.md               # This file
```

---

## Services Overview

### User Service — Port 8001
Manages a fleet of users stored in memory.

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Health check |
| `/users` | GET | List all users |
| `/users` | POST | Create user `{name, email}` |
| `/users/{id}` | GET | Get user by ID (404 if not found) |
| `/users/{id}` | DELETE | Delete user by ID |

### Analytics Service — Port 8002
Logs and summarises arbitrary named events with numeric values.

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Health check |
| `/events` | GET | List all events |
| `/log` | POST | Log event `?event_name=&value=` |
| `/summary` | GET | Aggregate stats (total, avg, min, max) |
| `/events` | DELETE | Clear all events |

### Location Service — Port 8003
Stores named geographic locations with a category.

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Health check |
| `/locations` | GET | List all locations |
| `/locations` | POST | Add location `{name, latitude, longitude, category}` |
| `/locations/{id}` | DELETE | Delete location (404 if not found) |

**Categories:** Restaurant · Hotel · Office · Park · Other

### Vehicle Telemetry Service — Port 8004
Simulates and tracks telemetry for a fleet of 5 autonomous vehicles seeded around San Francisco.

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Health check |
| `/fleet/simulate` | POST | Advance all vehicles one simulation step |
| `/fleet/vehicles` | GET | Current state of all vehicles |
| `/fleet/kpis` | GET | Aggregated KPIs (counts, avg speed/battery, fleet health) |
| `/fleet/history` | GET | Last 100 telemetry readings across all vehicles |

**Vehicle fields:** id, name, status, speed (km/h), battery (%), latitude, longitude, distance\_km, cpu\_usage (%), sensor\_health, last\_updated

**Simulation rules:**
- 5 % chance per step to change status (Active 65 % · Idle 20 % · Charging 10 % · Error 5 %)
- Active → speed 20–120 km/h, battery drains, GPS random-walks
- Charging → speed 0, battery charges up to 100 %
- Idle → speed 0, negligible battery drain
- Error → speed 0, sensor\_health = "Error"

### LiDAR Bridge Service — Port 8005
Bridges a ROS2/ROS1 robot running `rosbridge_server` over WiFi to a REST API. Subscribes to a `sensor_msgs/LaserScan` topic, converts polar data to Cartesian coordinates, and caches the results for the dashboard.

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Health check + connection state |
| `/scan/status` | GET | Connection info, scan rate (Hz), scan count |
| `/scan/latest` | GET | Latest scan — Cartesian points `[{x, y, range, angle_deg}]` |
| `/scan/history` | GET | Last 30 scan summaries (min/max/mean range, point count) |
| `/config` | POST | Update `{robot_ip, robot_port, ros_topic}` and reconnect |

**Default settings:** `ws://192.168.1.100:9090`, topic `/scan`
The service auto-reconnects every 3 s if the robot is unreachable.

---

## Dashboard Pages

| Page | Description |
|------|-------------|
| 🏠 **Home** | Service status indicators and quick fleet / event statistics |
| 👥 **Users** | Create users, view table, refresh |
| 📈 **Analytics** | Log events, summary metrics, value trend chart |
| 📊 **Reports** | Aggregate user and analytics reports |
| 🗺️ **Map** | Add named locations, view on interactive map (color-coded by category), delete entries |
| 🚗 **Fleet KPIs** | Simulate AV telemetry, KPI metrics, status donut chart, battery bar, speed trend, live position map, fleet table |
| 📡 **LiDAR** | Live LiDAR point cloud from rosbridge, scan statistics, history chart, auto-refresh |

---

## Quick Start

### 1. Install Dependencies

```bash
pip install -r requirements.txt
```

> **Python 3.12 / 3.13 / 3.14** — all packages are pinned to versions with pre-built wheels. No C compiler required.

### 2. Start the Services

Open a separate terminal for each service:

```bash
# Terminal 1
python user_service.py        # → http://localhost:8001

# Terminal 2
python analytics_service.py   # → http://localhost:8002

# Terminal 3
python location_service.py    # → http://localhost:8003

# Terminal 4
python vehicle_service.py     # → http://localhost:8004

# Terminal 5  (optional — requires a ROS robot on the network)
python lidar_service.py       # → http://localhost:8005

# Terminal 6
streamlit run streamlit_app.py  # → http://localhost:8501
```

> The dashboard requires **User Service** and **Analytics Service** to be running.
> Location, Vehicle, and LiDAR services are handled gracefully per-page — the rest of the dashboard still works if they are offline.

### 3. LiDAR Setup (Robot Side)

On the ROS robot:
```bash
# ROS 1
roslaunch rosbridge_server rosbridge_websocket.launch

# ROS 2
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Then open the **📡 LiDAR** page in the dashboard, expand **⚙️ Connection Settings**, enter the robot's IP address, and click **🔌 Apply & Reconnect**.

---

## Example API Calls

```bash
# Add a user
curl -X POST "http://localhost:8001/users" \
  -H "Content-Type: application/json" \
  -d '{"name": "Jane", "email": "jane@example.com"}'

# Log an analytics event
curl -X POST "http://localhost:8002/log?event_name=purchase&value=99.5"

# Add a map location
curl -X POST "http://localhost:8003/locations" \
  -H "Content-Type: application/json" \
  -d '{"name": "HQ", "latitude": 37.7749, "longitude": -122.4194, "category": "Office"}'

# Simulate one AV telemetry step
curl -X POST "http://localhost:8004/fleet/simulate"

# Check LiDAR connection status
curl "http://localhost:8005/scan/status"

# Point LiDAR service at a different robot
curl -X POST "http://localhost:8005/config" \
  -H "Content-Type: application/json" \
  -d '{"robot_ip": "10.0.0.42", "robot_port": 9090, "ros_topic": "/scan"}'
```

---

## Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                  STREAMLIT  (Port 8501)                          │
│   Home · Users · Analytics · Reports · Map · Fleet KPIs · LiDAR │
└───┬──────────┬──────────┬──────────┬──────────┬─────────────────┘
    │          │          │          │          │
    ▼          ▼          ▼          ▼          ▼
┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐ ┌────────────────────┐
│  User  │ │Analyt. │ │ Loctn. │ │Vehicle │ │   LiDAR Bridge     │
│  Svc   │ │  Svc   │ │  Svc   │ │  Svc   │ │       Svc          │
│ :8001  │ │ :8002  │ │ :8003  │ │ :8004  │ │      :8005         │
└────────┘ └────────┘ └────────┘ └────────┘ └────────┬───────────┘
                                                      │ WebSocket
                                              ┌───────▼───────────┐
                                              │   ROS Robot        │
                                              │ rosbridge_server   │
                                              │    :9090 (WiFi)    │
                                              │                    │
                                              │ /scan → LaserScan  │
                                              └───────────────────┘
```

---

## Dependencies

| Package | Version | Used by |
|---------|---------|---------|
| fastapi | 0.129.0 | All services |
| uvicorn | 0.40.0 | All services |
| pydantic | 2.12.5 | All services |
| requests | 2.32.5 | streamlit_app.py |
| streamlit | 1.54.0 | streamlit_app.py |
| pandas | 2.3.3 | streamlit_app.py |
| websockets | 14.2 | lidar_service.py |

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `pip install` fails on `pandas` / `numpy` | You are on a very new Python version. The pinned versions in `requirements.txt` have pre-built wheels for Python 3.12–3.14. Run `pip install -r requirements.txt` again. |
| Dashboard shows "Services not running" | Start `user_service.py` and `analytics_service.py` first — they are required. |
| Map page shows error | Start `location_service.py` in a separate terminal. |
| Fleet KPIs page shows error | Start `vehicle_service.py` in a separate terminal. |
| LiDAR shows "🔴 Disconnected" | Normal when robot is off. Start `lidar_service.py`, then set the correct robot IP in Connection Settings. |
| "Port already in use" | Another process holds the port. Kill it or change the port constant in the relevant service file. |
| Streamlit doesn't open browser | Navigate manually to `http://localhost:8501`. |
