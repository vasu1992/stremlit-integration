# Microservices + Streamlit Dashboard

A microservices architecture with a Streamlit dashboard covering user management, analytics, interactive maps, autonomous vehicle fleet telemetry, live LiDAR visualization via ROS/rosbridge, and a built-in dummy LiDAR simulator that requires no real robot.

---

## Project Structure

```
.
├── streamlit_app.py          # Unified dashboard frontend    (Port 8501)
├── user_service.py           # User management service       (Port 8011)
├── analytics_service.py      # Event analytics service       (Port 8012)
├── location_service.py       # Location / map service        (Port 8003)
├── vehicle_service.py        # AV fleet telemetry service    (Port 8004)
├── lidar_service.py          # RViz LiDAR bridge service     (Port 8005)
├── dummy_lidar_service.py    # Dummy LiDAR simulator service (Port 8006)
├── requirements.txt          # Python dependencies
└── README.md                 # This file
```

---

## Services Overview

### User Service — Port 8011
Manages a fleet of users stored in memory.

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Health check |
| `/users` | GET | List all users |
| `/users` | POST | Create user `{name, email}` |
| `/users/{id}` | GET | Get user by ID (404 if not found) |
| `/users/{id}` | DELETE | Delete user by ID |

### Analytics Service — Port 8012
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

### Dummy LiDAR Simulator — Port 8006
Generates synthetic 2-D laser-scan data entirely in software — **no robot or ROS installation required**. Simulates a robot at the origin of a rectangular room with a configurable number of moving circular obstacles. Mimics what a real `sensor_msgs/LaserScan` looks like in RViz.

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Health check + room / obstacle info |
| `/scan/status` | GET | Scan rate (Hz), total scan count, room dimensions |
| `/scan/latest` | GET | Latest scan — Cartesian points, obstacle centres, room dims |
| `/scan/history` | GET | Last 30 scan summaries (min/max/mean range, point count) |
| `/config` | POST | Update `{room_width, room_height, num_obstacles, noise_std}` |

**Simulation details:**
- **360 rays** cast per scan (1 ° angular resolution)
- Ray–AABB intersection for rectangular room walls
- Ray–sphere intersection for circular moving obstacles
- Gaussian noise applied to every range reading (`noise_std` metres)
- Obstacles bounce off inner walls and move continuously
- Scan loop runs at **10 Hz** in an asyncio background task
- `/scan/latest` response includes `obstacles` (centres) and `room` (dims) for dashboard overlay

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
| 🤖 **LiDAR Sim** | Dummy RViz-style point cloud from the simulator: layered Altair scatter (room boundary, scan points coloured by range, obstacle markers, robot origin), scan history chart, live config, auto-refresh |

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
python user_service.py          # → http://localhost:8011

# Terminal 2
python analytics_service.py     # → http://localhost:8012

# Terminal 3
python location_service.py      # → http://localhost:8003

# Terminal 4
python vehicle_service.py       # → http://localhost:8004

# Terminal 5  (optional — requires a ROS robot on the network)
python lidar_service.py         # → http://localhost:8005

# Terminal 6  (optional — no robot needed, runs standalone)
python dummy_lidar_service.py   # → http://localhost:8006

# Terminal 7
streamlit run streamlit_app.py  # → http://localhost:8501
```

> The dashboard requires **User Service** and **Analytics Service** to be running.
> Location, Vehicle, LiDAR Bridge, and Dummy LiDAR services are handled gracefully per-page — the rest of the dashboard still works if they are offline.

### 3. LiDAR Setup (Robot Side — real LiDAR only)

On the ROS robot:
```bash
# ROS 1
roslaunch rosbridge_server rosbridge_websocket.launch

# ROS 2
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Then open the **📡 LiDAR** page in the dashboard, expand **⚙️ Connection Settings**, enter the robot's IP address, and click **🔌 Apply & Reconnect**.

> To try LiDAR visualization without a robot, use the **🤖 LiDAR Sim** page instead — start `dummy_lidar_service.py` and navigate to that page.

---

## Example API Calls

```bash
# Add a user
curl -X POST "http://localhost:8011/users" \
  -H "Content-Type: application/json" \
  -d '{"name": "Jane", "email": "jane@example.com"}'

# Log an analytics event
curl -X POST "http://localhost:8012/log?event_name=purchase&value=99.5"

# Add a map location
curl -X POST "http://localhost:8003/locations" \
  -H "Content-Type: application/json" \
  -d '{"name": "HQ", "latitude": 37.7749, "longitude": -122.4194, "category": "Office"}'

# Simulate one AV telemetry step
curl -X POST "http://localhost:8004/fleet/simulate"

# Check LiDAR bridge connection status
curl "http://localhost:8005/scan/status"

# Point LiDAR bridge at a different robot
curl -X POST "http://localhost:8005/config" \
  -H "Content-Type: application/json" \
  -d '{"robot_ip": "10.0.0.42", "robot_port": 9090, "ros_topic": "/scan"}'

# Get latest dummy LiDAR scan (points + obstacle positions + room dims)
curl "http://localhost:8006/scan/latest"

# Reconfigure the dummy LiDAR simulator (bigger room, more obstacles)
curl -X POST "http://localhost:8006/config" \
  -H "Content-Type: application/json" \
  -d '{"room_width": 16.0, "room_height": 12.0, "num_obstacles": 5, "noise_std": 0.03}'
```

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        STREAMLIT  (Port 8501)                           │
│  Home · Users · Analytics · Reports · Map · Fleet KPIs · LiDAR · Sim   │
└───┬──────────┬──────────┬──────────┬──────────┬────────────┬────────────┘
    │          │          │          │          │            │
    ▼          ▼          ▼          ▼          ▼            ▼
┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐ ┌──────────┐ ┌──────────────┐
│  User  │ │Analyt. │ │ Loctn. │ │Vehicle │ │  LiDAR   │ │ Dummy LiDAR  │
│  Svc   │ │  Svc   │ │  Svc   │ │  Svc   │ │  Bridge  │ │  Simulator   │
│ :8011  │ │ :8012  │ │ :8003  │ │ :8004  │ │  :8005   │ │    :8006     │
└────────┘ └────────┘ └────────┘ └────────┘ └────┬─────┘ └──────────────┘
                                                  │ WebSocket        ▲
                                          ┌───────▼──────────┐       │
                                          │   ROS Robot       │  asyncio
                                          │ rosbridge_server  │  loop —
                                          │    :9090 (WiFi)   │  no robot
                                          │                   │  needed
                                          │ /scan → LaserScan │
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

> `dummy_lidar_service.py` uses only packages already in the list above (fastapi, uvicorn, pydantic) — no extra dependencies.

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `pip install` fails on `pandas` / `numpy` | You are on a very new Python version. The pinned versions in `requirements.txt` have pre-built wheels for Python 3.12–3.14. Run `pip install -r requirements.txt` again. |
| Dashboard shows "Services not running" | Start `user_service.py` (port 8011) and `analytics_service.py` (port 8012) first — they are required. |
| Map page shows error | Start `location_service.py` in a separate terminal. |
| Fleet KPIs page shows error | Start `vehicle_service.py` in a separate terminal. |
| LiDAR shows "🔴 Disconnected" | Normal when robot is off. Start `lidar_service.py`, then set the correct robot IP in Connection Settings. |
| LiDAR Sim page shows error | Start `dummy_lidar_service.py` in a separate terminal (`python dummy_lidar_service.py`). |
| LiDAR Sim scan rate reads 0.0 Hz | Service just started — wait one second for the asyncio loop to produce the first scan. |
| "Port already in use" | Another process holds the port. Kill it or change the port constant in the relevant service file. |
| Streamlit doesn't open browser | Navigate manually to `http://localhost:8501`. |
