# Operations Control Center

A microservices-based autonomous logistics platform with two complementary dashboards — a **Streamlit** operations dashboard and a **NiceGUI** real-time visualization app — covering user management, event analytics, AV fleet telemetry (Magdeburg, Germany), live LiDAR visualization via ROS/rosbridge, a built-in LiDAR simulator, a teleoperation interface with fleet integration, and a **live robot telemetry bridge** for Innok Robotics cloud-connected robots. No real robot required for most features.

---

## Project Structure

```
.
├── streamlit_app.py          # Streamlit operations dashboard     (Port 8501)
├── viz_app.py                # NiceGUI real-time visualization    (Port 8008)
├── user_service.py           # User management service            (Port 8081)
├── analytics_service.py      # Event analytics service            (Port 8082)
├── vehicle_service.py        # AV fleet telemetry service         (Port 8004)
├── lidar_service.py          # RViz LiDAR bridge service          (Port 8005)
├── dummy_lidar_service.py    # Dummy LiDAR simulator service      (Port 8006)
├── teleop_service.py         # Teleoperation interface service    (Port 8007)
├── robot_telemetry_service.py# Innok Robotics cloud telemetry    (Port 8083)
├── tugger train.png          # Home page vehicle image
├── cargo.jpg                 # Home page vehicle image
├── delivery robot.jpg        # Home page vehicle image
├── requirements.txt          # Python dependencies
└── README.md                 # This file
```

---

## Services Overview

### User Service — Port 8081
Manages a fleet of users stored in memory.

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Health check |
| `/users` | GET | List all users |
| `/users` | POST | Create user `{name, email}` |
| `/users/{id}` | GET | Get user by ID |
| `/users/{id}` | DELETE | Delete user by ID |

### Analytics Service — Port 8082
Logs and summarises arbitrary named events with numeric values.

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Health check |
| `/events` | GET | List all events |
| `/log` | POST | Log event `?event_name=&value=` |
| `/summary` | GET | Aggregate stats (total, avg, min, max) |
| `/events` | DELETE | Clear all events |

### Vehicle Telemetry Service — Port 8004
Simulates and tracks telemetry for a fleet of 5 autonomous vehicles seeded around **Magdeburg, Germany (52.12°N, 11.63°E)**. Supports dynamic vehicle registration and teleop position sync.

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Health check |
| `/fleet/simulate` | POST | Advance all vehicles one simulation step |
| `/fleet/vehicles` | GET | Current state of all vehicles |
| `/fleet/vehicles/{id}` | GET | Single vehicle by ID |
| `/fleet/kpis` | GET | KPIs — counts, avg speed/battery, fleet health |
| `/fleet/history` | GET | Last 100 telemetry readings |
| `/fleet/register` | POST | Register new vehicle `{name, vehicle_type, latitude, longitude}` |
| `/fleet/vehicles/{id}` | DELETE | Deregister vehicle from fleet |
| `/fleet/vehicles/{id}/position` | PATCH | Sync GPS position from teleop `{latitude, longitude, heading_deg, speed}` |
| `/fleet/vehicles/{id}/release` | POST | Return teleop-controlled vehicle to autonomous simulation |

**Simulation rules:**
- 5% chance per step to change status (Active 65% · Idle 20% · Charging 10% · Error 5%)
- Active → speed 20–120 km/h, battery drains, GPS random-walks around Magdeburg
- Charging → speed 0, battery charges up to 100%
- Vehicles under teleop control skip autonomous simulation until released

### LiDAR Bridge Service — Port 8005
Bridges a ROS2/ROS1 robot running `rosbridge_server` over WiFi to a REST API.

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Health check + connection state |
| `/scan/status` | GET | Connection info, scan rate (Hz), scan count |
| `/scan/latest` | GET | Latest scan — Cartesian points `[{x, y, range, angle_deg}]` |
| `/scan/history` | GET | Last 30 scan summaries |
| `/config` | POST | Update `{robot_ip, robot_port, ros_topic}` and reconnect |

**Default:** `ws://192.168.1.100:9090`, topic `/scan`. Auto-reconnects every 3s.

### Dummy LiDAR Simulator — Port 8006
Generates synthetic 2-D laser-scan data entirely in software — **no robot or ROS required**.

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Health check |
| `/scan/status` | GET | Scan rate (Hz), total scans, room dimensions |
| `/scan/latest` | GET | Latest scan — points, obstacle centres, room dims |
| `/scan/history` | GET | Last 30 scan summaries |
| `/config` | POST | Update `{room_width, room_height, num_obstacles, noise_std}` |

**Details:** 360 rays/scan at 10 Hz, ray–AABB room walls, moving circular obstacles, Gaussian noise.

### Teleoperation Service — Port 8007
Simulates remote control of a differential-drive robot with fleet vehicle integration and GPS tracking.

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Health check + robot status |
| `/robot/state` | GET | Full pose: x, y, heading, speed, battery, GPS lat/lon |
| `/robot/connection` | GET | Active fleet vehicle connection info |
| `/robot/connect` | POST | Connect to fleet vehicle `{vehicle_id, vehicle_name, battery, home_lat, home_lon}` |
| `/robot/disconnect` | POST | Disconnect from fleet vehicle |
| `/robot/cmd_vel` | POST | Apply velocity `{linear, angular, duration}` |
| `/robot/stop` | POST | Soft stop |
| `/robot/estop` | POST | Emergency stop |
| `/robot/reset` | POST | Return to origin, restore battery, clear history |
| `/robot/history` | GET | Last 50 commands |
| `/robot/path` | GET | Last 200 pose snapshots for trail chart |

**GPS tracking:** When connected to a fleet vehicle, the robot's local x/y displacement is converted to absolute GPS (lat/lon) and synced back to the vehicle service after every movement command.

### Robot Telemetry Service — Port 8083

Bridges an **Innok Robotics** cloud-connected robot to the local platform by polling the Innok REST API and caching the latest diagnostics and position data. Also accepts light-control commands that are forwarded to the robot's cloud API. Credentials are loaded from environment variables at startup and can be updated at runtime without restarting.

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Health check + connection state |
| `/robot/status` | GET | Connection state, last poll timestamp, any error |
| `/robot/diagnostics` | GET | Latest diagnostics (level, name, message, hardware_id, values) |
| `/robot/position` | GET | Latest pose — translation `{x,y,z}` + rotation quaternion `{x,y,z,w}` |
| `/robot/light` | GET | Current light state (`on` / `off`) and when it was last set |
| `/robot/light` | POST | Set light `{"state": "on"\|"off"}` — forwards to robot API and caches state |
| `/robot/poll` | POST | Trigger an immediate poll now |
| `/robot/config` | GET | Current config (password/apikey masked) |
| `/robot/config` | POST | Update credentials or poll interval at runtime |

**Innok API endpoints polled:**
- `GET /state/diagnostics?apikey=...` — hardware component status with key/value metrics
- `GET /state/position?apikey=...` — robot pose in a reference frame

**Credential environment variables:**

| Variable | Description |
|----------|-------------|
| `INNOK_TENANT` | Subdomain, e.g. `myrobot` → `myrobot.cloud.innok-robotics.de` |
| `INNOK_USER` | HTTP Basic auth username |
| `INNOK_PASSWORD` | HTTP Basic auth password |
| `INNOK_APIKEY` | `?apikey=` query parameter |
| `INNOK_POLL_INTERVAL` | Polling interval in seconds (default `5`) |
| `INNOK_VERIFY_SSL` | Set to `false` to skip TLS verification (default `true`) |

**Diagnostics level codes:** `0` = OK · `1` = WARN · `2` = ERROR

### NiceGUI Visualization App — Port 8008
Real-time visualization dashboard with three auto-refreshing tabs.

| Tab | Content | Refresh |
|-----|---------|---------|
| 🗺️ Fleet Map | Live vehicle markers on OpenStreetMap with status colours, KPI bar, hover telemetry | 3 s |
| 🕹️ Teleop | 2×2 SVG camera feeds (FRONT/RIGHT/LEFT/REAR) + Plotly trail chart + status bar | 1.5 s |
| 📡 LiDAR | Point cloud coloured by range, obstacles, room boundary toggle (Sim / Bridge) | 1.5 s |

---

## Dashboard Pages (Streamlit)

| Page | Description |
|------|-------------|
| 🏠 **Home** | Vehicle class showcase (Tugger Train, Cargo Bike, Delivery Robot), service health chips, quick fleet/event stats, link to NiceGUI Visualization |
| 👥 **Users** | Create users, view table, refresh |
| 📈 **Analytics** | Log events, summary metrics, value trend chart |
| 🚗 **Fleet KPIs** | Simulate AV telemetry, KPI metrics, status donut chart, battery bars, speed trend, live Magdeburg position map, fleet table, register/deregister vehicles |
| 📡 **LiDAR** | Live LiDAR point cloud from rosbridge, scan statistics, history chart |
| 🤖 **LiDAR Sim** | Synthetic RViz-style point cloud: room boundary, range-coloured scan points, obstacle markers, live config, auto-refresh |
| 🕹️ **Teleop** | Fleet vehicle connection panel, D-pad controls, camera feeds (4× synthetic SVG), robot trail chart, command history table, GPS sync to fleet map |
| 🦾 **Robot** | Light on/off toggle (forwards command to Innok REST API), live position (translation + quaternion), diagnostics table with per-component status, credential config form |

---

## Quick Start

### 1. Install Dependencies

```bash
pip install -r requirements.txt
```

### 2. Start the Services

Open a separate terminal for each service:

```bash
# Terminal 1  (required)
python user_service.py          # → http://localhost:8081

# Terminal 2  (required)
python analytics_service.py     # → http://localhost:8082

# Terminal 3
python vehicle_service.py       # → http://localhost:8004

# Terminal 4  (optional — requires a ROS robot on the network)
python lidar_service.py         # → http://localhost:8005

# Terminal 5  (optional — no robot needed)
python dummy_lidar_service.py   # → http://localhost:8006

# Terminal 6  (optional — no robot needed)
python teleop_service.py        # → http://localhost:8007

# Terminal 7  (optional — real-time NiceGUI visualization)
python viz_app.py               # → http://localhost:8008

# Terminal 8  (optional — Innok Robotics cloud robot bridge)
INNOK_TENANT=myrobot INNOK_USER=user INNOK_PASSWORD=pass INNOK_APIKEY=XXXXX \
python robot_telemetry_service.py   # → http://localhost:8083

# Terminal 9  (Streamlit dashboard)
streamlit run streamlit_app.py  # → http://localhost:8501
```

> The Streamlit dashboard requires **User Service (8081)** and **Analytics Service (8082)** to be running. All other services are handled gracefully — the rest of the dashboard works if they are offline.
>
> The Robot Telemetry Service can also be started without credentials — configure them later at runtime via `POST /robot/config`.

### 3. Innok Robotics Setup (Real Robot Only)

If you have an Innok Robotics cloud robot, configure credentials via env vars or at runtime:

```bash
# Option A — env vars at startup
INNOK_TENANT=myrobot INNOK_USER=user INNOK_PASSWORD=pass INNOK_APIKEY=XXXXX \
python robot_telemetry_service.py

# Option B — runtime config (start first, configure later)
python robot_telemetry_service.py

curl -X POST http://localhost:8083/robot/config \
  -H "Content-Type: application/json" \
  -d '{"tenant":"myrobot","username":"user","password":"pass","apikey":"XXXXX"}'

# Check connection status
curl http://localhost:8083/robot/status

# Fetch diagnostics
curl http://localhost:8083/robot/diagnostics

# Fetch position
curl http://localhost:8083/robot/position
```

### 4. LiDAR Setup (Real Robot Only)

On the ROS robot:
```bash
# ROS 1
roslaunch rosbridge_server rosbridge_websocket.launch

# ROS 2
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Then open **📡 LiDAR** in the dashboard → **⚙️ Connection Settings** → enter robot IP → **🔌 Apply & Reconnect**.

> For LiDAR without a robot, use **🤖 LiDAR Sim** instead.

### 5. Teleop Fleet Integration

1. Start `vehicle_service.py` and `teleop_service.py`
2. Navigate to **🕹️ Teleop** in the Streamlit dashboard
3. Expand **🔗 Fleet Vehicle Connection** and select an available vehicle
4. Click **🎮 Connect & Take Control** — the robot is seeded at the vehicle's GPS position
5. Use the D-pad to drive; the vehicle's position on the Fleet Map updates in real time
6. Click **🔌 Disconnect** to return the vehicle to autonomous simulation

---

## Architecture

```
┌──────────────────────────────────────────────────────────────────────────────┐
│               STREAMLIT Operations Dashboard  (Port 8501)                    │
│  Home · Users · Analytics · Fleet KPIs · LiDAR · LiDAR Sim · Teleop         │
└───┬──────────┬──────────┬─────────────┬────────────┬────────────┬────────────┘
    │          │          │             │            │            │
    ▼          ▼          ▼             ▼            ▼            ▼
┌────────┐ ┌────────┐ ┌────────┐  ┌──────────┐ ┌──────────┐ ┌──────────┐
│  User  │ │Analyt. │ │Vehicle │  │  LiDAR   │ │  Dummy   │ │  Teleop  │
│  Svc   │ │  Svc   │ │  Svc   │  │  Bridge  │ │  LiDAR   │ │   Svc    │
│ :8081  │ │ :8082  │ │ :8004  │  │  :8005   │ │  :8006   │ │  :8007   │
└────────┘ └────────┘ └────────┘  └────┬─────┘ └──────────┘ └────┬─────┘
               (Magdeburg, DE)         │ WebSocket          GPS sync │
                                 ┌─────▼──────────────┐            │
                                 │   ROS Robot         │            │
                                 │ rosbridge_server    │            │
                                 │    :9090 (WiFi)     │            │
                                 └─────────────────────┘            │
                                                                     ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│                 NiceGUI Real-Time Visualization  (Port 8008)                 │
│           Fleet Map  ·  Teleop Cameras + Trail  ·  LiDAR Point Cloud        │
└──────────────────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────────────────┐
│              Robot Telemetry Service  (Port 8009)                            │
│  Polls Innok Robotics cloud API · caches diagnostics + position              │
└───────────────────────────────┬──────────────────────────────────────────────┘
                                │ HTTPS / REST (polling)
                    ┌───────────▼───────────┐
                    │  Innok Robotics Cloud  │
                    │  <tenant>.cloud.       │
                    │  innok-robotics.de     │
                    │  /api/v1/state/...     │
                    └────────────────────────┘
```

---

## Example API Calls

```bash
# Add a user
curl -X POST "http://localhost:8081/users" \
  -H "Content-Type: application/json" \
  -d '{"name": "Jane", "email": "jane@example.com"}'

# Log an analytics event
curl -X POST "http://localhost:8082/log?event_name=purchase&value=99.5"

# Simulate one AV telemetry step
curl -X POST "http://localhost:8004/fleet/simulate"

# Register a new vehicle
curl -X POST "http://localhost:8004/fleet/register" \
  -H "Content-Type: application/json" \
  -d '{"name": "Scout Bot Alpha", "vehicle_type": "Delivery Robot", "latitude": 52.1205, "longitude": 11.6276}'

# Connect teleop to a fleet vehicle
curl -X POST "http://localhost:8007/robot/connect" \
  -H "Content-Type: application/json" \
  -d '{"vehicle_id": "AV-001", "vehicle_name": "Autonomous Vehicle 1", "battery": 85.0, "home_lat": 52.1305, "home_lon": 11.6326}'

# Drive the robot forward 1 second at 0.5 m/s
curl -X POST "http://localhost:8007/robot/cmd_vel" \
  -H "Content-Type: application/json" \
  -d '{"linear": 0.5, "angular": 0.0, "duration": 1.0}'

# Get dummy LiDAR scan
curl "http://localhost:8006/scan/latest"

# Reconfigure LiDAR simulator
curl -X POST "http://localhost:8006/config" \
  -H "Content-Type: application/json" \
  -d '{"room_width": 16.0, "room_height": 12.0, "num_obstacles": 5, "noise_std": 0.03}'

# Configure Innok robot credentials at runtime
curl -X POST "http://localhost:8083/robot/config" \
  -H "Content-Type: application/json" \
  -d '{"tenant":"myrobot","username":"user","password":"pass","apikey":"XXXXX"}'

# Get robot diagnostics
curl "http://localhost:8083/robot/diagnostics"

# Get robot position
curl "http://localhost:8083/robot/position"

# Trigger an immediate poll
curl -X POST "http://localhost:8083/robot/poll"

# Turn robot light on
curl -X POST "http://localhost:8083/robot/light" \
  -H "Content-Type: application/json" \
  -d '{"state": "on"}'

# Turn robot light off
curl -X POST "http://localhost:8083/robot/light" \
  -H "Content-Type: application/json" \
  -d '{"state": "off"}'

# Get current light state
curl "http://localhost:8083/robot/light"
```

---

## Dependencies

| Package | Used by |
|---------|---------|
| fastapi | All services |
| uvicorn | All services |
| pydantic | All services |
| websockets | lidar_service.py |
| requests | streamlit_app.py, viz_app.py |
| httpx | robot_telemetry_service.py |
| streamlit | streamlit_app.py |
| pandas | streamlit_app.py |
| altair | streamlit_app.py |
| nicegui | viz_app.py |
| plotly | viz_app.py |

Install all at once:
```bash
pip install -r requirements.txt
# plus: nicegui plotly altair
```

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| Dashboard shows "Core services offline" | Start `user_service.py` (8081) and `analytics_service.py` (8082) first. |
| Ports 8001/8002/8011/8012 blocked with WinError 10013 | Windows Hyper-V reserves these ports. Use ports 8081/8082 as configured. |
| Fleet KPIs page shows error | Start `vehicle_service.py` in a separate terminal. |
| Fleet positions show San Francisco | Restart `vehicle_service.py` — in-memory state from old run. |
| LiDAR shows "🔴 Disconnected" | Expected when robot is off. Start `lidar_service.py`, set robot IP in Connection Settings. |
| LiDAR Sim page shows error | Start `dummy_lidar_service.py` in a separate terminal. |
| Teleop page shows error | Start `teleop_service.py` in a separate terminal. |
| Teleop robot won't move after E-Stop | Click **🔄 Reset to Origin** to clear the emergency stop. |
| NiceGUI Viz page blank / error | Start `viz_app.py` then open `http://localhost:8008`. |
| "Port already in use" | Another process holds the port. Kill it: `python -c "import subprocess; subprocess.run(['taskkill','/F','/IM','python.exe'])"` (Windows) or `pkill python` (Linux/Mac). |
| Streamlit doesn't open browser | Navigate manually to `http://localhost:8501`. |
| Robot service shows "unconfigured" | No credentials set yet — `POST /robot/config` with `tenant`, `username`, `password`, `apikey`. |
| Robot service shows "error" | Check `GET /robot/status` for the error message. Verify credentials, tenant name, and network access to `*.cloud.innok-robotics.de`. |
| Robot position returns 503 "not yet available" | First poll hasn't succeeded yet — trigger one with `POST /robot/poll`. |