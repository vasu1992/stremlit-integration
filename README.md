# Operations Control Center

A microservices-based autonomous logistics platform with a **Streamlit** operations dashboard covering AV fleet telemetry (Magdeburg, Germany), live LiDAR visualization via ROS/rosbridge, a built-in LiDAR simulator, a teleoperation interface with fleet integration, MQTT-based traffic light monitoring, dual camera streaming (USB + WiFi IP camera), and a live robot telemetry bridge for Innok Robotics cloud-connected robots. No real robot required for most features.

---

## Project Structure

```
.
├── streamlit_app.py              # Streamlit operations dashboard       (Port 8501)
├── vehicle_service.py            # AV fleet telemetry service           (Port 8004)
├── lidar_service.py              # RViz LiDAR bridge service            (Port 8005)
├── dummy_lidar_service.py        # Dummy LiDAR simulator service        (Port 8006)
├── teleop_service.py             # Teleoperation interface service      (Port 8007)
├── mqtt_traffic_service.py       # MQTT traffic light service           (Port 8009)
├── camera_service.py             # USB + WiFi IP camera stream service  (Port 8010)
├── robot_telemetry_service.py    # Innok Robotics cloud telemetry       (Port 8083)
├── traffic_dashboard.py          # Standalone traffic light dashboard   (Port 8511)
├── tugger train.png              # Home page vehicle image
├── cargo.jpg                     # Home page vehicle image
├── delivery robot.jpg            # Home page vehicle image
├── requirements.txt              # Python dependencies
└── README.md                     # This file
```

---

## Port Reference

| Service | Port |
|---------|------|
| Streamlit Dashboard | 8501 |
| Vehicle Service | 8004 |
| LiDAR Bridge (real robot) | 8005 |
| LiDAR Simulator | 8006 |
| Teleop Service | 8007 |
| MQTT Traffic Light Service | 8009 |
| Camera Service (USB + IP) | 8010 |
| Robot Telemetry Service | 8083 |
| Traffic Light Dashboard (standalone) | 8511 |

> **Windows note:** Ports 8001, 8002, 8011, 8012 are reserved by Hyper-V (WinError 10013). Avoid these.

---

## Services Overview

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

**Default:** `ws://192.168.1.100:9090`, topic `/scan`. Auto-reconnects every 3 s.

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

### MQTT Traffic Light Service — Port 8009
Subscribes to a live MQTT broker (`service.ifak.eu:1883`, topic `iot_board/lsa/leitstelle`) and exposes real traffic-light data via REST. Subscribe-only — never publishes.

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Health check + MQTT connection status |
| `/traffic/lights` | GET | Current state of all known traffic lights |
| `/traffic/summary` | GET | Count by state (GREEN/YELLOW/RED), fault count |
| `/traffic/history` | GET | Recent state-change events `?limit=200` |
| `/traffic/raw` | GET | Last raw MQTT message (for debugging) |

**MQTT details:** Broker `service.ifak.eu:1883` · Topic `iot_board/lsa/leitstelle` · Auto-reconnects on disconnect.

### Camera Service — Port 8010
Streams both a **local USB camera** and a **WiFi IP camera** (OCC_cam1) as MJPEG over HTTP. The IP camera RTSP URL is probed automatically across 14 common path patterns — no manual configuration needed.

#### USB Camera endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Health check + USB camera status (resolution, FPS) |
| `/cameras` | GET | List all available USB camera indices (probes 0–7) |
| `/stream?index=N` | GET | Live MJPEG stream — embed with `<img src="...">` |
| `/frame` | GET | Single JPEG frame as base64 JSON |
| `/select` | POST | Switch active USB camera `{"index": N}` |

#### WiFi IP Camera endpoints (OCC_cam1 @ 192.168.178.177)

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/ip/health` | GET | Connection status, active RTSP URL, resolution, FPS |
| `/ip/stream` | GET | Live MJPEG stream — embed with `<img src="...">` |
| `/ip/frame` | GET | Single JPEG frame as base64 JSON |

**IP camera credentials:** user `OCC_cam1` · password `OCC@1234` · IP `192.168.178.177`

**Auto-probe RTSP paths** (tried in order, first success wins):
`/stream1`, `/stream`, `/live`, `/video1`, `/` (root), `/onvif1`, Hikvision `/Streaming/Channels/101`, Dahua `/cam/realmonitor?channel=1&subtype=0`, Reolink `/live/ch00_0`, port 8554 variants, and HTTP MJPEG fallbacks.

The active RTSP URL is reported in `/ip/health` → `active_url` and shown in the dashboard once connected.

### Robot Telemetry Service — Port 8083
Bridges an **Innok Robotics** cloud-connected robot to the local platform by polling the Innok REST API and caching the latest diagnostics and position data. Also accepts light-control commands forwarded to the robot's cloud API.

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Health check + connection state |
| `/robot/status` | GET | Connection state, last poll timestamp, any error |
| `/robot/diagnostics` | GET | Latest diagnostics (level, name, message, hardware_id, values) |
| `/robot/position` | GET | Latest pose — translation `{x,y,z}` + rotation quaternion `{x,y,z,w}` |
| `/robot/light` | GET | Current light state (`on` / `off`) and when it was last set |
| `/robot/light` | POST | Set light `{"state": "on"\|"off"}` — forwards to robot API |
| `/robot/poll` | POST | Trigger an immediate poll now |
| `/robot/config` | GET | Current config (password/apikey masked) |
| `/robot/config` | POST | Update credentials or poll interval at runtime |

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

---

## Dashboard Pages (Streamlit — Port 8501)

| Page | Description |
|------|-------------|
| 🏠 **Home** | Vehicle class showcase, service health chips, quick fleet stats |
| 🚗 **Fleet KPIs** | Simulate AV telemetry, KPI metrics, status donut, battery bars, speed trend, live Magdeburg position map, fleet table, register/deregister vehicles |
| 📡 **LiDAR** | Live LiDAR point cloud from rosbridge, scan statistics, history chart |
| 🤖 **LiDAR Sim** | Synthetic RViz-style point cloud: room boundary, range-coloured scan points, obstacle markers, live config, auto-refresh |
| 🕹️ **Teleop** | Fleet vehicle connection panel, D-pad controls, synthetic camera feeds (4× SVG), robot trail chart, command history, GPS sync to fleet map |
| 🚦 **Traffic Lights** | Live MQTT traffic light grid (GREEN/YELLOW/RED), intersection map (folium), state-change timeline chart, recent events table, link to standalone dashboard |
| 📷 **Camera** | Live USB camera MJPEG stream with index switcher + snapshots; Live WiFi IP camera (OCC_cam1) MJPEG stream with connection status + snapshots |
| 🦾 **Robot** | Light on/off toggle, live position (translation + quaternion), diagnostics table, credential config form, auto-refresh when connected |

---

## Quick Start

### 1. Install Dependencies

```bash
pip install -r requirements.txt
```

### 2. Start the Services

Open a separate terminal for each service:

```bash
# Terminal 1
python vehicle_service.py           # → http://localhost:8004

# Terminal 2  (optional — requires a ROS robot on the network)
python lidar_service.py             # → http://localhost:8005

# Terminal 3  (optional — no robot needed)
python dummy_lidar_service.py       # → http://localhost:8006

# Terminal 4  (optional)
python teleop_service.py            # → http://localhost:8007

# Terminal 5  (optional — needs MQTT network access to service.ifak.eu)
python mqtt_traffic_service.py      # → http://localhost:8009

# Terminal 6  (optional — USB camera or WiFi IP camera on same network)
python camera_service.py            # → http://localhost:8010

# Terminal 7  (optional — Innok Robotics cloud robot bridge)
INNOK_TENANT=myrobot INNOK_USER=user INNOK_PASSWORD=pass INNOK_APIKEY=XXXXX \
python robot_telemetry_service.py   # → http://localhost:8083

# Terminal 8  (optional — standalone traffic light dashboard)
python traffic_dashboard.py         # → http://localhost:8511

# Terminal 9  (Streamlit dashboard)
streamlit run streamlit_app.py      # → http://localhost:8501
```

> All services are optional — the dashboard degrades gracefully when a service is offline. `vehicle_service.py` is recommended as it powers the Home page stats and the Fleet KPIs page.

### 3. WiFi IP Camera Setup

The camera service auto-discovers the RTSP stream path on startup. Ensure:
1. Camera is powered on and connected to the same WiFi network (`192.168.178.x`)
2. Camera service is running: `python camera_service.py`
3. The terminal will print the working RTSP URL, e.g.:
   `[IP cam] Connected: rtsp://OCC_cam1:OCC%401234@192.168.178.177:554/live`

If none of the 14 candidate paths connect, the dashboard shows a warning. In that case, check your camera's RTSP path in its web interface and update `_IP_CAM_CANDIDATES` in [camera_service.py](camera_service.py).

### 4. Innok Robotics Setup (Real Robot Only)

```bash
# Option A — env vars at startup
INNOK_TENANT=myrobot INNOK_USER=user INNOK_PASSWORD=pass INNOK_APIKEY=XXXXX \
python robot_telemetry_service.py

# Option B — runtime config (start first, configure later)
python robot_telemetry_service.py

curl -X POST http://localhost:8083/robot/config \
  -H "Content-Type: application/json" \
  -d '{"tenant":"myrobot","username":"user","password":"pass","apikey":"XXXXX"}'
```

### 5. LiDAR Setup (Real Robot Only)

On the ROS robot:
```bash
# ROS 1
roslaunch rosbridge_server rosbridge_websocket.launch

# ROS 2
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Then open **📡 LiDAR** → **⚙️ Connection Settings** → enter robot IP → **🔌 Apply & Reconnect**.

> For LiDAR without a robot, use **🤖 LiDAR Sim** instead.

### 6. Teleop Fleet Integration

1. Start `vehicle_service.py` and `teleop_service.py`
2. Navigate to **🕹️ Teleop** in the Streamlit dashboard
3. Expand **🔗 Fleet Vehicle Connection** and select an available vehicle
4. Click **🎮 Connect & Take Control** — the robot is seeded at the vehicle's GPS position
5. Use the D-pad to drive; the vehicle's position on the Fleet Map updates in real time
6. Click **🔌 Disconnect** to return the vehicle to autonomous simulation

---

## Architecture

```
┌───────────────────────────────────────────────────────────────────────────────┐
│              STREAMLIT Operations Dashboard  (Port 8501)                      │
│  Home · Fleet KPIs · LiDAR · LiDAR Sim · Teleop · Traffic · Camera · Robot  │
└──┬────────┬────────┬──────────┬────────┬──────────┬────────────┬─────────────┘
   │        │        │          │        │          │            │
   ▼        ▼        ▼          ▼        ▼          ▼            ▼
┌──────┐ ┌──────┐ ┌──────┐ ┌──────┐ ┌────────┐ ┌────────┐ ┌────────┐
│Vehic.│ │LiDAR │ │Dummy │ │Teleop│ │ MQTT   │ │Camera  │ │ Robot  │
│ Svc  │ │Bridge│ │LiDAR │ │ Svc  │ │Traffic │ │  Svc   │ │Teleme. │
│:8004 │ │:8005 │ │:8006 │ │:8007 │ │ :8009  │ │ :8010  │ │ :8083  │
└──────┘ └──┬───┘ └──────┘ └──┬───┘ └───┬────┘ └───┬────┘ └───┬────┘
            │                  │         │           │           │
            │ WebSocket   GPS sync  MQTT broker  USB cam   Innok Cloud
            ▼                  ▼         ▼           ▼           ▼
       ┌─────────┐       ┌─────────┐ ┌──────────┐ ┌──────────┐ ┌──────────────┐
       │ROS Robot│       │ Vehicle │ │service.  │ │ WiFi IP  │ │*.cloud.innok │
       │rosbridge│       │ Service │ │ifak.eu   │ │ Camera   │ │-robotics.de  │
       │:9090    │       │ :8004   │ │:1883     │ │192.168.. │ │OCC_cam1      │
       └─────────┘       └─────────┘ └──────────┘ │.178.177  │ └──────────────┘
                                                   └──────────┘
```

---

## Example API Calls

```bash
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

# Traffic light summary
curl "http://localhost:8009/traffic/summary"

# Check camera service health (USB)
curl "http://localhost:8010/"

# Check WiFi IP camera connection + active RTSP URL
curl "http://localhost:8010/ip/health"

# Capture a single frame from the IP camera
curl "http://localhost:8010/ip/frame"

# Configure Innok robot credentials at runtime
curl -X POST "http://localhost:8083/robot/config" \
  -H "Content-Type: application/json" \
  -d '{"tenant":"myrobot","username":"user","password":"pass","apikey":"XXXXX"}'

# Get robot diagnostics
curl "http://localhost:8083/robot/diagnostics"

# Turn robot light on
curl -X POST "http://localhost:8083/robot/light" \
  -H "Content-Type: application/json" \
  -d '{"state": "on"}'
```

---

## Dependencies

| Package | Used by |
|---------|---------|
| `fastapi` | All services |
| `uvicorn` | All services |
| `pydantic` | All services |
| `requests` | streamlit_app.py |
| `httpx` | robot_telemetry_service.py |
| `websockets` | lidar_service.py |
| `paho-mqtt` | mqtt_traffic_service.py |
| `opencv-python` | camera_service.py (USB + RTSP streaming) |
| `streamlit` | streamlit_app.py |
| `pandas` | streamlit_app.py |
| `altair` | streamlit_app.py |

Install all at once:
```bash
pip install -r requirements.txt
```

Optional for interactive map on Traffic Lights page:
```bash
pip install folium streamlit-folium
```

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| Ports 8001/8002/8011/8012 blocked (WinError 10013) | Windows Hyper-V reserves these ports. The project uses 8004–8010/8083 to avoid them. |
| Fleet KPIs page shows error | Start `vehicle_service.py` in a separate terminal. |
| Fleet positions show wrong city | Restart `vehicle_service.py` — in-memory state from old run. |
| LiDAR shows "🔴 Disconnected" | Expected when no robot. Start `lidar_service.py`, set robot IP in Connection Settings. |
| LiDAR Sim page shows error | Start `dummy_lidar_service.py` in a separate terminal. |
| Teleop page shows error | Start `teleop_service.py` in a separate terminal. |
| Teleop robot won't move after E-Stop | Click **🔄 Reset to Origin** to clear the emergency stop. |
| Traffic Lights shows "MQTT Disconnected" | Start `mqtt_traffic_service.py`. Check outbound TCP access to `service.ifak.eu:1883`. |
| Camera page shows "unavailable" | Start `camera_service.py`. For USB: plug in a USB camera. |
| IP camera stream unavailable | Verify camera is on `192.168.178.x` WiFi. Check the terminal for `[IP cam]` probe output. If all 14 RTSP paths fail, look up your camera model's RTSP path and add it to `_IP_CAM_CANDIDATES` in `camera_service.py`. |
| `taskkill` fails in Git Bash | Use: `python -c "import subprocess; subprocess.run(['taskkill', '/PID', '<pid>', '/F'])"` |
| "Port already in use" | Kill the occupying process (see above). |
| Streamlit doesn't open browser | Navigate manually to `http://localhost:8501`. |
| Robot service shows "unconfigured" | Set credentials via `POST /robot/config` or env vars. |
| Robot position returns 503 "not yet available" | Trigger first poll: `curl -X POST http://localhost:8083/robot/poll`. |
