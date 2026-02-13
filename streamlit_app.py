import streamlit as st
import requests
import pandas as pd
import altair as alt
import time
from datetime import datetime

# Page configuration
st.set_page_config(
    page_title="Microservices Dashboard",
    page_icon="📊",
    layout="wide"
)

# Service URLs
USER_SERVICE = "http://localhost:8001"
ANALYTICS_SERVICE = "http://localhost:8002"
LOCATION_SERVICE = "http://localhost:8003"
VEHICLE_SERVICE = "http://localhost:8004"
LIDAR_SERVICE   = "http://localhost:8005"

# Title
st.title("📊 Microservices Dashboard")
st.markdown("---")

# Check if services are running
def check_services():
    try:
        requests.get(USER_SERVICE, timeout=1)
        requests.get(ANALYTICS_SERVICE, timeout=1)
        return True
    except Exception:
        return False

if not check_services():
    st.error("⚠️ ERROR: Services are not running! Please start them first.")
    st.info("""
    Start the services in separate terminals:

    Terminal 1: `python user_service.py`
    Terminal 2: `python analytics_service.py`
    Terminal 3: `python location_service.py`
    Terminal 4: `streamlit run streamlit_app.py`
    """)
    st.stop()

# Sidebar navigation
st.sidebar.title("Navigation")
page = st.sidebar.radio("Select Page", ["🏠 Home", "👥 Users", "📈 Analytics", "📊 Reports", "🗺️ Map", "🚗 Fleet KPIs", "📡 LiDAR"])

# ==================== HOME PAGE ====================
if page == "🏠 Home":
    st.header("Welcome to Microservices Dashboard")
    st.write("This dashboard connects to multiple microservices to display data.")
    
    # Service status
    st.subheader("Service Status")
    col1, col2, col3 = st.columns(3)
    
    try:
        with col1:
            user_resp = requests.get(f"{USER_SERVICE}/")
            st.success("✅ User Service")
            st.caption("Running on port 8001")
        
        with col2:
            analytics_resp = requests.get(f"{ANALYTICS_SERVICE}/")
            st.success("✅ Analytics Service")
            st.caption("Running on port 8002")
        
        with col3:
            st.success("✅ Streamlit Frontend")
            st.caption("Running on port 8501")
    except:
        st.error("Could not connect to services")
    
    # Quick stats
    st.subheader("Quick Statistics")
    col1, col2, col3 = st.columns(3)
    
    try:
        # Get user count
        users_resp = requests.get(f"{USER_SERVICE}/users")
        user_count = len(users_resp.json()["users"])
        
        # Get analytics summary
        analytics_resp = requests.get(f"{ANALYTICS_SERVICE}/summary")
        summary = analytics_resp.json()
        
        with col1:
            st.metric("Total Users", user_count)
        
        with col2:
            st.metric("Total Events", summary["total_events"])
        
        with col3:
            st.metric("Average Value", f"{summary['average_value']:.2f}")
    except Exception as e:
        st.error(f"Error loading statistics: {e}")

# ==================== USERS PAGE ====================
elif page == "👥 Users":
    st.header("User Management")
    
    col1, col2 = st.columns([2, 1])
    
    with col1:
        st.subheader("Create New User")
        with st.form("user_form"):
            name = st.text_input("Full Name", placeholder="John Doe")
            email = st.text_input("Email", placeholder="john@example.com")
            submit = st.form_submit_button("➕ Add User")
            
            if submit:
                if name and email:
                    try:
                        response = requests.post(
                            f"{USER_SERVICE}/users",
                            json={"name": name, "email": email}
                        )
                        if response.status_code == 200:
                            st.success(f"✅ User '{name}' added successfully!")
                            st.rerun()
                    except Exception as e:
                        st.error(f"Error: {e}")
                else:
                    st.warning("Please fill in all fields")
    
    with col2:
        st.subheader("Actions")
        if st.button("🔄 Refresh"):
            st.rerun()
    
    # Display all users
    st.subheader("All Users")
    try:
        response = requests.get(f"{USER_SERVICE}/users")
        users = response.json()["users"]
        
        if users:
            df = pd.DataFrame(users)
            st.dataframe(df, use_container_width=True)
        else:
            st.info("No users found. Create one above!")
    except Exception as e:
        st.error(f"Error loading users: {e}")

# ==================== ANALYTICS PAGE ====================
elif page == "📈 Analytics":
    st.header("Event Analytics")
    
    col1, col2 = st.columns([3, 1])
    
    with col1:
        st.subheader("Log New Event")
        with st.form("event_form"):
            event_name = st.selectbox(
                "Event Type",
                ["page_view", "button_click", "form_submit", "purchase", "login", "custom"]
            )
            event_value = st.number_input("Event Value", min_value=0.0, step=0.1)
            submit = st.form_submit_button("📝 Log Event")
            
            if submit:
                try:
                    response = requests.post(
                        f"{ANALYTICS_SERVICE}/log",
                        params={"event_name": event_name, "value": event_value}
                    )
                    if response.status_code == 200:
                        st.success("✅ Event logged successfully!")
                        st.rerun()
                except Exception as e:
                    st.error(f"Error: {e}")
    
    with col2:
        st.subheader("Actions")
        if st.button("🔄 Refresh"):
            st.rerun()
    
    # Display analytics summary
    st.subheader("Summary Statistics")
    try:
        response = requests.get(f"{ANALYTICS_SERVICE}/summary")
        summary = response.json()
        
        col1, col2, col3, col4 = st.columns(4)
        with col1:
            st.metric("Total Events", summary["total_events"])
        with col2:
            st.metric("Average", f"{summary['average_value']:.2f}")
        with col3:
            st.metric("Min", f"{summary['min_value']:.2f}")
        with col4:
            st.metric("Max", f"{summary['max_value']:.2f}")
    except Exception as e:
        st.error(f"Error loading summary: {e}")
    
    # Display all events
    st.subheader("All Events")
    try:
        response = requests.get(f"{ANALYTICS_SERVICE}/events")
        events = response.json()["events"]
        
        if events:
            df = pd.DataFrame(events)
            st.dataframe(df, use_container_width=True)
            
            # Chart
            st.subheader("Value Trend")
            chart_df = df[["timestamp", "value"]].copy()
            chart_df["timestamp"] = pd.to_datetime(chart_df["timestamp"])
            st.line_chart(chart_df.set_index("timestamp"))
        else:
            st.info("No events logged yet. Create one above!")
    except Exception as e:
        st.error(f"Error loading events: {e}")

# ==================== REPORTS PAGE ====================
elif page == "📊 Reports":
    st.header("Reports & Insights")
    
    st.subheader("User Report")
    try:
        response = requests.get(f"{USER_SERVICE}/users")
        users = response.json()["users"]
        
        col1, col2 = st.columns(2)
        with col1:
            st.metric("Total Users", len(users))
        with col2:
            st.metric("New Users Today", "5")  # Placeholder
        
        if users:
            st.write("User List:")
            for user in users:
                st.write(f"• {user['name']} ({user['email']})")
    except Exception as e:
        st.error(f"Error: {e}")
    
    st.markdown("---")
    
    st.subheader("Analytics Report")
    try:
        response = requests.get(f"{ANALYTICS_SERVICE}/summary")
        summary = response.json()
        
        st.write(f"**Total Events:** {summary['total_events']}")
        st.write(f"**Average Event Value:** {summary['average_value']:.2f}")
        st.write(f"**Range:** {summary['min_value']:.2f} - {summary['max_value']:.2f}")
    except Exception as e:
        st.error(f"Error: {e}")

# ==================== MAP PAGE ====================
elif page == "🗺️ Map":
    st.header("Location Map")

    CATEGORY_COLORS = {
        "Restaurant": "#FF4B4B",
        "Hotel": "#4B6BFF",
        "Office": "#4BC84B",
        "Park": "#00B400",
        "Other": "#B4B4B4",
    }

    col1, col2 = st.columns([2, 1])

    with col1:
        st.subheader("Add New Location")
        with st.form("location_form"):
            loc_name = st.text_input("Location Name", placeholder="Eiffel Tower")
            loc_lat = st.number_input("Latitude", value=48.8584, format="%.6f", step=0.0001)
            loc_lon = st.number_input("Longitude", value=2.2945, format="%.6f", step=0.0001)
            loc_category = st.selectbox("Category", list(CATEGORY_COLORS.keys()))
            submit = st.form_submit_button("📍 Add Location")

            if submit:
                if loc_name:
                    try:
                        response = requests.post(
                            f"{LOCATION_SERVICE}/locations",
                            json={
                                "name": loc_name,
                                "latitude": loc_lat,
                                "longitude": loc_lon,
                                "category": loc_category,
                            },
                        )
                        if response.status_code == 200:
                            st.success(f"✅ '{loc_name}' added!")
                            st.rerun()
                    except Exception as e:
                        st.error(f"Error: {e}")
                else:
                    st.warning("Please enter a location name")

    with col2:
        st.subheader("Actions")
        if st.button("🔄 Refresh"):
            st.rerun()

    # Fetch and display locations
    try:
        response = requests.get(f"{LOCATION_SERVICE}/locations", timeout=2)
        locations = response.json()["locations"]

        if locations:
            df_map = pd.DataFrame(locations)
            df_map["color"] = df_map["category"].map(CATEGORY_COLORS).fillna("#B4B4B4")

            # Legend
            st.subheader("Map")
            legend_cols = st.columns(len(CATEGORY_COLORS))
            for i, (cat, color) in enumerate(CATEGORY_COLORS.items()):
                with legend_cols[i]:
                    st.markdown(
                        f'<span style="color:{color}">■</span> {cat}',
                        unsafe_allow_html=True,
                    )

            st.map(df_map, latitude="latitude", longitude="longitude", color="color", size=80)

            # Locations table with delete buttons
            st.subheader(f"All Locations ({len(locations)})")
            for loc in locations:
                c1, c2, c3 = st.columns([3, 2, 1])
                with c1:
                    color = CATEGORY_COLORS.get(loc["category"], "#B4B4B4")
                    st.markdown(
                        f'<span style="color:{color}">■</span> **{loc["name"]}**',
                        unsafe_allow_html=True,
                    )
                with c2:
                    st.caption(f'{loc["category"]} · {loc["latitude"]:.4f}, {loc["longitude"]:.4f}')
                with c3:
                    if st.button("🗑️", key=f"del_{loc['id']}"):
                        try:
                            requests.delete(f"{LOCATION_SERVICE}/locations/{loc['id']}")
                            st.rerun()
                        except Exception as e:
                            st.error(f"Error: {e}")
        else:
            st.info("No locations yet. Add one using the form above!")
            st.map(pd.DataFrame({"latitude": [], "longitude": []}))

    except Exception as e:
        st.error(f"⚠️ Location service unavailable. Start it with: `python location_service.py`")

# ==================== FLEET KPIs PAGE ====================
elif page == "🚗 Fleet KPIs":
    st.header("Autonomous Vehicle Fleet — KPI Dashboard")

    STATUS_COLORS = {
        "Active":   "#00B400",
        "Idle":     "#FFA500",
        "Charging": "#4B6BFF",
        "Error":    "#FF4B4B",
    }
    HEALTH_ICONS = {"Good": "🟢", "Warning": "🟡", "Critical": "🔴"}

    try:
        # ── Simulate button ───────────────────────────────────────────────────
        if st.button("▶ Simulate New Readings"):
            requests.post(f"{VEHICLE_SERVICE}/fleet/simulate", timeout=2)
            st.rerun()

        # ── KPI metrics ───────────────────────────────────────────────────────
        kpi = requests.get(f"{VEHICLE_SERVICE}/fleet/kpis", timeout=2).json()

        c1, c2, c3, c4, c5 = st.columns(5)
        with c1:
            st.metric("Total Vehicles", kpi["total_vehicles"])
        with c2:
            st.metric(
                "Status Breakdown",
                f"🟢{kpi['active']} 🟡{kpi['idle']} 🔵{kpi['charging']} 🔴{kpi['error']}",
            )
        with c3:
            st.metric("Avg Speed", f"{kpi['avg_speed_kmh']} km/h")
        with c4:
            st.metric("Avg Battery", f"{kpi['avg_battery_pct']} %")
        with c5:
            icon = HEALTH_ICONS.get(kpi["fleet_health"], "")
            st.metric("Fleet Health", f"{icon} {kpi['fleet_health']}")

        st.markdown("---")

        # ── Vehicles data ─────────────────────────────────────────────────────
        vehicles = requests.get(f"{VEHICLE_SERVICE}/fleet/vehicles", timeout=2).json()["vehicles"]
        df_v = pd.DataFrame(vehicles)

        # ── Row: Pie chart + Battery bar ──────────────────────────────────────
        col_pie, col_bat = st.columns(2)

        with col_pie:
            st.subheader("Fleet Status")
            status_counts = df_v["status"].value_counts().reset_index()
            status_counts.columns = ["status", "count"]
            pie = (
                alt.Chart(status_counts)
                .mark_arc(innerRadius=50)
                .encode(
                    theta=alt.Theta("count:Q"),
                    color=alt.Color(
                        "status:N",
                        scale=alt.Scale(
                            domain=list(STATUS_COLORS.keys()),
                            range=list(STATUS_COLORS.values()),
                        ),
                        legend=alt.Legend(title="Status"),
                    ),
                    tooltip=["status:N", "count:Q"],
                )
                .properties(width=280, height=280)
            )
            st.altair_chart(pie, use_container_width=True)

        with col_bat:
            st.subheader("Battery Levels (%)")
            bat_df = df_v[["id", "battery"]].set_index("id")
            st.bar_chart(bat_df, color="#4B6BFF", use_container_width=True)

        # ── Speed trend ───────────────────────────────────────────────────────
        st.subheader("Speed Trend (km/h)")
        history = requests.get(f"{VEHICLE_SERVICE}/fleet/history", timeout=2).json()["history"]
        if history:
            hist_df = pd.DataFrame(history)
            hist_df["timestamp"] = pd.to_datetime(hist_df["timestamp"])
            speed_pivot = hist_df.pivot_table(
                index="timestamp", columns="vehicle_id", values="speed", aggfunc="mean"
            )
            st.line_chart(speed_pivot, use_container_width=True)
        else:
            st.info("No history yet — click **▶ Simulate New Readings** to generate data.")

        # ── Vehicle positions map ─────────────────────────────────────────────
        st.subheader("Live Vehicle Positions")
        df_map = df_v[["id", "latitude", "longitude", "status"]].copy()
        df_map["color"] = df_map["status"].map(STATUS_COLORS).fillna("#B4B4B4")
        st.map(df_map, latitude="latitude", longitude="longitude", color="color", size=120)

        # ── Fleet table ───────────────────────────────────────────────────────
        st.subheader("Fleet Telemetry Table")
        display_df = df_v[["id", "name", "status", "speed", "battery", "cpu_usage", "sensor_health", "distance_km", "last_updated"]].copy()
        display_df.columns = ["ID", "Name", "Status", "Speed (km/h)", "Battery (%)", "CPU (%)", "Sensor Health", "Distance (km)", "Last Updated"]
        st.dataframe(display_df, use_container_width=True, hide_index=True)

    except Exception as e:
        st.error("⚠️ Vehicle service unavailable. Start it with: `python vehicle_service.py`")

# ==================== LIDAR PAGE ====================
elif page == "📡 LiDAR":
    st.header("📡 LiDAR — RViz Bridge")

    # ── Connection settings expander ─────────────────────────────────────────
    with st.expander("⚙️ Connection Settings", expanded=False):
        with st.form("lidar_config_form"):
            cfg_ip    = st.text_input("Robot IP", value="192.168.1.100")
            cfg_port  = st.number_input("rosbridge Port", value=9090, step=1)
            cfg_topic = st.text_input("ROS Topic", value="/scan")
            if st.form_submit_button("🔌 Apply & Reconnect"):
                try:
                    requests.post(
                        f"{LIDAR_SERVICE}/config",
                        json={"robot_ip": cfg_ip, "robot_port": cfg_port, "ros_topic": cfg_topic},
                        timeout=3,
                    )
                    st.success("Reconnecting…")
                    st.rerun()
                except Exception as e:
                    st.error(f"Error: {e}")

    try:
        # ── Status row ───────────────────────────────────────────────────────
        status = requests.get(f"{LIDAR_SERVICE}/scan/status", timeout=2).json()

        conn_label = "🟢 Connected" if status["connected"] else "🔴 Disconnected"
        c1, c2, c3, c4 = st.columns(4)
        with c1:
            st.metric("Connection", conn_label)
        with c2:
            st.metric("Scan Rate", f"{status['scan_rate_hz']} Hz")
        with c3:
            scan = requests.get(f"{LIDAR_SERVICE}/scan/latest", timeout=2).json()
            pt_count = scan["point_count"] if scan else 0
            st.metric("Points / Scan", pt_count)
        with c4:
            if scan:
                rng = f"{scan['range_min_measured']} – {scan['range_max_measured']} m"
            else:
                rng = "—"
            st.metric("Range (min – max)", rng)

        st.markdown("---")

        # ── LiDAR scatter plot ────────────────────────────────────────────────
        col_chart, col_info = st.columns([3, 1])

        with col_chart:
            if scan and scan.get("points"):
                pts_df = pd.DataFrame(scan["points"])
                rate_str = f"{status['scan_rate_hz']} Hz" if status["connected"] else "—"
                chart_title = f"Latest Scan — {pt_count} points  ·  {rate_str}"

                scatter = (
                    alt.Chart(pts_df, title=chart_title)
                    .mark_circle(size=6, opacity=0.85)
                    .encode(
                        x=alt.X("x:Q", title="X (m)", scale=alt.Scale(zero=True)),
                        y=alt.Y("y:Q", title="Y (m)", scale=alt.Scale(zero=True)),
                        color=alt.Color(
                            "range:Q",
                            title="Range (m)",
                            scale=alt.Scale(scheme="redyellowgreen", reverse=True),
                        ),
                        tooltip=[
                            alt.Tooltip("x:Q", format=".3f"),
                            alt.Tooltip("y:Q", format=".3f"),
                            alt.Tooltip("range:Q", title="Range (m)", format=".3f"),
                            alt.Tooltip("angle_deg:Q", title="Angle (°)", format=".1f"),
                        ],
                    )
                    .properties(width="container", height=480)
                )

                # Robot origin marker
                origin_df = pd.DataFrame({"x": [0], "y": [0]})
                origin = (
                    alt.Chart(origin_df)
                    .mark_point(size=120, shape="triangle-up", color="#00FFFF", filled=True)
                    .encode(x="x:Q", y="y:Q")
                )

                st.altair_chart(scatter + origin, use_container_width=True)
            else:
                st.info(
                    "No scan data yet.\n\n"
                    "1. Ensure the robot is running `rosbridge_server`\n"
                    "2. Set the Robot IP in **⚙️ Connection Settings** above\n"
                    "3. Click **🔌 Apply & Reconnect**"
                )

        with col_info:
            st.subheader("Last Scan")
            if scan:
                st.write(f"**Time:** {scan['timestamp']}")
                st.write(f"**Points:** {scan['point_count']}")
                st.write(f"**Min range:** {scan['range_min_measured']} m")
                st.write(f"**Max range:** {scan['range_max_measured']} m")
                st.write(f"**Sensor min:** {scan['sensor_range_min']} m")
                st.write(f"**Sensor max:** {scan['sensor_range_max']} m")
                st.write(f"**Total scans:** {status['scan_count']}")
            else:
                st.write("Waiting for first scan…")

        # ── Scan history chart ────────────────────────────────────────────────
        st.subheader("Scan History — Range Statistics")
        hist = requests.get(f"{LIDAR_SERVICE}/scan/history", timeout=2).json()["history"]
        if hist:
            hist_df = pd.DataFrame(hist)
            hist_df["timestamp"] = pd.to_datetime(hist_df["timestamp"])
            st.line_chart(
                hist_df.set_index("timestamp")[["min_range", "mean_range", "max_range"]],
                use_container_width=True,
            )
        else:
            st.info("History will appear after the first scan is received.")

        # ── Auto-refresh ──────────────────────────────────────────────────────
        st.markdown("---")
        auto_refresh = st.toggle("Auto-refresh every 3 s", value=False)
        if auto_refresh:
            time.sleep(3)
            st.rerun()

    except Exception:
        st.error(
            "⚠️ LiDAR service unavailable.  "
            "Start it with: `python lidar_service.py`"
        )

st.markdown("---")
st.caption("Built with Streamlit + Microservices Architecture")
