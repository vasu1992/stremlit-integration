import streamlit as st
import requests
import pandas as pd
import altair as alt
import math
import time
from datetime import datetime
import paho.mqtt.client as mqtt
import ssl

# ─── Page config ──────────────────────────────────────────────────────────────
st.set_page_config(
    page_title="Operations Control Center",
    page_icon="🎯",
    layout="wide",
    initial_sidebar_state="expanded",
)

# ─── Service URLs ──────────────────────────────────────────────────────────────
VEHICLE_SERVICE     = "http://localhost:8004"
LIDAR_SERVICE       = "http://localhost:8005"
DUMMY_LIDAR_SERVICE = "http://localhost:8006"
TELEOP_SERVICE      = "http://localhost:8007"
TRAFFIC_SERVICE     = "http://localhost:8009"
CAMERA_SERVICE      = "http://localhost:8010"
ROBOT_SERVICE       = "http://localhost:8083"

# ─── MQTT HiveMQ Config ───────────────────────────────────────────────────────
MQTT_BROKER   = "ba941e31dbdd499594f2771c07004e2d.s1.eu.hivemq.cloud"
MQTT_PORT     = 8883
MQTT_USER     = "occ_MQTT"
MQTT_PASS     = "OCc@1234"
MQTT_TOPIC    = "OCC/rasp_disp"
MQTT_DEFAULT  = "OCC is online"


def mqtt_publish(message: str) -> tuple[bool, str]:
    """Publish a single message to MQTT broker with TLS. Returns (success, info)."""
    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        client.username_pw_set(MQTT_USER, MQTT_PASS)
        client.tls_set(cert_reqs=ssl.CERT_REQUIRED, tls_version=ssl.PROTOCOL_TLS_CLIENT)
        client.connect(MQTT_BROKER, MQTT_PORT, keepalive=10)
        result = client.publish(MQTT_TOPIC, message, qos=1)
        result.wait_for_publish(timeout=5)
        client.disconnect()
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            return True, f"Published to `{MQTT_TOPIC}`"
        return False, f"Publish error code: {result.rc}"
    except Exception as exc:
        return False, str(exc)


# Magdeburg fleet base coordinates (mirrors vehicle_service.py)
_BASE_LAT = 52.1205
_BASE_LON = 11.6276

# ─── Theme initialisation ─────────────────────────────────────────────────────
if "theme" not in st.session_state:
    st.session_state["theme"] = "dark"

_dark = st.session_state["theme"] == "dark"

# ─── Structural CSS (theme-independent) ───────────────────────────────────────
st.markdown("""
<style>
/* ── Radio dot hidden — nav looks like a menu ── */
section[data-testid="stSidebar"] [data-testid="stRadio"] [data-baseweb="radio"] > div:first-child {
    width: 0 !important; height: 0 !important;
    overflow: hidden !important; margin: 0 !important; padding: 0 !important;
}
/* ── Radio labels ── */
section[data-testid="stSidebar"] label {
    font-size: 1.08rem !important;
    font-weight: 500 !important;
    padding: 0.42rem 0.85rem !important;
    border-radius: 8px !important;
    display: block !important;
    transition: background 0.15s, color 0.15s !important;
    cursor: pointer !important;
    margin-bottom: 3px !important;
}
.sidebar-divider { border: none; margin: 6px 0.5rem; }

/* ── Main content ── */
.block-container {
    padding-top: 1rem !important;
    padding-bottom: 2rem !important;
    max-width: 1440px !important;
}

/* ── OCC top banner ── */
.occ-banner {
    border-left: 4px solid #388bfd;
    border-radius: 0 12px 12px 0;
    padding: 1rem 1.5rem;
    margin-bottom: 1.25rem;
    display: flex;
    align-items: center;
    gap: 1rem;
}
.occ-banner h2 { margin: 0 !important; font-size: 1.35rem !important; font-weight: 700 !important; }
.occ-banner p  { margin: 0 !important; font-size: 0.82rem !important; }

/* ── Vehicle image card ── */
.vcard {
    border-radius: 14px;
    overflow: hidden;
    border: 1px solid;
    transition: transform 0.2s, box-shadow 0.2s;
}
.vcard:hover { transform: translateY(-4px); box-shadow: 0 12px 30px rgba(0,0,0,0.45); }
.vcard-label {
    padding: 0.75rem 1rem 0.9rem 1rem;
    text-align: center;
}
.vcard-name { font-size: 1.0rem; font-weight: 700; margin: 0 0 0.2rem 0; }
.vcard-desc { font-size: 0.8rem; line-height: 1.5; margin: 0; opacity: 0.78; }

/* ── Service status chips ── */
.svc-chip {
    display: inline-flex; align-items: center; gap: 6px;
    padding: 5px 14px; border-radius: 20px;
    font-size: 0.82rem; font-weight: 500; margin-bottom: 4px;
}
.svc-ok  { background: rgba(63,185,80,0.12); border: 1px solid rgba(63,185,80,0.3); color: #3fb950; }
.svc-err { background: rgba(248,81,73,0.12); border: 1px solid rgba(248,81,73,0.3); color: #f85149; }

/* ── Metric tweaks ── */
[data-testid="stMetricValue"] { font-size: 1.45rem !important; font-weight: 700 !important; }
[data-testid="stMetricLabel"] { font-size: 0.82rem !important; }
</style>
""", unsafe_allow_html=True)

# ─── Theme-specific CSS injection ─────────────────────────────────────────────
if _dark:
    st.markdown("""
    <style>
    .stApp, [data-testid="stAppViewContainer"] { background-color: #0d1117 !important; }
    [data-testid="stHeader"] { background-color: #0d1117 !important; }
    h1,h2,h3,h4,h5,h6 { color: #f0f6fc !important; }
    p, li, div.stMarkdown { color: #c9d1d9 !important; }
    .occ-banner {
        background: linear-gradient(135deg, #090e18 0%, #0e1929 50%, #090e18 100%);
        border-color: #1a2d44;
    }
    .occ-banner h2 { color: #f0f6fc !important; }
    .occ-banner p  { color: #6a8099 !important; }
    /* Sidebar */
    section[data-testid="stSidebar"] {
        background: linear-gradient(180deg, #05080f 0%, #0a1020 55%, #05080f 100%) !important;
        border-right: 1px solid #16243a !important;
    }
    section[data-testid="stSidebar"] * { color: #a8bacf !important; }
    section[data-testid="stSidebar"] label { color: #a8bacf !important; }
    section[data-testid="stSidebar"] label:hover {
        background: rgba(56,139,253,0.12) !important; color: #79c0ff !important;
    }
    section[data-testid="stSidebar"] [aria-checked="true"] + div label {
        background: rgba(56,139,253,0.18) !important;
        color: #79c0ff !important; font-weight: 600 !important;
    }
    .sidebar-divider { border-top: 1px solid #16243a; }
    /* vehicle card */
    .vcard { border-color: #21262d; background: #161b22; }
    .vcard-name { color: #f0f6fc; }
    .vcard-desc { color: #8b949e; }
    [data-testid="stMetricLabel"] { color: #6a8099 !important; }
    </style>
    """, unsafe_allow_html=True)
else:
    st.markdown("""
    <style>
    .stApp, [data-testid="stAppViewContainer"] { background-color: #f5f7fa !important; }
    [data-testid="stHeader"] { background-color: #f5f7fa !important; }
    h1,h2,h3,h4,h5,h6 { color: #0d1117 !important; }
    p, li, div.stMarkdown { color: #24292f !important; }
    .occ-banner {
        background: linear-gradient(135deg, #ddeeff 0%, #eaf2ff 50%, #ddeeff 100%);
        border-color: #b0ccee;
    }
    .occ-banner h2 { color: #0d1117 !important; }
    .occ-banner p  { color: #57606a !important; }
    /* Sidebar (stays dark — brand identity) */
    section[data-testid="stSidebar"] {
        background: linear-gradient(180deg, #0d1a2e 0%, #162540 55%, #0d1a2e 100%) !important;
        border-right: 1px solid #1e3050 !important;
    }
    section[data-testid="stSidebar"] * { color: #b8cce4 !important; }
    section[data-testid="stSidebar"] label { color: #b8cce4 !important; }
    section[data-testid="stSidebar"] label:hover {
        background: rgba(56,139,253,0.15) !important; color: #79c0ff !important;
    }
    section[data-testid="stSidebar"] [aria-checked="true"] + div label {
        background: rgba(56,139,253,0.22) !important;
        color: #79c0ff !important; font-weight: 600 !important;
    }
    .sidebar-divider { border-top: 1px solid #1e3050; }
    /* vehicle card */
    .vcard { border-color: #d0dbe8; background: #ffffff; box-shadow: 0 2px 8px rgba(0,0,0,0.08); }
    .vcard-name { color: #0d1117; }
    .vcard-desc { color: #57606a; }
    [data-testid="stMetricLabel"] { color: #57606a !important; }
    /* dataframe / tables */
    [data-testid="stDataFrame"] { background: #ffffff !important; }
    </style>
    """, unsafe_allow_html=True)


# ─── Cached API helper ─────────────────────────────────────────────────────────
@st.cache_data(ttl=3)
def api_get(url: str) -> dict | None:
    """GET with a 3-second cache to minimise round-trips and reduce flicker."""
    try:
        r = requests.get(url, timeout=2)
        r.raise_for_status()
        return r.json()
    except Exception:
        return None


def api_post(url: str, **kwargs):
    """Non-cached POST; clears read-cache after mutation."""
    r = requests.post(url, timeout=3, **kwargs)
    api_get.clear()          # invalidate stale reads after any write
    return r


# ─── Sidebar navigation ────────────────────────────────────────────────────────
with st.sidebar:
    # ── Brand header ──
    st.markdown("""
    <div style="text-align:center; padding:1.1rem 0 1.2rem 0;">
        <div style="font-size:2.8rem; margin-bottom:4px; line-height:1;">🎯</div>
        <div style="font-size:1.05rem; font-weight:800; color:#f0f6fc; letter-spacing:0.4px;">Operations</div>
        <div style="font-size:1.05rem; font-weight:800; color:#388bfd; letter-spacing:0.4px; margin-bottom:10px;">Control Center</div>
        <div style="display:inline-flex; align-items:center; gap:6px;
                    background:rgba(63,185,80,0.12); padding:3px 14px;
                    border-radius:20px; border:1px solid rgba(63,185,80,0.28);">
            <span style="width:7px;height:7px;border-radius:50%;background:#3fb950;
                         display:inline-block;animation:blink 2s infinite;"></span>
            <span style="font-size:0.66rem; color:#3fb950; font-weight:700; letter-spacing:1.2px;">ONLINE</span>
        </div>
        <style>@keyframes blink{0%,100%{opacity:1}50%{opacity:.35}}</style>
    </div>
    """, unsafe_allow_html=True)

    st.markdown('<hr class="sidebar-divider">', unsafe_allow_html=True)

    # ── Single flat navigation ──
    _all_pages = [
        "🏠 Home",
        "🚗 Fleet KPIs",
        "📡 LiDAR",
        "🤖 LiDAR Sim",
        "🕹️ Teleop",
        "🚦 Traffic Lights",
        "📷 Camera",
        "🦾 Robot",
    ]
    page = st.radio("nav", _all_pages, label_visibility="collapsed", key="radio_nav")

    # ── Theme toggle ──
    st.markdown('<hr class="sidebar-divider">', unsafe_allow_html=True)
    _theme_label = "☀️  Light mode" if _dark else "🌙  Dark mode"
    if st.button(_theme_label, width='stretch', key="theme_btn"):
        st.session_state["theme"] = "light" if _dark else "dark"
        st.rerun()

    # ── Footer ──
    st.markdown(f"""
    <div style="padding:0.7rem 0.8rem 0 0.8rem; font-size:0.74rem; color:#3d5470; line-height:1.8;">
        📍 Magdeburg, DE<br>
        🕐 {datetime.now().strftime("%H:%M:%S")}<br>
        🚘 Fleet · 5 AVs
    </div>
    """, unsafe_allow_html=True)


# ─── OCC banner (top of every page) ──────────────────────────────────────────
_PAGE_ICONS = {
    "🏠 Home":      ("🏠", "Home"),
    "🚗 Fleet KPIs":("🚗", "Autonomous Vehicle Fleet — Magdeburg"),
    "📡 LiDAR":     ("📡", "LiDAR — RViz Bridge"),
    "🤖 LiDAR Sim": ("🤖", "LiDAR Simulator — Synthetic Scan"),
    "🕹️ Teleop":    ("🕹️", "Teleoperation Interface"),
    "🚦 Traffic Lights": ("🚦", "Traffic Light Monitor — MQTT"),
    "🦾 Robot":          ("🦾", "Robot Control & Telemetry — Innok Robotics"),
}
_icon, _title = _PAGE_ICONS.get(page, ("🎯", page))

st.markdown(f"""
<div class="occ-banner">
    <div style="font-size:2rem; line-height:1;">{_icon}</div>
    <div>
        <h2>{_title}</h2>
        <p>Operations Control Center &nbsp;·&nbsp; Magdeburg Autonomous Logistics Hub</p>
    </div>
</div>
""", unsafe_allow_html=True)


# ==================== HOME PAGE ====================
if page == "🏠 Home":

    # ── Fleet vehicle showcase ────────────────────────────────────────────────
    st.markdown("#### Managed Vehicle Classes")

    _border_dark  = "#21262d"
    _border_light = "#d0dbe8"
    _card_border  = _border_dark if _dark else _border_light

    c1, c2, c3 = st.columns(3, gap="medium")

    with c1:
        st.markdown(
            f'<div class="vcard" style="border-color:{_card_border};">',
            unsafe_allow_html=True,
        )
        st.image("tugger train.png", width='stretch')
        st.markdown("""
        <div class="vcard-label">
            <div class="vcard-name">Tugger Train</div>
            <div class="vcard-desc">Intralogistics train for high-volume goods movement along fixed facility routes.</div>
        </div></div>
        """, unsafe_allow_html=True)

    with c2:
        st.markdown(
            f'<div class="vcard" style="border-color:{_card_border};">',
            unsafe_allow_html=True,
        )
        st.image("cargo.jpg", width='stretch')
        st.markdown("""
        <div class="vcard-label">
            <div class="vcard-name">Cargo Bike</div>
            <div class="vcard-desc">Electric last-mile cargo bicycle for zero-emission urban and pedestrian-zone deliveries.</div>
        </div></div>
        """, unsafe_allow_html=True)

    with c3:
        st.markdown(
            f'<div class="vcard" style="border-color:{_card_border};">',
            unsafe_allow_html=True,
        )
        st.image("delivery robot.jpg", width='stretch')
        st.markdown("""
        <div class="vcard-label">
            <div class="vcard-name">Delivery Robot</div>
            <div class="vcard-desc">Autonomous ground robot navigating via LiDAR and cameras for unattended parcel delivery.</div>
        </div></div>
        """, unsafe_allow_html=True)

    st.markdown("")  # spacer

    # ── Service status & Live Statistics (on-demand) ─────────────────────────
    if "home_stats_loaded" not in st.session_state:
        st.session_state["home_stats_loaded"] = False

    if not st.session_state["home_stats_loaded"]:
        if st.button("🔍 Load Service Health & Live Statistics"):
            st.session_state["home_stats_loaded"] = True
            st.rerun()
    else:
        st.markdown("#### Service Health")

        def _svc_chip(label: str, url: str, port_label: str) -> None:
            data = api_get(f"{url}/")
            ok = data is not None
            dot = "🟢" if ok else "🔴"
            cls = "svc-ok" if ok else "svc-err"
            txt = "Running" if ok else "Offline"
            st.markdown(
                f'<div class="svc-chip {cls}">{dot}&nbsp; <b>{label}</b> &nbsp;— {txt} &nbsp;'
                f'<span style="opacity:.6;font-size:.77rem;">[{port_label}]</span></div>',
                unsafe_allow_html=True,
            )

        sc1, sc2, sc3 = st.columns(3)
        with sc1:
            _svc_chip("Vehicle Service",   VEHICLE_SERVICE,   "8004")
            _svc_chip("LiDAR Bridge",      LIDAR_SERVICE,     "8005")
        with sc2:
            _svc_chip("LiDAR Simulator",   DUMMY_LIDAR_SERVICE, "8006")
            _svc_chip("Teleop Service",    TELEOP_SERVICE,    "8007")
        with sc3:
            _svc_chip("Traffic Service",   TRAFFIC_SERVICE,   "8009")
            _svc_chip("Camera Service",    CAMERA_SERVICE,    "8010")
            _svc_chip("Robot Service",     ROBOT_SERVICE,     "8083")

        st.markdown("---")

        # ── Quick stats ──────────────────────────────────────────────────────
        st.markdown("#### Live Statistics")
        sq1, sq2 = st.columns(2)
        try:
            kpis = api_get(f"{VEHICLE_SERVICE}/fleet/kpis")

            with sq1:
                st.metric("🚗 Active AVs",  kpis["active"] if kpis else "—")
            with sq2:
                st.metric("🔋 Avg Battery", f"{kpis['avg_battery_pct']} %" if kpis else "—")
        except Exception as e:
            st.error(f"Error loading statistics: {e}")

        if st.button("🔄 Refresh"):
            st.rerun()

    st.markdown("---")

    # ── Raspberry Pi Display Broadcaster ─────────────────────────────────────
    st.markdown("#### 📡 Raspberry Pi Display")

    _disp_border = _border_dark if _dark else _border_light
    st.markdown(
        f'<div style="border:1px solid {_disp_border}; border-radius:10px; padding:1.1rem 1.3rem 0.8rem 1.3rem; margin-bottom:0.5rem;">',
        unsafe_allow_html=True,
    )

    if "mqtt_msg_input" not in st.session_state:
        st.session_state["mqtt_msg_input"] = MQTT_DEFAULT
    if "mqtt_last_status" not in st.session_state:
        st.session_state["mqtt_last_status"] = None
    if "mqtt_last_ok" not in st.session_state:
        st.session_state["mqtt_last_ok"] = None

    disp_col1, disp_col2 = st.columns([4, 1], gap="small")

    with disp_col1:
        msg_input = st.text_input(
            "Message to display on Pi",
            value=st.session_state["mqtt_msg_input"],
            placeholder="Type a message…",
            key="mqtt_text_input_widget",
            label_visibility="collapsed",
        )

    with disp_col2:
        send_clicked = st.button("📤 Send", use_container_width=True, type="primary")

    preset_col1, preset_col2, preset_col3, preset_col4 = st.columns(4, gap="small")
    with preset_col1:
        if st.button("✅ OCC is online",       use_container_width=True):
            st.session_state["mqtt_msg_input"] = "OCC is online"
            st.rerun()
    with preset_col2:
        if st.button("⚠️ Stand by",            use_container_width=True):
            st.session_state["mqtt_msg_input"] = "Stand by"
            st.rerun()
    with preset_col3:
        if st.button("🚧 Route closed",         use_container_width=True):
            st.session_state["mqtt_msg_input"] = "Route closed"
            st.rerun()
    with preset_col4:
        if st.button("🔴 Emergency stop",       use_container_width=True):
            st.session_state["mqtt_msg_input"] = "Emergency stop"
            st.rerun()

    if send_clicked:
        payload = msg_input.strip() or MQTT_DEFAULT
        with st.spinner("Publishing…"):
            ok, info = mqtt_publish(payload)
        st.session_state["mqtt_last_ok"]     = ok
        st.session_state["mqtt_last_status"] = info
        if ok:
            st.session_state["mqtt_msg_input"] = payload

    if st.session_state["mqtt_last_status"] is not None:
        if st.session_state["mqtt_last_ok"]:
            st.success(f"✅ {st.session_state['mqtt_last_status']}")
        else:
            st.error(f"❌ {st.session_state['mqtt_last_status']}")

    st.caption(f"Broker: `{MQTT_BROKER}:{MQTT_PORT}` · Topic: `{MQTT_TOPIC}` · QoS 1")
    st.markdown("</div>", unsafe_allow_html=True)


# ==================== FLEET KPIs PAGE ====================
elif page == "🚗 Fleet KPIs":

    STATUS_COLORS = {
        "Active":   "#00B400",
        "Idle":     "#FFA500",
        "Charging": "#4B6BFF",
        "Error":    "#FF4B4B",
    }
    HEALTH_ICONS = {"Good": "🟢", "Warning": "🟡", "Critical": "🔴"}

    try:
        if st.button("▶ Simulate New Readings"):
            api_post(f"{VEHICLE_SERVICE}/fleet/simulate")
            st.rerun()

        kpi = api_get(f"{VEHICLE_SERVICE}/fleet/kpis")
        if not kpi:
            raise ValueError("No KPI data")

        c1, c2, c3, c4, c5 = st.columns(5)
        with c1: st.metric("Total Vehicles", kpi["total_vehicles"])
        with c2: st.metric(
            "Status Breakdown",
            f"🟢{kpi['active']} 🟡{kpi['idle']} 🔵{kpi['charging']} 🔴{kpi['error']}",
        )
        with c3: st.metric("Avg Speed",   f"{kpi['avg_speed_kmh']} km/h")
        with c4: st.metric("Avg Battery", f"{kpi['avg_battery_pct']} %")
        with c5:
            icon = HEALTH_ICONS.get(kpi["fleet_health"], "")
            st.metric("Fleet Health", f"{icon} {kpi['fleet_health']}")

        st.markdown("---")

        vdata    = api_get(f"{VEHICLE_SERVICE}/fleet/vehicles")
        vehicles = vdata["vehicles"] if vdata else []
        df_v     = pd.DataFrame(vehicles)

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
            st.altair_chart(pie, width='stretch')

        with col_bat:
            st.subheader("Battery Levels (%)")
            bat_df = df_v[["id", "battery"]].set_index("id")
            st.bar_chart(bat_df, color="#4B6BFF", width='stretch')

        st.subheader("Speed Trend (km/h)")
        hdata   = api_get(f"{VEHICLE_SERVICE}/fleet/history")
        history = hdata["history"] if hdata else []
        if history:
            hist_df = pd.DataFrame(history)
            hist_df["timestamp"] = pd.to_datetime(hist_df["timestamp"])
            speed_pivot = hist_df.pivot_table(
                index="timestamp", columns="vehicle_id", values="speed", aggfunc="mean"
            )
            st.line_chart(speed_pivot, width='stretch')
        else:
            st.info("No history yet — click **▶ Simulate New Readings** to generate data.")

        st.subheader("Live Vehicle Positions — Magdeburg, DE")
        df_map         = df_v[["id", "latitude", "longitude", "status"]].copy()
        df_map["color"] = df_map["status"].map(STATUS_COLORS).fillna("#B4B4B4")
        st.map(df_map, latitude="latitude", longitude="longitude", color="color", size=120)

        st.subheader("Fleet Telemetry Table")
        display_df = df_v[[
            "id", "name", "status", "speed", "battery",
            "cpu_usage", "sensor_health", "distance_km", "last_updated",
        ]].copy()
        display_df.columns = [
            "ID", "Name", "Status", "Speed (km/h)", "Battery (%)",
            "CPU (%)", "Sensor Health", "Distance (km)", "Last Updated",
        ]
        st.dataframe(display_df, width='stretch', hide_index=True)

        # ── Per-vehicle actions (deregister) ──────────────────────────────────
        with st.expander("🗑️ Deregister a Vehicle", expanded=False):
            del_id = st.selectbox("Select vehicle to remove",
                                  [v["id"] for v in vehicles], key="del_vid")
            if st.button("Remove from Fleet", type="primary", key="del_btn"):
                try:
                    r = requests.delete(f"{VEHICLE_SERVICE}/fleet/vehicles/{del_id}", timeout=3)
                    if r.status_code == 200:
                        api_get.clear()
                        st.success(f"✅ {del_id} deregistered")
                        st.rerun()
                    else:
                        st.error(r.json().get("detail", "Error"))
                except Exception as ex:
                    st.error(f"Error: {ex}")

    except Exception:
        st.error("⚠️ Vehicle service unavailable. Start it with: `python vehicle_service.py`")

    # ── Register new vehicle ───────────────────────────────────────────────────
    st.markdown("---")
    with st.expander("➕ Register New Vehicle", expanded=False):
        with st.form("register_vehicle_form"):
            rc1, rc2 = st.columns(2)
            with rc1:
                reg_name  = st.text_input("Vehicle Name", placeholder="Cargo Bot Alpha")
                reg_type  = st.selectbox("Vehicle Type",
                    ["Autonomous Vehicle", "Tugger Train", "Cargo Bike", "Delivery Robot"])
            with rc2:
                reg_lat   = st.number_input("Latitude",  value=_BASE_LAT, format="%.6f", step=0.001,
                                             key="reg_lat")
                reg_lon   = st.number_input("Longitude", value=_BASE_LON, format="%.6f", step=0.001,
                                             key="reg_lon")
            submitted = st.form_submit_button("🚀 Register & Add to Fleet")
            if submitted:
                if reg_name.strip():
                    try:
                        r = requests.post(
                            f"{VEHICLE_SERVICE}/fleet/register",
                            json={"name": reg_name.strip(), "vehicle_type": reg_type,
                                  "latitude": reg_lat, "longitude": reg_lon},
                            timeout=3,
                        )
                        if r.status_code == 200:
                            api_get.clear()
                            new_v = r.json()["vehicle"]
                            st.success(f"✅ Registered **{new_v['name']}** as {new_v['id']}")
                            st.rerun()
                        else:
                            st.error(r.json().get("detail", "Registration failed"))
                    except Exception as ex:
                        st.error(f"Error: {ex}")
                else:
                    st.warning("Please enter a vehicle name.")


# ==================== LIDAR PAGE ====================
elif page == "📡 LiDAR":

    with st.expander("⚙️ Connection Settings", expanded=False):
        with st.form("lidar_config_form"):
            cfg_ip    = st.text_input("Robot IP",      value="192.168.1.100")
            cfg_port  = st.number_input("rosbridge Port", value=9090, step=1)
            cfg_topic = st.text_input("ROS Topic",     value="/scan")
            if st.form_submit_button("🔌 Apply & Reconnect"):
                try:
                    requests.post(
                        f"{LIDAR_SERVICE}/config",
                        json={"robot_ip": cfg_ip, "robot_port": cfg_port, "ros_topic": cfg_topic},
                        timeout=3,
                    )
                    api_get.clear()
                    st.success("Reconnecting…")
                    st.rerun()
                except Exception as e:
                    st.error(f"Error: {e}")

    try:
        status = api_get(f"{LIDAR_SERVICE}/scan/status")
        scan   = api_get(f"{LIDAR_SERVICE}/scan/latest")

        conn_label = "🟢 Connected" if (status and status["connected"]) else "🔴 Disconnected"
        c1, c2, c3, c4 = st.columns(4)
        with c1: st.metric("Connection", conn_label)
        with c2: st.metric("Scan Rate",  f"{status['scan_rate_hz']} Hz" if status else "—")
        with c3:
            pt_count = scan["point_count"] if scan else 0
            st.metric("Points / Scan", pt_count)
        with c4:
            rng = (f"{scan['range_min_measured']} – {scan['range_max_measured']} m"
                   if scan else "—")
            st.metric("Range (min – max)", rng)

        st.markdown("---")

        col_chart, col_info = st.columns([3, 1])

        with col_chart:
            if scan and scan.get("points"):
                pts_df    = pd.DataFrame(scan["points"])
                rate_str  = f"{status['scan_rate_hz']} Hz" if (status and status["connected"]) else "—"
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
                            alt.Tooltip("x:Q",         format=".3f"),
                            alt.Tooltip("y:Q",         format=".3f"),
                            alt.Tooltip("range:Q",     title="Range (m)", format=".3f"),
                            alt.Tooltip("angle_deg:Q", title="Angle (°)",  format=".1f"),
                        ],
                    )
                    .properties(width="container", height=480)
                )
                origin = (
                    alt.Chart(pd.DataFrame({"x": [0], "y": [0]}))
                    .mark_point(size=120, shape="triangle-up", color="#00FFFF", filled=True)
                    .encode(x="x:Q", y="y:Q")
                )
                st.altair_chart(scatter + origin, width='stretch')
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
                st.write(f"**Total scans:** {status['scan_count'] if status else '—'}")
            else:
                st.write("Waiting for first scan…")

        st.subheader("Scan History — Range Statistics")
        hdata = api_get(f"{LIDAR_SERVICE}/scan/history")
        hist  = hdata["history"] if hdata else []
        if hist:
            hist_df = pd.DataFrame(hist)
            hist_df["timestamp"] = pd.to_datetime(hist_df["timestamp"])
            st.line_chart(
                hist_df.set_index("timestamp")[["min_range", "mean_range", "max_range"]],
                width='stretch',
            )
        else:
            st.info("History will appear after the first scan is received.")

        st.markdown("---")
        if st.toggle("Auto-refresh every 3 s", value=False, key="lidar_ar"):
            time.sleep(3)
            api_get.clear()
            st.rerun()

    except Exception:
        st.error("⚠️ LiDAR service unavailable. Start it with: `python lidar_service.py`")


# ==================== DUMMY LIDAR SIM PAGE ====================
elif page == "🤖 LiDAR Sim":
    st.caption("Synthetic 2-D laser scan: robot (▲) in a rectangular room with moving obstacles (●).")

    with st.expander("⚙️ Simulation Settings", expanded=False):
        with st.form("lidar_sim_config_form"):
            c1, c2, c3, c4 = st.columns(4)
            with c1: cfg_w   = st.number_input("Room Width (m)",  min_value=2.0,  max_value=30.0, value=10.0, step=0.5)
            with c2: cfg_h   = st.number_input("Room Height (m)", min_value=2.0,  max_value=20.0, value=8.0,  step=0.5)
            with c3: cfg_obs = st.number_input("Obstacles",       min_value=0,    max_value=10,   value=3,    step=1)
            with c4: cfg_noise = st.number_input("Noise Std (m)", min_value=0.0,  max_value=0.5,  value=0.02, step=0.01, format="%.3f")
            if st.form_submit_button("⚙️ Apply"):
                try:
                    requests.post(
                        f"{DUMMY_LIDAR_SERVICE}/config",
                        json={"room_width": cfg_w, "room_height": cfg_h,
                              "num_obstacles": cfg_obs, "noise_std": cfg_noise},
                        timeout=3,
                    )
                    api_get.clear()
                    st.success("Settings applied — next scan will reflect changes.")
                    st.rerun()
                except Exception as e:
                    st.error(f"Error: {e}")

    try:
        status = api_get(f"{DUMMY_LIDAR_SERVICE}/scan/status")
        scan   = api_get(f"{DUMMY_LIDAR_SERVICE}/scan/latest")

        c1, c2, c3, c4, c5 = st.columns(5)
        with c1: st.metric("Simulator",      "🟢 Running")
        with c2: st.metric("Scan Rate",      f"{status['scan_rate_hz']} Hz" if status else "—")
        with c3: st.metric("Points / Scan",  scan["point_count"] if scan else 0)
        with c4:
            rng = (f"{scan['range_min_measured']} – {scan['range_max_measured']} m"
                   if scan else "—")
            st.metric("Range (min – max)", rng)
        with c5: st.metric("Total Scans", status["scan_count"] if status else "—")

        st.markdown("---")

        col_chart, col_info = st.columns([3, 1])

        with col_chart:
            if scan and scan.get("points"):
                pts_df = pd.DataFrame(scan["points"])
                room_w = scan["room"]["width"]
                room_h = scan["room"]["height"]
                hw, hh = room_w / 2, room_h / 2

                boundary_pts = [
                    {"bx": -hw, "by": -hh}, {"bx":  hw, "by": -hh},
                    {"bx":  hw, "by":  hh}, {"bx": -hw, "by":  hh},
                    {"bx": -hw, "by": -hh},
                ]
                boundary_df = pd.DataFrame(boundary_pts)
                boundary = (
                    alt.Chart(boundary_df)
                    .mark_line(color="#555555", strokeDash=[6, 3], strokeWidth=1.5)
                    .encode(x=alt.X("bx:Q", title="X (m)"), y=alt.Y("by:Q", title="Y (m)"))
                )

                scatter = (
                    alt.Chart(pts_df,
                              title=f"Scan #{status['scan_count'] if status else '?'}  ·  "
                                    f"{scan['point_count']} pts  ·  "
                                    f"{status['scan_rate_hz'] if status else '?'} Hz")
                    .mark_circle(size=5, opacity=0.80)
                    .encode(
                        x=alt.X("x:Q", title="X (m)", scale=alt.Scale(domain=[-hw - 0.5, hw + 0.5])),
                        y=alt.Y("y:Q", title="Y (m)", scale=alt.Scale(domain=[-hh - 0.5, hh + 0.5])),
                        color=alt.Color(
                            "range:Q",
                            title="Range (m)",
                            scale=alt.Scale(scheme="redyellowgreen", reverse=True),
                        ),
                        tooltip=[
                            alt.Tooltip("x:Q",         format=".3f"),
                            alt.Tooltip("y:Q",         format=".3f"),
                            alt.Tooltip("range:Q",     title="Range (m)", format=".3f"),
                            alt.Tooltip("angle_deg:Q", title="Angle (°)", format=".1f"),
                        ],
                    )
                    .properties(width="container", height=480)
                )

                origin = (
                    alt.Chart(pd.DataFrame({"x": [0], "y": [0]}))
                    .mark_point(size=150, shape="triangle-up", color="#00FFFF", filled=True)
                    .encode(x="x:Q", y="y:Q", tooltip=alt.value("Robot origin"))
                )

                layers = [boundary, scatter, origin]
                if scan.get("obstacles"):
                    obs_df = pd.DataFrame(scan["obstacles"])
                    obs_markers = (
                        alt.Chart(obs_df)
                        .mark_circle(size=200, color="#FF6B35", opacity=0.75, filled=True)
                        .encode(
                            x=alt.X("x:Q"),
                            y=alt.Y("y:Q"),
                            tooltip=[
                                alt.Tooltip("x:Q", title="Obstacle X", format=".2f"),
                                alt.Tooltip("y:Q", title="Obstacle Y", format=".2f"),
                            ],
                        )
                    )
                    layers.append(obs_markers)

                st.altair_chart(
                    alt.layer(*layers).resolve_scale(color="independent"),
                    width='stretch',
                )
                st.caption(
                    "▲ cyan = robot origin  ·  ● orange = obstacle centre  "
                    "·  dashed grey = room boundary  ·  dots coloured by range"
                )
            else:
                st.info("Waiting for first scan… (service may still be starting up)")

        with col_info:
            st.subheader("Last Scan")
            if scan:
                st.write(f"**Time:** {scan['timestamp']}")
                st.write(f"**Points:** {scan['point_count']}")
                st.write(f"**Min range:** {scan['range_min_measured']} m")
                st.write(f"**Max range:** {scan['range_max_measured']} m")
                st.write(f"**Room:** {scan['room']['width']} × {scan['room']['height']} m")
                st.write(f"**Obstacles:** {len(scan.get('obstacles', []))}")
                st.write(f"**Total scans:** {status['scan_count'] if status else '—'}")
            else:
                st.write("No data yet…")

        st.subheader("Scan History — Range Statistics")
        hdata = api_get(f"{DUMMY_LIDAR_SERVICE}/scan/history")
        hist  = (hdata or {}).get("history", [])
        if hist:
            hist_df = pd.DataFrame(hist)
            hist_df["timestamp"] = pd.to_datetime(hist_df["timestamp"])
            st.line_chart(
                hist_df.set_index("timestamp")[["min_range", "mean_range", "max_range"]],
                width='stretch',
            )
        else:
            st.info("Range history will appear after a few scans.")

        st.markdown("---")
        if st.toggle("Auto-refresh every 1 s", value=False, key="sim_ar"):
            time.sleep(1)
            api_get.clear()
            st.rerun()

    except Exception:
        st.error("⚠️ Dummy LiDAR service unavailable. Start it with: `python dummy_lidar_service.py`")


# ==================== TELEOP PAGE ====================
elif page == "🕹️ Teleop":
    st.caption("Remote-control the simulated robot with D-pad buttons. Connect to a fleet vehicle to sync GPS position in real time.")

    # ── Synthetic camera tile (SVG) ────────────────────────────────────────────
    def _cam_svg(label: str, heading_deg: float, speed: float, status: str) -> str:
        W, H, cx, cy = 400, 240, 200, 110
        border = {
            "Idle": "#334455", "Moving": "#00bb44",
            "Rotating": "#4488ff", "E-Stop": "#dd2222",
        }.get(status, "#334455")
        hud  = "#00cc44"
        warn = "#ff4444" if status == "E-Stop" else hud
        lid  = label.replace(" ", "_")
        lines = ""
        for i in range(8):
            s  = (i + 1) * 28
            xl = max(0, cx - s)
            xr = min(W, cx + s)
            lines += f'<line x1="{cx}" y1="{cy}" x2="{xl}" y2="{H}" stroke="#162616" stroke-width="1"/>'
            lines += f'<line x1="{cx}" y1="{cy}" x2="{xr}" y2="{H}" stroke="#162616" stroke-width="1"/>'
        for i in range(5):
            t  = (i + 1) / 6.0
            gy = cy + (H - cy) * t
            xl = cx - cx * (t ** 0.6)
            xr = cx + (W - cx) * (t ** 0.6)
            lines += f'<line x1="{xl:.0f}" y1="{gy:.0f}" x2="{xr:.0f}" y2="{gy:.0f}" stroke="#162616" stroke-width="1"/>'
        return (
            f'<svg width="{W}" height="{H}" xmlns="http://www.w3.org/2000/svg" '
            f'style="border-radius:10px;border:2px solid {border};display:block;">'
            f'<defs>'
            f'<linearGradient id="sky_{lid}" x1="0" y1="0" x2="0" y2="1">'
            f'<stop offset="0%" stop-color="#060b16"/><stop offset="100%" stop-color="#0b1620"/>'
            f'</linearGradient>'
            f'<linearGradient id="gnd_{lid}" x1="0" y1="0" x2="0" y2="1">'
            f'<stop offset="0%" stop-color="#0a0e0a"/><stop offset="100%" stop-color="#080c08"/>'
            f'</linearGradient>'
            f'</defs>'
            f'<rect width="{W}" height="{H}" fill="#080c10" rx="8"/>'
            f'<rect width="{W}" height="{cy}" fill="url(#sky_{lid})"/>'
            f'<rect y="{cy}" width="{W}" height="{H - cy}" fill="url(#gnd_{lid})"/>'
            f'{lines}'
            f'<line x1="0" y1="{cy}" x2="{W}" y2="{cy}" stroke="#1a3322" stroke-width="1"/>'
            f'<line x1="{cx - 24}" y1="{cy}" x2="{cx + 24}" y2="{cy}" stroke="{hud}" stroke-width="1.5" opacity=".85"/>'
            f'<line x1="{cx}" y1="{cy - 24}" x2="{cx}" y2="{cy + 24}" stroke="{hud}" stroke-width="1.5" opacity=".85"/>'
            f'<circle cx="{cx}" cy="{cy}" r="10" fill="none" stroke="{hud}" stroke-width="1.5" opacity=".55"/>'
            f'<rect x="8" y="8" width="136" height="24" fill="rgba(0,0,0,.7)" rx="4"/>'
            f'<text x="14" y="25" fill="{hud}" font-family="monospace" font-size="13" font-weight="bold">\U0001f4f7 {label}</text>'
            f'<rect x="{W - 60}" y="8" width="52" height="24" fill="rgba(0,0,0,.7)" rx="4"/>'
            f'<text x="{W - 54}" y="25" fill="{hud}" font-family="monospace" font-size="12">{heading_deg:.0f}\u00b0</text>'
            f'<rect x="8" y="{H - 28}" width="110" height="22" fill="rgba(0,0,0,.7)" rx="4"/>'
            f'<text x="14" y="{H - 13}" fill="{hud}" font-family="monospace" font-size="11">{abs(speed):.2f} m/s</text>'
            f'<rect x="{W - 102}" y="{H - 28}" width="94" height="22" fill="rgba(0,0,0,.7)" rx="4"/>'
            f'<text x="{W - 96}" y="{H - 13}" fill="{warn}" font-family="monospace" font-size="11">{status.upper()}</text>'
            f'</svg>'
        )

    try:
        state = api_get(f"{TELEOP_SERVICE}/robot/state")
        if not state:
            raise ValueError("No state")

        # ── Fleet connection panel ─────────────────────────────────────────────
        conn_data    = api_get(f"{TELEOP_SERVICE}/robot/connection") or {}
        connected_to = conn_data.get("vehicle_id")

        with st.expander("🔗 Fleet Vehicle Connection", expanded=(connected_to is None)):
            if connected_to:
                st.success(
                    f"🟢 Connected to **{conn_data.get('vehicle_name', connected_to)}**"
                    f" ({connected_to})"
                )
                ci1, ci2 = st.columns(2)
                with ci1:
                    h_lat = conn_data.get("home_lat")
                    h_lon = conn_data.get("home_lon")
                    if h_lat and h_lon:
                        st.caption(f"Home: {h_lat:.6f}°N, {h_lon:.6f}°E")
                with ci2:
                    if st.button("🔌 Disconnect", width='stretch', key="disc_btn"):
                        try:
                            requests.post(f"{TELEOP_SERVICE}/robot/disconnect", timeout=3)
                            requests.post(
                                f"{VEHICLE_SERVICE}/fleet/vehicles/{connected_to}/release",
                                timeout=3,
                            )
                        except Exception:
                            pass
                        api_get.clear()
                        st.rerun()
            else:
                st.info("Not connected to a fleet vehicle. Connect below to take position control.")
                fleet_data = api_get(f"{VEHICLE_SERVICE}/fleet/vehicles")
                fleet_list = (fleet_data or {}).get("vehicles", [])
                available  = [v for v in fleet_list if not v.get("teleop_active")]
                if available:
                    sel_map = {
                        f"{v['id']} — {v['name']}  \U0001f50b{v['battery']}%  [{v['status']}]": v
                        for v in available
                    }
                    chosen_lbl = st.selectbox(
                        "Available Vehicles", list(sel_map.keys()), key="fleet_sel",
                    )
                    chosen = sel_map[chosen_lbl]
                    if st.button(
                        "\U0001f3ae Connect & Take Control", type="primary",
                        key="conn_btn", width='stretch',
                    ):
                        try:
                            requests.post(
                                f"{TELEOP_SERVICE}/robot/connect",
                                json={
                                    "vehicle_id":   chosen["id"],
                                    "vehicle_name": chosen["name"],
                                    "battery":      chosen["battery"],
                                    "home_lat":     chosen["latitude"],
                                    "home_lon":     chosen["longitude"],
                                },
                                timeout=3,
                            )
                        except Exception:
                            pass
                        api_get.clear()
                        st.rerun()
                else:
                    st.warning(
                        "No available vehicles — vehicle service may be offline "
                        "or all vehicles are under teleop control."
                    )

        STATUS_COLORS_T = {"Idle": "🟡", "Moving": "🟢", "Rotating": "🔵", "E-Stop": "🔴"}
        icon = STATUS_COLORS_T.get(state["status"], "⚪")

        if state["status"] == "E-Stop":
            st.error("🚨 EMERGENCY STOP ACTIVE — click **Reset** to clear")
        elif state["battery"] < 20:
            st.warning(f"⚠️ Low battery: {state['battery']:.1f}%")

        conn_str = f"🔗 {state['connected_to']}" if state.get("connected_to") else "🔌 Standalone"
        m1, m2, m3, m4, m5, m6 = st.columns(6)
        with m1: st.metric("Status",    f"{icon} {state['status']}")
        with m2: st.metric("X (m)",     f"{state['x']:.3f}")
        with m3: st.metric("Y (m)",     f"{state['y']:.3f}")
        with m4: st.metric("Heading",   f"{state['heading']:.1f}°")
        with m5: st.metric("Battery",   f"{state['battery']:.1f}%")
        with m6: st.metric("Fleet Link", conn_str)

        st.markdown("---")

        col_ctrl, col_right = st.columns([1, 2])

        with col_ctrl:
            st.subheader("Controls")
            lin_speed  = st.slider("Linear speed (m/s)", 0.1, 2.0, 0.5, 0.1)
            turn_speed = st.slider("Turn speed (°/s)",   10.0, 90.0, 30.0, 5.0)
            step_dur   = st.slider("Step duration (s)",  0.1, 2.0, 0.5, 0.1)

            def _send_vel(linear: float, angular: float) -> None:
                resp = api_post(
                    f"{TELEOP_SERVICE}/robot/cmd_vel",
                    json={"linear": linear, "angular": angular, "duration": step_dur},
                )
                if connected_to and resp.ok:
                    rd = resp.json()
                    if rd.get("latitude"):
                        try:
                            requests.patch(
                                f"{VEHICLE_SERVICE}/fleet/vehicles/{connected_to}/position",
                                json={
                                    "latitude":    rd["latitude"],
                                    "longitude":   rd["longitude"],
                                    "heading_deg": rd["heading"],
                                    "speed":       round(abs(linear) * 3.6, 1),
                                },
                                timeout=2,
                            )
                        except Exception:
                            pass
                st.rerun()

            st.markdown("**D-Pad**")
            _, fc, _ = st.columns([1, 1, 1])
            with fc:
                if st.button("⬆️ Fwd", width='stretch',
                             disabled=(state["status"] == "E-Stop")):
                    _send_vel(lin_speed, 0.0)

            lc, mc, rc = st.columns(3)
            with lc:
                if st.button("⬅️ Left", width='stretch',
                             disabled=(state["status"] == "E-Stop")):
                    _send_vel(0.0, turn_speed)
            with mc:
                if st.button("⏹️ Stop", width='stretch'):
                    requests.post(f"{TELEOP_SERVICE}/robot/stop", timeout=3)
                    if connected_to:
                        try:
                            requests.patch(
                                f"{VEHICLE_SERVICE}/fleet/vehicles/{connected_to}/position",
                                json={
                                    "latitude":    state.get("latitude",  _BASE_LAT),
                                    "longitude":   state.get("longitude", _BASE_LON),
                                    "heading_deg": state["heading"],
                                    "speed":       0.0,
                                },
                                timeout=2,
                            )
                        except Exception:
                            pass
                    api_get.clear()
                    st.rerun()
            with rc:
                if st.button("➡️ Right", width='stretch',
                             disabled=(state["status"] == "E-Stop")):
                    _send_vel(0.0, -turn_speed)

            _, bc, _ = st.columns([1, 1, 1])
            with bc:
                if st.button("⬇️ Back", width='stretch',
                             disabled=(state["status"] == "E-Stop")):
                    _send_vel(-lin_speed, 0.0)

            st.markdown("---")
            st.markdown("**Diagonal**")
            dl, dr = st.columns(2)
            with dl:
                if st.button("↖️ Fwd-L", width='stretch',
                             disabled=(state["status"] == "E-Stop")):
                    _send_vel(lin_speed, turn_speed)
            with dr:
                if st.button("↗️ Fwd-R", width='stretch',
                             disabled=(state["status"] == "E-Stop")):
                    _send_vel(lin_speed, -turn_speed)

            st.markdown("---")
            if st.button("🚨 EMERGENCY STOP", width='stretch', type="primary"):
                requests.post(f"{TELEOP_SERVICE}/robot/estop", timeout=3)
                api_get.clear()
                st.rerun()

            if st.button("🔄 Reset to Origin", width='stretch'):
                requests.post(f"{TELEOP_SERVICE}/robot/reset", timeout=3)
                if connected_to:
                    try:
                        requests.post(
                            f"{VEHICLE_SERVICE}/fleet/vehicles/{connected_to}/release",
                            timeout=3,
                        )
                    except Exception:
                        pass
                api_get.clear()
                st.rerun()

        with col_right:
            # ── Camera feeds ──────────────────────────────────────────────────
            st.subheader("📷 Camera Feeds")
            _h  = state["heading"]
            _sp = state["speed"]
            _st = state["status"]

            cr1a, cr1b = st.columns(2)
            cr2a, cr2b = st.columns(2)
            with cr1a:
                st.markdown(_cam_svg("FRONT", _h,               _sp, _st), unsafe_allow_html=True)
            with cr1b:
                st.markdown(_cam_svg("RIGHT", (_h - 90) % 360,  _sp, _st), unsafe_allow_html=True)
            with cr2a:
                st.markdown(_cam_svg("LEFT",  (_h + 90) % 360,  _sp, _st), unsafe_allow_html=True)
            with cr2b:
                st.markdown(_cam_svg("REAR",  (_h + 180) % 360, _sp, _st), unsafe_allow_html=True)

            st.markdown("---")

            # ── Robot trail ────────────────────────────────────────────────────
            st.subheader("🗺️ Robot Trail")
            path_data = api_get(f"{TELEOP_SERVICE}/robot/path")
            path      = (path_data or {}).get("path", [])

            if path and len(path) >= 2:
                path_df = pd.DataFrame(path)

                trail = (
                    alt.Chart(path_df)
                    .mark_line(color="#4B6BFF", strokeWidth=2, opacity=0.6)
                    .encode(
                        x=alt.X("x:Q", title="X (m)"),
                        y=alt.Y("y:Q", title="Y (m)"),
                        order="timestamp:N",
                    )
                )
                dots = (
                    alt.Chart(path_df)
                    .mark_circle(size=20, color="#4B6BFF", opacity=0.4)
                    .encode(
                        x="x:Q", y="y:Q",
                        tooltip=[
                            alt.Tooltip("x:Q",       format=".3f"),
                            alt.Tooltip("y:Q",       format=".3f"),
                            alt.Tooltip("heading:Q", title="Heading (°)", format=".1f"),
                            alt.Tooltip("timestamp:N"),
                        ],
                    )
                )
                origin_mark = (
                    alt.Chart(pd.DataFrame({"x": [0], "y": [0]}))
                    .mark_point(size=120, shape="diamond", color="#888888", filled=True)
                    .encode(x="x:Q", y="y:Q", tooltip=alt.value("Origin"))
                )
                cur_df = pd.DataFrame([{
                    "x": state["x"], "y": state["y"], "heading": state["heading"],
                }])
                cur_pos = (
                    alt.Chart(cur_df)
                    .mark_circle(size=200, color="#FF4B4B", filled=True)
                    .encode(
                        x="x:Q", y="y:Q",
                        tooltip=[
                            alt.Tooltip("x:Q",       title="X (m)",       format=".3f"),
                            alt.Tooltip("y:Q",       title="Y (m)",       format=".3f"),
                            alt.Tooltip("heading:Q", title="Heading (°)", format=".1f"),
                        ],
                    )
                )
                hr = math.radians(state["heading"])
                arrow_df = pd.DataFrame([
                    {"ax": state["x"], "ay": state["y"]},
                    {"ax": state["x"] + 0.4 * math.cos(hr),
                     "ay": state["y"] + 0.4 * math.sin(hr)},
                ])
                arrow = (
                    alt.Chart(arrow_df)
                    .mark_line(color="#FF4B4B", strokeWidth=3)
                    .encode(x="ax:Q", y="ay:Q")
                )
                chart = (
                    alt.layer(trail, dots, origin_mark, cur_pos, arrow)
                    .resolve_scale(color="independent")
                    .properties(
                        width="container", height=320,
                        title=(
                            f"Trail — {len(path)} waypoints  ·  "
                            f"total {state['total_distance']:.2f} m"
                        ),
                    )
                )
                st.altair_chart(chart, width='stretch')
                st.caption("● red = robot (arrow = heading)  ◆ grey = origin  — blue = path")
            else:
                st.info("Move the robot to see its trail here.")

        st.subheader("Command History")
        hist_data = api_get(f"{TELEOP_SERVICE}/robot/history")
        hist      = (hist_data or {}).get("history", [])
        if hist:
            hist_df = pd.DataFrame(list(reversed(hist)))
            hist_df.columns = [c.replace("_", " ").title() for c in hist_df.columns]
            st.dataframe(hist_df, width='stretch', hide_index=True)
        else:
            st.info("No commands sent yet — use the D-pad controls above.")

    except Exception:
        st.error("⚠️ Teleoperation service unavailable. Start it with: `python teleop_service.py`")


# ==================== TRAFFIC LIGHTS PAGE ====================
if page == "🚦 Traffic Lights":

    _TL_SERVICE_URL = TRAFFIC_SERVICE

    # ── Connection status ───────────────────────────────────────────────────────
    health_data = api_get(f"{_TL_SERVICE_URL}/")
    if health_data is None:
        st.error("⚠️ MQTT Traffic Light Service is offline. Start it with: `python mqtt_traffic_service.py`")
        st.stop()

    mqtt_info = health_data.get("mqtt", {})
    connected = mqtt_info.get("connected", False)
    broker    = mqtt_info.get("broker", "—")
    topic     = mqtt_info.get("topic", "—")
    msgs      = mqtt_info.get("messages_received", 0)

    _conn_cls = "svc-ok" if connected else "svc-err"
    _conn_lbl = "MQTT Connected" if connected else "MQTT Disconnected"
    st.markdown(
        f'<span class="svc-chip {_conn_cls}">⬤ {_conn_lbl}</span>',
        unsafe_allow_html=True,
    )
    st.caption(f"Broker: **{broker}** · Topic: **{topic}** · Messages received: **{msgs}**")
    st.link_button("🚦 Open Full MQTT Dashboard", url="http://localhost:8511", help="Standalone traffic light dashboard")

    st.markdown("---")

    # ── Fetch lights data ──────────────────────────────────────────────────────
    lights_data   = api_get(f"{_TL_SERVICE_URL}/traffic/lights")
    summary_data  = api_get(f"{_TL_SERVICE_URL}/traffic/summary")
    history_data  = api_get(f"{_TL_SERVICE_URL}/traffic/history?limit=200")

    lights  = (lights_data  or {}).get("lights",  [])
    summary = summary_data or {}
    events  = (history_data or {}).get("events", [])

    # ── KPI row ────────────────────────────────────────────────────────────────
    k1, k2, k3, k4, k5 = st.columns(5)
    _by = summary.get("by_state", {})
    k1.metric("Total Lights",  summary.get("total",   len(lights)))
    k2.metric("🟢 Green",      _by.get("GREEN",  0))
    k3.metric("🟡 Yellow",     _by.get("YELLOW", 0))
    k4.metric("🔴 Red",        _by.get("RED",    0))
    k5.metric("⚠️ Faults",     summary.get("faults", 0))

    if not lights:
        st.info(f"No traffic-light data yet — waiting for MQTT messages on `{topic}` from `{broker}`.")
    else:
        # ── Traffic-light grid ────────────────────────────────────────────────
        st.subheader("Live Traffic Light States")

        _STATE_COLOR  = {"GREEN": "#238636", "YELLOW": "#d29922", "RED": "#da3633"}
        _STATE_BG     = {"GREEN": "rgba(35,134,54,0.12)",
                         "YELLOW": "rgba(210,153,34,0.12)",
                         "RED":    "rgba(218,54,51,0.12)"}
        _STATE_BORDER = {"GREEN": "rgba(35,134,54,0.35)",
                         "YELLOW": "rgba(210,153,34,0.35)",
                         "RED":    "rgba(218,54,51,0.35)"}
        _PHASE_BADGE  = {"fault":       "🔴 FAULT",
                         "maintenance": "🔧 MAINT",
                         "normal":      ""}

        COLS = 4
        rows = [lights[i:i+COLS] for i in range(0, len(lights), COLS)]

        for row in rows:
            grid_cols = st.columns(COLS, gap="small")
            for col, lt in zip(grid_cols, row):
                state   = lt.get("state", "RED")
                phase   = lt.get("phase", "normal")
                tis     = lt.get("time_in_state_s", 0)
                cycle   = lt.get("cycle_s", 60)
                pct     = min(100, int(tis / max(cycle, 1) * 100))
                badge   = _PHASE_BADGE.get(phase, "")
                bg      = _STATE_BG[state]
                border  = _STATE_BORDER[state]
                color   = _STATE_COLOR[state]
                ts_raw  = lt.get("timestamp", "")
                ts_disp = ts_raw[11:19] if len(ts_raw) >= 19 else ts_raw

                col.markdown(
                    f"""
                    <div style="
                        background:{bg}; border:1px solid {border};
                        border-radius:12px; padding:14px 14px 10px 14px;
                        margin-bottom:6px;">
                      <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:6px;">
                        <span style="font-size:0.78rem; font-weight:700; color:{color}; letter-spacing:1px;">{state}</span>
                        <span style="font-size:0.7rem; color:#888;">{lt['id']}</span>
                      </div>
                      <div style="font-size:0.72rem; color:#aaa; margin-bottom:8px; line-height:1.4;">{lt.get('intersection','')}</div>
                      <div style="background:#1e1e1e; border-radius:4px; height:6px; margin-bottom:6px;">
                        <div style="background:{color}; width:{pct}%; height:6px; border-radius:4px;"></div>
                      </div>
                      <div style="display:flex; justify-content:space-between; font-size:0.68rem; color:#777;">
                        <span>{tis}s / {cycle}s</span>
                        <span>{badge or ts_disp}</span>
                      </div>
                    </div>
                    """,
                    unsafe_allow_html=True,
                )

        # ── Map ────────────────────────────────────────────────────────────────
        st.subheader("Intersection Map")

        try:
            import folium
            from streamlit_folium import st_folium  # type: ignore

            _map = folium.Map(
                location=[52.1270, 11.6280], zoom_start=13,
                tiles="CartoDB dark_matter" if _dark else "CartoDB positron",
            )
            _COLOR_MAP = {"GREEN": "green", "YELLOW": "orange", "RED": "red"}
            for lt in lights:
                clr  = _COLOR_MAP.get(lt.get("state", "RED"), "gray")
                tis  = lt.get("time_in_state_s", 0)
                html = (
                    f"<b>{lt['id']}</b><br>"
                    f"{lt.get('intersection','')}<br>"
                    f"State: <b>{lt.get('state')}</b><br>"
                    f"Phase: {lt.get('phase')}<br>"
                    f"Time in state: {tis}s"
                )
                folium.CircleMarker(
                    location=[lt["lat"], lt["lon"]],
                    radius=10,
                    color=clr, fill=True, fill_color=clr, fill_opacity=0.75,
                    popup=folium.Popup(html, max_width=220),
                    tooltip=f"{lt['id']}: {lt.get('state')}",
                ).add_to(_map)

            st_folium(_map, width=None, height=420, returned_objects=[])

        except ImportError:
            # folium not installed – show a plain table instead
            if lights:
                _map_df = pd.DataFrame([{
                    "ID":           lt["id"],
                    "Intersection": lt.get("intersection",""),
                    "State":        lt.get("state",""),
                    "Phase":        lt.get("phase",""),
                    "Lat":          lt.get("lat"),
                    "Lon":          lt.get("lon"),
                } for lt in lights])
                st.dataframe(_map_df, width='stretch', hide_index=True)
                st.caption("Install `folium` and `streamlit-folium` for an interactive map.")

        # ── State-change history chart ─────────────────────────────────────────
        st.subheader("State-Change Timeline")
        if events:
            ev_df = pd.DataFrame(events)
            ev_df["ts"] = pd.to_datetime(ev_df["event_time"], errors="coerce")
            ev_df = ev_df.dropna(subset=["ts"])
            ev_df = ev_df.sort_values("ts")
            ev_df["ts_str"] = ev_df["ts"].dt.strftime("%H:%M:%S")

            # Count transitions per state over time (1-minute buckets)
            ev_df["minute"] = ev_df["ts"].dt.floor("1min")
            bucket = (
                ev_df.groupby(["minute", "state"])
                .size()
                .reset_index(name="count")
            )
            bucket["minute_str"] = bucket["minute"].dt.strftime("%H:%M")

            _SCHEME = alt.Scale(
                domain=["GREEN", "YELLOW", "RED"],
                range=["#238636", "#d29922", "#da3633"],
            )
            chart = (
                alt.Chart(bucket)
                .mark_bar()
                .encode(
                    x=alt.X("minute_str:O", title="Time (1-min bucket)",
                             axis=alt.Axis(labelAngle=-45)),
                    y=alt.Y("count:Q", title="State-change events"),
                    color=alt.Color("state:N", scale=_SCHEME, legend=alt.Legend(title="State")),
                    tooltip=["minute_str:O", "state:N", "count:Q"],
                )
                .properties(width="container", height=220)
            )
            st.altair_chart(chart, width='stretch')

            # Raw event table (most recent first)
            st.subheader("Recent Events")
            disp_df = ev_df[["event_time", "id", "intersection", "state", "phase",
                              "time_in_state_s"]].copy()
            disp_df = disp_df.sort_values("event_time", ascending=False).head(50)
            disp_df.columns = ["Timestamp", "ID", "Intersection", "State",
                                "Phase", "Time in State (s)"]
            st.dataframe(disp_df, width='stretch', hide_index=True)
        else:
            st.info("No events recorded yet.")

        # ── Auto-refresh ───────────────────────────────────────────────────────
        time.sleep(2)
        st.rerun()


# ==================== CAMERA PAGE ====================
elif page == "📷 Camera":
    import base64 as _b64
    import streamlit.components.v1 as _components

    st.title("📷 Camera Streams")

    # ── Service health check ───────────────────────────────────────────────────
    cam_info = None
    try:
        cam_info = requests.get(f"{CAMERA_SERVICE}/", timeout=2).json()
    except Exception:
        pass

    if cam_info is None:
        st.error(
            "⚠️ Camera service unavailable.  "
            "Start it with: `python camera_service.py`"
        )
        st.stop()

    # ── IP Camera health ───────────────────────────────────────────────────────
    ip_cam_info = None
    try:
        ip_cam_info = requests.get(f"{CAMERA_SERVICE}/ip/health", timeout=3).json()
    except Exception:
        pass

    st.markdown("---")

    # ═══════════════════════════════════════════════════════════════════════════
    # USB Camera section
    # ═══════════════════════════════════════════════════════════════════════════
    st.subheader("🔌 USB Camera")

    if not cam_info.get("camera_open"):
        st.warning("USB camera service is running but no camera is open.  "
                   "Check that a USB camera is connected.")

    col_sel, col_info = st.columns([1, 3])
    with col_sel:
        cam_list_data = None
        try:
            cam_list_data = requests.get(f"{CAMERA_SERVICE}/cameras", timeout=3).json()
        except Exception:
            pass

        available = (cam_list_data or {}).get("cameras", [0])
        selected_idx = st.selectbox(
            "Camera index",
            options=available if available else [0],
            index=0,
            key="cam_idx",
        )
        if st.button("Switch camera", key="cam_switch"):
            try:
                requests.post(
                    f"{CAMERA_SERVICE}/select",
                    json={"index": selected_idx},
                    timeout=3,
                )
                st.success(f"Switched to camera {selected_idx}")
                st.rerun()
            except Exception as e:
                st.error(f"Switch failed: {e}")

    with col_info:
        if cam_info.get("camera_open"):
            w   = cam_info.get("width",  "?")
            h   = cam_info.get("height", "?")
            fps = cam_info.get("fps",    0)
            _c1, _c2, _c3 = st.columns(3)
            _c1.metric("Resolution", f"{w} × {h}")
            _c2.metric("Camera index", cam_info.get("camera_index", 0))
            _c3.metric("FPS (reported)", f"{fps:.0f}")

    _bg = "#1e1e2e" if _dark else "#f5f5f5"
    usb_stream_url = f"{CAMERA_SERVICE}/stream?index={selected_idx}"
    _usb_html = f"""
    <div style="background:{_bg}; padding:12px; border-radius:10px; text-align:center;">
      <img
        src="{usb_stream_url}"
        alt="USB camera feed"
        style="max-width:100%; border-radius:8px; display:block; margin:auto;"
        onerror="this.alt='Stream unavailable — check camera_service.py';"
      />
      <p style="color:#888; font-size:0.75rem; margin-top:6px;">
        Live MJPEG stream &middot; {usb_stream_url}
      </p>
    </div>
    """
    _components.html(_usb_html, height=420, scrolling=False)

    st.markdown("---")

    # ═══════════════════════════════════════════════════════════════════════════
    # WiFi IP Camera section (OCC_cam1 @ 192.168.178.177)
    # ═══════════════════════════════════════════════════════════════════════════
    st.subheader("📡 WiFi IP Camera — OCC_cam1 (192.168.178.177)")

    if ip_cam_info is None:
        st.error("⚠️ Could not reach IP camera endpoint. Make sure `camera_service.py` is running.")
    elif not ip_cam_info.get("connected"):
        st.warning(
            "IP camera is not reachable. Verify the camera is powered on, "
            "connected to the same WiFi network (192.168.178.x), and the RTSP "
            "stream path in `camera_service.py` (`_IP_CAM_URL`) is correct for your camera model."
        )
    else:
        _iw  = ip_cam_info.get("width",  "?")
        _ih  = ip_cam_info.get("height", "?")
        _ifs = ip_cam_info.get("fps",    0)
        _ic1, _ic2, _ic3 = st.columns(3)
        _ic1.metric("Resolution", f"{_iw} × {_ih}")
        _ic2.metric("Camera", ip_cam_info.get("camera", "OCC_cam1"))
        _ic3.metric("FPS (reported)", f"{_ifs:.0f}")
        if ip_cam_info.get("active_url"):
            st.caption(f"Connected via: `{ip_cam_info['active_url']}`")

    ip_stream_url = f"{CAMERA_SERVICE}/ip/stream"
    _ip_html = f"""
    <div style="background:{_bg}; padding:12px; border-radius:10px; text-align:center;">
      <img
        src="{ip_stream_url}"
        alt="WiFi IP camera feed"
        style="max-width:100%; border-radius:8px; display:block; margin:auto;"
        onerror="this.style.display='none'; document.getElementById('ip-cam-err').style.display='block';"
      />
      <div id="ip-cam-err" style="display:none; color:#f87171; padding:40px 20px; font-size:0.9rem;">
        ⚠️ IP camera stream unavailable — check network connection and RTSP path in camera_service.py
      </div>
      <p style="color:#888; font-size:0.75rem; margin-top:6px;">
        Live MJPEG stream &middot; OCC_cam1 &middot; {ip_stream_url}
      </p>
    </div>
    """
    _components.html(_ip_html, height=420, scrolling=False)

    # ── Snapshot section ───────────────────────────────────────────────────────
    st.markdown("---")
    st.subheader("📸 Snapshot")
    _snap_col1, _snap_col2 = st.columns(2)

    with _snap_col1:
        if st.button("Capture USB snapshot", key="cam_snap"):
            try:
                snap      = requests.get(f"{CAMERA_SERVICE}/frame", timeout=5).json()
                img_bytes = _b64.b64decode(snap["frame"])
                ts        = datetime.fromtimestamp(snap["timestamp"]).strftime("%Y-%m-%d %H:%M:%S")
                st.image(img_bytes, caption=f"USB snapshot at {ts}", width='stretch')
            except Exception as e:
                st.error(f"USB snapshot failed: {e}")

    with _snap_col2:
        if st.button("Capture IP camera snapshot", key="ip_cam_snap"):
            try:
                snap      = requests.get(f"{CAMERA_SERVICE}/ip/frame", timeout=5).json()
                img_bytes = _b64.b64decode(snap["frame"])
                ts        = datetime.fromtimestamp(snap["timestamp"]).strftime("%Y-%m-%d %H:%M:%S")
                st.image(img_bytes, caption=f"OCC_cam1 snapshot at {ts}", width='stretch')
            except Exception as e:
                st.error(f"IP camera snapshot failed: {e}")


# ==================== ROBOT PAGE ====================
elif page == "🦾 Robot":

    # ── Service health ──────────────────────────────────────────────────────────
    robot_health = api_get(f"{ROBOT_SERVICE}/")
    if robot_health is None:
        st.error("⚠️ Robot Telemetry Service is offline. Start it with: `python robot_telemetry_service.py`")
        st.stop()

    # ── Connection status chip ──────────────────────────────────────────────────
    conn       = robot_health.get("robot_connection", "unconfigured")
    configured = robot_health.get("configured", False)
    _conn_colors = {
        "connected":    ("svc-ok",  "Connected"),
        "polling":      ("svc-warn","Polling…"),
        "error":        ("svc-err", "Error"),
        "unconfigured": ("svc-err", "Unconfigured"),
    }
    _chip_cls, _chip_lbl = _conn_colors.get(conn, ("svc-err", conn.title()))
    st.markdown(
        f'<span class="svc-chip {_chip_cls}">⬤ Robot {_chip_lbl}</span>',
        unsafe_allow_html=True,
    )

    st.markdown("---")

    col_ctrl, col_telem = st.columns([1, 2], gap="large")

    # ── LEFT: Light control + config ────────────────────────────────────────────
    with col_ctrl:
        st.subheader("💡 Light Control")

        light_data = api_get(f"{ROBOT_SERVICE}/robot/light")
        current_light = (light_data or {}).get("light", "off")
        light_updated = (light_data or {}).get("light_updated_at")

        _light_on = current_light == "on"
        _bulb = "💡" if _light_on else "🔦"
        _light_color = "#d29922" if _light_on else "#666"
        st.markdown(
            f"""<div style="text-align:center; padding:1.2rem 0 0.6rem 0;">
                <div style="font-size:3rem; line-height:1;">{_bulb}</div>
                <div style="font-size:1.1rem; font-weight:700; color:{_light_color};
                            margin-top:6px; letter-spacing:1px;">
                    {"ON" if _light_on else "OFF"}
                </div>
            </div>""",
            unsafe_allow_html=True,
        )
        if light_updated:
            st.caption(f"Last changed: {light_updated[11:19]}")

        _btn_label = "🔦 Turn OFF" if _light_on else "💡 Turn ON"
        _new_state  = "off" if _light_on else "on"
        if st.button(_btn_label, key="light_toggle", width='stretch'):
            try:
                resp = requests.post(
                    f"{ROBOT_SERVICE}/robot/light",
                    json={"state": _new_state},
                    timeout=5,
                )
                result = resp.json()
                if result.get("api_error"):
                    st.warning(
                        f"Light set to **{_new_state}** locally, "
                        f"but robot API error: `{result['api_error']}`"
                    )
                else:
                    st.success(f"Light turned **{_new_state}**.")
                api_get.clear()
                st.rerun()
            except Exception as e:
                st.error(f"Command failed: {e}")

        st.markdown("---")
        st.subheader("⚙️ Robot Credentials")

        status_data = api_get(f"{ROBOT_SERVICE}/robot/status")
        status      = status_data or {}
        cfg_data    = api_get(f"{ROBOT_SERVICE}/robot/config")
        cfg         = cfg_data or {}

        st.caption(
            f"Tenant: **{cfg.get('tenant') or '—'}**  "
            f"· Poll: **{cfg.get('poll_interval_seconds', 5)}s**  "
            f"· SSL verify: **{cfg.get('verify_ssl', True)}**"
        )
        if status.get("error"):
            st.error(f"Last error: `{status['error']}`")

        with st.expander("Configure credentials"):
            with st.form("robot_config_form"):
                f_tenant   = st.text_input("Tenant (subdomain)",   value=cfg.get("tenant", ""))
                f_user     = st.text_input("Username",              value=cfg.get("username", ""))
                f_pass     = st.text_input("Password",              type="password")
                f_apikey   = st.text_input("API Key",               type="password")
                f_interval = st.number_input("Poll interval (s)",   min_value=2, max_value=60,
                                             value=int(cfg.get("poll_interval_seconds", 5)))
                f_ssl      = st.checkbox("Verify SSL", value=cfg.get("verify_ssl", True))
                if st.form_submit_button("💾 Save & Reconnect", type="primary"):
                    payload: dict = {"poll_interval_seconds": f_interval, "verify_ssl": f_ssl}
                    if f_tenant: payload["tenant"]   = f_tenant
                    if f_user:   payload["username"] = f_user
                    if f_pass:   payload["password"] = f_pass
                    if f_apikey: payload["apikey"]   = f_apikey
                    try:
                        r = requests.post(f"{ROBOT_SERVICE}/robot/config",
                                          json=payload, timeout=6)
                        res = r.json()
                        if res.get("connection") == "connected":
                            st.success("Connected!")
                        else:
                            st.warning(f"Status: {res.get('connection')} — {res.get('error') or ''}")
                        api_get.clear()
                        st.rerun()
                    except Exception as e:
                        st.error(f"Config update failed: {e}")

        if st.button("🔄 Poll now", key="robot_poll"):
            try:
                requests.post(f"{ROBOT_SERVICE}/robot/poll", timeout=8)
                api_get.clear()
                st.rerun()
            except Exception as e:
                st.error(f"Poll failed: {e}")

    # ── RIGHT: Telemetry data ────────────────────────────────────────────────────
    with col_telem:
        # ── Position ────────────────────────────────────────────────────────────
        st.subheader("📍 Position")
        pos_data = api_get(f"{ROBOT_SERVICE}/robot/position")
        if pos_data:
            pos   = pos_data.get("position", {})
            trans = pos.get("translation", {})
            rot   = pos.get("rotation",    {})
            p1, p2, p3 = st.columns(3)
            p1.metric("X (m)", f"{trans.get('x', 0):.3f}")
            p2.metric("Y (m)", f"{trans.get('y', 0):.3f}")
            p3.metric("Z (m)", f"{trans.get('z', 0):.3f}")
            r1, r2, r3, r4 = st.columns(4)
            r1.metric("Quat X", f"{rot.get('x', 0):.4f}")
            r2.metric("Quat Y", f"{rot.get('y', 0):.4f}")
            r3.metric("Quat Z", f"{rot.get('z', 0):.4f}")
            r4.metric("Quat W", f"{rot.get('w', 1):.4f}")
            if pos_data.get("last_updated"):
                st.caption(f"Updated: {pos_data['last_updated'][11:19]}")
        elif not configured:
            st.info("Configure robot credentials to see position data.")
        else:
            st.warning("Position data not yet available — trigger a poll.")

        st.markdown("---")

        # ── Diagnostics ─────────────────────────────────────────────────────────
        st.subheader("🔬 Diagnostics")
        diag_data = api_get(f"{ROBOT_SERVICE}/robot/diagnostics")
        if diag_data:
            diags = diag_data.get("diagnostics", [])
            if diag_data.get("last_updated"):
                st.caption(f"Updated: {diag_data['last_updated'][11:19]}  · {len(diags)} components")

            _LEVEL_BADGE = {0: ("✅", "OK",   "#3fb950"),
                            1: ("⚠️", "WARN", "#d29922"),
                            2: ("❌", "ERROR","#f85149")}

            for d in diags:
                level   = d.get("level", 0)
                icon, label, color = _LEVEL_BADGE.get(level, ("❓", str(level), "#888"))
                name    = d.get("name", "Unknown")
                message = d.get("message", "")
                values  = d.get("values", [])

                with st.expander(f"{icon} **{name}** — {message}", expanded=(level > 0)):
                    st.markdown(
                        f'<span style="color:{color}; font-weight:700;">{label}</span>',
                        unsafe_allow_html=True,
                    )
                    if values:
                        val_df = pd.DataFrame(values)
                        if {"key", "value"}.issubset(val_df.columns):
                            val_df = val_df.rename(columns={"key": "Metric", "value": "Value"})
                        st.dataframe(val_df, width='stretch', hide_index=True)
        elif not configured:
            st.info("Configure robot credentials to see diagnostics.")
        else:
            st.warning("Diagnostics not yet available — trigger a poll or check credentials.")

    # ── Auto-refresh ────────────────────────────────────────────────────────────
    if configured and conn == "connected":
        time.sleep(5)
        st.rerun()


st.markdown("---")
st.caption("🎯 Operations Control Center  ·  Magdeburg Autonomous Logistics Hub  ·  Built with Streamlit + Microservices")
