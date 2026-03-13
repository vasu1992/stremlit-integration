"""
MQTT Traffic Light Dashboard  –  Port 8511
==========================================
Standalone Streamlit dashboard that displays live data received
from the MQTT Traffic Light Service (port 8009).

Run with:
    streamlit run traffic_dashboard.py --server.port 8511
"""

import time
import json

import pandas as pd
import altair as alt
import requests
import streamlit as st
from datetime import datetime

TRAFFIC_SERVICE = "http://localhost:8009"

st.set_page_config(
    page_title="MQTT Traffic Light Dashboard",
    page_icon="🚦",
    layout="wide",
    initial_sidebar_state="collapsed",
)

# ─── CSS ───────────────────────────────────────────────────────────────────────
st.markdown("""
<style>
.block-container { padding-top: 1.2rem !important; max-width: 1400px !important; }
[data-testid="stMetricValue"] { font-size: 1.6rem !important; font-weight: 700 !important; }
.tl-card {
    border-radius: 12px; padding: 14px; margin-bottom: 6px;
    border: 1px solid;
}
.badge {
    display: inline-block; padding: 2px 10px; border-radius: 20px;
    font-size: 0.75rem; font-weight: 700; letter-spacing: 0.8px;
}
.badge-green  { background: rgba(35,134,54,0.15);  border: 1px solid rgba(35,134,54,0.4);  color: #3fb950; }
.badge-yellow { background: rgba(210,153,34,0.15); border: 1px solid rgba(210,153,34,0.4); color: #e3b341; }
.badge-red    { background: rgba(218,54,51,0.15);  border: 1px solid rgba(218,54,51,0.4);  color: #f85149; }
.badge-ok   { background: rgba(63,185,80,0.1); border:1px solid rgba(63,185,80,0.3); color:#3fb950; }
.badge-off  { background: rgba(248,81,73,0.1); border:1px solid rgba(248,81,73,0.3); color:#f85149; }
</style>
""", unsafe_allow_html=True)

# ─── Header ────────────────────────────────────────────────────────────────────
st.markdown("""
<div style="border-left:4px solid #388bfd; border-radius:0 12px 12px 0;
            padding:0.9rem 1.4rem; margin-bottom:1rem;
            background:linear-gradient(135deg,#090e18,#0e1929,#090e18);">
    <span style="font-size:1.7rem;">🚦</span>
    <span style="font-size:1.25rem; font-weight:800; color:#f0f6fc; margin-left:10px;">
        MQTT Traffic Light Dashboard
    </span>
    <span style="font-size:0.78rem; color:#6a8099; margin-left:14px;">
        Live data from service.ifak.eu · iot_board/lsa/leitstelle
    </span>
</div>
""", unsafe_allow_html=True)


# ─── API helper ────────────────────────────────────────────────────────────────
def api_get(path: str) -> dict | None:
    try:
        r = requests.get(f"{TRAFFIC_SERVICE}{path}", timeout=2)
        r.raise_for_status()
        return r.json()
    except Exception:
        return None


# ─── Service check ─────────────────────────────────────────────────────────────
health = api_get("/")
if health is None:
    st.error("⚠️ MQTT Traffic Light Service is offline.")
    st.code("python mqtt_traffic_service.py", language="bash")
    st.stop()

mqtt = health.get("mqtt", {})

# ─── Connection status bar ─────────────────────────────────────────────────────
connected = mqtt.get("connected", False)
broker    = mqtt.get("broker", "—")
port_num  = mqtt.get("port", "—")
topic     = mqtt.get("topic", "—")
msgs_rx   = mqtt.get("messages_received", 0)

conn_cls = "badge-ok" if connected else "badge-off"
conn_lbl = "● CONNECTED" if connected else "● DISCONNECTED"

c_status, c_info, c_refresh = st.columns([1, 4, 1])

with c_status:
    st.markdown(
        f'<span class="badge {conn_cls}" style="font-size:0.8rem;padding:5px 14px;">'
        f'{conn_lbl}</span>',
        unsafe_allow_html=True,
    )

with c_info:
    st.caption(
        f"Broker: **{broker}:{port_num}**  ·  Topic: **{topic}**  ·  "
        f"Messages received: **{msgs_rx}**  ·  "
        f"Lights tracked: **{health.get('lights_tracked', 0)}**  ·  "
        f"Last update: **{datetime.now().strftime('%H:%M:%S')}**"
    )

with c_refresh:
    if st.button("🔄 Refresh", key="refresh_btn"):
        st.rerun()

st.markdown("---")

# ─── Fetch data ────────────────────────────────────────────────────────────────
lights_data  = api_get("/traffic/lights")
summary_data = api_get("/traffic/summary")
history_data = api_get("/traffic/history?limit=200")
raw_data     = api_get("/traffic/raw?limit=20")

lights  = (lights_data  or {}).get("lights", [])
summary = summary_data or {}
events  = (history_data or {}).get("events", [])
raw_msgs = (raw_data or {}).get("messages", [])

# ─── KPI metrics ───────────────────────────────────────────────────────────────
by_state = summary.get("by_state", {"GREEN": 0, "YELLOW": 0, "RED": 0})

m1, m2, m3, m4, m5, m6 = st.columns(6)
m1.metric("Total Lights",    summary.get("total",   len(lights)))
m2.metric("🟢 Green",        by_state.get("GREEN",  0))
m3.metric("🟡 Yellow",       by_state.get("YELLOW", 0))
m4.metric("🔴 Red",          by_state.get("RED",    0))
m5.metric("⚠️ Faults",       summary.get("faults",  0))
m6.metric("✅ Healthy",      summary.get("healthy", 0))

st.markdown("---")

if not lights and not raw_msgs:
    st.info(
        "No data received yet — waiting for MQTT messages on "
        f"`{topic}` from `{broker}`."
    )
else:
    # ─── Live light state grid ─────────────────────────────────────────────────
    if lights:
        st.subheader("Live Traffic Light States")

        _COLOR  = {"GREEN": "#3fb950", "YELLOW": "#e3b341", "RED": "#f85149"}
        _BG     = {
            "GREEN":  "rgba(35,134,54,0.10)",
            "YELLOW": "rgba(210,153,34,0.10)",
            "RED":    "rgba(218,54,51,0.10)",
        }
        _BORDER = {
            "GREEN":  "rgba(35,134,54,0.35)",
            "YELLOW": "rgba(210,153,34,0.35)",
            "RED":    "rgba(218,54,51,0.35)",
        }

        COLS = 4
        for row in [lights[i:i+COLS] for i in range(0, len(lights), COLS)]:
            cols = st.columns(COLS, gap="small")
            for col, lt in zip(cols, row):
                state  = lt.get("state", "RED")
                phase  = lt.get("phase", "normal")
                tis    = lt.get("time_in_state_s", 0)
                cycle  = max(lt.get("cycle_s", 60), 1)
                pct    = min(100, int(tis / cycle * 100))
                color  = _COLOR.get(state, "#888")
                bg     = _BG.get(state, "rgba(128,128,128,0.1)")
                border = _BORDER.get(state, "rgba(128,128,128,0.3)")
                ts_raw = lt.get("timestamp", "")
                ts     = ts_raw[11:19] if len(ts_raw) >= 19 else ts_raw
                fault_badge = ' <span style="color:#f85149;font-size:0.68rem;">⚠ FAULT</span>' \
                              if phase == "fault" else ""

                col.markdown(f"""
                <div style="background:{bg};border:1px solid {border};
                            border-radius:12px;padding:14px 14px 10px;">
                  <div style="display:flex;justify-content:space-between;
                              align-items:center;margin-bottom:4px;">
                    <span style="font-size:0.82rem;font-weight:700;
                                 color:{color};letter-spacing:1px;">{state}</span>
                    <span style="font-size:0.7rem;color:#888;">{lt.get('id','')}</span>
                  </div>
                  <div style="font-size:0.72rem;color:#aaa;margin-bottom:8px;
                              line-height:1.4;">{lt.get('intersection','Unknown')}</div>
                  <div style="background:#1e1e1e;border-radius:4px;
                              height:5px;margin-bottom:6px;">
                    <div style="background:{color};width:{pct}%;
                                height:5px;border-radius:4px;"></div>
                  </div>
                  <div style="display:flex;justify-content:space-between;
                              font-size:0.68rem;color:#777;">
                    <span>{tis}s / {cycle}s</span>
                    <span>{fault_badge or ts}</span>
                  </div>
                </div>
                """, unsafe_allow_html=True)

        st.markdown("")

    # ─── Two-column layout: raw messages + history ─────────────────────────────
    left, right = st.columns([1, 2], gap="large")

    # ── Raw MQTT payloads ──────────────────────────────────────────────────────
    with left:
        st.subheader("Raw MQTT Messages")
        st.caption(f"Last {len(raw_msgs)} messages received from broker")
        if raw_msgs:
            for i, msg in enumerate(reversed(raw_msgs)):
                with st.expander(f"Message {len(raw_msgs) - i}", expanded=(i == 0)):
                    st.json(msg)
        else:
            st.info("No messages yet.")

    # ── State-change timeline chart ────────────────────────────────────────────
    with right:
        st.subheader("State-Change Timeline")
        if events:
            ev_df = pd.DataFrame(events)
            ev_df["ts"] = pd.to_datetime(ev_df.get("event_time", ev_df.get("timestamp")),
                                          errors="coerce")
            ev_df = ev_df.dropna(subset=["ts"]).sort_values("ts")
            ev_df["minute"] = ev_df["ts"].dt.floor("1min")

            bucket = (
                ev_df.groupby(["minute", "state"])
                .size()
                .reset_index(name="count")
            )
            bucket["minute_str"] = bucket["minute"].dt.strftime("%H:%M")

            scheme = alt.Scale(
                domain=["GREEN", "YELLOW", "RED"],
                range=["#3fb950", "#e3b341", "#f85149"],
            )
            chart = (
                alt.Chart(bucket)
                .mark_bar()
                .encode(
                    x=alt.X("minute_str:O", title="Time (1-min bucket)",
                             axis=alt.Axis(labelAngle=-45)),
                    y=alt.Y("count:Q", title="Events"),
                    color=alt.Color("state:N", scale=scheme,
                                    legend=alt.Legend(title="State")),
                    tooltip=["minute_str:O", "state:N", "count:Q"],
                )
                .properties(width="container", height=240)
            )
            st.altair_chart(chart, width='stretch')
        else:
            st.info("No events recorded yet.")

    # ─── Recent events table ───────────────────────────────────────────────────
    st.subheader("Recent Event Log")
    if events:
        ev_df = pd.DataFrame(events)
        ts_col = "event_time" if "event_time" in ev_df.columns else "timestamp"
        keep = [c for c in [ts_col, "id", "intersection", "state", "phase",
                             "time_in_state_s", "topic"] if c in ev_df.columns]
        disp = ev_df[keep].sort_values(ts_col, ascending=False).head(50).copy()
        disp.columns = [c.replace("_", " ").title() for c in disp.columns]
        st.dataframe(disp, width='stretch', hide_index=True)
    else:
        st.info("No events yet.")

# ─── Auto-refresh ──────────────────────────────────────────────────────────────
st.markdown("---")
st.caption(
    "🚦 MQTT Traffic Light Dashboard  ·  Subscribe-only  ·  "
    f"service.ifak.eu:1883  ·  Auto-refreshing every 3 s"
)
time.sleep(3)
st.rerun()
