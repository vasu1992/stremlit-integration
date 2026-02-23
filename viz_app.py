"""
viz_app.py  –  NiceGUI Real-Time Visualization  (port 8008)

Run:  python viz_app.py
Deps: pip install nicegui plotly requests
"""
from __future__ import annotations

import math
from datetime import datetime

import plotly.graph_objects as go
import requests
from nicegui import ui

# ─── Service URLs ─────────────────────────────────────────────────────────────
VEHICLE_SERVICE     = "http://localhost:8004"
LIDAR_SERVICE       = "http://localhost:8005"
DUMMY_LIDAR_SERVICE = "http://localhost:8006"
TELEOP_SERVICE      = "http://localhost:8007"

_STATUS_COLORS = {
    "Active":   "#00B400",
    "Idle":     "#FFA500",
    "Charging": "#4B6BFF",
    "Error":    "#FF4B4B",
}
_TELEOP_BORDER = {
    "Idle":     "#334455",
    "Moving":   "#00bb44",
    "Rotating": "#4488ff",
    "E-Stop":   "#dd2222",
}


# ─── API helper ───────────────────────────────────────────────────────────────
def api_get(url: str):
    try:
        r = requests.get(url, timeout=2)
        r.raise_for_status()
        return r.json()
    except Exception:
        return None


# ─── SVG camera tile ──────────────────────────────────────────────────────────
def cam_svg(label: str, heading_deg: float, speed: float, status: str) -> str:
    W, H, cx, cy = 400, 240, 200, 110
    border = _TELEOP_BORDER.get(status, "#334455")
    hud    = "#00cc44"
    warn   = "#ff4444" if status == "E-Stop" else hud
    lid    = label.replace(" ", "_")

    lines = ""
    for i in range(8):
        s = (i + 1) * 28
        lines += (
            f'<line x1="{cx}" y1="{cy}" x2="{max(0, cx-s)}" y2="{H}" stroke="#162616" stroke-width="1"/>'
            f'<line x1="{cx}" y1="{cy}" x2="{min(W, cx+s)}" y2="{H}" stroke="#162616" stroke-width="1"/>'
        )
    for i in range(5):
        t  = (i + 1) / 6.0
        gy = cy + (H - cy) * t
        xl = cx - cx * (t ** 0.6)
        xr = cx + (W - cx) * (t ** 0.6)
        lines += (
            f'<line x1="{xl:.0f}" y1="{gy:.0f}" x2="{xr:.0f}" y2="{gy:.0f}" '
            f'stroke="#162616" stroke-width="1"/>'
        )

    return (
        f'<svg viewBox="0 0 {W} {H}" xmlns="http://www.w3.org/2000/svg" '
        f'style="width:100%;border-radius:10px;border:2px solid {border};display:block;">'
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
        f'<line x1="{cx-24}" y1="{cy}" x2="{cx+24}" y2="{cy}" stroke="{hud}" stroke-width="1.5" opacity=".85"/>'
        f'<line x1="{cx}" y1="{cy-24}" x2="{cx}" y2="{cy+24}" stroke="{hud}" stroke-width="1.5" opacity=".85"/>'
        f'<circle cx="{cx}" cy="{cy}" r="10" fill="none" stroke="{hud}" stroke-width="1.5" opacity=".55"/>'
        f'<rect x="8" y="8" width="136" height="24" fill="rgba(0,0,0,.7)" rx="4"/>'
        f'<text x="14" y="25" fill="{hud}" font-family="monospace" font-size="13" font-weight="bold">'
        f'&#x1F4F7; {label}</text>'
        f'<rect x="{W-60}" y="8" width="52" height="24" fill="rgba(0,0,0,.7)" rx="4"/>'
        f'<text x="{W-54}" y="25" fill="{hud}" font-family="monospace" font-size="12">{heading_deg:.0f}&#xb0;</text>'
        f'<rect x="8" y="{H-28}" width="110" height="22" fill="rgba(0,0,0,.7)" rx="4"/>'
        f'<text x="14" y="{H-13}" fill="{hud}" font-family="monospace" font-size="11">{abs(speed):.2f} m/s</text>'
        f'<rect x="{W-102}" y="{H-28}" width="94" height="22" fill="rgba(0,0,0,.7)" rx="4"/>'
        f'<text x="{W-96}" y="{H-13}" fill="{warn}" font-family="monospace" font-size="11">{status.upper()}</text>'
        f'</svg>'
    )


# ─── Plotly figure builders ───────────────────────────────────────────────────
def _fleet_fig(vehicles: list) -> go.Figure:
    fig = go.Figure()
    for v in vehicles:
        color = _STATUS_COLORS.get(v.get("status"), "#B4B4B4")
        teleop_marker = " 🕹️" if v.get("teleop_active") else ""
        fig.add_trace(go.Scattermap(
            lat=[v["latitude"]],
            lon=[v["longitude"]],
            mode="markers+text",
            marker=dict(size=20, color=color, opacity=0.9),
            text=[v["name"] + teleop_marker],
            textfont=dict(color="#ffffff", size=11),
            textposition="top right",
            hovertemplate=(
                f"<b>{v['name']}</b>{teleop_marker}<br>"
                f"ID: {v['id']}<br>"
                f"Status: {v['status']}<br>"
                f"Battery: {v['battery']}%<br>"
                f"Speed: {v['speed']} km/h<br>"
                f"Sensor: {v.get('sensor_health', '—')}<br>"
                f"Lat: {v['latitude']:.6f}<br>"
                f"Lon: {v['longitude']:.6f}"
                f"<extra></extra>"
            ),
            name=v["id"],
        ))
    fig.update_layout(
        map=dict(
            style="open-street-map",
            center=dict(lat=52.1205, lon=11.6276),
            zoom=14,
        ),
        margin=dict(l=0, r=0, t=0, b=0),
        showlegend=True,
        legend=dict(
            bgcolor="rgba(13,17,23,0.85)",
            bordercolor="#16243a",
            borderwidth=1,
            font=dict(color="#c9d1d9", size=11),
        ),
        paper_bgcolor="#0d1117",
        uirevision="fleet",   # preserves zoom/pan between updates
    )
    return fig


def _trail_fig(state: dict, path: list) -> go.Figure:
    fig = go.Figure()

    if path:
        xs = [p["x"] for p in path]
        ys = [p["y"] for p in path]
        fig.add_trace(go.Scatter(
            x=xs, y=ys,
            mode="lines",
            line=dict(color="#4B6BFF", width=2),
            opacity=0.6,
            name="Trail",
            showlegend=False,
        ))
        fig.add_trace(go.Scatter(
            x=xs, y=ys,
            mode="markers",
            marker=dict(size=4, color="#4B6BFF", opacity=0.4),
            showlegend=False,
        ))

    # Origin
    fig.add_trace(go.Scatter(
        x=[0], y=[0],
        mode="markers",
        marker=dict(size=12, color="#888888", symbol="diamond"),
        name="Origin",
    ))

    # Robot current position
    fig.add_trace(go.Scatter(
        x=[state["x"]], y=[state["y"]],
        mode="markers",
        marker=dict(size=16, color="#FF4B4B", symbol="circle"),
        name="Robot",
        hovertemplate=(
            f"x: {state['x']:.3f} m<br>"
            f"y: {state['y']:.3f} m<br>"
            f"hdg: {state['heading']:.1f}°<extra></extra>"
        ),
    ))

    # Heading arrow
    hr = math.radians(state["heading"])
    fig.add_annotation(
        x=state["x"] + 0.6 * math.cos(hr),
        y=state["y"] + 0.6 * math.sin(hr),
        ax=state["x"], ay=state["y"],
        xref="x", yref="y", axref="x", ayref="y",
        arrowhead=3, arrowsize=1.5,
        arrowcolor="#FF4B4B", arrowwidth=2.5,
        showarrow=True,
    )

    n_pts = len(path) if path else 0
    fig.update_layout(
        paper_bgcolor="#161b22",
        plot_bgcolor="#0d1117",
        font=dict(color="#c9d1d9", size=11),
        xaxis=dict(title="X (m)", gridcolor="#1c2a3a", zerolinecolor="#264a6a"),
        yaxis=dict(
            title="Y (m)", gridcolor="#1c2a3a",
            zerolinecolor="#264a6a", scaleanchor="x",
        ),
        margin=dict(l=48, r=20, t=44, b=44),
        legend=dict(bgcolor="rgba(0,0,0,0.3)", font=dict(color="#c9d1d9")),
        title=dict(
            text=f"Robot Trail — {n_pts} waypoints · {state['total_distance']:.2f} m total",
            font=dict(size=12, color="#c9d1d9"),
            x=0.02, xanchor="left",
        ),
        uirevision="trail",
    )
    return fig


def _lidar_fig(scan: dict) -> go.Figure:
    pts = scan.get("points", [])
    xs  = [p["x"]     for p in pts]
    ys  = [p["y"]     for p in pts]
    rs  = [p["range"] for p in pts]

    fig = go.Figure()

    # Point cloud
    fig.add_trace(go.Scatter(
        x=xs, y=ys,
        mode="markers",
        marker=dict(
            size=4, color=rs,
            colorscale="RdYlGn",
            reversescale=True,
            colorbar=dict(
                title=dict(text="Range (m)", font=dict(color="#c9d1d9")),
                thickness=14,
                tickfont=dict(color="#c9d1d9"),
                bgcolor="rgba(0,0,0,0.3)",
            ),
            opacity=0.88,
        ),
        hovertemplate="x: %{x:.3f} m<br>y: %{y:.3f} m<br>range: %{marker.color:.3f} m<extra></extra>",
        name="Points",
        showlegend=False,
    ))

    # Robot origin
    fig.add_trace(go.Scatter(
        x=[0], y=[0],
        mode="markers",
        marker=dict(size=16, color="#00FFFF", symbol="triangle-up"),
        name="Robot",
    ))

    # Room boundary (simulator only)
    if scan.get("room"):
        rw = scan["room"]["width"]  / 2
        rh = scan["room"]["height"] / 2
        bx = [-rw,  rw,  rw, -rw, -rw]
        by = [-rh, -rh,  rh,  rh, -rh]
        fig.add_trace(go.Scatter(
            x=bx, y=by,
            mode="lines",
            line=dict(color="#555555", dash="dash", width=1.5),
            name="Room",
            showlegend=False,
        ))

    # Obstacle centres (simulator only)
    if scan.get("obstacles"):
        obs = scan["obstacles"]
        fig.add_trace(go.Scatter(
            x=[o["x"] for o in obs],
            y=[o["y"] for o in obs],
            mode="markers",
            marker=dict(size=18, color="#FF6B35", opacity=0.75, symbol="circle"),
            name="Obstacles",
            hovertemplate="Obstacle<br>x: %{x:.2f} m<br>y: %{y:.2f} m<extra></extra>",
        ))

    fig.update_layout(
        paper_bgcolor="#0d1117",
        plot_bgcolor="#0d1117",
        font=dict(color="#c9d1d9", size=11),
        xaxis=dict(
            title="X (m)", gridcolor="#1a2d1a",
            zerolinecolor="#264a26", scaleanchor="y",
        ),
        yaxis=dict(
            title="Y (m)", gridcolor="#1a2d1a",
            zerolinecolor="#264a26",
        ),
        margin=dict(l=48, r=72, t=44, b=44),
        legend=dict(bgcolor="rgba(0,0,0,0.3)", font=dict(color="#c9d1d9")),
        title=dict(
            text=f"LiDAR Scan — {scan['point_count']} points",
            font=dict(size=13, color="#c9d1d9"),
            x=0.02, xanchor="left",
        ),
        uirevision="lidar",
    )
    return fig


# ─── Page ─────────────────────────────────────────────────────────────────────
@ui.page("/")
def main_page() -> None:
    ui.add_head_html("""
    <style>
        body, .q-page { background: #0d1117 !important; }
        .q-tab__label { font-size: 0.88rem !important; font-weight: 600 !important; }
        .q-tab-panels { background: #0d1117 !important; }
        .q-panel      { background: #0d1117 !important; }
    </style>
    """)

    # ── Header ────────────────────────────────────────────────────────────────
    with ui.header(elevated=True).classes(
        "bg-[#05080f] text-white items-center gap-3 px-6 py-2 border-b border-[#16243a]"
    ):
        ui.label("🎯").classes("text-3xl leading-none")
        with ui.column().classes("gap-0 leading-tight"):
            ui.label("Operations Control Center").classes(
                "text-base font-bold tracking-wide text-[#f0f6fc]"
            )
            ui.label(
                "Real-Time Visualization · Magdeburg Autonomous Logistics Hub"
            ).classes("text-[0.72rem] text-[#4a6a8a]")
        ui.space()
        clock_lbl = ui.label("").classes("text-[0.72rem] font-mono text-[#4a6a8a]")
        ui.timer(1.0, lambda: clock_lbl.set_text(datetime.now().strftime("%H:%M:%S")))

    # ── Tabs ──────────────────────────────────────────────────────────────────
    with ui.tabs().props("dense").classes(
        "bg-[#0a1020] text-[#79c0ff] border-b border-[#16243a] w-full"
    ) as tabs:
        t_fleet  = ui.tab("fleet",  label="🗺️  Fleet Map")
        t_teleop = ui.tab("teleop", label="🕹️  Teleop")
        t_lidar  = ui.tab("lidar",  label="📡  LiDAR")

    with ui.tab_panels(tabs, value=t_fleet).classes("w-full bg-[#0d1117] flex-1"):

        # ════════════════════════ FLEET MAP ══════════════════════════════════
        with ui.tab_panel(t_fleet).classes("p-4"):

            # KPI chips
            with ui.row().classes("gap-4 mb-3 flex-wrap items-center"):
                kpi_total  = ui.label("Total: —").classes("text-sm font-mono text-[#c9d1d9]")
                kpi_active = ui.label("🟢 Active: —").classes("text-sm font-mono")
                kpi_idle   = ui.label("🟡 Idle: —").classes("text-sm font-mono")
                kpi_charge = ui.label("🔵 Charging: —").classes("text-sm font-mono")
                kpi_bat    = ui.label("🔋 Avg Battery: — %").classes("text-sm font-mono text-[#c9d1d9]")
                kpi_speed  = ui.label("⚡ Avg Speed: — km/h").classes("text-sm font-mono text-[#c9d1d9]")
                kpi_health = ui.label("💚 Fleet Health: —").classes("text-sm font-mono text-[#c9d1d9]")

            fleet_plot = ui.plotly(go.Figure()).classes("w-full").style("height:560px")

            def update_fleet() -> None:
                vdata    = api_get(f"{VEHICLE_SERVICE}/fleet/vehicles")
                kpi_data = api_get(f"{VEHICLE_SERVICE}/fleet/kpis")
                vehicles = (vdata or {}).get("vehicles", [])

                if kpi_data:
                    kpi_total.set_text(f"Total: {kpi_data['total_vehicles']}")
                    kpi_active.set_text(f"🟢 Active: {kpi_data['active']}")
                    kpi_idle.set_text(f"🟡 Idle: {kpi_data['idle']}")
                    kpi_charge.set_text(f"🔵 Charging: {kpi_data['charging']}")
                    kpi_bat.set_text(f"🔋 Avg Battery: {kpi_data['avg_battery_pct']} %")
                    kpi_speed.set_text(f"⚡ Avg Speed: {kpi_data['avg_speed_kmh']} km/h")
                    kpi_health.set_text(f"💚 Fleet Health: {kpi_data['fleet_health']}")

                if vehicles:
                    fleet_plot.figure = _fleet_fig(vehicles)
                    fleet_plot.update()

            ui.timer(3.0, update_fleet)
            update_fleet()

        # ════════════════════════ TELEOP ═════════════════════════════════════
        with ui.tab_panel(t_teleop).classes("p-4"):

            # Status bar
            with ui.row().classes("gap-6 mb-3 items-center flex-wrap"):
                tl_status  = ui.label("Status: —").classes("text-sm font-mono text-[#c9d1d9]")
                tl_pos     = ui.label("Pos: (—, —) m").classes("text-sm font-mono text-[#c9d1d9]")
                tl_heading = ui.label("Hdg: —°").classes("text-sm font-mono text-[#c9d1d9]")
                tl_battery = ui.label("Bat: —%").classes("text-sm font-mono text-[#c9d1d9]")
                tl_dist    = ui.label("Dist: — m").classes("text-sm font-mono text-[#c9d1d9]")
                tl_fleet   = ui.label("Fleet: 🔌 Standalone").classes(
                    "text-sm font-mono text-[#4a6a8a]"
                )

            # 2×2 camera grid
            _PLACEHOLDER = (
                '<div style="background:#080c10;border:2px solid #334455;border-radius:10px;'
                'height:160px;display:flex;align-items:center;justify-content:center;'
                'color:#334455;font-family:monospace;font-size:13px;">📷 Waiting…</div>'
            )
            with ui.grid(columns=2).classes("w-full gap-2 mb-3"):
                cam_els = {lbl: ui.html(_PLACEHOLDER).classes("w-full") for lbl in ("FRONT", "RIGHT", "LEFT", "REAR")}

            # Trail chart
            trail_plot = ui.plotly(go.Figure()).classes("w-full").style("height:380px")

            def update_teleop() -> None:
                state     = api_get(f"{TELEOP_SERVICE}/robot/state")
                path_data = api_get(f"{TELEOP_SERVICE}/robot/path")
                conn_data = api_get(f"{TELEOP_SERVICE}/robot/connection") or {}

                if not state:
                    return

                h  = state["heading"]
                sp = state["speed"]
                st = state["status"]

                _DOT = {"Idle": "🟡", "Moving": "🟢", "Rotating": "🔵", "E-Stop": "🔴"}
                tl_status.set_text(f"Status: {_DOT.get(st, '⚪')} {st}")
                tl_pos.set_text(f"Pos: ({state['x']:.2f}, {state['y']:.2f}) m")
                tl_heading.set_text(f"Hdg: {h:.1f}°")
                tl_battery.set_text(f"Bat: {state['battery']:.1f}%")
                tl_dist.set_text(f"Dist: {state['total_distance']:.2f} m")

                if conn_data.get("vehicle_id"):
                    tl_fleet.set_text(
                        f"Fleet: 🔗 {conn_data.get('vehicle_name', conn_data['vehicle_id'])}"
                    )
                else:
                    tl_fleet.set_text("Fleet: 🔌 Standalone")

                # Camera feeds
                offsets = {"FRONT": 0, "RIGHT": -90, "LEFT": 90, "REAR": 180}
                for lbl, offset in offsets.items():
                    cam_els[lbl].set_content(cam_svg(lbl, (h + offset) % 360, sp, st))

                # Trail chart
                path = (path_data or {}).get("path", [])
                trail_plot.figure = _trail_fig(state, path)
                trail_plot.update()

            ui.timer(1.5, update_teleop)
            update_teleop()

        # ════════════════════════ LIDAR ═══════════════════════════════════════
        with ui.tab_panel(t_lidar).classes("p-4"):

            # Toolbar
            with ui.row().classes("gap-4 mb-3 items-center flex-wrap"):
                lidar_src = ui.toggle(
                    {"sim": "🤖 LiDAR Simulator", "bridge": "📡 LiDAR Bridge"},
                    value="sim",
                ).props("no-caps dense")
                ld_pts   = ui.label("Points: —").classes("text-sm font-mono text-[#c9d1d9]")
                ld_range = ui.label("Range: —").classes("text-sm font-mono text-[#c9d1d9]")
                ld_rate  = ui.label("Rate: —").classes("text-sm font-mono text-[#c9d1d9]")
                ld_scans = ui.label("Scans: —").classes("text-sm font-mono text-[#c9d1d9]")

            lidar_plot = ui.plotly(go.Figure()).classes("w-full").style("height:560px")

            def update_lidar() -> None:
                svc    = DUMMY_LIDAR_SERVICE if lidar_src.value == "sim" else LIDAR_SERVICE
                scan   = api_get(f"{svc}/scan/latest")
                status = api_get(f"{svc}/scan/status")

                if scan and scan.get("points"):
                    ld_pts.set_text(f"Points: {scan['point_count']}")
                    ld_range.set_text(
                        f"Range: {scan['range_min_measured']}–{scan['range_max_measured']} m"
                    )
                if status:
                    ld_rate.set_text(f"Rate: {status['scan_rate_hz']} Hz")
                    ld_scans.set_text(f"Scans: {status['scan_count']}")

                if scan and scan.get("points"):
                    lidar_plot.figure = _lidar_fig(scan)
                    lidar_plot.update()

            ui.timer(1.5, update_lidar)
            update_lidar()


# ─── Entry point ──────────────────────────────────────────────────────────────
if __name__ in {"__main__", "__mp_main__"}:
    ui.run(
        port=8008,
        title="OCC Visualization",
        dark=True,
        favicon="🎯",
        show=False,
        reload=False,
    )
