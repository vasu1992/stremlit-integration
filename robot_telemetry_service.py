"""
Robot Telemetry Service
=======================
Fetches telemetry data from the Innok Robotics cloud API and exposes it
via a local FastAPI service for other dashboard components to consume.

Innok API endpoints used:
  GET /state/diagnostics  – hardware diagnostics (level, name, message, values)
  GET /state/position     – pose in a reference frame (translation + rotation)

Credentials are loaded from environment variables (or set at runtime via
POST /robot/config):
  INNOK_TENANT    – subdomain, e.g. "myrobot"
  INNOK_USER      – HTTP Basic auth username
  INNOK_PASSWORD  – HTTP Basic auth password
  INNOK_APIKEY    – ?apikey= query parameter

Run:
  python robot_telemetry_service.py
  # or
  uvicorn robot_telemetry_service:app --port 8009 --reload
"""

import asyncio
import os
from datetime import datetime
from typing import Any

import httpx
from fastapi import FastAPI, HTTPException, BackgroundTasks
from pydantic import BaseModel
import uvicorn

app = FastAPI(title="Robot Telemetry Service", version="1.0.0")

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

class RobotConfig(BaseModel):
    tenant: str = ""
    username: str = ""
    password: str = ""
    apikey: str = ""
    poll_interval_seconds: int = 5
    verify_ssl: bool = True


# Runtime config – pre-loaded from env vars, overridable via POST /robot/config
_config = RobotConfig(
    tenant=os.getenv("INNOK_TENANT", ""),
    username=os.getenv("INNOK_USER", ""),
    password=os.getenv("INNOK_PASSWORD", ""),
    apikey=os.getenv("INNOK_APIKEY", ""),
    poll_interval_seconds=int(os.getenv("INNOK_POLL_INTERVAL", "5")),
    verify_ssl=os.getenv("INNOK_VERIFY_SSL", "true").lower() != "false",
)

# ---------------------------------------------------------------------------
# Cached state
# ---------------------------------------------------------------------------

_state: dict[str, Any] = {
    "connection": "unconfigured",   # unconfigured | polling | connected | error
    "last_poll_at": None,
    "error": None,
    "diagnostics": [],
    "position": None,
    "light": "off",                 # off | on  — tracks last commanded state
    "light_updated_at": None,
}


def _base_url() -> str:
    return f"https://{_config.tenant}.cloud.innok-robotics.de/api/v1"


def _is_configured() -> bool:
    return bool(_config.tenant and _config.username and _config.password)


# ---------------------------------------------------------------------------
# Polling logic
# ---------------------------------------------------------------------------

async def _fetch_diagnostics(client: httpx.AsyncClient) -> list[dict]:
    url = f"{_base_url()}/state/diagnostics"
    params = {"apikey": _config.apikey} if _config.apikey else {}
    resp = await client.get(
        url,
        params=params,
        auth=(_config.username, _config.password),
    )
    resp.raise_for_status()
    return resp.json()


async def _fetch_position(client: httpx.AsyncClient) -> dict:
    url = f"{_base_url()}/state/position"
    params = {"apikey": _config.apikey} if _config.apikey else {}
    resp = await client.get(
        url,
        params=params,
        auth=(_config.username, _config.password),
    )
    resp.raise_for_status()
    return resp.json()


async def _poll_once() -> None:
    """Fetch diagnostics and position; update _state in-place."""
    if not _is_configured():
        _state["connection"] = "unconfigured"
        return

    _state["connection"] = "polling"
    try:
        async with httpx.AsyncClient(verify=_config.verify_ssl, timeout=10.0) as client:
            diagnostics, position = await asyncio.gather(
                _fetch_diagnostics(client),
                _fetch_position(client),
            )
        _state["diagnostics"] = diagnostics
        _state["position"] = position
        _state["connection"] = "connected"
        _state["error"] = None
        _state["last_poll_at"] = datetime.now().isoformat()
    except httpx.HTTPStatusError as exc:
        _state["connection"] = "error"
        _state["error"] = f"HTTP {exc.response.status_code}: {exc.response.text[:200]}"
        _state["last_poll_at"] = datetime.now().isoformat()
    except Exception as exc:
        _state["connection"] = "error"
        _state["error"] = str(exc)
        _state["last_poll_at"] = datetime.now().isoformat()


async def _polling_loop() -> None:
    """Background task that polls the robot API on a fixed interval."""
    while True:
        await _poll_once()
        await asyncio.sleep(_config.poll_interval_seconds)


# ---------------------------------------------------------------------------
# Startup / shutdown
# ---------------------------------------------------------------------------

@app.on_event("startup")
async def startup_event():
    asyncio.create_task(_polling_loop())


# ---------------------------------------------------------------------------
# Endpoints
# ---------------------------------------------------------------------------

@app.get("/")
def root():
    return {
        "service": "Robot Telemetry Service",
        "status": "running",
        "robot_connection": _state["connection"],
        "configured": _is_configured(),
        "last_poll_at": _state["last_poll_at"],
    }


@app.get("/robot/status")
def get_status():
    """Overall connection status and last poll timestamp."""
    return {
        "connection": _state["connection"],
        "configured": _is_configured(),
        "last_poll_at": _state["last_poll_at"],
        "error": _state["error"],
        "tenant": _config.tenant or None,
        "poll_interval_seconds": _config.poll_interval_seconds,
    }


@app.get("/robot/diagnostics")
def get_diagnostics():
    """
    Latest diagnostics from the robot.

    Each entry contains:
      level       – severity (0 = OK, 1 = WARN, 2 = ERROR)
      name        – component name
      message     – human-readable status message
      hardware_id – hardware identifier string
      values      – list of {key, value} pairs with detailed metrics
    """
    if not _is_configured():
        raise HTTPException(status_code=503, detail="Robot credentials not configured. POST /robot/config first.")
    if _state["connection"] == "error":
        raise HTTPException(status_code=502, detail=f"Robot unreachable: {_state['error']}")
    return {
        "diagnostics": _state["diagnostics"],
        "count": len(_state["diagnostics"]),
        "last_updated": _state["last_poll_at"],
    }


@app.get("/robot/position")
def get_position():
    """
    Latest position/pose from the robot.

    Returns frame_id and a pose with:
      translation  – {x, y, z} in metres
      rotation     – {x, y, z, w} quaternion
    """
    if not _is_configured():
        raise HTTPException(status_code=503, detail="Robot credentials not configured. POST /robot/config first.")
    if _state["connection"] == "error":
        raise HTTPException(status_code=502, detail=f"Robot unreachable: {_state['error']}")
    if _state["position"] is None:
        raise HTTPException(status_code=503, detail="Position data not yet available.")
    return {
        "position": _state["position"],
        "last_updated": _state["last_poll_at"],
    }


@app.post("/robot/poll")
async def trigger_poll():
    """Trigger an immediate poll instead of waiting for the next interval."""
    await _poll_once()
    return {
        "polled_at": _state["last_poll_at"],
        "connection": _state["connection"],
        "error": _state["error"],
    }


# ---------------------------------------------------------------------------
# Light control
# ---------------------------------------------------------------------------

class LightCommand(BaseModel):
    state: str  # "on" or "off"


@app.get("/robot/light")
def get_light():
    """Return the current light state and when it was last changed."""
    return {
        "light": _state["light"],
        "light_updated_at": _state["light_updated_at"],
    }


@app.post("/robot/light")
async def set_light(body: LightCommand):
    """
    Turn the robot's light on or off.

    Sends a command to the Innok Robotics cloud API at
    POST /command/light  with body  {"state": "on"|"off"},
    then updates the cached state regardless of the API result so the
    dashboard always reflects the last-requested state.
    """
    if body.state not in ("on", "off"):
        raise HTTPException(status_code=400, detail='state must be "on" or "off"')

    api_error: str | None = None
    if _is_configured():
        try:
            url = f"{_base_url()}/command/light"
            params = {"apikey": _config.apikey} if _config.apikey else {}
            async with httpx.AsyncClient(verify=_config.verify_ssl, timeout=5.0) as client:
                resp = await client.post(
                    url,
                    params=params,
                    json={"state": body.state},
                    auth=(_config.username, _config.password),
                )
                resp.raise_for_status()
        except httpx.HTTPStatusError as exc:
            api_error = f"HTTP {exc.response.status_code}: {exc.response.text[:200]}"
        except Exception as exc:
            api_error = str(exc)

    _state["light"] = body.state
    _state["light_updated_at"] = datetime.now().isoformat()

    return {
        "light": _state["light"],
        "light_updated_at": _state["light_updated_at"],
        "api_error": api_error,
    }


# ---------------------------------------------------------------------------
# Config management
# ---------------------------------------------------------------------------

class ConfigUpdate(BaseModel):
    tenant: str | None = None
    username: str | None = None
    password: str | None = None
    apikey: str | None = None
    poll_interval_seconds: int | None = None
    verify_ssl: bool | None = None


@app.post("/robot/config")
async def update_config(body: ConfigUpdate):
    """
    Update robot credentials and connection settings at runtime.
    Only provided fields are changed; omit a field to leave it unchanged.
    Triggers an immediate poll after updating.
    """
    if body.tenant is not None:
        _config.tenant = body.tenant
    if body.username is not None:
        _config.username = body.username
    if body.password is not None:
        _config.password = body.password
    if body.apikey is not None:
        _config.apikey = body.apikey
    if body.poll_interval_seconds is not None:
        _config.poll_interval_seconds = body.poll_interval_seconds
    if body.verify_ssl is not None:
        _config.verify_ssl = body.verify_ssl

    # Immediately try the new credentials
    await _poll_once()

    return {
        "status": "config updated",
        "connection": _state["connection"],
        "error": _state["error"],
    }


@app.get("/robot/config")
def get_config():
    """Return current config (password is masked)."""
    return {
        "tenant": _config.tenant,
        "username": _config.username,
        "password": "***" if _config.password else "",
        "apikey": "***" if _config.apikey else "",
        "poll_interval_seconds": _config.poll_interval_seconds,
        "verify_ssl": _config.verify_ssl,
        "base_url": _base_url() if _config.tenant else None,
    }


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    print("Starting Robot Telemetry Service on port 8083...")
    print("Set credentials via env vars (INNOK_TENANT, INNOK_USER, INNOK_PASSWORD, INNOK_APIKEY)")
    print("or POST to /robot/config at runtime.")
    uvicorn.run(app, host="0.0.0.0", port=8083)
