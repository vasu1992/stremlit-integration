"""
camera_service.py — USB camera + IP camera MJPEG streaming service
Port: 8010
Endpoints:
  GET /           health + USB camera status
  GET /cameras    list available USB camera indices
  GET /stream     MJPEG stream for USB camera  (embed in <img src="...">)
  GET /frame      single JPEG frame (USB) as base64 JSON
  POST /select    switch active USB camera index  {"index": 1}

  GET /ip/health  IP camera connection status
  GET /ip/stream  MJPEG stream for WiFi IP camera  (embed in <img src="...">)
  GET /ip/frame   single JPEG frame (IP camera) as base64 JSON
"""

import base64
import time
import threading

import cv2
import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from pydantic import BaseModel

app = FastAPI(title="Camera Service", version="1.0.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# ─── USB Camera state ──────────────────────────────────────────────────────────
_lock = threading.Lock()
_cap: cv2.VideoCapture | None = None
_camera_index: int = 0


def _open_camera(index: int) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)   # CAP_DSHOW = faster init on Windows
    if not cap.isOpened():
        cap = cv2.VideoCapture(index)               # fallback — generic backend
    return cap


def _get_cap() -> cv2.VideoCapture:
    """Return the current capture; reopen if closed."""
    global _cap
    with _lock:
        if _cap is None or not _cap.isOpened():
            _cap = _open_camera(_camera_index)
        return _cap


# ─── USB MJPEG frame generator ─────────────────────────────────────────────────
def _generate_frames():
    cap = _get_cap()
    while True:
        with _lock:
            ok, frame = cap.read()
        if not ok:
            # camera disconnected — send a placeholder and wait
            time.sleep(0.5)
            cap = _get_cap()
            continue

        encode_params = [cv2.IMWRITE_JPEG_QUALITY, 80]
        _, buf = cv2.imencode(".jpg", frame, encode_params)
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + buf.tobytes()
            + b"\r\n"
        )


# ─── IP Camera (WiFi) state ────────────────────────────────────────────────────
# Credentials: user=OCC_cam1, password=OCC@1234, IP=192.168.178.177
# The '@' in the password is URL-encoded as '%40' in the RTSP URL.
# Probes common RTSP path suffixes automatically; first one that opens wins.
_IP_CAM_HOST = "192.168.178.177"
_IP_CAM_USER = "OCC_cam1"
_IP_CAM_PASS = "OCC@1234"   # '@' percent-encoded

def _ip_url(path: str, port: int = 554) -> str:
    return f"rtsp://{_IP_CAM_USER}:{_IP_CAM_PASS}@{_IP_CAM_HOST}:{port}{path}"

# Ordered list of RTSP paths to probe (covers most IP camera brands)
_IP_CAM_CANDIDATES: list[str] = [
    _ip_url("/stream1"),                          # generic / many brands
    _ip_url("/stream"),                           # generic
    _ip_url("/live"),                             # generic
    _ip_url("/video1"),                           # generic
    _ip_url("/"),                                 # bare root
    _ip_url("/onvif1"),                           # ONVIF cameras
    _ip_url("/Streaming/Channels/101"),           # Hikvision main stream
    _ip_url("/Streaming/Channels/102"),           # Hikvision sub-stream
    _ip_url("/cam/realmonitor?channel=1&subtype=0"),  # Dahua
    _ip_url("/live/ch00_0"),                      # Reolink / others
    _ip_url("/h264Preview_01_main"),              # Reolink alt
    _ip_url("/stream1", port=8554),               # non-standard port
    f"http://{_IP_CAM_HOST}/video.mjpg",          # HTTP MJPEG fallback
    f"http://{_IP_CAM_HOST}:8080/video",          # HTTP MJPEG fallback
]

_ip_lock = threading.Lock()
_ip_cap: cv2.VideoCapture | None = None
_ip_active_url: str = ""


def _open_ip_camera() -> cv2.VideoCapture:
    """Probe RTSP/HTTP candidates; return the first capture that opens."""
    global _ip_active_url
    for url in _IP_CAM_CANDIDATES:
        cap = cv2.VideoCapture()
        # Set a 4-second connection timeout before opening
        cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 4000)
        cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC,  4000)
        backend = cv2.CAP_FFMPEG if url.startswith("rtsp") else cv2.CAP_ANY
        cap.open(url, backend)
        if cap.isOpened():
            _ip_active_url = url
            print(f"[IP cam] Connected: {url}")
            return cap
        cap.release()
    # None worked — return an unopened capture so callers can detect failure
    _ip_active_url = ""
    print(f"[IP cam] Could not connect to {_IP_CAM_HOST} on any known path.")
    return cv2.VideoCapture()


def _get_ip_cap() -> cv2.VideoCapture:
    """Return the IP camera capture; reopen if closed."""
    global _ip_cap
    with _ip_lock:
        if _ip_cap is None or not _ip_cap.isOpened():
            _ip_cap = _open_ip_camera()
        return _ip_cap


def _generate_ip_frames():
    """MJPEG generator for the WiFi IP camera."""
    cap = _get_ip_cap()
    while True:
        with _ip_lock:
            ok, frame = cap.read()
        if not ok:
            time.sleep(0.5)
            cap = _get_ip_cap()
            continue

        encode_params = [cv2.IMWRITE_JPEG_QUALITY, 80]
        _, buf = cv2.imencode(".jpg", frame, encode_params)
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + buf.tobytes()
            + b"\r\n"
        )


# ─── USB Camera Routes ─────────────────────────────────────────────────────────
@app.get("/")
def health():
    cap = _get_cap()
    info: dict = {"status": "ok", "camera_index": _camera_index, "camera_open": cap.isOpened()}
    if cap.isOpened():
        info["width"]  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        info["height"] = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        info["fps"]    = cap.get(cv2.CAP_PROP_FPS)
    return info


@app.get("/cameras")
def list_cameras():
    """Probe the first 8 camera indices and report which ones open."""
    available = []
    for i in range(8):
        c = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        if c.isOpened():
            available.append(i)
            c.release()
    return {"cameras": available}


@app.get("/stream")
def stream(index: int = 0):
    """MJPEG stream.  Embed with: <img src='http://localhost:8010/stream'>"""
    global _cap, _camera_index
    if index != _camera_index:
        with _lock:
            if _cap is not None:
                _cap.release()
            _camera_index = index
            _cap = _open_camera(index)
    return StreamingResponse(
        _generate_frames(),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@app.get("/frame")
def single_frame():
    """Return one JPEG frame as base64 JSON."""
    cap = _get_cap()
    with _lock:
        ok, frame = cap.read()
    if not ok:
        raise HTTPException(status_code=503, detail="Camera not readable")
    _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
    return {
        "frame": base64.b64encode(buf).decode(),
        "width":  frame.shape[1],
        "height": frame.shape[0],
        "timestamp": time.time(),
    }


class SelectBody(BaseModel):
    index: int


@app.post("/select")
def select_camera(body: SelectBody):
    """Switch to a different camera index."""
    global _cap, _camera_index
    with _lock:
        if _cap is not None:
            _cap.release()
        _camera_index = body.index
        _cap = _open_camera(body.index)
        opened = _cap.isOpened()
    if not opened:
        raise HTTPException(status_code=404, detail=f"Camera index {body.index} not found")
    return {"status": "switched", "camera_index": body.index}


# ─── IP Camera Routes ──────────────────────────────────────────────────────────
@app.get("/ip/health")
def ip_camera_health():
    """Check WiFi IP camera connectivity."""
    cap = _get_ip_cap()
    connected = cap.isOpened()
    info: dict = {
        "status": "ok" if connected else "unavailable",
        "camera": "OCC_cam1",
        "ip": _IP_CAM_HOST,
        "connected": connected,
        "active_url": _ip_active_url if connected else None,
    }
    if connected:
        info["width"]  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        info["height"] = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        info["fps"]    = cap.get(cv2.CAP_PROP_FPS)
    return info


@app.get("/ip/stream")
def ip_stream():
    """MJPEG stream for the WiFi IP camera.  Embed with: <img src='http://localhost:8010/ip/stream'>"""
    return StreamingResponse(
        _generate_ip_frames(),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@app.get("/ip/frame")
def ip_single_frame():
    """Return one JPEG frame from the IP camera as base64 JSON."""
    cap = _get_ip_cap()
    with _ip_lock:
        ok, frame = cap.read()
    if not ok:
        raise HTTPException(status_code=503, detail="IP camera not readable — check network connection")
    _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
    return {
        "frame": base64.b64encode(buf).decode(),
        "width":  frame.shape[1],
        "height": frame.shape[0],
        "timestamp": time.time(),
        "source": "ip_camera",
        "camera": "OCC_cam1",
    }


# ─── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8010, log_level="info")
