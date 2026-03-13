"""
camera_service.py — USB camera MJPEG streaming service
Port: 8010
Endpoints:
  GET /           health + camera status
  GET /cameras    list available camera indices
  GET /stream     MJPEG video stream  (embed in <img src="...">)
  GET /frame      single JPEG frame as base64 JSON
  POST /select    switch active camera index  {"index": 1}
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

# ─── Camera state ──────────────────────────────────────────────────────────────
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


# ─── MJPEG frame generator ─────────────────────────────────────────────────────
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


# ─── Routes ────────────────────────────────────────────────────────────────────
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


# ─── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8010, log_level="info")
