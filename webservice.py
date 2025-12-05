#!/usr/bin/env python3
"""
Flask web service to stream two cameras (left/right) as MJPEG and show stats.
Reads camera device paths and settings from xler.yaml.
Run: python webservice.py --host 0.0.0.0 --port 8088
"""

import argparse
import threading
import time
from typing import Optional, Tuple, Any
from pathlib import Path

import yaml
from flask import Flask, Response, render_template, jsonify

try:
    import cv2  # type: ignore
except ImportError as exc:
    raise RuntimeError("OpenCV (opencv-python) is required: pip install opencv-python") from exc

app = Flask(__name__, template_folder="templates")


def load_app_settings(settings_path: str = "xler.yaml") -> dict:
    path = Path(settings_path)
    if not path.exists():
        return {}
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


class CameraReader:
    """Background frame reader that keeps the latest BGR frame and stats."""

    def __init__(self, device: str | int, resolution: Tuple[int, int], fps: float) -> None:
        self.device = device
        self.width, self.height = resolution
        self.fps = fps
        self._cap: Optional[Any] = None
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._latest_bgr: Optional[Any] = None
        self._last_ts: Optional[float] = None
        self._frame_count = 0

    def start(self) -> None:
        self._cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2) if isinstance(self.device, (int,)) or str(self.device).startswith("/dev/video") else cv2.VideoCapture(self.device)
        if not self._cap.isOpened():
            # try default backend
            self._cap = cv2.VideoCapture(self.device)
        if not self._cap.isOpened():
            raise RuntimeError(f"Failed to open camera {self.device}")
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))
        self._cap.set(cv2.CAP_PROP_FPS, float(self.fps))
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name=f"cam_reader_{self.device}", daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        period = 1.0 / max(1e-3, float(self.fps)) if self.fps > 0 else 1.0 / 30.0
        next_t = time.time()
        while not self._stop.is_set():
            now = time.time()
            sleep_t = next_t - now
            if sleep_t > 0:
                self._stop.wait(sleep_t)
                if self._stop.is_set():
                    break
            ret, frame = self._cap.read() if self._cap is not None else (False, None)
            next_t += period
            if not ret or frame is None:
                continue
            with self._lock:
                self._latest_bgr = frame
                self._last_ts = time.time()
                self._frame_count += 1

    def get_latest_jpeg(self) -> Optional[bytes]:
        with self._lock:
            frame = self._latest_bgr
        if frame is None:
            return None
        ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if not ok:
            return None
        return buf.tobytes()

    def stats(self) -> dict:
        with self._lock:
            fc = self._frame_count
            ts = self._last_ts
            w, h = self.width, self.height
        return {
            "resolution": {"width": w, "height": h},
            "fps_target": self.fps,
            "last_frame_ts": ts,
            "frames": fc,
        }

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None


# Global camera readers
left_reader: Optional[CameraReader] = None
right_reader: Optional[CameraReader] = None


@app.route("/")
def index():
    return render_template("index.html")


def _mjpeg_generator(reader: CameraReader):
    boundary = "frame"
    while True:
        jpeg = reader.get_latest_jpeg()
        if jpeg is None:
            time.sleep(0.02)
            continue
        yield (
            b"--" + boundary.encode() + b"\r\n"
            + b"Content-Type: image/jpeg\r\n"
            + f"Content-Length: {len(jpeg)}\r\n\r\n".encode()
            + jpeg
            + b"\r\n"
        )


@app.route("/stream/left.mjpg")
def stream_left():
    global left_reader
    if left_reader is None:
        return Response("Left camera not initialized", status=503)
    return Response(_mjpeg_generator(left_reader), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/stream/right.mjpg")
def stream_right():
    global right_reader
    if right_reader is None:
        return Response("Right camera not initialized", status=503)
    return Response(_mjpeg_generator(right_reader), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/api/stats")
def api_stats():
    return jsonify({
        "left": left_reader.stats() if left_reader else None,
        "right": right_reader.stats() if right_reader else None,
    })


def parse_resolution(value: str | Tuple[int, int]) -> Tuple[int, int]:
    if isinstance(value, tuple):
        return value
    import re
    m = re.match(r"(\d+)[xX](\d+)", str(value))
    if not m:
        raise ValueError(f"Invalid resolution: {value}")
    return int(m.group(1)), int(m.group(2))


def main():
    parser = argparse.ArgumentParser(description="XLER dual camera web service (Flask)")
    parser.add_argument("--host", default="0.0.0.0", help="Bind host (default 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8088, help="Port (default 8088)")
    parser.add_argument("--settings", default="xler.yaml", help="Settings YAML (default xler.yaml)")
    args = parser.parse_args()

    settings = load_app_settings(args.settings)
    camera = settings.get("camera", {})
    left_cfg = camera.get("left", {})
    right_cfg = camera.get("right", {})

    left_dev = left_cfg.get("device")
    right_dev = right_cfg.get("device")
    resolution = parse_resolution(camera.get("resolution", "1280x720"))
    fps = float(camera.get("fps", 30))

    if not left_dev or not right_dev:
        raise SystemExit("Camera devices not set in xler.yaml (camera.left/right.device)")

    # Initialize readers
    global left_reader, right_reader
    left_reader = CameraReader(left_dev, resolution, fps)
    right_reader = CameraReader(right_dev, resolution, fps)
    left_reader.start()
    right_reader.start()

    try:
        app.run(host=args.host, port=args.port, threaded=True)
    finally:
        left_reader.stop()
        right_reader.stop()


if __name__ == "__main__":
    main()
