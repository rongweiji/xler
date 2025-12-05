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
        ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
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
recorder = None  # type: ignore


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


class WebRecorder:
    """Server-side recorder reusing frames from CameraReader for saving.

    Mirrors the folder structure and metadata style used by camera_recorder.py.
    """

    def __init__(self, output_root: Path, left_folder: str, right_folder: str,
                 resolution: Tuple[int, int], fps: float) -> None:
        self.output_root = output_root
        self.left_folder = left_folder
        self.right_folder = right_folder
        self.resolution = resolution
        self.fps = float(fps)
        self._workspace_path: Optional[Path] = None
        self._metadata_file = None
        self._frame_counter = 0
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._saved_count = 0

    def _next_frame_index(self, left_path: Path) -> int:
        max_id = 0
        for p in left_path.glob("*.jpg"):
            s = p.stem
            if s.isdigit():
                max_id = max(max_id, int(s))
        return max_id

    def start(self, left: CameraReader, right: CameraReader) -> None:
        if self._thread and self._thread.is_alive():
            return
        # Prepare workspace
        next_idx = 1
        if self.output_root.exists():
            indices = []
            for child in self.output_root.iterdir():
                if child.is_dir() and child.name.startswith("workspace"):
                    suf = child.name[len("workspace"):]
                    if suf.isdigit():
                        indices.append(int(suf))
            if indices:
                next_idx = max(indices) + 1
        ws_name = f"workspace{next_idx}"
        self._workspace_path = self.output_root / ws_name
        self._workspace_path.mkdir(parents=True, exist_ok=True)
        self.left_path = self._workspace_path / self.left_folder
        self.right_path = self._workspace_path / self.right_folder
        self.left_path.mkdir(parents=True, exist_ok=True)
        self.right_path.mkdir(parents=True, exist_ok=True)
        meta_path = self._workspace_path / "frames_time.json"
        self._metadata_file = meta_path.open("a", encoding="utf-8")
        self._frame_counter = self._next_frame_index(self.left_path)
        self._saved_count = 0
        self._stop.clear()

        def _loop():
            period = 1.0 / max(1e-3, self.fps) if self.fps > 0 else 1.0/30.0
            next_t = time.time()
            while not self._stop.is_set():
                now = time.time()
                sleep_t = next_t - now
                if sleep_t > 0:
                    self._stop.wait(sleep_t)
                    if self._stop.is_set():
                        break
                lj = left.get_latest_jpeg()
                rj = right.get_latest_jpeg()
                ts = time.time()
                next_t += period
                if lj is None or rj is None:
                    continue
                # Save pair
                self._frame_counter += 1
                fid = f"{self._frame_counter:07d}"
                try:
                    with open(self.left_path / f"{fid}.jpg", "wb") as f:
                        f.write(lj)
                    with open(self.right_path / f"{fid}.jpg", "wb") as f:
                        f.write(rj)
                    if self._metadata_file is not None:
                        rec = {"filename": f"{fid}.jpg", "timestamp_ns": int(ts*1_000_000_000)}
                        self._metadata_file.write(json.dumps(rec) + "\n")
                    self._saved_count += 2
                except Exception:
                    # Continue even if a single write fails
                    pass

        self._thread = threading.Thread(target=_loop, name="web_recorder", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._metadata_file is not None:
            try:
                self._metadata_file.flush(); self._metadata_file.close()
            except Exception:
                pass
            self._metadata_file = None

    def status(self) -> dict:
        return {
            "recording": bool(self._thread and self._thread.is_alive()),
            "workspace": str(self._workspace_path) if self._workspace_path else None,
            "saved_files": self._saved_count,
            "fps": self.fps,
            "resolution": {"width": self.resolution[0], "height": self.resolution[1]},
            "output_root": str(self.output_root),
            "left_folder": self.left_folder,
            "right_folder": self.right_folder,
        }


@app.route("/api/record/status")
def api_record_status():
    global recorder
    return jsonify(recorder.status() if recorder else {"recording": False})


@app.route("/api/record/start", methods=["POST"])
def api_record_start():
    global recorder, left_reader, right_reader
    if recorder and recorder.status().get("recording"):
        return jsonify({"ok": True, "already": True, "status": recorder.status()})
    if not left_reader or not right_reader:
        return jsonify({"ok": False, "error": "cameras not initialized"}), 503
    # Recorder is created in main with proper config; just start it
    recorder.start(left_reader, right_reader)
    return jsonify({"ok": True, "status": recorder.status()})


@app.route("/api/record/stop", methods=["POST"])
def api_record_stop():
    global recorder
    if not recorder:
        return jsonify({"ok": True, "status": {"recording": False}})
    recorder.stop()
    return jsonify({"ok": True, "status": recorder.status()})


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

    # Prepare recorder with same structure as camera_recorder.py
    global recorder
    output_root = Path(camera.get("output_dir", "recordings"))
    left_folder = left_cfg.get("folder", "front_stereo_cam_left")
    right_folder = right_cfg.get("folder", "front_stereo_cam_right")
    recorder = WebRecorder(output_root=output_root,
                           left_folder=left_folder,
                           right_folder=right_folder,
                           resolution=resolution,
                           fps=fps)

    try:
        app.run(host=args.host, port=args.port, threaded=True)
    finally:
        left_reader.stop()
        right_reader.stop()


if __name__ == "__main__":
    main()
