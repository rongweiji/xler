#!/usr/bin/env python3
"""
Flask web service to stream two cameras (left/right) as MJPEG and show stats.
Reads camera device paths and settings from xler.yaml.
Run: python webservice.py --host 0.0.0.0 --port 8088
"""

import argparse
import threading
import time
import json
import os
from typing import Optional, Tuple, Any
from pathlib import Path

import yaml
from flask import Flask, Response, render_template, jsonify, request

try:
    import cv2  # type: ignore
except ImportError as exc:
    raise RuntimeError("OpenCV (opencv-python) is required: pip install opencv-python") from exc

# Motor control imports (reusing xler stack)
from motors2 import Motor, MotorNormMode, find_serial_port
from motors2.feetech import FeetechMotorsBus
from motors2.base_controller import LeKiwiBaseController

app = Flask(__name__, template_folder="templates")


def load_app_settings(settings_path: str = "xler.yaml") -> dict:
    path = Path(settings_path)
    if not path.exists():
        return {}
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def load_motor_config(config_path: str = "motors2/config.yaml", **overrides) -> dict:
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    if 'port' in overrides and overrides['port'] is not None:
        config['serial']['port'] = overrides['port']
    if 'motor_left' in overrides and overrides['motor_left'] is not None:
        config['motor_ids']['left'] = overrides['motor_left']
    if 'motor_right' in overrides and overrides['motor_right'] is not None:
        config['motor_ids']['right'] = overrides['motor_right']
    if 'motor_back' in overrides and overrides['motor_back'] is not None:
        config['motor_ids']['back'] = overrides['motor_back']
    if 'max_speed' in overrides and overrides['max_speed'] is not None:
        config['control']['max_speed_ms'] = overrides['max_speed']
    return config


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
capture_session = None  # type: ignore
drive_loop = None  # type: ignore


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
        # Line-buffered writes so records appear during recording
        self._metadata_file = meta_path.open("a", encoding="utf-8", buffering=1)
        self._frame_counter = self._next_frame_index(self.left_path)
        self._saved_count = 0
        self._stop.clear()
        self._last_flush_ts = time.time()
        self._since_flush = 0

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
                        self._since_flush += 1
                        # Flush + fsync every write to ensure durability/visibility on all platforms
                        try:
                            self._metadata_file.flush()
                            import os
                            os.fsync(self._metadata_file.fileno())
                        except Exception:
                            pass
                        self._last_flush_ts = time.time()
                        if self._since_flush >= 50:
                            self._since_flush = 0
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


class CaptureSession:
    """Button-triggered capture session that saves pairs into a persistent folder.

    Uses the same naming and metadata scheme, but only writes on demand.
    """

    def __init__(self, output_root: Path, session_folder: str,
                 left_folder: str, right_folder: str) -> None:
        self.output_root = output_root
        self.session_folder = session_folder
        self.left_folder = left_folder
        self.right_folder = right_folder
        self._workspace_path: Path = self.output_root / self.session_folder
        self._workspace_path.mkdir(parents=True, exist_ok=True)
        self.left_path = self._workspace_path / self.left_folder
        self.right_path = self._workspace_path / self.right_folder
        self.left_path.mkdir(parents=True, exist_ok=True)
        self.right_path.mkdir(parents=True, exist_ok=True)
        self._meta_path = self._workspace_path / "frames_time.json"
        self._metadata_file = self._meta_path.open("a", encoding="utf-8", buffering=1)
        self._frame_counter = self._next_frame_index(self.left_path)

    def _next_frame_index(self, left_path: Path) -> int:
        max_id = 0
        for p in left_path.glob("*.jpg"):
            s = p.stem
            if s.isdigit():
                max_id = max(max_id, int(s))
        return max_id

    def capture_once(self, left: CameraReader, right: CameraReader) -> dict:
        lj = left.get_latest_jpeg()
        rj = right.get_latest_jpeg()
        ts = time.time()
        if lj is None or rj is None:
            return {"ok": False, "error": "no frame"}
        self._frame_counter += 1
        fid = f"{self._frame_counter:07d}"
        try:
            with open(self.left_path / f"{fid}.jpg", "wb") as f:
                f.write(lj)
            with open(self.right_path / f"{fid}.jpg", "wb") as f:
                f.write(rj)
            rec = {"filename": f"{fid}.jpg", "timestamp_ns": int(ts*1_000_000_000)}
            self._metadata_file.write(json.dumps(rec) + "\n")
            try:
                self._metadata_file.flush(); import os; os.fsync(self._metadata_file.fileno())
            except Exception:
                pass
            return {"ok": True, "filename": f"{fid}.jpg", "timestamp_ns": rec["timestamp_ns"],
                    "workspace": str(self._workspace_path)}
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def status(self) -> dict:
        return {
            "workspace": str(self._workspace_path),
            "left_folder": self.left_folder,
            "right_folder": self.right_folder,
        }


class DriveLoop:
    """Simple control loop for driving the base from HTTP commands."""

    def __init__(self, config: dict, port_override: Optional[str] = None) -> None:
        self.config = config
        self.port_override = port_override
        self.connected = False
        self._lock = threading.Lock()
        self._cmd = {"x": 0.0, "y": 0.0, "theta": 0.0, "hold_until": 0.0}
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._controller: Optional[LeKiwiBaseController] = None
        self._bus: Optional[FeetechMotorsBus] = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        port = self.config['serial']['port']
        if port is None:
            port = find_serial_port()
        if port is None:
            print("[drive] No serial port found; drive disabled")
            return
        motors = {
            "base_left_wheel": Motor(self.config['motor_ids']['left'], self.config['motor']['model'], MotorNormMode.DEGREES),
            "base_right_wheel": Motor(self.config['motor_ids']['right'], self.config['motor']['model'], MotorNormMode.DEGREES),
            "base_back_wheel": Motor(self.config['motor_ids']['back'], self.config['motor']['model'], MotorNormMode.DEGREES),
        }
        bus = FeetechMotorsBus(port=port, motors=motors, protocol_version=self.config['serial']['protocol_version'])
        try:
            bus.connect()
            controller = LeKiwiBaseController(bus, self.config)
            controller.configure()
            self._controller = controller
            self._bus = bus
            self.connected = True
        except Exception as e:
            print(f"[drive] Failed to init motors: {e}")
            self.connected = False
            return

        def _loop():
            freq = self.config['control']['frequency']
            dt = 1.0 / freq
            max_speed = self.config['control']['max_speed_ms']
            max_rot = self.config['control']['max_rotation_degs']
            while not self._stop.is_set():
                now = time.time()
                with self._lock:
                    if now > self._cmd['hold_until']:
                        cx = cy = cth = 0.0
                    else:
                        cx = self._cmd['x'] * max_speed
                        cy = self._cmd['y'] * max_speed
                        cth = self._cmd['theta'] * max_rot
                try:
                    self._controller.move(x_vel=cx, y_vel=cy, theta_vel=cth)
                except Exception:
                    pass
                self._stop.wait(dt)
            # stop motors on exit
            try:
                self._controller.stop()
                self._controller.reset_to_position_mode()
                self._bus.disconnect()
            except Exception:
                pass

        self._stop.clear()
        self._thread = threading.Thread(target=_loop, name="drive_loop", daemon=True)
        self._thread.start()

    def set_command(self, x: float, y: float, theta: float, hold_ms: int = 400):
        with self._lock:
            self._cmd = {
                "x": max(-1.0, min(1.0, x)),
                "y": max(-1.0, min(1.0, y)),
                "theta": max(-1.0, min(1.0, theta)),
                "hold_until": time.time() + (hold_ms / 1000.0),
            }

    def stop_now(self):
        with self._lock:
            self._cmd = {"x": 0.0, "y": 0.0, "theta": 0.0, "hold_until": 0.0}

    def status(self) -> dict:
        with self._lock:
            cmd = dict(self._cmd)
        return {
            "connected": self.connected,
            "port": self.config['serial']['port'],
            "cmd": cmd,
            "max_speed_ms": self.config['control']['max_speed_ms'],
            "max_rotation_degs": self.config['control']['max_rotation_degs'],
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


@app.route("/api/capture/status")
def api_capture_status():
    global capture_session
    return jsonify(capture_session.status() if capture_session else None)


@app.route("/api/capture/once", methods=["POST"])
def api_capture_once():
    global capture_session, left_reader, right_reader
    if not capture_session:
        return jsonify({"ok": False, "error": "capture session not initialized"}), 503
    if not left_reader or not right_reader:
        return jsonify({"ok": False, "error": "cameras not initialized"}), 503
    return jsonify(capture_session.capture_once(left_reader, right_reader))


@app.route("/api/drive/status")
def api_drive_status():
    global drive_loop
    return jsonify(drive_loop.status() if drive_loop else {"connected": False})


@app.route("/api/drive/command", methods=["POST"])
def api_drive_command():
    global drive_loop
    if not drive_loop or not drive_loop.connected:
        return jsonify({"ok": False, "error": "drive not available"}), 503
    data = request.get_json(silent=True) or {}
    x = float(data.get("x", 0.0))
    y = float(data.get("y", 0.0))
    theta = float(data.get("theta", 0.0))
    hold_ms = int(data.get("hold_ms", 400))
    drive_loop.set_command(x, y, theta, hold_ms)
    return jsonify({"ok": True, "status": drive_loop.status()})


@app.route("/api/drive/stop", methods=["POST"])
def api_drive_stop():
    global drive_loop
    if not drive_loop or not drive_loop.connected:
        return jsonify({"ok": True, "status": {"connected": False}})
    drive_loop.stop_now()
    return jsonify({"ok": True, "status": drive_loop.status()})


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

    # Prepare capture session using a persistent calibration workspace
    global capture_session
    session_folder = camera.get("calibration_workspace", "workspace_cali")
    capture_session = CaptureSession(output_root=output_root,
                                     session_folder=session_folder,
                                     left_folder=left_folder,
                                     right_folder=right_folder)

    # Initialize drive loop
    global drive_loop
    motor_config = load_motor_config("motors2/config.yaml")
    drive_loop = DriveLoop(motor_config)
    drive_loop.start()

    try:
        app.run(host=args.host, port=args.port, threaded=True)
    finally:
        left_reader.stop()
        right_reader.stop()


if __name__ == "__main__":
    main()
