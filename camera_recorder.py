#!/usr/bin/env python
"""
Stereo camera recording utilities for XLER.

Provides a lightweight recorder that can grab frames from two V4L2 devices
and persist them as JPEG images. Designed to be triggered from the control
loop without blocking the main thread. Each saved frame embeds a timestamp
in the EXIF metadata when supported by Pillow.
"""

from __future__ import annotations

import json
import re
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any, Optional, Tuple

try:  # OpenCV is required for grabbing frames
    import cv2  # type: ignore
except ImportError:  # pragma: no cover - handled at runtime
    cv2 = None  # type: ignore

try:  # Pillow is required for saving images
    from PIL import Image  # type: ignore
except ImportError:  # pragma: no cover - handled at runtime
    Image = None  # type: ignore


class StereoCameraRecorder:
    """
    Capture synchronized frames from left/right cameras and persist them as JPEG.

    The recorder reads from two V4L2 devices (left/right), capturing frames
    every N invocations of ``maybe_capture``. Saving happens asynchronously to
    avoid stalling the control loop.
    """

    def __init__(
        self,
        *,
        left_device: str,
        right_device: str,
        frame_interval: int = 20,
        output_root: Path | str = Path("recordings"),
        left_folder: str = "front_stereo_cam_left",
        right_folder: str = "front_stereo_cam_right",
        resolution: str | Tuple[int, int] = "1280x720",
        fps: int = 30,
        pixel_format: str = "mjpeg",
    ) -> None:
        if cv2 is None:
            raise RuntimeError(
                "OpenCV (opencv-python) is required for camera recording. "
                "Install it inside your environment to enable this feature."
            )
        if Image is None:
            raise RuntimeError(
                "Pillow is required for camera recording. "
                "Install it inside your environment to enable this feature."
            )

        self.left_device = left_device
        self.right_device = right_device
        self.frame_interval = max(frame_interval, 1)
        self.output_root = Path(output_root)
        self.left_folder = left_folder
        self.right_folder = right_folder
        self.resolution = self._parse_resolution(resolution)
        self.fps = fps
        self.pixel_format = pixel_format.lower()

        self._lock = threading.Lock()
        self._left_capture: Optional["cv2.VideoCapture"] = None
        self._right_capture: Optional["cv2.VideoCapture"] = None
        self._executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="xler_cam")
        self._started = False

        # Monotonic image counter and metadata mapping
        self._frame_counter = 0
        self._metadata_path: Optional[Path] = None
        self._metadata: dict[str, dict[str, int]] = {}

    def start(self) -> None:
        """Open camera devices and prepare output directories."""
        with self._lock:
            if self._started:
                return

            self.left_path = self.output_root / self.left_folder
            self.right_path = self.output_root / self.right_folder
            self.left_path.mkdir(parents=True, exist_ok=True)
            self.right_path.mkdir(parents=True, exist_ok=True)

            # JSON mapping file in recordings root
            self._metadata_path = self.output_root / "frames_meta.json"
            if self._metadata_path.exists():
                try:
                    with self._metadata_path.open("r", encoding="utf-8") as f:
                        existing = json.load(f)
                    if isinstance(existing, dict):
                        self._metadata = existing  # type: ignore[assignment]
                        # Continue numbering from the max existing index (filenames as 7-digit numbers)
                        try:
                            max_id = max(int(name.split(".")[0]) for name in self._metadata.keys())
                        except ValueError:
                            max_id = 0
                        self._frame_counter = max_id
                except (json.JSONDecodeError, OSError):
                    # If corrupt, start fresh
                    self._metadata = {}
                    self._frame_counter = 0
            else:
                self._metadata = {}
                self._frame_counter = 0

            self._left_capture = self._open_capture(self.left_device)
            self._right_capture = self._open_capture(self.right_device)

            if not self._left_capture.isOpened():
                raise RuntimeError(f"Failed to open left camera device '{self.left_device}'")
            if not self._right_capture.isOpened():
                raise RuntimeError(f"Failed to open right camera device '{self.right_device}'")

            self._started = True

    def stop(self) -> None:
        """Release camera handles and drain outstanding save tasks."""
        with self._lock:
            if not self._started:
                return

            if self._left_capture is not None:
                self._left_capture.release()
                self._left_capture = None

            if self._right_capture is not None:
                self._right_capture.release()
                self._right_capture = None

            self._executor.shutdown(wait=True, cancel_futures=False)

            # Persist metadata mapping if available
            if self._metadata_path is not None:
                try:
                    with self._metadata_path.open("w", encoding="utf-8") as f:
                        json.dump(self._metadata, f, indent=2, sort_keys=True)
                except OSError:
                    pass

            self._started = False

    def maybe_capture(self, frame_index: int, timestamp: float) -> None:
        """
        Capture and save frames if the requested interval is met.

        Args:
            frame_index: Current control loop iteration
            timestamp:   Loop timestamp (seconds since epoch)
        """
        if not self._started or self.frame_interval <= 0:
            return

        if frame_index % self.frame_interval != 0:
            return

        # Avoid blocking the control loop longer than necessary.
        ret_left, frame_left = self._read_frame(self._left_capture)
        ret_right, frame_right = self._read_frame(self._right_capture)

        if not ret_left or frame_left is None:
            return
        if not ret_right or frame_right is None:
            return

        # Use a monotonic 7-digit counter for filenames
        with self._lock:
            self._frame_counter += 1
            frame_id = self._frame_counter
        self._executor.submit(
            self._save_pair,
            frame_left,
            frame_right,
            frame_index,
            timestamp,
            frame_id,
        )

    def _read_frame(self, capture: Optional["cv2.VideoCapture"]) -> tuple[bool, Optional[Any]]:
        if capture is None:
            return False, None
        return capture.read()

    def _parse_resolution(self, value: str | Tuple[int, int]) -> Tuple[int, int]:
        if isinstance(value, tuple):
            return value
        match = re.match(r"(\d+)[xX](\d+)", str(value))
        if not match:
            raise ValueError(f"Invalid resolution format: {value}")
        return int(match.group(1)), int(match.group(2))

    def _configure_capture(self, capture: "cv2.VideoCapture") -> None:
        width, height = self.resolution
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        capture.set(cv2.CAP_PROP_FPS, float(self.fps))

    def _open_capture(self, device) -> "cv2.VideoCapture":
        """
        Try several backend combinations to open a V4L2 device.

        Supports both numeric indices and explicit /dev/video paths.
        """
        attempts = []
        width, height = self.resolution
        if isinstance(device, int):
            attempts.extend([
                (device, cv2.CAP_V4L2),
                (device, cv2.CAP_ANY),
                (device, None),
            ])
        else:
            device_str = str(device)
            attempts.extend([
                (device_str, cv2.CAP_V4L2),
                (device_str, cv2.CAP_ANY),
                (device_str, None),
            ])
            match = re.match(r".*?/dev/video(\d+)$", device_str)
            if match:
                idx = int(match.group(1))
                attempts.extend([
                    (idx, cv2.CAP_V4L2),
                    (idx, cv2.CAP_ANY),
                    (idx, None),
                ])
            # Optional gstreamer pipeline attempt
            pipeline = (
                f"v4l2src device={device_str} ! "
                f"{self._gstreamer_caps()} ! "
                "videoconvert ! "
                "video/x-raw,format=BGR ! appsink"
            )
            attempts.append((pipeline, cv2.CAP_GSTREAMER))

        for source, backend in attempts:
            try:
                if backend is None:
                    capture = cv2.VideoCapture(source)
                else:
                    capture = cv2.VideoCapture(source, backend)
            except cv2.error:
                continue
            if capture.isOpened():
                self._configure_capture(capture)
                return capture
            capture.release()

        raise RuntimeError(f"Unable to open camera device '{device}' with V4L2 or default backend.")

    def _gstreamer_caps(self) -> str:
        width, height = self.resolution
        if self.pixel_format in {"mjpeg", "jpeg"}:
            return (
                f"image/jpeg,width={width},height={height},"
                f"framerate={self.fps}/1 ! jpegdec"
            )
        if self.pixel_format in {"yuyv", "yuy2"}:
            return (
                f"video/x-raw,format=YUY2,width={width},height={height},"
                f"framerate={self.fps}/1"
            )
        # Default raw format
        return (
            f"video/x-raw,width={width},height={height},"
            f"framerate={self.fps}/1"
        )

    def _save_pair(
        self,
        frame_left,
        frame_right,
        frame_index: int,
        timestamp: float,
        frame_id: int,
    ) -> None:
        """Persist the captured frames to disk."""
        # Convert BGR (OpenCV) to RGB for Pillow
        image_left = Image.fromarray(cv2.cvtColor(frame_left, cv2.COLOR_BGR2RGB))
        image_right = Image.fromarray(cv2.cvtColor(frame_right, cv2.COLOR_BGR2RGB))
        # Zero-padded 7-digit index
        filename = f"{frame_id:07d}"

        left_path = self.left_path / f"{filename}.jpg"
        right_path = self.right_path / f"{filename}.jpg"

        self._save_jpeg(image_left, left_path, timestamp)
        self._save_jpeg(image_right, right_path, timestamp)

        # Record mapping in memory (timestamp in nanoseconds)
        ts_ns = int(timestamp * 1_000_000_000)
        with self._lock:
            # One entry per filename; we rely on identical basenames for left/right
            if self._metadata is not None:
                self._metadata[f"{filename}.jpg"] = {
                    "timestamp_ns": ts_ns,
                    "frame_index": frame_index,
                }

    def _save_jpeg(self, image: "Image.Image", path: Path, timestamp: float) -> None:
        save_kwargs = {"format": "JPEG", "quality": 95}
        if hasattr(image, "getexif"):
            exif = image.getexif()
            timestamp_str = time.strftime("%Y:%m:%d %H:%M:%S", time.localtime(timestamp))
            exif[0x9003] = timestamp_str  # DateTimeOriginal
            exif[0x9004] = timestamp_str  # DateTimeDigitized
            save_kwargs["exif"] = exif.tobytes()
        image.save(path, **save_kwargs)


__all__ = ["StereoCameraRecorder"]
