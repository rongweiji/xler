"""
Camera Recorder Utility

Provides background video recording from camera using OpenCV.
Designed for MacBook camera but works with any OpenCV-compatible camera.

Example:
    from utils.camera_recorder import CameraRecorder

    # Create recorder
    recorder = CameraRecorder(output_folder="scan-video", fps=30)

    # Start recording
    if recorder.start():
        info = recorder.get_info()
        print(f"Recording: {info['width']}x{info['height']} @ {info['fps']}fps")

        # ... do work ...

        # Stop recording
        recorder.stop()
"""

import cv2
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Optional, Dict


class CameraRecorder:
    """
    Background camera recorder using OpenCV.

    Records video from default camera to MP4 file in background thread.
    Automatically creates output folder and generates timestamped filenames.
    """

    def __init__(self, output_folder="scan-video", fps=30, camera_index=0):
        """
        Initialize camera recorder.

        Args:
            output_folder: Folder to save videos (created if doesn't exist)
            fps: Frames per second for recording, default 30
            camera_index: Camera device index, default 0 (built-in camera)
        """
        self.output_folder = Path(output_folder)
        self.fps = fps
        self.camera_index = camera_index

        self.camera = None
        self.video_writer = None
        self.recording_thread = None
        self.is_recording = False
        self.filename = None

        # Video properties (set when camera opened)
        self.width = 0
        self.height = 0
        self.actual_fps = fps

    def start(self) -> bool:
        """
        Start recording video.

        Returns:
            True if recording started successfully, False otherwise
        """
        if self.is_recording:
            print("⚠️  Camera already recording")
            return False

        # Create output folder
        self.output_folder.mkdir(parents=True, exist_ok=True)

        # Open camera
        print(f"[Camera] Opening camera {self.camera_index}...")
        self.camera = cv2.VideoCapture(self.camera_index)

        if not self.camera.isOpened():
            print("❌ Failed to open camera")
            return False

        # Get camera properties
        self.width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.actual_fps = int(self.camera.get(cv2.CAP_PROP_FPS))

        # Use requested FPS if camera FPS is invalid
        if self.actual_fps <= 0:
            self.actual_fps = self.fps

        # Generate filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = self.output_folder / f"scan_{timestamp}.mp4"

        # Create video writer
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # MP4 codec
        self.video_writer = cv2.VideoWriter(
            str(self.filename),
            fourcc,
            self.actual_fps,
            (self.width, self.height)
        )

        if not self.video_writer.isOpened():
            print("❌ Failed to create video writer")
            self.camera.release()
            return False

        # Start recording thread
        self.is_recording = True
        self.recording_thread = threading.Thread(target=self._recording_loop, daemon=True)
        self.recording_thread.start()

        print(f"✅ Camera recording started: {self.width}x{self.height} @ {self.actual_fps}fps")
        print(f"📹 Saving to: {self.filename}")

        return True

    def stop(self):
        """Stop recording and release resources."""
        if not self.is_recording:
            return

        print("\n[Camera] Stopping recording...")
        self.is_recording = False

        # Wait for recording thread to finish
        if self.recording_thread:
            self.recording_thread.join(timeout=2.0)

        # Release resources
        if self.video_writer:
            self.video_writer.release()

        if self.camera:
            self.camera.release()

        print(f"✅ Video saved: {self.filename}")

    def get_info(self) -> Dict[str, any]:
        """
        Get current recording information.

        Returns:
            Dict with width, height, fps, filename, is_recording
        """
        return {
            "width": self.width,
            "height": self.height,
            "fps": self.actual_fps,
            "filename": str(self.filename) if self.filename else None,
            "is_recording": self.is_recording,
        }

    def _recording_loop(self):
        """
        Internal recording loop (runs in background thread).
        Continuously reads frames and writes to video file.
        """
        frame_count = 0
        start_time = time.time()

        while self.is_recording:
            ret, frame = self.camera.read()

            if not ret:
                print("⚠️  Failed to read frame from camera")
                break

            # Write frame to video
            self.video_writer.write(frame)
            frame_count += 1

            # Small sleep to prevent CPU overload
            # Target frame time = 1/fps seconds
            time.sleep(1.0 / self.actual_fps * 0.5)  # Sleep for half frame time

        elapsed = time.time() - start_time
        print(f"[Camera] Recorded {frame_count} frames in {elapsed:.1f}s ({frame_count/elapsed:.1f} fps avg)")

    def __del__(self):
        """Cleanup: ensure resources are released"""
        self.stop()
