#!/usr/bin/env python
"""
xler.py - Robot Base Control with LeKiwi Kinematics

Control a 3-wheel omnidirectional robot base using LeKiwi kinematics approach.
Uses WASD+QE keyboard controls for movement.

Features:
- Matrix-based kinematics with proper m/s to motor conversion
- Velocity scaling to prevent motor saturation
- Simple keyboard-only input
- Configuration via YAML file
- Self-contained motors2 with observations example

Usage:
    python xler.py
    python xler.py --port /dev/cu.usbserial-XXXXX
    python xler.py --speed 0.8
    python xler.py --show-config

Controls:
    W/A/S/D - Forward/Left/Back/Rightw
    Q/E - Rotate Left/Right
    SPACE - Stop
    ESC - Exit
"""

import argparse
import logging
import sys
import time
import glob
import platform
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent))

import yaml

# Import from motors2 (self-contained package)
from motors2 import Motor, MotorNormMode, find_serial_port
from motors2.feetech import FeetechMotorsBus
from motors2.base_controller import LeKiwiBaseController
from motors2.keyboard_input import KeyboardInput

from camera_recorder import StereoCameraRecorder

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def load_config(config_path: str = "motors2/config.yaml", **overrides) -> dict:
    """
    Load configuration from YAML file with optional overrides.

    Args:
        config_path: Path to config.yaml file
        **overrides: Key-value pairs to override config values

    Returns:
        Configuration dictionary
    """
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    # Apply command-line overrides
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


def load_app_settings(settings_path: str = "xler.yaml") -> dict:
    """Load auxiliary application settings (camera, orientation, etc.)."""
    path = Path(settings_path)
    if not path.exists():
        return {}
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


def print_config(config: dict):
    """Pretty print configuration."""
    print("\n" + "="*60)
    print("Configuration")
    print("="*60)
    print(f"\nMotor IDs:")
    print(f"  Left:  {config['motor_ids']['left']}")
    print(f"  Right: {config['motor_ids']['right']}")
    print(f"  Back:  {config['motor_ids']['back']}")
    print(f"\nSerial:")
    print(f"  Port: {config['serial']['port']}")
    print(f"  Baudrate: {config['serial']['baudrate']}")
    print(f"  Protocol: {config['serial']['protocol_version']}")
    print(f"\nMotor:")
    print(f"  Model: {config['motor']['model']}")
    print(f"  Torque Limit: {config['motor']['torque_limit']}")
    print(f"\nGeometry:")
    print(f"  Wheel Radius: {config['geometry']['wheel_radius_m']} m")
    print(f"  Base Radius: {config['geometry']['base_radius_m']} m")
    print(f"  Wheel Angles: {config['geometry']['wheel_axis_angles_deg']} deg")
    print(f"  Max Wheel Raw: {config['geometry']['max_wheel_raw']}")
    print(f"\nControl:")
    print(f"  Frequency: {config['control']['frequency']} Hz")
    print(f"  Max Speed: {config['control']['max_speed_ms']} m/s")
    print(f"  Max Rotation: {config['control']['max_rotation_degs']} deg/s")
    print("="*60 + "\n")


def print_banner():
    """Print startup banner."""
    print("\n" + "="*70)
    print("  XLER - Robot Base Control (LeKiwi Kinematics)")
    print("="*70)
    print("\n🤖 3-Wheel Omnidirectional Base Control")
    print("📐 Matrix-based kinematics with velocity scaling")
    print("⚡ Self-contained motors2 with observations")
    print("\n⌨️  Keyboard Controls:")
    print("  W - Forward    |  A - Strafe Left   |  Q - Rotate Left")
    print("  S - Backward   |  D - Strafe Right  |  E - Rotate Right")
    print("  SPACE - Stop   |  ESC - Exit")
    print("\n💡 Tips:")
    print("  - HOLD keys to move, RELEASE to stop")
    print("  - Combine W/A/S/D for diagonal movement")
    print("  - Add Q/E for rotation while moving")
    print("="*70 + "\n")


def main():
    parser = argparse.ArgumentParser(description="Robot base control with LeKiwi kinematics")
    parser.add_argument("--port", type=str, default=None, help="Serial port (overrides config.yaml)")
    parser.add_argument("--speed", type=float, default=None, help="Max speed in m/s (overrides config.yaml)")
    parser.add_argument("--motor-left", type=int, default=None, help="Left motor ID")
    parser.add_argument("--motor-right", type=int, default=None, help="Right motor ID")
    parser.add_argument("--motor-back", type=int, default=None, help="Back motor ID")
    parser.add_argument("--show-config", action="store_true", help="Show configuration and exit")
    parser.add_argument("--record-cameras", action="store_true", help="Enable stereo camera recording")
    parser.add_argument("--camera-left", type=str, default=None, help="Left camera device path (e.g. /dev/video1)")
    parser.add_argument("--camera-right", type=str, default=None, help="Right camera device path (e.g. /dev/video3)")
    parser.add_argument("--camera-frame-interval", type=int, default=None, help="Frames between captures (default: 20)")
    parser.add_argument("--camera-output-dir", type=str, default=None, help="Root directory for captured images")
    args = parser.parse_args()

    # Load configuration
    print("[init] Loading configuration from motors2/config.yaml...")
    config = load_config(
        motor_left=args.motor_left,
        motor_right=args.motor_right,
        motor_back=args.motor_back,
        max_speed=args.speed,
        port=args.port
    )
    app_settings = load_app_settings()
    camera_settings = app_settings.get("camera", {})

    # Show config and exit if requested
    if args.show_config:
        print_config(config)
        return

    # Auto-detect port if not specified
    port = config['serial']['port']
    if port is None:
        print("[init] Auto-detecting serial port...")
        port = find_serial_port()
        if port is None:
            print("❌ Error: Could not auto-detect serial port.")
            print("   Please specify with --port option or in motors2/config.yaml")
            return
        print(f"✅ Found serial port: {port}")
        config['serial']['port'] = port
    else:
        print(f"[init] Using serial port: {port}")

    # Create motors dict (like lekiwi_base does)
    # Note: motor names must match what base_controller expects
    motors = {
        "base_left_wheel": Motor(
            config['motor_ids']['left'],
            config['motor']['model'],
            MotorNormMode.DEGREES
        ),
        "base_right_wheel": Motor(
            config['motor_ids']['right'],
            config['motor']['model'],
            MotorNormMode.DEGREES
        ),
        "base_back_wheel": Motor(
            config['motor_ids']['back'],
            config['motor']['model'],
            MotorNormMode.DEGREES
        ),
    }

    # Initialize keyboard input
    print("\n[init] Initializing keyboard input...")
    keyboard = KeyboardInput()

    # Configure optional stereo camera recording
    recorder = None
    record_cameras = args.record_cameras or camera_settings.get("enabled", False)
    if record_cameras:
        left_cfg = camera_settings.get("left", {})
        right_cfg = camera_settings.get("right", {})
        left_device = args.camera_left or left_cfg.get("device")
        right_device = args.camera_right or right_cfg.get("device")
        frame_interval = args.camera_frame_interval or camera_settings.get("frame_interval", 20)
        output_dir = args.camera_output_dir or camera_settings.get("output_dir", "recordings")
        left_folder = left_cfg.get("folder", "front_stereo_cam_left")
        right_folder = right_cfg.get("folder", "front_stereo_cam_right")
        resolution = camera_settings.get("resolution", "1280x720")
        fps = camera_settings.get("fps", 30)
        pixel_format = camera_settings.get("pixel_format", "mjpeg")

        if not left_device or not right_device:
            logger.error("Camera recording requested but device paths are missing. "
                         "Check xler.yaml or provide --camera-left/--camera-right.")
            record_cameras = False
        else:
            try:
                recorder = StereoCameraRecorder(
                    left_device=left_device,
                    right_device=right_device,
                    frame_interval=frame_interval,
                    output_root=output_dir,
                    left_folder=left_folder,
                    right_folder=right_folder,
                    resolution=resolution,
                    fps=fps,
                    pixel_format=pixel_format,
                )
                recorder.start()
                logger.info(
                    "Stereo recording enabled: left=%s right=%s interval=%s output=%s res=%s fps=%s fmt=%s",
                    left_device,
                    right_device,
                    frame_interval,
                    output_dir,
                    resolution,
                    fps,
                    pixel_format,
                )
            except Exception as exc:
                logger.error("Failed to initialize camera recorder: %s", exc)
                recorder = None
                record_cameras = False

    # Create Feetech motor bus (like lekiwi_base does)
    print(f"\n[init] Creating FeetechMotorsBus on {port}...")
    bus = FeetechMotorsBus(
        port=port,
        motors=motors,
        protocol_version=config['serial']['protocol_version']
    )

    connected = False
    try:
        # Connect to motors
        print(f"[init] Connecting to motors...")
        bus.connect()
        connected = True

        # Create LeKiwi controller
        controller = LeKiwiBaseController(bus, config)
        controller.configure()

        # Print banner
        print_banner()

        # Print header for status output
        print("Ready! Starting control loop...\n")
        print(f"{'Time':<8} {'Cmd (x/y/θ)':<20} {'Obs (x/y/θ)':<20} {'Wheel Raw (L/R/B)':<20}")
        print("-" * 68)

        loop_counter = 0
        control_frequency = config['control']['frequency']
        loop_delay = 1.0 / control_frequency
        print_interval = config['control']['print_interval']
        max_speed = config['control']['max_speed_ms']
        max_rotation = config['control']['max_rotation_degs']

        start_time = time.time()
        last_loop_time = start_time

        while True:
            loop_counter += 1
            current_loop_time = time.time()
            dt = current_loop_time - last_loop_time
            last_loop_time = current_loop_time

            # Read keyboard input
            forward, strafe, rotate, exit_requested = keyboard.read()

            # Check for exit
            if exit_requested:
                print("\n[exit] ESC pressed, stopping...")
                break

            # Convert normalized input to m/s and deg/s
            x_vel = forward * max_speed      # Forward/backward in m/s
            y_vel = strafe * max_speed       # Left/right in m/s
            theta_vel = rotate * max_rotation  # Rotation in deg/s

            # Send movement command (LeKiwi kinematics)
            wheel_velocities = controller.move(
                x_vel=x_vel,
                y_vel=y_vel,
                theta_vel=theta_vel
            )

            # Read actual velocities from motors (observation)
            try:
                obs = controller.get_observation()
            except Exception as e:
                logger.warning(f"Failed to read observation: {e}")
                obs = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}

            # Capture stereo frames if requested
            if recorder is not None:
                recorder.maybe_capture(loop_counter, current_loop_time)

            # Only print when there's movement (commanded or observed)
            has_command = abs(x_vel) > 0.01 or abs(y_vel) > 0.01 or abs(theta_vel) > 0.01
            has_motion = abs(obs["x.vel"]) > 0.01 or abs(obs["y.vel"]) > 0.01 or abs(obs["theta.vel"]) > 0.01

            if (has_command or has_motion) and (loop_counter % print_interval == 0):
                elapsed = time.time() - start_time
                time_str = f"{elapsed:.1f}s"

                # Command velocities
                cmd_str = f"x:{x_vel:+.2f} y:{y_vel:+.2f} θ:{theta_vel:+4.0f}"

                # Observed velocities
                obs_str = f"x:{obs['x.vel']:+.2f} y:{obs['y.vel']:+.2f} θ:{obs['theta.vel']:+4.0f}"

                # Wheel raw velocities
                left_vel = wheel_velocities["base_left_wheel"]
                right_vel = wheel_velocities["base_right_wheel"]
                back_vel = wheel_velocities["base_back_wheel"]
                wheel_str = f"{left_vel:+5d} {right_vel:+5d} {back_vel:+5d}"

                print(f"{time_str:<8} {cmd_str:<20} {obs_str:<20} {wheel_str:<20}")

            # Maintain control frequency
            time.sleep(loop_delay)

    except ConnectionError as e:
        print(f"\n❌ Connection Failed: {e}")
        print(f"   Make sure motors are powered and port is correct")
        return

    except KeyboardInterrupt:
        print("\n\n[interrupt] Ctrl+C detected, stopping...")

    except Exception as e:
        print(f"\n❌ Unexpected Error: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # Cleanup
        if connected:
            print("\n[cleanup] Stopping motors...")
            try:
                controller.stop()
                controller.reset_to_position_mode()
                bus.disconnect()
            except Exception as e:
                logger.error(f"Cleanup error: {e}")

        if recorder is not None:
            print("[cleanup] Stopping camera recorder...")
            recorder.stop()

        keyboard.close()

        print("\n✅ Done. Motors stopped.\n")


if __name__ == "__main__":
    main()
