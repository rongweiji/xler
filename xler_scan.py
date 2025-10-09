#!/usr/bin/env python
"""
xler_scan.py - Robot Base Control with Video Recording

Combines movement control (keyboard + DualSense controller) with camera recording.
Records video from MacBook camera while controlling the robot base.

Usage:
    # Auto-detect serial port
    python xler_scan.py --speed 400

    # Manually specify port
    python xler_scan.py --port /dev/cu.usbserial-XXXXX --speed 400

Controls:
    Keyboard:
        W/A/S/D - Forward/Left/Back/Right
        Q/E - Rotate Left/Right
        SPACE - Stop
        ESC - Exit (stops recording and motors)

    DualSense Controller (PS5):
        Left Stick - Forward/Back + Strafe Left/Right
        Right Stick - Rotate Left/Right

Features:
    - Video recording starts automatically
    - Saved to scan-video/ folder with timestamp
    - Prints camera info (resolution, FPS) on start
    - Prints control signals in real-time
    - Clean shutdown with Ctrl+C or ESC
"""

import argparse
import sys
import time
import glob
import platform
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent))

from motors.motors_bus import Motor, MotorNormMode
from motors.feetech import FeetechMotorsBus, OperatingMode
from motors.movement_controller import OmniBaseController
from motors.input_handler import CombinedInputHandler
from motors.config_loader import load_motor_config, print_motor_config, validate_motor_config
from utils.camera_recorder import CameraRecorder


def find_serial_port():
    """
    Automatically detect the serial port for USB-to-Serial adapter.

    Returns:
        str: The detected port path, or None if not found
    """
    system = platform.system()

    if system == "Darwin":  # macOS
        patterns = [
            "/dev/cu.usbserial*",
            "/dev/cu.SLAB_USBtoUART*",
            "/dev/cu.wchusbserial*",
            "/dev/cu.usbmodem*",
        ]
        for pattern in patterns:
            ports = glob.glob(pattern)
            if ports:
                return ports[0]

    elif system == "Linux":
        patterns = [
            "/dev/ttyUSB*",
            "/dev/ttyACM*",
        ]
        for pattern in patterns:
            ports = glob.glob(pattern)
            if ports:
                return ports[0]

    elif system == "Windows":
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        if ports:
            return ports[0].device

    return None


def print_banner(has_controller):
    """Print startup banner with instructions"""
    print("\n" + "="*70)
    print("  XLER SCAN - Robot Base Control + Video Recording")
    print("="*70)
    print("\n📹 Camera: Recording started")
    print("🤖 Motors: Ready for control")

    if has_controller:
        print("🎮 Input: Keyboard + DualSense Controller")
    else:
        print("⌨️  Input: Keyboard Only")

    print("\n⌨️  Keyboard Controls:")
    print("  W/A/S/D - Forward/Left/Back/Right")
    print("  Q/E - Rotate Left/Right")
    print("  SPACE - Stop")
    print("  ESC - Exit")

    if has_controller:
        print("\n🎮 DualSense Controller:")
        print("  Left Stick - Forward/Back + Strafe")
        print("  Right Stick - Rotate")

    print("\n💡 Exit: Press ESC or Ctrl+C to stop recording and exit")
    print("="*70 + "\n")


def main():
    parser = argparse.ArgumentParser(description="Robot base control with video recording")
    parser.add_argument("--port", type=str, default=None, help="Serial port (overrides config.yaml)")
    parser.add_argument("--speed", type=int, default=None, help="Base movement speed (overrides config.yaml)")
    parser.add_argument("--motor-left", type=int, default=None, help="Left motor ID (overrides config.yaml)")
    parser.add_argument("--motor-right", type=int, default=None, help="Right motor ID (overrides config.yaml)")
    parser.add_argument("--motor-back", type=int, default=None, help="Back motor ID (overrides config.yaml)")
    parser.add_argument("--camera-fps", type=int, default=30, help="Camera FPS")
    parser.add_argument("--output-folder", type=str, default="scan-video", help="Video output folder")
    parser.add_argument("--show-config", action="store_true", help="Show motor configuration and exit")
    args = parser.parse_args()

    # Load motor configuration from config.yaml with CLI overrides
    print("[init] Loading motor configuration...")
    motor_config = load_motor_config(
        motor_left=args.motor_left,
        motor_right=args.motor_right,
        motor_back=args.motor_back,
        speed=args.speed,
        port=args.port
    )

    # Validate configuration
    if not validate_motor_config(motor_config):
        print("❌ Invalid motor configuration. Please check motors/config.yaml")
        return

    # Show config and exit if requested
    if args.show_config:
        print_motor_config(motor_config)
        return

    # Extract values from config
    motor_ids = motor_config['motor_ids']
    motor_settings = motor_config['motor_settings']
    serial_config = motor_config['serial']
    control_config = motor_config['control']

    # Use speed from args or config
    speed = args.speed if args.speed is not None else motor_settings['default_speed']

    # Auto-detect port if not specified in args or config
    port = args.port if args.port is not None else serial_config['port']
    if port is None:
        print("[init] Auto-detecting serial port...")
        port = find_serial_port()
        if port is None:
            print("❌ Error: Could not auto-detect serial port.")
            print("   Please specify with --port option or in motors/config.yaml")
            return
        print(f"✅ Found serial port: {port}")
    else:
        print(f"[init] Using serial port: {port}")

    # Initialize camera recorder
    print(f"\n[init] Initializing camera...")
    camera = CameraRecorder(output_folder=args.output_folder, fps=args.camera_fps)

    if not camera.start():
        print("❌ Failed to start camera. Continue without recording? (y/n)")
        response = input().strip().lower()
        if response != 'y':
            return
        camera = None
    else:
        # Print camera info
        info = camera.get_info()
        print(f"\n📹 Camera Info:")
        print(f"   Resolution: {info['width']}x{info['height']}")
        print(f"   FPS: {info['fps']}")
        print(f"   Output: {info['filename']}")

    # Initialize input handler
    print(f"\n[init] Initializing input handlers...")
    input_handler = CombinedInputHandler(enable_keyboard=True, enable_dualsense=True)

    # Create motor bus configuration
    motors = {}
    for role, motor_id in motor_ids.items():
        motor_name = f"motor_{motor_id}"
        motors[motor_name] = Motor(
            id=motor_id,
            model=motor_settings['model'],
            norm_mode=MotorNormMode.DEGREES
        )

    # Connect to motor bus
    print(f"\n[init] Connecting to motors on {port}...")
    bus = FeetechMotorsBus(
        port=port,
        motors=motors,
        protocol_version=serial_config['protocol_version']
    )

    connected = False
    try:
        bus.connect()
        connected = True

        # Create movement controller
        controller = OmniBaseController(
            bus=bus,
            motor_ids=motor_ids,
            max_velocity=motor_settings['max_velocity']
        )

        # Setup motors with configured torque limit
        controller.setup_motors(torque_limit=motor_settings['torque_limit'])

        # Print banner
        print_banner(input_handler.has_controller())

        sys.stdout.write("Ready! Starting control loop...\r\n\r\n")
        sys.stdout.write(f"{'Time':<8} {'Input':<45} {'Velocities (L/R/B)':<20}\r\n")
        sys.stdout.write("-" * 75 + "\r\n")
        sys.stdout.flush()

        loop_counter = 0
        control_frequency = control_config['frequency']
        loop_delay = 1.0 / control_frequency
        print_interval = control_config['print_interval']

        start_time = time.time()

        while True:
            loop_counter += 1

            # Read inputs
            state = input_handler.read()

            # Check for exit
            if state.exit_requested:
                sys.stdout.write("\r\n[exit] ESC pressed, stopping...\r\n")
                sys.stdout.flush()
                break

            # Send movement command
            # Pass speed directly as absolute motor speed value (e.g., 400)
            velocities = controller.move(
                forward=state.forward,
                strafe=state.strafe,
                rotate=state.rotate,
                speed=speed  # Absolute speed value (0-1023)
            )

            # Print control info periodically
            if loop_counter % print_interval == 0:
                elapsed = time.time() - start_time
                time_str = f"{elapsed:.1f}s"

                # Build input display string
                input_str = input_handler.get_debug_string()

                # Add velocity components to display
                if abs(state.forward) > 0.01 or abs(state.strafe) > 0.01 or abs(state.rotate) > 0.01:
                    input_str += f" → Fwd:{state.forward:+.2f} Str:{state.strafe:+.2f} Rot:{state.rotate:+.2f}"

                vel_str = f"{velocities['left']:+5d} {velocities['right']:+5d} {velocities['back']:+5d}"

                # Use \r\n for proper line breaks in raw terminal mode
                sys.stdout.write(f"{time_str:<8} {input_str:<45} {vel_str:<20}\r\n")
                sys.stdout.flush()

            # Maintain control frequency
            time.sleep(loop_delay)

    except ConnectionError as e:
        print(f"\n❌ Connection Failed: Could not connect to motors on {args.port}")
        print(f"   Make sure motors are powered and port is correct")
        return

    except KeyboardInterrupt:
        print("\n\n[interrupt] Ctrl+C detected, stopping...")

    except Exception as e:
        print(f"\n❌ Unexpected Error: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # Stop camera
        if camera:
            camera.stop()

        # Cleanup motors
        if connected:
            print("\n[cleanup] Stopping motors...")
            try:
                controller.stop()
                controller.reset_to_position_mode()
                bus.disconnect()
            except:
                pass

        # Cleanup input
        input_handler.close()

        print("\n✅ Done. Recording saved and motors stopped.")


if __name__ == "__main__":
    main()
