#!/usr/bin/env python
"""
Keyboard & DualSense Controller for 3-Wheel Omnidirectional Base

Real-time control for robot base with 3 servos in velocity mode.
Control runs at 20Hz - motors move when key HELD or joystick moved.

Motor Configuration:
- Motor ID 1: Left wheel
- Motor ID 2: Right wheel
- Motor ID 3: Back wheel

Keyboard Commands (HOLD to move, RELEASE to stop):
    W - Forward  (left & right motors forward)
    S - Back     (left & right motors backward)
    A - Left     (back motor one direction)
    D - Right    (back motor opposite direction)
    Q - Rotate Left  (left & right motors opposite directions)
    E - Rotate Right (left & right motors opposite directions)
    SPACE - Force stop all motors
    ESC - Exit

DualSense Controller (PS5):
    Left Stick (LX/LY) - Forward/Back + Strafe Left/Right
    Right Stick (RX)   - Rotate Left/Right

Example:
    # Auto-detect serial port
    python keyboard_controller_base_control.py --speed 400

    # Or manually specify port
    python keyboard_controller_base_control.py --port /dev/cu.usbserial-XXXXX --speed 400
"""

import argparse
import sys
import time
import glob
import platform
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from motors.motors_bus import Motor, MotorNormMode
from motors.feetech import FeetechMotorsBus
from motors.movement_controller import OmniBaseController
from motors.input_handler import CombinedInputHandler
from motors.config_loader import load_motor_config, print_motor_config, validate_motor_config


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


def print_controls(has_controller):
    """Print control instructions"""
    print("\n" + "="*60)
    print("  3-WHEEL OMNIDIRECTIONAL BASE CONTROL (20Hz)")
    print("="*60)
    print("\n📐 Motor Configuration:")
    print("  • Motor 1 (Left)  - Left wheel")
    print("  • Motor 2 (Right) - Right wheel")
    print("  • Motor 3 (Back)  - Back wheel")
    print("\n⌨️  Keyboard Controls (HOLD key to move, RELEASE to stop):")
    print("  W - Forward       (left & right forward)")
    print("  S - Backward      (left & right backward)")
    print("  A - Strafe Left   (back wheel rotates)")
    print("  D - Strafe Right  (back wheel opposite)")
    print("  Q - Rotate Left   (left back, right forward)")
    print("  E - Rotate Right  (left forward, right back)")
    print("  SPACE - Force STOP")
    print("  ESC - Exit program")

    if has_controller:
        print("\n🎮 DualSense Controller (PS5):")
        print("  Left Stick (LX/LY)  - Forward/Back + Strafe Left/Right")
        print("  Right Stick (RX)    - Rotate Left/Right")
        print("  💡 Controller input is combined with keyboard input!")

    print("\n💡 Tip: Motors stop IMMEDIATELY when you release controls!")
    print("="*60 + "\n")


def main():
    parser = argparse.ArgumentParser(description="Keyboard control for 3-wheel omnidirectional base")
    parser.add_argument("--port", type=str, default=None, help="Serial port (overrides config.yaml)")
    parser.add_argument("--speed", type=int, default=None, help="Base movement speed (overrides config.yaml)")
    parser.add_argument("--motor-left", type=int, default=None, help="Left motor ID (overrides config.yaml)")
    parser.add_argument("--motor-right", type=int, default=None, help="Right motor ID (overrides config.yaml)")
    parser.add_argument("--motor-back", type=int, default=None, help="Back motor ID (overrides config.yaml)")
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

    # Debug: Show speed being used
    print(f"[config] Using speed: {speed} (max_velocity: {motor_settings['max_velocity']})")
    print(f"[config] Speed multiplier will be: {speed / motor_settings['max_velocity']:.4f}")
    print(f"[config] Expected motor velocity: ~{int(speed)} when input is 1.0")

    # Auto-detect port if not specified in args or config
    port = args.port if args.port is not None else serial_config['port']
    if port is None:
        print("[init] Auto-detecting serial port...")
        port = find_serial_port()
        if port is None:
            print("❌ Error: Could not auto-detect serial port.")
            print("   Please connect your USB-to-Serial adapter and try again.")
            print("   Or manually specify the port with --port option or in motors/config.yaml")
            print()
            print("   On macOS, check available ports with: ls /dev/cu.*")
            print("   On Linux, check: ls /dev/ttyUSB*")
            print("   On Windows, use: COM3, COM4, etc.")
            return
        print(f"✅ Found serial port: {port}")
    else:
        print(f"[init] Using serial port: {port}")

    # Create motor configuration
    motors = {}
    for role, motor_id in motor_ids.items():
        motor_name = f"motor_{motor_id}"
        motors[motor_name] = Motor(
            id=motor_id,
            model=motor_settings['model'],
            norm_mode=MotorNormMode.DEGREES
        )

    # Initialize input handler
    print()
    print("[init] Checking for DualSense controller...")
    input_handler = CombinedInputHandler(enable_keyboard=True, enable_dualsense=True)

    if input_handler.has_controller():
        print("🎮 Input Mode: Keyboard + DualSense Controller")
    else:
        print("⌨️  Input Mode: Keyboard Only")

    # Connect to bus
    print()
    print(f"[init] Connecting to motors on {port}...")
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

        # Print controls
        print_controls(input_handler.has_controller())

        print("Ready! Press and HOLD keys to control (release to stop)...")
        print()
        print(f"ℹ️  Control runs at {control_config['frequency']}Hz - motors stop immediately when controls released")
        print(f"📊 Command stream display: Shows every {control_config['print_interval']} cycles")
        print()

        loop_counter = 0
        control_frequency = control_config['frequency']
        loop_delay = 1.0 / control_frequency
        print_interval = control_config['print_interval']

        print(f"{'Cycle':<6} {'Input':<15} {'Fwd':<6} {'Str':<6} {'Rot':<6} {'Left':<6} {'Right':<6} {'Back':<6}")
        print("-" * 75)

        while True:
            loop_counter += 1

            # Read inputs
            state = input_handler.read()

            # Check for exit
            if state.exit_requested:
                print("\n[exit] ESC pressed, stopping motors...")
                break

            # Send movement command
            # Pass speed directly as absolute motor speed value (e.g., 400)
            velocities = controller.move(
                forward=state.forward,
                strafe=state.strafe,
                rotate=state.rotate,
                speed=speed  # Absolute speed value (0-1023)
            )

            # DEBUG: Print actual values on first movement
            if loop_counter == 1 and (abs(state.forward) > 0 or abs(state.strafe) > 0 or abs(state.rotate) > 0):
                print()
                print(f"[DEBUG] Input: fwd={state.forward:.2f}, strafe={state.strafe:.2f}, rot={state.rotate:.2f}")
                print(f"[DEBUG] Speed parameter: {speed}")
                print(f"[DEBUG] Motor velocities: left={velocities['left']}, right={velocities['right']}, back={velocities['back']}")

            # Print command stream at regular intervals
            if loop_counter % print_interval == 0:
                # Determine input source for display
                has_kb = state.keyboard_key is not None
                has_ctrl = state.controller_active

                if has_kb and has_ctrl:
                    input_display = "KB+Ctrl"
                elif has_kb:
                    input_display = f"KB:{state.keyboard_key.upper()}"
                elif has_ctrl:
                    input_display = "Ctrl"
                else:
                    input_display = "none"

                # Print formatted output
                print(f"{loop_counter:<6d} {input_display:<15} "
                      f"{state.forward:+.2f}  {state.strafe:+.2f}  {state.rotate:+.2f}  "
                      f"{velocities['left']:+6d} {velocities['right']:+6d} {velocities['back']:+6d}")

            # Sleep to maintain control frequency
            time.sleep(loop_delay)

    except ConnectionError as e:
        print()
        print(f"❌ Connection Failed: Could not connect to motors on {port}")
        print("   Make sure:")
        print("   • Motors are powered on")
        print("   • USB cable is connected")
        print("   • Port is correct (on macOS, try: ls /dev/tty.* | grep usb)")
        print()
        print("   Hint: COM3 is a Windows port. On macOS, use /dev/tty.usbserial-* or similar")
        return

    except KeyboardInterrupt:
        print()
        print("[interrupt] Ctrl+C detected, stopping motors...")

    except Exception as e:
        print()
        print(f"❌ Unexpected Error: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # Cleanup
        if connected:
            print()
            print("[cleanup] Stopping all motors...")
            try:
                controller.stop()
                controller.reset_to_position_mode()
                bus.disconnect()
            except:
                pass

        # Close input handler
        input_handler.close()

        print("✅ Done. Motors stopped and disconnected.")


if __name__ == "__main__":
    main()
