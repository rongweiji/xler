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
from motors.feetech import FeetechMotorsBus, OperatingMode

# Try to import DualSense controller
try:
    from pydualsense import pydualsense
    DUALSENSE_AVAILABLE = True
except ImportError:
    DUALSENSE_AVAILABLE = False
    print("⚠️  pydualsense not installed. Controller support disabled.")


# Motor role mapping
MOTOR_ROLES = {
    "left": 1,    # Motor ID 1
    "right": 2,   # Motor ID 2
    "back": 3,    # Motor ID 3
}


# Terminal settings cache for Unix systems
_terminal_settings = None
_terminal_fd = None

def setup_terminal():
    """Setup terminal for raw input (Unix/Mac only)"""
    global _terminal_settings, _terminal_fd
    if sys.platform != 'win32':
        import tty
        import termios
        _terminal_fd = sys.stdin.fileno()
        _terminal_settings = termios.tcgetattr(_terminal_fd)
        tty.setraw(_terminal_fd)

def restore_terminal():
    """Restore terminal to normal mode (Unix/Mac only)"""
    global _terminal_settings, _terminal_fd
    if sys.platform != 'win32' and _terminal_settings is not None:
        import termios
        termios.tcsetattr(_terminal_fd, termios.TCSADRAIN, _terminal_settings)

def get_key_non_blocking():
    """
    Get keyboard input in non-blocking mode.
    Returns currently pressed key or None if no key pressed.

    IMPORTANT: Clears entire keyboard buffer to prevent lag when key is released.
    """
    if sys.platform == 'win32':
        import msvcrt

        last_key = None

        # Clear entire buffer - read all pending characters
        while msvcrt.kbhit():
            key = msvcrt.getch()
            # Handle special keys
            if key == b'\x1b':  # ESC
                last_key = 'esc'
            elif key == b' ':  # SPACE
                last_key = ' '
            else:
                try:
                    last_key = key.decode('utf-8').lower()
                except:
                    pass

        return last_key
    else:
        # Linux/Mac: use select for non-blocking input
        import select

        last_key = None

        # Clear entire buffer
        while True:
            rlist, _, _ = select.select([sys.stdin], [], [], 0)
            if rlist:
                ch = sys.stdin.read(1)
                if ord(ch) == 27:  # ESC
                    last_key = 'esc'
                elif ch == ' ':  # SPACE
                    last_key = ' '
                else:
                    last_key = ch.lower()
            else:
                break

        return last_key


def init_controller():
    """
    Try to initialize DualSense controller.
    Returns tuple: (controller object, calibration offsets) if successful, (None, None) otherwise.
    """
    if not DUALSENSE_AVAILABLE:
        return None, None

    try:
        ds = pydualsense()
        ds.init()

        print("✅ DualSense controller connected!")

        # Set LED to blue to indicate connection
        try:
            ds.light.setColorI(0, 0, 255)
        except:
            pass

        # Calibrate joysticks
        print("\n[Calibration] Please release all joysticks to center position...")
        print("[Calibration] Calibrating in 2 seconds...")
        time.sleep(2)

        # Read center position multiple times and average
        lx_samples = []
        ly_samples = []
        rx_samples = []
        ry_samples = []

        for i in range(10):
            lx_samples.append(ds.state.LX)
            ly_samples.append(ds.state.LY)
            rx_samples.append(ds.state.RX)
            ry_samples.append(ds.state.RY)
            time.sleep(0.05)

        # Calculate offsets (how much to subtract to get to 128)
        calibration = {
            'LX': sum(lx_samples) // len(lx_samples) - 128,
            'LY': sum(ly_samples) // len(ly_samples) - 128,
            'RX': sum(rx_samples) // len(rx_samples) - 128,
            'RY': sum(ry_samples) // len(ry_samples) - 128,
        }

        print(f"[Calibration] ✅ Complete!")
        print(f"[Calibration] Offsets - LX:{calibration['LX']:+4d} LY:{calibration['LY']:+4d} RX:{calibration['RX']:+4d} RY:{calibration['RY']:+4d}")

        return ds, calibration

    except Exception as e:
        print(f"⚠️  Controller init failed: {e}")
        return None, None


def read_controller_input(ds, calibration=None, deadzone=20):
    """
    Read DualSense controller joystick values and convert to velocity components.

    Args:
        ds: DualSense controller object
        calibration: Dict with calibration offsets for LX, LY, RX, RY
        deadzone: Deadzone threshold (0-128) to ignore stick drift

    Returns:
        Tuple of (forward_vel, strafe_vel, rotate_vel) normalized -1.0 to 1.0
        Returns (0, 0, 0) if controller disconnected
    """
    if ds is None:
        return (0.0, 0.0, 0.0)

    try:
        # Read joystick values (0-255, centered at 128)
        lx_raw = ds.state.LX
        ly_raw = ds.state.LY
        rx_raw = ds.state.RX
        ry_raw = ds.state.RY

        # Apply calibration offsets
        if calibration:
            lx = lx_raw - 128 - calibration['LX']
            ly = ly_raw - 128 - calibration['LY']
            rx = rx_raw - 128 - calibration['RX']
            ry = ry_raw - 128 - calibration['RY']
        else:
            lx = lx_raw - 128
            ly = ly_raw - 128
            rx = rx_raw - 128
            ry = ry_raw - 128

        # Apply deadzone
        if abs(lx) < deadzone:
            lx = 0
        if abs(ly) < deadzone:
            ly = 0
        if abs(rx) < deadzone:
            rx = 0
        if abs(ry) < deadzone:
            ry = 0

        # Normalize to -1.0 to 1.0
        forward_vel = -ly / 128.0   # Negative because stick up = negative Y
        strafe_vel = lx / 128.0     # Right = positive
        rotate_vel = rx / 128.0     # Right = positive rotation

        return (forward_vel, strafe_vel, rotate_vel)

    except Exception as e:
        print(f"⚠️  Controller read error: {e}")
        return (0.0, 0.0, 0.0)


def calculate_motor_velocities_from_components(forward, strafe, rotate, max_speed):
    """
    Calculate motor velocities from velocity components.

    Args:
        forward: Forward/back component (-1.0 to 1.0)
        strafe: Left/right strafe component (-1.0 to 1.0)
        rotate: Rotation component (-1.0 to 1.0)
        max_speed: Maximum motor speed (0-1023)

    Returns:
        Dict with motor roles mapped to velocities
    """
    velocities = {
        "left": int((forward + rotate) * max_speed),
        "right": int((forward - rotate) * max_speed),
        "back": int(strafe * max_speed)
    }

    # Clamp to valid range
    for key in velocities:
        velocities[key] = max(-1023, min(1023, velocities[key]))

    return velocities


def calculate_motor_velocities(command, speed):
    """
    Calculate motor velocities for keyboard commands.

    Returns dict with motor roles mapped to velocities.
    Positive = CW, Negative = CCW, 0 = stop
    """
    velocities = {"left": 0, "right": 0, "back": 0}

    if command == 'q':  # rotate left(CCW)
        velocities["left"] = speed
        velocities["right"] = speed
        velocities["back"] = 0

    elif command == 'e':  # rotate right(CW)
        velocities["left"] = -speed
        velocities["right"] = -speed
        velocities["back"] = 0

    elif command == 'a':  # Left strafe
        velocities["left"] = 0
        velocities["right"] = 0
        velocities["back"] = speed

    elif command == 'd':  # Right strafe
        velocities["left"] = 0
        velocities["right"] = 0
        velocities["back"] = -speed

    elif command == 'w':  # forward
        velocities["left"] = -speed
        velocities["right"] = speed
        velocities["back"] = 0

    elif command == 's':  # backward
        velocities["left"] = speed
        velocities["right"] = -speed
        velocities["back"] = 0

    elif command == ' ':  # Stop
        velocities["left"] = 0
        velocities["right"] = 0
        velocities["back"] = 0

    return velocities


def setup_motors(bus):
    """Configure all motors for velocity mode"""
    print("[setup] Configuring motors for velocity mode...")

    # Disable torque before mode change
    bus.disable_torque()
    time.sleep(0.1)

    # Set all motors to velocity mode
    for role, motor_id in MOTOR_ROLES.items():
        motor_name = f"motor_{motor_id}"
        bus.write("Operating_Mode", motor_name, OperatingMode.VELOCITY.value)
        bus.write("Torque_Limit", motor_name, 700)
        time.sleep(0.05)

    # Enable torque
    bus.enable_torque()
    time.sleep(0.1)
    print("[setup] Motors ready!")


def stop_all_motors(bus):
    """Stop all motors with retries"""
    for role, motor_id in MOTOR_ROLES.items():
        motor_name = f"motor_{motor_id}"
        for _ in range(5):
            bus.write("Goal_Velocity", motor_name, 0)
            time.sleep(0.01)
    time.sleep(0.2)


def send_velocities(bus, velocities):
    """Send velocity commands to motors based on role mapping"""
    for role, velocity in velocities.items():
        motor_id = MOTOR_ROLES[role]
        motor_name = f"motor_{motor_id}"
        bus.write("Goal_Velocity", motor_name, velocity)


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


def find_serial_port():
    """
    Automatically detect the serial port for USB-to-Serial adapter.

    Returns:
        str: The detected port path, or None if not found
    """
    system = platform.system()

    if system == "Darwin":  # macOS
        # Common patterns for USB serial adapters on macOS
        patterns = [
            "/dev/cu.usbserial*",
            "/dev/cu.SLAB_USBtoUART*",
            "/dev/cu.wchusbserial*",
            "/dev/cu.usbmodem*",
        ]
        for pattern in patterns:
            ports = glob.glob(pattern)
            if ports:
                return ports[0]  # Return first match

    elif system == "Linux":
        # Common patterns for USB serial adapters on Linux
        patterns = [
            "/dev/ttyUSB*",
            "/dev/ttyACM*",
        ]
        for pattern in patterns:
            ports = glob.glob(pattern)
            if ports:
                return ports[0]

    elif system == "Windows":
        # On Windows, try common COM ports
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        if ports:
            return ports[0].device

    return None


def main():
    parser = argparse.ArgumentParser(description="Keyboard control for 3-wheel omnidirectional base")
    parser.add_argument("--port", type=str, default=None, help="Serial port (auto-detected if not specified)")
    parser.add_argument("--speed", type=int, default=400, help="Base movement speed (0-1023)")
    parser.add_argument("--motor-left", type=int, default=1, help="Left motor ID (default: 1)")
    parser.add_argument("--motor-right", type=int, default=2, help="Right motor ID (default: 2)")
    parser.add_argument("--motor-back", type=int, default=3, help="Back motor ID (default: 3)")
    args = parser.parse_args()

    # Auto-detect port if not specified
    if args.port is None:
        print("[init] Auto-detecting serial port...")
        args.port = find_serial_port()
        if args.port is None:
            print("❌ Error: Could not auto-detect serial port.")
            print("   Please connect your USB-to-Serial adapter and try again.")
            print("   Or manually specify the port with --port option.")
            print("\n   On macOS, check available ports with: ls /dev/cu.*")
            print("   On Linux, check: ls /dev/ttyUSB*")
            print("   On Windows, use: COM3, COM4, etc.")
            return
        print(f"✅ Found serial port: {args.port}")

    # Update motor role mapping if custom IDs provided
    MOTOR_ROLES["left"] = args.motor_left
    MOTOR_ROLES["right"] = args.motor_right
    MOTOR_ROLES["back"] = args.motor_back

    # Create motor configuration
    motors = {}
    for role, motor_id in MOTOR_ROLES.items():
        motor_name = f"motor_{motor_id}"
        motors[motor_name] = Motor(
            id=motor_id,
            model="sts3215",
            norm_mode=MotorNormMode.DEGREES
        )

    # Initialize controller
    print("\n[init] Checking for DualSense controller...")
    controller, calibration = init_controller()

    if controller:
        print("🎮 Input Mode: Keyboard + DualSense Controller")
    else:
        print("⌨️  Input Mode: Keyboard Only")

    # Connect to bus
    print(f"\n[init] Connecting to motors on {args.port}...")
    bus = FeetechMotorsBus(
        port=args.port,
        motors=motors,
        protocol_version=0
    )

    # Setup terminal for keyboard input (macOS/Linux)
    setup_terminal()

    connected = False
    try:
        bus.connect()
        connected = True
        setup_motors(bus)
        print_controls(controller is not None)

        print("Ready! Press and HOLD keys to control (release to stop)...\n")
        print("ℹ️  Control runs at 20Hz - motors stop immediately when controls released")
        print("📊 Command stream display: Shows every 10th cycle (0.5s intervals)\n")

        loop_counter = 0
        control_frequency = 20  # Hz
        loop_delay = 1.0 / control_frequency  # 0.05 seconds = 50ms
        print_interval = 10  # Print every 10 loops (0.5 seconds at 20Hz)

        command_names = {
            'w': 'FORWARD',
            's': 'BACKWARD',
            'a': 'LEFT',
            'd': 'RIGHT',
            'q': 'ROTATE LEFT',
            'e': 'ROTATE RIGHT',
            ' ': 'STOP',
            None: 'STOPPED'  # No key pressed
        }

        print(f"{'Cycle':<6} {'Input':<10} {'Command':<15} {'Left':<6} {'Right':<6} {'Back':<6}")
        print("-" * 60)

        while True:
            loop_counter += 1

            # Get current key state (non-blocking)
            key = get_key_non_blocking()

            # Check for ESC to exit
            if key == 'esc':
                print("\n[exit] ESC pressed, stopping motors...")
                break

            # Read controller input
            controller_forward, controller_strafe, controller_rotate = read_controller_input(controller, calibration)

            # Determine keyboard command
            current_command = None
            if key in ['w', 'a', 's', 'd', 'q', 'e', ' ']:
                current_command = key

            # Calculate velocities from keyboard
            if current_command:
                kb_velocities = calculate_motor_velocities(current_command, args.speed)
            else:
                kb_velocities = {"left": 0, "right": 0, "back": 0}

            # Calculate velocities from controller
            ctrl_velocities = calculate_motor_velocities_from_components(
                controller_forward, controller_strafe, controller_rotate, args.speed
            )

            # Combine keyboard and controller inputs (additive)
            velocities = {
                "left": kb_velocities["left"] + ctrl_velocities["left"],
                "right": kb_velocities["right"] + ctrl_velocities["right"],
                "back": kb_velocities["back"] + ctrl_velocities["back"]
            }

            # Clamp to valid range
            for key_name in velocities:
                velocities[key_name] = max(-1023, min(1023, velocities[key_name]))

            # Always send velocities (even if zero) to maintain control
            send_velocities(bus, velocities)

            # Print command stream at regular intervals
            if loop_counter % print_interval == 0:
                # Determine input source for display
                has_kb = current_command is not None
                has_ctrl = abs(controller_forward) > 0.01 or abs(controller_strafe) > 0.01 or abs(controller_rotate) > 0.01

                if has_kb and has_ctrl:
                    input_display = f"KB+Ctrl"
                elif has_kb:
                    input_display = f"KB:{key}"
                elif has_ctrl:
                    input_display = "Ctrl"
                else:
                    input_display = "none"

                cmd_name = command_names.get(current_command, 'JOYSTICK' if has_ctrl else 'STOPPED')

                print(f"{loop_counter:<6d} {input_display:<10} {cmd_name:<15} {velocities['left']:+6d} {velocities['right']:+6d} {velocities['back']:+6d}")

            # Sleep to maintain control frequency (20Hz)
            time.sleep(loop_delay)

    except ConnectionError as e:
        print(f"\n❌ Connection Failed: Could not connect to motors on {args.port}")
        print(f"   Make sure:")
        print(f"   • Motors are powered on")
        print(f"   • USB cable is connected")
        print(f"   • Port is correct (on macOS, try: ls /dev/tty.* | grep usb)")
        print(f"\n   Hint: COM3 is a Windows port. On macOS, use /dev/tty.usbserial-* or similar")
        return

    except KeyboardInterrupt:
        print("\n\n[interrupt] Ctrl+C detected, stopping motors...")

    except Exception as e:
        print(f"\n❌ Unexpected Error: {e}")

    finally:
        # Restore terminal settings
        restore_terminal()

        if connected:
            print("\n[cleanup] Stopping all motors...")
            try:
                stop_all_motors(bus)
                bus.disable_torque()
                time.sleep(0.1)

                # Reset to position mode
                for role, motor_id in MOTOR_ROLES.items():
                    motor_name = f"motor_{motor_id}"
                    bus.write("Operating_Mode", motor_name, OperatingMode.POSITION.value)
                    time.sleep(0.05)
            except:
                pass

            try:
                bus.disconnect()
            except:
                pass

        # Close controller if connected
        if controller is not None:
            try:
                controller.close()
                print("✅ Controller disconnected.")
            except:
                pass

        print("✅ Done. Motors stopped and disconnected.")


if __name__ == "__main__":
    main()
