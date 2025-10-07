#!/usr/bin/env python
"""
Test Base Movements (Non-Interactive)

Tests all 6 movement commands sequentially without keyboard input.
Useful for testing the base movement logic.

Example:
    python test_base_movements.py --port COM3
    python test_base_movements.py --port COM3 --speed 300 --duration 2
"""

import argparse
import sys
import time
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from motors.motors_bus import Motor, MotorNormMode
from motors.feetech import FeetechMotorsBus, OperatingMode


# Motor role mapping
MOTOR_ROLES = {
    "left": 1,    # Motor ID 1
    "right": 2,   # Motor ID 2
    "back": 3,    # Motor ID 3
}


def calculate_motor_velocities(command, speed):
    """Calculate motor velocities for each movement command"""
    velocities = {"left": 0, "right": 0, "back": 0}

    if command == 'forward':
        velocities["left"] = speed
        velocities["right"] = speed
        velocities["back"] = 0

    elif command == 'back':
        velocities["left"] = -speed
        velocities["right"] = -speed
        velocities["back"] = 0

    elif command == 'left':
        velocities["left"] = 0
        velocities["right"] = 0
        velocities["back"] = speed

    elif command == 'right':
        velocities["left"] = 0
        velocities["right"] = 0
        velocities["back"] = -speed

    elif command == 'rotate_left':
        velocities["left"] = -speed
        velocities["right"] = speed
        velocities["back"] = 0

    elif command == 'rotate_right':
        velocities["left"] = speed
        velocities["right"] = -speed
        velocities["back"] = 0

    return velocities


def setup_motors(bus):
    """Configure all motors for velocity mode"""
    print("[setup] Configuring motors for velocity mode...")
    bus.disable_torque()
    time.sleep(0.1)

    for role, motor_id in MOTOR_ROLES.items():
        motor_name = f"motor_{motor_id}"
        bus.write("Operating_Mode", motor_name, OperatingMode.VELOCITY.value)
        bus.write("Torque_Limit", motor_name, 700)
        time.sleep(0.05)

    bus.enable_torque()
    time.sleep(0.1)
    print("[setup] Motors ready!\n")


def stop_all_motors(bus):
    """Stop all motors with retries"""
    for role, motor_id in MOTOR_ROLES.items():
        motor_name = f"motor_{motor_id}"
        for _ in range(5):
            bus.write("Goal_Velocity", motor_name, 0)
            time.sleep(0.01)
    time.sleep(0.3)


def send_velocities(bus, velocities):
    """Send velocity commands to motors"""
    for role, velocity in velocities.items():
        motor_id = MOTOR_ROLES[role]
        motor_name = f"motor_{motor_id}"
        bus.write("Goal_Velocity", motor_name, velocity)


def test_movement(bus, command, speed, duration):
    """Test a single movement command"""
    velocities = calculate_motor_velocities(command, speed)

    print(f"[{command.upper()}]")
    print(f"  Left motor (ID 1):  {velocities['left']:+5d}")
    print(f"  Right motor (ID 2): {velocities['right']:+5d}")
    print(f"  Back motor (ID 3):  {velocities['back']:+5d}")
    print(f"  Running for {duration}s...")

    send_velocities(bus, velocities)
    time.sleep(duration)

    print(f"  Stopping...\n")
    stop_all_motors(bus)


def main():
    parser = argparse.ArgumentParser(description="Test 3-wheel base movements sequentially")
    parser.add_argument("--port", type=str, default="COM3", help="Serial port")
    parser.add_argument("--speed", type=int, default=400, help="Movement speed (0-1023)")
    parser.add_argument("--duration", type=float, default=3.0, help="Duration per movement (seconds)")
    parser.add_argument("--motor-left", type=int, default=1, help="Left motor ID")
    parser.add_argument("--motor-right", type=int, default=2, help="Right motor ID")
    parser.add_argument("--motor-back", type=int, default=3, help="Back motor ID")
    args = parser.parse_args()

    # Update motor mapping
    MOTOR_ROLES["left"] = args.motor_left
    MOTOR_ROLES["right"] = args.motor_right
    MOTOR_ROLES["back"] = args.motor_back

    print("\n=== 3-Wheel Base Movement Test ===")
    print(f"Port: {args.port}")
    print(f"Speed: {args.speed}")
    print(f"Duration: {args.duration}s per movement")
    print(f"Motors: Left={args.motor_left}, Right={args.motor_right}, Back={args.motor_back}\n")

    # Create motor configuration
    motors = {}
    for role, motor_id in MOTOR_ROLES.items():
        motor_name = f"motor_{motor_id}"
        motors[motor_name] = Motor(
            id=motor_id,
            model="sts3215",
            norm_mode=MotorNormMode.DEGREES
        )

    # Connect
    bus = FeetechMotorsBus(
        port=args.port,
        motors=motors,
        protocol_version=0
    )

    try:
        bus.connect()
        setup_motors(bus)

        # Test all movements
        movements = [
            'forward',
            'back',
            'left',
            'right',
            'rotate_left',
            'rotate_right'
        ]

        print("Testing all movements:\n")
        for i, movement in enumerate(movements, 1):
            print(f"--- Test {i}/6 ---")
            test_movement(bus, movement, args.speed, args.duration)
            time.sleep(0.5)  # Pause between movements

        print("✅ All movements tested successfully!")

    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()

    finally:
        print("\n[cleanup] Stopping and resetting motors...")
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

        bus.disconnect()
        print("✅ Done.")


if __name__ == "__main__":
    main()
