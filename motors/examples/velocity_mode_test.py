#!/usr/bin/env python
"""
Velocity Mode Test for STS3215 Servos

Test servos using lerobot motors library in velocity mode.
Runs servos one-by-one with same speed, torque, and duration.

Example:
    python velocity_mode_test.py --port COM3 --motor-ids 1 2 3 --velocity 600
"""

import argparse
import sys
import time
from pathlib import Path

# Add parent directory to path so we can import motors package
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from motors.motors_bus import Motor, MotorNormMode
from motors.feetech import FeetechMotorsBus, OperatingMode

def main():
    ap = argparse.ArgumentParser(description="STS3215 velocity mode tester using lerobot motors")
    ap.add_argument("--port", type=str, default="COM3", help="Serial port")
    ap.add_argument("--baud", type=int, default=1_000_000, help="Baud rate")
    ap.add_argument("--motor-ids", type=int, nargs="+", default=[1, 2, 3], help="Motor IDs to test")
    ap.add_argument("--velocity", type=int, default=600, help="Rotation velocity (0-1023)")
    ap.add_argument("--torque-limit", type=int, default=700, help="Torque limit (0-1023)")
    ap.add_argument("--duration", type=float, default=5.0, help="Rotation duration per servo (seconds)")
    args = ap.parse_args()

    print(f"\n=== LeRobot Feetech Motor Test ===")
    print(f"Port: {args.port}, Baud: {args.baud}")
    print(f"Motors: {args.motor_ids}")
    print(f"Velocity: {args.velocity}, Torque: {args.torque_limit}, Duration: {args.duration}s\n")

    # Create motor configuration
    motors = {}
    for motor_id in args.motor_ids:
        motor_name = f"motor_{motor_id}"
        motors[motor_name] = Motor(
            id=motor_id,
            model="sts3215",
            norm_mode=MotorNormMode.DEGREES  # Use degrees normalization
        )

    # Initialize motor bus
    print("[init] Connecting to motor bus...")
    bus = FeetechMotorsBus(
        port=args.port,
        motors=motors,
        protocol_version=0  # STS3215 uses protocol version 0
    )

    try:
        print("[init] Connecting...")
        bus.connect()

        print("[init] Setting all motors to velocity mode...")
        # Disable torque before changing mode
        bus.disable_torque()
        time.sleep(0.1)

        # Set all motors to velocity mode
        for motor_name in motors.keys():
            bus.write("Operating_Mode", motor_name, OperatingMode.VELOCITY.value)
            time.sleep(0.05)

        # Set torque limit for all motors
        for motor_name in motors.keys():
            bus.write("Torque_Limit", motor_name, args.torque_limit)
            time.sleep(0.05)

        # Enable torque
        bus.enable_torque()
        time.sleep(0.1)

        print("\n[test] Starting one-by-one rotation test...\n")

        # Test each motor one by one
        for motor_id in args.motor_ids:
            motor_name = f"motor_{motor_id}"

            print(f"[motor {motor_id}] Rotating at velocity={args.velocity} for {args.duration}s...")

            # Write velocity to this motor
            bus.write("Goal_Velocity", motor_name, args.velocity)
            time.sleep(args.duration)

            # Stop this motor with retries (like XLeRobot does)
            print(f"[motor {motor_id}] Stopping...")
            for _ in range(5):
                bus.write("Goal_Velocity", motor_name, 0)
                time.sleep(0.02)

            time.sleep(0.3)  # Wait for motor to fully stop
            print(f"[motor {motor_id}] Done.\n")

        print("✅ All motors tested successfully!")

        # Final cleanup - stop all motors
        print("\n[cleanup] Stopping all motors...")
        stop_commands = {motor_name: 0 for motor_name in motors.keys()}

        # Write 0 to all motors with retries
        for _ in range(5):
            for motor_name in motors.keys():
                bus.write("Goal_Velocity", motor_name, 0)
                time.sleep(0.01)

        time.sleep(0.3)

        # Disable torque
        print("[cleanup] Disabling torque...")
        bus.disable_torque()
        time.sleep(0.1)

        # Switch back to position mode
        print("[cleanup] Resetting to position mode...")
        for motor_name in motors.keys():
            bus.write("Operating_Mode", motor_name, OperatingMode.POSITION.value)
            time.sleep(0.05)

        print("✅ Cleanup complete.")

    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()

        # Emergency stop
        print("\n[emergency] Stopping all motors...")
        try:
            for motor_name in motors.keys():
                for _ in range(10):
                    bus.write("Goal_Velocity", motor_name, 0)
                    time.sleep(0.01)
            bus.disable_torque()
        except:
            pass

    finally:
        print("\n[disconnect] Closing connection...")
        bus.disconnect()
        print("✅ Done.")

if __name__ == "__main__":
    main()
