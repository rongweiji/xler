#!/usr/bin/env python
"""
Basic Position Control Example

Simple example showing how to move servos to specific positions.

Example:
    python basic_position_control.py --port COM3
"""

import argparse
import sys
import time
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from motors.motors_bus import Motor, MotorNormMode
from motors.feetech import FeetechMotorsBus, OperatingMode


def main():
    parser = argparse.ArgumentParser(description="Basic STS3215 position control")
    parser.add_argument("--port", type=str, default="COM3", help="Serial port")
    parser.add_argument("--motor-ids", type=int, nargs="+", default=[1, 2, 3],
                       help="Motor IDs")
    args = parser.parse_args()

    print(f"\n=== Basic Position Control ===")
    print(f"Port: {args.port}")
    print(f"Motors: {args.motor_ids}\n")

    # Configure motors
    motors = {}
    for motor_id in args.motor_ids:
        motor_name = f"motor_{motor_id}"
        motors[motor_name] = Motor(
            id=motor_id,
            model="sts3215",
            norm_mode=MotorNormMode.DEGREES
        )

    # Connect to bus
    bus = FeetechMotorsBus(
        port=args.port,
        motors=motors,
        protocol_version=0
    )

    try:
        print("[init] Connecting to motors...")
        bus.connect()

        # Ensure motors are in position mode
        bus.disable_torque()
        for motor_name in motors.keys():
            bus.write("Operating_Mode", motor_name, OperatingMode.POSITION.value)
        bus.enable_torque()
        time.sleep(0.2)

        print("[test] Moving motors through positions...\n")

        # Example positions (in servo units: 0-1023)
        positions = [200, 512, 800, 512]  # Left, center, right, center

        for pos in positions:
            print(f"Moving all motors to position {pos}...")
            for motor_name in motors.keys():
                bus.write("Goal_Position", motor_name, pos)

            time.sleep(1.5)  # Wait for movement

        print("\n✅ Test complete!")

    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()

    finally:
        print("\n[cleanup] Disabling torque and disconnecting...")
        try:
            bus.disable_torque()
        except:
            pass
        bus.disconnect()
        print("✅ Done.")


if __name__ == "__main__":
    main()
