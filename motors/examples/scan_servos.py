#!/usr/bin/env python
"""
Scan and Detect Servos

Utility to scan for servos on the bus and display their information.

Example:
    python scan_servos.py --port COM3
    python scan_servos.py --port COM3 --max-id 10
"""

import argparse
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from motors.motors_bus import Motor, MotorNormMode
from motors.feetech import FeetechMotorsBus


def scan_for_servos(port, baud, max_id):
    """Scan for servos on the bus"""
    print(f"\n=== Scanning for Servos ===")
    print(f"Port: {port}")
    print(f"Baud: {baud}")
    print(f"Scanning IDs 1-{max_id}...\n")

    # Create a temporary motor configuration for scanning
    motors = {
        "scan_motor": Motor(
            id=1,  # Placeholder
            model="sts3215",
            norm_mode=MotorNormMode.DEGREES
        )
    }

    bus = FeetechMotorsBus(
        port=port,
        motors=motors,
        protocol_version=0
    )

    try:
        bus.connect()
        found_servos = []

        for servo_id in range(1, max_id + 1):
            # Try to ping this ID
            try:
                # Update the motor ID for this scan
                bus.motors["scan_motor"].id = servo_id

                # Try to read model number
                try:
                    model = bus.read("Model_Number", "scan_motor")
                    voltage = bus.read("Present_Voltage", "scan_motor")
                    temp = bus.read("Present_Temperature", "scan_motor")

                    print(f"✅ ID {servo_id:2d}: Model={model}, Voltage={voltage}, Temp={temp}°C")
                    found_servos.append(servo_id)
                except:
                    # Servo exists but can't read details
                    print(f"✅ ID {servo_id:2d}: Found (couldn't read details)")
                    found_servos.append(servo_id)

            except:
                # No servo at this ID
                pass

        print(f"\n=== Scan Complete ===")
        if found_servos:
            print(f"Found {len(found_servos)} servo(s): {found_servos}")
        else:
            print("No servos found")
            print("\nTroubleshooting:")
            print("  - Check power connections")
            print("  - Verify baud rate (default: 1000000)")
            print("  - Check serial port name")
            print("  - Try different cable")

    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()

    finally:
        bus.disconnect()


def main():
    parser = argparse.ArgumentParser(description="Scan for Feetech servos on the bus")
    parser.add_argument("--port", type=str, default="COM3",
                       help="Serial port (e.g., COM3 on Windows, /dev/ttyUSB0 on Linux)")
    parser.add_argument("--baud", type=int, default=1_000_000,
                       help="Baud rate (default: 1000000)")
    parser.add_argument("--max-id", type=int, default=8,
                       help="Maximum ID to scan (default: 8)")
    args = parser.parse_args()

    scan_for_servos(args.port, args.baud, args.max_id)


if __name__ == "__main__":
    main()
