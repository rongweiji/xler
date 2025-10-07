# emergency_stop.py
# Emergency stop all servos - disables torque on all IDs

import argparse
import time
from feetech_core import SerialBus, Servo

def main():
    ap = argparse.ArgumentParser(description="Emergency stop all servos")
    ap.add_argument("--port", type=str, default="COM3", help="Serial port")
    ap.add_argument("--baud", type=int, default=1_000_000, help="Baud rate")
    ap.add_argument("--max-id", type=int, default=10, help="Stop IDs 1..max-id")
    args = ap.parse_args()

    print(f"[emergency stop] Opening {args.port}...")
    bus = SerialBus(args.port, baud=args.baud, timeout=0.1)

    try:
        print(f"[stop] Force stopping servos 1..{args.max_id}...")
        for sid in range(1, args.max_id + 1):
            servo = Servo(id=sid)
            try:
                # Write 0 to Goal_Velocity with retries (XLeRobot pattern)
                servo.stop_rotation(bus, num_retries=10)
                time.sleep(0.3)  # Wait for motor to stop

                # Disable torque AFTER writing 0
                servo.disable_torque(bus)
                time.sleep(0.1)

                # Try to reset to position mode
                try:
                    servo.set_operating_mode(bus, 0)
                    time.sleep(0.1)
                except:
                    pass

                print(f"  ✅ Stopped servo {sid}")
            except Exception as e:
                print(f"  ⚠️  Servo {sid}: {e}")

        print("\n✅ All servos force stopped and torque disabled")
    finally:
        bus.close()

if __name__ == "__main__":
    main()
