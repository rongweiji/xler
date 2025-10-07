# test_servo_sequence.py
# Run a 1-by-1 continuous rotation test over detected servos.
# Usage:
#   python test_servo_sequence.py --port COM3 --max-id 3
#   python test_servo_sequence.py --port COM3 --velocity 300 --torque 500 --duration 5

import argparse
import time

from feetech_core import (
    SerialBus,
    Servo,
    find_feetech_port,
    scan_ids,
)

def main():
    ap = argparse.ArgumentParser(description="STS3215 continuous rotation tester (one-by-one)")
    ap.add_argument("--port", type=str, default=None, help="Serial port (auto-detect if omitted)")
    ap.add_argument("--baud", type=int, default=1_000_000, help="Baud rate")
    ap.add_argument("--max-id", type=int, default=8, help="Scan IDs from 1..max-id")
    ap.add_argument("--velocity", type=int, default=200, help="Rotation velocity (0-1023)")
    ap.add_argument("--torque", type=int, default=300, help="Torque limit (0-1023)")
    ap.add_argument("--duration", type=float, default=5.0, help="Rotation duration per servo (seconds)")
    ap.add_argument("--direction", type=str, default="cw", choices=["cw", "ccw"], help="Rotation direction")
    args = ap.parse_args()

    port = args.port or find_feetech_port(baud=args.baud, verbose=True)
    if not port:
        print("❌ Could not auto-detect port. Provide --port explicitly.")
        return

    print(f"\n[open] using port={port}, baud={args.baud}")
    bus = SerialBus(port, baud=args.baud, timeout=0.1)

    try:
        print(f"[scan] scanning IDs 1..{args.max_id} ...")
        ids = scan_ids(bus, 1, args.max_id, verbose=True)
        if not ids:
            print("❌ no servos found in that range.")
            return

        direction_cw = (args.direction == "cw")
        direction_str = "clockwise" if direction_cw else "counter-clockwise"

        print(f"\n=== Continuous Rotation Test (One-by-One) ===")
        print(f"Velocity: {args.velocity}, Torque: {args.torque}, Duration: {args.duration}s, Direction: {direction_str}")
        print(f"Found {len(ids)} servos: {ids}\n")

        # Configure all servos first (set to velocity mode with same torque)
        print("[setup] Configuring all servos for velocity mode...")
        for sid in ids:
            servo = Servo(id=sid)

            # Disable torque before changing mode
            print(f"  [servo {sid}] disabling torque...")
            servo.disable_torque(bus)
            time.sleep(0.05)

            # Set velocity mode (mode=1)
            print(f"  [servo {sid}] setting velocity mode...")
            servo.set_operating_mode(bus, 1)
            time.sleep(0.05)

            # Set torque limit
            print(f"  [servo {sid}] setting torque limit={args.torque}...")
            servo.set_torque_limit(bus, args.torque)
            time.sleep(0.05)

            # Enable torque
            print(f"  [servo {sid}] enabling torque...")
            servo.enable_torque(bus)
            time.sleep(0.05)

        print("\n[test] Starting rotation sequence...\n")

        # Now rotate each servo one-by-one
        for sid in ids:
            servo = Servo(id=sid)

            print(f"[servo {sid}] rotating at velocity={args.velocity} ({direction_str}) for {args.duration}s...")
            servo.rotate_continuous(bus, args.velocity, direction_cw)
            time.sleep(args.duration)

            print(f"[servo {sid}] stopping...")
            # Stop rotation - write 0 to Goal_Velocity with retries (like XLeRobot does)
            servo.stop_rotation(bus, num_retries=5)
            time.sleep(0.3)  # Wait for motor to actually stop
            # Disable torque after motor has stopped
            servo.disable_torque(bus)
            time.sleep(0.1)

        print("\n✅ All servos tested. Stopping and resetting all servos...")

        # Stop, disable, and reset to position mode
        for sid in ids:
            servo = Servo(id=sid)
            print(f"  [servo {sid}] stopping (writing 0 to Goal_Velocity)...")

            # Stop with retries (like XLeRobot's stop_base method)
            servo.stop_rotation(bus, num_retries=5)
            time.sleep(0.3)  # Wait for motor to process stop command

            # Disable torque AFTER motor has stopped
            print(f"  [servo {sid}] disabling torque...")
            servo.disable_torque(bus)
            time.sleep(0.1)

            # Switch back to position mode (requires torque disabled)
            print(f"  [servo {sid}] resetting to position mode...")
            servo.set_operating_mode(bus, 0)  # 0 = position mode
            time.sleep(0.1)

        print("✅ Done. All servos stopped and reset to position mode.")
    finally:
        # Emergency stop - ensure all servos are stopped before closing
        print("\n[cleanup] Emergency stop all servos...")
        try:
            for sid in range(1, args.max_id + 1):
                servo = Servo(id=sid)
                # Write 0 to Goal_Velocity with retries (XLeRobot pattern)
                servo.stop_rotation(bus, num_retries=10)
                time.sleep(0.2)
                # Disable torque AFTER writing 0
                servo.disable_torque(bus)
                time.sleep(0.05)
                try:
                    servo.set_operating_mode(bus, 0)  # Reset to position mode
                    time.sleep(0.05)
                except:
                    pass
        except:
            pass
        bus.close()
        print("[close] serial closed.")

if __name__ == "__main__":
    main()
