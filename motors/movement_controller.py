"""
Movement Controller for 3-Wheel Omnidirectional Base

Provides high-level movement control abstraction for robot base with
3 servos in velocity mode.

Motor Configuration:
- Motor ID 1: Left wheel
- Motor ID 2: Right wheel
- Motor ID 3: Back wheel

Example:
    from motors.feetech import FeetechMotorsBus, OperatingMode
    from motors.movement_controller import OmniBaseController

    # Initialize bus (see examples for setup)
    bus = FeetechMotorsBus(...)

    # Create controller
    controller = OmniBaseController(
        bus=bus,
        motor_ids={"left": 1, "right": 2, "back": 3},
        max_velocity=1023
    )

    # Move the base
    controller.move(forward=0.5, strafe=0.3, rotate=0.0, speed=1.0)
    controller.stop()
"""

import time
from motors.feetech import FeetechMotorsBus, OperatingMode


class OmniBaseController:
    """
    High-level controller for 3-wheel omnidirectional base.

    Handles kinematics calculation and motor command generation for
    a robot base with 3 wheels arranged in omnidirectional configuration.
    """

    def __init__(self, bus, motor_ids=None, max_velocity=1023):
        """
        Initialize the omnidirectional base controller.

        Args:
            bus: FeetechMotorsBus instance (must be connected)
            motor_ids: Dict mapping roles to motor IDs, e.g. {"left": 1, "right": 2, "back": 3}
                      Defaults to {left: 1, right: 2, back: 3}
            max_velocity: Maximum motor velocity (0-1023), default 1023
        """
        self.bus = bus
        self.motor_ids = motor_ids or {"left": 1, "right": 2, "back": 3}
        self.max_velocity = max_velocity

        # Validate motor IDs
        required_roles = {"left", "right", "back"}
        if set(self.motor_ids.keys()) != required_roles:
            raise ValueError(f"motor_ids must contain exactly these roles: {required_roles}")

    def setup_motors(self, torque_limit=700):
        """
        Configure all motors for velocity mode.

        Args:
            torque_limit: Torque limit for motors (0-1023), default 700
        """
        print("[OmniBase] Configuring motors for velocity mode...")

        # Disable torque before mode change
        self.bus.disable_torque()
        time.sleep(0.1)

        # Set all motors to velocity mode
        for role, motor_id in self.motor_ids.items():
            motor_name = f"motor_{motor_id}"
            self.bus.write("Operating_Mode", motor_name, OperatingMode.VELOCITY.value)
            self.bus.write("Torque_Limit", motor_name, torque_limit)
            time.sleep(0.05)

        # Enable torque
        self.bus.enable_torque()
        time.sleep(0.1)
        print("[OmniBase] Motors ready!")

    def move(self, forward=0.0, strafe=0.0, rotate=0.0, speed=None):
        """
        Move the base with specified velocity components.

        Args:
            forward: Forward/backward velocity (-1.0 to 1.0)
                    Positive = forward, Negative = backward
            strafe: Left/right strafe velocity (-1.0 to 1.0)
                   Positive = right, Negative = left
            rotate: Rotation velocity (-1.0 to 1.0)
                   Positive = rotate right (CW), Negative = rotate left (CCW)
            speed: Absolute speed value (0-1023). If None, uses self.max_velocity
                  This is the actual motor speed to use, NOT a multiplier
        """
        # Use provided speed or default to max_velocity
        max_speed = speed if speed is not None else self.max_velocity

        # Omnidirectional kinematics for 3-wheel configuration
        # Left and right wheels contribute to forward/backward and rotation
        # Back wheel handles strafe
        # NOTE: Motor wiring specifics from old working code:
        # - Left motor: negated (reversed wiring)
        # - Back motor: negated (reversed wiring)
        velocities = {
            "left": int(-(forward + rotate) * max_speed),   # Negated for left motor wiring
            "right": int((forward - rotate) * max_speed),   # Normal for right motor
            "back": int(strafe * max_speed)                # Negated for back motor wiring
        }

        # Clamp to valid range (-1023 to 1023)
        for role in velocities:
            velocities[role] = max(-1023, min(1023, velocities[role]))

        # Send commands to motors
        self._send_velocities(velocities)

        return velocities


    def stop(self):
        """Stop all motors immediately."""
        velocities = {"left": 0, "right": 0, "back": 0}

        # Send stop command multiple times for reliability
        for _ in range(5):
            self._send_velocities(velocities)
            time.sleep(0.01)
        time.sleep(0.2)

    def reset_to_position_mode(self):
        """
        Reset all motors to position mode.
        Useful for cleanup when exiting velocity control.
        """
        print("[OmniBase] Resetting motors to position mode...")
        self.stop()
        self.bus.disable_torque()
        time.sleep(0.1)

        for role, motor_id in self.motor_ids.items():
            motor_name = f"motor_{motor_id}"
            self.bus.write("Operating_Mode", motor_name, OperatingMode.POSITION.value)
            time.sleep(0.05)

    def _send_velocities(self, velocities):
        """
        Internal method to send velocity commands to motors.

        Args:
            velocities: Dict with roles mapped to velocity values
        """
        for role, velocity in velocities.items():
            motor_id = self.motor_ids[role]
            motor_name = f"motor_{motor_id}"
            self.bus.write("Goal_Velocity", motor_name, velocity)
