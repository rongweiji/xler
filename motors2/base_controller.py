#!/usr/bin/env python
"""
LeKiwi Base Controller - 3-Wheel Omnidirectional Robot

This module implements the lekiwi_base kinematics approach using rotation
matrices for accurate 3-wheel omnidirectional robot control.

Based on: https://github.com/chongxi/egocentric_control/tree/master/real/lekiwi_base

Uses the same imports and pattern as lekiwi_base.py
"""

import logging
import time
from typing import Any

import numpy as np

from .motors_bus import Motor, MotorCalibration, MotorNormMode
from .feetech import FeetechMotorsBus, OperatingMode

logger = logging.getLogger(__name__)


class LeKiwiBaseController:
    """
    Three-wheel omnidirectional base controller with LeKiwi kinematics.

    Based on lekiwi_base approach but simplified - no Robot class inheritance.
    Uses rotation matrix to convert body frame velocities to wheel velocities.

    Example:
        config = load_config("motors2/config.yaml")

        # Create motors dict
        motors = {
            "base_left_wheel": Motor(1, "sts3215", MotorNormMode.DEGREES),
            "base_right_wheel": Motor(2, "sts3215", MotorNormMode.DEGREES),
            "base_back_wheel": Motor(3, "sts3215", MotorNormMode.DEGREES),
        }

        # Create bus
        bus = FeetechMotorsBus(
            port="/dev/cu.usbserial-XXX",
            motors=motors,
            protocol_version=0
        )
        bus.connect()

        # Create controller
        controller = LeKiwiBaseController(bus, config)
        controller.configure()

        # Move
        controller.move(x_vel=0.5, y_vel=0.0, theta_vel=0.0)

        controller.stop()
        bus.disconnect()
    """

    def __init__(self, bus: FeetechMotorsBus, config: dict):
        """
        Initialize the LeKiwi base controller.

        Args:
            bus: Connected FeetechMotorsBus instance
            config: Configuration dict loaded from config.yaml
        """
        self.bus = bus
        self.config = config

        # Extract configuration
        self.motor_ids_config = config['motor_ids']
        self.motor_config = config['motor']
        self.geometry = config['geometry']

        # Geometry parameters
        self.wheel_radius = self.geometry['wheel_radius_m']
        self.base_radius = self.geometry['base_radius_m']
        self.wheel_angles = np.asarray(self.geometry['wheel_axis_angles_deg'], dtype=float)
        self.max_wheel_raw = self.geometry['max_wheel_raw']

        # Motor names (must match bus.motors keys)
        self.base_motors = list(self.bus.motors.keys())

        logger.info(f"[LeKiwi] Initialized with motors: {self.base_motors}")
        logger.info(f"[LeKiwi] Geometry: wheel_r={self.wheel_radius}m, base_r={self.base_radius}m")
        logger.info(f"[LeKiwi] Wheel angles: {self.wheel_angles} deg")

    def configure(self):
        """
        Configure all motors for velocity mode.
        Based on lekiwi_base.configure()
        """
        logger.info("[LeKiwi] Configuring motors for velocity mode...")
        self.bus.disable_torque()
        self.bus.configure_motors()
        for name in self.base_motors:
            self.bus.write("Operating_Mode", name, OperatingMode.VELOCITY.value)
        self.bus.enable_torque()
        logger.info("[LeKiwi] Motors ready!")

    @staticmethod
    def _degps_to_raw(degps: float) -> int:
        """
        Convert degrees per second to raw motor speed units.
        From lekiwi_base._degps_to_raw()

        Args:
            degps: Speed in degrees per second

        Returns:
            Raw motor speed value
        """
        steps_per_deg = 4096.0 / 360.0
        speed_int = int(round(degps * steps_per_deg))
        return max(min(speed_int, 0x7FFF), -0x8000)

    @staticmethod
    def _raw_to_degps(raw_speed: int) -> float:
        """
        Convert raw motor speed to degrees per second.
        From lekiwi_base._raw_to_degps()

        Args:
            raw_speed: Raw motor speed value

        Returns:
            Speed in degrees per second
        """
        steps_per_deg = 4096.0 / 360.0
        return raw_speed / steps_per_deg

    def _body_to_wheel_raw(
        self,
        x: float,
        y: float,
        theta: float,
    ) -> dict[str, int]:
        """
        Convert body frame velocities to wheel raw values.
        Based on lekiwi_base._body_to_wheel_raw()

        Args:
            x: Forward velocity in m/s (positive = forward)
            y: Lateral velocity in m/s (positive = right)
            theta: Rotation velocity in deg/s (positive = CCW)

        Returns:
            Dict mapping motor names to raw velocity values
        """
        # Convert theta from deg/s to rad/s
        theta_rad = theta * (np.pi / 180.0)

        # Body velocity vector
        velocity_vector = np.array([x, y, theta_rad])

        # Build rotation matrix for 3-wheel configuration
        # Each row: [cos(angle), sin(angle), base_radius]
        angles = np.radians(self.wheel_angles)
        m = np.array([[np.cos(a), np.sin(a), self.base_radius] for a in angles])

        # Calculate wheel linear speeds (m/s)
        wheel_linear_speeds = m.dot(velocity_vector)

        # Convert to wheel angular speeds (rad/s)
        wheel_angular_speeds = wheel_linear_speeds / self.wheel_radius

        # Convert to degrees per second
        wheel_degps = wheel_angular_speeds * (180.0 / np.pi)

        # Calculate raw values (steps per second)
        steps_per_deg = 4096.0 / 360.0
        raw_floats = [abs(degps) * steps_per_deg for degps in wheel_degps]

        # Apply velocity scaling if any wheel exceeds max
        max_raw_computed = max(raw_floats) if raw_floats else 0
        if max_raw_computed > self.max_wheel_raw:
            scale = self.max_wheel_raw / max_raw_computed
            wheel_degps = wheel_degps * scale

        # Convert to raw integer values
        wheel_raw = [self._degps_to_raw(degps) for degps in wheel_degps]

        # Map to motor names (order: left, back, right based on wheel_angles)
        # Assumes self.base_motors is ordered as [left, right, back] or similar
        # We need to match the wheel_angles order
        return {
            "base_left_wheel": wheel_raw[0],
            "base_back_wheel": wheel_raw[1],
            "base_right_wheel": wheel_raw[2],
        }

    def _wheel_raw_to_body(
        self,
        left_wheel_speed: int,
        back_wheel_speed: int,
        right_wheel_speed: int,
    ) -> dict[str, float]:
        """
        Convert wheel raw velocities to body frame velocities.
        Based on lekiwi_base._wheel_raw_to_body()

        Args:
            left_wheel_speed: Left wheel raw speed
            back_wheel_speed: Back wheel raw speed
            right_wheel_speed: Right wheel raw speed

        Returns:
            Dict with x.vel (m/s), y.vel (m/s), theta.vel (deg/s)
        """
        # Convert raw to deg/s
        wheel_degps = np.array([
            self._raw_to_degps(left_wheel_speed),
            self._raw_to_degps(back_wheel_speed),
            self._raw_to_degps(right_wheel_speed),
        ])

        # Convert to rad/s
        wheel_radps = wheel_degps * (np.pi / 180.0)

        # Convert to linear speeds (m/s)
        wheel_linear_speeds = wheel_radps * self.wheel_radius

        # Build rotation matrix (same as forward kinematics)
        angles = np.radians(self.wheel_angles)
        m = np.array([[np.cos(a), np.sin(a), self.base_radius] for a in angles])

        # Inverse kinematics: wheel speeds → body velocities
        m_inv = np.linalg.inv(m)
        velocity_vector = m_inv.dot(wheel_linear_speeds)

        x, y, theta_rad = velocity_vector
        theta = theta_rad * (180.0 / np.pi)

        return {"x.vel": x, "y.vel": y, "theta.vel": theta}

    def get_observation(self) -> dict[str, float]:
        """
        Read actual velocities from motors and convert to body frame.
        Based on lekiwi_base.get_observation()

        Returns:
            Dict with actual x.vel (m/s), y.vel (m/s), theta.vel (deg/s)
        """
        # Read present velocities from all motors
        base_wheel_vel = self.bus.sync_read("Present_Velocity", self.base_motors)

        # Convert to body frame
        base_vel = self._wheel_raw_to_body(
            base_wheel_vel["base_left_wheel"],
            base_wheel_vel["base_back_wheel"],
            base_wheel_vel["base_right_wheel"],
        )

        return base_vel

    def move(
        self,
        x_vel: float = 0.0,
        y_vel: float = 0.0,
        theta_vel: float = 0.0
    ) -> dict[str, int]:
        """
        Move the robot base with specified body frame velocities.

        Args:
            x_vel: Forward/backward velocity in m/s (positive = forward)
            y_vel: Left/right velocity in m/s (positive = right)
            theta_vel: Rotation velocity in deg/s (positive = counter-clockwise)

        Returns:
            Dict of computed wheel velocities for debugging
        """
        # Calculate wheel velocities
        wheel_velocities = self._body_to_wheel_raw(x_vel, y_vel, theta_vel)

        # Send to motors using sync_write
        self.bus.sync_write("Goal_Velocity", wheel_velocities)

        return wheel_velocities

    def stop(self):
        """
        Stop all motors immediately.
        Based on lekiwi_base.stop_base()
        """
        self.bus.sync_write("Goal_Velocity", dict.fromkeys(self.base_motors, 0), num_retry=5)
        logger.info("[LeKiwi] Motors stopped")

    def reset_to_position_mode(self):
        """
        Reset all motors to position mode.
        For cleanup before disconnecting.
        """
        logger.info("[LeKiwi] Resetting to position mode...")
        self.stop()
        self.bus.disable_torque()
        time.sleep(0.1)

        for name in self.base_motors:
            self.bus.write("Operating_Mode", name, OperatingMode.POSITION.value)
            time.sleep(0.05)

        logger.info("[LeKiwi] Reset complete")
