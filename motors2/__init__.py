"""
motors2 - Self-Contained 3-Wheel Base Control with LeKiwi Kinematics

This package provides complete motor control with LeRobot motor abstractions
and LeKiwi kinematics for 3-wheel omnidirectional robots.

Key components:
- motors_bus.py: Base motor bus abstraction (from LeRobot)
- feetech/: Feetech motor implementation (from LeRobot)
- base_controller.py: LeKiwi kinematics controller
- keyboard_input.py: Simple WASD+QE keyboard input
- config.yaml: Configuration file
"""

# Export motor bus classes
from .motors_bus import Motor, MotorCalibration, MotorNormMode, MotorsBus, NameOrID, Value

# Export LeKiwi controller
from .base_controller import LeKiwiBaseController

# Export visualizer
from .visualizer import PathVisualizer

# Export hardware utilities
from .hardware_utils import find_serial_port, list_serial_ports

__all__ = [
    # Motor classes
    "Motor",
    "MotorCalibration",
    "MotorNormMode",
    "MotorsBus",
    "NameOrID",
    "Value",
    # Controllers
    "LeKiwiBaseController",
    # Visualization
    "PathVisualizer",
    # Hardware utilities
    "find_serial_port",
    "list_serial_ports",
]
