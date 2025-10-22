"""
motors2 - Simple 3-Wheel Base Control with LeKiwi Kinematics

This package provides a simplified motor control interface using the
lekiwi_base kinematics approach with proper rotation matrix calculations.

Key components:
- base_controller.py: LeKiwi kinematics-based 3-wheel controller
- keyboard_input.py: Simple WASD+QE keyboard input handler
- config.yaml: Configuration file for motor IDs and settings
"""

from .base_controller import LeKiwiBaseController

__all__ = ["LeKiwiBaseController"]
