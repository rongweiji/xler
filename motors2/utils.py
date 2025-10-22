"""Utility classes and functions for motor control"""

import sys


# Error classes
class DeviceAlreadyConnectedError(Exception):
    """Raised when trying to connect to an already connected device"""
    pass


class DeviceNotConnectedError(Exception):
    """Raised when trying to use a device that is not connected"""
    pass


# Utility functions
def enter_pressed():
    """Check if enter key was pressed (stub implementation)"""
    # Simple stub - always return False for non-interactive use
    return False


def move_cursor_up(lines=1):
    """Move terminal cursor up"""
    # ANSI escape code to move cursor up
    sys.stdout.write(f'\033[{lines}A')
    sys.stdout.flush()
