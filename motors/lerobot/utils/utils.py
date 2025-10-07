"""Stub utility functions for lerobot motors"""

import sys
import select

def enter_pressed():
    """Check if enter key was pressed (stub implementation)"""
    # Simple stub - always return False for non-interactive use
    return False

def move_cursor_up(lines=1):
    """Move terminal cursor up (stub implementation)"""
    # ANSI escape code to move cursor up
    sys.stdout.write(f'\033[{lines}A')
    sys.stdout.flush()
