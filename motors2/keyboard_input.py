#!/usr/bin/env python
"""
Simple Keyboard Input Handler for Robot Control

Provides non-blocking keyboard input for WASD+QE control.
Supports Windows, macOS, and Linux.

Controls:
- W: Forward
- S: Backward
- A: Strafe Left
- D: Strafe Right
- Q: Rotate Left
- E: Rotate Right
- SPACE: Stop (all zeros)
- ESC: Exit

Based on motors/input_handler.py but simplified for keyboard-only use.
"""

import sys
import platform


class KeyboardInput:
    """
    Simple keyboard input handler for WASD+QE robot control.

    Reads keyboard input in a non-blocking manner and converts it to
    velocity commands (forward, strafe, rotate).
    """

    def __init__(self):
        """Initialize keyboard handler based on platform."""
        self.system = platform.system()

        # Platform-specific setup
        if self.system == "Windows":
            import msvcrt
            self._read_key = self._read_key_windows
        elif self.system in ["Linux", "Darwin"]:  # Darwin = macOS
            import tty
            import termios
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            self._read_key = self._read_key_unix
        else:
            raise RuntimeError(f"Unsupported platform: {self.system}")

        print(f"[KeyboardInput] Initialized for {self.system}")

    def _read_key_windows(self) -> str:
        """Read a single keypress on Windows (non-blocking)."""
        import msvcrt
        if msvcrt.kbhit():
            key = msvcrt.getch()
            # Handle special keys (arrows, etc.)
            if key in [b'\x00', b'\xe0']:
                msvcrt.getch()  # Read and discard the second byte
                return ''
            try:
                return key.decode('utf-8').lower()
            except:
                return ''
        return ''

    def _read_key_unix(self) -> str:
        """Read a single keypress on Unix/Linux/macOS (non-blocking)."""
        import select
        # Check if data is available to read (non-blocking)
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            # Handle escape sequences (arrow keys, etc.)
            if key == '\x1b':  # ESC sequence
                # Try to read next chars for arrow keys
                if select.select([sys.stdin], [], [], 0.01)[0]:
                    sys.stdin.read(1)  # Read '['
                    if select.select([sys.stdin], [], [], 0.01)[0]:
                        sys.stdin.read(1)  # Read direction
                    return 'esc'  # Treat as ESC for simplicity
                return 'esc'
            return key.lower()
        return ''

    def read(self) -> tuple[float, float, float, bool]:
        """
        Read keyboard input and return velocity commands.

        Returns:
            Tuple of (forward, strafe, rotate, exit_requested)
            - forward: -1.0 to 1.0 (W/S keys)
            - strafe: -1.0 to 1.0 (A/D keys)
            - rotate: -1.0 to 1.0 (Q/E keys)
            - exit_requested: True if ESC pressed
        """
        forward = 0.0
        strafe = 0.0
        rotate = 0.0
        exit_requested = False

        # Read all available keys (empty buffer)
        keys_pressed = set()
        while True:
            key = self._read_key()
            if not key:
                break
            keys_pressed.add(key)

        # Process keys
        for key in keys_pressed:
            if key == 'w':
                forward = 1.0
            elif key == 's':
                forward = -1.0
            elif key == 'a':
                strafe = -1.0
            elif key == 'd':
                strafe = 1.0
            elif key == 'q':
                rotate = 1.0
            elif key == 'e':
                rotate = -1.0
            elif key == ' ':
                # Space = force stop
                forward = 0.0
                strafe = 0.0
                rotate = 0.0
            elif key == 'esc' or key == '\x1b':
                exit_requested = True

        return forward, strafe, rotate, exit_requested

    def close(self):
        """Restore terminal settings on Unix systems."""
        if self.system in ["Linux", "Darwin"]:
            import termios
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            print("\n[KeyboardInput] Terminal settings restored")

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()
