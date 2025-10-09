"""
Input Handler for Robot Control

Provides abstraction for reading keyboard and DualSense controller inputs.
Supports non-blocking keyboard reading and controller calibration.

Example:
    from motors.input_handler import CombinedInputHandler

    # Create handler with both keyboard and controller
    handler = CombinedInputHandler(enable_keyboard=True, enable_dualsense=True)

    # Read inputs in control loop
    while True:
        state = handler.read()
        if state.exit_requested:
            break

        print(f"Forward: {state.forward}, Strafe: {state.strafe}, Rotate: {state.rotate}")
        print(handler.get_debug_string())
"""

import sys
import time
from dataclasses import dataclass
from typing import Optional, Tuple

# Try to import DualSense controller
try:
    from pydualsense import pydualsense
    DUALSENSE_AVAILABLE = True
except ImportError:
    DUALSENSE_AVAILABLE = False


@dataclass
class InputState:
    """Represents the current input state from all sources."""
    forward: float = 0.0        # -1.0 to 1.0 (negative = backward, positive = forward)
    strafe: float = 0.0         # -1.0 to 1.0 (negative = left, positive = right)
    rotate: float = 0.0         # -1.0 to 1.0 (negative = CCW, positive = CW)
    speed_multiplier: float = 1.0  # Speed multiplier (typically 0.5-2.0)
    exit_requested: bool = False   # True if ESC pressed or exit signal received
    keyboard_key: Optional[str] = None  # Current keyboard key pressed
    controller_active: bool = False     # True if controller has non-zero input


class KeyboardHandler:
    """
    Non-blocking keyboard input handler.
    Supports Windows, macOS, and Linux.
    """

    def __init__(self):
        self._terminal_settings = None
        self._terminal_fd = None
        self._setup_terminal()

    def _setup_terminal(self):
        """Setup terminal for raw input (Unix/Mac only)"""
        if sys.platform != 'win32':
            try:
                import tty
                import termios
                self._terminal_fd = sys.stdin.fileno()
                self._terminal_settings = termios.tcgetattr(self._terminal_fd)
                tty.setraw(self._terminal_fd)
            except:
                pass

    def restore_terminal(self):
        """Restore terminal to normal mode (Unix/Mac only)"""
        if sys.platform != 'win32' and self._terminal_settings is not None:
            try:
                import termios
                termios.tcsetattr(self._terminal_fd, termios.TCSADRAIN, self._terminal_settings)
            except:
                pass

    def read_key(self) -> Optional[str]:
        """
        Read keyboard input in non-blocking mode.

        Returns:
            Currently pressed key or None if no key pressed.
            Returns 'esc' for ESC key, ' ' for SPACE.
        """
        if sys.platform == 'win32':
            return self._read_key_windows()
        else:
            return self._read_key_unix()

    def _read_key_windows(self) -> Optional[str]:
        """Windows-specific non-blocking keyboard read"""
        import msvcrt

        last_key = None

        # Clear entire buffer - read all pending characters
        while msvcrt.kbhit():
            key = msvcrt.getch()
            # Handle special keys
            if key == b'\x1b':  # ESC
                last_key = 'esc'
            elif key == b' ':  # SPACE
                last_key = ' '
            else:
                try:
                    last_key = key.decode('utf-8').lower()
                except:
                    pass

        return last_key

    def _read_key_unix(self) -> Optional[str]:
        """Unix/Mac-specific non-blocking keyboard read"""
        import select

        last_key = None

        # Clear entire buffer
        while True:
            rlist, _, _ = select.select([sys.stdin], [], [], 0)
            if rlist:
                ch = sys.stdin.read(1)
                if ord(ch) == 27:  # ESC
                    last_key = 'esc'
                elif ch == ' ':  # SPACE
                    last_key = ' '
                else:
                    last_key = ch.lower()
            else:
                break

        return last_key

    def __del__(self):
        """Cleanup: restore terminal settings"""
        self.restore_terminal()


class DualSenseHandler:
    """
    DualSense (PS5) controller handler with calibration support.
    """

    def __init__(self, deadzone=20, auto_calibrate=True):
        """
        Initialize DualSense controller handler.

        Args:
            deadzone: Joystick deadzone threshold (0-128), default 20
            auto_calibrate: Automatically calibrate on init, default True
        """
        self.controller = None
        self.calibration = None
        self.deadzone = deadzone
        self.connected = False

        if not DUALSENSE_AVAILABLE:
            print("⚠️  pydualsense not installed. Controller support disabled.")
            return

        if auto_calibrate:
            self._init_and_calibrate()

    def _init_and_calibrate(self):
        """Initialize and calibrate the controller"""
        try:
            self.controller = pydualsense()
            self.controller.init()
            self.connected = True

            print("✅ DualSense controller connected!")

            # Set LED to blue to indicate connection
            try:
                self.controller.light.setColorI(0, 0, 255)
            except:
                pass

            # Calibrate joysticks
            print("\n[Calibration] Please release all joysticks to center position...")
            print("[Calibration] Calibrating in 2 seconds...")
            time.sleep(2)

            # Read center position multiple times and average
            lx_samples, ly_samples, rx_samples, ry_samples = [], [], [], []

            for _ in range(10):
                lx_samples.append(self.controller.state.LX)
                ly_samples.append(self.controller.state.LY)
                rx_samples.append(self.controller.state.RX)
                ry_samples.append(self.controller.state.RY)
                time.sleep(0.05)

            # Calculate offsets (how much to subtract to get to 128)
            self.calibration = {
                'LX': sum(lx_samples) // len(lx_samples) - 128,
                'LY': sum(ly_samples) // len(ly_samples) - 128,
                'RX': sum(rx_samples) // len(rx_samples) - 128,
                'RY': sum(ry_samples) // len(ry_samples) - 128,
            }

            print(f"[Calibration] ✅ Complete!")
            print(f"[Calibration] Offsets - LX:{self.calibration['LX']:+4d} LY:{self.calibration['LY']:+4d} "
                  f"RX:{self.calibration['RX']:+4d} RY:{self.calibration['RY']:+4d}")

        except Exception as e:
            print(f"⚠️  Controller init failed: {e}")
            self.connected = False
            self.controller = None

    def read(self) -> Tuple[float, float, float]:
        """
        Read controller joystick values and convert to velocity components.

        Returns:
            Tuple of (forward_vel, strafe_vel, rotate_vel) normalized -1.0 to 1.0
            Returns (0, 0, 0) if controller not connected
        """
        if not self.connected or self.controller is None:
            return (0.0, 0.0, 0.0)

        try:
            # Read joystick values (0-255, centered at 128)
            lx_raw = self.controller.state.LX
            ly_raw = self.controller.state.LY
            rx_raw = self.controller.state.RX
            ry_raw = self.controller.state.RY

            # Apply calibration offsets
            if self.calibration:
                lx = lx_raw - 128 - self.calibration['LX']
                ly = ly_raw - 128 - self.calibration['LY']
                rx = rx_raw - 128 - self.calibration['RX']
                ry = ry_raw - 128 - self.calibration['RY']
            else:
                lx = lx_raw - 128
                ly = ly_raw - 128
                rx = rx_raw - 128
                ry = ry_raw - 128

            # Apply deadzone
            if abs(lx) < self.deadzone:
                lx = 0
            if abs(ly) < self.deadzone:
                ly = 0
            if abs(rx) < self.deadzone:
                rx = 0
            if abs(ry) < self.deadzone:
                ry = 0

            # Normalize to -1.0 to 1.0
            forward_vel = -ly / 128.0   # Negative because stick up = negative Y
            strafe_vel = lx / 128.0     # Right = positive
            rotate_vel = rx / 128.0     # Right = positive rotation

            return (forward_vel, strafe_vel, rotate_vel)

        except Exception as e:
            print(f"⚠️  Controller read error: {e}")
            return (0.0, 0.0, 0.0)

    def get_raw_values(self) -> dict:
        """
        Get raw joystick values for debugging.

        Returns:
            Dict with raw LX, LY, RX, RY values (0-255)
        """
        if not self.connected or self.controller is None:
            return {"LX": 128, "LY": 128, "RX": 128, "RY": 128}

        try:
            return {
                "LX": self.controller.state.LX,
                "LY": self.controller.state.LY,
                "RX": self.controller.state.RX,
                "RY": self.controller.state.RY,
            }
        except:
            return {"LX": 128, "LY": 128, "RX": 128, "RY": 128}

    def close(self):
        """Close the controller connection"""
        if self.controller is not None:
            try:
                self.controller.close()
                print("✅ DualSense controller disconnected.")
            except:
                pass
            self.connected = False
            self.controller = None


class CombinedInputHandler:
    """
    Combined input handler that merges keyboard and DualSense controller inputs.
    """

    def __init__(self, enable_keyboard=True, enable_dualsense=True):
        """
        Initialize combined input handler.

        Args:
            enable_keyboard: Enable keyboard input, default True
            enable_dualsense: Enable DualSense controller input, default True
        """
        self.enable_keyboard = enable_keyboard
        self.enable_dualsense = enable_dualsense

        self.keyboard_handler = KeyboardHandler() if enable_keyboard else None
        self.dualsense_handler = DualSenseHandler() if enable_dualsense else None

        self.last_key = None
        self.last_controller_values = (0.0, 0.0, 0.0)

    def read(self) -> InputState:
        """
        Read combined input state from all enabled sources.

        Returns:
            InputState object with merged inputs
        """
        state = InputState()

        # Read keyboard
        if self.keyboard_handler:
            key = self.keyboard_handler.read_key()
            self.last_key = key

            if key == 'esc':
                state.exit_requested = True
                return state

            # Convert keyboard to velocity components
            kb_forward, kb_strafe, kb_rotate = self._keyboard_to_components(key)
            state.forward += kb_forward
            state.strafe += kb_strafe
            state.rotate += kb_rotate
            state.keyboard_key = key

        # Read controller
        if self.dualsense_handler and self.dualsense_handler.connected:
            ctrl_forward, ctrl_strafe, ctrl_rotate = self.dualsense_handler.read()
            self.last_controller_values = (ctrl_forward, ctrl_strafe, ctrl_rotate)

            state.forward += ctrl_forward
            state.strafe += ctrl_strafe
            state.rotate += ctrl_rotate

            # Check if controller has non-zero input
            if abs(ctrl_forward) > 0.01 or abs(ctrl_strafe) > 0.01 or abs(ctrl_rotate) > 0.01:
                state.controller_active = True

        # Clamp final values to -1.0 to 1.0
        state.forward = max(-1.0, min(1.0, state.forward))
        state.strafe = max(-1.0, min(1.0, state.strafe))
        state.rotate = max(-1.0, min(1.0, state.rotate))

        return state

    def _keyboard_to_components(self, key: Optional[str]) -> Tuple[float, float, float]:
        """
        Convert keyboard command to velocity components.

        Args:
            key: Keyboard key pressed

        Returns:
            Tuple of (forward, strafe, rotate) normalized to -1.0 to 1.0
        """
        forward, strafe, rotate = 0.0, 0.0, 0.0

        if key == 'w':  # forward
            forward = 1.0
        elif key == 's':  # backward
            forward = -1.0
        elif key == 'a':  # left strafe
            strafe = -1.0
        elif key == 'd':  # right strafe
            strafe = 1.0
        elif key == 'q':  # rotate left (CCW)
            rotate = -1.0
        elif key == 'e':  # rotate right (CW)
            rotate = 1.0
        elif key == ' ':  # stop
            forward, strafe, rotate = 0.0, 0.0, 0.0

        return (forward, strafe, rotate)

    def get_debug_string(self) -> str:
        """
        Get formatted debug string showing current input state.

        Returns:
            Formatted string with keyboard and controller info
        """
        parts = []

        # Keyboard info
        if self.keyboard_handler and self.last_key:
            key_display = self.last_key.upper() if self.last_key != ' ' else 'SPACE'
            parts.append(f"[KB: {key_display}]")

        # Controller info
        if self.dualsense_handler and self.dualsense_handler.connected:
            fwd, strafe, rot = self.last_controller_values
            if abs(fwd) > 0.01 or abs(strafe) > 0.01 or abs(rot) > 0.01:
                parts.append(f"[DS: Fwd:{fwd:+.2f} Str:{strafe:+.2f} Rot:{rot:+.2f}]")

        if not parts:
            return "[No Input]"

        return " ".join(parts)

    def has_controller(self) -> bool:
        """Check if DualSense controller is connected"""
        return self.dualsense_handler is not None and self.dualsense_handler.connected

    def close(self):
        """Cleanup: close all input handlers"""
        if self.keyboard_handler:
            self.keyboard_handler.restore_terminal()

        if self.dualsense_handler:
            self.dualsense_handler.close()
