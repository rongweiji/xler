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
from pydualsense import pydualsense


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
                tty.setcbreak(self._terminal_fd)
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
                if ch and len(ch) > 0:  # Check if we got a character
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
            print("[Calibration] Please release all joysticks to center position...")
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

    def get_controller_joystick_info(self) -> dict:
        """
        Get DualSense controller joystick information focused on 6 movement directions.
        
        Returns:
            Dict with joystick values and movement directions
        """
        if not self.dualsense_handler or not self.dualsense_handler.connected:
            return {}
        
        try:
            raw_values = self.dualsense_handler.get_raw_values()
            forward, strafe, rotate = self.dualsense_handler.read()
            
            return {
                # Raw joystick values (0-255, centered at 128)
                'raw': {
                    'left_x': raw_values['LX'],
                    'left_y': raw_values['LY'],
                    'right_x': raw_values['RX'],
                    'right_y': raw_values['RY'],
                },
                # Normalized movement values (-1.0 to 1.0)
                'movement': {
                    'forward': forward,
                    'strafe': strafe,
                    'rotate': rotate,
                },
                # Movement directions (boolean flags)
                'directions': {
                    'forward': forward > 0.1,
                    'backward': forward < -0.1,
                    'strafe_left': strafe < -0.1,
                    'strafe_right': strafe > 0.1,
                    'rotate_left': rotate < -0.1,
                    'rotate_right': rotate > 0.1,
                }
            }
        except Exception as e:
            return {'error': str(e)}


def main():
    """
    Test function for input_handler.py
    Run this file directly to test keyboard and DualSense controller inputs.
    """
    import argparse
    
    parser = argparse.ArgumentParser(description="Test input handler for keyboard and DualSense controller")
    parser.add_argument("--keyboard-only", action="store_true", help="Test keyboard input only")
    parser.add_argument("--controller-only", action="store_true", help="Test DualSense controller only")
    parser.add_argument("--show-joysticks", action="store_true", help="Show detailed joystick raw values")
    args = parser.parse_args()
    
    # Determine what to test
    enable_keyboard = not args.controller_only
    enable_dualsense = not args.keyboard_only
    
    print("=" * 80)
    print("🎮 INPUT HANDLER TEST")
    print("=" * 80)
    print()
    
    if enable_keyboard:
        print("⌨️  Keyboard Controls:")
        print("  W - Forward        S - Backward")
        print("  A - Strafe Left    D - Strafe Right") 
        print("  Q - Rotate Left    E - Rotate Right")
        print("  SPACE - Stop       ESC - Exit")
        print()
    
    if enable_dualsense:
        print("🎮 DualSense Controller:")
        print("  Left Stick (LX/LY) - Forward/Back + Strafe Left/Right")
        print("  Right Stick (RX)   - Rotate Left/Right")
        if args.show_joysticks:
            print("  Raw joystick values (0-255) will be displayed")
        print()
    
    print("Press ESC to exit the test")
    print("=" * 80)
    print()
    
    # Create input handler
    print("Initializing input handler...")
    handler = CombinedInputHandler(
        enable_keyboard=enable_keyboard,
        enable_dualsense=enable_dualsense
    )
    print("Input handler initialized.")
    
    # Check controller connection
    if enable_dualsense:
        if handler.has_controller():
            print("✅ DualSense controller connected!")
        else:
            print("❌ DualSense controller not connected")
            if not enable_keyboard:
                print("   Exiting since controller-only mode was requested")
                return
        print()
    
    # Add a clear separator before the main loop
    print("=" * 60)
    print("Starting input test...")
    print("=" * 60)
    
    # Test loop
    try:
        loop_count = 0
        last_state = None
        
        while True:
            loop_count += 1
            state = handler.read()
            
            # Check for exit
            if state.exit_requested:
                print("👋 Exiting test...")
                break
            
            # Only print when state changes or every 30 cycles
            state_changed = (state.forward != 0 or state.strafe != 0 or state.rotate != 0 or 
                           state.keyboard_key is not None or state.controller_active)
            
            if state_changed or loop_count % 30 == 0:
                # Build movement info
                movements = []
                if abs(state.forward) > 0.1:
                    movements.append(f"FWD:{state.forward:+.2f}" if state.forward > 0 else f"BWD:{abs(state.forward):.2f}")
                if abs(state.strafe) > 0.1:
                    movements.append(f"L:{abs(state.strafe):.2f}" if state.strafe < 0 else f"R:{state.strafe:.2f}")
                if abs(state.rotate) > 0.1:
                    movements.append(f"RL:{abs(state.rotate):.2f}" if state.rotate < 0 else f"RR:{state.rotate:.2f}")
                
                movement_str = ' | '.join(movements) if movements else 'None'
                
                # Build input source info
                sources = []
                if state.keyboard_key:
                    key_display = state.keyboard_key.upper() if state.keyboard_key != ' ' else 'SPACE'
                    sources.append(f"⌨️{key_display}")
                if state.controller_active:
                    sources.append("🎮")
                
                input_str = ' | '.join(sources) if sources else 'None'
                
                # Print single line with all info
                print(f"[{loop_count:4d}] Movement: {movement_str:<25} | Input: {input_str}")
                
                # Show detailed controller joystick info if requested (separate line)
                if args.show_joysticks and handler.has_controller():
                    joystick_info = handler.get_controller_joystick_info()
                    if 'error' not in joystick_info:
                        raw = joystick_info['raw']
                        print(f"      Controller: L({raw['left_x']:3d},{raw['left_y']:3d}) R({raw['right_x']:3d},{raw['right_y']:3d})")
                    else:
                        print(f"      ❌ Controller error: {joystick_info['error']}")
            
            # Small delay to prevent excessive CPU usage
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("👋 Test interrupted by user")
    
    finally:
        # Cleanup
        handler.close()
        print("✅ Input handler closed")


if __name__ == "__main__":
    main()
