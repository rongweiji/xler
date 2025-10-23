#!/usr/bin/env python
"""
Console.py - Central Control Panel for Robot Navigation

Real-time robot control with occupancy map visualization.

Features:
- Display occupancy map with robot position and orientation
- Real-time path tracking as robot moves
- Keyboard control (WASD+QE) for robot movement
- Arrow keys + comma/period to adjust robot pose on map
- Collision detection to prevent movement into obstacles
- Robot footprint visualization

Controls:
    MOVEMENT (Robot control):
    - W/A/S/D: Forward/Left/Back/Right
    - Q/E: Rotate Left/Right
    - SPACE: Stop

    POSE ADJUSTMENT (Map position):
    - Arrow Up/Down: Move robot position on map (Y axis)
    - Arrow Left/Right: Move robot position on map (X axis)
    - , (comma): Rotate robot on map (CCW)
    - . (period): Rotate robot on map (CW)
    - R: Reset robot to initial pose

    OTHER:
    - ESC: Exit

Usage:
    python Console.py
    python Console.py --port /dev/cu.usbserial-XXXXX
    python Console.py --no-motors  # Map visualization only (no hardware)
"""

import argparse
import logging
import sys
import time
import numpy as np
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent))

import yaml
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrow, Circle
from matplotlib.backends.backend_agg import FigureCanvasAgg
import matplotlib.patches as mpatches

# Import from motors2
from motors2 import Motor, MotorNormMode, find_serial_port
from motors2.feetech import FeetechMotorsBus
from motors2.base_controller import LeKiwiBaseController
from motors2.map_loader import OccupancyMap

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class ConsoleKeyboardInput:
    """
    Enhanced keyboard input for robot control + pose adjustment.

    Supports:
    - WASD+QE for robot movement
    - Arrow keys for pose translation
    - Comma/period for pose rotation
    - R for reset
    """

    def __init__(self):
        """Initialize keyboard handler."""
        import platform
        self.system = platform.system()

        if self.system == "Windows":
            import msvcrt
            self._read_key = self._read_key_windows
        elif self.system in ["Linux", "Darwin"]:
            import tty
            import termios
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            self._read_key = self._read_key_unix
        else:
            raise RuntimeError(f"Unsupported platform: {self.system}")

    def _read_key_windows(self) -> str:
        """Read keypress on Windows."""
        import msvcrt
        if msvcrt.kbhit():
            key = msvcrt.getch()
            if key in [b'\x00', b'\xe0']:  # Special key
                key2 = msvcrt.getch()
                # Arrow keys
                if key2 == b'H': return 'up'
                if key2 == b'P': return 'down'
                if key2 == b'K': return 'left'
                if key2 == b'M': return 'right'
                return ''
            try:
                return key.decode('utf-8').lower()
            except:
                return ''
        return ''

    def _read_key_unix(self) -> str:
        """Read keypress on Unix/macOS."""
        import select
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key == '\x1b':  # ESC sequence
                if select.select([sys.stdin], [], [], 0.01)[0]:
                    next_char = sys.stdin.read(1)
                    if next_char == '[':  # Arrow key
                        if select.select([sys.stdin], [], [], 0.01)[0]:
                            arrow = sys.stdin.read(1)
                            if arrow == 'A': return 'up'
                            if arrow == 'B': return 'down'
                            if arrow == 'C': return 'right'
                            if arrow == 'D': return 'left'
                    return 'esc'
                return 'esc'
            return key.lower()
        return ''

    def read(self):
        """
        Read keyboard and return control commands.

        Returns:
            Tuple: (forward, strafe, rotate,
                   pose_dx, pose_dy, pose_dtheta,
                   reset_requested, exit_requested)
        """
        forward = 0.0
        strafe = 0.0
        rotate = 0.0
        pose_dx = 0.0
        pose_dy = 0.0
        pose_dtheta = 0.0
        reset_requested = False
        exit_requested = False

        # Read all available keys
        keys_pressed = set()
        while True:
            key = self._read_key()
            if not key:
                break
            keys_pressed.add(key)

        # Process movement keys (WASD+QE)
        for key in keys_pressed:
            # Robot movement
            if key == 'w':
                forward = 1.0
            elif key == 's':
                forward = -1.0
            elif key == 'a':
                strafe = 1.0
            elif key == 'd':
                strafe = -1.0
            elif key == 'q':
                rotate = 1.0
            elif key == 'e':
                rotate = -1.0
            elif key == ' ':
                forward = strafe = rotate = 0.0

            # Pose adjustment (arrow keys)
            elif key == 'up':
                pose_dy = 0.05  # 5cm
            elif key == 'down':
                pose_dy = -0.05
            elif key == 'left':
                pose_dx = -0.05
            elif key == 'right':
                pose_dx = 0.05
            elif key == ',':
                pose_dtheta = 5.0  # 5 degrees
            elif key == '.':
                pose_dtheta = -5.0

            # Reset
            elif key == 'r':
                reset_requested = True

            # Exit
            elif key == 'esc' or key == '\x1b':
                exit_requested = True

        return (forward, strafe, rotate,
                pose_dx, pose_dy, pose_dtheta,
                reset_requested, exit_requested)

    def close(self):
        """Restore terminal settings."""
        if self.system in ["Linux", "Darwin"]:
            import termios
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)


class MapVisualizer:
    """
    Real-time map visualization with robot tracking.
    """

    def __init__(self, occupancy_map: OccupancyMap, robot_width: float, robot_height: float):
        """
        Initialize visualizer.

        Args:
            occupancy_map: Loaded occupancy map
            robot_width: Robot width in meters
            robot_height: Robot height in meters
        """
        self.map = occupancy_map
        self.robot_width = robot_width
        self.robot_height = robot_height

        # Robot state (world coordinates)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        # Path history
        self.path_x = []
        self.path_y = []

        # Setup plot
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        self.ax.set_aspect('equal')
        self.ax.set_title('Robot Navigation Console', fontsize=16, fontweight='bold')
        self.ax.set_xlabel('X (meters)', fontsize=12)
        self.ax.set_ylabel('Y (meters)', fontsize=12)

        # Display map
        map_img = self.map.get_display_image()
        extent = [
            self.map.origin[0],
            self.map.origin[0] + self.map.width * self.map.resolution,
            self.map.origin[1],
            self.map.origin[1] + self.map.height * self.map.resolution,
        ]
        self.ax.imshow(map_img, extent=extent, origin='lower', zorder=0)

        # Path line
        self.path_line, = self.ax.plot([], [], 'b-', linewidth=2, alpha=0.6, label='Path', zorder=5)

        # Robot visualization elements
        self.dynamic_patches = []

        # Info text
        self.text_info = self.ax.text(
            0.02, 0.98, '', transform=self.ax.transAxes,
            verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.9),
            fontsize=10, zorder=20
        )

        # Legend
        free_patch = mpatches.Patch(color='white', label='Free Space')
        occupied_patch = mpatches.Patch(color='black', label='Occupied')
        unknown_patch = mpatches.Patch(color='gray', label='Unknown')
        robot_patch = mpatches.Patch(color='red', label='Robot')
        path_patch = mpatches.Patch(color='blue', label='Path')
        self.ax.legend(handles=[free_patch, occupied_patch, unknown_patch, robot_patch, path_patch],
                      loc='upper right', fontsize=10)

        plt.tight_layout()
        plt.show(block=False)
        plt.pause(0.001)

    def update_robot_pose(self, x: float, y: float, theta: float, add_to_path: bool = True):
        """
        Update robot position and orientation.

        Args:
            x: Robot x position (meters)
            y: Robot y position (meters)
            theta: Robot orientation (degrees)
            add_to_path: If True, add position to path history
        """
        self.robot_x = x
        self.robot_y = y
        self.robot_theta = theta

        if add_to_path:
            self.path_x.append(x)
            self.path_y.append(y)

    def draw(self):
        """Redraw robot and path on map."""
        # Remove old dynamic elements
        for patch in self.dynamic_patches:
            try:
                patch.remove()
            except (ValueError, AttributeError):
                pass
        self.dynamic_patches.clear()

        # Draw path
        if len(self.path_x) > 0:
            self.path_line.set_data(self.path_x, self.path_y)

        # Draw robot footprint (rectangle)
        theta_rad = np.deg2rad(self.robot_theta)
        cos_t = np.cos(theta_rad)
        sin_t = np.sin(theta_rad)

        # Robot corners (rotated rectangle)
        w, h = self.robot_width, self.robot_height
        corners = np.array([
            [-w/2, -h/2],
            [w/2, -h/2],
            [w/2, h/2],
            [-w/2, h/2],
            [-w/2, -h/2],  # Close the loop
        ])

        # Rotate and translate
        rotation = np.array([[cos_t, -sin_t], [sin_t, cos_t]])
        corners_world = corners @ rotation.T + np.array([self.robot_x, self.robot_y])

        robot_poly = plt.Polygon(corners_world, closed=True,
                                facecolor='red', edgecolor='darkred',
                                linewidth=2, alpha=0.5, zorder=10)
        self.ax.add_patch(robot_poly)
        self.dynamic_patches.append(robot_poly)

        # Draw orientation arrow
        arrow_len = max(self.robot_width, self.robot_height) * 0.7
        dx = arrow_len * cos_t
        dy = arrow_len * sin_t

        orientation_arrow = FancyArrow(
            self.robot_x, self.robot_y, dx, dy,
            width=0.05, head_width=0.12, head_length=0.08,
            color='darkred', zorder=11
        )
        self.ax.add_patch(orientation_arrow)
        self.dynamic_patches.append(orientation_arrow)

        # Draw center point
        center_circle = Circle((self.robot_x, self.robot_y), 0.03,
                              color='yellow', zorder=12)
        self.ax.add_patch(center_circle)
        self.dynamic_patches.append(center_circle)

        # Check collision
        collision = self.map.check_robot_collision(
            self.robot_x, self.robot_y, self.robot_theta,
            self.robot_width, self.robot_height
        )

        # Update info text
        info_text = (
            f"Robot Pose:\n"
            f"  X: {self.robot_x:+.3f} m\n"
            f"  Y: {self.robot_y:+.3f} m\n"
            f"  �: {self.robot_theta:+.1f}�\n"
            f"\n"
            f"Status: {'COLLISION!' if collision else 'OK'}\n"
            f"Path points: {len(self.path_x)}\n"
            f"\n"
            f"Controls:\n"
            f"  WASD+QE: Move robot\n"
            f"  Arrows: Adjust pose\n"
            f"  , . : Rotate pose\n"
            f"  R: Reset  ESC: Exit"
        )
        self.text_info.set_text(info_text)

        # Change text color if collision
        if collision:
            self.text_info.set_bbox(dict(boxstyle='round', facecolor='red', alpha=0.9))
        else:
            self.text_info.set_bbox(dict(boxstyle='round', facecolor='lightblue', alpha=0.9))

        # Redraw
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def reset_path(self):
        """Clear path history."""
        self.path_x.clear()
        self.path_y.clear()

    def close(self):
        """Close visualization."""
        plt.close(self.fig)


def load_robot_config(config_path: str = "xler.yaml") -> dict:
    """Load robot dimensions from xler.yaml."""
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config


def main():
    parser = argparse.ArgumentParser(description="Robot Navigation Console")
    parser.add_argument("--port", type=str, default=None, help="Serial port for motors")
    parser.add_argument("--no-motors", action="store_true", help="Run without motor hardware (visualization only)")
    parser.add_argument("--map", type=str, default="omap/occupancy_map.yaml", help="Path to map YAML file")
    parser.add_argument("--init-x", type=float, default=0.0, help="Initial robot X position (meters)")
    parser.add_argument("--init-y", type=float, default=0.0, help="Initial robot Y position (meters)")
    parser.add_argument("--init-theta", type=float, default=0.0, help="Initial robot orientation (degrees)")
    args = parser.parse_args()

    print("="*70)
    print("  Robot Navigation Console")
    print("="*70)

    # Load robot configuration
    print("\n[init] Loading robot configuration from xler.yaml...")
    robot_config = load_robot_config()
    robot_width = robot_config['width'] / 1000.0  # mm to m
    robot_height = robot_config['height'] / 1000.0  # mm to m
    print(f"[init] Robot dimensions: {robot_width}m x {robot_height}m")

    # Load map
    print(f"\n[init] Loading occupancy map from {args.map}...")
    occupancy_map = OccupancyMap(args.map)

    # Initialize visualizer
    print("\n[init] Initializing map visualizer...")
    visualizer = MapVisualizer(occupancy_map, robot_width, robot_height)
    visualizer.update_robot_pose(args.init_x, args.init_y, args.init_theta)
    visualizer.draw()
    print(" Map visualization ready")

    # Initialize keyboard
    print("\n[init] Initializing keyboard input...")
    keyboard = ConsoleKeyboardInput()

    # Initialize motors (if enabled)
    controller = None
    bus = None
    connected = False

    if not args.no_motors:

        # Load motor config
        print("\n[init] Loading motor configuration...")
        motors_config = yaml.safe_load(open("motors2/config.yaml"))

        # Auto-detect port if needed
        port = args.port or motors_config['serial']['port']
        if port is None:
            print("[init] Auto-detecting serial port...")
            port = find_serial_port()
            if port:
                print(f"✅ Found: {port}")
            else:
                print("❌ No serial port found, running in visualization mode only")
                args.no_motors = True
        else:
            print(f"[init] Using port: {port}")

        if not args.no_motors:
            # Create motors
            motors = {
                "base_left_wheel": Motor(motors_config['motor_ids']['left'],
                                        motors_config['motor']['model'], MotorNormMode.DEGREES),
                "base_right_wheel": Motor(motors_config['motor_ids']['right'],
                                         motors_config['motor']['model'], MotorNormMode.DEGREES),
                "base_back_wheel": Motor(motors_config['motor_ids']['back'],
                                        motors_config['motor']['model'], MotorNormMode.DEGREES),
            }

            # Create bus
            print(f"\n[init] Connecting to motors on {port}...")
            bus = FeetechMotorsBus(port=port, motors=motors,
                                  protocol_version=motors_config['serial']['protocol_version'])

            try:
                bus.connect()
                connected = True

                # Create controller
                controller = LeKiwiBaseController(bus, motors_config)
                controller.configure()
                print(" Motors connected and configured")
            except Exception as e:
                print(f"L Motor connection failed: {e}")
                print("   Running in visualization mode only")
                args.no_motors = True
                connected = False

    # Print banner
    print("\n" + "="*70)
    print("  READY!")
    print("="*70)
    print("\n<� Controls:")
    print("  Movement:  W/A/S/D (move) + Q/E (rotate) + SPACE (stop)")
    print("  Pose Adj:  Arrow keys (translate) + ,/. (rotate) + R (reset)")
    print("  Exit:      ESC")
    print("="*70 + "\n")

    # Control loop
    try:
        if not args.no_motors:
            control_frequency = motors_config['control']['frequency']
            max_speed = motors_config['control']['max_speed_ms']
            max_rotation = motors_config['control']['max_rotation_degs']
        else:
            control_frequency = 20
            max_speed = 1.0
            max_rotation = 180.0

        loop_delay = 1.0 / control_frequency
        last_loop_time = time.time()

        while True:
            current_time = time.time()
            dt = current_time - last_loop_time
            last_loop_time = current_time

            # Read keyboard
            (forward, strafe, rotate,
             pose_dx, pose_dy, pose_dtheta,
             reset_requested, exit_requested) = keyboard.read()

            if exit_requested:
                print("\n[exit] ESC pressed, stopping...")
                break

            # Handle reset
            if reset_requested:
                print("[reset] Resetting robot pose...")
                visualizer.update_robot_pose(args.init_x, args.init_y, args.init_theta, add_to_path=False)
                visualizer.reset_path()
                visualizer.draw()
                time.sleep(0.1)
                continue

            # Handle pose adjustment (arrow keys) - takes priority
            if abs(pose_dx) > 0.001 or abs(pose_dy) > 0.001 or abs(pose_dtheta) > 0.001:
                new_x = visualizer.robot_x + pose_dx
                new_y = visualizer.robot_y + pose_dy
                new_theta = visualizer.robot_theta + pose_dtheta
                visualizer.update_robot_pose(new_x, new_y, new_theta, add_to_path=False)
                visualizer.draw()
                time.sleep(0.05)
                continue

            # Handle robot movement (WASD+QE)
            # Convert input to velocities (always, even if zero)
            x_vel = forward * max_speed
            y_vel = strafe * max_speed
            theta_vel = rotate * max_rotation

            if not args.no_motors and controller is not None:
                # WITH MOTORS: Send command and read actual velocities
                controller.move(x_vel=x_vel, y_vel=y_vel, theta_vel=theta_vel)

                # Read observation
                try:
                    obs = controller.get_observation()
                except Exception as e:
                    logger.warning(f"Failed to read observation: {e}")
                    obs = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
            else:
                # NO MOTORS: Use commanded velocities as observations
                obs = {"x.vel": x_vel, "y.vel": y_vel, "theta.vel": theta_vel}

            # Update robot position (dead reckoning) - only if moving
            has_motion = abs(obs["x.vel"]) > 0.01 or abs(obs["y.vel"]) > 0.01 or abs(obs["theta.vel"]) > 0.01

            if has_motion:
                theta_rad = np.deg2rad(visualizer.robot_theta)
                dx_world = obs["x.vel"] * np.cos(theta_rad) - obs["y.vel"] * np.sin(theta_rad)
                dy_world = obs["x.vel"] * np.sin(theta_rad) + obs["y.vel"] * np.cos(theta_rad)

                new_x = visualizer.robot_x + dx_world * dt
                new_y = visualizer.robot_y + dy_world * dt
                new_theta = visualizer.robot_theta + obs["theta.vel"] * dt

                # Check collision before updating
                collision = occupancy_map.check_robot_collision(
                    new_x, new_y, new_theta, robot_width, robot_height
                )

                if not collision:
                    visualizer.update_robot_pose(new_x, new_y, new_theta)
                else:
                    # Stop robot if collision detected
                    if controller is not None:
                        controller.stop()
                    print("[WARNING] Collision detected! Stopping.")

            # Update visualization (always)
            visualizer.draw()

            # Maintain loop frequency
            time.sleep(loop_delay)

    except KeyboardInterrupt:
        print("\n\n[interrupt] Ctrl+C detected, stopping...")

    except Exception as e:
        print(f"\nL Error: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # Cleanup
        if connected and controller is not None:
            print("\n[cleanup] Stopping motors...")
            try:
                controller.stop()
                controller.reset_to_position_mode()
                bus.disconnect()
            except Exception as e:
                logger.error(f"Cleanup error: {e}")

        keyboard.close()
        visualizer.close()
        print("\n Done.\n")


if __name__ == "__main__":
    main()
