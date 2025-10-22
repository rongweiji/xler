#!/usr/bin/env python
"""
Real-time 2D Path Visualization for Robot Movement

Visualizes robot trajectory based on observed velocities (x, y, theta).
Uses matplotlib with non-blocking updates for real-time display.

Features:
- Real-time path plotting
- Robot position and orientation display
- Trail of movement history
- Velocity vectors
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrow, Circle
from collections import deque
import threading
import time


class PathVisualizer:
    """
    Real-time 2D path visualizer for robot movement.

    Integrates observed velocities to track position and draws the path.
    """

    def __init__(self, max_history=500, update_interval=0.05):
        """
        Initialize the visualizer.

        Args:
            max_history: Maximum number of points to keep in path history
            update_interval: Minimum time between plot updates (seconds)
        """
        self.max_history = max_history
        self.update_interval = update_interval

        # Position state (integrated from velocities)
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.theta_pos = 0.0  # in degrees

        # Path history
        self.path_x = deque(maxlen=max_history)
        self.path_y = deque(maxlen=max_history)

        # Current velocities (for display)
        self.x_vel = 0.0
        self.y_vel = 0.0
        self.theta_vel = 0.0

        # Timing
        self.last_update_time = time.time()
        self.last_plot_time = 0.0

        # Plot setup
        self.fig = None
        self.ax = None
        self.path_line = None
        self.robot_circle = None
        self.text_info = None

        # Dynamic elements (arrows) - stored in list for easy cleanup
        self.dynamic_patches = []

        # Thread safety
        self.lock = threading.Lock()

        # Initialize plot in non-blocking mode
        self._setup_plot()

    def _setup_plot(self):
        """Setup matplotlib figure and axes."""
        plt.ion()  # Interactive mode

        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X Position (m)', fontsize=12)
        self.ax.set_ylabel('Y Position (m)', fontsize=12)
        self.ax.set_title('Robot Path Visualization (Real-time)', fontsize=14, fontweight='bold')

        # Initialize empty path line
        self.path_line, = self.ax.plot([], [], 'b-', linewidth=2, alpha=0.6, label='Path')

        # Robot position marker (circle)
        self.robot_circle = Circle((0, 0), 0.05, color='red', zorder=10, label='Robot')
        self.ax.add_patch(self.robot_circle)

        # Robot orientation arrow (will be updated)
        self.robot_arrow = None

        # Velocity vector arrow (will be updated)
        self.vel_arrow = None

        # Info text
        self.text_info = self.ax.text(
            0.02, 0.98, '', transform=self.ax.transAxes,
            verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8)
        )

        # Set initial view limits
        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-1, 1)

        self.ax.legend(loc='upper right')

        plt.tight_layout()
        plt.show(block=False)
        plt.pause(0.001)

    def update(self, obs: dict[str, float], dt: float):
        """
        Update robot position based on observed velocities.

        Args:
            obs: Observation dict with x.vel, y.vel, theta.vel
            dt: Time delta since last update (seconds)
        """
        with self.lock:
            # Extract velocities
            self.x_vel = obs.get('x.vel', 0.0)
            self.y_vel = obs.get('y.vel', 0.0)
            self.theta_vel = obs.get('theta.vel', 0.0)

            # Integrate velocities to get position (dead reckoning)
            # Need to transform body-frame velocities to world frame
            theta_rad = np.deg2rad(self.theta_pos)

            # Body frame to world frame transformation
            dx_world = self.x_vel * np.cos(theta_rad) - self.y_vel * np.sin(theta_rad)
            dy_world = self.x_vel * np.sin(theta_rad) + self.y_vel * np.cos(theta_rad)

            # Update position
            self.x_pos += dx_world * dt
            self.y_pos += dy_world * dt
            self.theta_pos += self.theta_vel * dt

            # Normalize theta to [-180, 180]
            self.theta_pos = ((self.theta_pos + 180) % 360) - 180

            # Add to path history
            self.path_x.append(self.x_pos)
            self.path_y.append(self.y_pos)

            # Update time
            self.last_update_time = time.time()

    def draw(self):
        """
        Draw the current state to the plot.

        Only updates plot if enough time has passed (for performance).
        """
        current_time = time.time()

        # Rate limit plot updates
        if current_time - self.last_plot_time < self.update_interval:
            return

        with self.lock:
            # Update path line
            if len(self.path_x) > 0:
                self.path_line.set_data(list(self.path_x), list(self.path_y))

            # Update robot position
            self.robot_circle.center = (self.x_pos, self.y_pos)

            # Remove old dynamic patches (arrows)
            for patch in self.dynamic_patches:
                try:
                    patch.remove()
                except (ValueError, AttributeError):
                    pass  # Already removed or not in axes
            self.dynamic_patches.clear()

            # Draw orientation arrow (pointing direction)
            theta_rad = np.deg2rad(self.theta_pos)
            arrow_len = 0.15
            dx_arrow = arrow_len * np.cos(theta_rad)
            dy_arrow = arrow_len * np.sin(theta_rad)

            robot_arrow = FancyArrow(
                self.x_pos, self.y_pos,
                dx_arrow, dy_arrow,
                width=0.03, head_width=0.08, head_length=0.05,
                color='red', zorder=11
            )
            self.ax.add_patch(robot_arrow)
            self.dynamic_patches.append(robot_arrow)

            # Draw velocity vector (body frame transformed to world)
            vel_scale = 0.2  # Scale factor for visibility
            theta_rad = np.deg2rad(self.theta_pos)
            vx_world = self.x_vel * np.cos(theta_rad) - self.y_vel * np.sin(theta_rad)
            vy_world = self.x_vel * np.sin(theta_rad) + self.y_vel * np.cos(theta_rad)
            vel_mag = np.sqrt(vx_world**2 + vy_world**2)

            if vel_mag > 0.01:  # Only draw if moving
                vel_arrow = FancyArrow(
                    self.x_pos, self.y_pos,
                    vx_world * vel_scale, vy_world * vel_scale,
                    width=0.02, head_width=0.05, head_length=0.04,
                    color='green', alpha=0.7, zorder=12
                )
                self.ax.add_patch(vel_arrow)
                self.dynamic_patches.append(vel_arrow)

            # Update info text
            info_text = (
                f"Position: x={self.x_pos:+.2f}m, y={self.y_pos:+.2f}m, θ={self.theta_pos:+.1f}°\n"
                f"Velocity: x={self.x_vel:+.2f}m/s, y={self.y_vel:+.2f}m/s, θ={self.theta_vel:+.1f}°/s\n"
                f"Path points: {len(self.path_x)}"
            )
            self.text_info.set_text(info_text)

            # Auto-scale view to keep robot in sight
            if len(self.path_x) > 0:
                all_x = list(self.path_x) + [self.x_pos]
                all_y = list(self.path_y) + [self.y_pos]

                x_min, x_max = min(all_x), max(all_x)
                y_min, y_max = min(all_y), max(all_y)

                # Add margin
                margin = 0.5
                x_range = max(x_max - x_min, 1.0)
                y_range = max(y_max - y_min, 1.0)

                self.ax.set_xlim(x_min - margin, x_max + margin)
                self.ax.set_ylim(y_min - margin, y_max + margin)

            # Redraw
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

            self.last_plot_time = current_time

    def reset(self):
        """Reset position and path history."""
        with self.lock:
            self.x_pos = 0.0
            self.y_pos = 0.0
            self.theta_pos = 0.0
            self.path_x.clear()
            self.path_y.clear()
            self.x_vel = 0.0
            self.y_vel = 0.0
            self.theta_vel = 0.0

    def close(self):
        """Close the visualization window."""
        plt.close(self.fig)
