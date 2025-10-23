#!/usr/bin/env python
"""
Occupancy Map Loader

Loads and parses ROS-style occupancy map files (PGM + YAML).
"""

import numpy as np
import yaml
from pathlib import Path
from typing import Tuple, Optional


class OccupancyMap:
    """
    Occupancy map representation for robot navigation.

    Attributes:
        data: 2D numpy array of occupancy values (0=free, 100=occupied, -1=unknown)
        resolution: Map resolution in meters/pixel
        origin: Map origin (x, y, theta) in meters
        width: Map width in pixels
        height: Map height in pixels
        occupied_thresh: Threshold for occupied cells (0-1)
        free_thresh: Threshold for free cells (0-1)
    """

    def __init__(self, yaml_path: str):
        """
        Load occupancy map from YAML file.

        Args:
            yaml_path: Path to the YAML metadata file
        """
        yaml_path = Path(yaml_path)
        if not yaml_path.exists():
            raise FileNotFoundError(f"Map YAML not found: {yaml_path}")

        # Load YAML metadata
        with open(yaml_path, 'r') as f:
            self.metadata = yaml.safe_load(f)

        # Extract metadata
        self.resolution = self.metadata['resolution']
        self.origin = np.array(self.metadata['origin'][:2])  # [x, y] only
        self.occupied_thresh = self.metadata.get('occupied_thresh', 0.65)
        self.free_thresh = self.metadata.get('free_thresh', 0.196)
        self.negate = self.metadata.get('negate', 0)

        # Load PGM image
        pgm_filename = self.metadata['image']
        pgm_path = yaml_path.parent / pgm_filename

        if not pgm_path.exists():
            raise FileNotFoundError(f"Map image not found: {pgm_path}")

        self.data = self._load_pgm(pgm_path)
        self.height, self.width = self.data.shape

        print(f"[OccupancyMap] Loaded map: {self.width}x{self.height} @ {self.resolution}m/px")
        print(f"[OccupancyMap] Origin: {self.origin}")

    def _load_pgm(self, pgm_path: Path) -> np.ndarray:
        """
        Load PGM image file.

        Args:
            pgm_path: Path to PGM file

        Returns:
            2D numpy array of occupancy values
        """
        with open(pgm_path, 'rb') as f:
            # Read PGM header
            header = f.readline().decode('ascii').strip()
            if header not in ['P5', 'P2']:
                raise ValueError(f"Unsupported PGM format: {header}")

            # Skip comments
            line = f.readline().decode('ascii').strip()
            while line.startswith('#'):
                line = f.readline().decode('ascii').strip()

            # Read width and height
            width, height = map(int, line.split())

            # Read max value
            max_val = int(f.readline().decode('ascii').strip())

            # Read image data
            if header == 'P5':  # Binary
                data = np.frombuffer(f.read(), dtype=np.uint8)
            else:  # ASCII
                data = np.fromfile(f, dtype=np.uint8, sep=' ')

            # Reshape to 2D
            data = data.reshape((height, width))

            # Debug: Print unique values and their counts
            unique_values, counts = np.unique(data, return_counts=True)
            print(f"\n[PGM Debug] Unique pixel values in PGM file:")
            for val, count in zip(unique_values, counts):
                percentage = (count / data.size) * 100
                print(f"  Value {val:3d}: {count:8d} pixels ({percentage:6.2f}%)")

            # This PGM already contains occupancy values (0=free, 100=occupied)
            # No conversion needed!
            occupancy = data.astype(np.int8)

            print(f"\n[OccupancyMap] Occupancy values: 0 (free), 100 (occupied)")
            print(f"[OccupancyMap] Free cells: {np.sum(occupancy == 0)} ({np.sum(occupancy == 0)/occupancy.size*100:.1f}%)")
            print(f"[OccupancyMap] Occupied cells: {np.sum(occupancy == 100)} ({np.sum(occupancy == 100)/occupancy.size*100:.1f}%)")
            print()

            return occupancy

    def world_to_map(self, x: float, y: float) -> Tuple[int, int]:
        """
        Convert world coordinates to map pixel coordinates.

        Args:
            x: World x coordinate (meters)
            y: World y coordinate (meters)

        Returns:
            (map_x, map_y) pixel coordinates
        """
        map_x = int((x - self.origin[0]) / self.resolution)
        map_y = int((y - self.origin[1]) / self.resolution)
        return map_x, map_y

    def map_to_world(self, map_x: int, map_y: int) -> Tuple[float, float]:
        """
        Convert map pixel coordinates to world coordinates.

        Args:
            map_x: Map x pixel
            map_y: Map y pixel

        Returns:
            (x, y) world coordinates in meters
        """
        x = map_x * self.resolution + self.origin[0]
        y = map_y * self.resolution + self.origin[1]
        return x, y

    def is_free(self, x: float, y: float) -> bool:
        """
        Check if a world coordinate is in free space.

        Args:
            x: World x coordinate (meters)
            y: World y coordinate (meters)

        Returns:
            True if free space, False otherwise
        """
        map_x, map_y = self.world_to_map(x, y)

        # Check bounds
        if not (0 <= map_x < self.width and 0 <= map_y < self.height):
            return False

        # Map contains 0 (free) or 100 (occupied)
        # Check if cell is free (value = 0)
        return self.data[map_y, map_x] == 0

    def is_occupied(self, x: float, y: float) -> bool:
        """
        Check if a world coordinate is occupied.

        Args:
            x: World x coordinate (meters)
            y: World y coordinate (meters)

        Returns:
            True if occupied, False otherwise
        """
        map_x, map_y = self.world_to_map(x, y)

        # Check bounds
        if not (0 <= map_x < self.width and 0 <= map_y < self.height):
            return True  # Out of bounds = occupied

        # Map contains 0 (free) or 100 (occupied)
        # Check if cell is occupied (value = 100)
        return self.data[map_y, map_x] == 100

    def check_robot_collision(
        self,
        x: float,
        y: float,
        theta: float,
        robot_width: float,
        robot_height: float
    ) -> bool:
        """
        Check if robot footprint collides with obstacles.

        Args:
            x: Robot center x (meters)
            y: Robot center y (meters)
            theta: Robot orientation (degrees)
            robot_width: Robot width (meters)
            robot_height: Robot height (meters)

        Returns:
            True if collision detected, False if free
        """
        theta_rad = np.deg2rad(theta)

        # Robot corners in robot frame (center at origin)
        corners_robot = np.array([
            [-robot_width/2, -robot_height/2],
            [robot_width/2, -robot_height/2],
            [robot_width/2, robot_height/2],
            [-robot_width/2, robot_height/2],
        ])

        # Rotation matrix
        cos_t = np.cos(theta_rad)
        sin_t = np.sin(theta_rad)
        rotation = np.array([[cos_t, -sin_t], [sin_t, cos_t]])

        # Transform to world frame
        corners_world = corners_robot @ rotation.T + np.array([x, y])

        # Check all corners
        for corner in corners_world:
            if self.is_occupied(corner[0], corner[1]):
                return True

        # Check edges (sample points along edges)
        num_samples = 5
        for i in range(4):
            p1 = corners_world[i]
            p2 = corners_world[(i + 1) % 4]

            for t in np.linspace(0, 1, num_samples):
                point = p1 + t * (p2 - p1)
                if self.is_occupied(point[0], point[1]):
                    return True

        return False

    def get_display_image(self) -> np.ndarray:
        """
        Get map as RGB image for visualization.

        Returns:
            RGB image (height, width, 3) as uint8
        """
        # Create RGB image based on occupancy values
        # Data contains: 0 (free) or 100 (occupied)
        # Display: 0 -> white (255), 100 -> black (0)

        # Convert: 0 -> 255, 100 -> 0
        grayscale = np.where(self.data == 0, 255, 0).astype(np.uint8)

        # Create RGB image (same value for all channels = grayscale)
        img = np.stack([grayscale, grayscale, grayscale], axis=-1)

        return img
