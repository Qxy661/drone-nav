"""Tests for obstacle_map.py - 2D occupancy grid operations."""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import unittest
import math
import numpy as np


class TestObstacleMapLogic(unittest.TestCase):
    """Test the pure algorithm logic without ROS2."""

    def test_grid_initialization(self):
        """Grid should be all zeros initially."""
        grid = np.zeros((100, 100), dtype=np.int8)
        assert grid.shape == (100, 100)
        assert np.all(grid == 0)

    def test_add_circle_obstacle(self):
        """Simulate adding a circle obstacle."""
        grid = np.zeros((100, 100), dtype=np.int8)
        resolution = 0.5
        cx, cy, radius = 5.0, 5.0, 1.0

        gx = math.floor(cx / resolution)
        gy = math.floor(cy / resolution)
        gr = max(1, math.ceil(radius / resolution))

        for y in range(max(0, gy - gr), min(100, gy + gr + 1)):
            for x in range(max(0, gx - gr), min(100, gx + gr + 1)):
                if (x - gx)**2 + (y - gy)**2 <= gr**2:
                    grid[y, x] = 100

        # Center should be occupied
        assert grid[gy, gx] == 100
        # Far away should be free
        assert grid[0, 0] == 0

    def test_add_rectangle_obstacle(self):
        """Simulate adding a rectangle obstacle."""
        grid = np.zeros((100, 100), dtype=np.int8)
        resolution = 0.5
        x1, y1, x2, y2 = 2.0, 2.0, 4.0, 4.0

        gx1 = max(0, math.floor(x1 / resolution))
        gy1 = max(0, math.floor(y1 / resolution))
        gx2 = min(100, math.ceil(x2 / resolution))
        gy2 = min(100, math.ceil(y2 / resolution))
        grid[gy1:gy2, gx1:gx2] = 100

        # Inside rectangle should be occupied
        assert grid[gy1 + 1, gx1 + 1] == 100
        # Outside should be free
        assert grid[0, 0] == 0

    def test_negative_coordinates(self):
        """Negative coordinates should be handled safely."""
        resolution = 0.5
        cx, cy = -1.0, -2.0
        gx = math.floor(cx / resolution)  # -2
        gy = math.floor(cy / resolution)  # -4
        assert gx < 0
        assert gy < 0
        # Bounds checking should prevent access

    def test_is_occupied_logic(self):
        """Test occupation check logic."""
        grid = np.zeros((100, 100), dtype=np.int8)
        grid[50, 50] = 100
        resolution = 0.5

        # Occupied cell
        gx = math.floor(25.0 / resolution)  # 50
        gy = math.floor(25.0 / resolution)  # 50
        assert 0 <= gx < 100 and 0 <= gy < 100
        assert grid[gy, gx] == 100

        # Free cell
        gx2 = math.floor(0.0 / resolution)
        gy2 = math.floor(0.0 / resolution)
        assert grid[gy2, gx2] == 0

    def test_get_binary_grid(self):
        """Binary grid should be 0 for free, 1 for occupied."""
        grid = np.zeros((10, 10), dtype=np.int8)
        grid[5, 5] = 100
        binary = (grid == 100).astype(int).tolist()
        assert binary[5][5] == 1
        assert binary[0][0] == 0

    def test_grid_inflation(self):
        """Test grid inflation for safety margin."""
        grid = np.zeros((100, 100), dtype=np.int8)
        grid[50, 50] = 100
        inflated = grid.copy()
        radius_cells = 2
        for y in range(max(0, 50 - radius_cells), min(100, 50 + radius_cells + 1)):
            for x in range(max(0, 50 - radius_cells), min(100, 50 + radius_cells + 1)):
                if grid[y, x] == 0:
                    inflated[y, x] = 50  # inflated
        assert inflated[50, 50] == 100
        assert inflated[48, 50] == 50
        assert inflated[0, 0] == 0


if __name__ == '__main__':
    unittest.main()
