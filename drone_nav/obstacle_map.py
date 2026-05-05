"""obstacle_map.py - 2D occupancy grid map management ROS2 node."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid, MapMetaData
import yaml
import numpy as np
from typing import List, Tuple, Optional


class ObstacleMap(Node):
    """Maintains a 2D occupancy grid map."""

    def __init__(self):
        super().__init__("obstacle_map")
        self.declare_parameter("map_width", 100)
        self.declare_parameter("map_height", 100)
        self.declare_parameter("resolution", 0.5)
        self.declare_parameter("obstacles_file", "")
        self.declare_parameter("publish_rate", 1.0)

        self.width = self.get_parameter("map_width").value
        self.height = self.get_parameter("map_height").value
        self.resolution = self.get_parameter("resolution").value
        self.obstacles_file = self.get_parameter("obstacles_file").value
        self.publish_rate = self.get_parameter("publish_rate").value

        # Initialize grid: 0 = free, 100 = occupied, -1 = unknown
        self.grid = np.zeros((self.height, self.width), dtype=np.int8)

        # Load obstacles from file if specified
        if self.obstacles_file:
            self._load_obstacles(self.obstacles_file)

        # Publisher for OccupancyGrid message
        self.map_pub = self.create_publisher(
            OccupancyGrid, "/nav/obstacle_map", 10)

        # Timer for periodic map publishing
        self.timer = self.create_timer(
            1.0 / self.publish_rate, self._publish_map)

        self.get_logger().info(
            f"ObstacleMap initialized: {self.width}x{self.height} "
            f"resolution={self.resolution}m")

    def _load_obstacles(self, filepath):
        """Load obstacles from YAML config file."""
        try:
            with open(filepath, "r") as f:
                data = yaml.safe_load(f)
            for obs in data.get("obstacles", []):
                if obs["type"] == "circle":
                    self.add_circle_obstacle(
                        obs["cx"], obs["cy"], obs["radius"])
                elif obs["type"] == "rectangle":
                    self.add_rectangle_obstacle(
                        obs["x1"], obs["y1"], obs["x2"], obs["y2"])
            self.get_logger().info(f"Loaded obstacles from {filepath}")
        except Exception as e:
            self.get_logger().error(f"Failed to load obstacles: {e}")

    def add_circle_obstacle(self, cx, cy, radius):
        """Add a circular obstacle in world coordinates."""
        gx = int(cx / self.resolution)
        gy = int(cy / self.resolution)
        gr = int(radius / self.resolution)
        for y in range(max(0, gy-gr), min(self.height, gy+gr+1)):
            for x in range(max(0, gx-gr), min(self.width, gx+gr+1)):
                if (x-gx)**2 + (y-gy)**2 <= gr**2:
                    self.grid[y, x] = 100

    def add_rectangle_obstacle(self, x1, y1, x2, y2):
        """Add a rectangular obstacle in world coordinates."""
        gx1 = max(0, int(x1 / self.resolution))
        gy1 = max(0, int(y1 / self.resolution))
        gx2 = min(self.width, int(x2 / self.resolution))
        gy2 = min(self.height, int(y2 / self.resolution))
        self.grid[gy1:gy2, gx1:gx2] = 100

    def is_occupied(self, x, y):
        """Check if world coordinate (x,y) is occupied."""
        gx = int(x / self.resolution)
        gy = int(y / self.resolution)
        if 0 <= gx < self.width and 0 <= gy < self.height:
            return self.grid[gy, gx] == 100
        return True  # Out of bounds = occupied

    def get_binary_grid(self):
        """Return binary grid (0=free, 1=occupied) for path planners."""
        return (self.grid == 100).astype(int).tolist()

    def _publish_map(self):
        """Publish OccupancyGrid message."""
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info = MapMetaData()
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.data = self.grid.flatten().tolist()
        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
