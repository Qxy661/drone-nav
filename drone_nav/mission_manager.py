"""mission_manager.py - High-level mission orchestration ROS2 node."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from enum import Enum
import math
from typing import Optional, List, Tuple

class MissionState(Enum):
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    NAVIGATING = "NAVIGATING"
    ARRIVED = "ARRIVED"
    FAILED = "FAILED"


class MissionManager(Node):
    """High-level mission state machine."""

    def __init__(self):
        super().__init__("mission_manager")
        self.declare_parameter("test_mode", False)
        self.declare_parameter("replan_on_obstacle", True)
        self.declare_parameter("planner_type", "astar")
        self.declare_parameter("update_rate", 5.0)

        self.test_mode = self.get_parameter("test_mode").value
        self.replan = self.get_parameter("replan_on_obstacle").value
        self.planner_type = self.get_parameter("planner_type").value
        self.update_rate = self.get_parameter("update_rate").value

        self.state = MissionState.IDLE
        self.goal_position = None
        self.current_path = []
        self.current_path_idx = 0

        self.state_pub = self.create_publisher(String, "/nav/mission_state", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/nav/goal_pose", 10)
        self.create_subscription(String, "/nav/navigator_status", self._nav_status_cb, 10)
        self.create_subscription(PoseStamped, "/nav/obstacle_detected", self._obstacle_cb, 10)
        self.timer = self.create_timer(1.0/self.update_rate, self._update)
        self._planner = None
        self.get_logger().info(f"MissionManager started state={self.state.value} planner={self.planner_type}")

    def set_goal(self, x, y, z=0.0):
        """Set a new navigation goal."""
        self.goal_position = (x, y, z)
        self.get_logger().info(f"New goal set: ({x}, {y}, {z})")
        self._transition_to(MissionState.PLANNING)

    def _transition_to(self, new_state):
        old = self.state.value
        self.state = new_state
        self.get_logger().info(f"State: {old} -> {new_state.value}")
        self._publish_state()

    def _publish_state(self):
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)

    def _plan_path(self):
        """Plan path to goal using selected algorithm."""
        if self.goal_position is None:
            self._transition_to(MissionState.IDLE)
            return
        try:
            from drone_nav.path_planner import AStarPlanner, RRTPlanner
            grid = [[0]*100 for _ in range(100)]
            start = (0, 0)
            goal = (int(self.goal_position[0]), int(self.goal_position[1]))
            goal = (min(goal[0], 99), min(goal[1], 99))
            if self.planner_type == "astar":
                planner = AStarPlanner(grid)
            else:
                planner = RRTPlanner(grid, seed=42)
            path = planner.plan(start, goal)
            if path:
                self.current_path = path
                self.current_path_idx = 0
                self._transition_to(MissionState.NAVIGATING)
                self.get_logger().info(f"Path planned with {len(path)} waypoints")
            else:
                self.get_logger().warn("No path found!")
                self._transition_to(MissionState.FAILED)
        except Exception as e:
            self.get_logger().error(f"Planning failed: {e}")
            self._transition_to(MissionState.FAILED)

    def _nav_status_cb(self, msg):
        if msg.data == "COMPLETED":
            self._transition_to(MissionState.ARRIVED)
        elif msg.data == "STOPPED":
            self._transition_to(MissionState.IDLE)

    def _obstacle_cb(self, msg):
        if self.replan and self.state == MissionState.NAVIGATING:
            self.get_logger().warn("Obstacle detected, replanning...")
            self._transition_to(MissionState.PLANNING)

    def _update(self):
        if self.state == MissionState.PLANNING:
            self._plan_path()
        elif self.state == MissionState.ARRIVED:
            self.get_logger().info("Mission complete!")
            self._transition_to(MissionState.IDLE)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
