"""
航点导航 ROS2 节点
Waypoint Navigator Node

通过 MAVROS 发送航点, 控制无人机按航线飞行
支持: 起飞 -> 航点序列 -> 降落

MAVROS 航点协议:
- /mavros/mission/push  - 上传航点到飞控
- /mavros/mission/pull  - 从飞控下载航点
- /mavros/mission/clear - 清除航点
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.msg import State, Waypoint, WaypointList
from mavros_msgs.srv import (WaypointPush, WaypointClear,
                               CommandBool, SetMode)


class NavigatorState:
    IDLE = "idle"
    UPLOADING = "uploading"
    TAKING_OFF = "taking_off"
    NAVIGATING = "navigating"
    HOVERING = "hovering"
    LANDING = "landing"
    COMPLETE = "complete"


class WaypointNavigatorNode(Node):
    def __init__(self):
        super().__init__("waypoint_navigator")

        self.declare_parameter("test_mode", True)
        self.declare_parameter("takeoff_altitude", 2.0)
        self.declare_parameter("waypoint_radius", 0.5)  # 到达判定半径(米)
        self.declare_parameter("cruise_speed", 1.0)

        self.test_mode = self.get_parameter("test_mode").value
        self.takeoff_alt = self.get_parameter("takeoff_altitude").value
        self.wp_radius = self.get_parameter("waypoint_radius").value
        self.cruise_speed = self.get_parameter("cruise_speed").value

        # 状态
        self.state = NavigatorState.IDLE
        self.fcu_connected = False
        self.armed = False
        self.current_mode = ""
        self.local_pos = (0.0, 0.0, 0.0)
        self.waypoints = []
        self.current_wp_idx = 0

        # QoS
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # 订阅
        self.create_subscription(State, "/mavros/state", self._state_cb, 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._pos_cb, qos)
        self.create_subscription(String, "/nav/waypoints", self._wp_cb, 10)

        # 发布
        self.pos_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10)
        self.status_pub = self.create_publisher(String, "/nav/status", 10)

        # 服务客户端
        if not self.test_mode:
            self.wp_push_client = self.create_client(
                WaypointPush, "/mavros/mission/push")
            self.wp_clear_client = self.create_client(
                WaypointClear, "/mavros/mission/clear")
            self.arming_client = self.create_client(
                CommandBool, "/mavros/cmd/arming")
            self.set_mode_client = self.create_client(
                SetMode, "/mavros/set_mode")

        # 主循环 10Hz
        self.timer = self.create_timer(0.1, self._update)

        mode = "TEST" if self.test_mode else "REAL"
        self.get_logger().info(f"WaypointNavigator started ({mode})")

    def _state_cb(self, msg):
        self.fcu_connected = msg.connected
        self.armed = msg.armed
        self.current_mode = msg.mode

    def _pos_cb(self, msg):
        p = msg.pose.position
        self.local_pos = (p.x, p.y, p.z)

    def _wp_cb(self, msg):
        """接收航点列表 JSON
        格式: {"waypoints": [{"x": 0, "y": 0, "z": 2}, ...]}
        """
        try:
            data = json.loads(msg.data)
            self.waypoints = [
                (wp["x"], wp["y"], wp["z"])
                for wp in data["waypoints"]
            ]
            self.current_wp_idx = 0
            self.state = NavigatorState.TAKING_OFF
            self.get_logger().info(
                f"Received {len(self.waypoints)} waypoints")
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f"Invalid waypoint msg: {e}")

    def _update(self):
        self._publish_status()

        if self.state == NavigatorState.IDLE:
            pass

        elif self.state == NavigatorState.TAKING_OFF:
            if self.test_mode:
                self.get_logger().info("[TEST] Takeoff -> navigating")
                self.state = NavigatorState.NAVIGATING
            else:
                self._send_position(0, 0, self.takeoff_alt)
                if self.local_pos[2] > self.takeoff_alt * 0.9:
                    self.get_logger().info("Reached takeoff altitude")
                    self.state = NavigatorState.NAVIGATING

        elif self.state == NavigatorState.NAVIGATING:
            if self.current_wp_idx >= len(self.waypoints):
                self.get_logger().info("All waypoints reached!")
                self.state = NavigatorState.LANDING
                return

            wp = self.waypoints[self.current_wp_idx]
            if self.test_mode:
                self.get_logger().info(
                    f"[TEST] Waypoint {self.current_wp_idx}: {wp}")
                self.current_wp_idx += 1
                time.sleep(0.5)  # simulate travel time
            else:
                self._send_position(*wp)
                dist = math.sqrt(
                    (self.local_pos[0] - wp[0])**2 +
                    (self.local_pos[1] - wp[1])**2 +
                    (self.local_pos[2] - wp[2])**2)
                if dist < self.wp_radius:
                    self.get_logger().info(
                        f"Reached waypoint {self.current_wp_idx}")
                    self.current_wp_idx += 1

        elif self.state == NavigatorState.LANDING:
            if self.test_mode:
                self.get_logger().info("[TEST] Landing complete")
                self.state = NavigatorState.COMPLETE
            else:
                self._set_mode("LAND")
                if not self.armed:
                    self.state = NavigatorState.COMPLETE

    def _send_position(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        self.pos_pub.publish(msg)

    def _set_mode(self, mode):
        if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            return
        req = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client.call_async(req)

    def _publish_status(self):
        status = {
            "state": self.state,
            "current_wp": self.current_wp_idx,
            "total_wps": len(self.waypoints),
            "position": list(self.local_pos),
            "test_mode": self.test_mode,
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
