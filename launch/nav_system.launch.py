from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Real hardware mode launch file - connects to actual drone via MAVROS."""
    return LaunchDescription([
        DeclareLaunchArgument("waypoints_file", default_value=""),

        Node(
            package="mavros",
            executable="mavros_node",
            name="mavros",
            parameters=[{
                "fcu_url": "udp://:14550@",
                "gcs_url": "udp://:14551@127.0.0.1:14550",
            }],
            output="screen",
        ),
        Node(
            package="drone_nav",
            executable="obstacle_map",
            name="obstacle_map",
            parameters=["src/drone_nav/config/nav_params.yaml"],
            output="screen",
        ),
        Node(
            package="drone_nav",
            executable="waypoint_navigator",
            name="waypoint_navigator",
            parameters=[
                "src/drone_nav/config/nav_params.yaml",
                {"test_mode": False},
            ],
            output="screen",
        ),
        Node(
            package="drone_nav",
            executable="mission_manager",
            name="mission_manager",
            parameters=[
                "src/drone_nav/config/nav_params.yaml",
                {"test_mode": False},
            ],
            output="screen",
        ),
    ])
