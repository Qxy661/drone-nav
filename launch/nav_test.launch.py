from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Test mode launch file - SITL, no real hardware."""
    return LaunchDescription([
        DeclareLaunchArgument("test_mode", default_value="true"),
        DeclareLaunchArgument("waypoints_file", default_value=""),

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
                {"test_mode": LaunchConfiguration("test_mode")},
            ],
            output="screen",
        ),
        Node(
            package="drone_nav",
            executable="mission_manager",
            name="mission_manager",
            parameters=[
                "src/drone_nav/config/nav_params.yaml",
                {"test_mode": LaunchConfiguration("test_mode")},
            ],
            output="screen",
        ),
    ])
