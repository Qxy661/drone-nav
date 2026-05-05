from setuptools import setup

package_name = "drone_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/nav_test.launch.py", "launch/nav_system.launch.py"]),
        ("share/" + package_name + "/config", ["config/nav_params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="developer",
    maintainer_email="dev@example.com",
    description="Autonomous drone navigation and path planning package.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "waypoint_navigator = drone_nav.waypoint_navigator:main",
            "obstacle_map = drone_nav.obstacle_map:main",
            "mission_manager = drone_nav.mission_manager:main",
        ],
    },
)
