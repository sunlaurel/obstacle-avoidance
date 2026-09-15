"""Launch Gazebo + ROS 2 autonomous waypoint navigation."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("obstacle_avoidance")
    world = os.path.join(pkg, "worlds", "obstacle_course.sdf")
    bridge = os.path.join(pkg, "config", "bridge.yaml")
    rviz_cfg = os.path.join(pkg, "rviz", "sim.rviz")
    gz_launch = os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")

    use_sim_time = LaunchConfiguration("use_sim_time")

    # GUI vs headless: ``-s`` is server-only (no gz-gui). ``-r`` starts playing.
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={"gz_args": f"-r -v 2 {world}", "on_exit_shutdown": "true"}.items(),
        condition=IfCondition(LaunchConfiguration("gui")),
    )
    gz_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={"gz_args": f"-r -s -v 2 {world}", "on_exit_shutdown": "true"}.items(),
        condition=UnlessCondition(LaunchConfiguration("gui")),
    )

    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        parameters=[{"config_file": bridge, "use_sim_time": use_sim_time}],
        output="screen",
    )

    waypoint_node = Node(
        package="obstacle_avoidance",
        executable="waypoint_generator",
        name="waypoint_generator",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "num_waypoints": ParameterValue(LaunchConfiguration("num_waypoints"), value_type=int),
                "seed": ParameterValue(LaunchConfiguration("seed"), value_type=int),
            }
        ],
        output="screen",
    )

    lidar_node = Node(
        package="obstacle_avoidance",
        executable="fake_lidar",
        name="fake_lidar",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    navigator_node = Node(
        package="obstacle_avoidance",
        executable="navigator",
        name="navigator",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_cfg],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true", description="Launch Gazebo GUI"),
            DeclareLaunchArgument("rviz", default_value="true", description="Launch RViz"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "num_waypoints",
                default_value="4",
                description="How many randomized XY goals to visit",
            ),
            DeclareLaunchArgument(
                "seed",
                default_value="0",
                description="RNG seed for waypoint sampling (0 = time-based)",
            ),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(pkg, "worlds")),
            gz_gui,
            gz_headless,
            bridge_node,
            waypoint_node,
            lidar_node,
            navigator_node,
            rviz_node,
        ]
    )
