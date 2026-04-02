#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cloud_topic = LaunchConfiguration("cloud_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = PathJoinSubstitution(
        [FindPackageShare("mapping"), "config", "octomap_params.yaml"]
    )

    octomap_server_node = Node(
        package="octomap_server",
        executable="color_octomap_server_node",
        name="octomap_server",
        output="log",
        parameters=[
            params_file,
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
        remappings=[("cloud_in", cloud_topic)],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("cloud_topic", default_value="/depth_camera/points"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            octomap_server_node,
        ]
    )
