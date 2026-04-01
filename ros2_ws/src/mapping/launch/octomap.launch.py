#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cloud_topic = LaunchConfiguration("cloud_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cloud_topic",
                default_value="/depth_camera/points",
                description="PointCloud2 topic for octomap_server",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Pass use_sim_time to octomap_server",
            ),
            Node(
                package="octomap_server",
                executable="octomap_server_node",
                name="octomap_server",
                output="screen",
                parameters=[
                    {
                        "frame_id": "world",
                        "resolution": 0.1,
                        "sensor_model.max_range": 30.0,
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    }
                ],
                remappings=[("cloud_in", cloud_topic)],
            ),
        ]
    )
