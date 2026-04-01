#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    depth_image_topic = LaunchConfiguration("depth_image_topic")
    depth_info_topic = LaunchConfiguration("depth_info_topic")
    enable_depth_cloud = LaunchConfiguration("enable_depth_cloud")
    enable_octomap = LaunchConfiguration("enable_octomap")
    octomap_cloud_topic = LaunchConfiguration("octomap_cloud_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declared_args = [
        DeclareLaunchArgument("depth_image_topic", default_value="/realsense/depth/image"),
        DeclareLaunchArgument("depth_info_topic", default_value="/realsense/depth/camera_info"),
        DeclareLaunchArgument("enable_depth_cloud", default_value="true"),
        DeclareLaunchArgument("enable_octomap", default_value="true"),
        DeclareLaunchArgument("octomap_cloud_topic", default_value="/depth_camera/points"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]

    depth_point_cloud_xyz_node = Node(
        package="depth_image_proc",
        executable="point_cloud_xyz_node",
        name="depth_point_cloud_xyz",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }
        ],
        remappings=[
            ("image_rect", depth_image_topic),
            ("camera_info", depth_info_topic),
            ("points", octomap_cloud_topic),
        ],
        condition=IfCondition(enable_depth_cloud),
    )

    octomap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("mapping"), "launch", "octomap.launch.py"])
        ),
        launch_arguments={
            "cloud_topic": octomap_cloud_topic,
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    enable_octomap,
                    "' == 'true' and '",
                    enable_depth_cloud,
                    "' == 'true'",
                ]
            )
        ),
    )

    return LaunchDescription(declared_args + [depth_point_cloud_xyz_node, octomap_launch])
