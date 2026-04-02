#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    color_image_topic = LaunchConfiguration("color_image_topic")
    color_info_topic = LaunchConfiguration("color_info_topic")
    depth_image_topic = LaunchConfiguration("depth_image_topic")
    depth_info_topic = LaunchConfiguration("depth_info_topic")
    octomap_cloud_topic = LaunchConfiguration("octomap_cloud_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declared_args = [
        DeclareLaunchArgument("color_image_topic", default_value="/realsense/rgb/left_image_raw"),
        DeclareLaunchArgument("color_info_topic", default_value="/realsense/rgb/left_image_info"),
        DeclareLaunchArgument("depth_image_topic", default_value="/realsense/depth/image"),
        DeclareLaunchArgument("depth_info_topic", default_value="/realsense/depth/camera_info"),
        DeclareLaunchArgument("octomap_cloud_topic", default_value="/depth_camera/points"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]

    depth_register_node = Node(
        package="depth_image_proc",
        executable="register_node",
        name="depth_register",
        output="screen",
        parameters=[
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
        remappings=[
            ("depth/image_rect", depth_image_topic),
            ("depth/camera_info", depth_info_topic),
            ("rgb/camera_info", color_info_topic),
            ("depth_registered/image_rect", "/depth_registered/image_rect"),
            ("depth_registered/camera_info", "/depth_registered/camera_info"),
        ],
    )

    depth_point_cloud_xyzrgb_node = Node(
        package="depth_image_proc",
        executable="point_cloud_xyzrgb_node",
        name="depth_point_cloud_xyzrgb",
        output="screen",
        parameters=[
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
        remappings=[
            ("rgb/image_rect_color", color_image_topic),
            ("/realsense/rgb/camera_info", color_info_topic),
            ("depth_registered/image_rect", "/depth_registered/image_rect"),
            ("points", octomap_cloud_topic),
        ],
    )

    octomap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("mapping"), "launch", "octomap.launch.py"]
            )
        ),
        launch_arguments={
            "cloud_topic": octomap_cloud_topic,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return LaunchDescription(
        declared_args
        + [
            depth_register_node,
            depth_point_cloud_xyzrgb_node,
            octomap_launch,
        ]
    )
