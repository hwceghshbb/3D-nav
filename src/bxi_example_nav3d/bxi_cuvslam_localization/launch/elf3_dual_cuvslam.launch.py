"""Compatibility entry point. Localization is intentionally head RGB-D only."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def setup(context):
    actions = []
    if as_bool(LaunchConfiguration("start_cameras").perform(context)):
        camera_launch = os.path.join(
            get_package_share_directory("bxi_depth_camera"),
            "launch",
            "cameras.launch.py",
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_launch),
                launch_arguments={
                    "config_file": LaunchConfiguration("camera_config"),
                    "enable_depth": "true",
                    "enable_color": "true",
                    "enable_gyro": "true",
                    "enable_accel": "true",
                }.items(),
            )
        )

    mode = "mapping" if as_bool(LaunchConfiguration("enable_mapping").perform(context)) else "localization"
    head_launch = os.path.join(
        get_package_share_directory("bxi_cuvslam_localization"),
        "launch",
        "elf3_head_cuvslam.launch.py",
    )
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(head_launch),
            launch_arguments={
                "mode": mode,
                "map_directory": LaunchConfiguration("map_directory"),
            }.items(),
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_config", default_value=""),
            DeclareLaunchArgument("start_cameras", default_value="true"),
            DeclareLaunchArgument("start_visual_slam", default_value="true"),
            DeclareLaunchArgument("enable_mapping", default_value="false"),
            DeclareLaunchArgument(
                "map_directory", default_value="/opt/bxi/maps/elf3_cuvslam"
            ),
            OpaqueFunction(function=setup),
        ]
    )
