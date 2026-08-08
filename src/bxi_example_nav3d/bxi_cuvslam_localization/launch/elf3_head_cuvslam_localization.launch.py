import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_file = os.path.join(
        get_package_share_directory("bxi_cuvslam_localization"),
        "launch",
        "elf3_head_cuvslam.launch.py",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_directory", default_value="/opt/bxi/maps/elf3_cuvslam"
            ),
            DeclareLaunchArgument(
                "head_color_topic",
                default_value="/hardware/head_depth_camera/color/image_raw",
            ),
            DeclareLaunchArgument(
                "head_depth_topic",
                default_value=(
                    "/hardware/head_depth_camera/aligned_depth_to_color/image_raw"
                ),
            ),
            DeclareLaunchArgument(
                "head_camera_info_topic",
                default_value="/hardware/head_depth_camera/color/camera_info",
            ),
            DeclareLaunchArgument("joint_states_topic", default_value="/hardware/joint_states"),
            DeclareLaunchArgument("imu_topic", default_value="/nav/imu"),
            DeclareLaunchArgument("start_imu_combiner", default_value="true"),
            DeclareLaunchArgument("require_initial_pose", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file),
                launch_arguments={
                    "mode": "localization",
                    "map_directory": LaunchConfiguration("map_directory"),
                    "head_color_topic": LaunchConfiguration("head_color_topic"),
                    "head_depth_topic": LaunchConfiguration("head_depth_topic"),
                    "head_camera_info_topic": LaunchConfiguration("head_camera_info_topic"),
                    "joint_states_topic": LaunchConfiguration("joint_states_topic"),
                    "imu_topic": LaunchConfiguration("imu_topic"),
                    "start_imu_combiner": LaunchConfiguration("start_imu_combiner"),
                    "require_initial_pose": LaunchConfiguration("require_initial_pose"),
                    "auto_relocalize": "true",
                }.items(),
            ),
        ]
    )
