import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    localization_share = get_package_share_directory("bxi_cuvslam_localization")
    nav_share = get_package_share_directory("bxi_scan_nav")
    localization_launch = os.path.join(
        localization_share, "launch", "elf3_dual_cuvslam.launch.py"
    )
    nav_launch = os.path.join(nav_share, "launch", "elf3_nav3d.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_config", default_value=""),
            DeclareLaunchArgument("input_pcd", default_value=""),
            DeclareLaunchArgument(
                "cuvslam_map_directory", default_value="/opt/bxi/maps/elf3_cuvslam"
            ),
            DeclareLaunchArgument("start_cameras", default_value="true"),
            DeclareLaunchArgument("start_visual_slam", default_value="true"),
            DeclareLaunchArgument("enable_mapping", default_value="false"),
            DeclareLaunchArgument("start_scan_planner", default_value="true"),
            DeclareLaunchArgument("start_octo_global_planner", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(localization_launch),
                launch_arguments={
                    "camera_config": LaunchConfiguration("camera_config"),
                    "start_cameras": LaunchConfiguration("start_cameras"),
                    "start_visual_slam": LaunchConfiguration("start_visual_slam"),
                    "enable_mapping": LaunchConfiguration("enable_mapping"),
                    "map_directory": LaunchConfiguration("cuvslam_map_directory"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_launch),
                launch_arguments={
                    "start_simulation": "false",
                    "start_simulated_cameras": "false",
                    "start_controller": "false",
                    "require_localization_valid": "true",
                    "start_scan_planner": LaunchConfiguration("start_scan_planner"),
                    "start_octo_global_planner": LaunchConfiguration(
                        "start_octo_global_planner"
                    ),
                    "start_rviz": LaunchConfiguration("start_rviz"),
                    "global_frame": "map",
                    "body_pose_topic": "/nav/base_footprint/pose",
                    "nav_depth_topic": "/hardware/head_depth_camera/depth/image_rect_raw",
                    "nav_camera_pose_topic": "/nav/head_depth_camera/pose",
                    "nav_camera_frame_id": (
                        "head_depth_camera_depth_optical_frame"
                    ),
                    "input_pcd": LaunchConfiguration("input_pcd"),
                }.items(),
            ),
        ]
    )
