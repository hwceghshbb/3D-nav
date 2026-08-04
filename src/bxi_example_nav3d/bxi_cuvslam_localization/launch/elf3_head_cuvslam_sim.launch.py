import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    cuvslam_share = get_package_share_directory("bxi_cuvslam_localization")
    nav_launch = os.path.join(
        get_package_share_directory("bxi_scan_nav"),
        "launch",
        "elf3_nav3d.launch.py",
    )
    cuvslam_launch = os.path.join(
        get_package_share_directory("bxi_cuvslam_localization"),
        "launch",
        "elf3_head_cuvslam.launch.py",
    )
    return LaunchDescription(
        [
            SetEnvironmentVariable(
                "FASTRTPS_DEFAULT_PROFILES_FILE",
                os.path.join(cuvslam_share, "config", "fastdds_large_data.xml"),
            ),
            DeclareLaunchArgument("mode", default_value="mapping"),
            DeclareLaunchArgument(
                "map_directory", default_value="/tmp/elf3_head_cuvslam_map"
            ),
            DeclareLaunchArgument("start_controller", default_value="false"),
            DeclareLaunchArgument("camera_width", default_value="424"),
            DeclareLaunchArgument("camera_height", default_value="240"),
            DeclareLaunchArgument("camera_hz", default_value="30.0"),
            DeclareLaunchArgument("start_rig_guard", default_value="true"),
            DeclareLaunchArgument("enable_slam_backend", default_value="false"),
            DeclareLaunchArgument(
                "error_record_path",
                default_value="/tmp/cuvslam_large_error_events.jsonl",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_launch),
                launch_arguments={
                    "start_simulation": "true",
                    "start_simulated_cameras": "true",
                    "publish_simulated_camera_pose": "false",
                    "start_controller": LaunchConfiguration("start_controller"),
                    "start_scan_planner": "false",
                    "start_octo_global_planner": "false",
                    "start_clicked_point_3d_goal": "false",
                    "start_rviz": "false",
                    "start_body_depth_camera": "true",
                    "camera_width": LaunchConfiguration("camera_width"),
                    "camera_height": LaunchConfiguration("camera_height"),
                    "body_camera_width": "48",
                    "body_camera_height": "36",
                    "sim_camera_publish_color": "true",
                    "sim_camera_publish_hz": LaunchConfiguration("camera_hz"),
                    "sim_camera_align_depth_to_color": "true",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(cuvslam_launch),
                launch_arguments={
                    "mode": LaunchConfiguration("mode"),
                    "map_directory": LaunchConfiguration("map_directory"),
                    "head_color_topic": "/simulation/head_depth_camera/color/image_raw",
                    "head_depth_topic": "/simulation/head_depth_camera/depth/image_rect_raw",
                    "head_camera_info_topic": "/simulation/head_depth_camera/color/camera_info",
                    "joint_states_topic": "/simulation/joint_states",
                    "map_anchor_odom_topic": "/simulation/odom",
                    "map_anchor_pose_topic": "",
                    "require_map_anchor": "true",
                    "imu_topic": "/simulation/imu_data",
                    "start_imu_combiner": "false",
                    "start_rig_guard": LaunchConfiguration("start_rig_guard"),
                    "start_odom_bridge": "false",
                    "start_pose_publishers": "false",
                    "start_sim_error_monitor": "true",
                    "sim_truth_odom_topic": "/simulation/odom",
                    "sim_error_record_path": LaunchConfiguration(
                        "error_record_path"
                    ),
                    "enable_slam_backend": LaunchConfiguration(
                        "enable_slam_backend"
                    ),
                    "require_imu": "false",
                    "require_head_lock": "false",
                }.items(),
            ),
        ]
    )
