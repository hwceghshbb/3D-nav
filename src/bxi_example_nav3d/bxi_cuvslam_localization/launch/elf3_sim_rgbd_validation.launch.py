import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    nav_launch = os.path.join(
        get_package_share_directory("bxi_scan_nav"),
        "launch",
        "elf3_nav3d.launch.py",
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_launch),
                launch_arguments={
                    "start_simulation": "true",
                    "start_simulated_cameras": "true",
                    "start_controller": "false",
                    "start_scan_planner": "false",
                    "start_octo_global_planner": "false",
                    "start_clicked_point_3d_goal": "false",
                    "start_rviz": "false",
                    "start_body_depth_camera": "true",
                    "camera_width": "320",
                    "camera_height": "240",
                    "body_camera_width": "320",
                    "body_camera_height": "240",
                    "sim_camera_publish_color": "true",
                    "sim_camera_publish_hz": "30.0",
                    "sim_camera_align_depth_to_color": "true",
                }.items(),
            ),
            Node(
                package="bxi_cuvslam_localization",
                executable="head_camera_rig_guard",
                name="head_camera_rig_guard",
                output="screen",
                parameters=[
                    {
                        "head_color_topic": (
                            "/simulation/head_depth_camera/color/image_raw"
                        ),
                        "head_depth_topic": (
                            "/simulation/head_depth_camera/depth/image_rect_raw"
                        ),
                        "joint_states_topic": "/simulation/joint_states",
                        "imu_topic": "/simulation/imu_data",
                        "rgb_depth_sync_limit_ms": 0.1,
                        "required_good_sets": 5,
                        "image_timeout_sec": 0.5,
                        "imu_timeout_sec": 0.5,
                        "joint_timeout_sec": 0.5,
                        # No motion controller runs in this camera-only test.
                        "require_head_lock": False,
                    }
                ],
            ),
        ]
    )
