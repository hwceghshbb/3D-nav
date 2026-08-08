"""ELF3 hardware localization and Octo/SCAN navigation entry point."""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    orb_share = get_package_share_path("bxi_orbslam3_ros2")
    combined_launch = (
        get_package_share_path("bxi_scan_nav")
        / "launch"
        / "elf3_rtab_cuvslam_nav.launch.py"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("localization_backend", default_value="rtabmap"),
            DeclareLaunchArgument("rtabmap_database", default_value="/opt/bxi/maps/site.db"),
            DeclareLaunchArgument(
                "orb_map_directory", default_value="/opt/bxi/maps/site_orb"
            ),
            DeclareLaunchArgument("orb_atlas_name", default_value="atlas"),
            DeclareLaunchArgument("orb_max_time_diff", default_value="0.008"),
            DeclareLaunchArgument(
                "orb_settings",
                default_value=str(
                    orb_share / "config" / "elf3_head_1280x720_rgbd.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "cuvslam_map_directory", default_value="/opt/bxi/maps/site_cuvslam"
            ),
            DeclareLaunchArgument(
                "input_pcd", default_value="/opt/bxi/maps/site_cloud.ply"
            ),
            DeclareLaunchArgument("start_scan_planner", default_value="true"),
            DeclareLaunchArgument("start_octo_global_planner", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument(
                "color_topic",
                default_value="/hardware/head_depth_camera/color/image_raw",
            ),
            DeclareLaunchArgument(
                "depth_topic",
                default_value=(
                    "/hardware/head_depth_camera/aligned_depth_to_color/image_raw"
                ),
            ),
            DeclareLaunchArgument(
                "raw_depth_topic",
                default_value="/hardware/head_depth_camera/depth/image_rect_raw",
            ),
            DeclareLaunchArgument(
                "depth_camera_info_topic",
                default_value="/hardware/head_depth_camera/depth/camera_info",
            ),
            DeclareLaunchArgument(
                "registered_depth_camera_info_topic",
                default_value=(
                    "/hardware/head_depth_camera/aligned_depth_to_color/camera_info"
                ),
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/hardware/head_depth_camera/color/camera_info",
            ),
            DeclareLaunchArgument(
                "nav_camera_frame_id",
                default_value="head_depth_camera_color_optical_frame",
            ),
            DeclareLaunchArgument(
                "body_camera_uses_head_mount", default_value="false"
            ),
            DeclareLaunchArgument("start_depth_registration", default_value="false"),
            DeclareLaunchArgument("start_input_guard", default_value="true"),
            DeclareLaunchArgument("require_head_lock", default_value="true"),
            DeclareLaunchArgument("enforce_flat_floor", default_value="true"),
            DeclareLaunchArgument("max_floor_z_drift_m", default_value="0.12"),
            DeclareLaunchArgument("max_floor_tilt_rad", default_value="0.30"),
            DeclareLaunchArgument("rtabmap_approx_sync", default_value="false"),
            DeclareLaunchArgument(
                "use_realsense_internal_tf", default_value="true"
            ),
            DeclareLaunchArgument("head_link_x", default_value="0.0628"),
            DeclareLaunchArgument("head_link_y", default_value="0.0175"),
            DeclareLaunchArgument("head_link_z", default_value="0.2515"),
            DeclareLaunchArgument("head_link_roll", default_value="0.0"),
            DeclareLaunchArgument("head_link_pitch", default_value="0.0"),
            DeclareLaunchArgument("head_link_yaw", default_value="0.0"),
            DeclareLaunchArgument("camera_width", default_value="1280"),
            DeclareLaunchArgument("camera_height", default_value="720"),
            DeclareLaunchArgument(
                "imu_topic", default_value="/hardware/head_depth_camera/imu"
            ),
            DeclareLaunchArgument("rtabmap_use_imu", default_value="false"),
            DeclareLaunchArgument("rtabmap_filter_imu", default_value="false"),
            DeclareLaunchArgument("rtabmap_force_3dof", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(combined_launch)),
                launch_arguments={
                    "localization_backend": LaunchConfiguration(
                        "localization_backend"
                    ),
                    "localization_mode": "localization",
                    "rtabmap_localization": "true",
                    "rtabmap_database": LaunchConfiguration("rtabmap_database"),
                    "orb_map_directory": LaunchConfiguration("orb_map_directory"),
                    "orb_atlas_name": LaunchConfiguration("orb_atlas_name"),
                    "orb_max_time_diff": LaunchConfiguration("orb_max_time_diff"),
                    "orb_settings": LaunchConfiguration("orb_settings"),
                    "cuvslam_map": LaunchConfiguration("cuvslam_map_directory"),
                    "octo_input_pcd": LaunchConfiguration("input_pcd"),
                    "start_simulation": "false",
                    "start_controller": "false",
                    "start_simulated_cameras": "false",
                    "start_scan_planner": LaunchConfiguration(
                        "start_scan_planner"
                    ),
                    "start_octo_global_planner": LaunchConfiguration(
                        "start_octo_global_planner"
                    ),
                    "start_rtabmap": "true",
                    "start_rviz": LaunchConfiguration("start_rviz"),
                    "global_frame": "map",
                    "color_topic": LaunchConfiguration("color_topic"),
                    "depth_topic": LaunchConfiguration("depth_topic"),
                    "raw_depth_topic": LaunchConfiguration("raw_depth_topic"),
                    "depth_camera_info_topic": LaunchConfiguration(
                        "depth_camera_info_topic"
                    ),
                    "registered_depth_camera_info_topic": LaunchConfiguration(
                        "registered_depth_camera_info_topic"
                    ),
                    "start_depth_registration": LaunchConfiguration(
                        "start_depth_registration"
                    ),
                    "start_input_guard": LaunchConfiguration("start_input_guard"),
                    "require_head_lock": LaunchConfiguration("require_head_lock"),
                    "enforce_flat_floor": LaunchConfiguration(
                        "enforce_flat_floor"
                    ),
                    "max_floor_z_drift_m": LaunchConfiguration(
                        "max_floor_z_drift_m"
                    ),
                    "max_floor_tilt_rad": LaunchConfiguration(
                        "max_floor_tilt_rad"
                    ),
                    "rtabmap_approx_sync": LaunchConfiguration(
                        "rtabmap_approx_sync"
                    ),
                    "use_realsense_internal_tf": LaunchConfiguration(
                        "use_realsense_internal_tf"
                    ),
                    "head_link_x": LaunchConfiguration("head_link_x"),
                    "head_link_y": LaunchConfiguration("head_link_y"),
                    "head_link_z": LaunchConfiguration("head_link_z"),
                    "head_link_roll": LaunchConfiguration("head_link_roll"),
                    "head_link_pitch": LaunchConfiguration("head_link_pitch"),
                    "head_link_yaw": LaunchConfiguration("head_link_yaw"),
                    "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                    "imu_topic": LaunchConfiguration("imu_topic"),
                    "rtabmap_use_imu": LaunchConfiguration("rtabmap_use_imu"),
                    "rtabmap_filter_imu": LaunchConfiguration(
                        "rtabmap_filter_imu"
                    ),
                    "rtabmap_force_3dof": LaunchConfiguration(
                        "rtabmap_force_3dof"
                    ),
                    "joint_states_topic": "/hardware/joint_states",
                    "body_pose_topic": "/nav/base_footprint/pose",
                    "nav_camera_pose_topic": "/nav/head_depth_camera/pose",
                    "nav_camera_frame_id": LaunchConfiguration(
                        "nav_camera_frame_id"
                    ),
                    "body_camera_uses_head_mount": LaunchConfiguration(
                        "body_camera_uses_head_mount"
                    ),
                    "camera_width": LaunchConfiguration("camera_width"),
                    "camera_height": LaunchConfiguration("camera_height"),
                }.items(),
            ),
        ]
    )
