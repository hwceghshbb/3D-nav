from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    scan_share = get_package_share_path("bxi_scan_nav")
    install_prefix = scan_share.parent.parent
    workspace_root = (
        install_prefix.parent if install_prefix.name == "install" else install_prefix
    )
    map_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    default_cuvslam_map = (
        workspace_root / "maps" / f"cuvslam_elf3_{map_timestamp}"
    )
    scan_launch = scan_share / "launch" / "elf3_rtab_scan_nav.launch.py"
    joystick_config = (
        scan_share / "config" / "joystick_mapping_safe.yaml"
    )
    feature_model = (
        get_package_share_path("bxi_example_py_elf3")
        / "data"
        / "mujoco_simulation"
        / "elf3_rooms_nav3d_features.xml"
    )
    cuvslam_launch = (
        get_package_share_path("bxi_cuvslam_localization")
        / "launch"
        / "elf3_head_cuvslam.launch.py"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("model_file", default_value=str(feature_model)),
            DeclareLaunchArgument(
                "database_path", default_value="/tmp/bxi_elf3_features_rtabmap.db"
            ),
            DeclareLaunchArgument("delete_db_on_start", default_value="true"),
            DeclareLaunchArgument(
                "cuvslam_map_directory",
                default_value=str(default_cuvslam_map),
            ),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("start_joystick", default_value="true"),
            DeclareLaunchArgument("mapping_camera_width", default_value="424"),
            DeclareLaunchArgument("mapping_camera_height", default_value="240"),
            DeclareLaunchArgument("mapping_camera_hz", default_value="30.0"),
            DeclareLaunchArgument("cuvslam_use_gpu", default_value="true"),
            DeclareLaunchArgument("cuvslam_async_sba", default_value="true"),
            DeclareLaunchArgument(
                "cuvslam_enable_slam_backend", default_value="true"
            ),
            DeclareLaunchArgument(
                "cuvslam_error_record_path",
                default_value="/tmp/cuvslam_large_error_events.jsonl",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(scan_launch)),
                launch_arguments={
                    "start_controller": "true",
                    "start_scan_planner": "false",
                    "start_global_planner": "false",
                    "start_pct_global_planner": "false",
                    "start_clicked_point_3d_goal": "false",
                    "start_keyboard_teleop": "false",
                    "start_rviz": LaunchConfiguration("start_rviz"),
                    "start_rtabmap": "true",
                    "localization": "false",
                    "delete_db_on_start": LaunchConfiguration(
                        "delete_db_on_start"
                    ),
                    "database_path": LaunchConfiguration("database_path"),
                    "model_file": LaunchConfiguration("model_file"),
                    "mapping_camera_name": "head_depth_camera",
                    "mapping_camera_width": LaunchConfiguration(
                        "mapping_camera_width"
                    ),
                    "mapping_camera_height": LaunchConfiguration(
                        "mapping_camera_height"
                    ),
                    "mapping_camera_hz": LaunchConfiguration(
                        "mapping_camera_hz"
                    ),
                    "mapping_camera_frame_id": (
                        "head_depth_camera_depth_optical_frame"
                    ),
                    "publish_simulated_camera_pose": "false",
                    "rtabmap_odom_topic": (
                        "/nav/odom"
                    ),
                    "global_frame": "world",
                    "depth_policy_camera_name": "body_depth_camera",
                    "rtabmap_viz": "false",
                }.items(),
            ),
            TimerAction(
                period=10.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(str(cuvslam_launch)),
                        launch_arguments={
                    "mode": "mapping",
                    "map_directory": LaunchConfiguration(
                        "cuvslam_map_directory"
                    ),
                    "head_color_topic": (
                        "/simulation/d435i/color/image_raw"
                    ),
                    "head_depth_topic": (
                        "/simulation/d435i/depth/image_rect_raw"
                    ),
                    "head_camera_info_topic": (
                        "/simulation/d435i/color/camera_info"
                    ),
                    "joint_states_topic": "/simulation/joint_states",
                    "map_anchor_odom_topic": "/simulation/odom",
                    "map_anchor_pose_topic": "",
                    "require_map_anchor": "true",
                    "imu_topic": "/simulation/imu_data",
                    "start_imu_combiner": "false",
                    "start_odom_bridge": "true",
                    "start_pose_publishers": "false",
                    "start_sim_error_monitor": "true",
                    "sim_truth_odom_topic": "/simulation/odom",
                    "sim_error_record_path": LaunchConfiguration(
                        "cuvslam_error_record_path"
                    ),
                    "enable_slam_backend": LaunchConfiguration(
                        "cuvslam_enable_slam_backend"
                    ),
                    "use_gpu": LaunchConfiguration("cuvslam_use_gpu"),
                    "async_sba": LaunchConfiguration("cuvslam_async_sba"),
                    "require_imu": "false",
                    "require_head_lock": "true",
                    "image_timeout_sec": "1.0",
                    "odom_timeout_sec": "0.75",
                    "auto_relocalize": "false",
                    # cuVSLAM is continuous local odometry. RTAB-Map owns the
                    # global world -> odom correction transform.
                    "map_frame": "odom",
                        }.items(),
                    )
                ],
            ),
            Node(
                package="remote_controller",
                executable="remote_controller",
                name="mapping_joystick_teleop",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(LaunchConfiguration("start_joystick")),
                arguments=[
                    "--config",
                    str(joystick_config),
                    "--driver",
                    "joystick",
                    "--hot-reload",
                    "true",
                    "__log_level:=debug",
                ],
            ),
        ]
    )
