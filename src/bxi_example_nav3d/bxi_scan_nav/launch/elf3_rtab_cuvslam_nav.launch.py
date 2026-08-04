from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    scan_share = get_package_share_path("bxi_scan_nav")
    cuvslam_share = get_package_share_path("bxi_cuvslam_localization")
    elf3_share = get_package_share_path("bxi_example_py_elf3")
    rtabmap_share = get_package_share_path("rtabmap_launch")
    workspace_root = scan_share.parent.parent
    if workspace_root.name == "install":
        workspace_root = workspace_root.parent

    rtab_map = workspace_root / "maps" / "elf3_rtabmap_20260803_201219.db"
    cuvslam_map = workspace_root / "maps" / "cuvslam_elf3_20260803_201219"
    octo_cloud = (
        workspace_root / "maps" / "elf3_rtabmap_20260803_201219_octo_cloud.ply"
    )
    model = (
        elf3_share
        / "data"
        / "mujoco_simulation"
        / "elf3_rooms_nav3d_features.xml"
    )

    mature_nav_launch = scan_share / "launch" / "elf3_nav3d.launch.py"
    cuvslam_launch = cuvslam_share / "launch" / "elf3_head_cuvslam.launch.py"
    rtabmap_launch = rtabmap_share / "launch" / "rtabmap.launch.py"
    rtabmap_args = (
        "--Rtabmap/DetectionRate 2 "
        "--Reg/Force3DoF false "
        "--RGBD/ForceOdom3DoF false "
        "--Grid/3D true "
        "--Grid/RayTracing true "
        "--RGBD/CreateOccupancyGrid true "
        "--Grid/RangeMax 4.5 "
        "--Grid/MinClusterSize 5"
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                "FASTRTPS_DEFAULT_PROFILES_FILE",
                str(cuvslam_share / "config" / "fastdds_large_data.xml"),
            ),
            DeclareLaunchArgument("rtabmap_database", default_value=str(rtab_map)),
            DeclareLaunchArgument("cuvslam_map", default_value=str(cuvslam_map)),
            DeclareLaunchArgument("octo_input_pcd", default_value=str(octo_cloud)),
            DeclareLaunchArgument("model_file", default_value=str(model)),
            DeclareLaunchArgument("start_simulation", default_value="true"),
            DeclareLaunchArgument("start_simulated_cameras", default_value="true"),
            DeclareLaunchArgument("start_controller", default_value="true"),
            DeclareLaunchArgument("start_scan_planner", default_value="true"),
            DeclareLaunchArgument("start_octo_global_planner", default_value="true"),
            DeclareLaunchArgument("start_rtabmap", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("camera_width", default_value="640"),
            DeclareLaunchArgument("camera_height", default_value="360"),
            DeclareLaunchArgument("camera_hz", default_value="30.0"),
            DeclareLaunchArgument("octo_cloud_scale", default_value="1.0"),
            DeclareLaunchArgument("octo_resolution", default_value="0.20"),
            DeclareLaunchArgument("octo_robot_radius", default_value="0.18"),
            DeclareLaunchArgument("octo_max_iterations", default_value="1000000"),
            DeclareLaunchArgument("octo_require_ground_support", default_value="true"),
            DeclareLaunchArgument(
                "octo_ground_support_xy_radius_cells", default_value="2"
            ),
            DeclareLaunchArgument(
                "octo_ground_support_depth_cells", default_value="3"
            ),
            DeclareLaunchArgument(
                "octo_enable_preblocked_costmap", default_value="false"
            ),
            DeclareLaunchArgument("octo_enable_clearance_cost", default_value="true"),
            DeclareLaunchArgument(
                "octo_clearance_cost_radius_cells", default_value="4"
            ),
            DeclareLaunchArgument("octo_clearance_cost_weight", default_value="0.8"),
            DeclareLaunchArgument("octo_vertical_padding_below", default_value="1.0"),
            DeclareLaunchArgument("octo_vertical_padding_above", default_value="0.6"),
            DeclareLaunchArgument("octo_start_z_offset", default_value="0.30"),
            DeclareLaunchArgument("octo_goal_z_offset", default_value="0.90"),
            DeclareLaunchArgument("octo_path_z_offset", default_value="0.0"),
            DeclareLaunchArgument(
                "octo_enforce_path_ground_clearance", default_value="true"
            ),
            DeclareLaunchArgument(
                "octo_path_min_ground_clearance", default_value="0.75"
            ),
            DeclareLaunchArgument(
                "octo_path_ground_search_depth", default_value="1.50"
            ),
            DeclareLaunchArgument(
                "octo_path_ground_xy_radius_cells", default_value="1"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(mature_nav_launch)),
                launch_arguments={
                    "start_simulation": LaunchConfiguration("start_simulation"),
                    "start_simulated_cameras": LaunchConfiguration(
                        "start_simulated_cameras"
                    ),
                    "publish_simulated_camera_pose": "false",
                    "require_localization_valid": "true",
                    "start_controller": LaunchConfiguration("start_controller"),
                    "start_scan_planner": LaunchConfiguration("start_scan_planner"),
                    "start_octo_global_planner": LaunchConfiguration(
                        "start_octo_global_planner"
                    ),
                    "start_clicked_point_3d_goal": "true",
                    "start_rviz": LaunchConfiguration("start_rviz"),
                    "model_file": LaunchConfiguration("model_file"),
                    "body_pose_topic": "/nav/base_footprint/pose",
                    "global_frame": "world",
                    "nav_camera_name": "head_depth_camera",
                    "nav_camera_output_prefix": "/simulation/d435i/",
                    "nav_camera_frame_id": (
                        "head_depth_camera_depth_optical_frame"
                    ),
                    "nav_depth_topic": "/simulation/d435i/depth/image_rect_raw",
                    "nav_camera_pose_topic": "/nav/head_depth_camera/pose",
                    "camera_width": LaunchConfiguration("camera_width"),
                    "camera_height": LaunchConfiguration("camera_height"),
                    "sim_camera_publish_hz": LaunchConfiguration("camera_hz"),
                    "sim_camera_publish_color": "true",
                    "sim_camera_align_depth_to_color": "true",
                    "camera_fx": "196.43553019246882",
                    "camera_fy": "196.43553019246882",
                    "camera_cx": "320.0",
                    "camera_cy": "180.0",
                    "input_pcd": LaunchConfiguration("octo_input_pcd"),
                    "octo_cloud_scale": LaunchConfiguration("octo_cloud_scale"),
                    "octo_resolution": LaunchConfiguration("octo_resolution"),
                    "octo_robot_radius": LaunchConfiguration("octo_robot_radius"),
                    "octo_max_iterations": LaunchConfiguration(
                        "octo_max_iterations"
                    ),
                    "octo_require_ground_support": LaunchConfiguration(
                        "octo_require_ground_support"
                    ),
                    "octo_ground_support_xy_radius_cells": LaunchConfiguration(
                        "octo_ground_support_xy_radius_cells"
                    ),
                    "octo_ground_support_depth_cells": LaunchConfiguration(
                        "octo_ground_support_depth_cells"
                    ),
                    "octo_enable_preblocked_costmap": LaunchConfiguration(
                        "octo_enable_preblocked_costmap"
                    ),
                    "octo_enable_clearance_cost": LaunchConfiguration(
                        "octo_enable_clearance_cost"
                    ),
                    "octo_clearance_cost_radius_cells": LaunchConfiguration(
                        "octo_clearance_cost_radius_cells"
                    ),
                    "octo_clearance_cost_weight": LaunchConfiguration(
                        "octo_clearance_cost_weight"
                    ),
                    "octo_vertical_padding_below": LaunchConfiguration(
                        "octo_vertical_padding_below"
                    ),
                    "octo_vertical_padding_above": LaunchConfiguration(
                        "octo_vertical_padding_above"
                    ),
                    "octo_start_z_offset": LaunchConfiguration("octo_start_z_offset"),
                    "octo_goal_z_offset": LaunchConfiguration("octo_goal_z_offset"),
                    "octo_path_z_offset": LaunchConfiguration("octo_path_z_offset"),
                    "octo_enforce_path_ground_clearance": LaunchConfiguration(
                        "octo_enforce_path_ground_clearance"
                    ),
                    "octo_path_min_ground_clearance": LaunchConfiguration(
                        "octo_path_min_ground_clearance"
                    ),
                    "octo_path_ground_search_depth": LaunchConfiguration(
                        "octo_path_ground_search_depth"
                    ),
                    "octo_path_ground_xy_radius_cells": LaunchConfiguration(
                        "octo_path_ground_xy_radius_cells"
                    ),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(rtabmap_launch)),
                condition=IfCondition(LaunchConfiguration("start_rtabmap")),
                launch_arguments={
                    "stereo": "false",
                    "localization": "true",
                    "rtabmap_viz": "false",
                    "rviz": "false",
                    "use_sim_time": "false",
                    "frame_id": "bxi_base_link",
                    "map_frame_id": "world",
                    "publish_tf_map": "true",
                    "namespace": "rtabmap",
                    "database_path": LaunchConfiguration("rtabmap_database"),
                    "topic_queue_size": "100",
                    "queue_size": "100",
                    "qos": "2",
                    "qos_image": "2",
                    "qos_camera_info": "2",
                    "qos_odom": "1",
                    "wait_for_transform": "0.1",
                    "approx_sync": "true",
                    "approx_sync_max_interval": "0.50",
                    "rgb_topic": "/simulation/d435i/color/image_raw",
                    "depth_topic": "/simulation/d435i/depth/image_rect_raw",
                    "camera_info_topic": "/simulation/d435i/depth/camera_info",
                    "depth": "true",
                    "rgbd_sync": "true",
                    "approx_rgbd_sync": "true",
                    "subscribe_rgbd": "true",
                    "visual_odometry": "false",
                    "icp_odometry": "false",
                    "odom_topic": "/nav/odom",
                    "publish_tf_odom": "false",
                    "map_topic": "map",
                    "args": rtabmap_args,
                    "output": "screen",
                }.items(),
            ),
            TimerAction(
                period=5.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(str(cuvslam_launch)),
                        launch_arguments={
                            "mode": "localization",
                            "map_frame": "odom",
                            "pose_target_frame": "world",
                            "map_directory": LaunchConfiguration("cuvslam_map"),
                            "head_color_topic": "/simulation/d435i/color/image_raw",
                            "head_depth_topic": (
                                "/simulation/d435i/depth/image_rect_raw"
                            ),
                            "head_camera_info_topic": (
                                "/simulation/d435i/color/camera_info"
                            ),
                            "joint_states_topic": "/simulation/joint_states",
                            "imu_topic": "/simulation/imu_data",
                            "start_imu_combiner": "false",
                            "start_rig_guard": "true",
                            "start_odom_bridge": "true",
                            "start_pose_publishers": "true",
                            "start_sim_error_monitor": "true",
                            "sim_truth_odom_topic": "/simulation/odom",
                            "require_imu": "false",
                            "require_head_lock": "true",
                            "require_initial_pose": "false",
                            "auto_relocalize": "true",
                            "enable_slam_backend": "true",
                            "use_gpu": "true",
                            "async_sba": "true",
                            "image_timeout_sec": "1.0",
                            "odom_timeout_sec": "0.75",
                        }.items(),
                    )
                ],
            ),
        ]
    )
