from ament_index_python.packages import PackageNotFoundError, get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import tempfile
import yaml


def start_mapping_rviz(context, config_path):
    start_rviz = LaunchConfiguration("start_rviz").perform(context).lower()
    localization = LaunchConfiguration("rtabmap_localization").perform(context).lower()
    if start_rviz not in ("1", "true", "yes", "on"):
        return []
    if localization in ("1", "true", "yes", "on"):
        return []
    global_frame = LaunchConfiguration("global_frame").perform(context)
    with open(config_path, encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)
    config["Visualization Manager"]["Global Options"]["Fixed Frame"] = global_frame
    with tempfile.NamedTemporaryFile(
        mode="w",
        prefix="bxi_rtab_mapping_",
        suffix=".rviz",
        encoding="utf-8",
        delete=False,
    ) as generated_config:
        yaml.safe_dump(config, generated_config, sort_keys=False)
        rviz_config_path = generated_config.name
    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rtab_mapping_rviz",
            output="screen",
            arguments=["-d", rviz_config_path],
        )
    ]


def generate_launch_description():
    scan_share = get_package_share_path("bxi_scan_nav")
    cuvslam_share = get_package_share_path("bxi_cuvslam_localization")
    orb_share = get_package_share_path("bxi_orbslam3_ros2")
    rtabmap_share = get_package_share_path("rtabmap_launch")
    workspace_root = scan_share.parent.parent
    if workspace_root.name == "install":
        workspace_root = workspace_root.parent

    rtab_map = workspace_root / "maps" / "elf3_rtabmap_20260803_201219.db"
    cuvslam_map = workspace_root / "maps" / "cuvslam_elf3_20260803_201219"
    octo_cloud = (
        workspace_root / "maps" / "elf3_rtabmap_20260803_201219_octo_cloud.ply"
    )
    # bxi_example_py_elf3 is only needed when this launch also starts the
    # simulation.  Hardware deployments build bxi_example_nav3d alone, so do
    # not make the optional simulation package a launch-time dependency.
    try:
        elf3_share = get_package_share_path("bxi_example_py_elf3")
        model = (
            elf3_share
            / "data"
            / "mujoco_simulation"
            / "elf3_rooms_nav3d_features.xml"
        )
    except PackageNotFoundError:
        model = ""

    mature_nav_launch = scan_share / "launch" / "elf3_nav3d.launch.py"
    localization_launch = cuvslam_share / "launch" / "elf3_localization.launch.py"
    rig_tf_launch = cuvslam_share / "launch" / "rig_static_tf.launch.py"
    rtabmap_launch = rtabmap_share / "launch" / "rtabmap.launch.py"
    mapping_rviz_config = scan_share / "config" / "elf3_rtab_mapping.rviz"
    navigation_rviz_enabled = PythonExpression(
        [
            "'",
            LaunchConfiguration("start_rviz"),
            "'.lower() == 'true' and '",
            LaunchConfiguration("rtabmap_localization"),
            "'.lower() == 'true'",
        ]
    )
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
            DeclareLaunchArgument(
                "localization_backend", default_value="rtabmap"
            ),
            DeclareLaunchArgument("localization_mode", default_value="localization"),
            DeclareLaunchArgument(
                "orb_map_directory",
                default_value=str(workspace_root / "maps" / "elf3_orbslam3"),
            ),
            DeclareLaunchArgument("orb_atlas_name", default_value="atlas"),
            DeclareLaunchArgument("orb_max_time_diff", default_value="0.01"),
            DeclareLaunchArgument(
                "orb_settings",
                default_value=str(
                    orb_share / "config" / "elf3_head_640x360_rgbd_imu.yaml"
                ),
            ),
            DeclareLaunchArgument("octo_input_pcd", default_value=str(octo_cloud)),
            DeclareLaunchArgument("model_file", default_value=str(model)),
            DeclareLaunchArgument("global_frame", default_value="world"),
            DeclareLaunchArgument(
                "color_topic", default_value="/simulation/d435i/color/image_raw"
            ),
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/simulation/d435i/depth/image_rect_raw",
            ),
            DeclareLaunchArgument(
                "raw_depth_topic",
                default_value="/simulation/d435i/depth/image_rect_raw",
            ),
            DeclareLaunchArgument(
                "depth_camera_info_topic",
                default_value="/simulation/d435i/depth/camera_info",
            ),
            DeclareLaunchArgument(
                "registered_depth_camera_info_topic",
                default_value="/simulation/d435i/depth/camera_info",
            ),
            DeclareLaunchArgument(
                "start_depth_registration", default_value="false"
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/simulation/d435i/color/camera_info",
            ),
            DeclareLaunchArgument("imu_topic", default_value="/simulation/imu_data"),
            DeclareLaunchArgument("rtabmap_use_imu", default_value="false"),
            DeclareLaunchArgument("rtabmap_filter_imu", default_value="false"),
            DeclareLaunchArgument(
                "rtabmap_motion_profile", default_value="stable"
            ),
            DeclareLaunchArgument(
                "rtabmap_mapping_profile", default_value="balanced"
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "rtabmap_odom_always_process_most_recent_frame",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "joint_states_topic", default_value="/simulation/joint_states"
            ),
            DeclareLaunchArgument(
                "body_pose_topic", default_value="/nav/base_footprint/pose"
            ),
            DeclareLaunchArgument(
                "nav_camera_pose_topic", default_value="/nav/head_depth_camera/pose"
            ),
            DeclareLaunchArgument(
                "nav_camera_frame_id",
                default_value="head_depth_camera_depth_optical_frame",
            ),
            DeclareLaunchArgument(
                "body_camera_uses_head_mount", default_value="false"
            ),
            DeclareLaunchArgument("start_simulation", default_value="true"),
            DeclareLaunchArgument("start_simulated_cameras", default_value="true"),
            DeclareLaunchArgument("start_controller", default_value="true"),
            DeclareLaunchArgument("start_scan_planner", default_value="true"),
            DeclareLaunchArgument("start_octo_global_planner", default_value="true"),
            DeclareLaunchArgument("start_rtabmap", default_value="true"),
            DeclareLaunchArgument("rtabmap_localization", default_value="true"),
            DeclareLaunchArgument(
                "rtabmap_topic_queue_size", default_value="100"
            ),
            DeclareLaunchArgument(
                "rtabmap_sync_queue_size", default_value="100"
            ),
            DeclareLaunchArgument(
                "rtabmap_approx_sync_max_interval", default_value="0.04"
            ),
            DeclareLaunchArgument(
                "rtabmap_odom_sensor_sync", default_value="true"
            ),
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
            OpaqueFunction(
                function=start_mapping_rviz,
                args=[str(mapping_rviz_config)],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(rig_tf_launch)),
                condition=IfCondition(
                    LaunchConfiguration("start_depth_registration")
                ),
                launch_arguments={
                    "body_camera_uses_head_mount": LaunchConfiguration(
                        "body_camera_uses_head_mount"
                    ),
                }.items(),
            ),
            Node(
                package="bxi_orbslam3_ros2",
                executable="sensor_qos_relay",
                name="depth_registration_qos_relay",
                output="screen",
                condition=IfCondition(
                    LaunchConfiguration("start_depth_registration")
                ),
                parameters=[
                    {
                        "depth_input": LaunchConfiguration("raw_depth_topic"),
                        "depth_info_input": LaunchConfiguration(
                            "depth_camera_info_topic"
                        ),
                        "color_info_input": LaunchConfiguration(
                            "camera_info_topic"
                        ),
                        "depth_output": (
                            "/nav/depth_registration/raw_depth_reliable"
                        ),
                        "depth_info_output": (
                            "/nav/depth_registration/depth_info_reliable"
                        ),
                        "color_info_output": (
                            "/nav/depth_registration/color_info_reliable"
                        ),
                    }
                ],
            ),
            Node(
                package="depth_image_proc",
                executable="register_node",
                name="nav_depth_register",
                output="screen",
                condition=IfCondition(
                    LaunchConfiguration("start_depth_registration")
                ),
                remappings=[
                    (
                        "depth/image_rect",
                        "/nav/depth_registration/raw_depth_reliable",
                    ),
                    (
                        "depth/camera_info",
                        "/nav/depth_registration/depth_info_reliable",
                    ),
                    (
                        "rgb/camera_info",
                        "/nav/depth_registration/color_info_reliable",
                    ),
                    ("depth_registered/image_rect", LaunchConfiguration("depth_topic")),
                    (
                        "depth_registered/camera_info",
                        LaunchConfiguration("registered_depth_camera_info_topic"),
                    ),
                ],
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
                    "start_rviz": navigation_rviz_enabled,
                    "model_file": LaunchConfiguration("model_file"),
                    "body_pose_topic": LaunchConfiguration("body_pose_topic"),
                    "global_frame": LaunchConfiguration("global_frame"),
                    "nav_camera_name": "head_depth_camera",
                    "nav_camera_output_prefix": "/simulation/d435i/",
                    "nav_camera_frame_id": LaunchConfiguration(
                        "nav_camera_frame_id"
                    ),
                    "nav_depth_topic": LaunchConfiguration("depth_topic"),
                    "nav_camera_pose_topic": LaunchConfiguration(
                        "nav_camera_pose_topic"
                    ),
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
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("start_rtabmap"),
                            "'.lower() == 'true' and '",
                            LaunchConfiguration("localization_backend"),
                            "' != 'rtabmap'",
                        ]
                    )
                ),
                launch_arguments={
                    "stereo": "false",
                    "localization": LaunchConfiguration("rtabmap_localization"),
                    "rtabmap_viz": "false",
                    "rviz": "false",
                    "use_sim_time": "false",
                    "frame_id": "bxi_base_link",
                    "map_frame_id": LaunchConfiguration("global_frame"),
                    "publish_tf_map": "true",
                    "namespace": "rtabmap",
                    "database_path": LaunchConfiguration("rtabmap_database"),
                    "topic_queue_size": LaunchConfiguration(
                        "rtabmap_topic_queue_size"
                    ),
                    "queue_size": LaunchConfiguration(
                        "rtabmap_sync_queue_size"
                    ),
                    "qos": "2",
                    "qos_image": "2",
                    "qos_camera_info": "2",
                    "qos_odom": "1",
                    "wait_for_transform": "0.1",
                    "approx_sync": "true",
                    "approx_sync_max_interval": LaunchConfiguration(
                        "rtabmap_approx_sync_max_interval"
                    ),
                    "rgb_topic": LaunchConfiguration("color_topic"),
                    "depth_topic": LaunchConfiguration("depth_topic"),
                    "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                    "depth": "true",
                    "rgbd_sync": PythonExpression(
                        [
                            "'false' if '",
                            LaunchConfiguration("localization_backend"),
                            "' == 'orbslam3' else 'true'",
                        ]
                    ),
                    "rgbd_topic": "/localization/orbslam3/rgbd_image",
                    "approx_rgbd_sync": "true",
                    "subscribe_rgbd": "true",
                    "visual_odometry": "false",
                    "icp_odometry": "false",
                    "odom_topic": "/nav/odom",
                    "odom_sensor_sync": LaunchConfiguration(
                        "rtabmap_odom_sensor_sync"
                    ),
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
                        PythonLaunchDescriptionSource(str(localization_launch)),
                        launch_arguments={
                            "localization_backend": LaunchConfiguration(
                                "localization_backend"
                            ),
                            "mode": LaunchConfiguration("localization_mode"),
                            "map_frame": LaunchConfiguration("global_frame"),
                            "odom_frame": "odom",
                            "pose_target_frame": LaunchConfiguration("global_frame"),
                            "rtabmap_database": LaunchConfiguration(
                                "rtabmap_database"
                            ),
                            "rtabmap_approx_sync_max_interval": LaunchConfiguration(
                                "rtabmap_approx_sync_max_interval"
                            ),
                            "cuvslam_map_directory": LaunchConfiguration(
                                "cuvslam_map"
                            ),
                            "orb_map_directory": LaunchConfiguration(
                                "orb_map_directory"
                            ),
                            "orb_atlas_name": LaunchConfiguration("orb_atlas_name"),
                            "orb_max_time_diff": LaunchConfiguration(
                                "orb_max_time_diff"
                            ),
                            "orb_settings": LaunchConfiguration("orb_settings"),
                            "color_topic": LaunchConfiguration("color_topic"),
                            "depth_topic": LaunchConfiguration("depth_topic"),
                            "camera_info_topic": LaunchConfiguration(
                                "camera_info_topic"
                            ),
                            "nav_camera_frame_id": LaunchConfiguration(
                                "nav_camera_frame_id"
                            ),
                            "nav_camera_pose_topic": LaunchConfiguration(
                                "nav_camera_pose_topic"
                            ),
                            "body_camera_uses_head_mount": LaunchConfiguration(
                                "body_camera_uses_head_mount"
                            ),
                            "start_rig_tf": PythonExpression(
                                [
                                    "'",
                                    LaunchConfiguration("start_depth_registration"),
                                    "'.lower() != 'true'",
                                ]
                            ),
                            "joint_states_topic": LaunchConfiguration(
                                "joint_states_topic"
                            ),
                            "imu_topic": LaunchConfiguration("imu_topic"),
                            "rtabmap_use_imu": LaunchConfiguration(
                                "rtabmap_use_imu"
                            ),
                            "rtabmap_filter_imu": LaunchConfiguration(
                                "rtabmap_filter_imu"
                            ),
                            "rtabmap_motion_profile": LaunchConfiguration(
                                "rtabmap_motion_profile"
                            ),
                            "rtabmap_mapping_profile": LaunchConfiguration(
                                "rtabmap_mapping_profile"
                            ),
                            "use_sim_time": LaunchConfiguration("use_sim_time"),
                            "rtabmap_odom_always_process_most_recent_frame": LaunchConfiguration(
                                "rtabmap_odom_always_process_most_recent_frame"
                            ),
                            "cuvslam_use_gpu": "true",
                        }.items(),
                    )
                ],
            ),
        ]
    )
