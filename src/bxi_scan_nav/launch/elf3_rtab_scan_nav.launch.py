import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def as_bool(value):
    return str(value).lower() in ("1", "true", "yes", "on")


def launch_setup(context, *args, **kwargs):
    nav_share = get_package_share_path("bxi_scan_nav")
    elf3_share = get_package_share_path("bxi_example_py_elf3")
    scan_share = get_package_share_path("scan_planner")
    rtabmap_share = get_package_share_path("rtabmap_launch")

    model = os.path.join(elf3_share, "data", "mujoco_simulation", "elf3_rooms_datagen.xml")
    onnx_file = os.path.join(elf3_share, "data", "mjlab_model", "model_normal.onnx")
    depth_policy_file = os.path.join(elf3_share, "data", "depth_policy", "lyp2", "dagger1.onnx")
    nav_config = os.path.join(nav_share, "config", "navigation.yaml")
    rviz_config = os.path.join(nav_share, "config", "elf3_rtab_mapping.rviz")
    planner_yaml = os.path.join(scan_share, "config", "planner.yaml")
    controllers_yaml = os.path.join(scan_share, "config", "controllers.yaml")
    nav2_planner_yaml = os.path.join(nav_share, "config", "nav2_global_planner.yaml")
    rtabmap_launch = os.path.join(rtabmap_share, "launch", "rtabmap.launch.py")
    stop_script = os.path.join(nav_share, "scripts", "stop_bxi_scan_nav_processes.sh")

    rtabmap_args = LaunchConfiguration("rtabmap_args").perform(context).strip()
    navigation_3d = as_bool(LaunchConfiguration("navigation_3d").perform(context))
    if as_bool(LaunchConfiguration("delete_db_on_start").perform(context)):
        rtabmap_args = (rtabmap_args + " --delete_db_on_start").strip()
    if navigation_3d:
        rtabmap_args = (
            rtabmap_args
            + " --Reg/Force3DoF false"
            + " --RGBD/ForceOdom3DoF false"
            + " --Grid/3D true"
            + " --Grid/RayTracing true"
            + " --Grid/RangeMax 5.0"
            + " --RGBD/CreateOccupancyGrid true"
        ).strip()

    planner_overrides = {
        "use_sim_time": False,
        "fsm.navi_mode": 3,
        "fsm.navigation_z": 0.8,
        "fsm.use_path_z": navigation_3d,
        "fsm.use_odom_z": navigation_3d,
        "fsm.thresh_replan": 1.0,
        "fsm.thresh_no_replan": 0.1,
        "fsm.planning_horizon": 3.0,
        "fsm.emergency_time": 1.0,
        "fsm.fail_safe": True,
        "fsm.max_replan_fail_count": 1000,
        "fsm.exec_interval_ms": 50,
        "fsm.safety_interval_ms": 100,
        "fsm.collision_check_dt": 0.05,
        "fsm.collision_check_horizon": 2.0,
        "grid_map.resolution": 0.05,
        "grid_map.sliding_map_size_x": 6.0,
        "grid_map.sliding_map_size_y": 6.0,
        "grid_map.sliding_map_size_z": 3.2,
        "grid_map.local_update_range_x": 3.0,
        "grid_map.local_update_range_y": 3.0,
        "grid_map.local_update_range_z": 1.8,
        "grid_map.map_sliding_en": True,
        "grid_map.map_sliding_thresh": 0.2,
        "grid_map.sensor_type": "depth",
        "grid_map.cloud_is_world": False,
        "grid_map.need_extrinsic": False,
        "grid_map.cx": 212.0,
        "grid_map.cy": 120.0,
        "grid_map.fx": 130.95702012831255,
        "grid_map.fy": 130.95702012831255,
        "grid_map.depth_filter_maxdist": 3.2,
        "grid_map.depth_filter_mindist": 0.25,
        "grid_map.depth_filter_margin": 1,
        "grid_map.k_depth_scaling_factor": 1000.0,
        "grid_map.skip_pixel": 10,
        "grid_map.body_height": 1.2,
        "grid_map.double_cylinder_radius": 0.22,
        "grid_map.double_cylinder_offset": 0.10,
        "grid_map.obstacles_inflation_z_up": 0.15,
        "grid_map.obstacles_inflation_z_down": 0.10,
        "grid_map.frame_id": "world",
        "grid_map.sliding_map_frame_id": "sliding_map",
        "grid_map.ground_height": 0.0,
        "grid_map.ground_filter_height": 0.25,
        "grid_map.p_hit": 0.85,
        "grid_map.p_miss": 0.30,
        "grid_map.p_min": 0.12,
        "grid_map.p_max": 0.98,
        "grid_map.p_occ": 0.80,
        "grid_map.max_ray_length": 3.2,
        "grid_map.vis_height": 0.3,
        "grid_map.occ_interval_ms": 250,
        "grid_map.vis_interval_ms": 500,
        "grid_map.inflate_vis_use_z_slice": True,
        "grid_map.inflate_vis_z_min": 0.25,
        "grid_map.inflate_vis_z_max": 2.2,
        "grid_map.show_occ_time": False,
        "optimization.dist0": 0.18,
        "manager.max_vel": 0.35,
        "manager.max_acc": 0.35,
        "manager.max_jerk": 4.0,
        "manager.control_points_distance": 0.2,
        "manager.feasibility_tolerance": 0.5,
        "manager.planning_horizon": 3.0,
        "optimization.lambda_smooth": 1.0,
        "optimization.lambda_collision": 1.0,
        "optimization.lambda_feasibility": 0.1,
        "optimization.lambda_fitness": 1.0,
        "optimization.max_vel": 0.35,
        "optimization.max_acc": 0.35,
        "optimization.order": 3,
    }
    controller_overrides = {
        "max_vx": 0.35,
        "max_vy": 0.15,
        "max_vyaw": 0.55,
        "finish_dist": 0.25,
    }

    return [
        Node(
            package="mujoco",
            executable="simulation",
            name="simulation_mujoco",
            output="screen",
            parameters=[{"simulation/model_file": model}],
            emulate_tty=True,
        ),
        Node(
            package="bxi_example_py_elf3",
            executable="bxi_example_py_elf3_mjlab",
            name="elf3_policy",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_controller")),
            parameters=[
                {"/topic_prefix": "simulation/"},
                {"/onnx_file": onnx_file},
                {"/policy_type": LaunchConfiguration("controller_policy")},
                {"/depth_policy_file": depth_policy_file},
                {"/depth_image_topic": "/simulation/origin_depth/depth/image_raw"},
            ],
            emulate_tty=True,
        ),
        Node(
            package="bxi_scan_nav",
            executable="rgbd_camera_publisher",
            name="d435i_rgbd",
            output="screen",
            parameters=[
                {
                    "model_file": model,
                    "width": 424,
                    "height": 240,
                    "publish_hz": 5.0,
                }
            ],
            emulate_tty=True,
        ),
        Node(
            package="bxi_scan_nav",
            executable="rgbd_camera_publisher",
            name="origin_depth_rgbd",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_depth_policy_camera")),
            parameters=[
                {
                    "model_file": model,
                    "camera_name": "origin_depth_cam",
                    "output_prefix": "/simulation/origin_depth/",
                    "frame_id": "origin_depth_cam_optical_frame",
                    "hidden_body_names": ["torso_link"],
                    "width": 48,
                    "height": 36,
                    "publish_hz": 20.0,
                }
            ],
            emulate_tty=True,
        ),
        Node(
            package="bxi_scan_nav",
            executable="d435i_pose_publisher",
            name="d435i_pose_publisher",
            output="screen",
            parameters=[
                nav_config,
                {
                    "preserve_base_footprint_z": navigation_3d,
                    "base_footprint_z_offset": -0.3,
                },
            ],
            emulate_tty=True,
        ),
        Node(
            package="bxi_scan_nav",
            executable="depth_to_pointcloud",
            name="d435i_depth_points",
            output="screen",
            parameters=[
                {
                    "depth_topic": "/simulation/d435i/depth/image_raw",
                    "camera_info_topic": "/simulation/d435i/depth/camera_info",
                    "pointcloud_topic": "/simulation/d435i/depth/points",
                    "pointcloud_topic_aliases": ["/cloudPointMapping"],
                    "stride": 10,
                    "invert_x": False,
                    "invert_y": False,
                }
            ],
            emulate_tty=True,
        ),
        Node(
            package="bxi_scan_nav",
            executable="keyboard_motion_teleop",
            name="mapping_keyboard_teleop",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_keyboard_teleop")),
            parameters=[
                {
                    "motion_commands_topic": "motion_commands",
                    "publish_hz": 50.0,
                    "max_vx": 0.18,
                    "max_vy": 0.08,
                    "max_yaw": 0.25,
                }
            ],
            emulate_tty=True,
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_rviz")),
            arguments=["-d", rviz_config],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch),
            condition=IfCondition(LaunchConfiguration("start_rtabmap")),
            launch_arguments={
                "stereo": "false",
                "localization": LaunchConfiguration("localization"),
                "rtabmap_viz": LaunchConfiguration("rtabmap_viz"),
                "rviz": "false",
                "use_sim_time": "false",
                "frame_id": "bxi_base_link" if navigation_3d else "bxi_base_footprint",
                "map_frame_id": "world",
                "publish_tf_map": "false",
                "namespace": "rtabmap",
                "database_path": LaunchConfiguration("database_path"),
                "topic_queue_size": "100",
                "queue_size": "100",
                "qos": "1",
                "qos_image": "1",
                "qos_camera_info": "1",
                "qos_odom": "1",
                "wait_for_transform": "0.1",
                "approx_sync": "true",
                "approx_sync_max_interval": "0.50",
                "rgb_topic": "/simulation/d435i/color/image_raw",
                "depth_topic": "/simulation/d435i/depth/image_raw",
                "camera_info_topic": "/simulation/d435i/depth/camera_info",
                "depth": "true",
                "rgbd_sync": "true",
                "approx_rgbd_sync": "true",
                "subscribe_rgbd": "true",
                "visual_odometry": "false",
                "icp_odometry": "false",
                "odom_topic": "/simulation/base/pose" if navigation_3d else "/simulation/base_footprint/pose",
                "publish_tf_odom": "false",
                "map_topic": "map",
                "args": rtabmap_args,
                "output": "screen",
            }.items(),
        ),
        Node(
            package="bxi_scan_nav",
            executable="clicked_point_3d_goal",
            name="clicked_point_3d_goal",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_clicked_point_3d_goal")),
            parameters=[
                {
                    "clicked_point_topic": "/clicked_point",
                    "goal_topic": "/move_base_simple/goal",
                    "odom_topic": "/simulation/base_footprint/pose",
                    "tomogram_topic": "/pct_tomogram_points",
                    "marker_topic": "/clicked_3d_goal_marker",
                    "target_frame": "world",
                    "snap_to_tomogram": True,
                    "snap_xy_radius": 0.80,
                    "snap_z_weight": 0.35,
                    "zero_floor_epsilon": 0.06,
                    "reject_on_tf_failure": False,
                }
            ],
            emulate_tty=True,
        ),
        Node(
            package="bxi_scan_nav",
            executable="split_level_global_planner",
            name="split_level_global_planner",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_3d_global_planner")),
            parameters=[
                {
                    "goal_topic": "/move_base_simple/goal",
                    "odom_topic": "/simulation/base_footprint/pose",
                    "path_topic": "/initial_path",
                    "debug_path_topic": "/split_level_global_path",
                    "lower_floor_z": 0.8,
                    "upper_floor_z": 2.0,
                    "upper_floor_min_x": 9.5,
                    "stair_lower_x": 7.85,
                    "stair_upper_x": 9.15,
                    "stair_y": 2.5,
                    "stair_step_count": 12,
                }
            ],
            emulate_tty=True,
        ),
        Node(
            package="bxi_scan_nav",
            executable="elevation_global_planner",
            name="elevation_global_planner",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_elevation_global_planner")),
            parameters=[
                {
                    "cloud_topic": "/rtabmap/cloud_map",
                    "odom_topic": "/simulation/base_footprint/pose",
                    "goal_topic": "/move_base_simple/goal",
                    "path_topic": "/initial_path",
                    "debug_path_topic": "/elevation_global_path",
                    "debug_map_topic": "/elevation_traversability_map",
                    "resolution": 0.20,
                    "robot_radius": 0.36,
                    "path_z_offset": 0.80,
                    "min_z": -0.20,
                    "max_z": 20.00,
                    "min_points_per_cell": 2,
                    "low_percentile": 0.20,
                    "obstacle_height": 0.45,
                    "max_step_height": 0.24,
                    "max_slope": 0.75,
                    "goal_search_radius": 1.0,
                    "start_search_radius": 0.8,
                    "path_sparsify_distance": 0.35,
                    "sample_stride": 1,
                    "max_points": 350000,
                }
            ],
            emulate_tty=True,
        ),
        Node(
            package="bxi_scan_nav",
            executable="pct_global_planner",
            name="pct_global_planner",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_pct_global_planner")),
            parameters=[
                {
                    "pct_root": os.environ.get(
                        "PCT_PLANNER_ROOT", "/home/hwc/code/elf-nav/third_party/PCT_planner"
                    ),
                    "cloud_topic": "/rtabmap/cloud_map",
                    "odom_topic": "/simulation/base/pose" if navigation_3d else "/simulation/base_footprint/pose",
                    "localization_pose_topic": "/rtabmap/localization_pose",
                    "use_localization_pose": True,
                    "goal_topic": "/move_base_simple/goal",
                    "path_topic": "/initial_path",
                    "debug_path_topic": "/pct_global_path",
                    "debug_map_topic": "/pct_traversability_map",
                    "tomogram_topic": "/pct_tomogram_points",
                    "resolution": 0.10,
                    "slice_dh": 0.50,
                    "ground_height": 0.0,
                    "trav_interval_min": 0.50,
                    "trav_interval_free": 0.65,
                    "trav_slope_max": 0.40,
                    "trav_step_max": 0.25,
                    "gateway_height_tolerance": 0.22,
                    "gateway_step_gradient_min": 0.08,
                    "trav_kernel_size": 7,
                    "trav_standable_ratio": 0.20,
                    "trav_cost_barrier": 50.0,
                    "safe_margin": 0.40,
                    "inflation": 0.20,
                    "a_star_cost_threshold": 49.0,
                    "planner_safe_cost_margin": 15.0,
                    "step_cost_weight": 0.20,
                    "max_heading_rate": 10.0,
                    "robot_z_offset": 0.80,
                    "min_z": -0.40,
                    "max_z": 20.00,
                    "goal_zero_epsilon": 0.06,
                    "goal_search_radius": 1.40,
                    "start_search_radius": 2.50,
                    "candidate_count": 24,
                    "max_plan_attempts": 64,
                    "optimize_trajectory": False,
                    "prepend_odom_start": True,
                    "sample_stride": 1,
                    "max_points": 1200000,
                    "max_tomogram_points": 500000,
                }
            ],
            emulate_tty=True,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_global_planner")),
            parameters=[nav2_planner_yaml],
            emulate_tty=True,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_global_planner",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_global_planner")),
            parameters=[
                {
                    "use_sim_time": False,
                    "autostart": True,
                    "node_names": ["planner_server"],
                }
            ],
            emulate_tty=True,
        ),
        Node(
            package="bxi_scan_nav",
            executable="nav2_global_path_planner",
            name="nav2_global_path_planner",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_global_planner")),
            parameters=[
                {
                    "goal_topic": "/move_base_simple/goal",
                    "odom_topic": "/simulation/base_footprint/pose",
                    "path_topic": "/nav2_global_path",
                    "compat_path_topic": "/rtabmap_global_path",
                    "planner_id": "GridBased",
                    "action_name": "compute_path_to_pose",
                    "default_frame_id": "world",
                }
            ],
            emulate_tty=True,
        ),
        Node(
            package="bxi_scan_nav",
            executable="global_path_local_target",
            name="global_path_local_target",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_global_planner")),
            parameters=[
                {
                    "global_path_topic": "/nav2_global_path",
                    "odom_topic": "/simulation/base_footprint/pose",
                    "local_path_topic": "/initial_path",
                    "debug_path_topic": "/scan_planner/local_target_path",
                    "lookahead_distance": 1.0,
                    "goal_tolerance": 0.35,
                    "publish_hz": 4.0,
                    "min_publish_target_shift": 0.12,
                    "costmap_topic": "/global_costmap/costmap",
                    "require_line_of_sight": True,
                    "occupied_threshold": 80,
                }
            ],
            emulate_tty=True,
        ),
        Node(
            package="bxi_scan_nav",
            executable="bspline_path_visualizer",
            name="bspline_path_visualizer",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_scan_planner")),
            parameters=[
                {
                    "bspline_topic": "/planning/bspline",
                    "path_topic": "/planning/bspline_path",
                    "frame_id": "world",
                    "sample_dt": 0.05,
                }
            ],
            emulate_tty=True,
        ),
        Node(
            package="scan_planner",
            executable="scan_planner_node",
            name="scan_planner_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_scan_planner")),
            parameters=[planner_overrides],
            remappings=[
                ("body_pose", "/simulation/base_footprint/pose"),
                ("sensor_pose", "/simulation/d435i/depth/pose"),
                ("depth", "/simulation/d435i/depth/image_raw"),
                ("initial_path", "/initial_path"),
            ],
            emulate_tty=True,
        ),
        Node(
            package="scan_planner",
            executable="closed_loop_controller",
            name="closed_loop_controller",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_scan_planner")),
            parameters=[controllers_yaml, controller_overrides],
            remappings=[
                ("body_pose", "/simulation/base_footprint/pose"),
                ("cmd_vel", "/scan_planner/cmd_vel"),
                ("planning/bspline", "/planning/bspline"),
            ],
            emulate_tty=True,
        ),
        Node(
            package="bxi_scan_nav",
            executable="cmd_vel_to_motion_commands",
            name="cmd_vel_to_motion_commands",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_scan_planner")),
            parameters=[nav_config],
            emulate_tty=True,
        ),
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[
                    ExecuteProcess(
                        cmd=["bash", stop_script],
                        output="screen",
                    )
                ]
            )
        ),
    ]


def generate_launch_description():
    mapping_args = (
        "--Rtabmap/DetectionRate 2 "
        "--RGBD/LinearUpdate 0.05 "
        "--RGBD/AngularUpdate 0.05 "
        "--Reg/Force3DoF false "
        "--RGBD/ForceOdom3DoF false "
        "--Grid/3D true "
        "--Grid/RayTracing true "
        "--RGBD/CreateOccupancyGrid true "
        "--Rtabmap/LoopThr 0.99 "
        "--RGBD/ProximityBySpace false "
        "--RGBD/ProximityByTime false "
        "--RGBD/OptimizeMaxError 1.0 "
        "--Optimizer/Strategy 1 "
        "--Optimizer/Robust false "
        "--Vis/MinInliers 8 "
        "--Kp/MaxFeatures 1000 "
        "--Grid/RangeMax 4.5 "
        "--Grid/MinClusterSize 5"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("start_controller", default_value="false"),
            DeclareLaunchArgument("controller_policy", default_value="depth_origin"),
            DeclareLaunchArgument("start_depth_policy_camera", default_value="true"),
            DeclareLaunchArgument("start_scan_planner", default_value="true"),
            DeclareLaunchArgument("start_rtabmap", default_value="true"),
            DeclareLaunchArgument("start_global_planner", default_value="false"),
            DeclareLaunchArgument("start_3d_global_planner", default_value="false"),
            DeclareLaunchArgument("start_elevation_global_planner", default_value="false"),
            DeclareLaunchArgument("start_pct_global_planner", default_value="true"),
            DeclareLaunchArgument("start_clicked_point_3d_goal", default_value="true"),
            DeclareLaunchArgument("start_keyboard_teleop", default_value="false"),
            DeclareLaunchArgument("start_rviz", default_value="false"),
            DeclareLaunchArgument("navigation_3d", default_value="true"),
            DeclareLaunchArgument("localization", default_value="false"),
            DeclareLaunchArgument("delete_db_on_start", default_value="false"),
            DeclareLaunchArgument("database_path", default_value="/tmp/bxi_elf3_rtabmap.db"),
            DeclareLaunchArgument("rtabmap_args", default_value=mapping_args),
            DeclareLaunchArgument("rtabmap_viz", default_value="false"),
            OpaqueFunction(function=launch_setup),
        ]
    )
