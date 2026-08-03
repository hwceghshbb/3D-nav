import os
from pathlib import Path

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    nav_share = get_package_share_path("bxi_scan_nav")
    elf3_share = get_package_share_path("bxi_example_py_elf3")
    scan_share = get_package_share_path("scan_planner")

    model_file = os.path.join(
        elf3_share, "data", "mujoco_simulation", "elf3_rooms_nav3d.xml"
    )
    state_machine_config = os.path.join(
        elf3_share, "config", "elf3_state_machine.yaml"
    )
    nav_config = os.path.join(nav_share, "config", "navigation.yaml")
    rviz_config = os.path.join(nav_share, "config", "elf3_octo_scan_nav.rviz")
    planner_yaml = os.path.join(scan_share, "config", "planner.yaml")
    controllers_yaml = os.path.join(scan_share, "config", "controllers.yaml")
    workspace_root = Path(nav_share).resolve().parents[2]
    default_map = str(
        workspace_root
        / "maps"
        / "bxi_elf3_rtabmap_20260801_172631_cloud_crop_cloud.ply"
    )

    planner_overrides = {
        "use_sim_time": False,
        "fsm.navi_mode": 3,
        "fsm.navigation_z": 0.8,
        "fsm.use_path_z": True,
        "fsm.use_odom_z": True,
        "fsm.local_path_z_offset": ParameterValue(
            LaunchConfiguration("scan_local_path_z_offset"), value_type=float
        ),
        "fsm.thresh_replan": 1.0,
        "fsm.thresh_no_replan": 0.1,
        "fsm.planning_horizon": ParameterValue(
            LaunchConfiguration("scan_planning_horizon"), value_type=float
        ),
        "fsm.waypoint_pass_through_speed_scale": ParameterValue(
            LaunchConfiguration("scan_waypoint_pass_through_speed_scale"),
            value_type=float,
        ),
        "fsm.min_waypoint_window_size": ParameterValue(
            LaunchConfiguration("scan_min_waypoint_window_size"), value_type=int
        ),
        "fsm.emergency_time": 1.0,
        "fsm.fail_safe": True,
        "fsm.max_replan_fail_count": ParameterValue(
            LaunchConfiguration("scan_max_replan_fail_count"), value_type=int
        ),
        "fsm.exec_interval_ms": ParameterValue(
            LaunchConfiguration("scan_fsm_exec_interval_ms"), value_type=int
        ),
        "fsm.safety_interval_ms": 100,
        "fsm.collision_check_dt": 0.05,
        "fsm.collision_check_horizon": 2.0,
        "fsm.periodic_replan_interval": ParameterValue(
            LaunchConfiguration("scan_replan_interval"), value_type=float
        ),
        "fsm.periodic_replan_min_remaining": ParameterValue(
            LaunchConfiguration("scan_replan_min_remaining"), value_type=float
        ),
        "fsm.enable_completion_driven_replan": ParameterValue(
            LaunchConfiguration("scan_enable_completion_replan"), value_type=bool
        ),
        "fsm.completion_replan_min_interval": ParameterValue(
            LaunchConfiguration("scan_completion_replan_min_interval"),
            value_type=float,
        ),
        "fsm.replan_stitch_lookahead": ParameterValue(
            LaunchConfiguration("scan_replan_stitch_lookahead"), value_type=float
        ),
        "fsm.replan_stitch_max_tracking_error": ParameterValue(
            LaunchConfiguration("scan_replan_stitch_max_tracking_error"),
            value_type=float,
        ),
        "fsm.keep_previous_traj_min_remaining": ParameterValue(
            LaunchConfiguration("scan_keep_previous_traj_min_remaining"),
            value_type=float,
        ),
        "fsm.enable_start_snap": ParameterValue(
            LaunchConfiguration("scan_enable_start_snap"), value_type=bool
        ),
        "fsm.start_snap_radius": ParameterValue(
            LaunchConfiguration("scan_start_snap_radius"), value_type=float
        ),
        "fsm.start_snap_z_radius": ParameterValue(
            LaunchConfiguration("scan_start_snap_z_radius"), value_type=float
        ),
        "fsm.enable_target_snap": ParameterValue(
            LaunchConfiguration("scan_enable_target_snap"), value_type=bool
        ),
        "fsm.target_snap_radius": ParameterValue(
            LaunchConfiguration("scan_target_snap_radius"), value_type=float
        ),
        "fsm.target_snap_z_radius": ParameterValue(
            LaunchConfiguration("scan_target_snap_z_radius"), value_type=float
        ),
        "fsm.enable_global_replan_on_local_failure": ParameterValue(
            LaunchConfiguration("scan_enable_global_replan_on_local_failure"),
            value_type=bool,
        ),
        "fsm.global_replan_request_cooldown": ParameterValue(
            LaunchConfiguration("scan_global_replan_request_cooldown"),
            value_type=float,
        ),
        "fsm.global_replan_goal_z_offset": ParameterValue(
            LaunchConfiguration("scan_global_replan_goal_z_offset"),
            value_type=float,
        ),
        "fsm.global_replan_goal_topic": "/move_base_simple/goal",
        "grid_map.resolution": ParameterValue(
            LaunchConfiguration("scan_grid_resolution"), value_type=float
        ),
        "grid_map.sliding_map_size_x": ParameterValue(
            LaunchConfiguration("scan_sliding_map_xy"), value_type=float
        ),
        "grid_map.sliding_map_size_y": ParameterValue(
            LaunchConfiguration("scan_sliding_map_xy"), value_type=float
        ),
        "grid_map.sliding_map_size_z": 3.2,
        "grid_map.local_update_range_x": ParameterValue(
            LaunchConfiguration("scan_local_update_range_xy"), value_type=float
        ),
        "grid_map.local_update_range_y": ParameterValue(
            LaunchConfiguration("scan_local_update_range_xy"), value_type=float
        ),
        "grid_map.local_update_range_z": 1.8,
        "grid_map.map_sliding_en": True,
        "grid_map.map_sliding_thresh": 0.2,
        "grid_map.sensor_type": "depth",
        "grid_map.cloud_is_world": False,
        "grid_map.need_extrinsic": False,
        "grid_map.cx": ParameterValue(LaunchConfiguration("camera_cx"), value_type=float),
        "grid_map.cy": ParameterValue(LaunchConfiguration("camera_cy"), value_type=float),
        "grid_map.fx": ParameterValue(LaunchConfiguration("camera_fx"), value_type=float),
        "grid_map.fy": ParameterValue(LaunchConfiguration("camera_fy"), value_type=float),
        "grid_map.depth_filter_maxdist": 4.5,
        "grid_map.depth_filter_mindist": 0.25,
        "grid_map.skip_pixel": ParameterValue(
            LaunchConfiguration("scan_skip_pixel"), value_type=int
        ),
        "grid_map.body_height": ParameterValue(
            LaunchConfiguration("scan_body_height"), value_type=float
        ),
        "grid_map.double_cylinder_radius": ParameterValue(
            LaunchConfiguration("scan_body_radius"), value_type=float
        ),
        "grid_map.double_cylinder_offset": ParameterValue(
            LaunchConfiguration("scan_body_offset"), value_type=float
        ),
        "grid_map.obstacles_inflation_z_up": ParameterValue(
            LaunchConfiguration("scan_inflation_z_up"), value_type=float
        ),
        "grid_map.obstacles_inflation_z_down": ParameterValue(
            LaunchConfiguration("scan_inflation_z_down"), value_type=float
        ),
        "grid_map.frame_id": "world",
        "grid_map.sliding_map_frame_id": "sliding_map",
        "grid_map.ground_height": 0.0,
        "grid_map.ground_filter_height": 0.25,
        "optimization.dist0": ParameterValue(
            LaunchConfiguration("scan_collision_clearance"), value_type=float
        ),
        "manager.max_vel": ParameterValue(
            LaunchConfiguration("scan_max_vel"), value_type=float
        ),
        "manager.max_acc": 0.35,
        "manager.max_jerk": 4.0,
        "manager.control_points_distance": ParameterValue(
            LaunchConfiguration("scan_control_points_distance"), value_type=float
        ),
        "manager.planning_horizon": ParameterValue(
            LaunchConfiguration("scan_planning_horizon"), value_type=float
        ),
        "optimization.lambda_smooth": 1.0,
        "optimization.lambda_collision": 1.0,
        "optimization.lambda_feasibility": 0.1,
        "optimization.lambda_fitness": ParameterValue(
            LaunchConfiguration("scan_path_follow_weight"), value_type=float
        ),
        "optimization.max_vel": ParameterValue(
            LaunchConfiguration("scan_max_vel"), value_type=float
        ),
        "optimization.max_acc": 0.35,
        "optimization.order": 3,
    }
    controller_overrides = {
        "max_vx": ParameterValue(
            LaunchConfiguration("scan_max_vel"), value_type=float
        ),
        "max_vy": ParameterValue(
            LaunchConfiguration("scan_max_vy"), value_type=float
        ),
        "max_vyaw": ParameterValue(
            LaunchConfiguration("scan_max_vyaw"), value_type=float
        ),
        "heading_error_threshold": ParameterValue(
            LaunchConfiguration("scan_heading_error_threshold"), value_type=float
        ),
        "finish_dist": 0.25,
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_simulation", default_value="true"),
            DeclareLaunchArgument("start_controller", default_value="true"),
            DeclareLaunchArgument("start_scan_planner", default_value="true"),
            DeclareLaunchArgument("start_octo_global_planner", default_value="true"),
            DeclareLaunchArgument("start_clicked_point_3d_goal", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("model_file", default_value=model_file),
            DeclareLaunchArgument("state_machine_config", default_value=state_machine_config),
            DeclareLaunchArgument("nav_camera_name", default_value="head_depth_camera"),
            DeclareLaunchArgument(
                "nav_camera_output_prefix",
                default_value="/simulation/head_depth_camera/",
            ),
            DeclareLaunchArgument(
                "nav_camera_frame_id",
                default_value="head_depth_camera_depth_optical_frame",
            ),
            DeclareLaunchArgument(
                "nav_depth_topic",
                default_value="/simulation/head_depth_camera/depth/image_rect_raw",
            ),
            DeclareLaunchArgument(
                "nav_camera_pose_topic",
                default_value="/simulation/head_depth_camera/depth/pose",
            ),
            DeclareLaunchArgument("start_body_depth_camera", default_value="true"),
            DeclareLaunchArgument("body_camera_width", default_value="48"),
            DeclareLaunchArgument("body_camera_height", default_value="36"),
            DeclareLaunchArgument("camera_width", default_value="640"),
            DeclareLaunchArgument("camera_height", default_value="480"),
            DeclareLaunchArgument("camera_fx", default_value="261.9140402566251"),
            DeclareLaunchArgument("camera_fy", default_value="261.9140402566251"),
            DeclareLaunchArgument("camera_cx", default_value="320.0"),
            DeclareLaunchArgument("camera_cy", default_value="240.0"),
            DeclareLaunchArgument(
                "input_pcd",
                default_value=default_map,
            ),
            DeclareLaunchArgument("octo_start_z_offset", default_value="0.30"),
            DeclareLaunchArgument("octo_goal_z_offset", default_value="0.90"),
            DeclareLaunchArgument("octo_path_z_offset", default_value="0.0"),
            DeclareLaunchArgument("octo_enforce_path_ground_clearance", default_value="true"),
            DeclareLaunchArgument("octo_path_min_ground_clearance", default_value="0.75"),
            DeclareLaunchArgument("octo_path_ground_search_depth", default_value="1.50"),
            DeclareLaunchArgument("octo_path_ground_xy_radius_cells", default_value="1"),
            DeclareLaunchArgument("octo_cloud_scale", default_value="1.0"),
            DeclareLaunchArgument("octo_resolution", default_value="0.20"),
            DeclareLaunchArgument("octo_robot_radius", default_value="0.18"),
            DeclareLaunchArgument("octo_max_iterations", default_value="1000000"),
            DeclareLaunchArgument("octo_require_ground_support", default_value="true"),
            DeclareLaunchArgument("octo_ground_support_xy_radius_cells", default_value="2"),
            DeclareLaunchArgument("octo_ground_support_depth_cells", default_value="3"),
            DeclareLaunchArgument("octo_enable_preblocked_costmap", default_value="false"),
            DeclareLaunchArgument("octo_enable_clearance_cost", default_value="true"),
            DeclareLaunchArgument("octo_clearance_cost_radius_cells", default_value="4"),
            DeclareLaunchArgument("octo_clearance_cost_weight", default_value="0.8"),
            DeclareLaunchArgument("octo_vertical_padding_below", default_value="1.0"),
            DeclareLaunchArgument("octo_vertical_padding_above", default_value="0.6"),
            DeclareLaunchArgument("scan_body_height", default_value="1.2"),
            DeclareLaunchArgument("scan_body_radius", default_value="0.22"),
            DeclareLaunchArgument("scan_body_offset", default_value="0.10"),
            DeclareLaunchArgument("scan_inflation_z_up", default_value="0.05"),
            DeclareLaunchArgument("scan_inflation_z_down", default_value="0.0"),
            DeclareLaunchArgument("scan_collision_clearance", default_value="0.22"),
            DeclareLaunchArgument("scan_planning_horizon", default_value="2.0"),
            DeclareLaunchArgument("scan_min_waypoint_window_size", default_value="4"),
            DeclareLaunchArgument("scan_waypoint_pass_through_speed_scale", default_value="0.7"),
            DeclareLaunchArgument("scan_path_follow_weight", default_value="4.0"),
            DeclareLaunchArgument("scan_grid_resolution", default_value="0.08"),
            DeclareLaunchArgument("scan_control_points_distance", default_value="0.28"),
            DeclareLaunchArgument("scan_skip_pixel", default_value="4"),
            DeclareLaunchArgument("scan_sliding_map_xy", default_value="6.0"),
            DeclareLaunchArgument("scan_local_update_range_xy", default_value="3.0"),
            DeclareLaunchArgument("scan_local_path_z_offset", default_value="0.20"),
            DeclareLaunchArgument("scan_max_replan_fail_count", default_value="8"),
            DeclareLaunchArgument("scan_fsm_exec_interval_ms", default_value="10"),
            DeclareLaunchArgument("scan_replan_interval", default_value="0.35"),
            DeclareLaunchArgument("scan_replan_min_remaining", default_value="0.20"),
            DeclareLaunchArgument("scan_enable_completion_replan", default_value="true"),
            DeclareLaunchArgument(
                "scan_completion_replan_min_interval", default_value="0.05"
            ),
            DeclareLaunchArgument("scan_replan_stitch_lookahead", default_value="0.0"),
            DeclareLaunchArgument(
                "scan_replan_stitch_max_tracking_error", default_value="0.20"
            ),
            DeclareLaunchArgument(
                "scan_keep_previous_traj_min_remaining", default_value="0.50"
            ),
            DeclareLaunchArgument("scan_enable_start_snap", default_value="true"),
            DeclareLaunchArgument("scan_start_snap_radius", default_value="0.35"),
            DeclareLaunchArgument("scan_start_snap_z_radius", default_value="0.25"),
            DeclareLaunchArgument("scan_enable_target_snap", default_value="true"),
            DeclareLaunchArgument("scan_target_snap_radius", default_value="0.45"),
            DeclareLaunchArgument("scan_target_snap_z_radius", default_value="0.35"),
            DeclareLaunchArgument(
                "scan_enable_global_replan_on_local_failure",
                default_value="true",
            ),
            DeclareLaunchArgument("scan_global_replan_request_cooldown", default_value="1.0"),
            DeclareLaunchArgument("scan_global_replan_goal_z_offset", default_value="0.90"),
            DeclareLaunchArgument("scan_max_vel", default_value="0.40"),
            DeclareLaunchArgument("scan_max_vy", default_value="0.18"),
            DeclareLaunchArgument("scan_max_vyaw", default_value="0.55"),
            DeclareLaunchArgument("scan_heading_error_threshold", default_value="0.8"),
            Node(
                package="mujoco",
                executable="simulation",
                name="simulation_mujoco",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_simulation")),
                parameters=[{"simulation/model_file": LaunchConfiguration("model_file")}],
                emulate_tty=True,
            ),
            Node(
                package="bxi_example_py_elf3",
                executable="bxi_example_py_elf3_demo",
                name="bxi_example_py_elf3_demo",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_controller")),
                parameters=[
                    {"/topic_prefix": "simulation/"},
                    {"/state_machine_config": LaunchConfiguration("state_machine_config")},
                ],
                emulate_tty=True,
            ),
            Node(
                package="bxi_scan_nav",
                executable="rgbd_camera_publisher",
                name="nav_rgbd_camera_publisher",
                output="screen",
                parameters=[
                    {
                        "model_file": LaunchConfiguration("model_file"),
                        "camera_name": LaunchConfiguration("nav_camera_name"),
                        "output_prefix": LaunchConfiguration("nav_camera_output_prefix"),
                        "frame_id": LaunchConfiguration("nav_camera_frame_id"),
                        "hidden_body_names": ["torso_link"],
                        "width": ParameterValue(
                            LaunchConfiguration("camera_width"), value_type=int
                        ),
                        "height": ParameterValue(
                            LaunchConfiguration("camera_height"), value_type=int
                        ),
                        "publish_hz": 15.0,
                        "publish_color": False,
                        "publish_raw_depth": False,
                    }
                ],
                emulate_tty=True,
            ),
            Node(
                package="bxi_scan_nav",
                executable="rgbd_camera_publisher",
                name="body_depth_camera_publisher",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_body_depth_camera")),
                parameters=[
                    {
                        "model_file": LaunchConfiguration("model_file"),
                        "camera_name": "body_depth_camera",
                        "output_prefix": "/simulation/body_depth_camera/",
                        "frame_id": "body_depth_camera_depth_optical_frame",
                        "hidden_body_names": ["torso_link"],
                        "width": ParameterValue(
                            LaunchConfiguration("body_camera_width"), value_type=int
                        ),
                        "height": ParameterValue(
                            LaunchConfiguration("body_camera_height"), value_type=int
                        ),
                        "publish_hz": 20.0,
                        "publish_color": False,
                        "publish_raw_depth": False,
                    }
                ],
                emulate_tty=True,
            ),
            Node(
                package="bxi_scan_nav",
                executable="d435i_pose_publisher",
                name="nav_camera_pose_publisher",
                output="screen",
                parameters=[
                    nav_config,
                    {
                        "model_file": LaunchConfiguration("model_file"),
                        "camera_name": LaunchConfiguration("nav_camera_name"),
                        "preserve_base_footprint_z": True,
                        "base_footprint_z_offset": -0.3,
                        "sensor_pose_topic": LaunchConfiguration("nav_camera_pose_topic"),
                        "sensor_frame_id": LaunchConfiguration("nav_camera_frame_id"),
                    },
                ],
                emulate_tty=True,
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
                        "target_frame": "world",
                        "snap_to_tomogram": False,
                        "zero_floor_epsilon": 0.06,
                        "reject_on_tf_failure": False,
                    }
                ],
                emulate_tty=True,
            ),
            Node(
                package="bxi_octo_global_planner",
                executable="octo_global_planner_node",
                name="bxi_octo_global_planner",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_octo_global_planner")),
                parameters=[
                    {
                        "frame_id": "world",
                        "input_pcd": LaunchConfiguration("input_pcd"),
                        "output_bt": "/tmp/bxi_octo_global_map.bt",
                        "cloud_scale": ParameterValue(
                            LaunchConfiguration("octo_cloud_scale"), value_type=float
                        ),
                        "octomap_resolution": ParameterValue(
                            LaunchConfiguration("octo_resolution"), value_type=float
                        ),
                        "robot_radius": ParameterValue(
                            LaunchConfiguration("octo_robot_radius"), value_type=float
                        ),
                        "max_iterations": ParameterValue(
                            LaunchConfiguration("octo_max_iterations"), value_type=int
                        ),
                        "require_ground_support": ParameterValue(
                            LaunchConfiguration("octo_require_ground_support"),
                            value_type=bool,
                        ),
                        "ground_support_xy_radius_cells": ParameterValue(
                            LaunchConfiguration("octo_ground_support_xy_radius_cells"),
                            value_type=int,
                        ),
                        "ground_support_depth_cells": ParameterValue(
                            LaunchConfiguration("octo_ground_support_depth_cells"),
                            value_type=int,
                        ),
                        "enable_preblocked_costmap": ParameterValue(
                            LaunchConfiguration("octo_enable_preblocked_costmap"),
                            value_type=bool,
                        ),
                        "vertical_search_padding_below": ParameterValue(
                            LaunchConfiguration("octo_vertical_padding_below"),
                            value_type=float,
                        ),
                        "vertical_search_padding_above": ParameterValue(
                            LaunchConfiguration("octo_vertical_padding_above"),
                            value_type=float,
                        ),
                        "snap_search_radius_cells": 20,
                        "preblocked_costmap_weight": 1.0,
                        "enable_clearance_cost": ParameterValue(
                            LaunchConfiguration("octo_enable_clearance_cost"),
                            value_type=bool,
                        ),
                        "clearance_cost_radius_cells": ParameterValue(
                            LaunchConfiguration("octo_clearance_cost_radius_cells"),
                            value_type=int,
                        ),
                        "clearance_cost_weight": ParameterValue(
                            LaunchConfiguration("octo_clearance_cost_weight"),
                            value_type=float,
                        ),
                        "odom_topic": "/simulation/base_footprint/pose",
                        "goal_topic": "/move_base_simple/goal",
                        "path_topic": "/initial_path",
                        "debug_path_topic": "/octo_global_path",
                        "map_marker_topic": "/octo_occupied_map",
                        "start_z_offset": ParameterValue(
                            LaunchConfiguration("octo_start_z_offset"),
                            value_type=float,
                        ),
                        "goal_z_offset": ParameterValue(
                            LaunchConfiguration("octo_goal_z_offset"), value_type=float
                        ),
                        "path_z_offset": ParameterValue(
                            LaunchConfiguration("octo_path_z_offset"), value_type=float
                        ),
                        "enforce_path_ground_clearance": ParameterValue(
                            LaunchConfiguration("octo_enforce_path_ground_clearance"),
                            value_type=bool,
                        ),
                        "path_min_ground_clearance": ParameterValue(
                            LaunchConfiguration("octo_path_min_ground_clearance"),
                            value_type=float,
                        ),
                        "path_ground_search_depth": ParameterValue(
                            LaunchConfiguration("octo_path_ground_search_depth"),
                            value_type=float,
                        ),
                        "path_ground_xy_radius_cells": ParameterValue(
                            LaunchConfiguration("octo_path_ground_xy_radius_cells"),
                            value_type=int,
                        ),
                        "publish_map": True,
                        "map_publish_period": 2.0,
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
            Node(
                package="scan_planner",
                executable="scan_planner_node",
                name="scan_planner_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_scan_planner")),
                parameters=[planner_yaml, planner_overrides],
                remappings=[
                    ("body_pose", "/simulation/base_footprint/pose"),
                    ("sensor_pose", LaunchConfiguration("nav_camera_pose_topic")),
                    ("depth", LaunchConfiguration("nav_depth_topic")),
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
                package="bxi_scan_nav",
                executable="cmd_vel_to_motion_commands",
                name="cmd_vel_to_motion_commands",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_scan_planner")),
                parameters=[
                    nav_config,
                    {
                        "max_vx": ParameterValue(
                            LaunchConfiguration("scan_max_vel"), value_type=float
                        ),
                        "max_vy": ParameterValue(
                            LaunchConfiguration("scan_max_vy"), value_type=float
                        ),
                        "max_yawdot": ParameterValue(
                            LaunchConfiguration("scan_max_vyaw"), value_type=float
                        ),
                    },
                ],
                emulate_tty=True,
            ),
        ]
    )
