import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_path("bxi_scan_nav")
    elf3_share = get_package_share_path("bxi_example_py_elf3")
    scan_share = get_package_share_path("scan_planner")

    model = os.path.join(elf3_share, "data", "mujoco_simulation", "elf3_rooms_bxsim_nav.xml")
    onnx_file = os.path.join(elf3_share, "data", "mjlab_model", "model_normal.onnx")
    depth_policy_file = "/home/hwc/code/elf-nav/bx_sim/policy/lyp2/dagger1.onnx"
    nav_config = os.path.join(nav_share, "config", "navigation.yaml")
    rviz_config = os.path.join(nav_share, "config", "elf3_octo_scan_nav.rviz")
    planner_yaml = os.path.join(scan_share, "config", "planner.yaml")
    controllers_yaml = os.path.join(scan_share, "config", "controllers.yaml")

    planner_overrides = {
        "use_sim_time": False,
        "fsm.navi_mode": 3,
        "fsm.navigation_z": 0.8,
        "fsm.use_path_z": True,
        "fsm.use_odom_z": True,
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
        "grid_map.cx": 320.0,
        "grid_map.cy": 240.0,
        "grid_map.fx": 261.9140402566251,
        "grid_map.fy": 261.9140402566251,
        "grid_map.depth_filter_maxdist": 4.5,
        "grid_map.depth_filter_mindist": 0.25,
        "grid_map.skip_pixel": 3,
        "grid_map.body_height": ParameterValue(
            LaunchConfiguration("scan_body_height"), value_type=float
        ),
        "grid_map.double_cylinder_radius": ParameterValue(
            LaunchConfiguration("scan_body_radius"), value_type=float
        ),
        "grid_map.double_cylinder_offset": ParameterValue(
            LaunchConfiguration("scan_body_offset"), value_type=float
        ),
        "grid_map.obstacles_inflation_z_up": 0.15,
        "grid_map.obstacles_inflation_z_down": 0.10,
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
        "manager.control_points_distance": 0.2,
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
        "max_vyaw": ParameterValue(
            LaunchConfiguration("scan_max_vyaw"), value_type=float
        ),
        "finish_dist": 0.25,
    }

    return LaunchDescription([
        DeclareLaunchArgument("start_controller", default_value="false"),
        DeclareLaunchArgument("start_scan_planner", default_value="true"),
        DeclareLaunchArgument("start_octo_global_planner", default_value="true"),
        DeclareLaunchArgument("start_clicked_point_3d_goal", default_value="true"),
        DeclareLaunchArgument("start_depth_policy_camera", default_value="true"),
        DeclareLaunchArgument("start_rviz", default_value="false"),
        DeclareLaunchArgument("model_file", default_value=model),
        DeclareLaunchArgument("controller_policy", default_value="depth_origin"),
        DeclareLaunchArgument("controller_onnx_file", default_value=onnx_file),
        DeclareLaunchArgument("depth_policy_file", default_value=depth_policy_file),
        DeclareLaunchArgument(
            "depth_image_topic",
            default_value="/simulation/origin_depth/depth/image_raw",
        ),
        DeclareLaunchArgument(
            "input_pcd",
            default_value="/home/hwc/code/elf-nav/third_party/OctoPlanner3D-ROS2/octomap/pcd_files/building2_9.pcd",
        ),
        DeclareLaunchArgument("octo_start_z_offset", default_value="0.30"),
        DeclareLaunchArgument("octo_goal_z_offset", default_value="0.90"),
        DeclareLaunchArgument("octo_path_z_offset", default_value="0.80"),
        DeclareLaunchArgument("octo_cloud_scale", default_value="1.0"),
        DeclareLaunchArgument("octo_resolution", default_value="0.20"),
        DeclareLaunchArgument("octo_robot_radius", default_value="0.05"),
        DeclareLaunchArgument("octo_require_ground_support", default_value="true"),
        DeclareLaunchArgument("octo_ground_support_xy_radius_cells", default_value="2"),
        DeclareLaunchArgument("octo_ground_support_depth_cells", default_value="3"),
        DeclareLaunchArgument("octo_enable_preblocked_costmap", default_value="false"),
        DeclareLaunchArgument("octo_vertical_padding_below", default_value="1.0"),
        DeclareLaunchArgument("octo_vertical_padding_above", default_value="0.6"),
        DeclareLaunchArgument("scan_body_height", default_value="1.2"),
        DeclareLaunchArgument("scan_body_radius", default_value="0.22"),
        DeclareLaunchArgument("scan_body_offset", default_value="0.10"),
        DeclareLaunchArgument("scan_collision_clearance", default_value="0.22"),
        DeclareLaunchArgument("scan_max_vel", default_value="0.25"),
        DeclareLaunchArgument("scan_max_vyaw", default_value="0.35"),
        Node(
            package="mujoco",
            executable="simulation",
            name="simulation_mujoco",
            output="screen",
            parameters=[{"simulation/model_file": LaunchConfiguration("model_file")}],
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
                {"/onnx_file": LaunchConfiguration("controller_onnx_file")},
                {"/policy_type": LaunchConfiguration("controller_policy")},
                {"/depth_policy_file": LaunchConfiguration("depth_policy_file")},
                {"/depth_image_topic": LaunchConfiguration("depth_image_topic")},
            ],
            emulate_tty=True,
        ),
        Node(
            package="bxi_scan_nav",
            executable="rgbd_camera_publisher",
            name="d435i_rgbd",
            output="screen",
            parameters=[{"model_file": LaunchConfiguration("model_file")}],
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
                    "model_file": LaunchConfiguration("model_file"),
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
                    "preserve_base_footprint_z": True,
                    "base_footprint_z_offset": -0.3,
                    "offset_x": 0.10,
                    "offset_z": 0.20,
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
                    "require_ground_support": ParameterValue(
                        LaunchConfiguration("octo_require_ground_support"), value_type=bool
                    ),
                    "ground_support_xy_radius_cells": ParameterValue(
                        LaunchConfiguration("octo_ground_support_xy_radius_cells"), value_type=int
                    ),
                    "ground_support_depth_cells": ParameterValue(
                        LaunchConfiguration("octo_ground_support_depth_cells"), value_type=int
                    ),
                    "enable_preblocked_costmap": ParameterValue(
                        LaunchConfiguration("octo_enable_preblocked_costmap"), value_type=bool
                    ),
                    "vertical_search_padding_below": ParameterValue(
                        LaunchConfiguration("octo_vertical_padding_below"), value_type=float
                    ),
                    "vertical_search_padding_above": ParameterValue(
                        LaunchConfiguration("octo_vertical_padding_above"), value_type=float
                    ),
                    "snap_search_radius_cells": 20,
                    "preblocked_costmap_weight": 1.0,
                    "odom_topic": "/simulation/base_footprint/pose",
                    "goal_topic": "/move_base_simple/goal",
                    "path_topic": "/initial_path",
                    "debug_path_topic": "/octo_global_path",
                    "map_marker_topic": "/octo_occupied_map",
                    "start_z_offset": ParameterValue(
                        LaunchConfiguration("octo_start_z_offset"), value_type=float
                    ),
                    "goal_z_offset": ParameterValue(
                        LaunchConfiguration("octo_goal_z_offset"), value_type=float
                    ),
                    "path_z_offset": ParameterValue(
                        LaunchConfiguration("octo_path_z_offset"), value_type=float
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
            parameters=[nav_config],
            emulate_tty=True,
        ),
    ])
