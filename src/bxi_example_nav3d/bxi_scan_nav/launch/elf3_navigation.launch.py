"""Standalone ELF3 navigation using only ROS topic and TF contracts."""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def float_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    nav_share = get_package_share_path("bxi_scan_nav")
    scan_share = get_package_share_path("scan_planner")

    mapping_launch = nav_share / "launch" / "elf3_rtab_mapping.launch.py"
    planner_config = scan_share / "config" / "planner.yaml"
    controller_config = scan_share / "config" / "controllers.yaml"
    rviz_config = nav_share / "config" / "elf3_rtab_mapping.rviz"

    planner_overrides = {
        "use_sim_time": ParameterValue(
            LaunchConfiguration("use_sim_time"), value_type=bool
        ),
        "fsm.navi_mode": 3,
        "fsm.navigation_z": float_param("navigation_z"),
        "fsm.use_path_z": True,
        "fsm.use_odom_z": True,
        "fsm.planning_horizon": float_param("planning_horizon"),
        "fsm.fail_safe": True,
        "fsm.max_replan_fail_count": 8,
        "fsm.exec_interval_ms": 20,
        "fsm.safety_interval_ms": 100,
        "fsm.collision_check_dt": 0.05,
        "fsm.collision_check_horizon": 2.0,
        "fsm.global_replan_goal_topic": LaunchConfiguration("goal_topic"),
        "grid_map.sensor_type": "depth",
        "grid_map.cloud_is_world": False,
        "grid_map.need_extrinsic": False,
        "grid_map.frame_id": LaunchConfiguration("map_frame_id"),
        "grid_map.sliding_map_frame_id": "sliding_map",
        "grid_map.resolution": float_param("local_map_resolution"),
        "grid_map.sliding_map_size_x": float_param("local_map_size_xy"),
        "grid_map.sliding_map_size_y": float_param("local_map_size_xy"),
        "grid_map.sliding_map_size_z": float_param("local_map_size_z"),
        "grid_map.local_update_range_x": float_param("local_update_range_xy"),
        "grid_map.local_update_range_y": float_param("local_update_range_xy"),
        "grid_map.local_update_range_z": float_param("local_update_range_z"),
        "grid_map.map_sliding_en": True,
        "grid_map.map_sliding_thresh": 0.2,
        "grid_map.cx": float_param("camera_cx"),
        "grid_map.cy": float_param("camera_cy"),
        "grid_map.fx": float_param("camera_fx"),
        "grid_map.fy": float_param("camera_fy"),
        "grid_map.depth_filter_maxdist": float_param("depth_max_distance"),
        "grid_map.depth_filter_mindist": float_param("depth_min_distance"),
        "grid_map.depth_filter_margin": 1,
        "grid_map.k_depth_scaling_factor": float_param("depth_scale"),
        "grid_map.skip_pixel": ParameterValue(
            LaunchConfiguration("depth_skip_pixel"), value_type=int
        ),
        "grid_map.body_height": float_param("body_height"),
        "grid_map.double_cylinder_radius": float_param("robot_radius"),
        "grid_map.double_cylinder_offset": 0.10,
        "grid_map.obstacles_inflation_z_up": 0.10,
        "grid_map.obstacles_inflation_z_down": 0.05,
        "grid_map.ground_height": 0.0,
        "grid_map.ground_filter_height": 0.25,
        "grid_map.max_ray_length": float_param("depth_max_distance"),
        "manager.max_vel": float_param("max_vx"),
        "manager.max_acc": 0.35,
        "manager.max_jerk": 4.0,
        "manager.control_points_distance": 0.25,
        "manager.planning_horizon": float_param("planning_horizon"),
        "optimization.dist0": float_param("collision_clearance"),
        "optimization.max_vel": float_param("max_vx"),
        "optimization.max_acc": 0.35,
    }

    controller_overrides = {
        "use_sim_time": ParameterValue(
            LaunchConfiguration("use_sim_time"), value_type=bool
        ),
        "max_vx": float_param("max_vx"),
        "max_vy": float_param("max_vy"),
        "max_vyaw": float_param("max_yaw_rate"),
        "finish_dist": float_param("goal_tolerance"),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rgb_topic", default_value="/camera/color/image_raw"
            ),
            DeclareLaunchArgument(
                "depth_topic", default_value="/camera/depth/image_rect_raw"
            ),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/camera/depth/camera_info"
            ),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument(
                "sensor_pose_topic", default_value="/camera/depth/pose"
            ),
            DeclareLaunchArgument("goal_topic", default_value="/goal_pose"),
            DeclareLaunchArgument(
                "cmd_vel_topic", default_value="/navigation/cmd_vel"
            ),
            DeclareLaunchArgument("base_frame_id", default_value="base_link"),
            DeclareLaunchArgument("map_frame_id", default_value="map"),
            DeclareLaunchArgument("publish_tf_map", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_mapping", default_value="true"),
            DeclareLaunchArgument("localization", default_value="false"),
            DeclareLaunchArgument("delete_db_on_start", default_value="false"),
            DeclareLaunchArgument(
                "database_path", default_value="/tmp/bxi_elf3_rtabmap.db"
            ),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("navigation_z", default_value="0.8"),
            DeclareLaunchArgument("planning_horizon", default_value="3.0"),
            DeclareLaunchArgument("local_map_resolution", default_value="0.08"),
            DeclareLaunchArgument("local_map_size_xy", default_value="6.0"),
            DeclareLaunchArgument("local_map_size_z", default_value="3.2"),
            DeclareLaunchArgument("local_update_range_xy", default_value="3.0"),
            DeclareLaunchArgument("local_update_range_z", default_value="1.8"),
            DeclareLaunchArgument("camera_cx", default_value="212.0"),
            DeclareLaunchArgument("camera_cy", default_value="120.0"),
            DeclareLaunchArgument("camera_fx", default_value="130.957"),
            DeclareLaunchArgument("camera_fy", default_value="130.957"),
            DeclareLaunchArgument("depth_min_distance", default_value="0.25"),
            DeclareLaunchArgument("depth_max_distance", default_value="4.5"),
            DeclareLaunchArgument("depth_scale", default_value="1000.0"),
            DeclareLaunchArgument("depth_skip_pixel", default_value="4"),
            DeclareLaunchArgument("body_height", default_value="1.2"),
            DeclareLaunchArgument("robot_radius", default_value="0.22"),
            DeclareLaunchArgument("collision_clearance", default_value="0.30"),
            DeclareLaunchArgument("max_vx", default_value="0.35"),
            DeclareLaunchArgument("max_vy", default_value="0.15"),
            DeclareLaunchArgument("max_yaw_rate", default_value="0.55"),
            DeclareLaunchArgument("goal_tolerance", default_value="0.25"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(mapping_launch)),
                condition=IfCondition(LaunchConfiguration("start_mapping")),
                launch_arguments={
                    "rgb_topic": LaunchConfiguration("rgb_topic"),
                    "depth_topic": LaunchConfiguration("depth_topic"),
                    "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                    "odom_topic": LaunchConfiguration("odom_topic"),
                    "base_frame_id": LaunchConfiguration("base_frame_id"),
                    "map_frame_id": LaunchConfiguration("map_frame_id"),
                    "publish_tf_map": LaunchConfiguration("publish_tf_map"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "localization": LaunchConfiguration("localization"),
                    "delete_db_on_start": LaunchConfiguration(
                        "delete_db_on_start"
                    ),
                    "database_path": LaunchConfiguration("database_path"),
                    "start_rviz": "false",
                }.items(),
            ),
            Node(
                package="bxi_scan_nav",
                executable="rtabmap_grid_astar_planner",
                name="global_path_planner",
                output="screen",
                parameters=[
                    {
                        "map_topic": "/rtabmap/map",
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "goal_topic": LaunchConfiguration("goal_topic"),
                        "scan_initial_path_topic": "/navigation/initial_path",
                        "debug_path_topic": "/navigation/global_path",
                        "robot_radius": float_param("robot_radius"),
                        "path_z": float_param("navigation_z"),
                    }
                ],
            ),
            Node(
                package="scan_planner",
                executable="scan_planner_node",
                name="scan_planner_node",
                output="screen",
                parameters=[planner_config, planner_overrides],
                remappings=[
                    ("body_pose", LaunchConfiguration("odom_topic")),
                    ("sensor_pose", LaunchConfiguration("sensor_pose_topic")),
                    ("depth", LaunchConfiguration("depth_topic")),
                    ("initial_path", "/navigation/initial_path"),
                ],
            ),
            Node(
                package="scan_planner",
                executable="closed_loop_controller",
                name="closed_loop_controller",
                output="screen",
                parameters=[controller_config, controller_overrides],
                remappings=[
                    ("body_pose", LaunchConfiguration("odom_topic")),
                    ("planning/bspline", "/planning/bspline"),
                    ("cmd_vel", LaunchConfiguration("cmd_vel_topic")),
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="navigation_rviz",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_rviz")),
                arguments=["-d", str(rviz_config)],
            ),
        ]
    )
