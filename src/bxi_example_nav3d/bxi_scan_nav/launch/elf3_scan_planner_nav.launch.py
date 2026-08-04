import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_path("bxi_scan_nav")
    elf3_share = get_package_share_path("bxi_example_py_elf3")
    scan_share = get_package_share_path("scan_planner")
    model = os.path.join(elf3_share, "data", "mujoco_simulation", "elf3_rooms_datagen.xml")
    onnx_file = os.path.join(elf3_share, "data", "mjlab_model", "model_normal.onnx")
    nav_config = os.path.join(nav_share, "config", "navigation.yaml")
    planner_yaml = os.path.join(scan_share, "config", "planner.yaml")
    controllers_yaml = os.path.join(scan_share, "config", "controllers.yaml")
    planner_overrides = {
        "use_sim_time": False,
        "fsm.navi_mode": 1,
        "fsm.planning_horizon": 3.5,
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
        "grid_map.body_height": 0.8,
        "grid_map.double_cylinder_radius": 0.35,
        "manager.max_vel": 0.35,
        "manager.max_acc": 0.35,
        "optimization.max_vel": 0.35,
        "optimization.max_acc": 0.35,
    }
    controller_overrides = {
        "max_vx": 0.35,
        "max_vy": 0.15,
        "max_vyaw": 0.55,
        "finish_dist": 0.25,
    }
    return LaunchDescription([
        DeclareLaunchArgument("start_controller", default_value="true"),
        DeclareLaunchArgument("start_scan_planner", default_value="true"),
        Node(
            package="mujoco", executable="simulation", name="simulation_mujoco",
            output="screen", parameters=[{"simulation/model_file": model}], emulate_tty=True,
        ),
        Node(
            package="bxi_example_py_elf3", executable="bxi_example_py_elf3_mjlab",
            name="elf3_policy", output="screen",
            condition=IfCondition(LaunchConfiguration("start_controller")),
            parameters=[{"/topic_prefix": "simulation/"}, {"/onnx_file": onnx_file}],
            emulate_tty=True,
        ),
        Node(
            package="bxi_scan_nav", executable="rgbd_camera_publisher", name="d435i_rgbd",
            output="screen", parameters=[{"model_file": model}], emulate_tty=True,
        ),
        Node(
            package="bxi_scan_nav", executable="d435i_pose_publisher", name="d435i_pose_publisher",
            output="screen", parameters=[nav_config], emulate_tty=True,
        ),
        Node(
            package="bxi_scan_nav", executable="cmd_vel_to_motion_commands", name="cmd_vel_to_motion_commands",
            output="screen", parameters=[nav_config], emulate_tty=True,
        ),
        Node(
            package="scan_planner", executable="scan_planner_node", name="scan_planner_node",
            output="screen", condition=IfCondition(LaunchConfiguration("start_scan_planner")),
            parameters=[planner_yaml, planner_overrides],
            remappings=[
                ("body_pose", "/simulation/odom"),
                ("sensor_pose", "/simulation/d435i/depth/pose"),
                ("depth", "/simulation/d435i/depth/image_raw"),
                ("move_base_simple/goal", "/move_base_simple/goal"),
                ("initial_path", "/initial_path"),
            ],
            emulate_tty=True,
        ),
        Node(
            package="scan_planner", executable="closed_loop_controller", name="closed_loop_controller",
            output="screen", condition=IfCondition(LaunchConfiguration("start_scan_planner")),
            parameters=[controllers_yaml, controller_overrides],
            remappings=[
                ("body_pose", "/simulation/odom"),
                ("cmd_vel", "/cmd_vel"),
                ("planning/bspline", "/planning/bspline"),
            ],
            emulate_tty=True,
        ),
    ])
