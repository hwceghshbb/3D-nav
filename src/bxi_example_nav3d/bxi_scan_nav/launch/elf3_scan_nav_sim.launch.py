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
    model = os.path.join(elf3_share, "data", "mujoco_simulation", "elf3_rooms_datagen.xml")
    onnx_file = os.path.join(elf3_share, "data", "mjlab_model", "model_normal.onnx")
    config = os.path.join(nav_share, "config", "navigation.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("start_controller", default_value="true"),
        Node(
            package="mujoco", executable="simulation", name="simulation_mujoco",
            output="screen", parameters=[{"simulation/model_file": model}], emulate_tty=True,
        ),
        Node(
            package="bxi_example_py_elf3", executable="bxi_example_py_elf3_mjlab",
            name="elf3_policy", output="screen",
            condition=IfCondition(LaunchConfiguration("start_controller")),
            emulate_tty=True,
            parameters=[{"/topic_prefix": "simulation/"}, {"/onnx_file": onnx_file}],
        ),
        Node(
            package="bxi_scan_nav", executable="rgbd_camera_publisher", name="d435i_rgbd",
            output="screen", parameters=[{"model_file": model}], emulate_tty=True,
        ),
        Node(
            package="bxi_scan_nav", executable="d435i_pose_publisher", name="d435i_pose_publisher",
            output="screen", parameters=[config], emulate_tty=True,
        ),
        Node(
            package="bxi_scan_nav", executable="depth_to_pointcloud", name="depth_to_pointcloud",
            output="screen", parameters=[config], emulate_tty=True,
        ),
        Node(
            package="bxi_scan_nav", executable="scan_planner_adapter", name="scan_planner_adapter",
            output="screen", parameters=[config], emulate_tty=True,
        ),
        Node(
            package="bxi_scan_nav", executable="cmd_vel_to_motion_commands", name="cmd_vel_to_motion_commands",
            output="screen", parameters=[config], emulate_tty=True,
        ),
    ])
