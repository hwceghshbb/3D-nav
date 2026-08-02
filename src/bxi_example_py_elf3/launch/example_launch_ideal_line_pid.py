import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_path("bxi_example_py_elf3")
    xml_file = os.path.join(package_share, "data/mujoco_simulation/elf3_400m_track.xml")
    state_machine_config = os.path.join(package_share, "config/elf3_400m_run_state_machine.yaml")

    arguments = [
        DeclareLaunchArgument("line_y", default_value="13.385"),
        DeclareLaunchArgument("line_heading", default_value="0.0"),
        DeclareLaunchArgument("initial_y", default_value="13.55"),
        DeclareLaunchArgument("error_sign", default_value="-1.0"),
        DeclareLaunchArgument("line_forward_vel", default_value="1.5"),
        DeclareLaunchArgument("line_max_amp_input_vx", default_value="1.5"),
        DeclareLaunchArgument("line_pid_kp", default_value="0.16"),
        DeclareLaunchArgument("line_pid_ki", default_value="0.0"),
        DeclareLaunchArgument("line_pid_kd", default_value="0.0"),
        DeclareLaunchArgument("line_pid_error_source", default_value="preview"),
        DeclareLaunchArgument("line_pid_heading_feedforward_gain", default_value="0.45"),
        DeclareLaunchArgument("line_max_yawdot", default_value="0.10"),
        DeclareLaunchArgument("line_max_yawdot_rate", default_value="0.35"),
        DeclareLaunchArgument("line_max_amp_input_yawdot", default_value="0.10"),
        DeclareLaunchArgument("line_offset_filter_alpha", default_value="0.35"),
    ]

    return LaunchDescription(
        arguments
        + [
            Node(
                package="mujoco",
                executable="simulation",
                name="simulation_mujoco",
                output="screen",
                parameters=[{"simulation/model_file": xml_file}],
                additional_env={"MUJOCO_GL": "egl"},
            ),
            Node(
                package="bxi_example_py_elf3",
                executable="bxi_example_py_elf3_demo",
                name="bxi_example_py_elf3_ideal_line",
                output="screen",
                parameters=[
                    {"/topic_prefix": "simulation/"},
                    {"/state_machine_config": state_machine_config},
                    {"/initial_cmd_vel": [0.0, 0.0, 0.0]},
                    {"/lock_initial_cmd_vel": False},
                    {"/use_sim_reset": False},
                    {
                        "/initial_base_pose": [
                            -14.76825,
                            LaunchConfiguration("initial_y"),
                            1.1,
                            0.0,
                            0.0,
                            0.0,
                            1.0,
                        ]
                    },
                    {"/wait_for_start_signal": False},
                    {"/hot_reload": False},
                ],
                emulate_tty=True,
            ),
            Node(
                package="bxi_example_py_elf3",
                executable="ideal_line_publisher",
                name="ideal_line_publisher",
                output="screen",
                parameters=[
                    {"line_y": ParameterValue(LaunchConfiguration("line_y"), value_type=float)},
                    {"line_heading": ParameterValue(LaunchConfiguration("line_heading"), value_type=float)},
                    {"error_sign": ParameterValue(LaunchConfiguration("error_sign"), value_type=float)},
                ],
                emulate_tty=True,
            ),
            Node(
                package="bxi_example_py_elf3",
                executable="line_follow_controller",
                name="line_follow_controller",
                output="screen",
                parameters=[
                    {"line_offset_topic": "/simulation/front_line_camera/line_offset"},
                    {"line_state_topic": "/simulation/front_line_camera/line_state"},
                    {"motion_commands_topic": "motion_commands"},
                    {"forward_vel": ParameterValue(LaunchConfiguration("line_forward_vel"), value_type=float)},
                    {"max_amp_input_vx": ParameterValue(LaunchConfiguration("line_max_amp_input_vx"), value_type=float)},
                    {"control_mode": "pid"},
                    {"pid_error_source": LaunchConfiguration("line_pid_error_source")},
                    {
                        "pid_heading_feedforward_gain": ParameterValue(
                            LaunchConfiguration("line_pid_heading_feedforward_gain"),
                            value_type=float,
                        )
                    },
                    {"yaw_gain": ParameterValue(LaunchConfiguration("line_pid_kp"), value_type=float)},
                    {"pid_kp": ParameterValue(LaunchConfiguration("line_pid_kp"), value_type=float)},
                    {"pid_ki": ParameterValue(LaunchConfiguration("line_pid_ki"), value_type=float)},
                    {"pid_kd": ParameterValue(LaunchConfiguration("line_pid_kd"), value_type=float)},
                    {"max_yawdot": ParameterValue(LaunchConfiguration("line_max_yawdot"), value_type=float)},
                    {"max_yawdot_rate": ParameterValue(LaunchConfiguration("line_max_yawdot_rate"), value_type=float)},
                    {"max_amp_input_yawdot": ParameterValue(LaunchConfiguration("line_max_amp_input_yawdot"), value_type=float)},
                    {"offset_filter_alpha": ParameterValue(LaunchConfiguration("line_offset_filter_alpha"), value_type=float)},
                    {"forward_accel_limit": 0.35},
                    {"enable_lateral_correction": False},
                ],
                emulate_tty=True,
            ),
        ]
    )
