import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_path("bxi_example_py_elf3")
    xml_file = os.path.join(
        package_share,
        "data/mujoco_simulation/elf3_400m_track.xml",
    )
    state_machine_config = os.path.join(
        package_share,
        "config/elf3_400m_run_state_machine.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "show_camera_view",
                default_value="false",
                description="Open a small OpenCV window for the robot front camera topic.",
            ),
            DeclareLaunchArgument(
                "camera_image_topic",
                default_value="/simulation/front_line_camera/image_raw",
                description="ROS image topic shown by the OpenCV viewer.",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/simulation/front_line_camera/camera_info",
                description="ROS camera info topic for the simulated front camera.",
            ),
            DeclareLaunchArgument(
                "wait_for_start_signal",
                default_value="true",
                description="Wait for /simulation/start_run before releasing the robot to run.",
            ),
            DeclareLaunchArgument(
                "start_signal_topic",
                default_value="/simulation/start_run",
                description="std_msgs/Bool topic used to start the 400m run.",
            ),
            DeclareLaunchArgument(
                "enable_line_follow",
                default_value="true",
                description="Publish motion_commands from the front camera line offset.",
            ),
            DeclareLaunchArgument(
                "line_enable_lateral_correction",
                default_value="false",
                description="Enable visual lateral velocity correction without changing forward speed.",
            ),
            DeclareLaunchArgument(
                "line_detector_type",
                default_value="traditional",
                description="Line detector backend: traditional or ufldv2.",
            ),
            DeclareLaunchArgument(
                "use_cpp_line_detector",
                default_value="false",
                description="Use the C++ OpenCV detector to publish line_state instead of Python camera detection.",
            ),
            DeclareLaunchArgument(
                "show_cpp_line_debug",
                default_value="false",
                description="Open a C++ OpenCV window for the C++ line detector debug image.",
            ),
            DeclareLaunchArgument(
                "ufldv2_model_path",
                default_value="/home/hwc/下载/ufldv2_tusimple_res18_320x800.onnx",
                description="Path to the UFLDv2 ONNX model used when line_detector_type:=ufldv2.",
            ),
            DeclareLaunchArgument(
                "ufldv2_dataset",
                default_value="auto",
                description="UFLDv2 dataset preset: auto, tusimple, culane, or curvelanes.",
            ),
            DeclareLaunchArgument(
                "line_forward_vel",
                default_value="1.5",
                description="Forward velocity command used by the line follower.",
            ),
            DeclareLaunchArgument(
                "line_max_amp_input_vx",
                default_value="1.5",
                description="Maximum forward velocity sent to the AMP state machine before its speed profile scaling.",
            ),
            DeclareLaunchArgument(
                "line_yaw_gain",
                default_value="0.28",
                description="Yaw proportional gain used by the line follower.",
            ),
            DeclareLaunchArgument(
                "line_control_mode",
                default_value="pid",
                description="Line follower control mode: stanley or pid.",
            ),
            DeclareLaunchArgument(
                "line_pid_error_source",
                default_value="offset",
                description="PID error source: heading, offset, preview, or control.",
            ),
            DeclareLaunchArgument(
                "line_pid_heading_feedforward_gain",
                default_value="0.45",
                description="Heading contribution used by the preview PID error.",
            ),
            DeclareLaunchArgument(
                "line_stanley_heading_gain",
                default_value="0.10",
                description="Stanley heading error gain.",
            ),
            DeclareLaunchArgument(
                "line_stanley_crosstrack_gain",
                default_value="0.18",
                description="Stanley cross-track error gain.",
            ),
            DeclareLaunchArgument(
                "line_stanley_yaw_gain",
                default_value="0.18",
                description="Stanley yaw-rate output scale.",
            ),
            DeclareLaunchArgument(
                "line_max_yawdot",
                default_value="0.12",
                description="Maximum absolute line follower yaw rate.",
            ),
            DeclareLaunchArgument(
                "line_max_yawdot_rate",
                default_value="0.35",
                description="Maximum line follower yaw-rate change per second.",
            ),
            DeclareLaunchArgument(
                "line_max_amp_input_yawdot",
                default_value="0.10",
                description="Maximum yaw command sent to the AMP state machine.",
            ),
            DeclareLaunchArgument(
                "line_offset_filter_alpha",
                default_value="0.18",
                description="Low-pass alpha for visual offset and heading errors.",
            ),
            DeclareLaunchArgument(
                "line_offset_deadband",
                default_value="0.04",
                description="Deadband for visual cross-track error.",
            ),
            DeclareLaunchArgument(
                "line_heading_deadband",
                default_value="0.04",
                description="Deadband for visual heading error.",
            ),
            DeclareLaunchArgument(
                "line_control_deadband",
                default_value="0.035",
                description="Deadband for combined visual control error.",
            ),
            DeclareLaunchArgument(
                "line_pid_derivative_filter_alpha",
                default_value="0.18",
                description="Low-pass alpha for the line follower PID derivative term.",
            ),
            DeclareLaunchArgument(
                "line_confidence_hold_threshold",
                default_value="0.20",
                description="Below this visual confidence the controller holds and decays the last estimate.",
            ),
            DeclareLaunchArgument(
                "line_confidence_full_speed_threshold",
                default_value="0.60",
                description="Confidence needed before the line follower uses full commanded speed.",
            ),
            DeclareLaunchArgument(
                "line_pid_kp",
                default_value="-1.0",
                description="Line follower PID proportional gain. Negative value reuses line_yaw_gain.",
            ),
            DeclareLaunchArgument(
                "line_pid_ki",
                default_value="0.0",
                description="Line follower PID integral gain.",
            ),
            DeclareLaunchArgument(
                "line_pid_kd",
                default_value="0.01",
                description="Line follower PID derivative gain.",
            ),
            DeclareLaunchArgument(
                "line_integral_limit",
                default_value="0.6",
                description="Absolute clamp for the line follower PID integral term.",
            ),
            Node(
                package="mujoco",
                executable="simulation",
                name="simulation_mujoco",
                output="screen",
                parameters=[
                    {"simulation/model_file": xml_file},
                ],
                emulate_tty=True,
            ),
            Node(
                package="bxi_example_py_elf3",
                executable="bxi_example_py_elf3_demo",
                name="bxi_example_py_elf3_400m_run",
                output="screen",
                parameters=[
                    {"/topic_prefix": "simulation/"},
                    {"/state_machine_config": state_machine_config},
                    {"/initial_cmd_vel": [0.0, 0.0, 0.0]},
                    {"/lock_initial_cmd_vel": False},
                    {"/use_sim_reset": False},
                    {"/initial_base_pose": [-14.76825, 13.385, 1.1, 0.0, 0.0, 0.0, 1.0]},
                    {
                        "/wait_for_start_signal": ParameterValue(
                            LaunchConfiguration("wait_for_start_signal"),
                            value_type=bool,
                        )
                    },
                    {"/start_signal_topic": LaunchConfiguration("start_signal_topic")},
                    {"/hot_reload": False},
                ],
                emulate_tty=True,
            ),
            Node(
                package="bxi_example_py_elf3",
                executable="front_line_camera_publisher",
                name="front_line_camera_publisher",
                output="screen",
                parameters=[
                    {"model_file": xml_file},
                    {"topic_prefix": "simulation/"},
                    {"image_topic": LaunchConfiguration("camera_image_topic")},
                    {"camera_info_topic": LaunchConfiguration("camera_info_topic")},
                    {"camera_name": "front_line_camera"},
                    {"camera_frame_id": "front_line_camera"},
                    {"width": 640},
                    {"height": 360},
                    {"publish_hz": 50.0},
                    {
                        "enable_line_detection": ParameterValue(
                            PythonExpression(["'", LaunchConfiguration("use_cpp_line_detector"), "' != 'true'"]),
                            value_type=bool,
                        )
                    },
                    {"line_detector_type": LaunchConfiguration("line_detector_type")},
                    {"ufldv2_model_path": LaunchConfiguration("ufldv2_model_path")},
                    {"ufldv2_dataset": LaunchConfiguration("ufldv2_dataset")},
                    {"line_offset_topic": "/simulation/front_line_camera/line_offset"},
                    {"line_state_topic": "/simulation/front_line_camera/line_state"},
                ],
                additional_env={"MUJOCO_GL": "egl"},
                emulate_tty=True,
            ),
            Node(
                package="bxi_example_cpp_line_detector",
                executable="cpp_line_detector",
                name="cpp_line_detector",
                output="screen",
                parameters=[
                    {"image_topic": LaunchConfiguration("camera_image_topic")},
                    {"line_offset_topic": "/simulation/front_line_camera/line_offset"},
                    {"line_state_topic": "/simulation/front_line_camera/line_state"},
                    {"debug_image_topic": "/simulation/front_line_camera/cpp_line_debug"},
                    {"enable_debug_image": True},
                    {"roi_top_ratio": 0.50},
                    {"roi_bottom_ratio": 0.96},
                    {"threshold_value": 155},
                    {"canny_low": 60.0},
                    {"canny_high": 150.0},
                ],
                condition=IfCondition(LaunchConfiguration("use_cpp_line_detector")),
                emulate_tty=True,
            ),
            Node(
                package="bxi_example_cpp_line_detector",
                executable="cpp_line_debug_viewer",
                name="cpp_line_debug_viewer",
                output="screen",
                parameters=[
                    {"image_topic": "/simulation/front_line_camera/cpp_line_debug"},
                    {"window_name": "cpp_line_debug"},
                    {"window_width": 640},
                    {"window_height": 360},
                ],
                condition=IfCondition(LaunchConfiguration("show_cpp_line_debug")),
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
                    {
                        "enable_lateral_correction": ParameterValue(
                            LaunchConfiguration("line_enable_lateral_correction"),
                            value_type=bool,
                        )
                    },
                    {"lateral_velocity_gain": 0.35},
                    {"max_lateral_velocity": 0.20},
                    {"forward_accel_limit": 0.35},
                    {
                        "max_amp_input_vx": ParameterValue(
                            LaunchConfiguration("line_max_amp_input_vx"),
                            value_type=float,
                        )
                    },
                    {
                        "max_amp_input_yawdot": ParameterValue(
                            LaunchConfiguration("line_max_amp_input_yawdot"),
                            value_type=float,
                        )
                    },
                    {
                        "forward_vel": ParameterValue(
                            LaunchConfiguration("line_forward_vel"),
                            value_type=float,
                        )
                    },
                    {"control_mode": LaunchConfiguration("line_control_mode")},
                    {"pid_error_source": LaunchConfiguration("line_pid_error_source")},
                    {
                        "pid_heading_feedforward_gain": LaunchConfiguration(
                            "line_pid_heading_feedforward_gain"
                        )
                    },
                    {
                        "stanley_heading_gain": ParameterValue(
                            LaunchConfiguration("line_stanley_heading_gain"),
                            value_type=float,
                        )
                    },
                    {
                        "stanley_crosstrack_gain": ParameterValue(
                            LaunchConfiguration("line_stanley_crosstrack_gain"),
                            value_type=float,
                        )
                    },
                    {
                        "stanley_yaw_gain": ParameterValue(
                            LaunchConfiguration("line_stanley_yaw_gain"),
                            value_type=float,
                        )
                    },
                    {
                        "yaw_gain": ParameterValue(
                            LaunchConfiguration("line_yaw_gain"),
                            value_type=float,
                        )
                    },
                    {
                        "pid_kp": ParameterValue(
                            LaunchConfiguration("line_pid_kp"),
                            value_type=float,
                        )
                    },
                    {
                        "pid_ki": ParameterValue(
                            LaunchConfiguration("line_pid_ki"),
                            value_type=float,
                        )
                    },
                    {
                        "pid_kd": ParameterValue(
                            LaunchConfiguration("line_pid_kd"),
                            value_type=float,
                        )
                    },
                    {
                        "integral_limit": ParameterValue(
                            LaunchConfiguration("line_integral_limit"),
                            value_type=float,
                        )
                    },
                    {
                        "max_yawdot": ParameterValue(
                            LaunchConfiguration("line_max_yawdot"),
                            value_type=float,
                        )
                    },
                    {
                        "max_yawdot_rate": ParameterValue(
                            LaunchConfiguration("line_max_yawdot_rate"),
                            value_type=float,
                        )
                    },
                    {
                        "offset_filter_alpha": ParameterValue(
                            LaunchConfiguration("line_offset_filter_alpha"),
                            value_type=float,
                        )
                    },
                    {
                        "offset_deadband": ParameterValue(
                            LaunchConfiguration("line_offset_deadband"),
                            value_type=float,
                        )
                    },
                    {
                        "heading_deadband": ParameterValue(
                            LaunchConfiguration("line_heading_deadband"),
                            value_type=float,
                        )
                    },
                    {
                        "control_deadband": ParameterValue(
                            LaunchConfiguration("line_control_deadband"),
                            value_type=float,
                        )
                    },
                    {
                        "pid_derivative_filter_alpha": ParameterValue(
                            LaunchConfiguration("line_pid_derivative_filter_alpha"),
                            value_type=float,
                        )
                    },
                    {
                        "confidence_hold_threshold": ParameterValue(
                            LaunchConfiguration("line_confidence_hold_threshold"),
                            value_type=float,
                        )
                    },
                    {
                        "confidence_full_speed_threshold": ParameterValue(
                            LaunchConfiguration("line_confidence_full_speed_threshold"),
                            value_type=float,
                        )
                    },
                    {"publish_hz": 50.0},
                ],
                condition=IfCondition(LaunchConfiguration("enable_line_follow")),
                emulate_tty=True,
            ),
            Node(
                package="bxi_example_py_elf3",
                executable="front_line_camera_viewer",
                name="front_line_camera_viewer",
                output="screen",
                parameters=[
                    {"image_topic": LaunchConfiguration("camera_image_topic")},
                    {"window_name": "front_line_camera"},
                    {"window_width": 640},
                    {"window_height": 360},
                    {"line_detector_type": LaunchConfiguration("line_detector_type")},
                    {"ufldv2_model_path": LaunchConfiguration("ufldv2_model_path")},
                    {"ufldv2_dataset": LaunchConfiguration("ufldv2_dataset")},
                ],
                condition=IfCondition(LaunchConfiguration("show_camera_view")),
            ),
        ]
    )
