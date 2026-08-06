from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    localization_share = get_package_share_path("bxi_cuvslam_localization")
    elf3_share = get_package_share_path("bxi_example_py_elf3")
    workspace_root = localization_share.resolve().parents[2]
    localization_launch = localization_share / "launch" / "elf3_localization.launch.py"
    default_model = (
        elf3_share
        / "data"
        / "mujoco_simulation"
        / "elf3_rooms_nav3d_features.xml"
    )
    state_machine_config = elf3_share / "config" / "elf3_state_machine.yaml"
    return LaunchDescription(
        [
            DeclareLaunchArgument("localization_backend", default_value="rtabmap"),
            DeclareLaunchArgument("mode", default_value="odometry"),
            DeclareLaunchArgument("orb_sensor_mode", default_value="rgbd"),
            DeclareLaunchArgument("motion_start_delay", default_value="15.0"),
            DeclareLaunchArgument(
                "rtabmap_database",
                default_value="/tmp/elf3_localization_benchmark.db",
            ),
            DeclareLaunchArgument("model_file", default_value=str(default_model)),
            DeclareLaunchArgument("estimate_topic", default_value="/nav/odom"),
            DeclareLaunchArgument(
                "output_path",
                default_value=str(
                    workspace_root / "maps" / "localization_benchmark.json"
                ),
            ),
            Node(
                package="mujoco",
                executable="simulation",
                name="simulation_mujoco",
                output="screen",
                parameters=[
                    {"simulation/model_file": LaunchConfiguration("model_file")}
                ],
                emulate_tty=True,
            ),
            Node(
                package="bxi_example_py_elf3",
                executable="bxi_example_py_elf3_demo",
                name="elf3_policy",
                output="screen",
                parameters=[
                    {"/topic_prefix": "simulation/"},
                    {"/state_machine_config": str(state_machine_config)},
                ],
                emulate_tty=True,
            ),
            Node(
                package="bxi_cuvslam_localization",
                executable="benchmark_rgbd_camera_publisher",
                name="localization_benchmark_rgbd",
                output="screen",
                parameters=[
                    {
                        "model_file": LaunchConfiguration("model_file"),
                        "camera_name": "head_depth_camera",
                        "output_prefix": "/simulation/d435i/",
                        "frame_id": "head_depth_camera_depth_optical_frame",
                        "color_frame_id": "head_depth_camera_color_optical_frame",
                        "hidden_body_names": ["torso_link"],
                        "width": 640,
                        "height": 360,
                        "publish_hz": 30.0,
                        "publish_color": True,
                        "publish_raw_depth": False,
                        "depth_encoding": "16UC1",
                        "align_depth_to_color": True,
                    }
                ],
                emulate_tty=True,
            ),
            Node(
                package="bxi_cuvslam_localization",
                executable="benchmark_rgbd_camera_publisher",
                name="localization_benchmark_body_depth",
                output="screen",
                parameters=[
                    {
                        "model_file": LaunchConfiguration("model_file"),
                        "camera_name": "body_depth_camera",
                        "output_prefix": "/simulation/body_depth_camera/",
                        "frame_id": "body_depth_camera_depth_optical_frame",
                        "color_frame_id": "body_depth_camera_color_optical_frame",
                        "width": 424,
                        "height": 240,
                        "publish_hz": 30.0,
                        "publish_color": False,
                        "publish_raw_depth": False,
                        "depth_encoding": "16UC1",
                        "align_depth_to_color": False,
                    }
                ],
                emulate_tty=True,
            ),
            Node(
                package="bxi_cuvslam_localization",
                executable="sim_imu_specific_force",
                name="orb_sim_imu_specific_force",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("localization_backend"),
                            "' == 'orbslam3' and '",
                            LaunchConfiguration("orb_sensor_mode"),
                            "' == 'rgbd_inertial'",
                        ]
                    )
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(localization_launch)),
                launch_arguments={
                    "localization_backend": LaunchConfiguration(
                        "localization_backend"
                    ),
                    "mode": LaunchConfiguration("mode"),
                    "rtabmap_database": LaunchConfiguration("rtabmap_database"),
                    "cuvslam_map_directory": "/tmp/elf3_cuvslam_benchmark",
                    "orb_map_directory": "/tmp/elf3_orbslam3_benchmark",
                    "orb_imu_topic": "/nav/simulation/imu_specific_force",
                    "orb_sensor_mode": LaunchConfiguration("orb_sensor_mode"),
                }.items(),
            ),
            Node(
                package="bxi_cuvslam_localization",
                executable="localization_benchmark_motion",
                output="screen",
                parameters=[
                    {
                        "start_delay_sec": ParameterValue(
                            LaunchConfiguration("motion_start_delay"),
                            value_type=float,
                        ),
                        "linear_speed": 0.18,
                        "yaw_speed": 0.25,
                    }
                ],
            ),
            Node(
                package="bxi_cuvslam_localization",
                executable="localization_benchmark",
                name="localization_benchmark",
                output="screen",
                parameters=[
                    {
                        "backend_name": LaunchConfiguration(
                            "localization_backend"
                        ),
                        "estimate_topic": LaunchConfiguration("estimate_topic"),
                        "truth_topic": "/simulation/odom",
                        "output_path": LaunchConfiguration("output_path"),
                        "warmup_sec": 12.0,
                    }
                ],
            ),
        ]
    )
