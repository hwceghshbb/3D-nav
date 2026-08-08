import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("bxi_cuvslam_localization")
    config = os.path.join(share, "config", "localization.yaml")
    static_tf_launch = os.path.join(share, "launch", "rig_static_tf.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="localization"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("pose_target_frame", default_value="map"),
            DeclareLaunchArgument("map_directory", default_value="/opt/bxi/maps/elf3_cuvslam"),
            DeclareLaunchArgument(
                "head_color_topic",
                default_value="/hardware/head_depth_camera/color/image_raw",
            ),
            DeclareLaunchArgument(
                "head_depth_topic",
                default_value=(
                    "/hardware/head_depth_camera/aligned_depth_to_color/image_raw"
                ),
            ),
            DeclareLaunchArgument(
                "head_camera_info_topic",
                default_value="/hardware/head_depth_camera/color/camera_info",
            ),
            DeclareLaunchArgument("joint_states_topic", default_value="/hardware/joint_states"),
            DeclareLaunchArgument("map_anchor_odom_topic", default_value=""),
            DeclareLaunchArgument("map_anchor_pose_topic", default_value=""),
            DeclareLaunchArgument("require_map_anchor", default_value="false"),
            DeclareLaunchArgument("imu_topic", default_value="/nav/imu"),
            DeclareLaunchArgument("require_imu", default_value="true"),
            DeclareLaunchArgument("require_head_lock", default_value="true"),
            DeclareLaunchArgument("image_timeout_sec", default_value="0.35"),
            DeclareLaunchArgument("odom_timeout_sec", default_value="0.25"),
            DeclareLaunchArgument("start_imu_combiner", default_value="true"),
            DeclareLaunchArgument(
                "use_realsense_internal_tf", default_value="false"
            ),
            DeclareLaunchArgument("start_rig_guard", default_value="true"),
            DeclareLaunchArgument("require_rig_ready", default_value="true"),
            DeclareLaunchArgument("start_odom_bridge", default_value="true"),
            DeclareLaunchArgument("start_pose_publishers", default_value="true"),
            DeclareLaunchArgument("start_sim_error_monitor", default_value="false"),
            DeclareLaunchArgument(
                "sim_truth_odom_topic", default_value="/simulation/odom"
            ),
            DeclareLaunchArgument(
                "sim_error_translation_threshold_m", default_value="0.50"
            ),
            DeclareLaunchArgument(
                "sim_error_rotation_threshold_deg", default_value="20.0"
            ),
            DeclareLaunchArgument(
                "sim_error_record_path",
                default_value="/tmp/cuvslam_large_error_events.jsonl",
            ),
            DeclareLaunchArgument("use_gpu", default_value="true"),
            DeclareLaunchArgument("async_sba", default_value="true"),
            DeclareLaunchArgument("enable_slam_backend", default_value="true"),
            DeclareLaunchArgument("auto_relocalize", default_value="true"),
            DeclareLaunchArgument("require_initial_pose", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(static_tf_launch),
                launch_arguments={
                    "use_realsense_internal_tf": LaunchConfiguration(
                        "use_realsense_internal_tf"
                    )
                }.items(),
            ),
            Node(
                package="bxi_cuvslam_localization",
                executable="imu_combiner",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_imu_combiner")),
                parameters=[config],
            ),
            Node(
                package="bxi_cuvslam_localization",
                executable="head_camera_rig_guard",
                name="head_camera_rig_guard",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_rig_guard")),
                parameters=[
                    config,
                    {
                        "head_color_topic": LaunchConfiguration("head_color_topic"),
                        "head_depth_topic": LaunchConfiguration("head_depth_topic"),
                        "joint_states_topic": LaunchConfiguration("joint_states_topic"),
                        "imu_topic": LaunchConfiguration("imu_topic"),
                        "require_imu": ParameterValue(
                            LaunchConfiguration("require_imu"), value_type=bool
                        ),
                        "require_head_lock": ParameterValue(
                            LaunchConfiguration("require_head_lock"), value_type=bool
                        ),
                        "image_timeout_sec": ParameterValue(
                            LaunchConfiguration("image_timeout_sec"), value_type=float
                        ),
                    },
                ],
            ),
            Node(
                package="bxi_cuvslam_localization",
                executable="head_cuvslam_node",
                name="head_cuvslam_node",
                output="screen",
                parameters=[
                    config,
                    {
                        "mode": LaunchConfiguration("mode"),
                        "map_frame": LaunchConfiguration("map_frame"),
                        "map_directory": LaunchConfiguration("map_directory"),
                        "color_topic": LaunchConfiguration("head_color_topic"),
                        "depth_topic": LaunchConfiguration("head_depth_topic"),
                        "camera_info_topic": LaunchConfiguration("head_camera_info_topic"),
                        "map_anchor_odom_topic": LaunchConfiguration(
                            "map_anchor_odom_topic"
                        ),
                        "map_anchor_pose_topic": LaunchConfiguration(
                            "map_anchor_pose_topic"
                        ),
                        "require_map_anchor": ParameterValue(
                            LaunchConfiguration("require_map_anchor"), value_type=bool
                        ),
                        "auto_relocalize": ParameterValue(
                            LaunchConfiguration("auto_relocalize"), value_type=bool
                        ),
                        "require_initial_pose": ParameterValue(
                            LaunchConfiguration("require_initial_pose"), value_type=bool
                        ),
                        "use_gpu": ParameterValue(
                            LaunchConfiguration("use_gpu"), value_type=bool
                        ),
                        "async_sba": ParameterValue(
                            LaunchConfiguration("async_sba"), value_type=bool
                        ),
                        "enable_slam_backend": ParameterValue(
                            LaunchConfiguration("enable_slam_backend"), value_type=bool
                        ),
                    },
                ],
                emulate_tty=True,
            ),
            Node(
                package="bxi_cuvslam_localization",
                executable="localization_odom_bridge",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_odom_bridge")),
                parameters=[
                    config,
                    {
                        "odom_frame": ParameterValue(
                            LaunchConfiguration("map_frame"), value_type=str
                        ),
                        "odom_timeout_sec": ParameterValue(
                            LaunchConfiguration("odom_timeout_sec"), value_type=float
                        ),
                        "require_rig_ready": ParameterValue(
                            LaunchConfiguration("require_rig_ready"), value_type=bool
                        ),
                    },
                ],
            ),
            Node(
                package="bxi_cuvslam_localization",
                executable="cuvslam_sim_error_monitor",
                name="cuvslam_sim_error_monitor",
                output="screen",
                condition=IfCondition(
                    LaunchConfiguration("start_sim_error_monitor")
                ),
                parameters=[
                    {
                        "truth_topic": LaunchConfiguration(
                            "sim_truth_odom_topic"
                        ),
                        "translation_threshold_m": ParameterValue(
                            LaunchConfiguration(
                                "sim_error_translation_threshold_m"
                            ),
                            value_type=float,
                        ),
                        "rotation_threshold_deg": ParameterValue(
                            LaunchConfiguration(
                                "sim_error_rotation_threshold_deg"
                            ),
                            value_type=float,
                        ),
                        "record_path": LaunchConfiguration(
                            "sim_error_record_path"
                        ),
                    }
                ],
            ),
            Node(
                package="bxi_cuvslam_localization",
                executable="tf_pose_publisher",
                name="base_global_pose_publisher",
                output="screen",
                condition=IfCondition(
                    LaunchConfiguration("start_pose_publishers")
                ),
                parameters=[
                    {
                        "target_frame": ParameterValue(
                            LaunchConfiguration("pose_target_frame"), value_type=str
                        ),
                        "tracked_frame": "bxi_base_link",
                        "output_topic": "/nav/base_footprint/pose",
                        "valid_topic": "/nav/localization_valid",
                        "odom_topic": "/nav/odom",
                        "publish_hz": 30.0,
                        "transform_timeout_sec": 0.1,
                        "copy_odom_twist": True,
                    },
                ],
            ),
            Node(
                package="bxi_cuvslam_localization",
                executable="tf_pose_publisher",
                name="head_camera_global_pose_publisher",
                output="screen",
                condition=IfCondition(
                    LaunchConfiguration("start_pose_publishers")
                ),
                parameters=[
                    {
                        "target_frame": ParameterValue(
                            LaunchConfiguration("pose_target_frame"), value_type=str
                        ),
                        "tracked_frame": (
                            "head_depth_camera_depth_optical_frame"
                        ),
                        "output_topic": "/nav/head_depth_camera/pose",
                        "valid_topic": "/nav/localization_valid",
                        "odom_topic": "/nav/odom",
                        "publish_hz": 30.0,
                        "transform_timeout_sec": 0.1,
                        "copy_odom_twist": False,
                    },
                ],
            ),
        ]
    )
