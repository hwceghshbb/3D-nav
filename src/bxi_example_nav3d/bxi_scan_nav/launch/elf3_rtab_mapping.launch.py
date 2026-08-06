from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def as_bool(value):
    return str(value).lower() in ("1", "true", "yes", "on")


def launch_setup(context, *args, **kwargs):
    nav_share = get_package_share_path("bxi_scan_nav")
    rtabmap_share = get_package_share_path("rtabmap_launch")

    rtabmap_args = LaunchConfiguration("rtabmap_args").perform(context).strip()
    if as_bool(LaunchConfiguration("delete_db_on_start").perform(context)):
        rtabmap_args = (rtabmap_args + " --delete_db_on_start").strip()

    rtabmap_launch = rtabmap_share / "launch" / "rtabmap.launch.py"
    rviz_config = nav_share / "config" / "elf3_rtab_mapping.rviz"

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(rtabmap_launch)),
            launch_arguments={
                "stereo": "false",
                "localization": LaunchConfiguration("localization"),
                "rtabmap_viz": LaunchConfiguration("rtabmap_viz"),
                "rviz": "false",
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "frame_id": LaunchConfiguration("base_frame_id"),
                "map_frame_id": LaunchConfiguration("map_frame_id"),
                "publish_tf_map": LaunchConfiguration("publish_tf_map"),
                "namespace": LaunchConfiguration("namespace"),
                "database_path": LaunchConfiguration("database_path"),
                "topic_queue_size": LaunchConfiguration("topic_queue_size"),
                "queue_size": LaunchConfiguration("queue_size"),
                "qos": LaunchConfiguration("qos"),
                "qos_image": LaunchConfiguration("qos_image"),
                "qos_camera_info": LaunchConfiguration("qos_camera_info"),
                "qos_odom": LaunchConfiguration("qos_odom"),
                "wait_for_transform": LaunchConfiguration("wait_for_transform"),
                "approx_sync": "true",
                "approx_sync_max_interval": LaunchConfiguration(
                    "approx_sync_max_interval"
                ),
                "rgb_topic": LaunchConfiguration("rgb_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "depth": "true",
                "rgbd_sync": "true",
                "approx_rgbd_sync": "true",
                "subscribe_rgbd": "true",
                "visual_odometry": "false",
                "icp_odometry": "false",
                "odom_topic": LaunchConfiguration("odom_topic"),
                "publish_tf_odom": "false",
                "map_topic": "map",
                "args": rtabmap_args,
                "output": "screen",
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rtab_mapping_rviz",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_rviz")),
            arguments=["-d", str(rviz_config)],
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
        "--Rtabmap/LoopThr 0.11 "
        "--RGBD/ProximityBySpace true "
        "--RGBD/ProximityByTime false "
        "--RGBD/OptimizeMaxError 1.0 "
        "--Optimizer/Strategy 1 "
        "--Optimizer/Robust true "
        "--Vis/MinInliers 20 "
        "--Kp/MaxFeatures 2000 "
        "--Grid/RangeMax 4.5 "
        "--Grid/MinClusterSize 5"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rgb_topic", default_value="/camera/color/image_raw"
            ),
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/camera/depth/image_rect_raw",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/depth/camera_info",
            ),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("base_frame_id", default_value="base_link"),
            DeclareLaunchArgument("map_frame_id", default_value="map"),
            DeclareLaunchArgument("namespace", default_value="rtabmap"),
            DeclareLaunchArgument("publish_tf_map", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("localization", default_value="false"),
            DeclareLaunchArgument("delete_db_on_start", default_value="false"),
            DeclareLaunchArgument(
                "database_path", default_value="/tmp/bxi_elf3_rtabmap.db"
            ),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("rtabmap_viz", default_value="false"),
            DeclareLaunchArgument("topic_queue_size", default_value="100"),
            DeclareLaunchArgument("queue_size", default_value="100"),
            DeclareLaunchArgument("qos", default_value="2"),
            DeclareLaunchArgument("qos_image", default_value="2"),
            DeclareLaunchArgument("qos_camera_info", default_value="2"),
            DeclareLaunchArgument("qos_odom", default_value="1"),
            DeclareLaunchArgument("wait_for_transform", default_value="0.1"),
            DeclareLaunchArgument(
                "approx_sync_max_interval", default_value="0.50"
            ),
            DeclareLaunchArgument("rtabmap_args", default_value=mapping_args),
            OpaqueFunction(function=launch_setup),
        ]
    )
