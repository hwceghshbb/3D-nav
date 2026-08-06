from pathlib import Path

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


BACKENDS = ("cuvslam", "rtabmap", "orbslam3")


def as_bool(value):
    return str(value).lower() in ("1", "true", "yes", "on")


def common_bridge(
    backend,
    input_topic,
    odom_frame,
    require_rig_ready=False,
    anchor_initial_pose=False,
    publish_tf=True,
):
    return Node(
        package="bxi_cuvslam_localization",
        executable="localization_odom_bridge",
        name=f"{backend}_odom_bridge",
        output="screen",
        parameters=[
            {
                "input_odom_topic": input_topic,
                "output_odom_topic": "/nav/odom",
                "valid_topic": "/nav/localization_valid",
                "odom_frame": odom_frame,
                "base_frame": "bxi_base_link",
                "require_rig_ready": require_rig_ready,
                "backend_name": backend,
                "anchor_initial_pose": anchor_initial_pose,
                "publish_tf": publish_tf,
                "initial_base_position": [0.0, 0.0, 1.1],
                "odom_timeout_sec": 0.5,
                # RTAB-Map publishes a finite identity pose with very large
                # covariance when visual tracking is lost. Do not treat that
                # sentinel as valid navigation odometry.
                "require_covariance": backend == "rtabmap",
                "max_position_variance": 0.25,
                "max_orientation_variance": 0.25,
            }
        ],
    )


def common_pose_publishers(target_frame, camera_frame, camera_pose_topic):
    return [
        Node(
            package="bxi_cuvslam_localization",
            executable="tf_pose_publisher",
            name="base_global_pose_publisher",
            output="screen",
            parameters=[
                {
                    "target_frame": target_frame,
                    "tracked_frame": "bxi_base_link",
                    "output_topic": "/nav/base_footprint/pose",
                    "valid_topic": "/nav/localization_valid",
                    "odom_topic": "/nav/odom",
                    "publish_hz": 30.0,
                    "transform_timeout_sec": 0.1,
                    "copy_odom_twist": True,
                }
            ],
        ),
        Node(
            package="bxi_cuvslam_localization",
            executable="tf_pose_publisher",
            name="head_camera_global_pose_publisher",
            output="screen",
            parameters=[
                {
                    "target_frame": target_frame,
                    "tracked_frame": camera_frame,
                    "output_topic": camera_pose_topic,
                    "valid_topic": "/nav/localization_valid",
                    "odom_topic": "/nav/odom",
                    "publish_hz": 30.0,
                    "transform_timeout_sec": 0.1,
                    "copy_odom_twist": False,
                }
            ],
        ),
    ]


def launch_setup(context, *args, **kwargs):
    backend = LaunchConfiguration("localization_backend").perform(context).lower()
    if backend not in BACKENDS:
        raise RuntimeError(
            f"localization_backend must be one of {BACKENDS}, got {backend!r}"
        )

    mode = LaunchConfiguration("mode").perform(context).lower()
    if mode not in ("mapping", "localization", "odometry"):
        raise RuntimeError("mode must be mapping, localization, or odometry")

    color_topic = LaunchConfiguration("color_topic").perform(context)
    depth_topic = LaunchConfiguration("depth_topic").perform(context)
    camera_info_topic = LaunchConfiguration("camera_info_topic").perform(context)
    imu_topic = LaunchConfiguration("imu_topic").perform(context)
    orb_imu_topic = LaunchConfiguration("orb_imu_topic").perform(context)
    rtabmap_use_imu = as_bool(
        LaunchConfiguration("rtabmap_use_imu").perform(context)
    )
    rtabmap_filter_imu = as_bool(
        LaunchConfiguration("rtabmap_filter_imu").perform(context)
    )
    filtered_imu_topic = LaunchConfiguration("rtabmap_filtered_imu_topic").perform(
        context
    )
    rtabmap_imu_topic = (
        (filtered_imu_topic if rtabmap_filter_imu else imu_topic)
        if rtabmap_use_imu
        else "/localization/rtabmap/unused_imu"
    )
    rtabmap_motion_profile = LaunchConfiguration("rtabmap_motion_profile").perform(
        context
    ).lower()
    if rtabmap_motion_profile not in ("stable", "fast"):
        raise RuntimeError("rtabmap_motion_profile must be stable or fast")
    if rtabmap_motion_profile == "fast":
        rtabmap_odom_args = (
            "--Odom/Strategy 0 --Odom/ImageDecimation 2 "
            "--Odom/GuessMotion false "
            "--Odom/KeyFrameThr 0.50 --OdomF2M/MaxSize 1200 "
            "--OdomF2M/BundleAdjustmentMaxFrames 3 "
            "--Vis/MinInliers 20 --Vis/MaxFeatures 1200 "
            "--Vis/MinInliersDistribution 0.005 "
            "--Vis/MinDepth 0.25 --Vis/MaxDepth 5.0 "
            "--Vis/PnPReprojError 3.0 "
            "--Odom/FilteringStrategy 0 --Odom/ResetCountdown 15"
        )
    else:
        rtabmap_odom_args = (
            "--Odom/Strategy 0 --Odom/ImageDecimation 2 "
            "--Odom/GuessMotion false --Odom/KeyFrameThr 0.5 "
            "--OdomF2M/MaxSize 1600 "
            "--OdomF2M/BundleAdjustmentMaxFrames 5 "
            "--Vis/MinInliers 20 --Vis/MaxFeatures 1600 "
            "--Vis/MinInliersDistribution 0.005 "
            "--Vis/MinDepth 0.25 --Vis/MaxDepth 5.0 "
            "--Odom/FilteringStrategy 0 --Odom/ResetCountdown 10"
        )
    rtabmap_mapping_profile = LaunchConfiguration("rtabmap_mapping_profile").perform(
        context
    ).lower()
    if rtabmap_mapping_profile not in ("balanced", "fine"):
        raise RuntimeError("rtabmap_mapping_profile must be balanced or fine")
    if rtabmap_mapping_profile == "fine":
        rtabmap_mapping_args = (
            "--Mem/ImagePreDecimation 1 --Rtabmap/DetectionRate 1.0 "
            "--Grid/DepthDecimation 2 --Grid/CellSize 0.04 "
            "--Grid/RangeMin 0.25 --Grid/RangeMax 5.0 "
            "--Grid/NoiseFilteringRadius 0.08 "
            "--Grid/NoiseFilteringMinNeighbors 3 "
        )
    else:
        rtabmap_mapping_args = (
            "--Mem/ImagePreDecimation 2 --Rtabmap/DetectionRate 2.0 "
            "--Grid/DepthDecimation 4 --Grid/CellSize 0.05 "
            "--Grid/RangeMin 0.25 --Grid/RangeMax 5.0 "
        )
    map_frame = LaunchConfiguration("map_frame").perform(context)
    odom_frame = LaunchConfiguration("odom_frame").perform(context)
    pose_target_frame = LaunchConfiguration("pose_target_frame").perform(context)
    nav_camera_frame_id = LaunchConfiguration("nav_camera_frame_id").perform(context)
    nav_camera_pose_topic = LaunchConfiguration("nav_camera_pose_topic").perform(
        context
    )
    orb_sensor_mode = LaunchConfiguration("orb_sensor_mode").perform(context).lower()
    if orb_sensor_mode not in ("rgbd", "rgbd_inertial"):
        raise RuntimeError("orb_sensor_mode must be rgbd or rgbd_inertial")
    orb_strict_localization_only = as_bool(
        LaunchConfiguration("orb_strict_localization_only").perform(context)
    )
    start_rig_tf = as_bool(LaunchConfiguration("start_rig_tf").perform(context))

    localization_share = get_package_share_path("bxi_cuvslam_localization")
    rig_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(localization_share / "launch" / "rig_static_tf.launch.py")
        ),
        launch_arguments={
            "body_camera_uses_head_mount": LaunchConfiguration(
                "body_camera_uses_head_mount"
            ),
        }.items(),
    )

    if backend == "cuvslam":
        cuvslam_launch = localization_share / "launch" / "elf3_head_cuvslam.launch.py"
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(cuvslam_launch)),
                launch_arguments={
                    "mode": "localization" if mode == "localization" else "mapping",
                    "map_frame": odom_frame,
                    "pose_target_frame": pose_target_frame,
                    "map_directory": LaunchConfiguration("cuvslam_map_directory"),
                    "head_color_topic": color_topic,
                    "head_depth_topic": depth_topic,
                    "head_camera_info_topic": camera_info_topic,
                    "imu_topic": imu_topic,
                    "joint_states_topic": LaunchConfiguration("joint_states_topic"),
                    "start_imu_combiner": "false",
                    "start_sim_error_monitor": "false",
                    "use_gpu": LaunchConfiguration("cuvslam_use_gpu"),
                }.items(),
            )
        ]

    actions = [rig_tf] if start_rig_tf else []
    if backend == "rtabmap":
        if rtabmap_use_imu and rtabmap_filter_imu:
            actions.append(
                Node(
                    package="imu_filter_madgwick",
                    executable="imu_filter_madgwick_node",
                    name="nav_camera_imu_filter",
                    output="screen",
                    parameters=[
                        {
                            "use_mag": False,
                            "publish_tf": False,
                            "world_frame": "enu",
                            "gain": 0.1,
                            "zeta": 0.0,
                            "orientation_stddev": 0.02,
                        }
                    ],
                    remappings=[
                        ("imu/data_raw", imu_topic),
                        ("imu/data", filtered_imu_topic),
                    ],
                )
            )
        rtabmap_launch = (
            get_package_share_path("rtabmap_launch") / "launch" / "rtabmap.launch.py"
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(rtabmap_launch)),
                launch_arguments={
                    "stereo": "false",
                    "depth": "true",
                    "visual_odometry": "true",
                    "icp_odometry": "false",
                    "localization": "true" if mode == "localization" else "false",
                    "rtabmap_viz": "false",
                    "rviz": "false",
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "frame_id": "bxi_base_link",
                    "map_frame_id": map_frame,
                    "vo_frame_id": odom_frame,
                    "publish_tf_odom": "true",
                    "publish_tf_map": "true",
                    "database_path": LaunchConfiguration("rtabmap_database"),
                    "rgb_topic": color_topic,
                    "depth_topic": depth_topic,
                    "camera_info_topic": camera_info_topic,
                    "imu_topic": rtabmap_imu_topic,
                    "odom_topic": "/localization/rtabmap/odom",
                    "rgbd_sync": "true",
                    "subscribe_rgbd": "true",
                    "approx_rgbd_sync": "true",
                    "approx_sync": "true",
                    "approx_sync_max_interval": LaunchConfiguration(
                        "rtabmap_approx_sync_max_interval"
                    ),
                    "qos": "2",
                    "qos_odom": "2",
                    "qos_imu": "2",
                    "topic_queue_size": "30",
                    "queue_size": "30",
                    "wait_imu_to_init": "true" if rtabmap_use_imu else "false",
                    "odom_always_process_most_recent_frame": LaunchConfiguration(
                        "rtabmap_odom_always_process_most_recent_frame"
                    ),
                    "odom_args": rtabmap_odom_args,
                    "args": (
                        "--Reg/Force3DoF false --RGBD/ForceOdom3DoF false "
                        "--Mem/SaveDepth16Format true "
                        + rtabmap_mapping_args
                        + "--Rtabmap/LoopThr 0.11 "
                        "--RGBD/OptimizeMaxError 1.0 "
                        "--RGBD/ProximityBySpace true "
                        "--Optimizer/Robust true"
                    ),
                }.items(),
            )
        )
        actions.append(
            common_bridge(
                "rtabmap",
                "/localization/rtabmap/odom",
                odom_frame,
                publish_tf=False,
            )
        )
        actions.extend(
            common_pose_publishers(
                pose_target_frame, nav_camera_frame_id, nav_camera_pose_topic
            )
        )
        return actions

    orb_share = get_package_share_path("bxi_orbslam3_ros2")
    vocabulary = Path(LaunchConfiguration("orb_vocabulary").perform(context))
    settings = Path(LaunchConfiguration("orb_settings").perform(context))
    map_directory = Path(
        LaunchConfiguration("orb_map_directory").perform(context)
    ).expanduser()
    map_directory.mkdir(parents=True, exist_ok=True)
    if not vocabulary.is_file():
        raise RuntimeError(f"ORB-SLAM3 vocabulary not found: {vocabulary}")
    if not settings.is_file():
        raise RuntimeError(f"ORB-SLAM3 settings not found: {settings}")
    atlas_stem = LaunchConfiguration("orb_atlas_name").perform(context)
    atlas_file = map_directory / f"{atlas_stem}.osa"
    if mode == "localization" and not atlas_file.is_file():
        raise RuntimeError(f"ORB-SLAM3 atlas not found: {atlas_file}")

    environment = {
        "ORB_SLAM3_VIEWER": "0",
        "ORB_SLAM3_WORKING_DIRECTORY": str(map_directory),
        "ORB_SLAM3_LOAD_ATLAS": atlas_stem if mode == "localization" else "",
        "ORB_SLAM3_SAVE_ATLAS": atlas_stem if mode == "mapping" else "",
        # ORB-SLAM3 creates a new current map after loading an atlas. Keeping
        # local mapping enabled lets place recognition merge that session into
        # the loaded map. Strict only-tracking mode freezes this new map before
        # a merge can happen, so it is experimental and opt-in.
        "ORB_SLAM3_LOCALIZATION_ONLY": (
            "1" if mode == "localization" and orb_strict_localization_only else "0"
        ),
        "ORB_SLAM3_SENSOR_MODE": orb_sensor_mode,
    }
    actions.append(
        Node(
            package="bxi_orbslam3_ros2",
            executable="rgbd_inertial",
            name="orbslam3_rgbd_inertial",
            output="screen",
            arguments=[str(vocabulary), str(settings)],
            additional_env=environment,
            parameters=[
                {
                    "publish_pose": True,
                    "use_imu": orb_sensor_mode == "rgbd_inertial",
                    "publish_tf": False,
                    "map_frame_id": odom_frame,
                    "odom_frame_id": odom_frame,
                    "child_frame_id": "bxi_base_link",
                    "output_base_pose": True,
                    "base_from_camera_translation": [0.0628, 0.0175, 0.2515],
                    "base_from_camera_quaternion": [-0.5, 0.5, -0.5, 0.5],
                    "max_time_diff": ParameterValue(
                        LaunchConfiguration("orb_max_time_diff"), value_type=float
                    ),
                    "nominal_frame_dt": 1.0 / 30.0,
                    "allow_unsynced_rgbd": False,
                    "use_logical_time": False,
                    "imu_axis_mode": "ros_base_to_orb_camera",
                }
            ],
            remappings=[
                ("camera/rgb", color_topic),
                ("camera/depth", depth_topic),
                ("camera/camera_info", camera_info_topic),
                ("camera/imu", orb_imu_topic),
                ("orbslam3/odom", "/localization/orbslam3/odom"),
                ("orbslam3/pose", "/localization/orbslam3/pose"),
                ("orbslam3/map_points", "/localization/orbslam3/map_points"),
                ("orbslam3/rgbd_image", "/localization/orbslam3/rgbd_image"),
            ],
        )
    )
    actions.append(
        common_bridge(
            "orbslam3",
            "/localization/orbslam3/odom",
            odom_frame,
            anchor_initial_pose=mode != "localization",
        )
    )
    actions.extend(
        common_pose_publishers(
            pose_target_frame, nav_camera_frame_id, nav_camera_pose_topic
        )
    )
    return actions


def generate_launch_description():
    workspace_root = get_package_share_path("bxi_cuvslam_localization").resolve().parents[2]
    orb_share = get_package_share_path("bxi_orbslam3_ros2")
    return LaunchDescription(
        [
            DeclareLaunchArgument("localization_backend", default_value="rtabmap"),
            DeclareLaunchArgument("mode", default_value="odometry"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument(
                "nav_camera_frame_id",
                default_value="head_depth_camera_depth_optical_frame",
            ),
            DeclareLaunchArgument(
                "nav_camera_pose_topic", default_value="/nav/head_depth_camera/pose"
            ),
            DeclareLaunchArgument(
                "body_camera_uses_head_mount", default_value="false"
            ),
            DeclareLaunchArgument("start_rig_tf", default_value="true"),
            DeclareLaunchArgument(
                "pose_target_frame", default_value=LaunchConfiguration("map_frame")
            ),
            DeclareLaunchArgument(
                "color_topic", default_value="/simulation/d435i/color/image_raw"
            ),
            DeclareLaunchArgument(
                "depth_topic", default_value="/simulation/d435i/depth/image_rect_raw"
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/simulation/d435i/color/camera_info",
            ),
            DeclareLaunchArgument("imu_topic", default_value="/simulation/imu_data"),
            DeclareLaunchArgument("rtabmap_use_imu", default_value="false"),
            DeclareLaunchArgument("rtabmap_filter_imu", default_value="false"),
            DeclareLaunchArgument(
                "rtabmap_filtered_imu_topic", default_value="/nav/camera_imu"
            ),
            DeclareLaunchArgument(
                "rtabmap_motion_profile", default_value="stable"
            ),
            DeclareLaunchArgument(
                "rtabmap_mapping_profile", default_value="balanced"
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "rtabmap_odom_always_process_most_recent_frame",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "orb_imu_topic", default_value=LaunchConfiguration("imu_topic")
            ),
            DeclareLaunchArgument(
                "joint_states_topic", default_value="/simulation/joint_states"
            ),
            DeclareLaunchArgument(
                "rtabmap_database",
                default_value=str(workspace_root / "maps" / "elf3_localization.db"),
            ),
            DeclareLaunchArgument(
                "rtabmap_approx_sync_max_interval", default_value="0.04"
            ),
            DeclareLaunchArgument(
                "cuvslam_map_directory",
                default_value=str(workspace_root / "maps" / "elf3_cuvslam"),
            ),
            DeclareLaunchArgument("cuvslam_use_gpu", default_value="true"),
            DeclareLaunchArgument(
                "orb_vocabulary",
                default_value=str(orb_share / "Vocabulary" / "ORBvoc.txt"),
            ),
            DeclareLaunchArgument(
                "orb_settings",
                default_value=str(
                    get_package_share_path("bxi_orbslam3_ros2")
                    / "config"
                    / "elf3_head_640x360_rgbd_imu.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "orb_map_directory",
                default_value=str(workspace_root / "maps" / "elf3_orbslam3"),
            ),
            DeclareLaunchArgument("orb_atlas_name", default_value="atlas"),
            DeclareLaunchArgument("orb_sensor_mode", default_value="rgbd"),
            DeclareLaunchArgument("orb_max_time_diff", default_value="0.01"),
            DeclareLaunchArgument(
                "orb_strict_localization_only", default_value="false"
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
