from pathlib import Path

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
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
                # Full-resolution RTAB RGB-D occasionally has an isolated
                # scheduling stall. Keep its watchdog tolerant without
                # weakening the faster localization backends.
                "odom_timeout_sec": 0.8 if backend == "rtabmap" else 0.5,
                # RTAB-Map publishes a finite identity pose with very large
                # covariance when visual tracking is lost. Do not treat that
                # sentinel as valid navigation odometry.
                "require_covariance": backend == "rtabmap",
                "max_position_variance": 0.25,
                "max_orientation_variance": 0.25,
                "allow_origin_reset_recovery": False,
                "enforce_flat_floor": ParameterValue(
                    LaunchConfiguration("enforce_flat_floor"), value_type=bool
                ),
                "max_floor_z_drift_m": ParameterValue(
                    LaunchConfiguration("max_floor_z_drift_m"), value_type=float
                ),
                "max_floor_tilt_rad": ParameterValue(
                    LaunchConfiguration("max_floor_tilt_rad"), value_type=float
                ),
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
    rtabmap_imu_world_frame = LaunchConfiguration(
        "rtabmap_imu_world_frame"
    ).perform(context).lower()
    if rtabmap_imu_world_frame not in ("enu", "ned", "nwu"):
        raise RuntimeError("rtabmap_imu_world_frame must be enu, ned, or nwu")
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
    rtabmap_force_3dof = as_bool(
        LaunchConfiguration("rtabmap_force_3dof").perform(context)
    )
    if rtabmap_motion_profile not in ("stable", "fast"):
        raise RuntimeError("rtabmap_motion_profile must be stable or fast")
    rtabmap_odom_parameters = {
        "Odom/Strategy": "0",
        "Odom/ImageDecimation": "2",
        "Odom/GuessMotion": "false",
        "Odom/KeyFrameThr": "0.50",
        "OdomF2M/MaxSize": "1200" if rtabmap_motion_profile == "fast" else "1600",
        "OdomF2M/BundleAdjustmentMaxFrames": (
            "3" if rtabmap_motion_profile == "fast" else "5"
        ),
        "Vis/MinInliers": "20",
        "Vis/MaxFeatures": "1200" if rtabmap_motion_profile == "fast" else "1600",
        "Vis/MinInliersDistribution": "0.005",
        "Vis/MinDepth": "0.25",
        "Vis/MaxDepth": "5.0",
        "Vis/PnPReprojError": "3.0",
        "Odom/FilteringStrategy": "0",
        "Odom/ResetCountdown": "15" if rtabmap_motion_profile == "fast" else "10",
        "Reg/Force3DoF": "true" if rtabmap_force_3dof else "false",
    }
    rtabmap_mapping_profile = LaunchConfiguration("rtabmap_mapping_profile").perform(
        context
    ).lower()
    if rtabmap_mapping_profile not in ("balanced", "fine"):
        raise RuntimeError("rtabmap_mapping_profile must be balanced or fine")
    rtabmap_mapping_parameters = {
        "Mem/ImagePreDecimation": "1" if rtabmap_mapping_profile == "fine" else "2",
        "Rtabmap/DetectionRate": "2.0",
        "RGBD/LinearUpdate": "0.05" if rtabmap_mapping_profile == "fine" else "0.10",
        "RGBD/AngularUpdate": "0.05" if rtabmap_mapping_profile == "fine" else "0.10",
        "Grid/DepthDecimation": "2" if rtabmap_mapping_profile == "fine" else "4",
        "Grid/CellSize": "0.04" if rtabmap_mapping_profile == "fine" else "0.05",
        "Grid/RangeMin": "0.25",
        "Grid/RangeMax": "5.0",
        "Reg/Force3DoF": "true" if rtabmap_force_3dof else "false",
        "Optimizer/Slam2D": "true" if rtabmap_force_3dof else "false",
    }
    if rtabmap_mapping_profile == "fine":
        rtabmap_mapping_parameters.update(
            {
                "Grid/NoiseFilteringRadius": "0.08",
                "Grid/NoiseFilteringMinNeighbors": "3",
            }
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
    orb_imu_axis_mode = LaunchConfiguration("orb_imu_axis_mode").perform(context)
    if orb_imu_axis_mode not in ("none", "ros_base_to_orb_camera"):
        raise RuntimeError(
            "orb_imu_axis_mode must be none or ros_base_to_orb_camera"
        )
    orb_strict_localization_only = as_bool(
        LaunchConfiguration("orb_strict_localization_only").perform(context)
    )
    start_rig_tf = as_bool(LaunchConfiguration("start_rig_tf").perform(context))
    start_orb_node = as_bool(
        LaunchConfiguration("start_orb_node").perform(context)
    )
    start_input_guard = as_bool(
        LaunchConfiguration("start_input_guard").perform(context)
    )
    rtabmap_approx_sync = (
        "true"
        if as_bool(LaunchConfiguration("rtabmap_approx_sync").perform(context))
        else "false"
    )
    rtabmap_topic_queue_size = int(
        LaunchConfiguration("rtabmap_topic_queue_size").perform(context)
    )
    rtabmap_sync_queue_size = int(
        LaunchConfiguration("rtabmap_sync_queue_size").perform(context)
    )
    if rtabmap_topic_queue_size <= 0 or rtabmap_sync_queue_size <= 0:
        raise RuntimeError("RTAB-Map queue sizes must be positive")

    localization_share = get_package_share_path("bxi_cuvslam_localization")
    rig_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(localization_share / "launch" / "rig_static_tf.launch.py")
        ),
        launch_arguments={
            "body_camera_uses_head_mount": LaunchConfiguration(
                "body_camera_uses_head_mount"
            ),
            "use_realsense_internal_tf": LaunchConfiguration(
                "use_realsense_internal_tf"
            ),
            "head_link_x": LaunchConfiguration("head_link_x"),
            "head_link_y": LaunchConfiguration("head_link_y"),
            "head_link_z": LaunchConfiguration("head_link_z"),
            "head_link_roll": LaunchConfiguration("head_link_roll"),
            "head_link_pitch": LaunchConfiguration("head_link_pitch"),
            "head_link_yaw": LaunchConfiguration("head_link_yaw"),
        }.items(),
    )

    input_guard = Node(
        package="bxi_cuvslam_localization",
        executable="head_camera_rig_guard",
        name="localization_input_guard",
        output="screen",
        parameters=[
            {
                "head_color_topic": color_topic,
                "head_depth_topic": depth_topic,
                "joint_states_topic": LaunchConfiguration("joint_states_topic"),
                "imu_topic": imu_topic,
                "require_imu": False,
                "require_head_lock": ParameterValue(
                    LaunchConfiguration("require_head_lock"), value_type=bool
                ),
                "rgb_depth_sync_limit_ms": ParameterValue(
                    LaunchConfiguration("rgb_depth_sync_limit_ms"), value_type=float
                ),
                "required_good_sets": 10,
            }
        ],
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
                    "start_rig_guard": LaunchConfiguration("start_input_guard"),
                    "require_rig_ready": LaunchConfiguration("start_input_guard"),
                    "require_head_lock": LaunchConfiguration("require_head_lock"),
                    "start_sim_error_monitor": "false",
                    "use_gpu": LaunchConfiguration("cuvslam_use_gpu"),
                    "use_realsense_internal_tf": LaunchConfiguration(
                        "use_realsense_internal_tf"
                    ),
                }.items(),
            )
        ]

    actions = [rig_tf] if start_rig_tf else []
    if start_input_guard:
        actions.append(input_guard)
    if backend == "rtabmap":
        intra_process = [{"use_intra_process_comms": True}]
        rgbd_topic = "/rtabmap/rgbd_image"
        odom_topic = "/localization/rtabmap/odom"
        common_parameters = {
            "frame_id": "bxi_base_link",
            "approx_sync": rtabmap_approx_sync == "true",
            "approx_sync_max_interval": ParameterValue(
                LaunchConfiguration("rtabmap_approx_sync_max_interval"),
                value_type=float,
            ),
            # odom_info is emitted after visual odometry. Keep enough source
            # images for slower robot CPUs so CoreWrapper can still pair the
            # delayed result with its originating RGB-D frame.
            "topic_queue_size": rtabmap_topic_queue_size,
            "sync_queue_size": rtabmap_sync_queue_size,
            "qos": 2,
            "qos_image": 2,
            "qos_camera_info": 2,
            "qos_odom": 2,
            "qos_imu": 2,
            "wait_for_transform": 0.1,
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            ),
        }
        component_nodes = []
        if rtabmap_use_imu and rtabmap_filter_imu:
            component_nodes.append(
                ComposableNode(
                    package="imu_filter_madgwick",
                    plugin="ImuFilterMadgwickRos",
                    namespace="nav",
                    name="camera_imu_filter",
                    parameters=[
                        {
                            "use_mag": False,
                            "publish_tf": False,
                            "world_frame": rtabmap_imu_world_frame,
                            "gain": ParameterValue(
                                LaunchConfiguration("rtabmap_imu_gain"),
                                value_type=float,
                            ),
                            "zeta": ParameterValue(
                                LaunchConfiguration("rtabmap_imu_zeta"),
                                value_type=float,
                            ),
                            "orientation_stddev": 0.02,
                        }
                    ],
                    remappings=[
                        ("imu/data_raw", imu_topic),
                        ("imu/data", filtered_imu_topic),
                    ],
                    extra_arguments=intra_process,
                )
            )
        component_nodes.extend(
            [
                ComposableNode(
                    package="rtabmap_sync",
                    plugin="rtabmap_sync::RGBDSync",
                    namespace="rtabmap",
                    name="rgbd_sync",
                    parameters=[common_parameters],
                    remappings=[
                        ("rgb/image", color_topic),
                        ("depth/image", depth_topic),
                        ("rgb/camera_info", camera_info_topic),
                        ("rgbd_image", rgbd_topic),
                    ],
                    extra_arguments=intra_process,
                ),
                ComposableNode(
                    package="rtabmap_odom",
                    plugin="rtabmap_odom::RGBDOdometry",
                    namespace="rtabmap",
                    name="rgbd_odometry",
                    parameters=[
                        {
                            **common_parameters,
                            **rtabmap_odom_parameters,
                            "subscribe_rgbd": True,
                            "subscribe_odom_info": True,
                            "imu_queue_size": 2000,
                            "odom_frame_id": odom_frame,
                            # The guarded bridge is the sole odom -> base TF
                            # authority, so rejected jumps cannot leak into TF.
                            "publish_tf": False,
                            "wait_imu_to_init": rtabmap_use_imu,
                            "always_process_most_recent_frame": ParameterValue(
                                LaunchConfiguration(
                                    "rtabmap_odom_always_process_most_recent_frame"
                                ),
                                value_type=bool,
                            ),
                        }
                    ],
                    remappings=[
                        ("rgbd_image", rgbd_topic),
                        ("imu", rtabmap_imu_topic),
                        ("odom", odom_topic),
                    ],
                    extra_arguments=intra_process,
                ),
                ComposableNode(
                    package="rtabmap_slam",
                    plugin="rtabmap_slam::CoreWrapper",
                    namespace="rtabmap",
                    name="rtabmap",
                    parameters=[
                        {
                            **common_parameters,
                            **rtabmap_mapping_parameters,
                            # Reuse the synchronized RGBDImage consumed by odometry.
                            # This keeps CoreWrapper on the exact same frame and
                            # avoids a second pair of large raw-image subscriptions.
                            "subscribe_rgbd": True,
                            "subscribe_depth": False,
                            "subscribe_odom_info": True,
                            "map_frame_id": map_frame,
                            # Leave this empty so CoreWrapper subscribes to
                            # /nav/odom.  Setting an odom frame makes it read
                            # the raw odometry TF instead, bypassing the
                            # localization bridge and its fail-closed checks.
                            "odom_frame_id": "",
                            "publish_tf": True,
                            "database_path": LaunchConfiguration(
                                "rtabmap_database"
                            ),
                            "delete_db_on_start": mode == "mapping",
                            "latch": False,
                            "Mem/IncrementalMemory": (
                                "false" if mode == "localization" else "true"
                            ),
                            "Mem/InitWMWithAllNodes": (
                                "true" if mode == "localization" else "false"
                            ),
                            "Mem/SaveDepth16Format": "true",
                            "Mem/UseOdomGravity": "true",
                            "Rtabmap/LoopThr": "0.15",
                            "RGBD/OptimizeMaxError": "1.0",
                            "RGBD/ProximityBySpace": "true",
                            "RGBD/NeighborLinkRefining": "true",
                            "RGBD/CreateOccupancyGrid": "true",
                            "Optimizer/Robust": "true",
                            "Optimizer/GravitySigma": "0.3",
                            "Grid/3D": "true",
                            "Grid/RayTracing": "true",
                        }
                    ],
                    remappings=[
                        ("rgbd_image", rgbd_topic),
                        ("imu", rtabmap_imu_topic),
                        # Only insert frames whose odometry passed the
                        # continuity, covariance and flat-floor checks in the
                        # localization bridge.  Feeding the raw RTAB odometry
                        # here let mapping continue after tracking jumps even
                        # though /nav/localization_valid was already false.
                        ("odom", "/nav/odom"),
                    ],
                    extra_arguments=intra_process,
                ),
            ]
        )
        actions.append(
            ComposableNodeContainer(
                name="rtabmap_rgbd_imu_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                output="screen",
                emulate_tty=True,
                composable_node_descriptions=component_nodes,
            )
        )
        actions.append(
            common_bridge(
                "rtabmap",
                odom_topic,
                odom_frame,
                require_rig_ready=start_input_guard,
                anchor_initial_pose=mode != "localization",
                publish_tf=True,
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
    if start_orb_node:
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
                            LaunchConfiguration("orb_max_time_diff"),
                            value_type=float,
                        ),
                        "nominal_frame_dt": 1.0 / 30.0,
                        "allow_unsynced_rgbd": False,
                        "use_logical_time": False,
                        "imu_axis_mode": orb_imu_axis_mode,
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
            require_rig_ready=start_input_guard,
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
            DeclareLaunchArgument("start_orb_node", default_value="true"),
            DeclareLaunchArgument("start_input_guard", default_value="false"),
            DeclareLaunchArgument("require_head_lock", default_value="true"),
            DeclareLaunchArgument("enforce_flat_floor", default_value="false"),
            DeclareLaunchArgument("max_floor_z_drift_m", default_value="0.12"),
            DeclareLaunchArgument("max_floor_tilt_rad", default_value="0.30"),
            DeclareLaunchArgument("rgb_depth_sync_limit_ms", default_value="2.0"),
            DeclareLaunchArgument(
                "use_realsense_internal_tf", default_value="false"
            ),
            DeclareLaunchArgument("head_link_x", default_value="0.0628"),
            DeclareLaunchArgument("head_link_y", default_value="0.0175"),
            DeclareLaunchArgument("head_link_z", default_value="0.2515"),
            DeclareLaunchArgument("head_link_roll", default_value="0.0"),
            DeclareLaunchArgument("head_link_pitch", default_value="0.0"),
            DeclareLaunchArgument("head_link_yaw", default_value="0.0"),
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
            DeclareLaunchArgument("rtabmap_approx_sync", default_value="true"),
            DeclareLaunchArgument("rtabmap_topic_queue_size", default_value="100"),
            DeclareLaunchArgument("rtabmap_sync_queue_size", default_value="100"),
            DeclareLaunchArgument("rtabmap_filter_imu", default_value="false"),
            DeclareLaunchArgument("rtabmap_imu_world_frame", default_value="enu"),
            DeclareLaunchArgument("rtabmap_imu_gain", default_value="0.1"),
            DeclareLaunchArgument("rtabmap_imu_zeta", default_value="0.0"),
            DeclareLaunchArgument(
                "rtabmap_filtered_imu_topic", default_value="/nav/camera_imu"
            ),
            DeclareLaunchArgument(
                "rtabmap_motion_profile", default_value="stable"
            ),
            DeclareLaunchArgument("rtabmap_force_3dof", default_value="false"),
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
            DeclareLaunchArgument(
                "orb_imu_axis_mode", default_value="ros_base_to_orb_camera"
            ),
            DeclareLaunchArgument("orb_max_time_diff", default_value="0.01"),
            DeclareLaunchArgument(
                "orb_strict_localization_only", default_value="false"
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
