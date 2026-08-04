import json
from pathlib import Path
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from .raw_image import parse_serialized_image


def quaternion_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return np.asarray(
        [
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        ],
        dtype=np.float64,
    )


def quaternion_rotate(quaternion, vector):
    xyz = np.asarray(quaternion[:3], dtype=np.float64)
    value = np.asarray(vector, dtype=np.float64)
    return value + 2.0 * np.cross(xyz, np.cross(xyz, value) + quaternion[3] * value)


def compose_pose(left_t, left_q, right_t, right_q):
    return (
        np.asarray(left_t) + quaternion_rotate(left_q, right_t),
        quaternion_multiply(left_q, right_q),
    )


def inverse_pose(translation, quaternion):
    inverse_q = np.asarray(
        [-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3]]
    )
    return -quaternion_rotate(inverse_q, translation), inverse_q


class HeadCuvslamNode(Node):
    def __init__(self):
        super().__init__("head_cuvslam_node")
        self._declare_parameters()
        self.mode = str(self.get_parameter("mode").value).lower()
        if self.mode not in ("mapping", "localization"):
            raise ValueError("mode must be 'mapping' or 'localization'")

        try:
            import cuvslam
        except ImportError as error:
            raise RuntimeError(
                "cuVSLAM is not installed for ROS Python. Run "
                "scripts/install_cuvslam_ros_python.sh first."
            ) from error
        self.cuvslam = cuvslam
        self.tracker = None
        self.camera_info = None
        self.last_color_message = None
        self.last_depth_message = None
        self.last_processed_stamp_ns = -1
        self.last_color_array = None
        self.last_track_stamp_ns = None
        self.last_output_translation = None
        self.last_output_stamp_ns = None
        self.initial_guess = self._parameter_pose("initial_guess")
        self.map_origin = self._parameter_pose("map_origin")
        self.initial_pose_received = False
        self.latest_map_anchor = None
        self.relocalization_started = False
        self.relocalized = self.mode == "mapping"
        self.save_in_progress = False
        self.map_save_event = threading.Event()
        self.state_lock = threading.Lock()
        self.frame_count = 0
        self.last_saved_frame_count = 0
        self.waiting_anchor_logged = False
        self.input_counts = {
            "color": 0,
            "depth": 0,
            "pairs": 0,
            "odom": 0,
            "slam": 0,
        }
        self.last_input_counts = self.input_counts.copy()
        self.last_diagnostic_time = time.monotonic()
        self.track_time_total_ms = 0.0
        self.track_time_max_ms = 0.0
        self.track_time_samples = 0
        self.stage_time_totals_ms = {
            "convert": 0.0,
            "track": 0.0,
            "publish": 0.0,
            "total": 0.0,
        }
        self.stage_time_max_ms = self.stage_time_totals_ms.copy()
        self.input_bytes = {"color": 0, "depth": 0}
        self.last_input_bytes = self.input_bytes.copy()
        self.frame_processing_lock = threading.Lock()

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.odom_publisher = self.create_publisher(
            Odometry,
            self.get_parameter("output_odom_topic").value,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.slam_pose_publisher = self.create_publisher(
            Odometry,
            self.get_parameter("output_slam_pose_topic").value,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.status_publisher = self.create_publisher(
            String, self.get_parameter("status_topic").value, latched_qos
        )
        self.relocalized_publisher = self.create_publisher(
            Bool, self.get_parameter("relocalized_topic").value, latched_qos
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        camera_callback_group = ReentrantCallbackGroup()
        self.create_subscription(
            Image,
            self.get_parameter("color_topic").value,
            self.color_callback,
            qos_profile_sensor_data,
            callback_group=camera_callback_group,
            raw=True,
        )
        self.create_subscription(
            Image,
            self.get_parameter("depth_topic").value,
            self.depth_callback,
            qos_profile_sensor_data,
            callback_group=camera_callback_group,
            raw=True,
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self.camera_info_callback,
            qos_profile_sensor_data,
            callback_group=camera_callback_group,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("initial_pose_topic").value,
            self.initial_pose_callback,
            10,
        )
        anchor_odom_topic = str(self.get_parameter("map_anchor_odom_topic").value)
        if anchor_odom_topic:
            self.create_subscription(
                Odometry,
                anchor_odom_topic,
                self.map_anchor_odom_callback,
                qos_profile_sensor_data,
            )
        anchor_pose_topic = str(self.get_parameter("map_anchor_pose_topic").value)
        if anchor_pose_topic:
            self.create_subscription(
                PoseWithCovarianceStamped,
                anchor_pose_topic,
                self.map_anchor_pose_callback,
                10,
            )
        self.create_service(Trigger, "~/save_map", self.save_map_service)
        self.create_service(Trigger, "~/relocalize", self.relocalize_service)
        self.create_timer(5.0, self.publish_input_diagnostics)
        self.publish_relocalized(self.relocalized)
        self.publish_status("waiting_for_head_rgbd")

    def _declare_parameters(self):
        parameters = {
            "mode": "localization",
            "color_topic": "/hardware/head_depth_camera/color/image_raw",
            "depth_topic": "/hardware/head_depth_camera/depth/image_rect_raw",
            "camera_info_topic": "/hardware/head_depth_camera/color/camera_info",
            "initial_pose_topic": "/initialpose",
            "map_anchor_odom_topic": "",
            "map_anchor_pose_topic": "",
            "require_map_anchor": False,
            "output_odom_topic": "/visual_slam/tracking/odometry",
            "output_slam_pose_topic": "/visual_slam/tracking/slam_odometry",
            "status_topic": "/nav/relocalization_status",
            "relocalized_topic": "/nav/relocalized",
            "map_frame": "map",
            "base_frame": "bxi_base_link",
            "publish_tf": False,
            "map_directory": "/opt/bxi/maps/elf3_cuvslam",
            "auto_relocalize": True,
            "require_initial_pose": False,
            "save_map_on_shutdown": True,
            "use_gpu": True,
            "async_sba": True,
            "enable_slam_backend": True,
            "slam_sync_mode": True,
            "slam_throttling_time_ms": 0,
            "sync_tolerance_ms": 1.0,
            "horizontal_search_radius": 8.0,
            "vertical_search_radius": 8.0,
            "horizontal_step": 0.5,
            "vertical_step": 0.25,
            "angular_step_rads": 0.05,
            "rig_from_camera_translation": [0.0628, 0.0175, 0.2515],
            "rig_from_camera_quaternion": [-0.5, 0.5, -0.5, 0.5],
            "map_origin_translation": [0.0, 0.0, 0.0],
            "map_origin_quaternion": [0.0, 0.0, 0.0, 1.0],
            "initial_guess_translation": [0.0, 0.0, 0.0],
            "initial_guess_quaternion": [0.0, 0.0, 0.0, 1.0],
        }
        for name, default in parameters.items():
            self.declare_parameter(name, default)

    def _parameter_pose(self, prefix):
        translation = np.asarray(
            self.get_parameter(f"{prefix}_translation").value, dtype=np.float64
        )
        quaternion = np.asarray(
            self.get_parameter(f"{prefix}_quaternion").value, dtype=np.float64
        )
        quaternion /= np.linalg.norm(quaternion)
        return translation, quaternion

    def camera_info_callback(self, message):
        self.camera_info = message

    def color_callback(self, serialized):
        message = parse_serialized_image(serialized)
        self.input_counts["color"] += 1
        self.input_bytes["color"] += len(message.data)
        self.last_color_message = message
        self.try_process_pair()

    def depth_callback(self, serialized):
        message = parse_serialized_image(serialized)
        self.input_counts["depth"] += 1
        self.input_bytes["depth"] += len(message.data)
        self.last_depth_message = message
        self.try_process_pair()

    def initial_pose_callback(self, message):
        pose = message.pose.pose
        self.initial_guess = (
            np.asarray([pose.position.x, pose.position.y, pose.position.z]),
            np.asarray(
                [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ]
            ),
        )
        self.initial_pose_received = True
        self.get_logger().info("Updated cuVSLAM relocalization initial guess")
        if self.mode == "localization" and self.tracker is not None:
            self.start_relocalization()

    def map_anchor_odom_callback(self, message):
        self.update_map_anchor(message.pose.pose)

    def map_anchor_pose_callback(self, message):
        self.update_map_anchor(message.pose.pose)

    def update_map_anchor(self, pose):
        if self.tracker is not None:
            return
        self.latest_map_anchor = (
            np.asarray([pose.position.x, pose.position.y, pose.position.z]),
            np.asarray(
                [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ]
            ),
        )

    def try_process_pair(self):
        if not self.frame_processing_lock.acquire(blocking=False):
            return
        try:
            self._try_process_pair()
        finally:
            self.frame_processing_lock.release()

    def _try_process_pair(self):
        if self.camera_info is None or self.last_color_message is None or self.last_depth_message is None:
            return
        color_stamp = self.last_color_message.stamp_ns
        depth_stamp = self.last_depth_message.stamp_ns
        tolerance_ns = int(float(self.get_parameter("sync_tolerance_ms").value) * 1e6)
        if abs(color_stamp - depth_stamp) > tolerance_ns:
            return
        stamp_ns = max(color_stamp, depth_stamp)
        if stamp_ns <= self.last_processed_stamp_ns:
            return
        self.last_processed_stamp_ns = stamp_ns
        self.input_counts["pairs"] += 1
        total_started = time.perf_counter()
        try:
            convert_started = time.perf_counter()
            color = self.image_to_color(self.last_color_message)
            depth, depth_scale = self.image_to_depth(self.last_depth_message)
            convert_ms = (time.perf_counter() - convert_started) * 1000.0
            if color.shape[:2] != depth.shape:
                raise ValueError(
                    f"unaligned RGB-D dimensions: color={color.shape[:2]} depth={depth.shape}"
                )
            if self.tracker is None:
                if (
                    self.mode == "mapping"
                    and bool(self.get_parameter("require_map_anchor").value)
                    and self.latest_map_anchor is None
                ):
                    if not self.waiting_anchor_logged:
                        self.get_logger().info("Waiting for the global mapping pose anchor")
                        self.publish_status("waiting_for_map_anchor")
                        self.waiting_anchor_logged = True
                    return
                self.initialize_tracker(depth_scale)
            track_started = time.perf_counter()
            pose_estimate, slam_pose = self.tracker.track(
                stamp_ns, [color], depths=[depth]
            )
            track_ms = (time.perf_counter() - track_started) * 1000.0
            self.last_color_array = color
            self.last_track_stamp_ns = stamp_ns
            self.frame_count += 1
            if (
                self.mode == "localization"
                and not self.relocalized
                and not self.relocalization_started
                and bool(self.get_parameter("auto_relocalize").value)
                and (
                    not bool(self.get_parameter("require_initial_pose").value)
                    or self.initial_pose_received
                )
            ):
                self.start_relocalization()
            publish_started = time.perf_counter()
            if self.relocalized and pose_estimate.world_from_rig is not None:
                # RTAB-Map requires continuous frame-to-frame odometry. slam_pose
                # includes pose-graph/loop-closure corrections and can jump.
                self.publish_pose(pose_estimate.world_from_rig.pose, stamp_ns)
                self.input_counts["odom"] += 1
            if self.relocalized and slam_pose is not None:
                self.publish_slam_pose(slam_pose, stamp_ns)
                self.input_counts["slam"] += 1
            publish_ms = (time.perf_counter() - publish_started) * 1000.0
            total_ms = (time.perf_counter() - total_started) * 1000.0
            samples = {
                "convert": convert_ms,
                "track": track_ms,
                "publish": publish_ms,
                "total": total_ms,
            }
            for name, value in samples.items():
                self.stage_time_totals_ms[name] += value
                self.stage_time_max_ms[name] = max(
                    self.stage_time_max_ms[name], value
                )
            self.track_time_total_ms += track_ms
            self.track_time_max_ms = max(self.track_time_max_ms, track_ms)
            self.track_time_samples += 1
        except Exception as error:
            self.get_logger().error(
                f"cuVSLAM frame rejected: {error}", throttle_duration_sec=2.0
            )
            self.publish_status(f"tracking_error: {error}")

    def publish_input_diagnostics(self):
        now = time.monotonic()
        elapsed = max(now - self.last_diagnostic_time, 1e-6)
        rates = {
            name: (count - self.last_input_counts[name]) / elapsed
            for name, count in self.input_counts.items()
        }
        bandwidth = {
            name: (count - self.last_input_bytes[name]) / elapsed / (1024.0 * 1024.0)
            for name, count in self.input_bytes.items()
        }
        stage_summary = " ".join(
            f"{name}={self.stage_time_totals_ms[name] / self.track_time_samples:.2f}/"
            f"{self.stage_time_max_ms[name]:.2f}ms"
            for name in ("convert", "track", "publish", "total")
        ) if self.track_time_samples else "no processed samples"
        self.get_logger().info(
            "cuVSLAM rates: "
            + " ".join(f"{name}={rate:.1f}Hz" for name, rate in rates.items())
            + " bandwidth="
            + "/".join(f"{bandwidth[name]:.1f}" for name in ("color", "depth"))
            + "MiB/s(color/depth) stages(avg/max): "
            + stage_summary
        )
        self.last_input_counts = self.input_counts.copy()
        self.last_diagnostic_time = now
        self.last_input_bytes = self.input_bytes.copy()
        self.track_time_total_ms = 0.0
        self.track_time_max_ms = 0.0
        self.track_time_samples = 0
        for name in self.stage_time_totals_ms:
            self.stage_time_totals_ms[name] = 0.0
            self.stage_time_max_ms[name] = 0.0

    def initialize_tracker(self, depth_scale):
        info = self.camera_info
        camera = self.cuvslam.Camera(
            size=(int(info.width), int(info.height)),
            focal=(float(info.k[0]), float(info.k[4])),
            principal=(float(info.k[2]), float(info.k[5])),
        )
        rig_t, rig_q = self._parameter_pose("rig_from_camera")
        camera.rig_from_camera = self.cuvslam.Pose(
            translation=rig_t, rotation=rig_q
        )
        rgbd = self.cuvslam.Tracker.OdometryRGBDSettings(
            depth_scale_factor=depth_scale,
            depth_camera_id=0,
            enable_depth_stereo_tracking=False,
        )
        odom_config = self.cuvslam.Tracker.OdometryConfig(
            odometry_mode=self.cuvslam.Tracker.OdometryMode.RGBD,
            rgbd_settings=rgbd,
            use_gpu=bool(self.get_parameter("use_gpu").value),
            async_sba=bool(self.get_parameter("async_sba").value),
            enable_final_landmarks_export=False,
        )
        slam_config = None
        if bool(self.get_parameter("enable_slam_backend").value):
            slam_config = self.cuvslam.Tracker.SlamConfig(
                sync_mode=bool(self.get_parameter("slam_sync_mode").value),
                planar_constraints=False,
                max_map_size=0,
                throttling_time_ms=int(
                    self.get_parameter("slam_throttling_time_ms").value
                ),
            )
        self.cuvslam.warm_up_gpu()
        if self.mode == "mapping":
            if self.latest_map_anchor is not None:
                self.map_origin = self.latest_map_anchor
                self.get_logger().info(
                    "Anchored cuVSLAM origin to the current global map pose"
                )
            else:
                self.get_logger().warning(
                    "No mapping pose anchor received; cuVSLAM map origin will be local"
                )
        self.tracker = self.cuvslam.Tracker(
            self.cuvslam.Rig([camera]), odom_config, slam_config
        )
        self.load_metadata_if_present()
        self.publish_status(f"{self.mode}_tracking")
        self.get_logger().info(
            f"Head RGB-D cuVSLAM started: {info.width}x{info.height}, "
            f"depth_scale={depth_scale:g}, use_gpu={odom_config.use_gpu}, "
            f"async_sba={odom_config.async_sba}"
        )

    @staticmethod
    def image_to_color(message):
        channels_by_encoding = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}
        encoding = message.encoding.lower()
        if encoding not in channels_by_encoding:
            raise ValueError(f"unsupported color encoding {message.encoding}")
        channels = channels_by_encoding[encoding]
        rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
        image = rows[:, : message.width * channels].reshape(message.height, message.width, channels)
        if encoding in ("bgr8", "bgra8"):
            image = image[:, :, (2, 1, 0)]
        elif channels == 4:
            image = image[:, :, :3]
        elif channels == 1:
            image = image[:, :, 0]
        return np.ascontiguousarray(image)

    @staticmethod
    def image_to_depth(message):
        encoding = message.encoding.lower()
        if encoding in ("16uc1", "mono16"):
            dtype, scale = np.uint16, 1000.0
        elif encoding == "32fc1":
            dtype, scale = np.float32, 1000.0
        else:
            raise ValueError(f"unsupported depth encoding {message.encoding}")
        row_values = message.step // np.dtype(dtype).itemsize
        rows = np.frombuffer(message.data, dtype=dtype).reshape(message.height, row_values)
        depth = np.ascontiguousarray(rows[:, : message.width])
        if dtype == np.float32:
            depth = np.clip(depth * 1000.0, 0.0, 65535.0).astype(np.uint16)
        return depth, scale

    def start_relocalization(self):
        if self.mode != "localization" or self.tracker is None or self.last_color_array is None:
            return False, "waiting for tracker and first RGB-D frame"
        map_path = Path(str(self.get_parameter("map_directory").value)).expanduser()
        if not map_path.is_dir():
            return False, f"cuVSLAM map directory does not exist: {map_path}"
        if (
            bool(self.get_parameter("require_initial_pose").value)
            and not self.initial_pose_received
        ):
            return False, "waiting for /initialpose"
        with self.state_lock:
            if self.relocalization_started:
                return False, "relocalization already running"
            self.relocalization_started = True
            self.relocalized = False
        self.publish_relocalized(False)
        guess_t, guess_q = self.initial_guess
        origin_inverse = inverse_pose(*self.map_origin)
        cuvslam_guess_t, cuvslam_guess_q = compose_pose(
            *origin_inverse, guess_t, guess_q
        )
        guess = self.cuvslam.Pose(
            translation=cuvslam_guess_t, rotation=cuvslam_guess_q
        )
        settings = self.cuvslam.Tracker.SlamLocalizationSettings(
            horizontal_search_radius=float(self.get_parameter("horizontal_search_radius").value),
            vertical_search_radius=float(self.get_parameter("vertical_search_radius").value),
            horizontal_step=float(self.get_parameter("horizontal_step").value),
            vertical_step=float(self.get_parameter("vertical_step").value),
            angular_step_rads=float(self.get_parameter("angular_step_rads").value),
        )
        self.tracker.localize_in_map(
            str(map_path),
            self.last_track_stamp_ns,
            guess,
            [self.last_color_array],
            settings,
            lambda: self.publish_status("relocalizing"),
            self.relocalization_finished,
        )
        return True, "relocalization started"

    def relocalization_finished(self, pose, error_message):
        with self.state_lock:
            self.relocalization_started = False
            self.relocalized = pose is not None
        self.publish_relocalized(self.relocalized)
        if pose is None:
            message = error_message or "no matching map pose"
            self.publish_status(f"relocalization_failed: {message}")
            self.get_logger().error(f"cuVSLAM relocalization failed: {message}")
        else:
            self.publish_status("localized")
            self.get_logger().info(
                "cuVSLAM relocalization succeeded at "
                f"[{pose.translation[0]:.2f}, {pose.translation[1]:.2f}, {pose.translation[2]:.2f}]"
            )

    def publish_pose(self, pose, stamp_ns):
        message = self.pose_message(pose, stamp_ns)
        translation = np.asarray(
            [
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                message.pose.pose.position.z,
            ]
        )
        if self.last_output_translation is not None and self.last_output_stamp_ns is not None:
            dt = (stamp_ns - self.last_output_stamp_ns) / 1e9
            if dt > 0.0:
                velocity = (translation - self.last_output_translation) / dt
                message.twist.twist.linear.x, message.twist.twist.linear.y, message.twist.twist.linear.z = velocity
        self.last_output_translation = translation
        self.last_output_stamp_ns = stamp_ns
        self.odom_publisher.publish(message)

        if bool(self.get_parameter("publish_tf").value):
            transform = TransformStamped()
            transform.header = message.header
            transform.child_frame_id = message.child_frame_id
            transform.transform.translation.x = message.pose.pose.position.x
            transform.transform.translation.y = message.pose.pose.position.y
            transform.transform.translation.z = message.pose.pose.position.z
            transform.transform.rotation = message.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

    def publish_slam_pose(self, pose, stamp_ns):
        self.slam_pose_publisher.publish(self.pose_message(pose, stamp_ns))

    def pose_message(self, pose, stamp_ns):
        translation, quaternion = compose_pose(
            *self.map_origin, pose.translation, pose.rotation
        )
        message = Odometry()
        message.header.stamp.sec = stamp_ns // 1_000_000_000
        message.header.stamp.nanosec = stamp_ns % 1_000_000_000
        message.header.frame_id = str(self.get_parameter("map_frame").value)
        message.child_frame_id = str(self.get_parameter("base_frame").value)
        message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = translation
        (
            message.pose.pose.orientation.x,
            message.pose.pose.orientation.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w,
        ) = quaternion
        message.pose.covariance[0] = message.pose.covariance[7] = message.pose.covariance[14] = 0.02
        message.pose.covariance[21] = message.pose.covariance[28] = message.pose.covariance[35] = 0.01
        return message

    def save_map_service(self, _request, response):
        accepted, message = self.start_map_save()
        response.success = accepted
        response.message = message
        return response

    def start_map_save(self, publish_status=True):
        if self.mode != "mapping":
            return False, "map saving is available only in mapping mode"
        if not bool(self.get_parameter("enable_slam_backend").value):
            return False, "cuVSLAM SLAM backend is disabled"
        if self.tracker is None or self.frame_count < 2:
            return False, "not enough tracked frames to save a map"
        with self.state_lock:
            if self.save_in_progress:
                return False, "map save already running"
            self.save_in_progress = True
        self.map_save_event.clear()
        path = Path(str(self.get_parameter("map_directory").value)).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        if publish_status and rclpy.ok():
            self.publish_status("saving_map")
        self.tracker.save_map(str(path), self.map_save_finished)
        return True, f"saving cuVSLAM map to {path}"

    def map_save_finished(self, success):
        with self.state_lock:
            self.save_in_progress = False
        if success:
            self.last_saved_frame_count = self.frame_count
            self.write_metadata()
            if rclpy.ok():
                self.publish_status("map_saved")
            self.get_logger().info(
                f"cuVSLAM map saved to {self.get_parameter('map_directory').value}"
            )
        else:
            if rclpy.ok():
                self.publish_status("map_save_failed")
            self.get_logger().error("cuVSLAM map save failed")
        self.map_save_event.set()

    def relocalize_service(self, _request, response):
        accepted, message = self.start_relocalization()
        response.success = accepted
        response.message = message
        return response

    def write_metadata(self):
        path = Path(str(self.get_parameter("map_directory").value)).expanduser()
        info = self.camera_info
        metadata = {
            "format": "bxi_head_cuvslam_map_v1",
            "cuvslam_version": str(self.cuvslam.__version__),
            "map_frame": str(self.get_parameter("map_frame").value),
            "base_frame": str(self.get_parameter("base_frame").value),
            "camera": {
                "width": int(info.width),
                "height": int(info.height),
                "fx": float(info.k[0]),
                "fy": float(info.k[4]),
                "cx": float(info.k[2]),
                "cy": float(info.k[5]),
            },
            "rig_from_camera": {
                "translation": list(self._parameter_pose("rig_from_camera")[0]),
                "quaternion": list(self._parameter_pose("rig_from_camera")[1]),
            },
            "map_origin": {
                "translation": list(self.map_origin[0]),
                "quaternion": list(self.map_origin[1]),
            },
            "tracked_frames": self.frame_count,
        }
        (path / "bxi_metadata.json").write_text(
            json.dumps(metadata, indent=2), encoding="utf-8"
        )

    def load_metadata_if_present(self):
        if self.mode != "localization":
            return
        path = Path(str(self.get_parameter("map_directory").value)).expanduser()
        metadata_path = path / "bxi_metadata.json"
        if not metadata_path.is_file():
            self.get_logger().warning(f"No cuVSLAM map metadata at {metadata_path}")
            return
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        origin = metadata.get("map_origin", {})
        if "translation" in origin and "quaternion" in origin:
            self.map_origin = (
                np.asarray(origin["translation"], dtype=np.float64),
                np.asarray(origin["quaternion"], dtype=np.float64),
            )
            if not self.initial_pose_received:
                self.initial_guess = self.map_origin
        camera = metadata.get("camera", {})
        current = (self.camera_info.width, self.camera_info.height, self.camera_info.k[0], self.camera_info.k[4])
        saved = (camera.get("width"), camera.get("height"), camera.get("fx"), camera.get("fy"))
        if any(abs(float(a) - float(b)) > 1e-3 for a, b in zip(current, saved) if b is not None):
            raise RuntimeError(f"camera calibration differs from saved cuVSLAM map: current={current}, saved={saved}")

    def publish_status(self, text):
        message = String()
        message.data = text
        self.status_publisher.publish(message)

    def publish_relocalized(self, value):
        message = Bool()
        message.data = bool(value)
        self.relocalized_publisher.publish(message)

    def destroy_node(self):
        if (
            self.mode == "mapping"
            and bool(self.get_parameter("save_map_on_shutdown").value)
            and bool(self.get_parameter("enable_slam_backend").value)
            and self.tracker is not None
            and self.frame_count >= 2
            and self.frame_count > self.last_saved_frame_count
        ):
            accepted, _ = self.start_map_save(publish_status=False)
            if accepted:
                self.map_save_event.wait(timeout=30.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HeadCuvslamNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            executor.shutdown()
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        if rclpy.ok():
            rclpy.shutdown()
