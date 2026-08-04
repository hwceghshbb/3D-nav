from functools import partial
import math

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, JointState
from std_msgs.msg import Bool

from .core import head_lock_error, max_timestamp_delta_ms, stamp_to_nanoseconds


class HeadCameraRigGuard(Node):
    def __init__(self):
        super().__init__("head_camera_rig_guard")
        defaults = {
            "head_color_topic": "/hardware/head_depth_camera/color/image_raw",
            "head_depth_topic": "/hardware/head_depth_camera/depth/image_rect_raw",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.declare_parameter("joint_states_topic", "/hardware/joint_states")
        self.declare_parameter("imu_topic", "/nav/imu")
        self.declare_parameter("ready_topic", "/nav/rig_ready")
        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("rgb_depth_sync_limit_ms", 1.0)
        self.declare_parameter("image_timeout_sec", 0.35)
        self.declare_parameter("imu_timeout_sec", 0.15)
        self.declare_parameter("joint_timeout_sec", 0.35)
        self.declare_parameter("require_imu", True)
        self.declare_parameter("require_head_lock", True)
        self.declare_parameter("head_joint_tolerance_rad", 0.015)
        self.declare_parameter("head_z_target_rad", 0.0)
        self.declare_parameter("head_y_target_rad", 0.0)
        self.declare_parameter("required_good_sets", 10)

        self.image_stamps = {}
        self.image_received_ns = {}
        self.last_pair_stamp = None
        self.last_head_pair_delta_ms = math.inf
        self.last_set_synchronized = False
        self.good_sets = 0
        self.joint_positions = {}
        self.last_joint_received_ns = None
        self.last_imu_received_ns = None
        self.ready = False

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.ready_publisher = self.create_publisher(
            Bool, self.get_parameter("ready_topic").value, latched_qos
        )
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, self.get_parameter("diagnostics_topic").value, 10
        )
        for name in defaults:
            self.create_subscription(
                Image,
                self.get_parameter(name).value,
                partial(self.image_callback, name),
                qos_profile_sensor_data,
            )
        self.create_subscription(
            JointState,
            self.get_parameter("joint_states_topic").value,
            self.joint_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            self.get_parameter("imu_topic").value,
            self.imu_callback,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(0.1, self.evaluate)
        self.publish_ready(False)

    def image_callback(self, name, message):
        self.image_stamps[name] = stamp_to_nanoseconds(message.header.stamp)
        self.image_received_ns[name] = self.get_clock().now().nanoseconds
        color_key = "head_color_topic"
        depth_key = "head_depth_topic"
        if color_key not in self.image_stamps or depth_key not in self.image_stamps:
            return
        pair_delta_ms = max_timestamp_delta_ms(
            [self.image_stamps[color_key], self.image_stamps[depth_key]]
        )
        if pair_delta_ms > float(
            self.get_parameter("rgb_depth_sync_limit_ms").value
        ):
            return
        pair_stamp = max(self.image_stamps[color_key], self.image_stamps[depth_key])
        if self.last_pair_stamp == pair_stamp:
            return
        self.last_pair_stamp = pair_stamp
        self.last_head_pair_delta_ms = pair_delta_ms
        self.last_set_synchronized = True
        self.good_sets += 1

    def joint_callback(self, message):
        self.joint_positions.update(zip(message.name, message.position))
        self.last_joint_received_ns = self.get_clock().now().nanoseconds

    def imu_callback(self, _message):
        self.last_imu_received_ns = self.get_clock().now().nanoseconds

    def publish_ready(self, ready):
        if ready != self.ready:
            self.get_logger().info(f"Head RGB-D input ready={ready}")
        self.ready = ready
        message = Bool()
        message.data = ready
        self.ready_publisher.publish(message)

    def evaluate(self):
        now_ns = self.get_clock().now().nanoseconds
        image_timeout_ns = int(
            float(self.get_parameter("image_timeout_sec").value) * 1e9
        )
        imu_timeout_ns = int(float(self.get_parameter("imu_timeout_sec").value) * 1e9)
        joint_timeout_ns = int(
            float(self.get_parameter("joint_timeout_sec").value) * 1e9
        )
        images_fresh = len(self.image_received_ns) == 2 and all(
            now_ns - received_ns <= image_timeout_ns
            for received_ns in self.image_received_ns.values()
        )
        imu_fresh = not bool(self.get_parameter("require_imu").value) or (
            self.last_imu_received_ns is not None
            and now_ns - self.last_imu_received_ns <= imu_timeout_ns
        )
        targets = {
            "head_z_joint": float(self.get_parameter("head_z_target_rad").value),
            "head_y_joint": float(self.get_parameter("head_y_target_rad").value),
        }
        lock_error = head_lock_error(self.joint_positions, targets)
        tolerance = float(self.get_parameter("head_joint_tolerance_rad").value)
        joints_fresh = (
            self.last_joint_received_ns is not None
            and now_ns - self.last_joint_received_ns <= joint_timeout_ns
        )
        head_locked = not bool(
            self.get_parameter("require_head_lock").value
        ) or (joints_fresh and lock_error is not None and lock_error[1] <= tolerance)
        enough_good_sets = self.good_sets >= int(
            self.get_parameter("required_good_sets").value
        )
        ready = (
            images_fresh
            and imu_fresh
            and head_locked
            and self.last_set_synchronized
            and enough_good_sets
        )
        self.publish_ready(ready)
        self.publish_diagnostics(
            images_fresh, imu_fresh, head_locked, lock_error, enough_good_sets
        )

    def publish_diagnostics(
        self, images_fresh, imu_fresh, head_locked, lock_error, enough_good_sets
    ):
        failures = []
        if not images_fresh:
            failures.append("camera stream timeout")
        if not self.last_set_synchronized:
            failures.append("camera timestamps not synchronized")
        if not enough_good_sets:
            failures.append("waiting for stable synchronized frames")
        if not imu_fresh:
            failures.append("IMU timeout")
        if not head_locked:
            failures.append("head is not locked")

        status = DiagnosticStatus()
        status.name = "ELF3 head RGB-D cuVSLAM input"
        status.hardware_id = "head_rgbd"
        status.level = DiagnosticStatus.OK if not failures else DiagnosticStatus.ERROR
        status.message = "ready" if not failures else "; ".join(failures)
        status.values = [
            KeyValue(
                key="head_rgb_depth_delta_ms",
                value=f"{self.last_head_pair_delta_ms:.3f}",
            ),
            KeyValue(key="good_sets", value=str(self.good_sets)),
            KeyValue(
                key="head_lock_error_rad",
                value="missing" if lock_error is None else f"{lock_error[1]:.6f}",
            ),
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status.append(status)
        self.diagnostics_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = HeadCameraRigGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
