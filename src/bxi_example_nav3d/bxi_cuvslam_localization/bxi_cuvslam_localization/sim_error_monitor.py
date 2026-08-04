from collections import deque
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from std_msgs.msg import Bool

from .core import pose_error, stamp_to_nanoseconds


def pose_values(message):
    pose = message.pose.pose
    return (
        [pose.position.x, pose.position.y, pose.position.z],
        [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ],
    )


class CuvslamSimErrorMonitor(Node):
    def __init__(self):
        super().__init__("cuvslam_sim_error_monitor")
        self.declare_parameter(
            "estimate_topic", "/visual_slam/tracking/odometry"
        )
        self.declare_parameter("truth_topic", "/simulation/odom")
        self.declare_parameter("alarm_topic", "/nav/cuvslam_large_error")
        self.declare_parameter("translation_threshold_m", 0.50)
        self.declare_parameter("rotation_threshold_deg", 20.0)
        self.declare_parameter("max_stamp_delta_ms", 25.0)
        self.declare_parameter("truth_sample_hz", 60.0)
        self.declare_parameter("required_consecutive_samples", 5)
        self.declare_parameter("warmup_samples", 30)
        self.declare_parameter(
            "record_path", "/tmp/cuvslam_large_error_events.jsonl"
        )

        self.truth_buffer = deque(maxlen=300)
        self.valid_samples = 0
        self.consecutive_large_errors = 0
        self.recorded = False
        self.last_truth_sample_ns = None
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.alarm_publisher = self.create_publisher(
            Bool, str(self.get_parameter("alarm_topic").value), latched_qos
        )
        self.truth_subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter("truth_topic").value),
            self.truth_callback,
            qos_profile_sensor_data,
            raw=True,
        )
        self.estimate_subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter("estimate_topic").value),
            self.estimate_callback,
            qos_profile_sensor_data,
        )
        self.publish_alarm(False)
        self.get_logger().info(
            "cuVSLAM simulation error monitor ready: translation>=%.2fm, "
            "rotation>=%.1fdeg, consecutive=%d"
            % (
                float(self.get_parameter("translation_threshold_m").value),
                float(self.get_parameter("rotation_threshold_deg").value),
                int(self.get_parameter("required_consecutive_samples").value),
            )
        )

    def truth_callback(self, serialized):
        if self.recorded:
            return
        now_ns = time.monotonic_ns()
        sample_period_ns = int(
            1e9 / max(1.0, float(self.get_parameter("truth_sample_hz").value))
        )
        if (
            self.last_truth_sample_ns is not None
            and now_ns - self.last_truth_sample_ns < sample_period_ns
        ):
            return
        self.last_truth_sample_ns = now_ns
        message = deserialize_message(serialized, Odometry)
        position, quaternion = pose_values(message)
        self.truth_buffer.append(
            (
                stamp_to_nanoseconds(message.header.stamp),
                message.header.frame_id,
                position,
                quaternion,
            )
        )

    def estimate_callback(self, estimate):
        if self.recorded or not self.truth_buffer:
            return
        estimate_stamp_ns = stamp_to_nanoseconds(estimate.header.stamp)
        truth_stamp_ns, truth_frame, truth_position, truth_quaternion = min(
            self.truth_buffer, key=lambda item: abs(item[0] - estimate_stamp_ns)
        )
        stamp_delta_ms = abs(truth_stamp_ns - estimate_stamp_ns) / 1e6
        if stamp_delta_ms > float(self.get_parameter("max_stamp_delta_ms").value):
            return

        self.valid_samples += 1
        if self.valid_samples <= int(self.get_parameter("warmup_samples").value):
            return
        estimate_position, estimate_quaternion = pose_values(estimate)
        translation_error, rotation_error_rad = pose_error(
            estimate_position,
            estimate_quaternion,
            truth_position,
            truth_quaternion,
        )
        rotation_error_deg = math.degrees(rotation_error_rad)
        large_error = (
            translation_error
            >= float(self.get_parameter("translation_threshold_m").value)
            or rotation_error_deg
            >= float(self.get_parameter("rotation_threshold_deg").value)
        )
        self.consecutive_large_errors = (
            self.consecutive_large_errors + 1 if large_error else 0
        )
        if self.consecutive_large_errors < int(
            self.get_parameter("required_consecutive_samples").value
        ):
            return

        event = {
            "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
            "estimate_stamp_ns": estimate_stamp_ns,
            "truth_stamp_ns": truth_stamp_ns,
            "stamp_delta_ms": stamp_delta_ms,
            "translation_error_m": translation_error,
            "rotation_error_deg": rotation_error_deg,
            "consecutive_large_error_samples": self.consecutive_large_errors,
            "thresholds": {
                "translation_m": float(
                    self.get_parameter("translation_threshold_m").value
                ),
                "rotation_deg": float(
                    self.get_parameter("rotation_threshold_deg").value
                ),
            },
            "estimate": {
                "frame_id": estimate.header.frame_id,
                "position": estimate_position,
                "quaternion_xyzw": estimate_quaternion,
            },
            "truth": {
                "frame_id": truth_frame,
                "position": truth_position,
                "quaternion_xyzw": truth_quaternion,
            },
        }
        self.record_event(event)

    def record_event(self, event):
        path = Path(str(self.get_parameter("record_path").value)).expanduser()
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(event, ensure_ascii=True) + "\n")
        except OSError as error:
            self.get_logger().error(f"Cannot record cuVSLAM error event: {error}")
            return
        self.recorded = True
        self.truth_buffer.clear()
        self.publish_alarm(True)
        self.get_logger().error(
            "Large cuVSLAM localization error recorded once: "
            "translation=%.3fm rotation=%.2fdeg stamp_delta=%.2fms path=%s"
            % (
                event["translation_error_m"],
                event["rotation_error_deg"],
                event["stamp_delta_ms"],
                path,
            )
        )
        self.destroy_subscription(self.truth_subscription)
        self.destroy_subscription(self.estimate_subscription)
        self.truth_subscription = None
        self.estimate_subscription = None

    def publish_alarm(self, value):
        message = Bool()
        message.data = value
        self.alarm_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = CuvslamSimErrorMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        if rclpy.ok():
            rclpy.shutdown()
