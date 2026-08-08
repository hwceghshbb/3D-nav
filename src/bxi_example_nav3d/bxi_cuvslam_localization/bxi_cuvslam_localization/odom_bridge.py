from copy import deepcopy
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from .core import (
    covariance_is_acceptable,
    flat_floor_pose_is_plausible,
    gravity_tilt_error,
    initial_pose_alignment,
    pose_increment_is_plausible,
    quaternion_is_valid,
    stamp_to_nanoseconds,
    transform_pose,
)


class LocalizationOdomBridge(Node):
    def __init__(self):
        super().__init__("localization_odom_bridge")
        self.declare_parameter("input_odom_topic", "/visual_slam/tracking/odometry")
        self.declare_parameter("rig_ready_topic", "/nav/rig_ready")
        self.declare_parameter("output_odom_topic", "/nav/odom")
        self.declare_parameter("valid_topic", "/nav/localization_valid")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "bxi_base_link")
        self.declare_parameter("odom_timeout_sec", 0.25)
        self.declare_parameter("max_position_variance", 0.25)
        self.declare_parameter("max_orientation_variance", 0.25)
        self.declare_parameter("require_covariance", False)
        self.declare_parameter("require_rig_ready", True)
        self.declare_parameter("backend_name", "cuvslam")
        self.declare_parameter("anchor_initial_pose", False)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("initial_base_position", [0.0, 0.0, 1.1])
        self.declare_parameter("check_pose_continuity", True)
        self.declare_parameter("max_translation_jump_m", 0.75)
        self.declare_parameter("max_rotation_jump_rad", 0.70)
        self.declare_parameter("max_linear_speed_mps", 2.0)
        self.declare_parameter("max_angular_speed_rps", 3.0)
        self.declare_parameter("recovery_stable_samples", 5)
        self.declare_parameter("allow_origin_reset_recovery", False)
        self.declare_parameter("enforce_flat_floor", False)
        self.declare_parameter("max_floor_z_drift_m", 0.12)
        self.declare_parameter("max_floor_tilt_rad", 0.20)

        self.rig_ready = not bool(
            self.get_parameter("require_rig_ready").value
        )
        self.last_valid_odom_ns = None
        self.initial_alignment = None
        self.last_accepted_pose = None
        self.recovery_pose = None
        self.recovery_samples = 0
        self.floor_reference_pose = None
        self.valid = False
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            Odometry,
            self.get_parameter("output_odom_topic").value,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.valid_publisher = self.create_publisher(
            Bool, self.get_parameter("valid_topic").value, latched_qos
        )
        self.tf_broadcaster = (
            TransformBroadcaster(self)
            if bool(self.get_parameter("publish_tf").value)
            else None
        )
        self.ready_subscription = None
        if bool(self.get_parameter("require_rig_ready").value):
            self.ready_subscription = self.create_subscription(
                Bool,
                self.get_parameter("rig_ready_topic").value,
                self.ready_callback,
                latched_qos,
            )
        self.create_subscription(
            Odometry,
            self.get_parameter("input_odom_topic").value,
            self.odom_callback,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(0.05, self.watchdog)
        self.publish_valid(False)

    def ready_callback(self, message):
        self.rig_ready = bool(message.data)
        if not self.rig_ready:
            self.publish_valid(False)

    def odom_callback(self, message):
        if not self.rig_ready or not self.message_is_valid(message):
            self.publish_valid(False)
            return
        if not self.pose_is_continuous(message):
            self.publish_valid(False)
            return
        if not self.pose_is_within_floor_bounds(message):
            self.publish_valid(False)
            return
        output = deepcopy(message)
        output.header.frame_id = str(self.get_parameter("odom_frame").value)
        output.child_frame_id = str(self.get_parameter("base_frame").value)
        if bool(self.get_parameter("anchor_initial_pose").value):
            self.apply_initial_pose_anchor(output)
        self.publisher.publish(output)
        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = output.header
            transform.child_frame_id = output.child_frame_id
            transform.transform.translation.x = output.pose.pose.position.x
            transform.transform.translation.y = output.pose.pose.position.y
            transform.transform.translation.z = output.pose.pose.position.z
            transform.transform.rotation = output.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)
        self.last_valid_odom_ns = self.get_clock().now().nanoseconds
        self.last_accepted_pose = self.pose_sample(message)
        self.recovery_pose = None
        self.recovery_samples = 0
        self.publish_valid(True)

    def pose_is_within_floor_bounds(self, message):
        if not bool(self.get_parameter("enforce_flat_floor").value):
            return True
        current = self.pose_sample(message)
        if self.floor_reference_pose is None:
            self.floor_reference_pose = current
            return True
        plausible = flat_floor_pose_is_plausible(
            self.floor_reference_pose[0],
            self.floor_reference_pose[1],
            current[0],
            current[1],
            float(self.get_parameter("max_floor_z_drift_m").value),
            float(self.get_parameter("max_floor_tilt_rad").value),
        )
        if not plausible:
            z_drift = abs(current[0][2] - self.floor_reference_pose[0][2])
            tilt = gravity_tilt_error(
                self.floor_reference_pose[1], current[1]
            )
            self.get_logger().error(
                "Rejected flat-floor localization drift: "
                f"z={z_drift:.3f}m tilt={math.degrees(tilt):.1f}deg; "
                "odom, TF and mapping input are now blocked",
                throttle_duration_sec=1.0,
            )
        return plausible

    @staticmethod
    def pose_sample(message):
        pose = message.pose.pose
        return (
            (pose.position.x, pose.position.y, pose.position.z),
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
            stamp_to_nanoseconds(message.header.stamp),
        )

    def samples_are_continuous(self, previous, current):
        return pose_increment_is_plausible(
            *previous,
            *current,
            float(self.get_parameter("max_translation_jump_m").value),
            float(self.get_parameter("max_rotation_jump_rad").value),
            float(self.get_parameter("max_linear_speed_mps").value),
            float(self.get_parameter("max_angular_speed_rps").value),
        )

    def pose_is_continuous(self, message):
        if not bool(self.get_parameter("check_pose_continuity").value):
            return True
        current = self.pose_sample(message)
        if current[2] <= 0:
            return False
        if self.last_accepted_pose is None:
            return True
        if self.samples_are_continuous(self.last_accepted_pose, current):
            return True

        if self.recovery_pose is None or not self.samples_are_continuous(
            self.recovery_pose, current
        ):
            self.recovery_samples = 1
        else:
            self.recovery_samples += 1
        self.recovery_pose = current
        required = max(
            1, int(self.get_parameter("recovery_stable_samples").value)
        )
        self.get_logger().warning(
            "Rejected localization pose discontinuity; "
            f"stable recovery samples={self.recovery_samples}/{required}",
            throttle_duration_sec=1.0,
        )
        if self.recovery_samples < required:
            return False

        if not bool(
            self.get_parameter("allow_origin_reset_recovery").value
        ):
            self.get_logger().error(
                "Localization is stable in a different origin, but automatic "
                "origin reset recovery is disabled; waiting for relocalization "
                "into the existing map",
                throttle_duration_sec=1.0,
            )
            return False

        self.get_logger().warning(
            "Accepting a new localization origin after stable recovery"
        )
        self.last_accepted_pose = current
        self.recovery_pose = None
        self.recovery_samples = 0
        return True

    def apply_initial_pose_anchor(self, output):
        pose = output.pose.pose
        position = (pose.position.x, pose.position.y, pose.position.z)
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if self.initial_alignment is None:
            target = tuple(
                float(value)
                for value in self.get_parameter("initial_base_position").value
            )
            if len(target) != 3:
                raise ValueError("initial_base_position must contain x, y, z")
            self.initial_alignment = initial_pose_alignment(
                position, quaternion, target
            )
            self.get_logger().info(
                "Anchored first base pose at "
                f"[{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}]"
            )
        position, quaternion = transform_pose(
            *self.initial_alignment, position, quaternion
        )
        pose.position.x, pose.position.y, pose.position.z = position
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = quaternion

    def message_is_valid(self, message):
        position = message.pose.pose.position
        twist = message.twist.twist
        scalars = (
            position.x,
            position.y,
            position.z,
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
        )
        orientation = message.pose.pose.orientation
        if not all(math.isfinite(value) for value in scalars):
            return False
        if not quaternion_is_valid(
            (orientation.x, orientation.y, orientation.z, orientation.w)
        ):
            return False
        if bool(self.get_parameter("require_covariance").value):
            return covariance_is_acceptable(
                message.pose.covariance,
                float(self.get_parameter("max_position_variance").value),
                float(self.get_parameter("max_orientation_variance").value),
            )
        return True

    def watchdog(self):
        if self.last_valid_odom_ns is None:
            self.publish_valid(False)
            return
        age_sec = (
            self.get_clock().now().nanoseconds - self.last_valid_odom_ns
        ) / 1e9
        if age_sec > float(self.get_parameter("odom_timeout_sec").value):
            self.publish_valid(False)

    def publish_valid(self, valid):
        if valid != self.valid:
            backend = str(self.get_parameter("backend_name").value)
            if valid:
                self.get_logger().info(f"{backend} localization valid=True")
            else:
                self.get_logger().warning(f"{backend} localization valid=False")
        self.valid = valid
        message = Bool()
        message.data = valid
        self.valid_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationOdomBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
