from copy import deepcopy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from .core import covariance_is_acceptable, quaternion_is_valid


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

        self.rig_ready = False
        self.last_valid_odom_ns = None
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
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
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
        output = deepcopy(message)
        output.header.frame_id = str(self.get_parameter("odom_frame").value)
        output.child_frame_id = str(self.get_parameter("base_frame").value)
        self.publisher.publish(output)
        transform = TransformStamped()
        transform.header = output.header
        transform.child_frame_id = output.child_frame_id
        transform.transform.translation.x = output.pose.pose.position.x
        transform.transform.translation.y = output.pose.pose.position.y
        transform.transform.translation.z = output.pose.pose.position.z
        transform.transform.rotation = output.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)
        self.last_valid_odom_ns = self.get_clock().now().nanoseconds
        self.publish_valid(True)

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
            if valid:
                self.get_logger().info("cuVSLAM localization valid=True")
            else:
                self.get_logger().warning("cuVSLAM localization valid=False")
        self.valid = valid
        message = Bool()
        message.data = valid
        self.valid_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
