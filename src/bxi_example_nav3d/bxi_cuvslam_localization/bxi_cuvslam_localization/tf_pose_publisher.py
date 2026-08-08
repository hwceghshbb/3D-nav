from copy import deepcopy

from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
try:
    from rclpy.exceptions import RCLError
except ImportError:  # ROS 2 Humble exports RCLError from the pybind module.
    from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


class TfPosePublisher(Node):
    def __init__(self):
        super().__init__("tf_pose_publisher")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("tracked_frame", "bxi_base_link")
        self.declare_parameter("output_topic", "/nav/base_footprint/pose")
        self.declare_parameter("valid_topic", "/nav/localization_valid")
        self.declare_parameter("odom_topic", "/nav/odom")
        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("transform_timeout_sec", 0.1)
        self.declare_parameter("copy_odom_twist", True)

        self.localization_valid = False
        self.last_odom = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(
            Odometry, self.get_parameter("output_topic").value, qos_profile_sensor_data
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool,
            self.get_parameter("valid_topic").value,
            self.valid_callback,
            latched_qos,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self.odom_callback,
            qos_profile_sensor_data,
        )
        rate = max(float(self.get_parameter("publish_hz").value), 1.0)
        self.timer = self.create_timer(1.0 / rate, self.publish_pose)

    def valid_callback(self, message):
        self.localization_valid = bool(message.data)

    def odom_callback(self, message):
        self.last_odom = message

    def publish_pose(self):
        if not self.localization_valid:
            return
        target = str(self.get_parameter("target_frame").value)
        tracked = str(self.get_parameter("tracked_frame").value)
        timeout = Duration(
            seconds=float(self.get_parameter("transform_timeout_sec").value)
        )
        try:
            transform = self.buffer.lookup_transform(target, tracked, Time(), timeout)
        except TransformException as error:
            self.get_logger().warning(
                f"Cannot publish {tracked} pose in {target}: {error}",
                throttle_duration_sec=2.0,
            )
            return

        message = Odometry()
        message.header = deepcopy(transform.header)
        message.header.frame_id = target
        message.child_frame_id = tracked
        message.pose.pose.position.x = transform.transform.translation.x
        message.pose.pose.position.y = transform.transform.translation.y
        message.pose.pose.position.z = transform.transform.translation.z
        message.pose.pose.orientation = deepcopy(transform.transform.rotation)
        if bool(self.get_parameter("copy_odom_twist").value) and self.last_odom:
            message.twist = deepcopy(self.last_odom.twist)
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = TfPosePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
