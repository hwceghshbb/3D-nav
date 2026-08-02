import math

import nav_msgs.msg
import rclpy
import std_msgs.msg
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class IdealLinePublisher(Node):
    def __init__(self):
        super().__init__("ideal_line_publisher")
        self.declare_parameter("odom_topic", "/simulation/odom")
        self.declare_parameter("line_offset_topic", "/simulation/front_line_camera/line_offset")
        self.declare_parameter("line_state_topic", "/simulation/front_line_camera/line_state")
        self.declare_parameter("line_y", 13.385)
        self.declare_parameter("line_heading", 0.0)
        self.declare_parameter("lane_half_width", 0.60)
        self.declare_parameter("error_sign", 1.0)
        self.declare_parameter("publish_hz", 50.0)

        odom_topic = str(self.get_parameter("odom_topic").value)
        offset_topic = str(self.get_parameter("line_offset_topic").value)
        state_topic = str(self.get_parameter("line_state_topic").value)
        self.line_y = float(self.get_parameter("line_y").value)
        self.line_heading = float(self.get_parameter("line_heading").value)
        self.lane_half_width = max(0.05, abs(float(self.get_parameter("lane_half_width").value)))
        self.error_sign = 1.0 if float(self.get_parameter("error_sign").value) >= 0.0 else -1.0
        publish_hz = max(1.0, float(self.get_parameter("publish_hz").value))

        self.latest_odom = None
        self.publish_count = 0
        self.offset_pub = self.create_publisher(std_msgs.msg.Float32, offset_topic, 10)
        self.state_pub = self.create_publisher(std_msgs.msg.Float32MultiArray, state_topic, 10)
        odom_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_sub = self.create_subscription(
            nav_msgs.msg.Odometry, odom_topic, self.odom_callback, odom_qos
        )
        self.timer = self.create_timer(1.0 / publish_hz, self.timer_callback)
        self.get_logger().info(
            f"ideal line: y={self.line_y:.3f}, heading={self.line_heading:.3f}, "
            f"half_width={self.lane_half_width:.3f}, sign={self.error_sign:.0f}"
        )

    def odom_callback(self, msg):
        self.latest_odom = msg

    def timer_callback(self):
        if self.latest_odom is None:
            return
        pose = self.latest_odom.pose.pose
        yaw = self.quaternion_yaw(pose.orientation)
        lateral_error = self.line_y - pose.position.y
        offset = self.error_sign * lateral_error / self.lane_half_width
        heading_error = self.error_sign * self.wrap_angle(self.line_heading - yaw) / math.pi
        offset = max(-1.0, min(1.0, offset))
        heading_error = max(-1.0, min(1.0, heading_error))
        margin = max(-1.0, min(1.0, 1.0 - abs(offset)))

        offset_msg = std_msgs.msg.Float32()
        offset_msg.data = float(offset)
        self.offset_pub.publish(offset_msg)

        state_msg = std_msgs.msg.Float32MultiArray()
        state_msg.data = [float(offset), float(heading_error), 1.0, float(offset), margin, 2.0, 0.0]
        self.state_pub.publish(state_msg)
        self.publish_count += 1
        if self.publish_count % 100 == 0:
            self.get_logger().info(
                f"ideal error: x={pose.position.x:.3f} y={pose.position.y:.3f} "
                f"yaw={yaw:.3f} offset={offset:.3f}"
            )

    @staticmethod
    def quaternion_yaw(quaternion):
        sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(sin_yaw, cos_yaw)

    @staticmethod
    def wrap_angle(angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    node = IdealLinePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
