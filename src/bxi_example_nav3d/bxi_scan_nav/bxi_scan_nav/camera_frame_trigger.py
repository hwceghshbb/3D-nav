import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header


class CameraFrameTrigger(Node):
    def __init__(self):
        super().__init__("camera_frame_trigger")
        self.declare_parameter("topic", "/simulation/rgbd_frame_trigger")
        self.declare_parameter("publish_hz", 30.0)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(
            Header, str(self.get_parameter("topic").value), qos
        )
        period = 1.0 / max(
            float(self.get_parameter("publish_hz").value), 1.0
        )
        self.timer = self.create_timer(period, self.publish)

    def publish(self):
        message = Header()
        message.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = CameraFrameTrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
