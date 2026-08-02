import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class StartRunKeyboard(Node):
    def __init__(self):
        super().__init__("start_run_keyboard")
        self.declare_parameter("start_signal_topic", "/simulation/start_run")
        self.start_signal_topic = self.get_parameter("start_signal_topic").value
        self.pub = self.create_publisher(Bool, self.start_signal_topic, 1)
        self.get_logger().info(
            f"press Enter or 's' to publish start signal on {self.start_signal_topic}; "
            "press 'q' to quit"
        )

    def publish_start(self):
        msg = Bool()
        msg.data = True
        self.pub.publish(msg)
        self.get_logger().info("start signal published")


def read_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)
    node = StartRunKeyboard()
    try:
        while rclpy.ok():
            key = read_key()
            if key in ("\r", "\n", "s", "S"):
                node.publish_start()
                rclpy.spin_once(node, timeout_sec=0.1)
            elif key in ("q", "Q", "\x03"):
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
