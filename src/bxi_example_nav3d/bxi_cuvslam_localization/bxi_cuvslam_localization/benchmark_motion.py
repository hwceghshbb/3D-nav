from communication.msg import MotionCommands
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class BenchmarkMotion(Node):
    def __init__(self):
        super().__init__("localization_benchmark_motion")
        self.declare_parameter("motion_commands_topic", "motion_commands")
        self.declare_parameter("start_delay_sec", 15.0)
        self.declare_parameter("activate_normal_depth", True)
        self.declare_parameter("activation_time_sec", 8.0)
        self.declare_parameter("linear_speed", 0.22)
        self.declare_parameter("yaw_speed", 0.32)
        self.publisher = self.create_publisher(
            MotionCommands,
            str(self.get_parameter("motion_commands_topic").value),
            10,
        )
        self.started_ns = self.get_clock().now().nanoseconds
        self.create_timer(0.02, self.publish_command)
        self.get_logger().info("Deterministic localization benchmark motion ready")

    def publish_command(self):
        elapsed = (self.get_clock().now().nanoseconds - self.started_ns) / 1e9
        elapsed -= float(self.get_parameter("start_delay_sec").value)
        linear = float(self.get_parameter("linear_speed").value)
        yaw = float(self.get_parameter("yaw_speed").value)
        command = MotionCommands()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "localization_benchmark"
        command.height_des = 1.0
        activation_time = float(self.get_parameter("activation_time_sec").value)
        if (
            bool(self.get_parameter("activate_normal_depth").value)
            and activation_time <= elapsed + float(self.get_parameter("start_delay_sec").value)
            < activation_time + 2.0
        ):
            command.btn_10 = 7
        if elapsed >= 0.0:
            phase = elapsed % 13.0
            if phase < 8.0:
                command.vel_des.x = linear
            else:
                command.yawdot_des = yaw
        self.publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = BenchmarkMotion()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
