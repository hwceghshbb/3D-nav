from communication.msg import MotionCommands
from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class NavigationCommandBridge(Node):
    def __init__(self):
        super().__init__("navigation_command_bridge")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("motion_commands_topic", "motion_commands")
        self.declare_parameter("max_vx", 0.35)
        self.declare_parameter("max_vy", 0.15)
        self.declare_parameter("max_yawdot", 0.55)
        self.declare_parameter("policy_vx_scale", 3.0)
        self.declare_parameter("policy_vy_scale", 2.0)
        self.declare_parameter("policy_yawdot_scale", 2.0)
        self.declare_parameter("height_des", 1.0)
        self.declare_parameter("require_localization_valid", False)
        self.declare_parameter("localization_valid_topic", "/nav/localization_valid")

        self.require_valid = bool(
            self.get_parameter("require_localization_valid").value
        )
        self.localization_valid = not self.require_valid
        self.publisher = self.create_publisher(
            MotionCommands,
            str(self.get_parameter("motion_commands_topic").value),
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self.command_callback,
            10,
        )
        if self.require_valid:
            latched_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                Bool,
                str(self.get_parameter("localization_valid_topic").value),
                self.valid_callback,
                latched_qos,
            )
            self.create_timer(0.05, self.stop_if_invalid)

    def valid_callback(self, message):
        self.localization_valid = bool(message.data)

    def stop_if_invalid(self):
        if not self.localization_valid:
            self.publish_command(0.0, 0.0, 0.0)

    def command_callback(self, message):
        if not self.localization_valid:
            self.publish_command(0.0, 0.0, 0.0)
            return
        self.publish_command(message.linear.x, message.linear.y, message.angular.z)

    def publish_command(self, vx, vy, yawdot):
        command = MotionCommands()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "scan_planner"
        command.vel_des.x = self.scaled(vx, "max_vx", "policy_vx_scale")
        command.vel_des.y = self.scaled(vy, "max_vy", "policy_vy_scale")
        command.height_des = float(self.get_parameter("height_des").value)
        command.yawdot_des = self.scaled(
            yawdot, "max_yawdot", "policy_yawdot_scale"
        )
        self.publisher.publish(command)

    def scaled(self, value, limit_name, scale_name):
        limit = abs(float(self.get_parameter(limit_name).value))
        scale = abs(float(self.get_parameter(scale_name).value))
        scale = scale if scale > 1.0e-6 else 1.0
        return max(-limit, min(limit, float(value))) / scale


def main(args=None):
    rclpy.init(args=args)
    node = NavigationCommandBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
