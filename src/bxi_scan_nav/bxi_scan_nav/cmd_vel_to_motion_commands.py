import communication.msg as bxi_msg
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelToMotionCommands(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_motion_commands")
        self.declare_parameter("cmd_vel_topic", "/scan_planner/cmd_vel")
        self.declare_parameter("motion_commands_topic", "motion_commands")
        self.declare_parameter("max_vx", 0.35)
        self.declare_parameter("max_vy", 0.15)
        self.declare_parameter("max_yawdot", 0.55)
        self.declare_parameter("policy_vx_scale", 3.0)
        self.declare_parameter("policy_vy_scale", 2.0)
        self.declare_parameter("policy_yawdot_scale", 2.0)
        self.declare_parameter("height_des", 1.0)
        self.publisher = self.create_publisher(
            bxi_msg.MotionCommands,
            self.get_parameter("motion_commands_topic").value,
            10,
        )
        self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            self.cmd_vel_callback,
            10,
        )

    def cmd_vel_callback(self, msg):
        command = bxi_msg.MotionCommands()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "scan_planner"
        vx = self.clamp(msg.linear.x, self.get_parameter("max_vx").value)
        vy = self.clamp(msg.linear.y, self.get_parameter("max_vy").value)
        yawdot = self.clamp(msg.angular.z, self.get_parameter("max_yawdot").value)
        command.vel_des.x = vx / self.scale("policy_vx_scale")
        command.vel_des.y = vy / self.scale("policy_vy_scale")
        command.vel_des.z = 0.0
        command.height_des = float(self.get_parameter("height_des").value)
        command.yawdot_des = yawdot / self.scale("policy_yawdot_scale")
        self.publisher.publish(command)

    @staticmethod
    def clamp(value, limit):
        limit = abs(float(limit))
        return float(max(-limit, min(limit, value)))

    def scale(self, parameter_name):
        value = abs(float(self.get_parameter(parameter_name).value))
        return value if value > 1.0e-6 else 1.0


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToMotionCommands()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
