import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data


class Nav2GlobalPathPlanner(Node):
    def __init__(self):
        super().__init__("nav2_global_path_planner")
        self.declare_parameter("goal_topic", "/move_base_simple/goal")
        self.declare_parameter("odom_topic", "/simulation/base_footprint/pose")
        self.declare_parameter("path_topic", "/nav2_global_path")
        self.declare_parameter("compat_path_topic", "/rtabmap_global_path")
        self.declare_parameter("planner_id", "GridBased")
        self.declare_parameter("action_name", "compute_path_to_pose")
        self.declare_parameter("default_frame_id", "world")

        path_qos = QoSProfile(depth=1)
        path_qos.reliability = ReliabilityPolicy.RELIABLE
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.odom = None
        self.path_pub = self.create_publisher(Path, self.get_parameter("path_topic").value, path_qos)
        self.compat_path_pub = self.create_publisher(
            Path, self.get_parameter("compat_path_topic").value, path_qos
        )
        self.action_client = ActionClient(
            self, ComputePathToPose, self.get_parameter("action_name").value
        )
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.odom_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            PoseStamped, self.get_parameter("goal_topic").value, self.goal_callback, 10
        )
        self.get_logger().info("Nav2 global path planner bridge ready")

    def odom_callback(self, msg):
        self.odom = msg

    def goal_callback(self, msg):
        if self.odom is None:
            self.get_logger().warn("Ignoring goal until base footprint odometry is available")
            return
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 planner action server is not available")
            return

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = self.start_pose()
        goal_msg.goal = self.goal_pose(msg)
        goal_msg.planner_id = str(self.get_parameter("planner_id").value)
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def start_pose(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = str(self.get_parameter("default_frame_id").value)
        pose.pose = self.odom.pose.pose
        pose.pose.position.z = 0.0
        return pose

    def goal_pose(self, msg):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = msg.header.frame_id or str(self.get_parameter("default_frame_id").value)
        pose.pose = msg.pose
        pose.pose.position.z = 0.0
        return pose

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 planner rejected goal")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        path = result.path
        if not path.poses:
            self.get_logger().error("Nav2 returned an empty global path")
            return
        self.path_pub.publish(path)
        self.compat_path_pub.publish(path)
        self.get_logger().info("Published Nav2 global path with %d poses" % len(path.poses))


def main(args=None):
    rclpy.init(args=args)
    node = Nav2GlobalPathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
