import math

import rclpy
import tf2_ros
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker


class ClickedPoint3DGoal(Node):
    def __init__(self):
        super().__init__("clicked_point_3d_goal")
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("goal_topic", "/move_base_simple/goal")
        self.declare_parameter("odom_topic", "/simulation/base_footprint/pose")
        self.declare_parameter("tomogram_topic", "/pct_tomogram_points")
        self.declare_parameter("marker_topic", "/clicked_3d_goal_marker")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("snap_to_tomogram", True)
        self.declare_parameter("snap_xy_radius", 0.80)
        self.declare_parameter("snap_z_weight", 0.35)
        self.declare_parameter("max_tomogram_points", 200000)
        self.declare_parameter("reject_on_tf_failure", False)
        self.declare_parameter("zero_floor_epsilon", 0.06)

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.odom = None
        self.tomogram_points = []
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.goal_pub = self.create_publisher(PoseStamped, self.get_parameter("goal_topic").value, 10)
        self.marker_pub = self.create_publisher(Marker, self.get_parameter("marker_topic").value, latched_qos)
        self.create_subscription(
            PointStamped, self.get_parameter("clicked_point_topic").value, self.clicked_point_callback, 10
        )
        self.create_subscription(
            PointCloud2, self.get_parameter("tomogram_topic").value, self.tomogram_callback, latched_qos
        )
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.odom_callback, qos_profile_sensor_data
        )
        self.get_logger().info(
            "3D clicked goal bridge ready: %s -> %s"
            % (self.get_parameter("clicked_point_topic").value, self.get_parameter("goal_topic").value)
        )

    def odom_callback(self, msg):
        self.odom = msg

    def tomogram_callback(self, msg):
        points = []
        max_points = max(1, int(self.get_parameter("max_tomogram_points").value))
        target_frame = str(self.get_parameter("target_frame").value)
        for index, point in enumerate(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)):
            if index >= max_points:
                break
            points.append((float(point[0]), float(point[1]), float(point[2])))
        if msg.header.frame_id and msg.header.frame_id != target_frame:
            self.get_logger().warn(
                "Tomogram frame is %s, expected %s; snapping will use raw coordinates"
                % (msg.header.frame_id, target_frame)
            )
        self.tomogram_points = points

    def clicked_point_callback(self, msg):
        point = self.point_in_target_frame(msg)
        if point is None:
            return
        raw_point = point
        if bool(self.get_parameter("snap_to_tomogram").value):
            point = self.snap_point(point)

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = str(self.get_parameter("target_frame").value)
        goal.pose.position.x = point[0]
        goal.pose.position.y = point[1]
        goal.pose.position.z = self.goal_z(point[2])
        goal.pose.orientation.z, goal.pose.orientation.w = self.goal_yaw_quaternion(
            goal.pose.position.x, goal.pose.position.y
        )
        self.goal_pub.publish(goal)
        self.publish_marker(goal)
        self.get_logger().info(
            "Published 3D goal: raw=(%.2f, %.2f, %.2f) goal=(%.2f, %.2f, %.2f)"
            % (
                raw_point[0],
                raw_point[1],
                raw_point[2],
                goal.pose.position.x,
                goal.pose.position.y,
                goal.pose.position.z,
            )
        )

    def point_in_target_frame(self, msg):
        target_frame = str(self.get_parameter("target_frame").value)
        source_frame = msg.header.frame_id or target_frame
        point = (msg.point.x, msg.point.y, msg.point.z)
        if source_frame == target_frame:
            return point

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.10),
            )
        except Exception as exc:
            message = "No TF %s -> %s for clicked point: %s" % (source_frame, target_frame, exc)
            if bool(self.get_parameter("reject_on_tf_failure").value):
                self.get_logger().error(message)
                return None
            self.get_logger().warn(message + "; using raw coordinates")
            return point
        return self.apply_transform(point, transform)

    def snap_point(self, point):
        if not self.tomogram_points:
            return point
        radius = float(self.get_parameter("snap_xy_radius").value)
        z_weight = float(self.get_parameter("snap_z_weight").value)
        radius_sq = radius * radius
        best_point = None
        best_score = float("inf")
        for candidate in self.tomogram_points:
            dx = candidate[0] - point[0]
            dy = candidate[1] - point[1]
            xy_sq = dx * dx + dy * dy
            if xy_sq > radius_sq:
                continue
            dz = candidate[2] - point[2]
            score = xy_sq + z_weight * dz * dz
            if score < best_score:
                best_score = score
                best_point = candidate
        return best_point if best_point is not None else point

    def goal_z(self, z):
        epsilon = float(self.get_parameter("zero_floor_epsilon").value)
        if abs(z) < epsilon:
            return epsilon
        return z

    def goal_yaw_quaternion(self, goal_x, goal_y):
        if self.odom is None:
            return 0.0, 1.0
        dx = goal_x - self.odom.pose.pose.position.x
        dy = goal_y - self.odom.pose.pose.position.y
        if math.hypot(dx, dy) < 1e-3:
            return 0.0, 1.0
        yaw = math.atan2(dy, dx)
        return math.sin(yaw * 0.5), math.cos(yaw * 0.5)

    def publish_marker(self, goal):
        marker = Marker()
        marker.header = goal.header
        marker.ns = "clicked_3d_goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = goal.pose
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.35
        marker.color.r = 1.0
        marker.color.g = 0.25
        marker.color.b = 0.05
        marker.color.a = 0.95
        self.marker_pub.publish(marker)

    @staticmethod
    def apply_transform(point, transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w
        x, y, z = point
        rx = (
            (1.0 - 2.0 * (qy * qy + qz * qz)) * x
            + 2.0 * (qx * qy - qz * qw) * y
            + 2.0 * (qx * qz + qy * qw) * z
        )
        ry = (
            2.0 * (qx * qy + qz * qw) * x
            + (1.0 - 2.0 * (qx * qx + qz * qz)) * y
            + 2.0 * (qy * qz - qx * qw) * z
        )
        rz = (
            2.0 * (qx * qz - qy * qw) * x
            + 2.0 * (qy * qz + qx * qw) * y
            + (1.0 - 2.0 * (qx * qx + qy * qy)) * z
        )
        return rx + translation.x, ry + translation.y, rz + translation.z


def main(args=None):
    rclpy.init(args=args)
    node = ClickedPoint3DGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
