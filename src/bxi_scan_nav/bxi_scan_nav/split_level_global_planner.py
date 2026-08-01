import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data


class SplitLevelGlobalPlanner(Node):
    def __init__(self):
        super().__init__('split_level_global_planner')
        self.declare_parameter('goal_topic', '/move_base_simple/goal')
        self.declare_parameter('odom_topic', '/simulation/base_footprint/pose')
        self.declare_parameter('path_topic', '/initial_path')
        self.declare_parameter('debug_path_topic', '/split_level_global_path')
        self.declare_parameter('lower_floor_z', 0.8)
        self.declare_parameter('upper_floor_z', 2.0)
        self.declare_parameter('upper_floor_min_x', 11.5)
        self.declare_parameter('stair_lower_x', 7.6)
        self.declare_parameter('stair_upper_x', 11.4)
        self.declare_parameter('stair_y', 2.5)
        self.declare_parameter('stair_step_count', 14)
        self.declare_parameter('goal_z_auto', True)
        self.declare_parameter('same_floor_midpoint', True)

        path_qos = QoSProfile(depth=1)
        path_qos.reliability = ReliabilityPolicy.RELIABLE
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.odom = None
        self.path_pub = self.create_publisher(Path, self.get_parameter('path_topic').value, 10)
        self.debug_pub = self.create_publisher(Path, self.get_parameter('debug_path_topic').value, path_qos)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, self.get_parameter('goal_topic').value, self.goal_callback, 10)
        self.get_logger().info('Split-level 3D global planner ready: goal=%s path=%s' % (
            self.get_parameter('goal_topic').value,
            self.get_parameter('path_topic').value,
        ))

    def odom_callback(self, msg):
        self.odom = msg

    def goal_callback(self, goal):
        if self.odom is None:
            self.get_logger().warn('No odometry yet; ignoring goal')
            return
        path = self.make_path(goal)
        self.path_pub.publish(path)
        self.debug_pub.publish(path)
        self.get_logger().info('Published 3D split-level path with %d waypoints' % len(path.poses))

    def make_path(self, goal):
        start = self.odom.pose.pose.position
        goal_z = self.goal_z(goal)
        start_level = self.level_for_z(start.z)
        goal_level = self.level_for_z(goal_z)

        points = [(start.x, start.y, start.z)]
        if start_level == goal_level:
            if bool(self.get_parameter('same_floor_midpoint').value):
                points.append(((start.x + goal.pose.position.x) * 0.5, (start.y + goal.pose.position.y) * 0.5, goal_z))
            points.append((goal.pose.position.x, goal.pose.position.y, goal_z))
        elif start_level == 'lower':
            points.extend(self.stair_up_points())
            points.append((goal.pose.position.x, goal.pose.position.y, goal_z))
        else:
            points.extend(self.stair_down_points())
            points.append((goal.pose.position.x, goal.pose.position.y, goal_z))

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = goal.header.frame_id or 'world'
        for x, y, z in self.deduplicate(points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(z)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def goal_z(self, goal):
        if not bool(self.get_parameter('goal_z_auto').value) and abs(goal.pose.position.z) > 1e-3:
            return goal.pose.position.z
        upper_min_x = float(self.get_parameter('upper_floor_min_x').value)
        if abs(goal.pose.position.z) > 1e-3:
            return goal.pose.position.z
        if goal.pose.position.x >= upper_min_x:
            return float(self.get_parameter('upper_floor_z').value)
        return float(self.get_parameter('lower_floor_z').value)

    def level_for_z(self, z):
        lower_z = float(self.get_parameter('lower_floor_z').value)
        upper_z = float(self.get_parameter('upper_floor_z').value)
        return 'upper' if abs(z - upper_z) < abs(z - lower_z) else 'lower'

    def stair_up_points(self):
        lower_x = float(self.get_parameter('stair_lower_x').value)
        upper_x = float(self.get_parameter('stair_upper_x').value)
        stair_y = float(self.get_parameter('stair_y').value)
        lower_z = float(self.get_parameter('lower_floor_z').value)
        upper_z = float(self.get_parameter('upper_floor_z').value)
        count = max(2, int(self.get_parameter('stair_step_count').value))
        points = [(lower_x - 0.55, stair_y, lower_z), (lower_x, stair_y, lower_z)]
        for index in range(1, count + 1):
            ratio = index / count
            points.append((lower_x + (upper_x - lower_x) * ratio, stair_y, lower_z + (upper_z - lower_z) * ratio))
        points.append((upper_x + 0.55, stair_y, upper_z))
        return points

    def stair_down_points(self):
        points = self.stair_up_points()
        points.reverse()
        return points

    @staticmethod
    def deduplicate(points):
        result = []
        for point in points:
            if result and math.dist(result[-1], point) < 1e-3:
                continue
            result.append(point)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SplitLevelGlobalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
