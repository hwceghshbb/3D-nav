import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data


class GlobalPathLocalTarget(Node):
    def __init__(self):
        super().__init__("global_path_local_target")
        self.declare_parameter("global_path_topic", "/rtabmap_global_path")
        self.declare_parameter("odom_topic", "/simulation/base_footprint/pose")
        self.declare_parameter("local_path_topic", "/initial_path")
        self.declare_parameter("debug_path_topic", "/scan_planner/local_target_path")
        self.declare_parameter("lookahead_distance", 2.0)
        self.declare_parameter("goal_tolerance", 0.35)
        self.declare_parameter("publish_hz", 8.0)
        self.declare_parameter("min_publish_target_shift", 0.15)
        self.declare_parameter("costmap_topic", "/global_costmap/costmap")
        self.declare_parameter("require_line_of_sight", True)
        self.declare_parameter("occupied_threshold", 80)

        path_qos = QoSProfile(depth=1)
        path_qos.reliability = ReliabilityPolicy.RELIABLE
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.path = None
        self.odom = None
        self.costmap = None
        self.last_target_xy = None
        self.last_blocked_log_time = 0.0
        self.local_pub = self.create_publisher(
            Path, self.get_parameter("local_path_topic").value, 10
        )
        self.debug_pub = self.create_publisher(
            Path, self.get_parameter("debug_path_topic").value, path_qos
        )
        self.create_subscription(
            Path, self.get_parameter("global_path_topic").value, self.path_callback, path_qos
        )
        self.create_subscription(
            OccupancyGrid, self.get_parameter("costmap_topic").value, self.costmap_callback, path_qos
        )
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.odom_callback, qos_profile_sensor_data
        )
        publish_hz = max(float(self.get_parameter("publish_hz").value), 1.0)
        self.timer = self.create_timer(1.0 / publish_hz, self.publish_local_target)
        self.get_logger().info(
            "Global path local target ready: %s -> %s"
            % (
                self.get_parameter("global_path_topic").value,
                self.get_parameter("local_path_topic").value,
            )
        )

    def path_callback(self, msg):
        self.path = msg if msg.poses else None
        self.last_target_xy = None

    def costmap_callback(self, msg):
        self.costmap = msg

    def odom_callback(self, msg):
        self.odom = msg

    def publish_local_target(self):
        if self.path is None or self.odom is None or not self.path.poses:
            return

        robot_xy = (
            self.odom.pose.pose.position.x,
            self.odom.pose.pose.position.y,
        )
        closest_index = self.closest_path_index(robot_xy)
        if closest_index is None:
            return

        raw_target_index = self.lookahead_index(closest_index)
        target_index = self.line_of_sight_target_index(robot_xy, closest_index, raw_target_index)
        if target_index is None or target_index <= closest_index:
            self.publish_hold_path(robot_xy)
            return
        target_pose = self.path.poses[target_index]
        if (
            target_index == len(self.path.poses) - 1
            and self.distance_xy(robot_xy, target_pose) < float(self.get_parameter("goal_tolerance").value)
        ):
            return
        target_xy = (target_pose.pose.position.x, target_pose.pose.position.y)
        if self.last_target_xy is not None:
            shift = math.hypot(target_xy[0] - self.last_target_xy[0], target_xy[1] - self.last_target_xy[1])
            if shift < float(self.get_parameter("min_publish_target_shift").value):
                return
        self.last_target_xy = target_xy

        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.path.header.frame_id or "world"
        for source_pose in self.path.poses[closest_index : target_index + 1]:
            local_pose = PoseStamped()
            local_pose.header = msg.header
            local_pose.pose = source_pose.pose
            msg.poses.append(local_pose)
        self.local_pub.publish(msg)
        self.debug_pub.publish(msg)

    def publish_hold_path(self, robot_xy):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.path.header.frame_id or "world"

        hold_pose = PoseStamped()
        hold_pose.header = msg.header
        hold_pose.pose.position.x = robot_xy[0]
        hold_pose.pose.position.y = robot_xy[1]
        hold_pose.pose.position.z = self.odom.pose.pose.position.z
        hold_pose.pose.orientation = self.odom.pose.pose.orientation
        msg.poses.append(hold_pose)

        self.local_pub.publish(msg)
        self.debug_pub.publish(msg)

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_blocked_log_time > 2.0:
            self.get_logger().warn(
                "Global costmap blocks the local target; publishing hold path instead of crossing an obstacle"
            )
            self.last_blocked_log_time = now

    def closest_path_index(self, robot_xy):
        best_index = None
        best_distance = float("inf")
        for index, pose in enumerate(self.path.poses):
            distance = self.distance_xy(robot_xy, pose)
            if distance < best_distance:
                best_distance = distance
                best_index = index
        return best_index

    def lookahead_index(self, start_index):
        lookahead = max(float(self.get_parameter("lookahead_distance").value), 0.2)
        distance = 0.0
        previous_pose = self.path.poses[start_index]
        for index in range(start_index + 1, len(self.path.poses)):
            current_pose = self.path.poses[index]
            distance += math.hypot(
                current_pose.pose.position.x - previous_pose.pose.position.x,
                current_pose.pose.position.y - previous_pose.pose.position.y,
            )
            if distance >= lookahead:
                return index
            previous_pose = current_pose
        return len(self.path.poses) - 1

    def line_of_sight_target_index(self, robot_xy, closest_index, target_index):
        if not bool(self.get_parameter("require_line_of_sight").value):
            return target_index
        if self.costmap is None:
            return None
        if target_index <= closest_index:
            return None
        for index in range(target_index, closest_index, -1):
            target = self.path.poses[index].pose.position
            if self.line_is_free(robot_xy, (target.x, target.y)):
                return index
        return None

    def line_is_free(self, start_xy, end_xy):
        start_cell = self.world_to_cell(*start_xy)
        end_cell = self.world_to_cell(*end_xy)
        if start_cell is None or end_cell is None:
            return False
        for cell in self.raytrace(start_cell, end_cell):
            if self.cell_is_occupied(cell):
                return False
        return True

    def world_to_cell(self, x, y):
        info = self.costmap.info
        origin = info.origin.position
        yaw = self.yaw_from_quaternion(info.origin.orientation)
        dx = x - origin.x
        dy = y - origin.y
        cos_yaw = math.cos(-yaw)
        sin_yaw = math.sin(-yaw)
        mx = cos_yaw * dx - sin_yaw * dy
        my = sin_yaw * dx + cos_yaw * dy
        cx = int(math.floor(mx / info.resolution))
        cy = int(math.floor(my / info.resolution))
        if 0 <= cx < info.width and 0 <= cy < info.height:
            return cx, cy
        return None

    def cell_is_occupied(self, cell):
        index = cell[1] * self.costmap.info.width + cell[0]
        value = self.costmap.data[index]
        return value < 0 or value >= int(self.get_parameter("occupied_threshold").value)

    @staticmethod
    def raytrace(start, end):
        x0, y0 = start
        x1, y1 = end
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        step_x = 1 if x0 < x1 else -1
        step_y = 1 if y0 < y1 else -1
        error = dx - dy
        x, y = x0, y0
        while True:
            yield x, y
            if x == x1 and y == y1:
                break
            double_error = 2 * error
            if double_error > -dy:
                error -= dy
                x += step_x
            if double_error < dx:
                error += dx
                y += step_y

    @staticmethod
    def yaw_from_quaternion(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    @staticmethod
    def distance_xy(robot_xy, pose):
        return math.hypot(
            pose.pose.position.x - robot_xy[0],
            pose.pose.position.y - robot_xy[1],
        )


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPathLocalTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
