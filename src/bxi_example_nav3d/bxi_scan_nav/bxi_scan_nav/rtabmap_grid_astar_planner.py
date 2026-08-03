import heapq
import math
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RtabmapGridAstarPlanner(Node):
    def __init__(self):
        super().__init__("rtabmap_grid_astar_planner")
        self.declare_parameter("map_topic", "/rtabmap/map")
        self.declare_parameter("odom_topic", "/simulation/odom")
        self.declare_parameter("goal_topic", "/move_base_simple/goal")
        self.declare_parameter("scan_initial_path_topic", "/initial_path")
        self.declare_parameter("debug_path_topic", "/rtabmap_global_path")
        self.declare_parameter("robot_radius", 0.38)
        self.declare_parameter("occupied_threshold", 55)
        self.declare_parameter("allow_unknown", False)
        self.declare_parameter("goal_search_radius", 0.8)
        self.declare_parameter("path_sparsify_distance", 0.35)
        self.declare_parameter("path_z", 0.0)
        self.declare_parameter("max_expansions", 250000)
        self.declare_parameter("debug_inflated_map_topic", "/rtabmap_global_inflated_map")

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.map_msg = None
        self.inflated = None
        self.odom_msg = None
        self.pending_goal = None

        self.path_pub = self.create_publisher(
            Path, self.get_parameter("scan_initial_path_topic").value, 10
        )
        self.debug_path_pub = self.create_publisher(
            Path, self.get_parameter("debug_path_topic").value, map_qos
        )
        self.debug_inflated_map_pub = self.create_publisher(
            OccupancyGrid, self.get_parameter("debug_inflated_map_topic").value, map_qos
        )
        self.create_subscription(
            OccupancyGrid, self.get_parameter("map_topic").value, self.map_callback, map_qos
        )
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.odom_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            PoseStamped, self.get_parameter("goal_topic").value, self.goal_callback, 10
        )
        self.get_logger().info(
            "RTAB-Map grid A* ready: map=%s goal=%s output=%s"
            % (
                self.get_parameter("map_topic").value,
                self.get_parameter("goal_topic").value,
                self.get_parameter("scan_initial_path_topic").value,
            )
        )

    def map_callback(self, msg):
        self.map_msg = msg
        self.inflated = self.build_inflated_grid(msg)
        self.publish_inflated_map(msg)
        if self.pending_goal is not None and self.odom_msg is not None:
            self.plan_to_goal(self.pending_goal)

    def odom_callback(self, msg):
        self.odom_msg = msg

    def goal_callback(self, msg):
        self.pending_goal = msg
        self.plan_to_goal(msg)

    def build_inflated_grid(self, msg):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        allow_unknown = bool(self.get_parameter("allow_unknown").value)
        radius_cells = max(0, int(math.ceil(float(self.get_parameter("robot_radius").value) / resolution)))

        occupied = [False] * (width * height)
        for index, value in enumerate(msg.data):
            if value < 0:
                occupied[index] = not allow_unknown
            elif value >= occupied_threshold:
                occupied[index] = True

        if radius_cells <= 0:
            return occupied

        inflated = occupied[:]
        offsets = []
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy <= radius_cells * radius_cells:
                    offsets.append((dx, dy))

        for index, is_occupied in enumerate(occupied):
            if not is_occupied:
                continue
            x = index % width
            y = index // width
            for dx, dy in offsets:
                nx = x + dx
                ny = y + dy
                if 0 <= nx < width and 0 <= ny < height:
                    inflated[ny * width + nx] = True
        return inflated

    def plan_to_goal(self, goal):
        if self.map_msg is None or self.inflated is None:
            self.get_logger().warn("No RTAB-Map occupancy grid yet; goal cached")
            return
        if self.odom_msg is None:
            self.get_logger().warn("No odometry yet; goal cached")
            return

        start_world = (
            self.odom_msg.pose.pose.position.x,
            self.odom_msg.pose.pose.position.y,
        )
        goal_world = (goal.pose.position.x, goal.pose.position.y)
        start_cell = self.world_to_cell(start_world[0], start_world[1])
        goal_cell = self.world_to_cell(goal_world[0], goal_world[1])
        if start_cell is None or goal_cell is None:
            self.get_logger().error("Start or goal is outside RTAB-Map grid")
            return

        start_cell = self.nearest_free(start_cell, 0.4)
        goal_cell = self.nearest_free(
            goal_cell, float(self.get_parameter("goal_search_radius").value)
        )
        if start_cell is None or goal_cell is None:
            self.get_logger().error("No nearby free start/goal cell in RTAB-Map grid")
            return

        cells = self.astar(start_cell, goal_cell)
        if not cells:
            self.get_logger().error("A* failed on RTAB-Map grid")
            return

        path = self.cells_to_path(self.sparsify(cells))
        self.path_pub.publish(path)
        self.debug_path_pub.publish(path)
        self.pending_goal = None
        self.get_logger().info("Published global path with %d waypoints for SCAN" % len(path.poses))

    def world_to_cell(self, x, y):
        info = self.map_msg.info
        origin = info.origin.position
        yaw = yaw_from_quaternion(info.origin.orientation)
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

    def cell_to_world(self, cell):
        info = self.map_msg.info
        origin = info.origin.position
        yaw = yaw_from_quaternion(info.origin.orientation)
        mx = (cell[0] + 0.5) * info.resolution
        my = (cell[1] + 0.5) * info.resolution
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return (
            origin.x + cos_yaw * mx - sin_yaw * my,
            origin.y + sin_yaw * mx + cos_yaw * my,
        )

    def nearest_free(self, cell, radius_m):
        if self.is_free(cell):
            return cell
        resolution = self.map_msg.info.resolution
        max_radius = max(1, int(math.ceil(radius_m / resolution)))
        queue = deque([(cell[0], cell[1], 0)])
        seen = {cell}
        while queue:
            x, y, distance = queue.popleft()
            if distance > max_radius:
                continue
            if self.is_free((x, y)):
                return x, y
            for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                if not self.in_bounds((nx, ny)) or (nx, ny) in seen:
                    continue
                seen.add((nx, ny))
                queue.append((nx, ny, distance + 1))
        return None

    def astar(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (self.heuristic(start, goal), 0.0, start))
        came_from = {}
        g_score = {start: 0.0}
        visited = set()
        max_expansions = int(self.get_parameter("max_expansions").value)
        expansions = 0

        while open_set and expansions < max_expansions:
            _, cost, current = heapq.heappop(open_set)
            if current in visited:
                continue
            visited.add(current)
            expansions += 1
            if current == goal:
                return self.reconstruct(came_from, current)

            for neighbor, step_cost in self.neighbors(current):
                if neighbor in visited:
                    continue
                next_cost = cost + step_cost
                if next_cost >= g_score.get(neighbor, float("inf")):
                    continue
                came_from[neighbor] = current
                g_score[neighbor] = next_cost
                priority = next_cost + self.heuristic(neighbor, goal)
                heapq.heappush(open_set, (priority, next_cost, neighbor))

        self.get_logger().warn("A* expansions=%d open=%d" % (expansions, len(open_set)))
        return []

    def neighbors(self, cell):
        x, y = cell
        for dx, dy, cost in (
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (1, 1, math.sqrt(2.0)),
        ):
            neighbor = (x + dx, y + dy)
            if dx != 0 and dy != 0:
                if not self.is_free((x + dx, y)) or not self.is_free((x, y + dy)):
                    continue
            if self.is_free(neighbor):
                yield neighbor, cost

    def publish_inflated_map(self, msg):
        if self.inflated is None:
            return
        inflated_msg = OccupancyGrid()
        inflated_msg.header = msg.header
        inflated_msg.info = msg.info
        inflated_msg.data = [100 if occupied else 0 for occupied in self.inflated]
        self.debug_inflated_map_pub.publish(inflated_msg)

    def in_bounds(self, cell):
        return 0 <= cell[0] < self.map_msg.info.width and 0 <= cell[1] < self.map_msg.info.height

    def is_free(self, cell):
        if not self.in_bounds(cell):
            return False
        return not self.inflated[cell[1] * self.map_msg.info.width + cell[0]]

    @staticmethod
    def heuristic(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def reconstruct(came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def sparsify(self, cells):
        if len(cells) <= 2:
            return cells
        min_distance = float(self.get_parameter("path_sparsify_distance").value)
        min_cells = max(1, int(round(min_distance / self.map_msg.info.resolution)))
        sparse = [cells[0]]
        last = cells[0]
        last_dir = None
        for cell in cells[1:-1]:
            direction = (
                max(-1, min(1, cell[0] - last[0])),
                max(-1, min(1, cell[1] - last[1])),
            )
            dist_cells = math.hypot(cell[0] - sparse[-1][0], cell[1] - sparse[-1][1])
            if last_dir is not None and direction != last_dir and dist_cells >= min_cells:
                sparse.append(cell)
            elif dist_cells >= min_cells * 2:
                sparse.append(cell)
            last = cell
            last_dir = direction
        if sparse[-1] != cells[-1]:
            sparse.append(cells[-1])
        return sparse

    def cells_to_path(self, cells):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_msg.header.frame_id or "world"
        path_z = float(self.get_parameter("path_z").value)
        for cell in cells:
            x, y = self.cell_to_world(cell)
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = path_z
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = RtabmapGridAstarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
