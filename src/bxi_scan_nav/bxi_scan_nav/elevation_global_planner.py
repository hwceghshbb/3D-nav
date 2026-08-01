import heapq
import math
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class ElevationCell:
    __slots__ = ("count", "z_min", "z_max", "samples", "height", "occupied", "free")

    def __init__(self):
        self.count = 0
        self.z_min = float("inf")
        self.z_max = -float("inf")
        self.samples = []
        self.height = 0.0
        self.occupied = False
        self.free = False

    def add(self, z, max_samples):
        self.count += 1
        self.z_min = min(self.z_min, z)
        self.z_max = max(self.z_max, z)
        if len(self.samples) < max_samples:
            self.samples.append(z)

    def finalize(self, low_percentile, obstacle_height):
        if not self.samples:
            return
        values = sorted(self.samples)
        index = min(len(values) - 1, max(0, int(round((len(values) - 1) * low_percentile))))
        self.height = values[index]
        self.occupied = (self.z_max - self.height) > obstacle_height
        self.free = not self.occupied


class ElevationGlobalPlanner(Node):
    def __init__(self):
        super().__init__("elevation_global_planner")
        self.declare_parameter("cloud_topic", "/rtabmap/cloud_map")
        self.declare_parameter("odom_topic", "/simulation/base_footprint/pose")
        self.declare_parameter("goal_topic", "/move_base_simple/goal")
        self.declare_parameter("path_topic", "/initial_path")
        self.declare_parameter("debug_path_topic", "/elevation_global_path")
        self.declare_parameter("debug_map_topic", "/elevation_traversability_map")
        self.declare_parameter("resolution", 0.20)
        self.declare_parameter("robot_radius", 0.38)
        self.declare_parameter("path_z_offset", 0.80)
        self.declare_parameter("min_z", -0.20)
        self.declare_parameter("max_z", 3.80)
        self.declare_parameter("min_points_per_cell", 2)
        self.declare_parameter("max_samples_per_cell", 64)
        self.declare_parameter("low_percentile", 0.20)
        self.declare_parameter("obstacle_height", 0.45)
        self.declare_parameter("max_step_height", 0.24)
        self.declare_parameter("max_slope", 0.75)
        self.declare_parameter("unknown_is_blocked", True)
        self.declare_parameter("goal_search_radius", 1.0)
        self.declare_parameter("start_search_radius", 0.8)
        self.declare_parameter("path_sparsify_distance", 0.35)
        self.declare_parameter("max_expansions", 350000)
        self.declare_parameter("sample_stride", 1)
        self.declare_parameter("max_points", 350000)

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.odom = None
        self.pending_goal = None
        self.grid = None
        self.inflated = None
        self.width = 0
        self.height = 0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.frame_id = "world"

        self.path_pub = self.create_publisher(Path, self.get_parameter("path_topic").value, 10)
        self.debug_path_pub = self.create_publisher(Path, self.get_parameter("debug_path_topic").value, latched_qos)
        self.debug_map_pub = self.create_publisher(OccupancyGrid, self.get_parameter("debug_map_topic").value, latched_qos)
        self.create_subscription(PointCloud2, self.get_parameter("cloud_topic").value, self.cloud_callback, latched_qos)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, self.get_parameter("goal_topic").value, self.goal_callback, 10)
        self.get_logger().info(
            "Elevation global planner ready: cloud=%s goal=%s path=%s"
            % (
                self.get_parameter("cloud_topic").value,
                self.get_parameter("goal_topic").value,
                self.get_parameter("path_topic").value,
            )
        )

    def cloud_callback(self, msg):
        points = []
        min_z = float(self.get_parameter("min_z").value)
        max_z = float(self.get_parameter("max_z").value)
        stride = max(1, int(self.get_parameter("sample_stride").value))
        max_points = max(1, int(self.get_parameter("max_points").value))
        for index, point in enumerate(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)):
            if index % stride != 0:
                continue
            x, y, z = float(point[0]), float(point[1]), float(point[2])
            if not (min_z <= z <= max_z):
                continue
            points.append((x, y, z))
            if len(points) >= max_points:
                break

        if not points:
            self.get_logger().warn("Received empty usable elevation cloud")
            return

        self.frame_id = msg.header.frame_id or "world"
        self.build_grid(points)
        self.publish_debug_map()
        if self.pending_goal is not None and self.odom is not None:
            self.plan_to_goal(self.pending_goal)

    def odom_callback(self, msg):
        self.odom = msg

    def goal_callback(self, msg):
        self.pending_goal = msg
        self.plan_to_goal(msg)

    def build_grid(self, points):
        resolution = float(self.get_parameter("resolution").value)
        margin = float(self.get_parameter("goal_search_radius").value) + 1.0
        min_x = min(p[0] for p in points) - margin
        max_x = max(p[0] for p in points) + margin
        min_y = min(p[1] for p in points) - margin
        max_y = max(p[1] for p in points) + margin
        self.origin_x = math.floor(min_x / resolution) * resolution
        self.origin_y = math.floor(min_y / resolution) * resolution
        self.width = max(1, int(math.ceil((max_x - self.origin_x) / resolution)))
        self.height = max(1, int(math.ceil((max_y - self.origin_y) / resolution)))
        self.grid = [ElevationCell() for _ in range(self.width * self.height)]

        max_samples = int(self.get_parameter("max_samples_per_cell").value)
        for x, y, z in points:
            cell = self.world_to_cell_unchecked(x, y)
            if cell is None:
                continue
            self.grid[self.index(cell)].add(z, max_samples)

        min_points = int(self.get_parameter("min_points_per_cell").value)
        low_percentile = float(self.get_parameter("low_percentile").value)
        obstacle_height = float(self.get_parameter("obstacle_height").value)
        for cell in self.grid:
            if cell.count < min_points:
                continue
            cell.finalize(low_percentile, obstacle_height)
        self.inflated = self.build_inflated()
        free_count = sum(1 for index, cell in enumerate(self.grid) if cell.free and not self.inflated[index])
        self.get_logger().info("Elevation map updated: %dx%d free=%d" % (self.width, self.height, free_count))

    def build_inflated(self):
        inflated = [False] * (self.width * self.height)
        resolution = float(self.get_parameter("resolution").value)
        radius = float(self.get_parameter("robot_radius").value)
        radius_cells = max(0, int(math.ceil(radius / resolution)))
        offsets = []
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy <= radius_cells * radius_cells:
                    offsets.append((dx, dy))

        for y in range(self.height):
            for x in range(self.width):
                index = self.index((x, y))
                cell = self.grid[index]
                if not cell.occupied:
                    continue
                for dx, dy in offsets:
                    nx = x + dx
                    ny = y + dy
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        inflated[self.index((nx, ny))] = True
        return inflated

    def plan_to_goal(self, goal):
        if self.grid is None or self.inflated is None:
            self.get_logger().warn("No elevation map yet; goal cached")
            return
        if self.odom is None:
            self.get_logger().warn("No odometry yet; goal cached")
            return

        start = self.nearest_free(
            self.world_to_cell(self.odom.pose.pose.position.x, self.odom.pose.pose.position.y),
            float(self.get_parameter("start_search_radius").value),
        )
        goal_cell = self.nearest_free(
            self.world_to_cell(goal.pose.position.x, goal.pose.position.y),
            float(self.get_parameter("goal_search_radius").value),
        )
        if start is None or goal_cell is None:
            self.get_logger().error("No traversable start/goal on elevation map")
            return

        cells = self.astar(start, goal_cell)
        if not cells:
            self.get_logger().error("Elevation A* failed")
            return

        path = self.cells_to_path(self.sparsify(cells))
        self.path_pub.publish(path)
        self.debug_path_pub.publish(path)
        self.pending_goal = None
        self.get_logger().info("Published elevation global path with %d waypoints" % len(path.poses))

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
            if current == goal:
                return self.reconstruct(came_from, current)
            visited.add(current)
            expansions += 1
            for neighbor, step_cost in self.neighbors(current):
                if neighbor in visited:
                    continue
                next_cost = cost + step_cost
                if next_cost >= g_score.get(neighbor, float("inf")):
                    continue
                came_from[neighbor] = current
                g_score[neighbor] = next_cost
                heapq.heappush(open_set, (next_cost + self.heuristic(neighbor, goal), next_cost, neighbor))
        self.get_logger().warn("Elevation A* expansions=%d open=%d" % (expansions, len(open_set)))
        return []

    def neighbors(self, cell):
        x, y = cell
        current_height = self.grid[self.index(cell)].height
        resolution = float(self.get_parameter("resolution").value)
        max_step = float(self.get_parameter("max_step_height").value)
        max_slope = float(self.get_parameter("max_slope").value)
        for dx, dy, distance in (
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
                if not self.is_traversable((x + dx, y)) or not self.is_traversable((x, y + dy)):
                    continue
            if not self.is_traversable(neighbor):
                continue
            neighbor_height = self.grid[self.index(neighbor)].height
            height_delta = abs(neighbor_height - current_height)
            travel_distance = distance * resolution
            if height_delta > max(max_step, max_slope * travel_distance):
                continue
            climb_cost = 2.5 * max(0.0, neighbor_height - current_height)
            yield neighbor, distance + climb_cost

    def nearest_free(self, cell, radius_m):
        if cell is None:
            return None
        if self.is_traversable(cell):
            return cell
        resolution = float(self.get_parameter("resolution").value)
        max_radius = max(1, int(math.ceil(radius_m / resolution)))
        queue = deque([(cell[0], cell[1], 0)])
        seen = {cell}
        while queue:
            x, y, distance = queue.popleft()
            if distance > max_radius:
                continue
            if self.is_traversable((x, y)):
                return x, y
            for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                if not self.in_bounds((nx, ny)) or (nx, ny) in seen:
                    continue
                seen.add((nx, ny))
                queue.append((nx, ny, distance + 1))
        return None

    def publish_debug_map(self):
        if self.grid is None:
            return
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.info.resolution = float(self.get_parameter("resolution").value)
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0
        data = []
        for index, cell in enumerate(self.grid):
            if not cell.free:
                data.append(-1 if not cell.occupied else 100)
            elif self.inflated[index]:
                data.append(100)
            else:
                data.append(0)
        msg.data = data
        self.debug_map_pub.publish(msg)

    def cells_to_path(self, cells):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        offset = float(self.get_parameter("path_z_offset").value)
        for cell in cells:
            x, y = self.cell_to_world(cell)
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = self.grid[self.index(cell)].height + offset
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def sparsify(self, cells):
        if len(cells) <= 2:
            return cells
        resolution = float(self.get_parameter("resolution").value)
        min_cells = max(1, int(round(float(self.get_parameter("path_sparsify_distance").value) / resolution)))
        sparse = [cells[0]]
        last_direction = None
        for cell in cells[1:-1]:
            previous = sparse[-1]
            direction = (
                max(-1, min(1, cell[0] - previous[0])),
                max(-1, min(1, cell[1] - previous[1])),
            )
            distance = math.hypot(cell[0] - previous[0], cell[1] - previous[1])
            if direction != last_direction and distance >= min_cells:
                sparse.append(cell)
            elif distance >= min_cells * 2:
                sparse.append(cell)
            last_direction = direction
        if sparse[-1] != cells[-1]:
            sparse.append(cells[-1])
        return sparse

    def world_to_cell(self, x, y):
        cell = self.world_to_cell_unchecked(x, y)
        if cell is not None and self.in_bounds(cell):
            return cell
        return None

    def world_to_cell_unchecked(self, x, y):
        resolution = float(self.get_parameter("resolution").value)
        return int(math.floor((x - self.origin_x) / resolution)), int(math.floor((y - self.origin_y) / resolution))

    def cell_to_world(self, cell):
        resolution = float(self.get_parameter("resolution").value)
        return self.origin_x + (cell[0] + 0.5) * resolution, self.origin_y + (cell[1] + 0.5) * resolution

    def in_bounds(self, cell):
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height

    def index(self, cell):
        return cell[1] * self.width + cell[0]

    def is_traversable(self, cell):
        if not self.in_bounds(cell):
            return False
        index = self.index(cell)
        return self.grid[index].free and not self.inflated[index]

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


def main(args=None):
    rclpy.init(args=args)
    node = ElevationGlobalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
