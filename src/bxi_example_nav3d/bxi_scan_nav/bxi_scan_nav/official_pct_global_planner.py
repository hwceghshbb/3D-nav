import math
import os
import sys
from types import SimpleNamespace

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class OfficialPctGlobalPlanner(Node):
    def __init__(self):
        super().__init__("pct_global_planner")
        self.declare_parameter("pct_root", "")
        self.declare_parameter("cloud_topic", "/rtabmap/cloud_map")
        self.declare_parameter("odom_topic", "/simulation/base/pose")
        self.declare_parameter("localization_pose_topic", "/rtabmap/localization_pose")
        self.declare_parameter("use_localization_pose", True)
        self.declare_parameter("goal_topic", "/move_base_simple/goal")
        self.declare_parameter("path_topic", "/initial_path")
        self.declare_parameter("debug_path_topic", "/pct_global_path")
        self.declare_parameter("debug_map_topic", "/pct_traversability_map")
        self.declare_parameter("tomogram_topic", "/pct_tomogram_points")
        self.declare_parameter("resolution", 0.10)
        self.declare_parameter("slice_dh", 0.50)
        self.declare_parameter("ground_height", 0.0)
        self.declare_parameter("trav_interval_min", 0.50)
        self.declare_parameter("trav_interval_free", 0.65)
        self.declare_parameter("trav_slope_max", 0.40)
        self.declare_parameter("trav_step_max", 0.17)
        self.declare_parameter("gateway_height_tolerance", 0.22)
        self.declare_parameter("gateway_step_gradient_min", 0.08)
        self.declare_parameter("trav_kernel_size", 7)
        self.declare_parameter("trav_standable_ratio", 0.20)
        self.declare_parameter("trav_cost_barrier", 50.0)
        self.declare_parameter("safe_margin", 0.40)
        self.declare_parameter("inflation", 0.20)
        self.declare_parameter("a_star_cost_threshold", 49.0)
        self.declare_parameter("planner_safe_cost_margin", 15.0)
        self.declare_parameter("step_cost_weight", 0.20)
        self.declare_parameter("max_heading_rate", 10.0)
        self.declare_parameter("robot_z_offset", 0.80)
        self.declare_parameter("goal_zero_epsilon", 0.06)
        self.declare_parameter("goal_search_radius", 1.40)
        self.declare_parameter("start_search_radius", 2.50)
        self.declare_parameter("candidate_count", 24)
        self.declare_parameter("max_plan_attempts", 64)
        self.declare_parameter("optimize_trajectory", False)
        self.declare_parameter("optimizer_max_iterations", 100)
        self.declare_parameter("min_z", -0.40)
        self.declare_parameter("max_z", 20.00)
        self.declare_parameter("sample_stride", 1)
        self.declare_parameter("max_points", 1200000)
        self.declare_parameter("max_tomogram_points", 500000)
        self.declare_parameter("unknown_is_blocked", True)
        self.declare_parameter("prepend_odom_start", True)

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.official_tomogram = None
        self.official_planner = None
        self.official_ele_planner = None
        self.official_modules_error = None
        self.odom = None
        self.localization_pose = None
        self.pending_goal = None
        self.frame_id = "world"
        self.center = None
        self.map_dim_x = 0
        self.map_dim_y = 0
        self.layer_count = 0
        self.ground_layers = None
        self.ceiling_layers = None
        self.traversability_layers = None
        self.gateway_layers = None
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.map_ready = False
        self.cloud_update_in_progress = False
        self.last_plan_signature = None
        self.map_signature = None
        self.last_failed_signature = None

        self.path_pub = self.create_publisher(Path, self.get_parameter("path_topic").value, 10)
        self.debug_path_pub = self.create_publisher(
            Path, self.get_parameter("debug_path_topic").value, latched_qos
        )
        self.debug_map_pub = self.create_publisher(
            OccupancyGrid, self.get_parameter("debug_map_topic").value, latched_qos
        )
        self.tomogram_pub = self.create_publisher(
            PointCloud2, self.get_parameter("tomogram_topic").value, latched_qos
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter("cloud_topic").value,
            self.cloud_callback,
            latched_qos,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self.odom_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("localization_pose_topic").value,
            self.localization_pose_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter("goal_topic").value,
            self.goal_callback,
            10,
        )

        self.load_official_modules()
        self.get_logger().info(
            "Official PCT global planner ready: cloud=%s goal=%s path=%s"
            % (
                self.get_parameter("cloud_topic").value,
                self.get_parameter("goal_topic").value,
                self.get_parameter("path_topic").value,
            )
        )

    def load_official_modules(self):
        pct_root = os.path.abspath(str(self.get_parameter("pct_root").value))
        tomography_scripts = os.path.join(pct_root, "tomography", "scripts")
        planner_lib = os.path.join(pct_root, "planner", "lib")
        gtsam_lib = os.path.join(
            pct_root,
            "planner",
            "lib",
            "3rdparty",
            "gtsam-4.1.1",
            "install",
            "lib",
        )
        smoothing_lib = os.path.join(planner_lib, "build", "src", "common", "smoothing")
        library_paths = [gtsam_lib, smoothing_lib, planner_lib]
        current_library_path = os.environ.get("LD_LIBRARY_PATH", "")
        os.environ["LD_LIBRARY_PATH"] = ":".join(
            path for path in library_paths + [current_library_path] if path
        )
        for path in (tomography_scripts, planner_lib):
            if path not in sys.path:
                sys.path.insert(0, path)

        try:
            import a_star
            import traj_opt
            from tomogram import Tomogram
            from ele_planner import OfflineElePlanner

            self.official_a_star_module = a_star
            self.official_traj_opt_module = traj_opt
            self.official_tomogram_class = Tomogram
            self.official_ele_planner = OfflineElePlanner
            self.get_logger().info("Loaded official PCT tomography and planner modules from %s" % pct_root)
        except Exception as exc:
            self.official_modules_error = exc
            self.get_logger().error(
                "Cannot load official PCT modules from %s: %s. Build planner/lib and install cupy-cuda12x."
                % (pct_root, exc)
            )

    def odom_callback(self, msg):
        self.odom = msg
        if self.pending_goal is not None and self.map_ready:
            self.plan_to_goal(self.pending_goal)

    def localization_pose_callback(self, msg):
        if msg.header.frame_id and self.frame_id and msg.header.frame_id != self.frame_id:
            return
        self.localization_pose = msg
        if self.pending_goal is not None and self.map_ready:
            self.plan_to_goal(self.pending_goal)

    def planning_pose(self):
        if bool(self.get_parameter("use_localization_pose").value) and self.localization_pose is not None:
            return self.localization_pose.pose
        return None if self.odom is None else self.odom.pose

    def goal_callback(self, msg):
        self.pending_goal = msg
        self.plan_to_goal(msg)

    def cloud_callback(self, msg):
        if self.official_modules_error is not None or self.cloud_update_in_progress:
            return
        points = self.read_cloud_points(msg)
        if points.shape[0] < 10:
            self.get_logger().warn("Official PCT received too few usable cloud points")
            return
        self.cloud_update_in_progress = True
        try:
            self.frame_id = msg.header.frame_id or "world"
            self.build_official_tomogram(points)
            self.publish_debug_map()
            self.publish_tomogram_points()
            if self.pending_goal is not None and self.odom is not None:
                self.plan_to_goal(self.pending_goal)
        except Exception as exc:
            self.get_logger().error("Official PCT map update failed: %s" % exc)
        finally:
            self.cloud_update_in_progress = False

    def read_cloud_points(self, msg):
        minimum_z = float(self.get_parameter("min_z").value)
        maximum_z = float(self.get_parameter("max_z").value)
        stride = max(1, int(self.get_parameter("sample_stride").value))
        maximum_points = max(1, int(self.get_parameter("max_points").value))
        points = []
        for point_index, point in enumerate(
            point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ):
            if point_index % stride != 0:
                continue
            x_value, y_value, z_value = float(point[0]), float(point[1]), float(point[2])
            if not (minimum_z <= z_value <= maximum_z):
                continue
            points.append((x_value, y_value, z_value))
            if len(points) >= maximum_points:
                break
        return np.asarray(points, dtype=np.float32).reshape((-1, 3))

    def build_official_tomogram(self, points):
        resolution = float(self.get_parameter("resolution").value)
        slice_dh = float(self.get_parameter("slice_dh").value)
        ground_height = float(self.get_parameter("ground_height").value)
        points_min = np.min(points, axis=0)
        points_max = np.max(points, axis=0)
        center = (points_max[:2] + points_min[:2]) * 0.5
        map_dim_x = max(4, int(math.ceil((points_max[0] - points_min[0]) / resolution)) + 4)
        map_dim_y = max(4, int(math.ceil((points_max[1] - points_min[1]) / resolution)) + 4)
        layer_count = max(2, int(math.ceil((points_max[2] - ground_height) / slice_dh)))
        slice_h0 = ground_height + slice_dh

        config = SimpleNamespace(
            map=SimpleNamespace(resolution=resolution, slice_dh=slice_dh),
            trav=SimpleNamespace(
                kernel_size=int(self.get_parameter("trav_kernel_size").value),
                interval_min=float(self.get_parameter("trav_interval_min").value),
                interval_free=float(self.get_parameter("trav_interval_free").value),
                slope_max=float(self.get_parameter("trav_slope_max").value),
                step_max=float(self.get_parameter("trav_step_max").value),
                standable_ratio=float(self.get_parameter("trav_standable_ratio").value),
                cost_barrier=float(self.get_parameter("trav_cost_barrier").value),
                safe_margin=float(self.get_parameter("safe_margin").value),
                inflation=float(self.get_parameter("inflation").value),
            ),
        )
        self.official_tomogram = self.official_tomogram_class(config)
        self.official_tomogram.initMappingEnv(
            center, map_dim_x, map_dim_y, layer_count, slice_h0
        )
        layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c, _ = self.official_tomogram.point2map(points)
        self.center = np.asarray(center, dtype=np.float64)
        self.map_dim_x = map_dim_x
        self.map_dim_y = map_dim_y
        self.layer_count = int(layers_t.shape[0])
        self.origin_x = float(self.center[0] - 0.5 * self.map_dim_x * resolution)
        self.origin_y = float(self.center[1] - 0.5 * self.map_dim_y * resolution)
        self.traversability_layers = np.asarray(layers_t, dtype=np.float64)
        self.ground_layers = np.asarray(layers_g, dtype=np.float64)
        self.ceiling_layers = np.asarray(layers_c, dtype=np.float64)
        self.gateway_layers = self.build_official_gateway_layers(
            self.traversability_layers,
            self.ground_layers,
            float(self.get_parameter("a_star_cost_threshold").value),
            float(self.get_parameter("gateway_height_tolerance").value),
            float(self.get_parameter("gateway_step_gradient_min").value),
        )
        self.ground_layers_for_planner = np.nan_to_num(self.ground_layers, nan=-100.0)
        self.ceiling_layers_for_planner = np.nan_to_num(self.ceiling_layers, nan=1e6)
        self.initialize_official_planner(trav_grad_x, trav_grad_y)
        self.map_ready = True
        new_map_signature = (
            points.shape[0],
            self.map_dim_x,
            self.map_dim_y,
            self.layer_count,
            tuple(np.round(points_min, 2)),
            tuple(np.round(points_max, 2)),
        )
        if new_map_signature != self.map_signature:
            self.last_failed_signature = None
        self.map_signature = new_map_signature
        finite_counts = np.sum(np.isfinite(self.ground_layers), axis=(1, 2))
        height_ranges = []
        for layer_index in range(self.layer_count):
            heights = self.ground_layers[layer_index]
            valid_heights = heights[np.isfinite(heights)]
            if valid_heights.size == 0:
                height_ranges.append("empty")
            else:
                height_ranges.append("%.2f..%.2f" % (valid_heights.min(), valid_heights.max()))
        gateway_up_count = int(np.count_nonzero(self.gateway_layers[:-1] == 2.0))
        gateway_down_count = int(np.count_nonzero(self.gateway_layers[1:] == -2.0))
        self.get_logger().info(
            "Official PCT tomogram updated: layers=%d map=%dx%d points=%d"
            % (self.layer_count, self.map_dim_x, self.map_dim_y, points.shape[0])
        )
        self.get_logger().info(
            "PCT layers: cells=%s heights=%s gateways(up=%d down=%d)"
            % (finite_counts.tolist(), height_ranges, gateway_up_count, gateway_down_count)
        )

    @staticmethod
    def build_official_gateway_layers(
        traversability_layers,
        ground_layers,
        cost_threshold=49.0,
        height_tolerance=0.22,
        step_gradient_min=0.08,
    ):
        gateway_layers = np.zeros_like(traversability_layers, dtype=np.float64)
        if traversability_layers.shape[0] < 2:
            return gateway_layers
        valid_lower = np.isfinite(ground_layers[:-1])
        valid_upper = np.isfinite(ground_layers[1:])
        height_difference = np.abs(
            np.nan_to_num(ground_layers[1:], nan=-100.0)
            - np.nan_to_num(ground_layers[:-1], nan=-100.0)
        )
        cost_difference = traversability_layers[1:] - traversability_layers[:-1]
        gateway_up = (
            (cost_difference < -8.0)
            & (height_difference <= height_tolerance)
            & valid_lower
            & valid_upper
        )
        gateway_down = (
            (cost_difference > 8.0)
            & (height_difference <= height_tolerance)
            & valid_lower
            & valid_upper
        )

        local_gradient = np.zeros_like(ground_layers, dtype=np.float64)
        if ground_layers.shape[1] > 2:
            x_difference = np.abs(ground_layers[:, 2:, :] - ground_layers[:, :-2, :])
            local_gradient[:, 1:-1, :] = np.nan_to_num(x_difference, nan=0.0)
        if ground_layers.shape[2] > 2:
            y_difference = np.abs(ground_layers[:, :, 2:] - ground_layers[:, :, :-2])
            local_gradient[:, :, 1:-1] = np.maximum(
                local_gradient[:, :, 1:-1], np.nan_to_num(y_difference, nan=0.0)
            )
        step_candidate = (
            (height_difference <= height_tolerance)
            & (np.maximum(local_gradient[:-1], local_gradient[1:]) >= step_gradient_min)
            & valid_lower
            & valid_upper
            & (
                (traversability_layers[:-1] <= cost_threshold)
                | (traversability_layers[1:] <= cost_threshold)
            )
        )
        stair_gateway = step_candidate & (gateway_up | gateway_down)
        inferred_gateway = step_candidate & ~(gateway_up | gateway_down)
        gateway_up = stair_gateway | inferred_gateway
        gateway_down = stair_gateway | inferred_gateway
        gateway_layers[:-1][gateway_up] = 2.0
        gateway_layers[1:][gateway_down] = -2.0
        return gateway_layers

    def initialize_official_planner(self, trav_grad_x, trav_grad_y):
        planner = self.official_ele_planner(
            float(self.get_parameter("max_heading_rate").value), False
        )
        cost_map = self.traversability_layers.reshape(
            (-1, self.traversability_layers.shape[-1])
        ).astype(np.float64)
        height_map = self.ground_layers_for_planner.reshape(
            (-1, self.ground_layers_for_planner.shape[-1])
        ).astype(np.float64)
        ceiling_map = self.ceiling_layers_for_planner.reshape(
            (-1, self.ceiling_layers_for_planner.shape[-1])
        ).astype(np.float64)
        gateway_map = self.gateway_layers.reshape(
            (-1, self.gateway_layers.shape[-1])
        ).astype(np.float64)
        gradient_x = np.asarray(trav_grad_x, dtype=np.float64).reshape(
            (-1, self.traversability_layers.shape[-1])
        )
        gradient_y = np.asarray(trav_grad_y, dtype=np.float64).reshape(
            (-1, self.traversability_layers.shape[-1])
        )
        planner.init_map(
            float(self.get_parameter("a_star_cost_threshold").value),
            float(self.get_parameter("planner_safe_cost_margin").value),
            float(self.get_parameter("resolution").value),
            self.layer_count,
            float(self.get_parameter("step_cost_weight").value),
            cost_map,
            height_map,
            ceiling_map,
            gateway_map,
            gradient_y,
            -gradient_x,
        )
        planner.set_max_iterations(int(self.get_parameter("optimizer_max_iterations").value))
        self.official_planner = planner

    def plan_to_goal(self, goal):
        if not self.map_ready:
            self.get_logger().warn("No official PCT tomogram yet; goal cached")
            return
        planning_pose = self.planning_pose()
        if planning_pose is None:
            self.get_logger().warn("No base pose yet; goal cached")
            return

        start_floor = planning_pose.pose.position.z - float(self.get_parameter("robot_z_offset").value)
        goal_floor = self.goal_floor_height(goal)
        start_candidates = self.nearby_safe_nodes(
            planning_pose.pose.position.x,
            planning_pose.pose.position.y,
            start_floor,
            float(self.get_parameter("start_search_radius").value),
        )
        goal_candidates = self.nearby_safe_nodes(
            goal.pose.position.x,
            goal.pose.position.y,
            goal_floor,
            float(self.get_parameter("goal_search_radius").value),
        )
        if not start_candidates:
            self.get_logger().error("Official PCT has no safe start cell near current base pose")
            return
        if not goal_candidates:
            self.get_logger().error("Official PCT has no safe goal cell near requested goal")
            return

        failure_signature = (
            self.map_signature,
            tuple(round(float(value), 2) for value in (goal.pose.position.x, goal.pose.position.y, goal.pose.position.z)),
            tuple(start_candidates[0]),
        )
        if failure_signature == self.last_failed_signature:
            return

        optimize = bool(self.get_parameter("optimize_trajectory").value)
        maximum_attempts = max(1, int(self.get_parameter("max_plan_attempts").value))
        attempts = 0
        selected_start = None
        selected_goal = None
        path_matrix = None
        for start_candidate in start_candidates:
            for goal_candidate in goal_candidates:
                if attempts >= maximum_attempts:
                    break
                attempts += 1
                start_index = np.asarray(start_candidate, dtype=np.int32)
                goal_index = np.asarray(goal_candidate, dtype=np.int32)
                try:
                    success = self.official_planner.plan(start_index, goal_index, optimize)
                except Exception as exc:
                    self.get_logger().error("Official PCT plan call failed: %s" % exc)
                    return
                if not success:
                    continue
                selected_start = start_candidate
                selected_goal = goal_candidate
                if optimize:
                    path_matrix = self.optimized_path_matrix()
                else:
                    path_matrix = np.asarray(self.official_planner.get_debug_path())
                if path_matrix is not None and path_matrix.shape[0] > 0:
                    break
            if path_matrix is not None and path_matrix.shape[0] > 0:
                break

        if path_matrix is None or path_matrix.shape[0] == 0:
            self.last_failed_signature = failure_signature
            self.get_logger().error(
                "Official PCT A* failed after %d attempts (start=%d goal=%d)"
                % (attempts, len(start_candidates), len(goal_candidates))
            )
            return

        path = self.path_from_official_matrix(path_matrix, optimized=optimize)
        self.path_pub.publish(path)
        self.debug_path_pub.publish(path)
        self.pending_goal = None
        self.last_plan_signature = (selected_start, selected_goal)
        self.get_logger().info(
            "Published official PCT path with %d poses, start=%s goal=%s"
            % (len(path.poses), selected_start, selected_goal)
        )

    def optimized_path_matrix(self):
        optimizer = self.official_planner.get_trajectory_optimizer_wnoj()
        trajectory = np.asarray(optimizer.get_result_matrix(), dtype=np.float64)
        heights = np.asarray(optimizer.get_heights(), dtype=np.float64).reshape(-1)
        if trajectory.ndim != 2 or trajectory.shape[0] == 0 or trajectory.shape[1] < 4:
            return None
        path_matrix = np.zeros((trajectory.shape[0], 4), dtype=np.float64)
        path_matrix[:, 0] = trajectory[:, 0]
        path_matrix[:, 1] = trajectory[:, 3]
        path_matrix[:, 2] = heights
        path_matrix[:, 3] = np.nan
        return path_matrix

    def goal_floor_height(self, goal):
        goal_height = float(goal.pose.position.z)
        epsilon = float(self.get_parameter("goal_zero_epsilon").value)
        if abs(goal_height) <= epsilon:
            return None
        return goal_height

    def nearby_safe_nodes(self, world_x, world_y, target_floor, radius_m):
        resolution = float(self.get_parameter("resolution").value)
        center_x = int(round((world_x - self.center[0]) / resolution)) + self.map_dim_x // 2
        center_y = int(round((world_y - self.center[1]) / resolution)) + self.map_dim_y // 2
        radius_cells = max(1, int(math.ceil(radius_m / resolution)))
        candidates = []
        cost_threshold = float(self.get_parameter("a_star_cost_threshold").value)
        for x_index in range(center_x - radius_cells, center_x + radius_cells + 1):
            for y_index in range(center_y - radius_cells, center_y + radius_cells + 1):
                if not (0 <= x_index < self.map_dim_x and 0 <= y_index < self.map_dim_y):
                    continue
                xy_distance = math.hypot(x_index - center_x, y_index - center_y) * resolution
                if xy_distance > radius_m:
                    continue
                for layer_index in range(self.layer_count):
                    floor_height = self.ground_layers[layer_index, x_index, y_index]
                    cost = self.traversability_layers[layer_index, x_index, y_index]
                    if not np.isfinite(floor_height) or not np.isfinite(cost) or cost > cost_threshold:
                        continue
                    z_distance = 0.0 if target_floor is None else abs(floor_height - target_floor)
                    candidates.append(
                        (
                            xy_distance + 2.0 * z_distance,
                            [layer_index, y_index, x_index],
                        )
                    )
        candidates.sort(key=lambda item: item[0])
        maximum_candidates = max(1, int(self.get_parameter("candidate_count").value))
        return [candidate for _, candidate in candidates[:maximum_candidates]]

    def path_from_official_matrix(self, path_matrix, optimized=False):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id
        planning_pose = self.planning_pose()
        if bool(self.get_parameter("prepend_odom_start").value) and planning_pose is not None:
            start_pose = PoseStamped()
            start_pose.header = path.header
            start_pose.pose = planning_pose.pose
            path.poses.append(start_pose)

        previous_world = None
        for matrix_row in path_matrix:
            if optimized:
                column_index = float(matrix_row[0])
                row_index = float(matrix_row[1])
                floor_height = float(matrix_row[2])
                world_x = self.center[0] + (row_index - self.map_dim_x * 0.5) * float(
                    self.get_parameter("resolution").value
                )
                world_y = self.center[1] + (column_index - self.map_dim_y * 0.5) * float(
                    self.get_parameter("resolution").value
                )
            else:
                layer_index = int(round(float(matrix_row[0])))
                row_index = int(round(float(matrix_row[1])))
                column_index = int(round(float(matrix_row[2])))
                if not (0 <= layer_index < self.layer_count):
                    continue
                if not (0 <= row_index < self.map_dim_x and 0 <= column_index < self.map_dim_y):
                    continue
                floor_height = float(self.ground_layers[layer_index, row_index, column_index])
                if not np.isfinite(floor_height):
                    continue
                world_x = self.center[0] + (row_index - self.map_dim_x * 0.5) * float(
                    self.get_parameter("resolution").value
                )
                world_y = self.center[1] + (column_index - self.map_dim_y * 0.5) * float(
                    self.get_parameter("resolution").value
                )
            world_z = floor_height + float(self.get_parameter("robot_z_offset").value)
            current_world = (world_x, world_y, world_z)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = world_z
            if previous_world is not None:
                yaw = math.atan2(world_y - previous_world[1], world_x - previous_world[0])
                pose.pose.orientation.z = math.sin(0.5 * yaw)
                pose.pose.orientation.w = math.cos(0.5 * yaw)
            else:
                pose.pose.orientation.w = 1.0
            path.poses.append(pose)
            previous_world = current_world
        if len(path.poses) > 1 and path.poses[0].pose.orientation.w == 1.0:
            next_pose = path.poses[1]
            yaw = math.atan2(
                next_pose.pose.position.y - path.poses[0].pose.position.y,
                next_pose.pose.position.x - path.poses[0].pose.position.x,
            )
            path.poses[0].pose.orientation.z = math.sin(0.5 * yaw)
            path.poses[0].pose.orientation.w = math.cos(0.5 * yaw)
        return path

    def publish_debug_map(self):
        minimum_cost = np.min(self.traversability_layers, axis=0)
        valid_map = np.any(np.isfinite(self.ground_layers), axis=0)
        cost_threshold = float(self.get_parameter("a_star_cost_threshold").value)
        occupancy = np.full((self.map_dim_y, self.map_dim_x), -1, dtype=np.int8)
        occupancy[np.transpose(valid_map) & (np.transpose(minimum_cost) <= cost_threshold)] = 0
        occupancy[np.transpose(valid_map) & (np.transpose(minimum_cost) > cost_threshold)] = 100
        if not bool(self.get_parameter("unknown_is_blocked").value):
            occupancy[occupancy < 0] = 0
        message = OccupancyGrid()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.info.resolution = float(self.get_parameter("resolution").value)
        message.info.width = self.map_dim_x
        message.info.height = self.map_dim_y
        message.info.origin.position.x = self.origin_x
        message.info.origin.position.y = self.origin_y
        message.info.origin.orientation.w = 1.0
        message.data = occupancy.reshape(-1).tolist()
        self.debug_map_pub.publish(message)

    def publish_tomogram_points(self):
        valid_indices = np.argwhere(np.isfinite(self.ground_layers))
        maximum_points = max(1, int(self.get_parameter("max_tomogram_points").value))
        if valid_indices.shape[0] > maximum_points:
            selected_indices = np.linspace(
                0, valid_indices.shape[0] - 1, maximum_points, dtype=np.int64
            )
            valid_indices = valid_indices[selected_indices]
        resolution = float(self.get_parameter("resolution").value)
        points = [
            (
                float(self.center[0] + (row_index - self.map_dim_x * 0.5) * resolution),
                float(self.center[1] + (column_index - self.map_dim_y * 0.5) * resolution),
                float(self.ground_layers[layer_index, row_index, column_index]),
            )
            for layer_index, row_index, column_index in valid_indices
        ]
        self.tomogram_pub.publish(point_cloud2.create_cloud_xyz32(self.make_header(), points))

    def make_header(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        return header


def main(args=None):
    rclpy.init(args=args)
    node = OfficialPctGlobalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
