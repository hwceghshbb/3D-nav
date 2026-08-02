import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray, String


class TrackProgressEvaluator(Node):
    """Evaluate the run against the simulated track geometry, never control it."""

    def __init__(self):
        super().__init__("track_progress_evaluator")
        self.declare_parameter("odom_topic", "/simulation/odom")
        self.declare_parameter("line_state_topic", "/simulation/front_line_camera/line_state")
        self.declare_parameter("state_topic", "/track_eval/state")
        self.declare_parameter("debug_topic", "/track_eval/debug")
        self.declare_parameter("straight_half_length", 14.76825)
        self.declare_parameter("lane_center_radius", 13.385)
        self.declare_parameter("lane_width", 1.22)
        self.declare_parameter("robot_half_width", 0.30)
        self.declare_parameter("sample_count", 1200)
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("log_period", 1.0)

        self.a = float(self.get_parameter("straight_half_length").value)
        self.radius = float(self.get_parameter("lane_center_radius").value)
        self.lane_width = float(self.get_parameter("lane_width").value)
        self.robot_half_width = float(self.get_parameter("robot_half_width").value)
        self.points, self.tangents, self.arc_lengths = self._build_centerline(
            int(self.get_parameter("sample_count").value)
        )
        self.track_length = float(
            self.arc_lengths[-1] + np.linalg.norm(self.points[0] - self.points[-1])
        )

        self.pose = None
        self.start_xy = None
        self.last_xy = None
        self.travel_distance = 0.0
        self.net_displacement = 0.0
        self.speed = 0.0
        self.line_state = []
        self.last_index = None
        self.max_index_step = max(20, int(self.points.shape[0] / 40))
        self.last_progress = None
        self.unwrapped_progress = 0.0
        self.max_abs_cte = 0.0
        self.sum_abs_cte = 0.0
        self.sample_count = 0
        self.inside_count = 0
        self.line_valid_count = 0
        self.lost_line_count = 0
        self.last_line_mode = 3
        self.last_log_time = self.get_clock().now()

        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.odom_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("line_state_topic").value),
            self.line_callback,
            10,
        )
        self.state_pub = self.create_publisher(
            Float32MultiArray, str(self.get_parameter("state_topic").value), 10
        )
        self.debug_pub = self.create_publisher(
            String, str(self.get_parameter("debug_topic").value), 10
        )
        period = 1.0 / max(float(self.get_parameter("publish_hz").value), 1.0)
        self.create_timer(period, self.timer_callback)
        self.get_logger().info(
            "track evaluator: stadium prior map length=%.2fm lane_width=%.2fm"
            % (self.track_length, self.lane_width)
        )

    def _build_centerline(self, sample_count):
        # Start is the left end of the top straight, heading +x, matching the launch pose.
        segment_count = max(sample_count // 4, 40)
        samples = []
        for x in np.linspace(-self.a, self.a, segment_count, endpoint=False):
            samples.append((x, self.radius))
        for theta in np.linspace(math.pi / 2.0, -math.pi / 2.0, segment_count, endpoint=False):
            samples.append((self.a + self.radius * math.cos(theta), self.radius * math.sin(theta)))
        for x in np.linspace(self.a, -self.a, segment_count, endpoint=False):
            samples.append((x, -self.radius))
        for theta in np.linspace(-math.pi / 2.0, math.pi / 2.0, segment_count, endpoint=False):
            samples.append((-self.a + self.radius * math.cos(theta), self.radius * math.sin(theta)))
        points = np.asarray(samples, dtype=np.float64)
        deltas = np.roll(points, -1, axis=0) - points
        lengths = np.linalg.norm(deltas, axis=1)
        tangents = deltas / np.maximum(lengths[:, None], 1e-9)
        arc_lengths = np.concatenate(([0.0], np.cumsum(lengths)))[:-1]
        return points, tangents, arc_lengths

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.pose = (float(p.x), float(p.y), yaw)
        xy = np.asarray((float(p.x), float(p.y)), dtype=np.float64)
        if self.start_xy is None:
            self.start_xy = xy.copy()
            self.last_xy = xy.copy()
        else:
            step = float(np.linalg.norm(xy - self.last_xy))
            # Reject odometry teleportation while retaining normal walking
            # motion and turning-induced lateral displacement.
            if step <= 0.50:
                self.travel_distance += step
            self.last_xy = xy
            self.net_displacement = float(np.linalg.norm(xy - self.start_xy))
        self.speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    def line_callback(self, msg):
        self.line_state = list(msg.data)
        if len(self.line_state) >= 7:
            self.last_line_mode = int(round(self.line_state[6]))

    def _nearest_track_point(self, x, y):
        position = np.asarray((x, y))
        if self.last_index is None:
            distances = np.sum((self.points - position) ** 2, axis=1)
            index = int(np.argmin(distances))
        else:
            offsets = np.arange(-self.max_index_step, self.max_index_step + 1)
            indices = (self.last_index + offsets) % self.points.shape[0]
            distances = np.sum((self.points[indices] - position) ** 2, axis=1)
            index = int(indices[int(np.argmin(distances))])
        point = self.points[index]
        tangent = self.tangents[index]
        delta = np.asarray((x, y)) - point
        signed_error = float(tangent[0] * delta[1] - tangent[1] * delta[0])
        heading = math.atan2(float(tangent[1]), float(tangent[0]))
        return index, signed_error, heading

    @staticmethod
    def _angle_error(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def timer_callback(self):
        if self.pose is None:
            return
        x, y, yaw = self.pose
        index, cte, track_heading = self._nearest_track_point(x, y)
        progress = float(self.arc_lengths[index])
        if self.last_progress is None:
            self.last_progress = progress
            self.unwrapped_progress = progress
        else:
            delta = progress - self.last_progress
            if delta < -self.track_length * 0.5:
                delta += self.track_length
            elif delta > self.track_length * 0.5:
                delta -= self.track_length
            if abs(delta) > self.track_length * 0.04:
                self.get_logger().warn(
                    "prior-map progress jump rejected: %.2fm; keeping previous estimate"
                    % delta
                )
                progress = self.last_progress
                delta = 0.0
            # Do not hide a genuine reverse run in the evaluator.
            self.unwrapped_progress += delta
            self.last_progress = progress

        abs_cte = abs(cte)
        safe_half_width = max(self.lane_width * 0.5 - self.robot_half_width, 0.05)
        raw_inside = abs_cte <= self.lane_width * 0.5
        inside = abs_cte <= safe_half_width
        if self.last_index is not None and abs(index - self.last_index) > self.max_index_step:
            self.get_logger().warn("prior-map nearest point jumped; track localization may be invalid")
        self.last_index = index

        self.sample_count += 1
        self.inside_count += int(inside)
        self.sum_abs_cte += abs_cte
        self.max_abs_cte = max(self.max_abs_cte, abs_cte)
        confidence = self.line_state[2] if len(self.line_state) >= 3 else 0.0
        line_valid = confidence >= 0.35 and self.last_line_mode != 3
        self.line_valid_count += int(line_valid)
        if not line_valid and self.last_line_mode == 3:
            self.lost_line_count += 1

        heading_error = self._angle_error(yaw - track_heading)
        lap = math.floor(self.unwrapped_progress / self.track_length) if self.unwrapped_progress >= 0.0 else -1
        state = Float32MultiArray()
        state.data = [
            float(self.unwrapped_progress),
            float(self.unwrapped_progress / self.track_length),
            float(lap),
            float(cte),
            float(raw_inside),
            float(inside),
            float(heading_error),
            float(self.speed),
            float(confidence),
            float(self.last_line_mode),
            float(self.max_abs_cte),
            float(self.sum_abs_cte / max(self.sample_count, 1)),
            float(self.inside_count / max(self.sample_count, 1)),
            float(self.line_valid_count / max(self.sample_count, 1)),
            float(self.lost_line_count),
            float(self.travel_distance),
            float(self.net_displacement),
            float(x),
            float(y),
        ]
        self.state_pub.publish(state)

        now = self.get_clock().now()
        elapsed = (now - self.last_log_time).nanoseconds * 1e-9
        if elapsed >= float(self.get_parameter("log_period").value):
            self.last_log_time = now
            message = (
                "map_progress=%.1fm (%.1f%%) traveled=%.1fm displacement=%.1fm "
                "pose=(%.2f,%.2f) lap=%d cte=%+.2fm inside=%s safe=%s "
                "speed=%.2fm/s line_conf=%.2f line_mode=%d inside_ratio=%.1f%%"
                % (
                    self.unwrapped_progress,
                    100.0 * self.unwrapped_progress / self.track_length,
                    self.travel_distance,
                    self.net_displacement,
                    x,
                    y,
                    lap,
                    cte,
                    str(raw_inside).lower(),
                    str(inside).lower(),
                    self.speed,
                    confidence,
                    self.last_line_mode,
                    100.0 * self.inside_count / max(self.sample_count, 1),
                )
            )
            self.debug_pub.publish(String(data=message))
            self.get_logger().info(message)


def main(args=None):
    rclpy.init(args=args)
    node = TrackProgressEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
