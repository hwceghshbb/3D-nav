import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from scan_planner_msgs.msg import Bspline


class BsplinePathVisualizer(Node):
    def __init__(self):
        super().__init__("bspline_path_visualizer")
        self.declare_parameter("bspline_topic", "/planning/bspline")
        self.declare_parameter("path_topic", "/planning/bspline_path")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("sample_dt", 0.05)
        self.publisher = self.create_publisher(Path, self.get_parameter("path_topic").value, 10)
        self.create_subscription(
            Bspline, self.get_parameter("bspline_topic").value, self.bspline_callback, 10
        )

    def bspline_callback(self, msg):
        if msg.order <= 0 or len(msg.pos_pts) <= msg.order or len(msg.knots) < len(msg.pos_pts) + msg.order + 1:
            self.get_logger().warn("Ignoring invalid B-spline for visualization")
            return

        start = msg.knots[msg.order]
        end = msg.knots[len(msg.knots) - msg.order - 1]
        if end <= start:
            return

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = str(self.get_parameter("frame_id").value)
        sample_dt = max(float(self.get_parameter("sample_dt").value), 0.01)
        sample_count = max(2, int(math.ceil((end - start) / sample_dt)) + 1)
        for index in range(sample_count):
            point = self.evaluate(msg, min(end, start + index * sample_dt))
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = point[2]
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.publisher.publish(path)

    @staticmethod
    def evaluate(msg, u):
        degree = msg.order
        knots = msg.knots
        max_span = len(msg.pos_pts) - 1
        span = degree
        while span < max_span and knots[span + 1] < u:
            span += 1

        points = [
            [
                msg.pos_pts[span - degree + i].x,
                msg.pos_pts[span - degree + i].y,
                msg.pos_pts[span - degree + i].z,
            ]
            for i in range(degree + 1)
        ]

        for r in range(1, degree + 1):
            for i in range(degree, r - 1, -1):
                denominator = knots[i + 1 + span - r] - knots[i + span - degree]
                alpha = 0.0 if abs(denominator) < 1.0e-9 else (u - knots[i + span - degree]) / denominator
                points[i] = [
                    (1.0 - alpha) * points[i - 1][axis] + alpha * points[i][axis]
                    for axis in range(3)
                ]
        return points[degree]


def main(args=None):
    rclpy.init(args=args)
    node = BsplinePathVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
