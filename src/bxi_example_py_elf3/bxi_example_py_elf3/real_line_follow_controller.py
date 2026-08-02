"""Real-camera high-speed line following controller for ELF3."""

from __future__ import annotations

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import std_msgs.msg
import communication.msg as bxi_msg

from .fast_line_detector import FastLineDetector, FastLineConfig


class RealLineFollowController(Node):
    def __init__(self) -> None:
        super().__init__("real_line_follow_controller")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("motion_commands_topic", "motion_commands")
        self.declare_parameter("debug_image_topic", "~/debug_image")
        self.declare_parameter("status_topic", "~/status")
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("fixed_forward_speed", 2.0)
        self.declare_parameter("max_yawdot", 1.2)
        self.declare_parameter("yaw_gain", 1.8)
        self.declare_parameter("min_confidence", 0.25)
        self.declare_parameter("image_timeout", 0.25)
        self.declare_parameter("enable_motion", False)
        self.declare_parameter("debug_image", True)
        self.declare_parameter("invert_yaw", False)
        self.declare_parameter("roi_top_ratio", 0.28)
        self.declare_parameter("temporal_alpha", 0.55)
        self.declare_parameter("line_color", "red")

        image_topic = str(self.get_parameter("image_topic").value)
        command_topic = str(self.get_parameter("motion_commands_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.fixed_speed = float(self.get_parameter("fixed_forward_speed").value)
        self.max_yawdot = abs(float(self.get_parameter("max_yawdot").value))
        self.yaw_gain = float(self.get_parameter("yaw_gain").value)
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.image_timeout = float(self.get_parameter("image_timeout").value)
        self.enable_motion = bool(self.get_parameter("enable_motion").value)
        self.debug_enabled = bool(self.get_parameter("debug_image").value)
        self.invert_yaw = bool(self.get_parameter("invert_yaw").value)
        roi_top = float(self.get_parameter("roi_top_ratio").value)
        alpha = float(self.get_parameter("temporal_alpha").value)
        line_color = str(self.get_parameter("line_color").value).lower()
        if line_color not in {"red", "white", "auto"}:
            raise ValueError("line_color must be red, white, or auto")

        self.bridge = CvBridge()
        self.detector = FastLineDetector(FastLineConfig(
            roi_top_ratio=roi_top,
            temporal_alpha=alpha,
            line_color=line_color,
        ))
        self.latest_image: np.ndarray | None = None
        self.latest_stamp = 0.0
        self.latest_detection: dict[str, object] | None = None
        self.image_count = 0
        self.last_yawdot = 0.0
        self.last_log_time = 0.0

        self.command_pub = self.create_publisher(bxi_msg.MotionCommands, command_topic, 10)
        self.status_pub = self.create_publisher(std_msgs.msg.Float32MultiArray, self.status_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 1)
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / max(float(self.get_parameter("publish_hz").value), 1.0), self.control_callback)
        self.no_image_warned = False
        self.get_logger().warn(
            f"real line follow ready: {image_topic} -> {command_topic}; "
            f"enable_motion={self.enable_motion}, fixed_speed={self.fixed_speed:.2f} m/s"
        )

    def image_callback(self, msg: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"image conversion failed: {exc}")
            return
        self.latest_image = np.asarray(image).copy()
        self.latest_stamp = time.monotonic()
        self.image_count += 1

    def control_callback(self) -> None:
        now = time.monotonic()
        image = self.latest_image
        fresh = image is not None and now - self.latest_stamp <= self.image_timeout
        if image is None and not self.no_image_warned:
            self.no_image_warned = True
            self.get_logger().error(
                "no image received; start RealSense and check image_topic "
                "with: ros2 topic hz /camera/camera/color/image_raw"
            )
        elif image is not None and self.no_image_warned:
            self.no_image_warned = False
            self.get_logger().info(f"image stream connected: {self.image_count} frames received")
        detection = self.detector.detect(image) if fresh else None
        self.latest_detection = detection

        confidence = float(detection["confidence"]) if detection else 0.0
        control = float(detection["control_error"]) if detection else 0.0
        safe = fresh and detection is not None and confidence >= self.min_confidence
        yawdot = self.yaw_gain * control if safe else 0.0
        if self.invert_yaw:
            yawdot = -yawdot
        yawdot = float(np.clip(yawdot, -self.max_yawdot, self.max_yawdot))
        max_delta = self.max_yawdot * 8.0 / 50.0
        yawdot = float(np.clip(yawdot, self.last_yawdot - max_delta, self.last_yawdot + max_delta))
        self.last_yawdot = yawdot

        msg = bxi_msg.MotionCommands()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "real_fast_line_follow"
        if self.enable_motion and safe:
            msg.vel_des.x = self.fixed_speed
            msg.yawdot_des = yawdot
        else:
            msg.vel_des.x = 0.0
            msg.yawdot_des = 0.0
        self.command_pub.publish(msg)

        status = std_msgs.msg.Float32MultiArray()
        status.data = [float(control), float(yawdot), confidence, 1.0 if safe else 0.0, float(self.fixed_speed)]
        self.status_pub.publish(status)
        if self.debug_enabled and detection is not None:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(self.make_debug_image(image, detection), encoding="bgr8"))

        if now - self.last_log_time > 1.0:
            self.last_log_time = now
            self.get_logger().info(
                f"safe={safe} conf={confidence:.2f} control={control:+.3f} "
                f"yawdot={yawdot:+.3f} motion={self.enable_motion}"
            )

    @staticmethod
    def make_debug_image(image: np.ndarray, detection: dict[str, object]) -> np.ndarray:
        output = image.copy()
        points = np.asarray(detection["curve_points"], dtype=np.int32)
        if len(points) > 1:
            cv2.polylines(output, [points], False, (0, 255, 0), 2, cv2.LINE_AA)
        for point in np.asarray(detection["points"], dtype=np.int32):
            cv2.circle(output, tuple(point), 3, (255, 0, 0), -1)
        text = f"c={float(detection['confidence']):.2f} e={float(detection['control_error']):+.3f} lost={detection['lost_frames']}"
        cv2.putText(output, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(output, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        return output


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RealLineFollowController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
