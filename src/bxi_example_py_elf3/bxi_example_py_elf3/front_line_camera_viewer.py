import cv2
import numpy as np
import rclpy
import sensor_msgs.msg
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data

from bxi_example_py_elf3.traditional_line_detector import TraditionalLineDetector
from bxi_example_py_elf3.ufldv2_line_detector import UFLDv2LineDetector


class FrontLineCameraViewer(Node):
    def __init__(self):
        super().__init__("front_line_camera_viewer")

        self.declare_parameter("image_topic", "/simulation/front_line_camera/image_raw")
        self.declare_parameter("window_name", "front_line_camera")
        self.declare_parameter("window_width", 640)
        self.declare_parameter("window_height", 360)
        self.declare_parameter("enable_line_detection", True)
        self.declare_parameter("line_detector_type", "traditional")
        self.declare_parameter("ufldv2_model_path", "")
        self.declare_parameter("ufldv2_dataset", "auto")
        self.declare_parameter("debug_mask", False)
        self.declare_parameter("show_candidates", False)

        self.image_topic = self.get_parameter("image_topic").value
        self.window_name = self.get_parameter("window_name").value
        self.window_width = int(self.get_parameter("window_width").value)
        self.window_height = int(self.get_parameter("window_height").value)
        self.enable_line_detection = bool(self.get_parameter("enable_line_detection").value)
        self.line_detector_type = str(self.get_parameter("line_detector_type").value).lower()
        self.ufldv2_model_path = self.get_parameter("ufldv2_model_path").value
        self.ufldv2_dataset = self.get_parameter("ufldv2_dataset").value
        self.debug_mask = bool(self.get_parameter("debug_mask").value)
        self.show_candidates = bool(self.get_parameter("show_candidates").value)

        qos = QoSProfile(
            depth=1,
            durability=qos_profile_sensor_data.durability,
            reliability=qos_profile_sensor_data.reliability,
        )
        self.sub = self.create_subscription(
            sensor_msgs.msg.Image,
            self.image_topic,
            self.image_callback,
            qos,
        )
        self.latest_image = None
        self.latest_stamp = None
        self.latest_detection = None
        self.line_detector = self.create_line_detector()

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.window_width, self.window_height)
        self.timer = self.create_timer(1.0 / 30.0, self.show_latest_image)
        self.get_logger().info(f"showing image topic: {self.image_topic}")

    def create_line_detector(self):
        if self.line_detector_type == "ufldv2":
            self.get_logger().info(f"using UFLDv2 line detector: {self.ufldv2_model_path}")
            return UFLDv2LineDetector(
                self.ufldv2_model_path,
                self.window_width,
                self.window_height,
                dataset=self.ufldv2_dataset,
            )
        self.get_logger().info("using traditional line detector")
        return TraditionalLineDetector(self.window_width, self.window_height)

    def image_callback(self, msg):
        image = self.ros_image_to_cv2(msg)
        if image is None:
            return
        self.latest_image = image
        self.latest_stamp = msg.header.stamp
        if self.enable_line_detection:
            self.latest_detection = self.detect_track_lines(image)

    def ros_image_to_cv2(self, msg):
        if msg.encoding not in ("bgr8", "rgb8", "mono8", "bgra8", "rgba8"):
            self.get_logger().warn(f"unsupported image encoding: {msg.encoding}", throttle_duration_sec=2.0)
            return None

        channels = {
            "mono8": 1,
            "bgr8": 3,
            "rgb8": 3,
            "bgra8": 4,
            "rgba8": 4,
        }[msg.encoding]

        expected_step = msg.width * channels
        if msg.step < expected_step:
            self.get_logger().warn("invalid image step", throttle_duration_sec=2.0)
            return None

        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if raw.size < msg.height * msg.step:
            self.get_logger().warn("image data is shorter than expected", throttle_duration_sec=2.0)
            return None

        rows = raw.reshape((msg.height, msg.step))
        packed = rows[:, :expected_step]
        if channels == 1:
            image = packed.reshape((msg.height, msg.width))
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        image = packed.reshape((msg.height, msg.width, channels))
        if msg.encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if msg.encoding == "bgra8":
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        if msg.encoding == "rgba8":
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        return image.copy()

    def show_latest_image(self):
        if self.latest_image is None:
            image = np.zeros((self.window_height, self.window_width, 3), dtype=np.uint8)
            cv2.putText(
                image,
                f"waiting: {self.image_topic}",
                (24, self.window_height // 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (220, 220, 220),
                1,
                cv2.LINE_AA,
            )
        else:
            image = self.latest_image

        if self.enable_line_detection and self.latest_detection is not None:
            if self.debug_mask:
                image = self.latest_detection["debug_mask"]
            else:
                image = self.draw_line_detection(image, self.latest_detection)

        cv2.imshow(self.window_name, image)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            rclpy.shutdown()
        elif key == ord("m"):
            self.debug_mask = not self.debug_mask

    def detect_track_lines(self, image):
        return self.line_detector.detect(image)

    def draw_line_detection(self, image, detection):
        overlay = image.copy()
        height, width = overlay.shape[:2]
        roi_top = detection["roi_top"]
        roi_bottom = detection["roi_bottom"]
        roi_left = detection["roi_left"]
        roi_right = detection["roi_right"]

        def points_in_roi(points):
            return [
                (int(x), int(y))
                for x, y in points
                if roi_left <= int(x) <= roi_right and roi_top <= int(y) <= roi_bottom
            ]

        cv2.rectangle(
            overlay,
            (roi_left, roi_top),
            (roi_right, roi_bottom),
            (50, 180, 50),
            1,
        )

        if self.show_candidates:
            for points in detection["candidate_curves"]:
                points = points_in_roi(points)
                if len(points) >= 2:
                    cv2.polylines(
                        overlay,
                        [np.asarray(points, dtype=np.int32)],
                        False,
                        (120, 120, 120),
                        1,
                        cv2.LINE_AA,
                    )

        for x, y in points_in_roi(detection["left_points"][::8]):
            cv2.circle(overlay, (int(x), int(y)), 3, (255, 160, 0), -1)

        for x, y in points_in_roi(detection["right_points"][::8]):
            cv2.circle(overlay, (int(x), int(y)), 3, (0, 220, 255), -1)

        curves = (
            ("left_curve_points", (255, 160, 0), 2),
            ("right_curve_points", (0, 220, 255), 2),
            ("center_curve_points", (0, 255, 0), 3),
        )
        for key, color, thickness in curves:
            points = points_in_roi(detection[key])
            if len(points) >= 2:
                cv2.polylines(
                    overlay,
                    [np.asarray(points, dtype=np.int32)],
                    False,
                    color,
                    thickness,
                    cv2.LINE_AA,
                )

        center_x = int(width * 0.5)
        cv2.line(overlay, (center_x, 0), (center_x, height - 1), (80, 80, 255), 1)
        raw_center_x = int(np.clip(detection["raw_lane_center"], 0, width - 1))
        cv2.line(overlay, (raw_center_x, int(roi_top)), (raw_center_x, int(roi_bottom)), (120, 220, 220), 1)
        target_x, target_y = detection["target_point"]
        cv2.circle(overlay, (int(target_x), int(target_y)), 6, (80, 255, 80), -1)

        status = (
            f"ctrl={detection['control_error']:+.3f} "
            f"offset={detection['multi_point_offset']:+.3f} "
            f"head={detection['heading_error']:+.3f} "
            f"confidence={detection['confidence']:.2f} "
            "m:mask q:quit"
        )
        cv2.putText(
            overlay,
            status,
            (12, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (0, 0, 0),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            overlay,
            status,
            (12, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (245, 245, 245),
            1,
            cv2.LINE_AA,
        )

        return overlay

    def destroy_node(self):
        cv2.destroyWindow(self.window_name)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FrontLineCameraViewer()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        else:
            node.destroy_node()


if __name__ == "__main__":
    main()
