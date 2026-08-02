import math
import os
from threading import Lock

import cv2
import mujoco
import nav_msgs.msg
import numpy as np
import rclpy
import sensor_msgs.msg
import std_msgs.msg
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data

from bxi_example_py_elf3.traditional_line_detector import TraditionalLineDetector
from bxi_example_py_elf3.ufldv2_line_detector import UFLDv2LineDetector


class FrontLineCameraPublisher(Node):
    def __init__(self):
        super().__init__("front_line_camera_publisher")

        package_share = get_package_share_directory("bxi_example_py_elf3")
        default_model_file = os.path.join(
            package_share,
            "data",
            "mujoco_simulation",
            "elf3_400m_track.xml",
        )

        self.declare_parameter("model_file", default_model_file)
        self.declare_parameter("camera_name", "front_line_camera")
        self.declare_parameter("topic_prefix", "simulation/")
        self.declare_parameter("image_topic", "/simulation/front_line_camera/image_raw")
        self.declare_parameter("camera_info_topic", "/simulation/front_line_camera/camera_info")
        self.declare_parameter("camera_frame_id", "front_line_camera")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 360)
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("enable_line_detection", True)
        self.declare_parameter("line_detector_type", "traditional")
        self.declare_parameter("ufldv2_model_path", "")
        self.declare_parameter("ufldv2_dataset", "auto")
        self.declare_parameter("line_offset_topic", "/simulation/front_line_camera/line_offset")
        self.declare_parameter("line_state_topic", "/simulation/front_line_camera/line_state")

        model_file = self.get_parameter("model_file").value
        self.camera_name = self.get_parameter("camera_name").value
        topic_prefix = self.get_parameter("topic_prefix").value
        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        self.camera_frame_id = self.get_parameter("camera_frame_id").value
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        publish_hz = float(self.get_parameter("publish_hz").value)
        self.enable_line_detection = bool(self.get_parameter("enable_line_detection").value)
        self.line_detector_type = str(self.get_parameter("line_detector_type").value).lower()
        ufldv2_model_path = self.get_parameter("ufldv2_model_path").value
        ufldv2_dataset = self.get_parameter("ufldv2_dataset").value
        line_offset_topic = self.get_parameter("line_offset_topic").value
        line_state_topic = self.get_parameter("line_state_topic").value

        self.model = mujoco.MjModel.from_xml_path(model_file)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetData(self.model, self.data)
        self.renderer = mujoco.Renderer(self.model, height=self.height, width=self.width)

        self.camera_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_CAMERA,
            self.camera_name,
        )
        if self.camera_id < 0:
            raise RuntimeError(f"camera not found in MuJoCo model: {self.camera_name}")

        self.joint_qpos_addr = self.build_joint_qpos_addr()
        self.lock = Lock()
        self.latest_base_pose = None
        self.latest_joint_pos = {}
        self.line_detector = self.create_line_detector(ufldv2_model_path, ufldv2_dataset)
        self.last_line_offset = 0.0
        self.line_debug_counter = 0

        qos = QoSProfile(
            depth=1,
            durability=qos_profile_sensor_data.durability,
            reliability=qos_profile_sensor_data.reliability,
        )
        self.image_pub = self.create_publisher(sensor_msgs.msg.Image, image_topic, qos)
        self.camera_info_pub = self.create_publisher(
            sensor_msgs.msg.CameraInfo, camera_info_topic, qos
        )
        self.line_offset_pub = self.create_publisher(
            std_msgs.msg.Float32,
            line_offset_topic,
            10,
        )
        self.line_state_pub = self.create_publisher(
            std_msgs.msg.Float32MultiArray,
            line_state_topic,
            10,
        )
        self.odom_sub = self.create_subscription(
            nav_msgs.msg.Odometry, topic_prefix + "odom", self.odom_callback, qos
        )
        self.joint_sub = self.create_subscription(
            sensor_msgs.msg.JointState,
            topic_prefix + "joint_states",
            self.joint_callback,
            qos,
        )

        period = 1.0 / max(publish_hz, 1.0)
        self.timer = self.create_timer(period, self.timer_callback)
        self.get_logger().info(
            f"publishing MuJoCo camera '{self.camera_name}' to {image_topic} "
            f"({self.width}x{self.height})"
        )

    def create_line_detector(self, ufldv2_model_path, ufldv2_dataset):
        if self.line_detector_type == "ufldv2":
            self.get_logger().info(f"using UFLDv2 line detector: {ufldv2_model_path}")
            return UFLDv2LineDetector(
                ufldv2_model_path,
                self.width,
                self.height,
                dataset=ufldv2_dataset,
            )
        self.get_logger().info("using traditional line detector")
        return TraditionalLineDetector(self.width, self.height)

    def build_joint_qpos_addr(self):
        joint_qpos_addr = {}
        for joint_id in range(self.model.njnt):
            name = mujoco.mj_id2name(
                self.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                joint_id,
            )
            if not name or name == "world_joint":
                continue
            joint_qpos_addr[name] = int(self.model.jnt_qposadr[joint_id])
        return joint_qpos_addr

    def odom_callback(self, msg):
        pose = msg.pose.pose
        with self.lock:
            self.latest_base_pose = (
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
                float(pose.orientation.w),
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
            )

    def joint_callback(self, msg):
        with self.lock:
            if msg.name:
                for name, pos in zip(msg.name, msg.position):
                    self.latest_joint_pos[name] = float(pos)
            else:
                joint_names = list(self.joint_qpos_addr.keys())
                for name, pos in zip(joint_names, msg.position):
                    self.latest_joint_pos[name] = float(pos)

    def timer_callback(self):
        with self.lock:
            base_pose = self.latest_base_pose
            joint_pos = dict(self.latest_joint_pos)

        if base_pose is not None:
            self.data.qpos[0:7] = np.asarray(base_pose, dtype=np.float64)

        for name, pos in joint_pos.items():
            qpos_addr = self.joint_qpos_addr.get(name)
            if qpos_addr is not None:
                self.data.qpos[qpos_addr] = pos

        mujoco.mj_forward(self.model, self.data)
        self.renderer.update_scene(self.data, camera=self.camera_name)
        rgb = self.renderer.render()

        stamp = self.get_clock().now().to_msg()
        image_msg = sensor_msgs.msg.Image()
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = self.camera_frame_id
        image_msg.height = self.height
        image_msg.width = self.width
        image_msg.encoding = "rgb8"
        image_msg.is_bigendian = 0
        image_msg.step = self.width * 3
        image_msg.data = rgb.tobytes()
        self.image_pub.publish(image_msg)

        if self.enable_line_detection:
            self.publish_line_detection(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))

        self.camera_info_pub.publish(self.make_camera_info(stamp))

    def publish_line_detection(self, image):
        detection = self.line_detector.detect(image)
        self.line_debug_counter += 1
        if self.line_debug_counter % 50 == 0:
            self.get_logger().info(
                "line detector: confidence=%.3f mode=%.0f candidates=%d offset=%.3f margin=%.3f"
                % (
                    detection.get("confidence", 0.0),
                    detection.get("mode", 3.0),
                    len(detection.get("candidate_curves", [])),
                    detection.get("offset_norm", 0.0),
                    detection.get("boundary_margin", -1.0),
                )
            )
        if detection["confidence"] <= 0.0:
            self.last_line_offset *= 0.95
            offset = self.last_line_offset
            heading_error = 0.0
            confidence = 0.0
            control_error = offset
        else:
            offset = float(np.clip(detection["multi_point_offset"], -1.0, 1.0))
            heading_error = float(np.clip(detection["heading_error"], -1.0, 1.0))
            confidence = float(np.clip(detection["confidence"], 0.0, 1.0))
            control_error = float(np.clip(detection["control_error"], -1.0, 1.0))
            alpha = 0.18 + 0.22 * confidence
            self.last_line_offset = (1.0 - alpha) * self.last_line_offset + alpha * control_error

        offset_msg = std_msgs.msg.Float32()
        offset_msg.data = float(self.last_line_offset)
        self.line_offset_pub.publish(offset_msg)

        state_msg = std_msgs.msg.Float32MultiArray()
        state_msg.data = [
            float(offset),
            float(heading_error),
            float(confidence),
            float(control_error),
            float(np.clip(detection.get("boundary_margin", -1.0 if confidence <= 0.0 else 1.0), -1.0, 1.0)),
            float(np.clip(detection.get("lane_width_norm", 0.0), 0.0, 2.0)),
            float(detection.get("mode", 3.0 if confidence <= 0.0 else 0.0)),
        ]
        self.line_state_pub.publish(state_msg)

    def make_camera_info(self, stamp):
        camera_info = sensor_msgs.msg.CameraInfo()
        camera_info.header.stamp = stamp
        camera_info.header.frame_id = self.camera_frame_id
        camera_info.width = self.width
        camera_info.height = self.height

        fovy = float(self.model.cam_fovy[self.camera_id])
        focal = (self.height * 0.5) / math.tan(math.radians(fovy) * 0.5)
        cx = self.width * 0.5
        cy = self.height * 0.5

        camera_info.k = [focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0]
        camera_info.p = [
            focal,
            0.0,
            cx,
            0.0,
            0.0,
            focal,
            cy,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        return camera_info

    def destroy_node(self):
        if hasattr(self, "renderer"):
            self.renderer.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FrontLineCameraPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
