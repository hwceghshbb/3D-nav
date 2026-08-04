from array import array as byte_array
import math
from threading import Lock
import time

import mujoco
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Header


class RgbdCameraPublisher(Node):
    def __init__(self):
        super().__init__("rgbd_camera_publisher")
        self.declare_parameter("model_file", "")
        self.declare_parameter("camera_name", "d435i_depth")
        self.declare_parameter("topic_prefix", "/simulation/")
        self.declare_parameter("output_prefix", "/simulation/d435i/")
        self.declare_parameter("frame_id", "d435i_depth_optical_frame")
        self.declare_parameter("color_frame_id", "d435i_color_optical_frame")
        self.declare_parameter("hidden_body_names", [""])
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("publish_hz", 15.0)
        self.declare_parameter("publish_color", False)
        self.declare_parameter("publish_raw_depth", False)
        self.declare_parameter("depth_encoding", "16UC1")
        self.declare_parameter("color_vfov_deg", 43.173360)
        self.declare_parameter("depth_vfov_deg", 58.605976)
        self.declare_parameter("align_depth_to_color", False)
        self.declare_parameter("trigger_topic", "")
        model_file = str(self.get_parameter("model_file").value)
        if not model_file:
            raise RuntimeError("model_file is required")
        self.model = mujoco.MjModel.from_xml_path(model_file)
        self.data = mujoco.MjData(self.model)
        self.camera_name = str(self.get_parameter("camera_name").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.color_frame_id = str(self.get_parameter("color_frame_id").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.model.vis.global_.offwidth = max(
            int(self.model.vis.global_.offwidth), self.width
        )
        self.model.vis.global_.offheight = max(
            int(self.model.vis.global_.offheight), self.height
        )
        camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, self.camera_name)
        if camera_id < 0:
            raise RuntimeError(f"camera not found in MuJoCo model: {self.camera_name}")
        self.camera_id = camera_id
        self.renderer = mujoco.Renderer(self.model, height=self.height, width=self.width)
        hidden_body_names = [
            str(name)
            for name in self.get_parameter("hidden_body_names").value
            if str(name)
        ]
        self.hidden_geom_ids = self.find_hidden_geom_ids(hidden_body_names)
        self.lock = Lock()
        self.base_pose = None
        self.joint_positions = {}
        self.timing_started = time.monotonic()
        self.timing_count = 0
        self.timing_total_ms = {
            "state": 0.0,
            "forward": 0.0,
            "depth": 0.0,
            "color": 0.0,
            "publish": 0.0,
            "total": 0.0,
        }
        self.timing_max_total_ms = 0.0
        prefix = str(self.get_parameter("topic_prefix").value)
        output = str(self.get_parameter("output_prefix").value)
        # Match the hardware camera package: image consumers may drop stale
        # frames instead of applying DDS backpressure to the camera pipeline.
        image_qos = qos_profile_sensor_data
        self.publish_color = bool(self.get_parameter("publish_color").value)
        self.publish_raw_depth = bool(self.get_parameter("publish_raw_depth").value)
        self.depth_encoding = str(self.get_parameter("depth_encoding").value).upper()
        if self.depth_encoding not in ("16UC1", "32FC1"):
            raise ValueError(
                f"depth_encoding must be 16UC1 or 32FC1, got {self.depth_encoding}"
            )
        self.color_vfov_deg = float(self.get_parameter("color_vfov_deg").value)
        self.depth_vfov_deg = float(self.get_parameter("depth_vfov_deg").value)
        self.align_depth_to_color = bool(
            self.get_parameter("align_depth_to_color").value
        )
        self.rgb_pub = (
            self.create_publisher(Image, output + "color/image_raw", image_qos)
            if self.publish_color
            else None
        )
        self.depth_pub = (
            self.create_publisher(Image, output + "depth/image_raw", image_qos)
            if self.publish_raw_depth
            else None
        )
        self.depth_rect_pub = self.create_publisher(
            Image, output + "depth/image_rect_raw", image_qos
        )
        self.info_pub = self.create_publisher(CameraInfo, output + "depth/camera_info", image_qos)
        self.color_info_pub = (
            self.create_publisher(
                CameraInfo, output + "color/camera_info", image_qos
            )
            if self.publish_color
            else None
        )
        self.create_subscription(Odometry, prefix + "odom", self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(JointState, prefix + "joint_states", self.joint_callback, qos_profile_sensor_data)
        trigger_topic = str(self.get_parameter("trigger_topic").value)
        if trigger_topic:
            self.trigger_subscription = self.create_subscription(
                Header, trigger_topic, self.publish, image_qos
            )
            self.timer = None
        else:
            period = 1.0 / max(
                float(self.get_parameter("publish_hz").value), 1.0
            )
            self.timer = self.create_timer(period, self.publish)

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        with self.lock:
            self.base_pose = [p.x, p.y, p.z, q.w, q.x, q.y, q.z]

    def joint_callback(self, msg):
        with self.lock:
            self.joint_positions.update(dict(zip(msg.name, msg.position)))

    def publish(self, trigger=None):
        total_started = time.perf_counter()
        with self.lock:
            if self.base_pose is not None:
                self.data.qpos[:7] = np.asarray(self.base_pose)
            for name, position in self.joint_positions.items():
                joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                if joint_id >= 0:
                    self.data.qpos[self.model.jnt_qposadr[joint_id]] = position
        state_finished = time.perf_counter()
        mujoco.mj_forward(self.model, self.data)
        forward_finished = time.perf_counter()
        with self.hide_configured_geoms():
            depth_vfov = (
                self.color_vfov_deg
                if self.align_depth_to_color
                else self.depth_vfov_deg
            )
            self.model.cam_fovy[self.camera_id] = depth_vfov
            self.renderer.update_scene(self.data, camera=self.camera_name)
            self.renderer.enable_depth_rendering()
            depth = self.renderer.render().astype(np.float32)
            self.renderer.disable_depth_rendering()
            depth_finished = time.perf_counter()
            if self.rgb_pub is not None:
                self.model.cam_fovy[self.camera_id] = self.color_vfov_deg
                self.renderer.update_scene(self.data, camera=self.camera_name)
                rgb = self.renderer.render()
        color_finished = time.perf_counter()
        stamp = (
            trigger.stamp
            if trigger is not None
            else self.get_clock().now().to_msg()
        )
        if self.rgb_pub is not None:
            color_msg = self.image(
                rgb,
                "rgb8",
                stamp,
                self.width * 3,
                self.color_frame_id,
            )
            self.rgb_pub.publish(color_msg)
            self.color_info_pub.publish(
                self.camera_info(stamp, self.color_frame_id, self.color_vfov_deg)
            )
        depth_frame_id = (
            self.color_frame_id if self.align_depth_to_color else self.frame_id
        )
        if self.depth_encoding == "16UC1":
            depth_data = np.clip(depth * 1000.0, 0.0, 65535.0).astype(np.uint16)
            depth_step = self.width * 2
        else:
            depth_data = depth
            depth_step = self.width * 4
        depth_msg = self.image(
            depth_data,
            self.depth_encoding,
            stamp,
            depth_step,
            depth_frame_id,
        )
        if self.depth_pub is not None:
            self.depth_pub.publish(depth_msg)
        self.depth_rect_pub.publish(depth_msg)
        self.info_pub.publish(
            self.camera_info(stamp, depth_frame_id, depth_vfov)
        )
        publish_finished = time.perf_counter()
        self.record_timing(
            total_started,
            state_finished,
            forward_finished,
            depth_finished,
            color_finished,
            publish_finished,
        )

    def record_timing(self, start, state, forward, depth, color, published):
        samples = {
            "state": (state - start) * 1000.0,
            "forward": (forward - state) * 1000.0,
            "depth": (depth - forward) * 1000.0,
            "color": (color - depth) * 1000.0,
            "publish": (published - color) * 1000.0,
            "total": (published - start) * 1000.0,
        }
        self.timing_count += 1
        for name, value in samples.items():
            self.timing_total_ms[name] += value
        self.timing_max_total_ms = max(
            self.timing_max_total_ms, samples["total"]
        )

        now = time.monotonic()
        elapsed = now - self.timing_started
        if elapsed < 5.0:
            return
        averages = {
            name: value / max(self.timing_count, 1)
            for name, value in self.timing_total_ms.items()
        }
        self.get_logger().info(
            f"RGB-D timing {self.width}x{self.height}: "
            f"rate={self.timing_count / elapsed:.1f}Hz "
            + " ".join(
                f"{name}={averages[name]:.1f}ms"
                for name in ("state", "forward", "depth", "color", "publish", "total")
            )
            + f" max_total={self.timing_max_total_ms:.1f}ms"
        )
        self.timing_started = now
        self.timing_count = 0
        self.timing_total_ms = {name: 0.0 for name in self.timing_total_ms}
        self.timing_max_total_ms = 0.0

    def image(self, array, encoding, stamp, step, frame_id=None):
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id or self.frame_id
        msg.height, msg.width, msg.encoding = array.shape[0], array.shape[1], encoding
        msg.is_bigendian = 0
        msg.step = step
        msg.data = byte_array("B", array.tobytes())
        return msg

    def camera_info(self, stamp, frame_id, vertical_fov_deg):
        info = CameraInfo()
        info.header.stamp, info.header.frame_id = stamp, frame_id
        info.width, info.height = self.width, self.height
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        focal = self.height * 0.5 / math.tan(
            math.radians(vertical_fov_deg) * 0.5
        )
        info.k = [
            focal, 0.0, self.width * 0.5,
            0.0, focal, self.height * 0.5,
            0.0, 0.0, 1.0,
        ]
        info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        info.p = [
            focal, 0.0, self.width * 0.5, 0.0,
            0.0, focal, self.height * 0.5, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return info

    def find_hidden_geom_ids(self, body_names):
        hidden_body_names = set(body_names or [])
        hidden_geom_ids = []
        if not hidden_body_names:
            return hidden_geom_ids
        for geom_id in range(self.model.ngeom):
            body_id = int(self.model.geom_bodyid[geom_id])
            body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id)
            if body_name in hidden_body_names:
                hidden_geom_ids.append(geom_id)
        if hidden_geom_ids:
            self.get_logger().info(
                f"Hide {len(hidden_geom_ids)} geoms for camera {self.camera_name}: "
                f"{sorted(hidden_body_names)}"
            )
        return hidden_geom_ids

    def hide_configured_geoms(self):
        class HiddenGeomContext:
            def __init__(self, node):
                self.node = node
                self.backup = None
                self.scene_option_backup = None
                self.hidden_group = 5

            def __enter__(self):
                if not self.node.hidden_geom_ids:
                    return
                ids = self.node.hidden_geom_ids
                self.backup = self.node.model.geom_group[ids].copy()
                scene_option = getattr(self.node.renderer, "_scene_option", None)
                if scene_option is not None:
                    self.scene_option_backup = scene_option.geomgroup.copy()
                    scene_option.geomgroup[self.hidden_group] = 0
                self.node.model.geom_group[ids] = self.hidden_group

            def __exit__(self, exc_type, exc, tb):
                if not self.node.hidden_geom_ids:
                    return
                self.node.model.geom_group[self.node.hidden_geom_ids] = self.backup
                scene_option = getattr(self.node.renderer, "_scene_option", None)
                if scene_option is not None and self.scene_option_backup is not None:
                    scene_option.geomgroup[:] = self.scene_option_backup

        return HiddenGeomContext(self)

    def destroy_node(self):
        self.renderer.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RgbdCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
