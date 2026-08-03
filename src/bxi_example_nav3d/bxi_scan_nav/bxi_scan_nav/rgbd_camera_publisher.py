import math
from threading import Lock

import mujoco
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState


class RgbdCameraPublisher(Node):
    def __init__(self):
        super().__init__("rgbd_camera_publisher")
        self.declare_parameter("model_file", "")
        self.declare_parameter("camera_name", "d435i_depth")
        self.declare_parameter("topic_prefix", "/simulation/")
        self.declare_parameter("output_prefix", "/simulation/d435i/")
        self.declare_parameter("frame_id", "d435i_depth_optical_frame")
        self.declare_parameter("hidden_body_names", [""])
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("publish_hz", 15.0)
        self.declare_parameter("publish_color", False)
        self.declare_parameter("publish_raw_depth", False)
        model_file = str(self.get_parameter("model_file").value)
        if not model_file:
            raise RuntimeError("model_file is required")
        self.model = mujoco.MjModel.from_xml_path(model_file)
        self.data = mujoco.MjData(self.model)
        self.camera_name = str(self.get_parameter("camera_name").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
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
        prefix = str(self.get_parameter("topic_prefix").value)
        output = str(self.get_parameter("output_prefix").value)
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.publish_color = bool(self.get_parameter("publish_color").value)
        self.publish_raw_depth = bool(self.get_parameter("publish_raw_depth").value)
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
        self.create_subscription(Odometry, prefix + "odom", self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(JointState, prefix + "joint_states", self.joint_callback, qos_profile_sensor_data)
        period = 1.0 / max(float(self.get_parameter("publish_hz").value), 1.0)
        self.timer = self.create_timer(period, self.publish)

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        with self.lock:
            self.base_pose = [p.x, p.y, p.z, q.w, q.x, q.y, q.z]

    def joint_callback(self, msg):
        with self.lock:
            self.joint_positions.update(dict(zip(msg.name, msg.position)))

    def publish(self):
        with self.lock:
            if self.base_pose is not None:
                self.data.qpos[:7] = np.asarray(self.base_pose)
            for name, position in self.joint_positions.items():
                joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                if joint_id >= 0:
                    self.data.qpos[self.model.jnt_qposadr[joint_id]] = position
        mujoco.mj_forward(self.model, self.data)
        with self.hide_configured_geoms():
            self.renderer.update_scene(self.data, camera=self.camera_name)
            self.renderer.enable_depth_rendering()
            depth = self.renderer.render().astype(np.float32)
            self.renderer.disable_depth_rendering()
            if self.rgb_pub is not None:
                self.renderer.update_scene(self.data, camera=self.camera_name)
                rgb = self.renderer.render()
        stamp = self.get_clock().now().to_msg()
        if self.rgb_pub is not None:
            self.rgb_pub.publish(self.image(rgb, "rgb8", stamp, self.width * 3))
        depth_msg = self.image(depth, "32FC1", stamp, self.width * 4)
        if self.depth_pub is not None:
            self.depth_pub.publish(depth_msg)
        self.depth_rect_pub.publish(depth_msg)
        info = CameraInfo()
        info.header.stamp, info.header.frame_id = stamp, self.frame_id
        info.width, info.height = self.width, self.height
        focal = self.height * 0.5 / math.tan(math.radians(float(self.model.cam_fovy[self.camera_id])) * 0.5)
        info.k = [focal, 0.0, self.width * 0.5, 0.0, focal, self.height * 0.5, 0.0, 0.0, 1.0]
        info.p = [focal, 0.0, self.width * 0.5, 0.0, 0.0, focal, self.height * 0.5, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info)

    def image(self, array, encoding, stamp, step):
        msg = Image()
        msg.header.stamp, msg.header.frame_id = stamp, self.frame_id
        msg.height, msg.width, msg.encoding = array.shape[0], array.shape[1], encoding
        msg.is_bigendian, msg.step, msg.data = 0, step, array.tobytes()
        return msg

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
