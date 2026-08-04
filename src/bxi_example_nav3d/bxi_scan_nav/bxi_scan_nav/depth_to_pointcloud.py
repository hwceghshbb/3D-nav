from array import array as byte_array

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterType, ParameterValue
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


class DepthToPointCloud(Node):
    def __init__(self):
        super().__init__("depth_to_pointcloud")
        self.declare_parameter("depth_topic", "/simulation/d435i/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/simulation/d435i/depth/camera_info")
        self.declare_parameter("pointcloud_topic", "/scan_planner/local_cloud")
        self.declare_parameter(
            "pointcloud_topic_aliases",
            ParameterValue(type=ParameterType.PARAMETER_STRING_ARRAY, string_array_value=[]),
        )
        self.declare_parameter("stride", 4)
        self.declare_parameter("invert_x", False)
        self.declare_parameter("invert_y", False)
        self.info = None
        cloud_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(
            PointCloud2, self.get_parameter("pointcloud_topic").value, cloud_qos
        )
        self.alias_publishers = [
            self.create_publisher(PointCloud2, topic, cloud_qos)
            for topic in self.get_parameter("pointcloud_topic_aliases").value
            if topic
        ]
        self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self.info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.get_parameter("depth_topic").value,
            self.depth_callback,
            qos_profile_sensor_data,
        )

    def info_callback(self, msg):
        self.info = msg

    def depth_callback(self, msg):
        if self.info is None or msg.encoding not in ("16UC1", "32FC1"):
            return
        stride = max(1, int(self.get_parameter("stride").value))
        invert_x = bool(self.get_parameter("invert_x").value)
        invert_y = bool(self.get_parameter("invert_y").value)
        fx, fy, cx, cy = self.info.k[0], self.info.k[4], self.info.k[2], self.info.k[5]
        is_float = msg.encoding == "32FC1"
        dtype = np.float32 if is_float else np.uint16
        row_values = msg.step // np.dtype(dtype).itemsize
        depth_rows = np.frombuffer(msg.data, dtype=dtype).reshape(
            msg.height, row_values
        )
        z = depth_rows[::stride, : msg.width : stride].astype(np.float32)
        if not is_float:
            z *= 0.001

        u = np.arange(0, msg.width, stride, dtype=np.float32)
        v = np.arange(0, msg.height, stride, dtype=np.float32)
        x = (u[np.newaxis, :] - cx) * z / fx
        y = (v[:, np.newaxis] - cy) * z / fy
        if invert_x:
            x *= -1.0
        if invert_y:
            y *= -1.0
        valid = np.isfinite(z) & (z > 0.15) & (z <= 12.0)
        points = np.column_stack((x[valid], y[valid], z[valid])).astype(
            np.float32, copy=False
        )
        point_bytes = points.nbytes
        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.height = 1
        cloud.width = points.shape[0]
        cloud.is_dense = False
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = point_bytes
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.data = byte_array("B", points.tobytes())
        self.publisher.publish(cloud)
        for publisher in self.alias_publishers:
            publisher.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = DepthToPointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
