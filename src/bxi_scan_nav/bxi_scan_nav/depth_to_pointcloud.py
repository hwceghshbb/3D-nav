import struct

import rclpy
from rcl_interfaces.msg import ParameterType, ParameterValue
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
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
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(
            PointCloud2, self.get_parameter("pointcloud_topic").value, qos
        )
        self.alias_publishers = [
            self.create_publisher(PointCloud2, topic, qos)
            for topic in self.get_parameter("pointcloud_topic_aliases").value
            if topic
        ]
        self.create_subscription(
            CameraInfo, self.get_parameter("camera_info_topic").value, self.info_callback, qos
        )
        self.create_subscription(
            Image, self.get_parameter("depth_topic").value, self.depth_callback, qos
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
        bytes_per_pixel = 4 if is_float else 2
        points = bytearray()
        for v in range(0, msg.height, stride):
            for u in range(0, msg.width, stride):
                offset = v * msg.step + u * bytes_per_pixel
                raw = msg.data[offset:offset + bytes_per_pixel]
                if len(raw) != bytes_per_pixel:
                    continue
                depth = struct.unpack_from("<f" if is_float else "<H", raw)[0]
                z = depth if is_float else depth * 0.001
                if z <= 0.15 or z > 12.0:
                    continue
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                if invert_x:
                    x = -x
                if invert_y:
                    y = -y
                points.extend(struct.pack("<fff", x, y, z))
        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.height = 1
        cloud.width = len(points) // 12
        cloud.is_dense = False
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = len(points)
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.data = points
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
