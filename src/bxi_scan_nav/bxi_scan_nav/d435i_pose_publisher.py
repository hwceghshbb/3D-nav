import math

from geometry_msgs.msg import TransformStamped
import nav_msgs.msg
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


def quaternion_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def rotate_vector(q, v):
    conjugate = (q[0], -q[1], -q[2], -q[3])
    rotated = quaternion_multiply(quaternion_multiply(q, (0.0, *v)), conjugate)
    return rotated[1], rotated[2], rotated[3]


def matrix_to_quaternion(matrix):
    m00, m01, m02 = matrix[0]
    m10, m11, m12 = matrix[1]
    m20, m21, m22 = matrix[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return (0.25 * scale, (m21 - m12) / scale, (m02 - m20) / scale, (m10 - m01) / scale)
    if m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        return ((m21 - m12) / scale, 0.25 * scale, (m01 + m10) / scale, (m02 + m20) / scale)
    if m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        return ((m02 - m20) / scale, (m01 + m10) / scale, 0.25 * scale, (m12 + m21) / scale)
    scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
    return ((m10 - m01) / scale, (m02 + m20) / scale, (m12 + m21) / scale, 0.25 * scale)


def normalize(q):
    norm = math.sqrt(sum(value * value for value in q))
    if norm <= 1e-9:
        return 1.0, 0.0, 0.0, 0.0
    return tuple(value / norm for value in q)


def quaternion_to_yaw(quaternion):
    w, x, y, z = quaternion
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_quaternion(yaw):
    half_yaw = 0.5 * yaw
    return math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)


def inverse_quaternion(quaternion):
    w, x, y, z = normalize(quaternion)
    return w, -x, -y, -z


class D435iPosePublisher(Node):
    def __init__(self):
        super().__init__("d435i_pose_publisher")
        self.declare_parameter("odom_topic", "/simulation/odom")
        self.declare_parameter("base_pose_topic", "/simulation/base/pose")
        self.declare_parameter("base_footprint_pose_topic", "/simulation/base_footprint/pose")
        self.declare_parameter("sensor_pose_topic", "/simulation/d435i/depth/pose")
        self.declare_parameter("world_frame_id", "world")
        self.declare_parameter("base_footprint_frame_id", "bxi_base_footprint")
        self.declare_parameter("base_frame_id", "bxi_base_link")
        self.declare_parameter("sensor_frame_id", "d435i_depth_optical_frame")
        self.declare_parameter("offset_x", 0.16)
        self.declare_parameter("offset_y", 0.0)
        self.declare_parameter("offset_z", 0.25)
        self.declare_parameter("preserve_base_footprint_z", False)
        self.declare_parameter("base_footprint_z_offset", 0.0)
        self.declare_parameter("stamp_with_now", True)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_hz", 30.0)
        publish_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_publish_time = None
        self.base_publisher = self.create_publisher(
            nav_msgs.msg.Odometry,
            self.get_parameter("base_pose_topic").value,
            publish_qos,
        )
        self.base_footprint_publisher = self.create_publisher(
            nav_msgs.msg.Odometry,
            self.get_parameter("base_footprint_pose_topic").value,
            publish_qos,
        )
        self.publisher = self.create_publisher(
            nav_msgs.msg.Odometry,
            self.get_parameter("sensor_pose_topic").value,
            publish_qos,
        )
        self.create_subscription(
            nav_msgs.msg.Odometry,
            self.get_parameter("odom_topic").value,
            self.odom_callback,
            qos_profile_sensor_data,
        )
        optical_axes_in_body = (
            (0.0, 0.0, 1.0),
            (-1.0, 0.0, 0.0),
            (0.0, -1.0, 0.0),
        )
        self.optical_quat_in_body = normalize(matrix_to_quaternion(optical_axes_in_body))

    def odom_callback(self, msg):
        now = self.get_clock().now()
        publish_hz = max(float(self.get_parameter("publish_hz").value), 1.0)
        if self.last_publish_time is not None:
            elapsed = (now - self.last_publish_time).nanoseconds * 1e-9
            if elapsed < 1.0 / publish_hz:
                return
        self.last_publish_time = now
        body_q = normalize((
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        ))
        offset = (
            float(self.get_parameter("offset_x").value),
            float(self.get_parameter("offset_y").value),
            float(self.get_parameter("offset_z").value),
        )
        offset_world = rotate_vector(body_q, offset)
        sensor_q = normalize(quaternion_multiply(body_q, self.optical_quat_in_body))
        world_frame = str(self.get_parameter("world_frame_id").value)
        base_footprint_frame = str(self.get_parameter("base_footprint_frame_id").value)
        base_frame = str(self.get_parameter("base_frame_id").value)
        sensor_frame = str(self.get_parameter("sensor_frame_id").value)
        stamp = now.to_msg() if bool(self.get_parameter("stamp_with_now").value) else msg.header.stamp
        yaw = quaternion_to_yaw(body_q)
        footprint_q = yaw_to_quaternion(yaw)
        base_in_footprint_q = normalize(quaternion_multiply(inverse_quaternion(footprint_q), body_q))
        if bool(self.get_parameter("preserve_base_footprint_z").value):
            footprint_z = msg.pose.pose.position.z + float(self.get_parameter("base_footprint_z_offset").value)
        else:
            footprint_z = 0.0

        footprint_pose = nav_msgs.msg.Odometry()
        footprint_pose.header.stamp = stamp
        footprint_pose.header.frame_id = world_frame
        footprint_pose.child_frame_id = base_footprint_frame
        footprint_pose.pose.pose.position.x = msg.pose.pose.position.x
        footprint_pose.pose.pose.position.y = msg.pose.pose.position.y
        footprint_pose.pose.pose.position.z = footprint_z
        footprint_pose.pose.pose.orientation.w = footprint_q[0]
        footprint_pose.pose.pose.orientation.x = footprint_q[1]
        footprint_pose.pose.pose.orientation.y = footprint_q[2]
        footprint_pose.pose.pose.orientation.z = footprint_q[3]
        footprint_pose.twist = msg.twist
        self.base_footprint_publisher.publish(footprint_pose)

        base_pose = nav_msgs.msg.Odometry()
        base_pose.header.stamp = stamp
        base_pose.header.frame_id = world_frame
        base_pose.child_frame_id = base_frame
        base_pose.pose = msg.pose
        base_pose.twist = msg.twist
        self.base_publisher.publish(base_pose)
        pose = nav_msgs.msg.Odometry()
        pose.header.stamp = stamp
        pose.header.frame_id = world_frame
        pose.child_frame_id = sensor_frame
        pose.pose.pose.position.x = msg.pose.pose.position.x + offset_world[0]
        pose.pose.pose.position.y = msg.pose.pose.position.y + offset_world[1]
        pose.pose.pose.position.z = msg.pose.pose.position.z + offset_world[2]
        pose.pose.pose.orientation.w = sensor_q[0]
        pose.pose.pose.orientation.x = sensor_q[1]
        pose.pose.pose.orientation.y = sensor_q[2]
        pose.pose.pose.orientation.z = sensor_q[3]
        self.publisher.publish(pose)
        if bool(self.get_parameter("publish_tf").value):
            footprint_transform = TransformStamped()
            footprint_transform.header.stamp = stamp
            footprint_transform.header.frame_id = world_frame
            footprint_transform.child_frame_id = base_footprint_frame
            footprint_transform.transform.translation.x = msg.pose.pose.position.x
            footprint_transform.transform.translation.y = msg.pose.pose.position.y
            footprint_transform.transform.translation.z = footprint_z
            footprint_transform.transform.rotation.w = footprint_q[0]
            footprint_transform.transform.rotation.x = footprint_q[1]
            footprint_transform.transform.rotation.y = footprint_q[2]
            footprint_transform.transform.rotation.z = footprint_q[3]
            base_transform = TransformStamped()
            base_transform.header.stamp = stamp
            base_transform.header.frame_id = base_footprint_frame
            base_transform.child_frame_id = base_frame
            base_transform.transform.translation.x = 0.0
            base_transform.transform.translation.y = 0.0
            base_transform.transform.translation.z = msg.pose.pose.position.z - footprint_z
            base_transform.transform.rotation.w = base_in_footprint_q[0]
            base_transform.transform.rotation.x = base_in_footprint_q[1]
            base_transform.transform.rotation.y = base_in_footprint_q[2]
            base_transform.transform.rotation.z = base_in_footprint_q[3]
            sensor_transform = TransformStamped()
            sensor_transform.header.stamp = stamp
            sensor_transform.header.frame_id = base_frame
            sensor_transform.child_frame_id = sensor_frame
            sensor_transform.transform.translation.x = offset[0]
            sensor_transform.transform.translation.y = offset[1]
            sensor_transform.transform.translation.z = offset[2]
            sensor_transform.transform.rotation.w = self.optical_quat_in_body[0]
            sensor_transform.transform.rotation.x = self.optical_quat_in_body[1]
            sensor_transform.transform.rotation.y = self.optical_quat_in_body[2]
            sensor_transform.transform.rotation.z = self.optical_quat_in_body[3]
            self.tf_broadcaster.sendTransform([footprint_transform, base_transform, sensor_transform])


def main(args=None):
    rclpy.init(args=args)
    node = D435iPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
