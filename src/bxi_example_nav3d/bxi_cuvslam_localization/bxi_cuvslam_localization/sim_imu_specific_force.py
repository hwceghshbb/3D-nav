from copy import deepcopy
import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from .core import stamp_to_nanoseconds


def gravity_in_body_frame(quaternion, gravity_mps2):
    x, y, z, w = (float(value) for value in quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-9:
        raise ValueError("invalid body orientation quaternion")
    x, y, z, w = (value / norm for value in (x, y, z, w))
    # The odometry quaternion is R_world_body. Specific force at rest is
    # R_world_body^T * [0, 0, g].
    return (
        gravity_mps2 * 2.0 * (x * z - y * w),
        gravity_mps2 * 2.0 * (y * z + x * w),
        gravity_mps2 * (1.0 - 2.0 * (x * x + y * y)),
    )


class SimImuSpecificForce(Node):
    def __init__(self):
        super().__init__("sim_imu_specific_force")
        self.declare_parameter("input_imu_topic", "/simulation/imu_data")
        self.declare_parameter("truth_odom_topic", "/simulation/odom")
        self.declare_parameter("output_imu_topic", "/nav/simulation/imu_specific_force")
        self.declare_parameter("gravity_mps2", 9.80665)
        self.declare_parameter("max_pose_age_sec", 0.02)

        self.latest_pose = None
        self.publisher = self.create_publisher(
            Imu,
            str(self.get_parameter("output_imu_topic").value),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("truth_odom_topic").value),
            self.odom_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter("input_imu_topic").value),
            self.imu_callback,
            qos_profile_sensor_data,
        )
        self.published = 0
        self.dropped = 0
        self.create_timer(5.0, self.report)

    def odom_callback(self, message):
        orientation = message.pose.pose.orientation
        self.latest_pose = (
            stamp_to_nanoseconds(message.header.stamp),
            (orientation.x, orientation.y, orientation.z, orientation.w),
        )

    def imu_callback(self, message):
        if self.latest_pose is None:
            self.dropped += 1
            return
        pose_stamp_ns, orientation = self.latest_pose
        imu_stamp_ns = stamp_to_nanoseconds(message.header.stamp)
        max_age_ns = int(
            float(self.get_parameter("max_pose_age_sec").value) * 1e9
        )
        if abs(imu_stamp_ns - pose_stamp_ns) > max_age_ns:
            self.dropped += 1
            return
        try:
            gravity = gravity_in_body_frame(
                orientation,
                float(self.get_parameter("gravity_mps2").value),
            )
        except ValueError:
            self.dropped += 1
            return
        output = deepcopy(message)
        output.linear_acceleration.x += gravity[0]
        output.linear_acceleration.y += gravity[1]
        output.linear_acceleration.z += gravity[2]
        self.publisher.publish(output)
        self.published += 1

    def report(self):
        self.get_logger().info(
            "Simulation IMU specific-force adapter: published=%d dropped=%d"
            % (self.published, self.dropped)
        )


def main(args=None):
    rclpy.init(args=args)
    node = SimImuSpecificForce()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
