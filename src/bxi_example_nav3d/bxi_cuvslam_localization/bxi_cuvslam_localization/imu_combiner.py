from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from .core import stamp_to_nanoseconds


class ImuCombiner(Node):
    def __init__(self):
        super().__init__("cuvslam_imu_combiner")
        self.declare_parameter(
            "gyro_topic", "/hardware/body_depth_camera/gyro/sample"
        )
        self.declare_parameter(
            "accel_topic", "/hardware/body_depth_camera/accel/sample"
        )
        self.declare_parameter("output_topic", "/nav/imu")
        self.declare_parameter("output_frame_id", "body_depth_camera_imu_frame")
        self.declare_parameter("max_accel_age_ms", 20.0)
        self.declare_parameter("gyro_variance", 5.9536e-8)
        self.declare_parameter("accel_variance", 3.467e-6)

        self.accel = None
        self.publisher = self.create_publisher(
            Imu, self.get_parameter("output_topic").value, qos_profile_sensor_data
        )
        self.create_subscription(
            Imu,
            self.get_parameter("accel_topic").value,
            self.accel_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            self.get_parameter("gyro_topic").value,
            self.gyro_callback,
            qos_profile_sensor_data,
        )

    def accel_callback(self, message):
        self.accel = message

    def gyro_callback(self, gyro):
        if self.accel is None:
            return
        age_ns = abs(
            stamp_to_nanoseconds(gyro.header.stamp)
            - stamp_to_nanoseconds(self.accel.header.stamp)
        )
        max_age_ns = int(
            float(self.get_parameter("max_accel_age_ms").value) * 1_000_000.0
        )
        if age_ns > max_age_ns:
            self.get_logger().warning(
                f"Skipping IMU sample: gyro/accel delta={age_ns / 1e6:.2f} ms",
                throttle_duration_sec=2.0,
            )
            return

        message = Imu()
        message.header = deepcopy(gyro.header)
        message.header.frame_id = str(self.get_parameter("output_frame_id").value)
        message.orientation_covariance[0] = -1.0
        message.angular_velocity = deepcopy(gyro.angular_velocity)
        message.linear_acceleration = deepcopy(self.accel.linear_acceleration)
        gyro_variance = float(self.get_parameter("gyro_variance").value)
        accel_variance = float(self.get_parameter("accel_variance").value)
        for index in (0, 4, 8):
            message.angular_velocity_covariance[index] = gyro_variance
            message.linear_acceleration_covariance[index] = accel_variance
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ImuCombiner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
