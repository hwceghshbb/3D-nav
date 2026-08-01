import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


class ScanPlannerAdapter(Node):
    """Topic contract between BXI ROS2 and the SCAN-Planner ROS interface."""

    def __init__(self):
        super().__init__("scan_planner_adapter")
        self.declare_parameter("odom_in", "/simulation/odom")
        self.declare_parameter("odom_out", "/scan_planner/odom")
        self.declare_parameter("cloud_in", "/scan_planner/local_cloud")
        self.declare_parameter("cloud_out", "/scan_planner/depth_cloud")
        self.odom_pub = self.create_publisher(Odometry, self.get_parameter("odom_out").value, qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(PointCloud2, self.get_parameter("cloud_out").value, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.get_parameter("odom_in").value, self.odom_pub.publish, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.get_parameter("cloud_in").value, self.cloud_pub.publish, qos_profile_sensor_data)


def main(args=None):
    rclpy.init(args=args)
    node = ScanPlannerAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
