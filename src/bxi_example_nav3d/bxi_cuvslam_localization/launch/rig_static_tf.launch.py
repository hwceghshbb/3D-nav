from launch import LaunchDescription
from launch_ros.actions import Node


# The values below are URDF-derived initial estimates at head_y=head_z=0.
# Replace them with calibrated base-to-imager extrinsics before real navigation.
CAMERA_TRANSFORMS = (
    (
        "head_depth_camera_color_optical_frame",
        (0.0628, 0.0175, 0.2515, -1.5708, 0.0, -1.5708),
    ),
    (
        "head_depth_camera_depth_optical_frame",
        (0.0628, 0.0175, 0.2515, -1.5708, 0.0, -1.5708),
    ),
    (
        "body_depth_camera_color_optical_frame",
        (0.1152, 0.0175, -0.1358, -2.7925, 0.0, -1.5708),
    ),
    (
        "body_depth_camera_depth_optical_frame",
        (0.1152, 0.0175, -0.1358, -2.7925, 0.0, -1.5708),
    ),
    (
        "body_depth_camera_imu_frame",
        (0.1152, 0.0175, -0.1358, -2.7925, 0.0, -1.5708),
    ),
)


def static_transform_node(child_frame, transform):
    x, y, z, roll, pitch, yaw = transform
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"static_tf_{child_frame}",
        output="screen",
        arguments=[
            "--x", str(x),
            "--y", str(y),
            "--z", str(z),
            "--roll", str(roll),
            "--pitch", str(pitch),
            "--yaw", str(yaw),
            "--frame-id", "bxi_base_link",
            "--child-frame-id", child_frame,
        ],
    )


def generate_launch_description():
    return LaunchDescription(
        [static_transform_node(child, transform) for child, transform in CAMERA_TRANSFORMS]
    )
