from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# The values below are URDF-derived initial estimates at head_y=head_z=0.
# Replace them with calibrated base-to-imager extrinsics before real navigation.
HEAD_CAMERA_TRANSFORM = (0.0628, 0.0175, 0.2515, -1.5708, 0.0, -1.5708)
BODY_CAMERA_TRANSFORM = (0.1152, 0.0175, -0.1358, -2.7925, 0.0, -1.5708)

# Factory Depth -> Color calibration for robot D435I serial 261722072591.
# `rs-enumerate-devices -c` reports the transform mapping depth-frame points
# into the color frame, which is T_color_depth in the TF tree below.
ROBOT_DEPTH_TO_COLOR = (
    0.0150657101,
    -0.0000313659,
    -0.0001458338,
    0.0090394825,
    0.0037685111,
    -0.0032921457,
    0.9999466225,
)

CAMERA_TRANSFORMS = (
    # MuJoCo labels the body IMU messages with the model name. The simulated
    # IMU is body-fixed, so expose that label as an alias of the navigation base.
    ("elf3", (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
    (
        "head_depth_camera_color_optical_frame",
        HEAD_CAMERA_TRANSFORM,
    ),
    (
        "head_depth_camera_depth_optical_frame",
        HEAD_CAMERA_TRANSFORM,
    ),
    (
        "body_depth_camera_color_optical_frame",
        BODY_CAMERA_TRANSFORM,
    ),
    (
        "body_depth_camera_imu_frame",
        BODY_CAMERA_TRANSFORM,
    ),
    (
        "body_depth_camera_imu_optical_frame",
        BODY_CAMERA_TRANSFORM,
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


def depth_to_color_node(context):
    values = [
        LaunchConfiguration(name).perform(context)
        for name in (
            "depth_to_color_x",
            "depth_to_color_y",
            "depth_to_color_z",
            "depth_to_color_qx",
            "depth_to_color_qy",
            "depth_to_color_qz",
            "depth_to_color_qw",
        )
    ]
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_body_depth_camera_depth_optical_frame",
        output="screen",
        arguments=[
            "--x", values[0],
            "--y", values[1],
            "--z", values[2],
            "--qx", values[3],
            "--qy", values[4],
            "--qz", values[5],
            "--qw", values[6],
            "--frame-id", "body_depth_camera_color_optical_frame",
            "--child-frame-id", "body_depth_camera_depth_optical_frame",
        ],
    )


def launch_setup(context, *args, **kwargs):
    body_camera_uses_head_mount = (
        LaunchConfiguration("body_camera_uses_head_mount")
        .perform(context)
        .lower()
        in ("1", "true", "yes", "on")
    )
    nodes = []
    for child, transform in CAMERA_TRANSFORMS:
        if body_camera_uses_head_mount and child.startswith("body_depth_camera_"):
            transform = HEAD_CAMERA_TRANSFORM
        nodes.append(static_transform_node(child, transform))
    nodes.append(depth_to_color_node(context))
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "body_camera_uses_head_mount", default_value="false"
            ),
            DeclareLaunchArgument(
                "depth_to_color_x", default_value=str(ROBOT_DEPTH_TO_COLOR[0])
            ),
            DeclareLaunchArgument(
                "depth_to_color_y", default_value=str(ROBOT_DEPTH_TO_COLOR[1])
            ),
            DeclareLaunchArgument(
                "depth_to_color_z", default_value=str(ROBOT_DEPTH_TO_COLOR[2])
            ),
            DeclareLaunchArgument(
                "depth_to_color_qx", default_value=str(ROBOT_DEPTH_TO_COLOR[3])
            ),
            DeclareLaunchArgument(
                "depth_to_color_qy", default_value=str(ROBOT_DEPTH_TO_COLOR[4])
            ),
            DeclareLaunchArgument(
                "depth_to_color_qz", default_value=str(ROBOT_DEPTH_TO_COLOR[5])
            ),
            DeclareLaunchArgument(
                "depth_to_color_qw", default_value=str(ROBOT_DEPTH_TO_COLOR[6])
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
