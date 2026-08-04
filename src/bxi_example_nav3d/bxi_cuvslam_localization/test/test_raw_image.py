from array import array

from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image

from bxi_cuvslam_localization.raw_image import parse_serialized_image


def test_parse_serialized_image_without_copying_payload():
    message = Image()
    message.header.stamp.sec = 123
    message.header.stamp.nanosec = 456
    message.header.frame_id = "camera_optical_frame"
    message.height = 2
    message.width = 3
    message.encoding = "16UC1"
    message.is_bigendian = 0
    message.step = 6
    message.data = array("B", range(12))

    parsed = parse_serialized_image(serialize_message(message))

    assert parsed.stamp_ns == 123_000_000_456
    assert parsed.frame_id == message.header.frame_id
    assert parsed.height == message.height
    assert parsed.width == message.width
    assert parsed.encoding == message.encoding
    assert parsed.step == message.step
    assert bytes(parsed.data) == bytes(message.data)


def test_rejects_truncated_payload():
    message = Image(height=1, width=1, encoding="mono8", step=1)
    message.data = array("B", [42])
    serialized = serialize_message(message)

    try:
        parse_serialized_image(serialized[:-1])
    except ValueError as error:
        assert "truncated Image payload" in str(error)
    else:
        raise AssertionError("truncated payload was accepted")
