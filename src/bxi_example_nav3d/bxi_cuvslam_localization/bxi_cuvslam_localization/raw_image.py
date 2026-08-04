from dataclasses import dataclass
import struct


@dataclass(frozen=True)
class RawImage:
    stamp_ns: int
    frame_id: str
    height: int
    width: int
    encoding: str
    is_bigendian: int
    step: int
    data: memoryview


def _align(offset, alignment):
    return (offset + alignment - 1) & ~(alignment - 1)


def parse_serialized_image(serialized):
    """Parse sensor_msgs/Image CDR without copying its pixel payload."""
    view = memoryview(serialized)
    if len(view) < 4:
        raise ValueError("serialized Image is shorter than the CDR header")

    representation = int.from_bytes(view[:2], byteorder="big")
    if representation not in (0, 1):
        raise ValueError(f"unsupported CDR representation 0x{representation:04x}")
    endian = "<" if representation == 1 else ">"
    offset = 4

    def unpack(fmt, alignment):
        nonlocal offset
        offset = _align(offset, alignment)
        size = struct.calcsize(fmt)
        if offset + size > len(view):
            raise ValueError("truncated serialized Image")
        value = struct.unpack_from(endian + fmt, view, offset)[0]
        offset += size
        return value

    def string():
        nonlocal offset
        size = unpack("I", 4)
        if size == 0 or offset + size > len(view):
            raise ValueError("invalid string in serialized Image")
        value = bytes(view[offset : offset + size - 1]).decode("utf-8")
        if view[offset + size - 1] != 0:
            raise ValueError("unterminated string in serialized Image")
        offset += size
        return value

    sec = unpack("i", 4)
    nanosec = unpack("I", 4)
    frame_id = string()
    height = unpack("I", 4)
    width = unpack("I", 4)
    encoding = string()
    is_bigendian = unpack("B", 1)
    step = unpack("I", 4)
    data_size = unpack("I", 4)
    if offset + data_size > len(view):
        raise ValueError(
            f"truncated Image payload: expected {data_size}, have {len(view) - offset}"
        )
    data = view[offset : offset + data_size]
    return RawImage(
        stamp_ns=sec * 1_000_000_000 + nanosec,
        frame_id=frame_id,
        height=height,
        width=width,
        encoding=encoding,
        is_bigendian=is_bigendian,
        step=step,
        data=data,
    )
