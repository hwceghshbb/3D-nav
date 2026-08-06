import math
from typing import Iterable, Mapping, Optional, Sequence, Tuple


def stamp_to_nanoseconds(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def timestamps_within_limit(timestamps_ns: Sequence[int], limit_ms: float) -> bool:
    if not timestamps_ns:
        return False
    return max(timestamps_ns) - min(timestamps_ns) <= int(limit_ms * 1_000_000.0)


def max_timestamp_delta_ms(timestamps_ns: Sequence[int]) -> float:
    if not timestamps_ns:
        return math.inf
    return (max(timestamps_ns) - min(timestamps_ns)) / 1_000_000.0


def head_lock_error(
    positions: Mapping[str, float],
    targets: Mapping[str, float],
) -> Optional[Tuple[str, float]]:
    worst = None
    for name, target in targets.items():
        if name not in positions or not math.isfinite(float(positions[name])):
            return name, math.inf
        error = abs(float(positions[name]) - float(target))
        if worst is None or error > worst[1]:
            worst = (name, error)
    return worst


def quaternion_is_valid(values: Iterable[float], norm_tolerance: float = 0.05) -> bool:
    values = tuple(float(value) for value in values)
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        return False
    norm = math.sqrt(sum(value * value for value in values))
    return norm > 1e-6 and abs(norm - 1.0) <= norm_tolerance


def covariance_is_acceptable(
    covariance: Sequence[float],
    max_position_variance: float,
    max_orientation_variance: float,
) -> bool:
    if len(covariance) != 36:
        return False
    diagonal = [float(covariance[index]) for index in (0, 7, 14, 21, 28, 35)]
    if not all(math.isfinite(value) and value >= 0.0 for value in diagonal):
        return False
    return (
        max(diagonal[:3]) <= max_position_variance
        and max(diagonal[3:]) <= max_orientation_variance
    )


def quaternion_multiply(left, right):
    lx, ly, lz, lw = (float(value) for value in left)
    rx, ry, rz, rw = (float(value) for value in right)
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def quaternion_rotate(quaternion, vector):
    qx, qy, qz, qw = (float(value) for value in quaternion)
    vx, vy, vz = (float(value) for value in vector)
    # q * [v, 0] * conjugate(q), expanded to avoid an extra dependency.
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def initial_pose_alignment(source_position, source_quaternion, target_position):
    """Return a transform that anchors the first base pose upright at target_position."""
    source_inverse = (
        -float(source_quaternion[0]),
        -float(source_quaternion[1]),
        -float(source_quaternion[2]),
        float(source_quaternion[3]),
    )
    rotated_source = quaternion_rotate(source_inverse, source_position)
    translation = tuple(
        float(target) - value
        for target, value in zip(target_position, rotated_source)
    )
    return translation, source_inverse


def transform_pose(alignment_translation, alignment_quaternion, position, quaternion):
    rotated = quaternion_rotate(alignment_quaternion, position)
    return (
        tuple(
            float(offset) + value
            for offset, value in zip(alignment_translation, rotated)
        ),
        quaternion_multiply(alignment_quaternion, quaternion),
    )


def pose_error(estimate_position, estimate_quaternion, truth_position, truth_quaternion):
    position_delta = [
        float(estimate) - float(truth)
        for estimate, truth in zip(estimate_position, truth_position)
    ]
    translation_error = math.sqrt(sum(value * value for value in position_delta))

    estimate_quaternion = tuple(float(value) for value in estimate_quaternion)
    truth_quaternion = tuple(float(value) for value in truth_quaternion)
    estimate_norm = math.sqrt(sum(value * value for value in estimate_quaternion))
    truth_norm = math.sqrt(sum(value * value for value in truth_quaternion))
    if estimate_norm <= 1e-9 or truth_norm <= 1e-9:
        return translation_error, math.inf
    dot = sum(
        estimate * truth
        for estimate, truth in zip(estimate_quaternion, truth_quaternion)
    ) / (estimate_norm * truth_norm)
    # q and -q encode the same rotation.
    dot = max(-1.0, min(1.0, abs(dot)))
    return translation_error, 2.0 * math.acos(dot)
