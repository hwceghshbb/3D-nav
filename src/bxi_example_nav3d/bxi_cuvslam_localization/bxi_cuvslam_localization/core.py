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


def gravity_tilt_error(reference_quaternion, current_quaternion):
    """Return the angle between the reference and current body Z axes."""
    reference_up = quaternion_rotate(reference_quaternion, (0.0, 0.0, 1.0))
    current_up = quaternion_rotate(current_quaternion, (0.0, 0.0, 1.0))
    reference_norm = math.sqrt(sum(value * value for value in reference_up))
    current_norm = math.sqrt(sum(value * value for value in current_up))
    if reference_norm <= 1e-9 or current_norm <= 1e-9:
        return math.inf
    cosine = sum(
        reference * current
        for reference, current in zip(reference_up, current_up)
    ) / (reference_norm * current_norm)
    return math.acos(max(-1.0, min(1.0, cosine)))


def flat_floor_pose_is_plausible(
    reference_position,
    reference_quaternion,
    current_position,
    current_quaternion,
    max_z_drift,
    max_tilt,
):
    return (
        abs(float(current_position[2]) - float(reference_position[2]))
        <= float(max_z_drift)
        and gravity_tilt_error(reference_quaternion, current_quaternion)
        <= float(max_tilt)
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


def pose_increment_is_plausible(
    previous_position,
    previous_quaternion,
    previous_stamp_ns,
    current_position,
    current_quaternion,
    current_stamp_ns,
    max_translation_jump,
    max_rotation_jump,
    max_linear_speed,
    max_angular_speed,
):
    """Check continuity without assuming that pose covariance is meaningful."""
    dt = (int(current_stamp_ns) - int(previous_stamp_ns)) / 1e9
    if dt <= 0.0:
        return False
    translation, rotation = pose_error(
        current_position,
        current_quaternion,
        previous_position,
        previous_quaternion,
    )
    translation_limit = min(
        float(max_translation_jump), float(max_linear_speed) * dt + 0.05
    )
    rotation_limit = min(
        float(max_rotation_jump), float(max_angular_speed) * dt + 0.05
    )
    return translation <= translation_limit and rotation <= rotation_limit


def pop_synchronized_timestamps(first_stamps, second_stamps, limit_ms):
    """Pop the oldest match, discarding only frames that can no longer pair."""
    limit_ns = max(0, int(float(limit_ms) * 1e6))
    dropped = False
    while first_stamps and second_stamps:
        first = int(first_stamps[0])
        second = int(second_stamps[0])
        if abs(first - second) <= limit_ns:
            first_stamps.popleft()
            second_stamps.popleft()
            return (first, second), dropped
        if first < second:
            first_stamps.popleft()
        else:
            second_stamps.popleft()
        dropped = True
    return None, dropped
