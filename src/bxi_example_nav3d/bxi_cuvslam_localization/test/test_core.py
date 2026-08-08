import math
from collections import deque

import numpy as np

from bxi_cuvslam_localization.core import (
    covariance_is_acceptable,
    flat_floor_pose_is_plausible,
    gravity_tilt_error,
    head_lock_error,
    max_timestamp_delta_ms,
    initial_pose_alignment,
    pose_error,
    pose_increment_is_plausible,
    pop_synchronized_timestamps,
    quaternion_is_valid,
    timestamps_within_limit,
    transform_pose,
)
from bxi_cuvslam_localization.head_cuvslam_node import compose_pose, inverse_pose


def test_camera_sync_accepts_five_milliseconds_inclusive():
    stamps = [1_000_000_000, 1_001_000_000, 1_004_000_000, 1_005_000_000]
    assert timestamps_within_limit(stamps, 5.0)
    assert max_timestamp_delta_ms(stamps) == 5.0


def test_camera_sync_rejects_late_camera():
    stamps = [1_000_000_000, 1_001_000_000, 1_004_000_000, 1_005_000_001]
    assert not timestamps_within_limit(stamps, 5.0)


def test_head_lock_checks_both_joints():
    error = head_lock_error(
        {"head_z_joint": 0.002, "head_y_joint": -0.012},
        {"head_z_joint": 0.0, "head_y_joint": 0.0},
    )
    assert error == ("head_y_joint", 0.012)


def test_head_lock_reports_missing_joint():
    name, error = head_lock_error(
        {"head_z_joint": 0.0},
        {"head_z_joint": 0.0, "head_y_joint": 0.0},
    )
    assert name == "head_y_joint"
    assert math.isinf(error)


def test_quaternion_validation():
    assert quaternion_is_valid((0.0, 0.0, 0.0, 1.0))
    assert not quaternion_is_valid((0.0, 0.0, 0.0, 0.0))
    assert not quaternion_is_valid((0.0, 0.0, 0.0, 1.2))


def test_flat_floor_guard_ignores_yaw_but_rejects_tilt_and_z_drift():
    identity = (0.0, 0.0, 0.0, 1.0)
    yaw = (0.0, 0.0, math.sin(0.5), math.cos(0.5))
    pitch = (0.0, math.sin(0.15), 0.0, math.cos(0.15))
    assert math.isclose(gravity_tilt_error(identity, yaw), 0.0)
    assert flat_floor_pose_is_plausible(
        (0.0, 0.0, 1.1), identity,
        (4.0, -3.0, 1.15), yaw,
        0.12, 0.20,
    )
    assert not flat_floor_pose_is_plausible(
        (0.0, 0.0, 1.1), identity,
        (0.0, 0.0, 1.25), identity,
        0.12, 0.20,
    )
    assert not flat_floor_pose_is_plausible(
        (0.0, 0.0, 1.1), identity,
        (0.0, 0.0, 1.1), pitch,
        0.12, 0.20,
    )


def test_covariance_thresholds():
    covariance = [0.0] * 36
    covariance[0] = covariance[7] = covariance[14] = 0.1
    covariance[21] = covariance[28] = covariance[35] = 0.2
    assert covariance_is_acceptable(covariance, 0.1, 0.2)
    covariance[14] = 0.11
    assert not covariance_is_acceptable(covariance, 0.1, 0.2)


def test_pose_error_handles_quaternion_sign_and_translation():
    translation, rotation = pose_error(
        [1.3, 2.4, 3.0],
        [0.0, 0.0, 0.0, -1.0],
        [1.0, 2.0, 3.0],
        [0.0, 0.0, 0.0, 1.0],
    )
    assert math.isclose(translation, 0.5)
    assert math.isclose(rotation, 0.0)


def test_pose_error_reports_rotation_angle():
    _, rotation = pose_error(
        [0.0, 0.0, 0.0],
        [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    )
    assert math.isclose(rotation, math.pi / 2.0)


def test_pose_increment_accepts_normal_robot_motion():
    assert pose_increment_is_plausible(
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        1_000_000_000,
        [0.20, 0.0, 0.0],
        [0.0, 0.0, math.sin(0.1), math.cos(0.1)],
        1_500_000_000,
        0.30,
        0.30,
        1.0,
        1.0,
    )


def test_pose_increment_rejects_jump_and_non_monotonic_stamp():
    common = (
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        1_000_000_000,
    )
    limits = (0.30, 0.30, 1.0, 1.0)
    assert not pose_increment_is_plausible(
        *common,
        [2.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        1_100_000_000,
        *limits,
    )
    assert not pose_increment_is_plausible(*common, *common, *limits)


def test_timestamp_pairing_waits_for_the_other_stream():
    colors = deque([1_000_000_000])
    depths = deque()
    pair, dropped = pop_synchronized_timestamps(colors, depths, 2.0)
    assert pair is None
    assert not dropped
    assert list(colors) == [1_000_000_000]

    depths.append(1_001_000_000)
    pair, dropped = pop_synchronized_timestamps(colors, depths, 2.0)
    assert pair == (1_000_000_000, 1_001_000_000)
    assert not dropped


def test_timestamp_pairing_discards_frames_outside_the_window():
    colors = deque([1_000_000_000, 1_010_000_000])
    depths = deque([1_010_500_000])
    pair, dropped = pop_synchronized_timestamps(colors, depths, 2.0)
    assert pair == (1_010_000_000, 1_010_500_000)
    assert dropped


def test_initial_pose_alignment_sets_base_height_and_preserves_motion():
    source_position = [-0.0628, -0.0175, -0.2515]
    source_quaternion = [0.0, 0.0, 0.0, 1.0]
    alignment = initial_pose_alignment(
        source_position, source_quaternion, [0.0, 0.0, 1.1]
    )
    anchored_position, anchored_quaternion = transform_pose(
        *alignment, source_position, source_quaternion
    )
    assert np.allclose(anchored_position, [0.0, 0.0, 1.1])
    assert np.allclose(anchored_quaternion, [0.0, 0.0, 0.0, 1.0])

    moved_position, _ = transform_pose(
        *alignment, [source_position[0] + 2.0, source_position[1], source_position[2]], source_quaternion
    )
    assert np.allclose(moved_position, [2.0, 0.0, 1.1])


def test_map_origin_pose_composition():
    translation, quaternion = compose_pose(
        [1.0, 2.0, 3.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.5, -0.5, 1.0],
        [0.0, 0.0, 0.0, 1.0],
    )
    assert np.allclose(translation, [1.5, 1.5, 4.0])
    assert np.allclose(quaternion, [0.0, 0.0, 0.0, 1.0])


def test_pose_inverse_recovers_local_pose():
    origin = ([3.0, -2.0, 1.0], [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)])
    map_pose = compose_pose(*origin, [0.4, 0.2, -0.1], [0.0, 0.0, 0.0, 1.0])
    recovered = compose_pose(*inverse_pose(*origin), *map_pose)
    assert np.allclose(recovered[0], [0.4, 0.2, -0.1])
    assert np.allclose(recovered[1], [0.0, 0.0, 0.0, 1.0])
