import math

from bxi_cuvslam_localization.sim_imu_specific_force import gravity_in_body_frame


def test_identity_body_gets_positive_z_specific_force():
    gravity = gravity_in_body_frame((0.0, 0.0, 0.0, 1.0), 9.80665)
    assert gravity == (0.0, 0.0, 9.80665)


def test_body_roll_rotates_gravity_into_body_y():
    half_angle = math.pi / 4.0
    gravity = gravity_in_body_frame(
        (math.sin(half_angle), 0.0, 0.0, math.cos(half_angle)),
        9.80665,
    )
    assert math.isclose(gravity[0], 0.0, abs_tol=1e-9)
    assert math.isclose(gravity[1], 9.80665, abs_tol=1e-9)
    assert math.isclose(gravity[2], 0.0, abs_tol=1e-9)
