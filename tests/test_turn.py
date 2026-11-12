"""Fast checks of the analytical reference and optional outer control loop."""

import numpy as np
import pytest

from sim.drive import Drive, YawRatePI
from sim.motor import KE
from sim.skid_model import steady_turn


def test_hand_computed_square_and_mu_cancellation():
    # Four corners at (+/-1,+/-1): at W=U/2=1, slips have equal x/y
    # magnitudes, and each corner's yaw moment is zero independently.
    xy = [[1, 1], [-1, 1], [1, -1], [-1, -1]]
    speeds = [-2, -2, 2, 2]
    for mu in (.3, .9):
        result = steady_turn(xy, [10]*4, mu, speeds, radius=.1,
                             torque_limits=[.3]*4)
        assert result["yaw_rate_rad_s"] == pytest.approx(1)
        assert result["net_force_body_N"] == pytest.approx([0, 0], abs=1e-10)
        assert result["yaw_moment_Nm"] == pytest.approx(0, abs=1e-9)
        assert np.abs(result["longitudinal_force_N"]) == pytest.approx([mu*10/np.sqrt(2)]*4)
        assert np.abs(result["axle_torque_Nm"]) == pytest.approx([mu/np.sqrt(2)]*4)
        assert result["exceeds_torque_limit"] == [mu == .9]*4


def test_asymmetric_loads_report_translation_and_reverse_symmetry():
    xy = [[1, 1], [-1, 1], [1, -1], [-1, -1]]
    forward = steady_turn(xy, [10, 20, 10, 10], .9, [-2, -2, 2, 2])
    reverse = steady_turn(xy, [10, 20, 10, 10], .9, [2, 2, -2, -2])
    assert np.linalg.norm(forward["net_force_body_N"]) > 0
    assert reverse["yaw_rate_rad_s"] == pytest.approx(-forward["yaw_rate_rad_s"])
    assert reverse["net_force_body_N"] == pytest.approx(-np.array(forward["net_force_body_N"]))


def test_zero_command_and_invalid_geometry():
    assert steady_turn([[1, 1], [-1, -1]], [10, 10], .5, [0, 0])["yaw_rate_rad_s"] == 0
    with pytest.raises(ValueError):
        steady_turn([[0, 0]], [10], .5, [1])


def test_prescribed_translation_is_relative_to_surface_speed():
    xy = [[1, 1], [-1, 1], [1, -1], [-1, -1]]
    speeds = np.array([-2, -2, 2, 2])
    fixed = steady_turn(xy, [10]*4, .5, speeds)
    moving = steady_turn(xy, [10]*4, .5, speeds + .3, translation_velocity=(.3, 0))
    assert moving["yaw_rate_rad_s"] == pytest.approx(fixed["yaw_rate_rad_s"])
    assert moving["longitudinal_force_N"] == pytest.approx(fixed["longitudinal_force_N"])


def test_outer_loop_sign_noise_and_speed_antiwindup():
    drive = Drive(.8, .12)
    pi = YawRatePI(noise_std=0)
    targets = pi.speeds(drive, 0, .5, 0, 24)
    assert targets[3] > drive.speeds(0, .5)[3]
    assert pi.integral > 0
    before = pi.integral
    for _ in range(100):
        targets = pi.speeds(drive, 0, 100, 0, 24)
    assert targets == pytest.approx([-24/KE]*3 + [24/KE]*3)
    assert pi.integral == before
    # Opposite error unwinds a saturated controller.
    pi.integral = 100
    pi.speeds(drive, 0, .5, 1, 24)
    assert pi.integral < 100
    a, b = YawRatePI(), YawRatePI()
    samples = []
    for _ in range(1000):
        assert a.speeds(drive, 0, 0, 0, 24) == pytest.approx(b.speeds(drive, 0, 0, 0, 24))
        samples.append(a.measurement)
    assert np.std(samples) == pytest.approx(.01, rel=.1)


def test_outer_pi_tracks_a_slow_reduced_yaw_plant():
    drive, pi = Drive(.8, .12), YawRatePI(noise_std=0)
    yaw = 0.
    for _ in range(3000):
        targets = pi.speeds(drive, 0, .5, yaw, 24)
        # Assumed yaw response: 0.5 s lag and 0.5 steady gain.
        kinematic_yaw = (targets[3]-targets[0])*.12/(.8*1.25)
        yaw += .01 * (.5*kinematic_yaw-yaw)/.5
    assert yaw == pytest.approx(.5, abs=.002)
