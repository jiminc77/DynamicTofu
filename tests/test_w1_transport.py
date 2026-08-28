import numpy as np

from src.transport import TIMEBASE_HZ, g_trk, realized_accel, trapezoid_reversal


LEVELS = (1, 2.5, 5, 10, 20, 30)


def grid(profile, rate=TIMEBASE_HZ):
    start = profile.phase_timestamps["accel_out"][0]
    end = profile.phase_timestamps["settle"][1]
    return np.arange(round(start * rate), round(end * rate) + 1) / rate


def test_command_profile_invariants():
    p = trapezoid_reversal(10)
    t = grid(p)
    a, v, x = p.a_cmd(t), p.v_cmd(t), p.x_cmd(t)
    assert np.max(np.abs(a)) == 10
    plateau = p.plateau_windows["accel_out"]
    assert (
        round(plateau["end"] * TIMEBASE_HZ)
        - round(plateau["start"] * TIMEBASE_HZ)
        == round(0.10 * TIMEBASE_HZ)
    )

    # The return leg is the time-reversed, sign-reflected outbound leg.
    out = np.linspace(9.30, 9.80, 2401)
    back = np.linspace(10.10, 10.60, 2401)
    np.testing.assert_allclose(p.a_cmd(back), -p.a_cmd(out), atol=2e-12)
    np.testing.assert_allclose(p.v_cmd(back), -p.v_cmd(out), atol=2e-12)
    np.testing.assert_allclose(
        p.x_cmd(back), p.leg_displacement - p.x_cmd(out), atol=2e-12
    )

    nonzero_signs = np.sign(v[np.abs(v) > 1e-12])
    assert np.count_nonzero(np.diff(nonzero_signs) != 0) == 1
    dwell = (t >= 9.80) & (t < 10.10)
    assert np.all(v[dwell] == 0.0)
    assert abs(p.x_cmd(9.30)) <= 1e-12
    assert abs(p.x_cmd(11.60)) <= 1e-12
    assert abs(p.delta_v - 1.5) < 1e-14
    assert abs(p.leg_displacement - 0.45) < 1e-14

    # Acceleration is continuous, so its grid slope measures commanded jerk.
    jerk = np.diff(a) * TIMEBASE_HZ
    assert np.max(np.abs(jerk)) <= p.jerk * (1 + 1e-10)


def test_duration_is_independent_of_acceleration_level():
    durations = [trapezoid_reversal(level).transport_duration for level in LEVELS]
    np.testing.assert_allclose(durations, 1.30, atol=1e-14)
    # Transport excludes the final 1.0 s settle and is exactly 1.30 s.
    for level in LEVELS:
        p = trapezoid_reversal(level)
        assert abs(p.phase_timestamps["settle"][0] - 9.30 - 1.30) < 1e-14


def test_estimator_synthetic_exact_and_attenuated():
    p = trapezoid_reversal(5)
    t = grid(p, rate=240)

    exact = realized_accel(t, p.v_cmd(t), p.plateau_windows)
    assert max(row["abs_err"] for row in exact.values()) < 1e-12
    assert all(row["n_samples"] >= 5 for row in exact.values())

    # Independent noisy linear traces exercise the regression rather than the profile.
    rng = np.random.default_rng(17)
    noisy_v = p.v_cmd(t).copy()
    for window in p.plateau_windows.values():
        mask = (t > window["start"]) & (t < window["end"])
        noisy_v[mask] += rng.normal(0.0, 1e-5, mask.sum())
    noisy = g_trk(t, noisy_v, p.plateau_windows)
    assert noisy["pass"]
    assert noisy["max_abs_err"] < 0.005

    attenuated = g_trk(t, 0.90 * p.v_cmd(t), p.plateau_windows)
    assert not attenuated["pass"]
    assert attenuated["max_abs_err"] > 0.05


def test_gate_fails_closed_on_insufficient_samples():
    p = trapezoid_reversal(1)
    sparse_t = np.array([w["start"] + 0.05 for w in p.plateau_windows.values()])
    sparse_v = p.v_cmd(sparse_t)
    result = g_trk(sparse_t, sparse_v, p.plateau_windows)
    assert not result["samples_valid"]
    assert not result["pass"]
