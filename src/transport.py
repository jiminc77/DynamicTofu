"""CPU-only command profile and tracking estimator for the W1 transport protocol."""
from dataclasses import dataclass
from typing import Callable

import numpy as np

TIMEBASE_HZ = 4800


@dataclass(frozen=True)
class TransportProfile:
    a_cmd: Callable
    v_cmd: Callable
    x_cmd: Callable
    phase_timestamps: dict
    plateau_windows: dict
    transport_duration: float
    delta_v: float
    leg_displacement: float
    jerk: float

    def phase_at(self, t):
        """Return the protocol phase using integer-substep membership."""
        k = round(float(t) * TIMEBASE_HZ)
        for name, (start, end) in self.phase_timestamps.items():
            ks, ke = round(start * TIMEBASE_HZ), round(end * TIMEBASE_HZ)
            if ks <= k < ke or (name == "settle" and ks <= k <= ke):
                return name
        return None


def trapezoid_reversal(
    a_peak, T_j=0.05, T_a=0.10, T_c=0.10, T_dwell=0.30,
    t0=9.30, settle=1.00,
):
    """Construct the analytic, jerk-limited out-and-back W1 command.

    The returned callables accept either a scalar or a NumPy array of absolute
    times. Positive acceleration denotes the outbound (+x) direction.
    """
    a_peak = float(a_peak)
    durations = (T_j, T_a, T_c, T_dwell, settle)
    if not np.isfinite(a_peak) or a_peak <= 0:
        raise ValueError("a_peak must be finite and positive")
    if any(not np.isfinite(x) or x <= 0 for x in durations):
        raise ValueError("all phase durations must be finite and positive")

    # Each motion leg is acceleration lobe, cruise, deceleration lobe.
    # (duration, jerk); zero-jerk plateau acceleration is inherited.
    j = a_peak / T_j
    moving = [
        (T_j, j), (T_a, 0.0), (T_j, -j),
        (T_c, 0.0),
        (T_j, -j), (T_a, 0.0), (T_j, j),
    ]
    segments = moving + [(T_dwell, 0.0)] + [
        (T_j, -j), (T_a, 0.0), (T_j, j),
        (T_c, 0.0),
        (T_j, j), (T_a, 0.0), (T_j, -j),
    ]

    starts = []
    elapsed = x0 = v0 = acc0 = 0.0
    for duration, jerk in segments:
        starts.append((elapsed, duration, x0, v0, acc0, jerk))
        x0 += v0 * duration + 0.5 * acc0 * duration**2 + jerk * duration**3 / 6.0
        v0 += acc0 * duration + 0.5 * jerk * duration**2
        acc0 += jerk * duration
        elapsed += duration
    moving_end = elapsed
    total = moving_end + settle
    dwell_start = sum(duration for duration, _ in moving)
    dwell_end = dwell_start + T_dwell

    def evaluate(t, component):
        values = np.asarray(t, dtype=float)
        flat = values.ravel()
        out = np.zeros_like(flat)
        for start, duration, sx, sv, sa, sj in starts:
            mask = (flat >= t0 + start) & (flat < t0 + start + duration)
            u = flat[mask] - (t0 + start)
            if component == "a":
                out[mask] = sa + sj * u
            elif component == "v":
                out[mask] = sv + sa * u + 0.5 * sj * u**2
            else:
                out[mask] = sx + sv * u + 0.5 * sa * u**2 + sj * u**3 / 6.0
        if component in (("a", "v")):
            # Eliminate harmless integration roundoff: the protocol requires
            # an exact full stop throughout the complete dwell.
            out[(flat >= t0 + dwell_start) & (flat < t0 + dwell_end)] = 0.0
        if component == "x":
            # During final settle the carriage remains at its returned origin.
            out[(flat >= t0 + moving_end) & (flat <= t0 + total)] = 0.0
        result = out.reshape(values.shape)
        return float(result) if values.ndim == 0 else result

    b0 = t0
    b1 = b0 + 2 * T_j + T_a
    b2 = b1 + T_c
    b3 = b2 + 2 * T_j + T_a
    b4 = b3 + T_dwell
    b5 = b4 + 2 * T_j + T_a
    b6 = b5 + T_c
    b7 = b6 + 2 * T_j + T_a
    b8 = b7 + settle
    phases = {
        "accel_out": (b0, b1), "cruise_out": (b1, b2),
        "decel_out": (b2, b3), "dwell": (b3, b4),
        "accel_back": (b4, b5), "cruise_back": (b5, b6),
        "decel_back": (b6, b7), "settle": (b7, b8),
    }
    plateaus = {
        "accel_out": {"start": b0 + T_j, "end": b0 + T_j + T_a, "a_cmd": a_peak},
        "decel_out": {"start": b2 + T_j, "end": b2 + T_j + T_a, "a_cmd": -a_peak},
        "accel_back": {"start": b4 + T_j, "end": b4 + T_j + T_a, "a_cmd": -a_peak},
        "decel_back": {"start": b6 + T_j, "end": b6 + T_j + T_a, "a_cmd": a_peak},
    }
    return TransportProfile(
        a_cmd=lambda t: evaluate(t, "a"),
        v_cmd=lambda t: evaluate(t, "v"),
        x_cmd=lambda t: evaluate(t, "x"),
        phase_timestamps=phases, plateau_windows=plateaus,
        transport_duration=moving_end, delta_v=a_peak * (T_a + T_j),
        leg_displacement=a_peak * (T_a + T_j) * (2 * T_j + T_a + T_c),
        jerk=j,
    )


def realized_accel(t_arr, vx_arr, plateau_windows):
    """Fit velocity slopes strictly inside each pre-registered plateau."""
    t = np.asarray(t_arr, dtype=float)
    v = np.asarray(vx_arr, dtype=float)
    if t.ndim != 1 or v.ndim != 1 or t.shape != v.shape:
        raise ValueError("t_arr and vx_arr must be same-length one-dimensional arrays")
    results = {}
    for name, window in plateau_windows.items():
        start, end, commanded = window["start"], window["end"], window["a_cmd"]
        mask = (t > start) & (t < end) & np.isfinite(t) & np.isfinite(v)
        tw, vw = t[mask], v[mask]
        n = int(tw.size)
        if n >= 2:
            centered = tw - tw.mean()
            slope = float(np.dot(centered, vw - vw.mean()) / np.dot(centered, centered))
            fitted = vw.mean() + slope * centered
            ss_res = float(np.dot(vw - fitted, vw - fitted))
            ss_tot = float(np.dot(vw - vw.mean(), vw - vw.mean()))
            r2 = 1.0 if ss_tot == 0.0 and ss_res == 0.0 else (1.0 - ss_res / ss_tot if ss_tot else float("nan"))
            error = abs(slope - commanded) / abs(commanded)
        else:
            slope = error = r2 = float("nan")
        results[name] = {"a_fit": slope, "abs_err": float(error), "n_samples": n, "r2": float(r2)}
    return results


def g_trk(t_arr, vx_arr, plateau_windows, tolerance=0.05, min_samples=5):
    """Evaluate G-TRK, failing closed on thin or non-finite fits."""
    fits = realized_accel(t_arr, vx_arr, plateau_windows)
    samples_valid = all(item["n_samples"] >= min_samples for item in fits.values())
    errors = [item["abs_err"] for item in fits.values()]
    finite = bool(errors) and bool(np.all(np.isfinite(errors)))
    max_error = float(max(errors)) if finite else float("nan")
    return {
        "fits": fits, "max_abs_err": max_error,
        "samples_valid": samples_valid,
        "pass": bool(samples_valid and finite and max_error <= tolerance),
    }
