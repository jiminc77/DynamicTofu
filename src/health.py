"""Solver-health predicate (frozen tolerances, plan G-N2).

Machine-checked "no blow-up": NaN/inf scan, max particle speed, Jp finiteness,
sparse-grid rebuild status, block-volume drift. Evaluated every probe/trial;
results embedded in every JSON. Limits are FROZEN (pending-approval.md,
"Frozen tolerances"): max particle speed <= 5.0 m/s; gentle-hold volume drift
<= 2%; no NaN/inf in particle_q/qd/Jp; sparse-grid rebuild must not raise.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class HealthLimits:
    max_particle_speed_ms: float = 5.0
    volume_drift_frac: float = 0.02  # gentle-hold contexts only


@dataclass
class HealthAccumulator:
    """Accumulates worst-case health metrics across a run."""

    limits: HealthLimits = field(default_factory=HealthLimits)
    max_speed_ms: float = 0.0
    nan_free: bool = True
    jp_finite: bool = True
    grid_ok: bool = True
    grid_error: str = ""
    n_checks: int = 0

    def check_tick(self, particle_q: np.ndarray, particle_qd: np.ndarray, jp: np.ndarray | None, mpm_solver=None) -> bool:
        """One health evaluation. Returns True when this tick is healthy."""
        self.n_checks += 1
        ok = True
        if not np.isfinite(particle_q).all() or not np.isfinite(particle_qd).all():
            self.nan_free = False
            ok = False
        speed = float(np.max(np.linalg.norm(particle_qd, axis=1))) if len(particle_qd) else 0.0
        self.max_speed_ms = max(self.max_speed_ms, speed)
        if speed > self.limits.max_particle_speed_ms:
            ok = False
        if jp is not None and not np.isfinite(jp).all():
            self.jp_finite = False
            ok = False
        if mpm_solver is not None:
            try:
                check = getattr(mpm_solver, "check_sparse_grid_rebuild_status", None)
                if check is not None:
                    check()
            except Exception as exc:  # noqa: BLE001 - any raise is a health failure by contract
                self.grid_ok = False
                self.grid_error = repr(exc)
                ok = False
        return ok

    @property
    def clean(self) -> bool:
        return (
            self.nan_free
            and self.jp_finite
            and self.grid_ok
            and self.max_speed_ms <= self.limits.max_particle_speed_ms
        )

    def report(self) -> dict:
        return {
            "clean": bool(self.clean),
            "max_particle_speed_ms": float(self.max_speed_ms),
            "speed_limit_ms": self.limits.max_particle_speed_ms,
            "nan_free": bool(self.nan_free),
            "jp_finite": bool(self.jp_finite),
            "sparse_grid_ok": bool(self.grid_ok),
            "sparse_grid_error": self.grid_error,
            "n_checks": int(self.n_checks),
        }


def block_volume_estimate(particle_q: np.ndarray) -> float:
    """Axis-aligned convex proxy for gentle-hold volume-drift tracking.

    Product of per-axis (95th - 5th percentile) spans: robust to stray
    particles, monotone under bulk compression, cheap. Used only as a
    drift ratio against the settled reference, never as an absolute volume.
    """
    if len(particle_q) == 0:
        return 0.0
    lo = np.percentile(particle_q, 5.0, axis=0)
    hi = np.percentile(particle_q, 95.0, axis=0)
    return float(np.prod(np.maximum(hi - lo, 1e-9)))


def volume_drift(reference_volume: float, current_volume: float) -> float:
    if reference_volume <= 0.0:
        return 0.0
    return abs(current_volume - reference_volume) / reference_volume
