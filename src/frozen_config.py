"""CPU-only enforcement of the pre-registered VBD production configuration."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any


FROZEN_PRODUCTION = {
    "substeps": 80,
    "friction_epsilon": 2.0e-4,
    "mu_pair": 1.0,
    "contact_ke": 1.0e3,
    "contact_kd": 1.0,
    "soft_contact_margin": 1.0e-3,
    "cell_m": 0.005,
    "particle_radius": 0.0025,
    "correct_mass": True,
    "nu": 0.45,
    "E_pa": (7.0e3, 15.0e3, 25.0e3),
}

RIG_PRE_EDIT_SHA256 = "11011fb9e53544d4da75f1ad1e17932ccfc9d867e81eb8c146eb994209156475"
NEWTON_COMMIT = "b74df534"


def _values(obj: Any) -> Mapping[str, Any]:
    if not isinstance(obj, Mapping):
        return {key: getattr(obj, key, None) for key in FROZEN_PRODUCTION}
    for container_key in ("frozen_config", "production_config", "config"):
        nested = obj.get(container_key)
        if isinstance(nested, Mapping):
            return nested
    return obj


def assert_frozen(obj: Any) -> None:
    """Assert all frozen values, reporting all deviations in one failure."""
    values = _values(obj)
    mismatches = []
    for key, expected in FROZEN_PRODUCTION.items():
        actual = values.get(key)
        valid = actual in expected if key == "E_pa" else actual == expected
        if not valid:
            expectation = f"one of {expected!r}" if key == "E_pa" else repr(expected)
            mismatches.append(f"{key}: expected {expectation}, got {actual!r}")
    assert not mismatches, "Frozen production config mismatch: " + "; ".join(mismatches)


def frozen_provenance() -> dict[str, Any]:
    """Return JSON-friendly frozen values and their source pins for receipts."""
    config = dict(FROZEN_PRODUCTION)
    config["E_pa"] = list(config["E_pa"])
    return {
        "frozen_config": config,
        "frozen_check": True,
        "rig_pre_edit_sha256": RIG_PRE_EDIT_SHA256,
        "newton_commit": NEWTON_COMMIT,
    }
