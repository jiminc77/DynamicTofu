"""Pure host-side implementation of the frozen E1 judgment reducer."""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class JudgmentThresholds:
    jp_dev: float = 0.05
    damage_frac: float = 0.10
    contact_loss_s: float = 0.2
    rel_disp_m: float = 0.02
    slip_net_m: float = 0.005
    slip_peak_m: float = 0.008


def evaluate(samples, lift_complete, settle_end, particle_count=None, thresholds=JudgmentThresholds()):
    """Reduce sample dictionaries. Samples must already be in strictly ascending time order."""
    if not samples:
        return {"labels": [], "label_set": set(), "peak_damage_fraction": 0.0}
    times = np.asarray([s["t"] for s in samples], dtype=float)
    if not np.all(np.isfinite(times)) or np.any(np.diff(times) <= 0):
        raise ValueError("sample timestamps must be strictly increasing and unique")
    labels = set()
    peak_damage = 0.0
    loss_start = None
    damage_latch_t = None
    drop_t = None
    for s in samples:
        t = float(s["t"])
        if t < lift_complete or t > settle_end:
            continue
        jp = np.asarray(s.get("jp", []), dtype=float).reshape(-1)
        indices = np.asarray(s.get("particle_indices", np.arange(jp.size)))
        if jp.size:
            if particle_count is None:
                denominator = jp.size
            else:
                denominator = particle_count
            # The explicit stable index ordering is part of the reducer contract.
            order = np.argsort(indices, kind="stable")
            count = 0
            for value in jp[order]:
                count += int(abs(float(value) - 1.0) > thresholds.jp_dev)
            fraction = count / denominator
            peak_damage = max(peak_damage, fraction)
            if fraction > thresholds.damage_frac:
                labels.add("damage")
                if damage_latch_t is None:
                    damage_latch_t = t
        established = bool(s.get("grasp_established", True))
        bilateral = bool(s.get("bilateral_contact", True))
        if established and not bilateral:
            if loss_start is None:
                loss_start = t
            if t - loss_start > thresholds.contact_loss_s:
                labels.add("drop")
                if drop_t is None:
                    drop_t = t
        else:
            loss_start = None
        if float(s.get("relative_displacement_m", 0.0)) > thresholds.rel_disp_m:
            labels.add("drop")
            if drop_t is None:
                drop_t = t
        if (float(s.get("slip_net_m", 0.0)) > thresholds.slip_net_m or
                float(s.get("slip_peak_m", 0.0)) > thresholds.slip_peak_m):
            labels.add("slip")
    # Frozen precedence: slip is defined "without drop" over the FINAL label set,
    # so a drop occurring after a slip event still suppresses slip.
    if "drop" in labels:
        labels.discard("slip")
    # Pre-registered cell-outcome precedence (external order 2026-08-27, before
    # Stage A): damage latched AFTER grasp loss attributes the cell to drop
    # (impact/table compaction, not the grasp); damage while grasped colors as
    # damage. Raw labels are unchanged either way.
    damage_after_drop = (
        damage_latch_t is not None and drop_t is not None and damage_latch_t > drop_t
    )
    if "damage" in labels and not damage_after_drop:
        cell_color = "damage"
    elif "drop" in labels:
        cell_color = "drop"
    elif "slip" in labels:
        cell_color = "slip"
    else:
        cell_color = "intact"
    return {"labels": sorted(labels), "label_set": labels,
            "peak_damage_fraction": peak_damage,
            "damage_latch_t": damage_latch_t, "drop_t": drop_t,
            "damage_after_drop": bool(damage_after_drop),
            "cell_color": cell_color}


judge_trial = evaluate
