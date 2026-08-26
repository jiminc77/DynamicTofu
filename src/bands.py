"""Certified planned-denominator band reduction and shape routing."""
import math


def required_pass(n):
    return math.ceil(2 * n / 3)


def reduce_cell(outcomes, n_planned):
    if n_planned not in (3, 5):
        raise ValueError("n_planned must be 3 or 5")
    valid = [o for o in outcomes if o is not None]
    k = sum(not set(o) for o in valid)
    certified = len(valid) == n_planned
    return {"n_planned": n_planned, "resolved": len(valid), "k_pass": k,
            "certified": certified,
            "cell_pass": (k >= required_pass(n_planned)) if certified else None}


def estimate_band(cells):
    """cells maps force to a reduce_cell result (or outcome iterable plus n_planned)."""
    axis = sorted(cells)
    normalized = {}
    for f, c in cells.items():
        normalized[f] = c if isinstance(c, dict) else reduce_cell(c, len(c))
    passed = [f for f in axis if normalized[f]["certified"] and normalized[f]["cell_pass"]]
    result = {"F_min": None, "F_max": None, "band_width_n": None,
              "band_status": "empty", "censored_low": False, "censored_high": False,
              "interior_failures": [], "interior_gaps": [], "cells": normalized}
    if not passed:
        return result
    lo, hi = min(passed), max(passed)
    result.update(F_min=lo, F_max=hi, band_width_n=hi-lo,
                  band_status="single_point" if len(passed) == 1 else "contiguous",
                  censored_low=lo == axis[0], censored_high=hi == axis[-1])
    interior = [f for f in axis if lo < f < hi]
    failures = [f for f in interior if normalized[f]["certified"] and not normalized[f]["cell_pass"]]
    gaps = [f for f in interior if not normalized[f]["certified"]]
    result["interior_failures"], result["interior_gaps"] = failures, gaps
    if gaps:
        result["band_status"] = "gapped"
    elif failures:
        result["band_status"] = "intermittent"
    return result


def estimate_a_star(bands):
    axis = sorted(bands)
    empty = [a for a in axis if bands[a]["band_status"] == "empty"]
    if not empty:
        return None, "no_vanishing_observed"
    candidates = [a for a in empty if all(bands[x]["band_status"] == "empty" for x in axis if x > a)]
    if candidates:
        a = min(candidates)
        # An empty only at the highest sampled acceleration is right-censored.
        if a == axis[-1]:
            return None, "censored_above"
        return a, "observed"
    return None, "nonmonotone"


def shape_checkpoint(bands, force_axis, a_star=None):
    accels = sorted(bands)
    statuses = {a: {k: bands[a].get(k) for k in ("band_status", "censored_low", "censored_high")} for a in accels}
    u_max = [a for a in accels if bands[a]["band_status"] != "empty" and not bands[a]["censored_high"]]
    u_min = [a for a in accels if bands[a]["band_status"] != "empty" and not bands[a]["censored_low"]]
    base = {"Delta_max": None, "Delta_min": None, "C_max": None, "C_min": None,
            "U_max_count": len(u_max), "U_min_count": len(u_min), "N_amb": 0,
            "bands": statuses, "low_confidence": False}
    if all(bands[a]["band_status"] == "empty" for a in accels) or all(bands[a]["censored_low"] and bands[a]["censored_high"] for a in accels):
        return base | {"outcome": "scale_failure", "branch": "scale_failure"}
    if any(bands[a]["band_status"] == "empty" for a in accels) and a_star is not None:
        return base | {"outcome": "structure_present", "branch": "band_vanishing"}
    if len(u_max) < 3 and len(u_min) < 3:
        return base | {"outcome": "ambiguous", "branch": "insufficient_usable_bands"}
    namb = 0
    for b in bands.values():
        for c in b.get("cells", {}).values():
            k, n = c.get("k_pass", 0), c.get("n_planned", 0)
            namb = max(namb, int(k not in (0, n)))
    base["N_amb"] = namb
    decisions = []
    for side, usable, expected in (("max", u_max, "down"), ("min", u_min, "up")):
        if len(usable) < 3:
            continue
        vals = [bands[a]["F_" + side] for a in usable]
        idx = [force_axis.index(v) for v in vals]
        delta = idx[0] - idx[-1]
        c = sum((x >= y if expected == "down" else x <= y) for x, y in zip(idx, idx[1:]))
        base["Delta_" + side], base["C_" + side] = delta, c
        pass_sets = [{f for f, q in bands[a].get("cells", {}).items() if q.get("certified") and q.get("cell_pass")} for a in usable]
        identical = bool(pass_sets) and all(x == pass_sets[0] for x in pass_sets[1:])
        structured = c >= len(usable)-2 and abs(delta) >= max(2, namb+1)
        no_structure = (abs(delta) <= 1 and c < len(usable)-2) or identical
        decisions.append("structure" if structured else "none" if no_structure else "ambiguous")
    outcome = "structure_present" if "structure" in decisions else "no_structure" if decisions and all(x == "none" for x in decisions) else "ambiguous"
    usable_union = set(u_max) | set(u_min)
    low = sum(bands[a]["band_status"] == "intermittent" for a in usable_union) >= 2
    if low and outcome == "no_structure": outcome = "ambiguous"
    base["low_confidence"] = low
    return base | {"outcome": outcome, "branch": "index_test"}
