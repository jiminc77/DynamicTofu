"""Fail-closed E1 stage schedule selection."""
STAGE_SETS = (("A", 75), ("A+B", 225), ("A+B+C(i)", 270),
              ("A+B+C(i)+C(ii)", 310))
DROP_ORDER = ("C(ii)", "C(i)", "B(6000)", "B(2000)")
RESERVED_E2_TRIALS = 9


def select_schedule(T, T_E2):
    """Select using trial times in seconds and return all budgets in seconds."""
    T, T_E2 = float(T), float(T_E2)
    if T < 0 or T_E2 < 0:
        raise ValueError("trial times must be non-negative")
    r_e2 = RESERVED_E2_TRIALS * T_E2 * 1.25
    overflow = max(0.0, r_e2 - 4 * 3600.0)
    e1_budget = 24 * 3600.0 - overflow
    available = 0.8 * e1_budget
    t_a_max = available / 75.0
    executable = [(name, n) for name, n in STAGE_SETS if n * T <= available]
    selected = executable[-1] if executable else None
    escalation = T > t_a_max
    assert escalation == (selected is None)
    if selected is None:
        dropped = list(DROP_ORDER)
    else:
        n = selected[1]
        dropped = (["C(ii)"] if n < 310 else []) + (["C(i)"] if n < 270 else [])
        if n < 225:
            dropped += ["B(6000)", "B(2000)"]
    return {"R_E2": r_e2, "E2_overflow": overflow, "E1_budget": e1_budget,
            "T_A_max": t_a_max, "selected_stage_set": selected[0] if selected else None,
            "selected_trial_count": selected[1] if selected else 0,
            "dropped_stages": dropped, "escalate": escalation,
            "reserved_e2_trials": RESERVED_E2_TRIALS}
