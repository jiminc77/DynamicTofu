"""Persist the pre-probe force/substep variants (reproducibility for the escalation).

Re-runs the E=15 kPa force sweep at substeps=40 and the decisive F=1.2 N /
substeps=80 HOLD, persisting full per-run metrics + series + provenance, and
captures a hold clip for the substeps=80 pass.

Run: cd newton && uv run --no-sync python ../scripts/vbd/tofu_variants.py
"""

from __future__ import annotations

import json
import os
import subprocess
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.dirname(__file__))

from tofu_probe import run_cell  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def main() -> int:
    out = {"gate": "V_day2_preprobe_variants",
           "git_sha": subprocess.check_output(["git", "-C", ROOT, "rev-parse", "HEAD"], text=True).strip(),
           "runs": {}}
    # force sweep at substeps=40 (under-resolved, non-monotonic)
    for F in (0.6, 1.0, 1.2, 1.5):
        res, series = run_cell(15e3, F, eps=2e-4, substeps=40)
        out["runs"][f"E15_F{F}_sub40"] = {"result": res, "series_tail": series[-8:]}
        print(f"E15 F={F} sub40: slip={res['hold_slip_mm']}mm held={res['held_lt2mm']} "
              f"final={res['final_com_rise_mm']}mm peak_strain={res['peak_principal_strain']}", flush=True)
    # DECISIVE hold: F=1.2 N at substeps=80, with a clip
    snap = os.path.join(ROOT, "reports", "media", "frames", "tofu_hold_E15_F12_sub80")
    res80, series80 = run_cell(15e3, 1.2, eps=2e-4, substeps=80, snap_dir=snap)
    out["runs"]["E15_F1.2_sub80"] = {"result": res80, "series": series80}
    print(f"E15 F=1.2 sub80: slip={res80['hold_slip_mm']}mm held={res80['held_lt2mm']} "
          f"final={res80['final_com_rise_mm']}mm peak_strain={res80['peak_principal_strain']}", flush=True)
    out["decisive_hold"] = out["runs"]["E15_F1.2_sub80"]["result"]
    json.dump(out, open(os.path.join(ROOT, "reports", "logs", "vbd", "tofu_probe_variants.json"), "w"), indent=2, default=str)
    print("saved tofu_probe_variants.json; snap ->", snap)
    return 0


if __name__ == "__main__":
    sys.exit(main())
