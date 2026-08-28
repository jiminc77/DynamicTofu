"""Day-2 finalize: mesh convergence (h=4 mm on 2 cells) + clip captures.

Mesh conv: E=15/F=1.0 (intact-ish) and E=7/F=1.5 (near damage-onset, high strain)
at cell_m=0.004 vs the h=5 mm grid. Clips: holding (E15/F1.0), slipping (E7/F0.4),
high-strain (E7/F2.0), each with snapshots for rendering.

Run: cd newton && uv run --no-sync python ../scripts/vbd/tofu_finalize.py
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
FR = os.path.join(ROOT, "reports", "media", "frames")


def main() -> int:
    out = {"gate": "V_day2_finalize", "git_sha": subprocess.check_output(["git", "-C", ROOT, "rev-parse", "HEAD"], text=True).strip()}
    grid = json.load(open(os.path.join(ROOT, "reports", "logs", "vbd", "tofu_grid.json")))["cells"]

    # --- mesh convergence h=4mm ---
    mc = {}
    for E, F, name in ((15e3, 1.0, "E15_F1.0"), (7e3, 1.5, "E7_F1.5")):
        res5 = grid[name]
        res4, _ = run_cell(E, F, eps=2e-4, substeps=80, cell_m=0.004)
        mc[name] = {"h5mm": {"slip_mm": res5["hold_slip_mm"], "label": res5["label"],
                             "peak_strain": res5["peak_principal_strain"], "p99": res5["hold_mean_p99_strain"]},
                    "h4mm": {"slip_mm": res4["hold_slip_mm"], "held": res4["held_lt2mm"],
                             "peak_strain": res4["peak_principal_strain"], "p99": res4["hold_mean_p99_strain"]},
                    "label_invariant": (res5["label"] == ("intact" if res4["held_lt2mm"] else "slip"))}
        print(f"meshconv {name}: h5 slip={res5['hold_slip_mm']} p99={res5['hold_mean_p99_strain']} | "
              f"h4 slip={res4['hold_slip_mm']} p99={res4['hold_mean_p99_strain']} label_inv={mc[name]['label_invariant']}", flush=True)
    out["mesh_convergence"] = mc

    # --- clips ---
    for E, F, tag in ((15e3, 1.0, "hold"), (7e3, 0.4, "slip"), (7e3, 2.0, "highstrain")):
        res, _ = run_cell(E, F, eps=2e-4, substeps=80, snap_dir=os.path.join(FR, f"tofu_clip_{tag}"))
        print(f"clip {tag} E{int(E/1000)}/F{F}: slip={res['hold_slip_mm']}mm label={'intact' if res['held_lt2mm'] else 'slip'} peak_strain={res['peak_principal_strain']}", flush=True)
    json.dump(out, open(os.path.join(ROOT, "reports", "logs", "vbd", "tofu_meshconv.json"), "w"), indent=2, default=str)
    print("finalize done -> tofu_meshconv.json + clip frames")
    return 0


if __name__ == "__main__":
    sys.exit(main())
