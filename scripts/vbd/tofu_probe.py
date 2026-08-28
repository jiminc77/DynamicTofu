"""Day-2 pre-probe: ONE tofu cell (external ruling).

E=15 kPa, nu 0.45, F=0.6 N, h=5 mm, r=2.5 mm, ke=pad=1e3, kd=1.0, mu=1.0,
eps=2e-4, substeps=40, margin=1e-3, lift 50 mm/2.5 s, hold 5 s.
If <2 mm rel slip -> proceed to the full grid. If it creeps -> eps ladder.

Logs the full per-cell metric set incl. Green-strain fields (placeholder
damage threshold; real threshold pending external sign-off).

Run: cd newton && uv run --no-sync python ../scripts/vbd/tofu_probe.py [eps]
"""

from __future__ import annotations

import json
import os
import subprocess
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import numpy as np

from src.vbd_rig2 import Vbd2Config, run_vbd2

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
PLACEHOLDER_EPS_DAMAGE = 0.15   # placeholder (15% principal strain); real value pending sign-off


def tofu_cfg(E, F, eps=2e-4):
    return Vbd2Config(E_pa=E, nu=0.45, grip_force_n=F, cell_m=0.005, particle_radius=0.0025,
                      contact_ke=1.0e3, contact_kd=1.0, mu_pair=1.0, friction_epsilon=eps,
                      soft_contact_margin=1.0e-3, substeps=40, lift_s=2.5, hold_s=5.0, lift_height_m=0.05)


def run_cell(E, F, eps=2e-4, snap_dir=None, thr=PLACEHOLDER_EPS_DAMAGE):
    from src.vbd_rig2 import Vbd2Rig, FPS, GRAB_Z
    cfg = tofu_cfg(E, F, eps)
    rig = Vbd2Rig(cfg)
    t_ramp = cfg.ramp_s; t_pre = cfg.ramp_s + cfg.preload_s
    t_lift = t_pre + cfg.lift_s; t_end = t_lift + cfg.hold_s
    n = int(t_end * FPS)
    ref_rel = None; series = []; si = 0
    for f in range(n):
        t = rig.sim_time
        cf = F * min(1.0, t / cfg.ramp_s)
        lt = GRAB_Z + cfg.lift_height_m * min(1.0, max(0.0, t - t_pre) / cfg.lift_s)
        rig.step(cf, lt)
        if f % 6 == 0:
            m = rig.metrics(); ss = rig.strain_stats(thr)
            rel = m["com_z"] - m["palm_z"]
            phase = ("ramp" if t < t_ramp else "preload" if t < t_pre else "lift" if t < t_lift else "hold")
            if ref_rel is None and t >= t_pre:
                ref_rel = rel
            m["rel_slip_mm"] = abs(rel - ref_rel) * 1000 if ref_rel is not None else 0.0
            m["phase"] = phase; m["contacts"] = rig.contact_count()
            m.update(ss)
            series.append(m)
        if snap_dir and f % 8 == 0:
            os.makedirs(snap_dir, exist_ok=True)
            s0 = rig.state_0
            np.savez_compressed(os.path.join(snap_dir, f"f_{si:04d}.npz"),
                                particle_q=s0.particle_q.numpy()[rig.soft_start:rig.soft_end].astype(np.float32),
                                body_q=s0.body_q.numpy().astype(np.float32), t=np.float64(rig.sim_time)); si += 1
    hold = [s for s in series if s["phase"] == "hold"]
    slip = max((s["rel_slip_mm"] for s in hold), default=999)
    fvy = float(np.mean([s["finger_vy"] for s in hold])) if hold else 999
    held = slip < 2.0 and all(s["finite"] for s in series)
    weight = rig._weight_n
    fn_applied = F  # force-controlled; at equilibrium (fvy~0) contact Fn ~ applied
    load_frac = weight / (2.0 * cfg.mu_pair * fn_applied)
    peak_strain = max((s["max_principal_strain"] for s in series), default=0)
    hold_strain = np.mean([s["max_principal_strain"] for s in hold]) if hold else 0
    hold_p99 = np.mean([s["p99_vol_weighted_strain"] for s in hold]) if hold else 0
    hold_dmg = np.mean([s["damaged_vol_frac"] for s in hold]) if hold else 0
    bbox_final = series[-1]["bbox"]
    return {"E_pa": E, "grip_force_n": F, "friction_epsilon": eps,
            "hold_slip_mm": round(slip, 2), "held_lt2mm": bool(held), "finite": all(s["finite"] for s in series),
            "mean_finger_speed_hold": round(fvy, 5), "Fn_applied_equilibrium_n": F,
            "coulomb_load_fraction": round(load_frac, 3), "weight_n": round(weight, 4),
            "contacts_hold_mean": int(np.mean([s["contacts"] for s in hold])) if hold else 0,
            "bbox_final": [round(b, 4) for b in bbox_final],
            "peak_principal_strain": round(peak_strain, 4), "hold_mean_principal_strain": round(float(hold_strain), 4),
            "hold_mean_p99_strain": round(float(hold_p99), 4), "hold_mean_damaged_vol_frac_placeholder": round(float(hold_dmg), 4),
            "final_com_rise_mm": round(series[-1]["com_rise"] * 1000, 1)}, series


def main() -> int:
    eps = float(sys.argv[1]) if len(sys.argv) > 1 else 2e-4
    snap = os.path.join(ROOT, "reports", "media", "frames", "tofu_probe_E15_F06")
    res, series = run_cell(15e3, 0.6, eps=eps, snap_dir=snap)
    out = {"gate": "V_day2_preprobe", "git_sha": subprocess.check_output(["git", "-C", ROOT, "rev-parse", "HEAD"], text=True).strip(),
           "cell": "E15kPa_nu0.45_F0.6N_h5mm_r2.5mm_ke1e3_mu1_eps%.0e_sub40_margin1e-3_lift50mm2.5s_hold5s" % eps,
           "placeholder_damage_threshold": PLACEHOLDER_EPS_DAMAGE, "result": res,
           "decision": "proceed_to_grid" if res["held_lt2mm"] else "run_eps_ladder"}
    json.dump(out, open(os.path.join(ROOT, "reports", "logs", "vbd", "tofu_probe.json"), "w"), indent=2, default=str)
    print(json.dumps(res, indent=1))
    print("DECISION:", out["decision"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
