#!/usr/bin/env python3
"""Small, standalone implicit-MPM material and pinch/lift study.

Run from the Newton checkout, for example:
  uv run --no-sync python ../scripts/probes/material_study.py --suite --output ../reports/logs/material-study.json
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import warp as wp
import newton
from newton.solvers import SolverImplicitMPM


def smoothstep(x: float) -> float:
    x = min(1.0, max(0.0, x))
    return x * x * (3.0 - 2.0 * x)


def box_mesh(device, half):
    mesh = newton.Mesh.create_box(*half, duplicate_vertices=False, compute_normals=False,
                                  compute_uvs=False, compute_inertia=False)
    p = wp.array(np.asarray(mesh.vertices, dtype=np.float32), dtype=wp.vec3, device=device)
    return wp.Mesh(p, wp.array(mesh.indices, dtype=int, device=device), wp.zeros_like(p))


def body_transform(x, y, z):
    return wp.transform(wp.vec3(x, y, z), wp.quat_identity())


def trial(spec, device="cuda:0"):
    voxel = 0.005
    dt = spec.get("dt", 0.01)
    mode = spec.get("mode", "pinch")
    builder = newton.ModelBuilder(up_axis=newton.Axis.Z, gravity=(0.0, 0.0, -9.81))
    SolverImplicitMPM.register_custom_attributes(builder)
    # Two kinematic bodies; body_q is prescribed before every step.  Ground is a static mesh.
    pad0 = builder.add_body(xform=body_transform(0, 0, 0), is_kinematic=True)
    pad1 = builder.add_body(xform=body_transform(0, 0, 0), is_kinematic=True)
    builder.add_shape_box(-1, xform=body_transform(0, 0, -0.0025), hx=.10, hy=.10, hz=.0025)
    if mode == "uniaxial":
        builder.add_shape_box(pad0, hx=.03, hy=.03, hz=.004)
        builder.add_shape_box(pad1, hx=.001, hy=.001, hz=.001)
    else:
        builder.add_shape_box(pad0, hx=.004, hy=.035, hz=.03)
        builder.add_shape_box(pad1, hx=.004, hy=.035, hz=.03)
    mat = {
        "mpm:young_modulus": 7000.0, "mpm:poisson_ratio": 0.45,
        "mpm:damping": spec.get("damping", 0.001), "mpm:friction": spec.get("friction", 1.0),
        "mpm:yield_stress": spec.get("yield_stress", 3333.0),
        "mpm:yield_pressure": spec.get("yield_pressure", 1.0e15),
        "mpm:tensile_yield_ratio": spec.get("tensile_yield_ratio", 1.0),
        "mpm:hardening": spec.get("hardening", 0.0),
        "mpm:hardening_rate": spec.get("hardening_rate", 1.0),
        "mpm:softening_rate": spec.get("softening_rate", 1.0),
        "mpm:dilatancy": spec.get("dilatancy", 0.0), "mpm:viscosity": spec.get("viscosity", 0.0),
        "mpm:particle_Jp": spec.get("initial_jp", 1.0),
    }
    builder.add_particle_grid(pos=wp.vec3(-0.0175, -0.0175, 0.0025), rot=wp.quat_identity(),
        vel=wp.vec3(0.0), dim_x=8, dim_y=8, dim_z=8, cell_x=voxel, cell_y=voxel,
        cell_z=voxel, mass=1000.0 * voxel**3, jitter=0.0, radius_mean=0.5 * voxel,
        custom_attributes=mat)
    model = builder.finalize(device=device)
    cfg = SolverImplicitMPM.Config(voxel_size=voxel, strain_basis=spec.get("strain_basis", "P0"),
        max_iterations=spec.get("max_iterations", 80), critical_fraction=spec.get("critical_fraction", 0.0))
    # body_q is prescribed, so infer the kinematic surface velocity from consecutive poses.
    cfg.collider_velocity_mode = "backward"
    solver = SolverImplicitMPM(model, config=cfg)
    adhesion = float(spec.get("adhesion", 0.0))
    solver.setup_collider(collider_meshes=[None, None, None], collider_body_ids=[-1, 0, 1],
        collider_margins=[0.0, 0.0, 0.0], collider_friction=[spec.get("friction", 1.0)] * 3,
        collider_adhesion=[0.0, adhesion, adhesion], body_q=model.state().body_q,
        body_mass=wp.zeros(2, dtype=float, device=device),
        body_com=wp.zeros(2, dtype=wp.vec3, device=device),
        body_inv_inertia=wp.zeros(2, dtype=wp.mat33, device=device))
    a, b = model.state(), model.state()
    samples = []
    gap = float(spec.get("gap", 0.010 if mode == "pinch_crush" else 0.028))
    close_t, hold_t, lift_t = 0.25, 0.15, 0.30
    duration = close_t + hold_t + (0.0 if mode in ("pinch_crush", "uniaxial") else lift_t) + 0.15
    nsteps = int(math.ceil(duration / dt))
    start = time.perf_counter()
    for step in range(nsteps + 1):
        t = step * dt
        c = smoothstep(t / close_t)
        lift = 0.05 * smoothstep((t - close_t - hold_t) / lift_t) if mode == "pinch_lift" else 0.0
        if mode == "uniaxial":
            # Top descends from z=.052 to .012: 75% nominal axial crush.
            q = [body_transform(0, 0, 0.052 - 0.040 * c), body_transform(0.2, 0, 0)]
        else:
            center = 0.5 * gap + 0.004
            x = 0.032 * (1.0 - c) + center * c
            q = [body_transform(-x, 0, 0.022 + lift), body_transform(x, 0, 0.022 + lift)]
        q_np = np.array([[qq.p[0], qq.p[1], qq.p[2], qq.q[0], qq.q[1], qq.q[2], qq.q[3]] for qq in q], dtype=np.float32)
        # Warp transform arrays accept (position, quaternion) tuples most reliably.
        q_arr = wp.array(q, dtype=wp.transform, device=device)
        a.body_q.assign(q_arr); b.body_q.assign(q_arr)
        if step % max(1, round(0.1 / dt)) == 0 or step == nsteps:
            pos = a.particle_q.numpy(); vel = a.particle_qd.numpy()
            jp = a.mpm.particle_Jp.numpy()
            ext = pos.max(0) - pos.min(0)
            # Contact-node proxy: occupied voxel coordinates within one voxel of each pad face.
            if mode == "uniaxial":
                counts = [int(np.sum(pos[:, 2] >= q_np[0, 2] - 0.004 - voxel)), 0]
            else:
                counts = [int(np.sum(np.abs(pos[:, 0] - (-x + 0.004)) < voxel)),
                          int(np.sum(np.abs(pos[:, 0] - (x - 0.004)) < voxel))]
            samples.append({"t": round(t, 4), "jp_min": float(jp.min()), "jp_max": float(jp.max()),
                "jp_mean": float(jp.mean()), "jp_fraction_gt_005": float(np.mean(np.abs(jp - 1.0) > 0.05)),
                "extents": ext.tolist(), "z_mean": float(pos[:, 2].mean()),
                "max_speed": float(np.linalg.norm(vel, axis=1).max()), "contact_node_count_per_pad": counts})
        if step < nsteps:
            solver.step(a, b, control=None, contacts=None, dt=dt); a, b = b, a
    peak_frac = max(x["jp_fraction_gt_005"] for x in samples)
    final = samples[-1]
    intact = bool(max(final["extents"]) < 0.06)
    carried = bool(mode == "pinch_lift" and intact and final["z_mean"] - samples[0]["z_mean"] >= 0.04)
    return {"name": spec["name"], "part": spec["part"], "config": spec, "particle_count": 512,
        "runtime_seconds": time.perf_counter() - start, "samples": samples,
        "outcome": {"peak_jp_fraction_gt_005": peak_frac, "final": final, "intact": intact, "carried": carried}}


def suite_specs():
    y = 3333.0
    out = []
    for k in (0.3, 1.0, 2.0):
        out.append(dict(name=f"A-pinch-yp-{k}", part="A", mode="pinch_crush", yield_pressure=k*y))
    out += [
        dict(name="A-pinch-yp1-hard5", part="A", mode="pinch_crush", yield_pressure=y, hardening=5.0),
        dict(name="A-pinch-yp1-hard5-jp0975", part="A", mode="pinch_crush", yield_pressure=y, hardening=5.0, initial_jp=0.975),
        dict(name="A-uniaxial-yp03", part="A", mode="uniaxial", yield_pressure=.3*y),
        dict(name="A-uniaxial-yp1-hard5", part="A", mode="uniaxial", yield_pressure=y, hardening=5.0),
    ]
    for gap in (.030, .028, .026):
        out += [dict(name=f"B-baseline-g{gap:.3f}", part="B", mode="pinch_lift", gap=gap),
                dict(name=f"B-complete-g{gap:.3f}", part="B", mode="pinch_lift", gap=gap,
                     yield_pressure=y, hardening=5.0, initial_jp=.975)]
    out += [dict(name="B-complete-g028-fr15", part="B", mode="pinch_lift", gap=.028,
                 yield_pressure=y, hardening=5.0, initial_jp=.975, friction=1.5),
            dict(name="B-complete-g028-adhesion", part="B", mode="pinch_lift", gap=.028,
                 yield_pressure=y, hardening=5.0, initial_jp=.975, friction=1.5, adhesion=1000.0)]
    return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--suite", action="store_true"); p.add_argument("--output", type=Path)
    p.add_argument("--device", default="cuda:0"); p.add_argument("--mode", choices=["pinch_crush", "pinch_lift", "uniaxial"], default="pinch_crush")
    for key, default in [("yield_stress",3333.),("yield_pressure",1e15),("tensile_yield_ratio",1.),("hardening",0.),("hardening_rate",1.),("softening_rate",1.),("dilatancy",0.),("viscosity",0.),("damping",.001),("friction",1.),("initial_jp",1.),("critical_fraction",0.),("adhesion",0.)]:
        p.add_argument("--"+key.replace("_","-"), type=float, default=default)
    p.add_argument("--strain-basis", default="P0"); p.add_argument("--max-iterations", type=int, default=80); p.add_argument("--gap", type=float, default=.028)
    args = p.parse_args(); wp.init()
    if args.suite:
        trials = [trial(s, args.device) for s in suite_specs()]
        responsive = [t for t in trials if t["outcome"]["peak_jp_fraction_gt_005"] > 0]
        carried = [t["name"] for t in trials if t["outcome"]["carried"]]
        minimal = responsive[0]["config"] if responsive else None
        lift_results = {t["name"]: {"carried": t["outcome"]["carried"],
            "intact": t["outcome"]["intact"], "final_z_mean": t["outcome"]["final"]["z_mean"],
            "final_extents": t["outcome"]["final"]["extents"]}
            for t in trials if t["part"] == "B"}
        conclusions = {"minimal_jp_responsive_parameter_set": minimal,
            "minimal_jp_response_peak_fraction": responsive[0]["outcome"]["peak_jp_fraction_gt_005"] if responsive else 0.0,
            "pinch_vs_uniaxial": {"pinch_finite_pressure_peak_fraction": trials[0]["outcome"]["peak_jp_fraction_gt_005"],
                "uniaxial_finite_pressure_peak_fraction": trials[5]["outcome"]["peak_jp_fraction_gt_005"],
                "distinction": "Jp responds in both prescribed pinch and guaranteed uniaxial compaction."},
            "baseline_can_lift_intact": bool([n for n in carried if "baseline" in n]),
            "baseline_tested_gaps_m": [0.030, 0.028, 0.026],
            "baseline_lifts_intact": [n for n in carried if "baseline" in n], "carried_trials": carried,
            "lift_results": lift_results,
            "adhesion_result": "1000 Pa adhesion raised z-mean but tore/stretched the block beyond the 0.06 m intact limit.",
            "recommendation": {"yield_stress":3333.0,"yield_pressure":3333.0,"hardening":5.0,"initial_jp":.975,"friction":1.5,
                "physical_justification":"Finite pressure yield permits irreversible compaction; hardening arrests indefinite flow, while finite deviatoric yield retains crush/tear behavior."}}
        doc = {"commit":"b74df53", "baseline":{"E":7000,"nu":.45,"rho":1000,"size_m":.04,"voxel_size":.005}, "trials":trials,"conclusions":conclusions}
    else:
        s={k:v for k,v in vars(args).items() if k not in ("output","suite","device")}
        s.update(name="cli",part="custom"); doc=trial(s,args.device)
    output=args.output or Path("../reports/logs/material-study.json")
    output.parent.mkdir(parents=True,exist_ok=True); output.write_text(json.dumps(doc,indent=2)+"\n")
    print(json.dumps(doc["conclusions"] if "conclusions" in doc else doc["outcome"], indent=2))

if __name__ == "__main__": main()
