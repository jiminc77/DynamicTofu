"""SolverCoupledProxy assembly + per-finger action-reaction (AR) gate.

Contract (pending-approval.md, "File-level changes" + "Action-reaction gate"):
- Rigid Entry (SolverMuJoCo, substeps=4) + MPM Entry (in_place=True).
- Proxy lagged, collision_pipeline=lambda _m: None, Config(iterations=1),
  mass_scale=1.0 (sensitivity probed at G-N2 across iterations {1,2,4}).
- Exposes solver("mpm"); owns the PER-FINGER AR check.

AR gate grounding: the harvest kernel (solver_implicit_mpm.py:3990-3993)
computes f_world = impulse/dt and atomically accumulates
spatial_vector(f_world, cross(pos - com_world, f_world)) per body, i.e.
components [0:3] = force [N], [3:6] = torque [N*m]. The harvested per-body
wrench is read from the proxy mapping's `coupling_forces` buffer.

Frozen AR tolerances:
- AR-1 force  ||F_b - F_b_harvest||  <= max(0.02*||F_b_harvest||, 0.01 N)
- AR-1 torque ||T_b - T_b_harvest||  <= max(0.02*||T_b_harvest||, 1e-4 N*m)
- AR-2 balance | |F_L.nL| - |F_R.nR| | <= max(0.10*max(.), 0.02 N), compressive sign
- AR-3 block-present finger deflection > 10x block-absent noise floor
- AR-4 global residual: LOGGED ONLY, never a gate.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import warp as wp

import newton
from newton.solvers import SolverImplicitMPM, SolverMuJoCo
from newton.solvers.experimental.coupled import SolverCoupledProxy

# frozen coupling parameters (config block)
RIGID_SUBSTEPS = 4
PROXY_ITERATIONS = 1
MASS_SCALE = 1.0
MPM_MAX_ITERATIONS = 50


def make_mpm_config(voxel_size: float) -> "SolverImplicitMPM.Config":
    cfg = SolverImplicitMPM.Config()
    cfg.voxel_size = voxel_size
    cfg.grid_type = "sparse"
    cfg.max_iterations = MPM_MAX_ITERATIONS
    return cfg


def coupling_params_dict(frame_dt: float, voxel_size: float) -> dict:
    return {
        "composition": "SolverCoupledProxy(mjc: SolverMuJoCo substeps=4; mpm: SolverImplicitMPM in_place)",
        "rigid_substeps": RIGID_SUBSTEPS,
        "proxy_iterations": PROXY_ITERATIONS,
        "proxy_mode": "lagged",
        "mass_scale": MASS_SCALE,
        "mpm_max_iterations": MPM_MAX_ITERATIONS,
        "voxel_size_m": voxel_size,
        "frame_dt_s": frame_dt,
    }


def build_coupled_solver(
    model: newton.Model,
    meta,
    *,
    proxy_iterations: int = PROXY_ITERATIONS,
    rigid_substeps: int = RIGID_SUBSTEPS,
    mass_scale: float = MASS_SCALE,
    voxel_size: float,
):
    """Mirror of examples/multiphysics/example_mujoco_mpm_coupled_solver.py."""
    rigid_bodies = list(range(*meta.rigid_body_range))
    mpm_config = make_mpm_config(voxel_size)

    if model.particle_count == 0:
        # block-absent twin (AR-3): identical rigid entry/substepping, no MPM entry
        return SolverCoupledProxy(
            model=model,
            entries=[
                SolverCoupledProxy.Entry(
                    name="mjc",
                    solver=lambda v: SolverMuJoCo(model=v, use_mujoco_contacts=False, njmax=256, nconmax=256),
                    bodies=rigid_bodies,
                    joints=list(range(model.joint_count)),
                    substeps=rigid_substeps,
                ),
            ],
            coupling=SolverCoupledProxy.Config(proxies=[], iterations=proxy_iterations),
        )

    solver = SolverCoupledProxy(
        model=model,
        entries=[
            SolverCoupledProxy.Entry(
                name="mjc",
                solver=lambda v: SolverMuJoCo(model=v, use_mujoco_contacts=False, njmax=256, nconmax=256),
                bodies=rigid_bodies,
                joints=list(range(model.joint_count)),
                substeps=rigid_substeps,
            ),
            SolverCoupledProxy.Entry(
                name="mpm",
                solver=lambda v: SolverImplicitMPM(model=v, config=mpm_config),
                particles=list(range(model.particle_count)),
                in_place=True,
            ),
        ],
        coupling=SolverCoupledProxy.Config(
            proxies=[
                SolverCoupledProxy.Proxy(
                    source="mjc",
                    destination="mpm",
                    bodies=rigid_bodies,
                    mass_scale=mass_scale,
                    mode="lagged",
                    collision_pipeline=lambda _model: None,
                )
            ],
            iterations=proxy_iterations,
        ),
    )
    return solver


def mpm_entry_state(solver):
    """The MPM entry's OWN state. The coupled wrapper does not sync custom
    state attributes (mpm:particle_Jp etc.) back to the parent state - the
    parent buffer stays at its initial value. All Jp/tactile reads MUST use
    this state. (Found empirically: parent Jp stayed 1.0 while the entry
    state showed Jp in [0.39, 957] under a crush.)"""
    return solver._entries["mpm"].state_0


def harvested_body_wrenches(solver) -> np.ndarray:
    """Per-body harvested spatial wrenches ([:, 0:3]=force N, [:, 3:6]=torque N*m)."""
    mappings = getattr(solver, "_proxy_mappings", None)
    if not mappings:
        raise RuntimeError("no proxy mappings on coupled solver; is the Proxy configured?")
    forces = mappings[0].coupling_forces
    if forces is None:
        raise RuntimeError("coupling_forces buffer not allocated")
    return forces.numpy().reshape(-1, 6)


def node_reduction_per_body(mpm_solver, state, body_q: np.ndarray, body_com: np.ndarray, dt_mpm: float):
    """Mirror the harvest kernel host-side: per-body force/torque from grid nodes.

    Returns dict body_index -> (F [3] N, Tau [3] N*m, n_nodes).
    """
    impulses, positions, collider_ids = mpm_solver.collect_collider_impulses(state)
    imp = impulses.numpy()
    pos = positions.numpy()
    cid = collider_ids.numpy().astype(int)
    body_of_collider = mpm_solver.collider_body_index.numpy().astype(int)

    out: dict[int, tuple[np.ndarray, np.ndarray, int]] = {}
    valid = cid >= 0
    for i in np.nonzero(valid)[0]:
        b = body_of_collider[cid[i]] if cid[i] < len(body_of_collider) else -1
        if b < 0:
            continue
        f = imp[i] / dt_mpm
        if not np.any(f):
            continue
        # com in world frame: body_q is (px,py,pz,qx,qy,qz,qw)
        q = body_q[b]
        com_world = q[:3] + _quat_rotate(q[3:7], body_com[b])
        r = pos[i] - com_world
        F, T, n = out.get(b, (np.zeros(3), np.zeros(3), 0))
        out[b] = (F + f, T + np.cross(r, f), n + 1)
    return out


def _quat_rotate(q_xyzw: np.ndarray, v: np.ndarray) -> np.ndarray:
    x, y, z, w = q_xyzw
    u = np.array([x, y, z])
    return 2.0 * np.dot(u, v) * u + (w * w - np.dot(u, u)) * v + 2.0 * w * np.cross(u, v)


@dataclass
class ARResult:
    ar1_pass: bool
    ar2_pass: bool
    per_finger: dict
    balance_residual_n: float
    global_residual_n: float  # AR-4, logged only

    def to_dict(self) -> dict:
        return {
            "ar1_pass": bool(self.ar1_pass),
            "ar2_pass": bool(self.ar2_pass),
            "per_finger": self.per_finger,
            "balance_residual_n": float(self.balance_residual_n),
            "global_residual_n_LOGGED_ONLY": float(self.global_residual_n),
        }


def ar_check(
    reduced: dict,
    harvested: np.ndarray,
    finger_bodies: list[int],
    pad_normals_world: dict[int, np.ndarray],
) -> ARResult:
    """AR-1 + AR-2 per finger; AR-4 computed for logging only."""
    per_finger = {}
    ar1_ok = True
    normal_mags = {}
    fsum = np.zeros(3)
    for b in finger_bodies:
        F, T, n_nodes = reduced.get(b, (np.zeros(3), np.zeros(3), 0))
        Fh = harvested[b, 0:3]
        Th = harvested[b, 3:6]
        f_err = float(np.linalg.norm(F - Fh))
        t_err = float(np.linalg.norm(T - Th))
        f_tol = max(0.02 * float(np.linalg.norm(Fh)), 0.01)
        t_tol = max(0.02 * float(np.linalg.norm(Th)), 1e-4)
        f_ok = f_err <= f_tol
        t_ok = t_err <= t_tol
        ar1_ok = ar1_ok and f_ok and t_ok
        nb = pad_normals_world[b]
        normal_comp = float(np.dot(F, nb))
        normal_mags[b] = normal_comp
        fsum += F
        per_finger[str(b)] = {
            "n_nodes": int(n_nodes),
            "F_reduced_n": F.tolist(),
            "F_harvest_n": Fh.tolist(),
            "force_err_n": f_err,
            "force_tol_n": f_tol,
            "force_ok": bool(f_ok),
            "Tau_reduced_nm": T.tolist(),
            "Tau_harvest_nm": Th.tolist(),
            "torque_err_nm": t_err,
            "torque_tol_nm": t_tol,
            "torque_ok": bool(t_ok),
            "normal_component_n": normal_comp,
        }
    mags = [abs(v) for v in normal_mags.values()]
    compressive = all(v > 0.0 for v in normal_mags.values())
    balance = abs(mags[0] - mags[1]) if len(mags) == 2 else float("inf")
    ar2_tol = max(0.10 * max(mags), 0.02) if mags else 0.0
    ar2_ok = compressive and balance <= ar2_tol
    return ARResult(
        ar1_pass=ar1_ok,
        ar2_pass=bool(ar2_ok),
        per_finger=per_finger,
        balance_residual_n=float(balance),
        global_residual_n=float(np.linalg.norm(fsum)),
    )
