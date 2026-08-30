"""Read-only per-pad soft/rigid contact wrench instrumentation for Newton b74df534."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import warp as wp

_NEWTON_PIN = "b74df534"
try:
    from newton._src.solvers.vbd.vbd_coupling_kernels import (
        _eval_soft_ef_contact,
        _harvest_vbd_body_particle_contact_forces_on_proxy_bodies_kernel,
    )
except ImportError as exc:  # isolate the deliberately private Newton dependency
    raise ImportError(
        f"pad_wrench requires Newton pin {_NEWTON_PIN}: private VBD coupling API unavailable"
    ) from exc


@dataclass(frozen=True)
class PreStepState:
    """Persistent snapshots required by the VBD contact force law."""

    body_q: wp.array
    particle_q: wp.array


def capture_pre_step(state) -> PreStepState:
    """Copy positions before collision/solver.step; returned buffers outlive state swaps."""
    return PreStepState(body_q=wp.clone(state.body_q), particle_q=wp.clone(state.particle_q))


@wp.kernel(enable_backward=False)
def _dump_pad_soft_contacts_kernel(
    dt: float,
    body_local_to_pad: wp.array[int],
    particle_q: wp.array[wp.vec3],
    particle_q_prev: wp.array[wp.vec3],
    particle_radius: wp.array[float],
    body_q: wp.array[wp.transform],
    body_q_prev: wp.array[wp.transform],
    body_qd: wp.array[wp.spatial_vector],
    body_com: wp.array[wp.vec3],
    friction_epsilon: float,
    penalty_k: wp.array[float],
    material_kd: wp.array[float],
    material_mu: wp.array[float],
    contact_count: wp.array[int],
    contact_indices: wp.array[wp.vec3i],
    contact_barycentric: wp.array[wp.vec3],
    contact_shape: wp.array[int],
    contact_body_pos: wp.array[wp.vec3],
    contact_body_vel: wp.array[wp.vec3],
    contact_normal: wp.array[wp.vec3],
    shape_margin: wp.array[float],
    shape_body: wp.array[wp.int32],
    out_force_body: wp.array[wp.vec3],
    out_contact_point: wp.array[wp.vec3],
    out_pad_id: wp.array[int],
):
    i = wp.tid()
    if i >= contact_count[0]:
        return
    shape = contact_shape[i]
    if shape < 0 or shape >= shape_body.shape[0]:
        return
    body = shape_body[shape]
    if body < 0 or body >= body_local_to_pad.shape[0]:
        return
    pad = body_local_to_pad[body]
    if pad < 0 or pad >= 2:
        return
    corners = contact_indices[i]
    if corners[0] < 0 or corners[0] >= particle_q.shape[0]:
        return
    force_particle, _hess, point = _eval_soft_ef_contact(
        i, corners, contact_barycentric[i], particle_q, particle_q_prev,
        particle_radius, penalty_k[i], material_kd[i], material_mu[i],
        friction_epsilon, shape_body, body_q, body_q_prev, body_qd, body_com,
        contact_shape, contact_body_pos, contact_body_vel, contact_normal,
        shape_margin, dt,
    )
    out_force_body[i] = -force_particle
    out_contact_point[i] = point
    out_pad_id[i] = pad


def _mapping(rig):
    mapping = np.full(rig.model.body_count, -1, dtype=np.int32)
    mapping[rig.b_left], mapping[rig.b_right] = 0, 1
    return wp.array(mapping, dtype=int, device=rig.model.device)


def _common_inputs(rig, pre_state, post_state, contacts, dt, mapping):
    s = rig.solver
    return [float(dt), mapping, post_state.particle_q, pre_state.particle_q,
            rig.model.particle_radius, post_state.body_q, pre_state.body_q,
            post_state.body_qd, rig.model.body_com, float(s.friction_epsilon),
            s.body_particle_contact_penalty_k, s.body_particle_contact_material_kd,
            s.body_particle_contact_material_mu, contacts.soft_contact_count,
            contacts.soft_contact_indices, contacts.soft_contact_barycentric,
            contacts.soft_contact_shape, contacts.soft_contact_body_pos,
            contacts.soft_contact_body_vel, contacts.soft_contact_normal,
            rig.model.shape_margin, rig.model.shape_body]


def collect_pad_contacts(rig, *, pre_state, post_state, contacts, dt):
    """Return active pad contacts in deterministic soft-contact-index order."""
    count_max = int(contacts.soft_contact_max)
    device = rig.model.device
    force = wp.zeros(count_max, dtype=wp.vec3, device=device)
    point = wp.zeros(count_max, dtype=wp.vec3, device=device)
    pad = wp.full(count_max, -1, dtype=int, device=device)
    inputs = _common_inputs(rig, pre_state, post_state, contacts, dt, _mapping(rig))
    wp.launch(_dump_pad_soft_contacts_kernel, dim=count_max, inputs=inputs,
              outputs=[force, point, pad], device=device)
    ids, forces, points = pad.numpy(), force.numpy(), point.numpy()
    return [{"contact_index": i, "pad_id": "left" if ids[i] == 0 else "right",
             "contact_point_world": points[i].astype(float).tolist(),
             "force_on_body_world": forces[i].astype(float).tolist()}
            for i in range(count_max) if ids[i] in (0, 1)]


def _rotate(q, v):
    qv = np.asarray(q[:3], dtype=np.float64)
    v = np.asarray(v, dtype=np.float64)
    return v + 2.0 * np.cross(qv, np.cross(qv, v) + float(q[3]) * v)


def collect_pad_wrench(rig, *, pre_state, post_state, contacts, dt):
    """Collect atomic kernel wrench plus a deterministic float64 contact sum."""
    device, mapping = rig.model.device, _mapping(rig)
    reduced = wp.zeros(2, dtype=wp.spatial_vector, device=device)
    wp.launch(_harvest_vbd_body_particle_contact_forces_on_proxy_bodies_kernel,
              dim=int(contacts.soft_contact_max),
              inputs=_common_inputs(rig, pre_state, post_state, contacts, dt, mapping),
              outputs=[reduced], device=device)
    kernel = reduced.numpy().astype(np.float64)
    records = collect_pad_contacts(rig, pre_state=pre_state, post_state=post_state,
                                   contacts=contacts, dt=dt)
    stable = np.zeros((2, 6), dtype=np.float64)
    body_q = post_state.body_q.numpy()
    com_local = rig.model.body_com.numpy()
    for rec in records:  # already in increasing contact index order
        p = 0 if rec["pad_id"] == "left" else 1
        body = rig.b_left if p == 0 else rig.b_right
        f = np.asarray(rec["force_on_body_world"], dtype=np.float64)
        cp = np.asarray(rec["contact_point_world"], dtype=np.float64)
        xform = body_q[body]
        com = np.asarray(xform[:3], dtype=np.float64) + _rotate(xform[3:7], com_local[body])
        stable[p, :3] += f
        stable[p, 3:] += np.cross(cp - com, f)
    result = {}
    for p, (name, body, rest_n) in enumerate((("left", rig.b_left, (0., 1., 0.)),
                                                ("right", rig.b_right, (0., -1., 0.)))):
        n = _rotate(body_q[body][3:7], rest_n)
        n /= np.linalg.norm(n)
        f = kernel[p, :3]
        fn = float(np.dot(f, n))
        ft = f - fn * n
        result[name] = {"force_world": f.tolist(), "torque_world": kernel[p, 3:].tolist(),
                        "force_world_stable": stable[p, :3].tolist(),
                        "torque_world_stable": stable[p, 3:].tolist(),
                        "Fn": fn, "Ft_vec": ft.tolist(), "Ft": float(np.linalg.norm(ft)),
                        "n_contacts": sum(r["pad_id"] == name for r in records)}
    return result
