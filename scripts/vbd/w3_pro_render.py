"""Professor-facing render of the frozen W3 dense trajectories (render only).

Usage: .venv-render/bin/python scripts/vbd/w3_pro_render.py --render
       .venv-render/bin/python scripts/vbd/w3_pro_render.py --scene slip
       .venv-render/bin/python scripts/vbd/w3_pro_render.py --smoke
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
import textwrap
from collections import defaultdict
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
CLIPS = ROOT / "reports/vbd/clips"
SCENES = ("intact", "slip", "damage")
KEY_TIMES = {"grip": 1.80, "lift": 4.30, "hold": 9.30, "accel_out_peak": 9.40,
             "dwell": 9.80, "return": 10.10, "settle": 10.60}
PAD_HALF = np.array((.022, .006, .022))
PAD_MOUNT_Z_OFFSET = -0.0568  # src/vbd_rig_panda.py
SOFT_CONTACT_MARGIN = 1e-3  # frozen src/frozen_config.py value
# The stored surface sits about 1.85 mm proud of the collider face even under
# firm grip. 3 mm is the smallest millimetre band yielding a stable footprint.
CONTACT_PROXIMITY_BAND = 3e-3
TAXEL_COLOR_CEILING = 0.6e-3
BOX_FACES = np.array(((0,1,3,2),(4,6,7,5),(0,4,5,1),(2,3,7,6),(0,2,6,4),(1,5,7,3)))
WIDTH, HEIGHT, FPS = 1280, 720, 30
BODY_REORDER = None
PANDA_RIG_GEOMETRY = None
FRAME_FORCE = None


def boundary_triangles(tets, vertices):
    faces = defaultdict(list)
    for a,b,c,d in np.asarray(tets, int):
        for f,o in (((a,b,c),d),((a,b,d),c),((a,c,d),b),((b,c,d),a)):
            faces[tuple(sorted(f))].append((list(f), o))
    out = []
    for entries in faces.values():
        if len(entries) != 1: continue
        f,o = entries[0]
        a,b,c = vertices[f]
        if np.dot(np.cross(b-a,c-a), vertices[o]-a) > 0: f[1],f[2] = f[2],f[1]
        out.append(f)
    return np.asarray(out, int)


def rotation(q):
    x,y,z,w = np.asarray(q,float) / max(np.linalg.norm(q), 1e-12)
    return np.array(((1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)),
                     (2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)),
                     (2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y))))


def transform(pose):
    m=np.eye(4); m[:3,:3]=rotation(pose[3:]); m[:3,3]=pose[:3]; return m


def v11_hand_visual_pose(pose):
    """Place the shell from the URDF hand frame inferred from finger joints."""
    matrix=np.eye(4)
    # URDF finger origins are +58.4 mm from fr3_hand. The visual shell needs
    # its arm-mount face above that plane; flip about mesh Y, then seat its
    # 66 mm extremum flush with the finger-mount plane.
    matrix[:3,:3]=np.diag((-1.,1.,-1.))
    matrix[:3,3]=np.array((0.,0.,.1244))
    return matrix


def v11_shell_attachment_checks():
    geometry=panda_rig_geometry()
    transformed=[]
    centers=[]
    pose=v11_hand_visual_pose(geometry["shape_transform"][0])
    for i in np.flatnonzero(geometry["shape_body"] == 1):
        va,vb=geometry["vertex_range"][i]
        vertices=geometry["vertices"][va:vb] @ pose[:3,:3].T + pose[:3,3]
        transformed.append(vertices)
        centers.append(vertices.mean(axis=0))
    shell=np.concatenate(transformed)
    gap_mm=abs(float(shell[:,2].min())-.0584)*1000
    housing_centroid_z=centers[2][2]
    flange_above=bool(centers[1][2] > housing_centroid_z
                      and centers[0][2] > housing_centroid_z
                      and centers[4][2] > housing_centroid_z)
    if gap_mm > 3 or not flange_above:
        raise RuntimeError("v11 shell attachment 3D gate failed")
    return {"gap_mm":gap_mm,"flange_above_centroid":flange_above}


def offset_pose(pose, xyz):
    matrix = transform(pose)
    matrix[:3, 3] += matrix[:3, :3] @ np.asarray(xyz, float)
    out = np.empty(7)
    out[:3] = matrix[:3, 3]
    out[3:] = pose[3:]
    return out


def panda_rig_geometry():
    """Load the exact visible shapes exported from the finalized PandaRig model."""
    global PANDA_RIG_GEOMETRY
    if PANDA_RIG_GEOMETRY is None:
        path=CLIPS/"panda"/"panda_rig_geometry.npz"
        if not path.exists():
            raise FileNotFoundError(f"missing PandaRig geometry export: {path}")
        with np.load(path,allow_pickle=False) as data:
            PANDA_RIG_GEOMETRY={key:np.array(data[key]) for key in data.files}
    return PANDA_RIG_GEOMETRY


def look_at(eye, target):
    # OpenGL camera looks down local -Z.
    z=(eye-target); z/=np.linalg.norm(z); x=np.cross(np.array((0.,0.,1.)),z); x/=np.linalg.norm(x)
    y=np.cross(z,x); m=np.eye(4); m[:3,:3]=np.column_stack((x,y,z)); m[:3,3]=eye; return m


def snapshots(scene, version=2):
    if version >= 8:
        files=sorted((CLIPS/"panda"/f"w3_{scene}_force40").glob("f_*.npz"))
        if not files: raise FileNotFoundError(f"missing force40 Panda trajectory for {scene}")
        return files
    if version >= 5:
        files=sorted((CLIPS/"panda"/f"w3_{scene}_dense").glob("f_*.npz"))
        if not files: raise FileNotFoundError(f"missing frozen Panda dense trajectory for {scene}")
        return files
    if scene == "slip" and version >= 4:
        suffix = "slip_dense_v4"
    elif scene == "slip" and version >= 3:
        suffix = "slip_dense_ext"
    else:
        suffix = f"{scene}_dense"
    files=sorted((CLIPS/f"w3_{suffix}").glob("f_*.npz"))
    if not files: raise FileNotFoundError(f"missing frozen dense trajectory for {scene}")
    return files


def load_frame(path):
    global FRAME_FORCE
    with np.load(path) as d:
        bodies=np.array(d["body_q"])
        if BODY_REORDER is not None:
            bodies=bodies[BODY_REORDER]
        FRAME_FORCE=({
            "fn_left":np.array(d["taxel_fn_left"]),
            "fn_right":np.array(d["taxel_fn_right"]),
            "ft_left":np.array(d["taxel_ft_left"]),
            "ft_right":np.array(d["taxel_ft_right"]),
            "net_left":np.array(d["net_shear_left"]),
            "net_right":np.array(d["net_shear_right"]),
        } if "taxel_fn_left" in d else None)
        return np.array(d["particle_q"]),bodies,float(d["t"])


def camera_for(files, scene, version=2):
    if version >= 5 and scene == "slip":
        manifest=json.loads((CLIPS/"panda"/"w3_panda_manifest.json").read_text())
        meta=next(item for item in manifest["scenes"] if item["scene"]=="slip")
        drop_t=float(meta["drop_t"])
        hand_x=[]
        landing=[]
        for path in files:
            q,b,t=load_frame(path)
            hand_x.append(float(b[1,0]))
            if t >= drop_t:
                landing.append(q[:,(0,2)])
        landing=np.concatenate(landing)
        margin=.07
        lo=min(min(hand_x),float(landing[:,0].min()))-margin
        hi=max(max(hand_x),float(landing[:,0].max()))+margin
        zlo=max(0.,float(landing[:,1].min())-.025)
        zhi=max(.14,float(landing[:,1].max())+.04)
        mid=(lo+hi)/2
        target=np.array((mid,0,(zlo+zhi)/2+.035))
        # Horizontal FOV is wider than vertical at 16:9. This distance keeps
        # both endpoints visible without making the grip event needlessly tiny.
        horizontal=max(.72,(hi-lo)*.98)/np.sqrt(2)
        eye=target+np.array((-horizontal,-horizontal,.27))
        if version >= 9:
            # Look across the hand's wide x/y face while cropping the tall
            # wrist coupling; retain the wide slip landing bounds.
            target=np.array((mid,0,max(.065,(zlo+zhi)/2+.015)))
            eye=target+np.array((-.42,-.52,.20))
        if version >= 10:
            target=np.array((mid,0,.14))
            eye=target+np.array((-.68,-.78,.30))
        return look_at(eye,target),(lo,hi),eye,target
    xs=[]
    for p in files:
        with np.load(p) as d:
            xs.append(float(d["body_q"][1,0]))
    # Damage has a longer actual track than the other scenes, so the old fixed
    # 10 cm padding made it tiny. Preserve its whole recorded excursion but use
    # only presentation clearance around that used segment.
    margin = .025 if scene == "damage" else .10
    lo,hi=min(xs)-margin,max(xs)+margin; mid=(lo+hi)/2; span=hi-lo
    target=np.array((mid,0,.050))
    distance = max(.34, span * (.90 if scene == "damage" else 1.35))
    eye=target+np.array((.12,-distance,.20))
    if version >= 5:
        # The genuine hand is substantially larger than the former cosmetic
        # shell. Keep one fixed, front three-quarter world camera for the
        # entire P-rig scene so the tofu/pad gap and finger length both read.
        target=np.array((mid,0,.10))
        horizontal=max(.60,span*1.7)/np.sqrt(2)
        eye=target+np.array((-horizontal,-horizontal,.27))
    if version >= 9:
        # The useful lower wedge, fingers, pads and tofu occupy the bottom
        # 10--12 cm of the exported hand. Aim below the wrist coupling.
        target=np.array((mid,0,.062))
        eye=target+np.array((-.34,-.42,.17))
    if version >= 10:
        target=np.array((mid,0,.14))
        eye=target+np.array((-.48,-.58,.24))
    if version >= 11 and scene == "intact":
        # Preserve the damage camera angle but tighten its distance so the
        # frozen 4.5 cm intact excursion remains visibly measurable.
        start_x=xs[0]
        target=np.array((start_x,0,.11))
        eye=target+np.array((-.48,-.58,.24))*.31
    return look_at(eye,target), (lo,hi), eye, target


def slip_settle_index(files, drop_t, radius=.01):
    """First grounded frame whose COM remains settled for 0.5 seconds."""
    frames=[load_frame(path) for path in files]
    times=np.array([frame[2] for frame in frames])
    com=np.array([frame[0].mean(axis=0) for frame in frames])
    zmin=np.array([frame[0][:,2].min() for frame in frames])
    dt=float(np.median(np.diff(times)))
    window=max(2,int(round(.5/dt)))
    for i in range(len(frames)-window+1):
        displacement=np.linalg.norm(com[i:i+window]-com[i],axis=1)
        if times[i] >= drop_t and zmin[i] <= .003 and displacement.max() <= radius:
            return i
    raise RuntimeError("Panda slip block did not settle in the dense capture")


def slip_v3_cameras(files):
    """Tight event camera plus a wide aftermath camera for the ejected tofu."""
    primary=camera_for(snapshots("slip",2),"slip")
    xyz=[]
    for path in files:
        q,_,t=load_frame(path)
        if t >= 9.55: xyz.append(q)
    points=np.concatenate(xyz)
    lo,hi=points[:,0].min()-.10,points[:,0].max()+.10
    mid=(lo+hi)/2; target=np.array((mid,0,.045))
    eye=target+np.array((.12,-max(.40,(hi-lo)*1.25),.22))
    aftermath=(look_at(eye,target),(lo,hi),eye,target)
    return primary,aftermath


def rest_poses(tets, first_particles):
    x0=first_particles[tets[:,0]]
    dm=np.stack((first_particles[tets[:,1]]-x0,first_particles[tets[:,2]]-x0,
                 first_particles[tets[:,3]]-x0),axis=-1)
    return np.linalg.inv(dm)


def vertex_strain(q,tets,inv_dm):
    x0=q[tets[:,0]]
    ds=np.stack((q[tets[:,1]]-x0,q[tets[:,2]]-x0,q[tets[:,3]]-x0),axis=-1)
    f=ds@inv_dm; e=.5*(np.swapaxes(f,1,2)@f-np.eye(3)); s=np.linalg.eigvalsh(e)[:,-1]
    total=np.zeros(len(q)); count=np.zeros(len(q))
    for k in range(4): np.add.at(total,tets[:,k],s); np.add.at(count,tets[:,k],1)
    return total/np.maximum(count,1)


def damage_colors(strain):
    """Opaque tofu amber -> failure red over [0.10, 0.22] Green strain."""
    mix = np.clip((np.asarray(strain) - .10) / .12, 0, 1) ** 1.5
    amber = np.array((.91, .61, .16))
    red = np.array((.78, .06, .045))
    rgb = amber + mix[:, None] * (red - amber)
    return np.column_stack((rgb, np.ones(len(rgb))))


def egl_available():
    os.environ.setdefault("PYOPENGL_PLATFORM","egl")
    try:
        import pyrender
        r=pyrender.OffscreenRenderer(8,8); r.delete(); return True, "EGL/pyrender"
    except Exception as exc:
        return False, f"matplotlib fallback (EGL failed: {type(exc).__name__}: {exc})"


def add_hud(rgb, scene, t, meta, slow, version=2):
    im=Image.fromarray(rgb); d=ImageDraw.Draw(im,"RGBA")
    try: font=ImageFont.truetype("DejaVuSans.ttf",24); bold=ImageFont.truetype("DejaVuSans-Bold.ttf",28)
    except OSError: font=bold=ImageFont.load_default()
    d.rounded_rectangle((25,22,650,153),12,fill=(255,255,255,218),outline=(35,45,55,90),width=2)
    d.text((45,34),f"W3 - {scene.upper()}     t={t:.2f} s",font=bold,fill=(24,31,40,255))
    d.text((45,78),f"commanded a={meta['a']:g} m/s2 / realized a={meta['realized_accel']:g} m/s2",font=font,fill=(30,38,48,255))
    d.text((45,113),f"F_g={meta['F']:g} N",font=font,fill=(30,38,48,255))
    if version >= 6:
        try: small=ImageFont.truetype("DejaVuSans.ttf",12)
        except OSError: small=font
        disclosure=("real Franka fr3 hand/finger meshes ARE the simulated bodies (drawn from the model); "
                    "contact is pads-only; pad mount matched to frozen engagement height (fidelity).")
        for line_no, line in enumerate(textwrap.wrap(disclosure, width=112)):
            d.text((28,160+16*line_no),line,font=small,fill=(50,60,70,255))
    elif version >= 5:
        try: small=ImageFont.truetype("DejaVuSans.ttf",12)
        except OSError: small=font
        disclosure=("Panda-hand rig (real Franka fr3 hand + 2 fingers on the x/z transport carriage); "
                    "fingertips replaced with our sensor-format tactile pads; pad mount matched to the "
                    "frozen engagement height (fidelity).")
        for line_no, line in enumerate(textwrap.wrap(disclosure, width=112)):
            d.text((28,160+16*line_no),line,font=small,fill=(50,60,70,255))
    elif version >= 4:
        try: small=ImageFont.truetype("DejaVuSans.ttf",13)
        except OSError: small=font
        d.text((28,160),"camera tracks lateral assembly drift (rig artifact; labels unaffected)",
               font=small,fill=(50,60,70,255))
        d.text((28,178),"visual shell only; simulated bodies are pads + palm (floating rig, not a full Panda simulation)",
               font=small,fill=(50,60,70,255))
    if slow:
        d.rounded_rectangle((1000,28,1248,78),10,fill=(185,38,45,235))
        d.text((1018,39),"SLOW MOTION x4",font=font,fill="white")
    elif version >= 10 and 2.2 <= t < 8.7:
        d.rounded_rectangle((984,28,1248,78),10,fill=(44,91,140,235))
        d.text((1000,39),"FAST-FORWARD x8",font=font,fill="white")
    return np.asarray(im)


def pad_contact_footprint(vertices, pad_pose, inner_face_sign,
                          margin=CONTACT_PROXIMITY_BAND):
    """Select the geometry-only contact proxy and return pad-local x/z + centroid.

    The inner face is local y = ``inner_face_sign * PAD_HALF[1]``. A vertex is
    selected when its perpendicular face distance is <= the frozen soft-contact
    proximity band and its local x/z projection lies on the 44 x 44 mm face.
    """
    local = (np.asarray(vertices) - pad_pose[:3]) @ rotation(pad_pose[3:])
    on_face = np.abs(local[:, 1] - inner_face_sign * PAD_HALF[1]) <= margin
    on_pad = (np.abs(local[:, 0]) <= PAD_HALF[0]) & (np.abs(local[:, 2]) <= PAD_HALF[2])
    points = local[on_face & on_pad][:, (0, 2)]
    centroid = points.mean(axis=0) if len(points) else None
    return points, centroid


def pad_taxel_depth(vertices, boundary, pad_pose, inner_face_sign,
                    band=CONTACT_PROXIMITY_BAND):
    """Return an 8x8 max depth grid sampled across surface triangles."""
    triangles=np.asarray(vertices)[boundary]
    weights=np.array(((1.,0.,0.),(0.,1.,0.),(0.,0.,1.),
                      (.5,.5,0.),(.5,0.,.5),(0.,.5,.5),(1/3,1/3,1/3)))
    samples=np.einsum("sk,fkd->fsd",weights,triangles).reshape(-1,3)
    local = (samples - pad_pose[:3]) @ rotation(pad_pose[3:])
    distance = np.abs(local[:, 1] - inner_face_sign * PAD_HALF[1])
    selected = ((distance <= band) & (np.abs(local[:, 0]) <= PAD_HALF[0])
                & (np.abs(local[:, 2]) <= PAD_HALF[2]))
    points = local[selected]
    grid = np.zeros((8, 8), dtype=float)
    if len(points):
        ix = np.minimum(((points[:, 0] + PAD_HALF[0]) / (2 * PAD_HALF[0]) * 8).astype(int), 7)
        iz = np.minimum(((points[:, 2] + PAD_HALF[2]) / (2 * PAD_HALF[2]) * 8).astype(int), 7)
        depth = np.maximum(0, band - distance[selected])
        for x, z, value in zip(ix, iz, depth):
            grid[z, x] = max(grid[z, x], value)
    return grid, int(selected.sum())


def add_tactile_insets(rgb, q, bodies, boundary, taxels=False, per_frame=False, version=2):
    """Post-composite two geometry-proxy tactile panels onto RGB."""
    im = Image.fromarray(rgb); draw = ImageDraw.Draw(im, "RGBA")
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", 13)
        small = ImageFont.truetype("DejaVuSans.ttf", 11)
        tiny = ImageFont.truetype("DejaVuSans.ttf", 8)
    except OSError:
        font = small = tiny = ImageFont.load_default()
    surface_vertices = q[np.unique(boundary)]
    tofu_center = surface_vertices.mean(axis=0)
    panel_size, edge_margin, gap = 180, 18, 12
    right_x = WIDTH - edge_margin - panel_size
    left_x = right_x - gap - panel_size
    for label, body_index, x0 in (("L", 2, left_x), ("R", 3, right_x)):
        pose = (offset_pose(bodies[body_index], (0,0,PAD_MOUNT_Z_OFFSET))
                if version >= 5 else bodies[body_index])
        tofu_local = (tofu_center - pose[:3]) @ rotation(pose[3:])
        inner_face_sign = 1.0 if tofu_local[1] >= 0 else -1.0
        points, centroid = pad_contact_footprint(surface_vertices, pose, inner_face_sign)
        y0, size = HEIGHT - edge_margin - panel_size, panel_size
        draw.rounded_rectangle((x0, y0, x0 + size, y0 + size), 9,
                               fill=(255, 255, 255, 232), outline=(45, 55, 65, 180), width=2)
        force_mode=version >= 8
        title = (f"{label} normal force" if force_mode else
                 (f"{label} penetration depth" if taxels else f"{label} pad contact footprint"))
        draw.text((x0 + 8, y0 + 7), title, font=font, fill=(20, 28, 36, 255))
        caption = ("force: validated collector @ 40 iter" if force_mode else "(geometry proxy)")
        draw.text((x0 + 8, y0 + 24), caption,font=tiny if force_mode else small,
                  fill=(75, 82, 90, 255))
        if force_mode:
            draw.text((x0 + 8, y0 + 35), "arrows = shear on pad from tofu",
                      font=tiny,fill=(75,82,90,255))
        if taxels and not force_mode:
            draw.text((x0 + 8, y0 + 38), "ATTR=GEOMETRY_ONLY", font=small,
                      fill=(75, 82, 90, 255))
        left, top, side = x0 + 35, y0 + (58 if taxels else 48), (108 if taxels else 116)
        draw.rectangle((left, top, left + side, top + side), fill=(231, 238, 244, 245),
                       outline=(55, 65, 75, 220), width=1)
        draw.line((left, top + side, left + side, top + side), fill=(20, 30, 40), width=2)
        draw.line((left, top, left, top + side), fill=(20, 30, 40), width=2)
        draw.text((left + side - 9, top + side + 2), "x", font=small, fill=(20, 30, 40))
        draw.text((left - 13, top - 5), "z", font=small, fill=(20, 30, 40))
        def pixel(p):
            return (left + (p[0] / (.044) + .5) * side,
                    top + (1 - (p[1] / (.044) + .5)) * side)
        if force_mode:
            if FRAME_FORCE is None:
                raise RuntimeError("v8 frame lacks force collector arrays")
            side_name="left" if label=="L" else "right"
            # Capture stores each pad's force along its own inward normal.
            grid=np.maximum(FRAME_FORCE["fn_"+side_name],0)
            shear=FRAME_FORCE["ft_"+side_name]
            net=FRAME_FORCE["net_"+side_name]
            stops=np.array(((.267,.005,.329),(.190,.407,.556),(.208,.719,.473),(.993,.906,.144)))
            ceiling=max(float(grid.max()),1e-9)
            def force_color(value):
                u=np.clip(value/ceiling,0,1)*(len(stops)-1)
                j=min(int(u),len(stops)-2); c=stops[j]+(u-j)*(stops[j+1]-stops[j])
                return tuple((c*255).astype(int))+(235,)
            cell=side/8
            shear_scale=max(float(np.linalg.norm(shear,axis=2).max()),1e-9)
            for z in range(8):
                for x in range(8):
                    xa=left+x*cell; ya=top+(7-z)*cell
                    draw.rectangle((xa,ya,xa+cell,ya+cell),fill=force_color(grid[z,x]),
                                   outline=(245,245,245,150),width=1)
                    vector=shear[z,x]/shear_scale*(cell*.38)
                    center=np.array((xa+cell/2,ya+cell/2))
                    end=center+np.array((vector[0],-vector[1]))
                    draw.line((*center,*end),fill=(255,255,255,225),width=1)
            net_scale=max(float(np.linalg.norm(net)),shear_scale)
            start=np.array((left+side/2,top+side/2))
            end=start+np.array((net[0],-net[1]))/net_scale*(side*.35)
            draw.line((*start,*end),fill=(230,35,55,255),width=4)
            draw.ellipse((end[0]-3,end[1]-3,end[0]+3,end[1]+3),fill=(230,35,55,255))
            draw.text((x0+8,y0+166),f"max {ceiling:.3f} N | red: net shear",
                      font=tiny,fill=(20,28,36,255))
        elif taxels:
            grid, count = pad_taxel_depth(q, boundary, pose, inner_face_sign)
            # Compact viridis-like perceptually increasing blue-to-yellow ramp.
            stops = np.array(((.267,.005,.329),(.190,.407,.556),(.208,.719,.473),(.993,.906,.144)))
            grid_max=grid.max()
            color_ceiling = max(grid_max, .1e-3) if per_frame else TAXEL_COLOR_CEILING
            def color(value):
                # Do not amplify sub-0.05 mm numerical/proximity noise.
                visible_value = 0.0 if per_frame and grid_max < .05e-3 else value
                u=np.clip(visible_value/color_ceiling,0,1)*(len(stops)-1)
                j=min(int(u),len(stops)-2); c=stops[j]+(u-j)*(stops[j+1]-stops[j])
                return tuple((c*255).astype(int))+(235,)
            cell=side/8
            for z in range(8):
                for x in range(8):
                    xa=left+x*cell; ya=top+(7-z)*cell
                    draw.rectangle((xa,ya,xa+cell,ya+cell),fill=color(grid[z,x]),
                                   outline=(245,245,245,150),width=1)
            max_mm=grid.max()*1000
            draw.text((x0+8,y0+166),f"n={count}  max={max_mm:.1f} mm",
                      font=small,fill=(20,28,36,255))
        else:
            for point in points:
                px, py = pixel(point)
                draw.ellipse((px - 3, py - 3, px + 3, py + 3), fill=(25, 115, 190, 185))
            if centroid is not None:
                cx, cy = pixel(centroid)
                draw.line((cx - 7, cy, cx + 7, cy), fill=(210, 30, 45, 255), width=2)
                draw.line((cx, cy - 7, cx, cy + 7), fill=(210, 30, 45, 255), width=2)
            draw.text((x0 + 150, y0 + 145), f"n={len(points)}", font=small, fill=(20, 28, 36, 255))
    return np.asarray(im)


def compose_overlays(rgb, q, bodies, boundary, scene, t, meta, version=2):
    slow_window = (9.20,9.60) if version >= 4 else ((9.25,9.55) if version >= 3 else (9.20,9.40))
    slow = scene == "slip" and slow_window[0] <= t <= slow_window[1]
    rgb = add_hud(rgb, scene, t, meta, slow, version)
    return add_tactile_insets(rgb, q, bodies, boundary, taxels=version >= 3,
                              per_frame=version >= 4, version=version)


def pyrender_frame(q,bodies,t,scene,meta,boundary,tets,inv_dm,camera,xlim,version=2,
                   return_masks=False):
    import pyrender, trimesh
    sc=pyrender.Scene(bg_color=(.95,.96,.97,1),ambient_light=(.42,.42,.42))
    if scene=="damage":
        colors=(damage_colors(vertex_strain(q,tets,inv_dm))*255).astype(np.uint8)
        tm=trimesh.Trimesh(q,boundary,vertex_colors=colors,process=False)
        mesh=pyrender.Mesh.from_trimesh(tm,smooth=True)
    else:
        tm=trimesh.Trimesh(q,boundary,process=False)
        mat=pyrender.MetallicRoughnessMaterial(baseColorFactor=(.91,.66,.25,1),metallicFactor=.05,roughnessFactor=.38)
        mesh=pyrender.Mesh.from_trimesh(tm,material=mat,smooth=True)
    sc.add(mesh)
    def box(ext,color,pose,alpha=1.0):
        tm=trimesh.creation.box(extents=ext)
        mat=pyrender.MetallicRoughnessMaterial(
            baseColorFactor=(*color,alpha), metallicFactor=.22, roughnessFactor=.3,
            alphaMode="BLEND" if alpha < 1 else "OPAQUE")
        sc.add(pyrender.Mesh.from_trimesh(tm,material=mat),pose=pose)
    def capsule(radius,height,color,pose,alpha=1.0):
        tm=trimesh.creation.capsule(radius=radius,height=height,count=(12,12))
        mat=pyrender.MetallicRoughnessMaterial(
            baseColorFactor=(*color,alpha), metallicFactor=.05, roughnessFactor=.35,
            alphaMode="BLEND" if alpha < 1 else "OPAQUE")
        sc.add(pyrender.Mesh.from_trimesh(tm,material=mat),pose=pose)
    if version >= 5:
        geometry=panda_rig_geometry()
        shell_nodes=[]
        gripper_nodes=[]
        for i,body_index in enumerate(geometry["shape_body"]):
            shape_pose=transform(geometry["shape_transform"][i])
            if version >= 11 and body_index == 1:
                shape_pose=v11_hand_visual_pose(geometry["shape_transform"][i])
            world_pose=transform(bodies[body_index]) @ shape_pose
            color=geometry["shape_color"][i]
            if geometry["shape_kind"][i] == 8:
                metallic,roughness=.05,.34
                if version >= 10:
                    if i in (2,4):
                        color=np.array((.949,.949,.941))
                        metallic,roughness=.05,.40
                    elif i in (0,1,3):
                        color=np.array((.690,.690,.710))
                        metallic,roughness=.70,.40
                    elif i in (5,6,7,8):
                        color=np.array((.165,.165,.165))
                        metallic,roughness=.05,.55
                elif version >= 9:
                    if body_index == 1:
                        color=np.array((.88,.88,.88))
                    elif body_index in (2,3):
                        color=np.array((.12,.12,.12))
                material=pyrender.MetallicRoughnessMaterial(
                    baseColorFactor=(*color,1),metallicFactor=metallic,roughnessFactor=roughness,
                    emissiveFactor=(color*.28 if version >= 10 and i in (2,4) else None))
                va,vb=geometry["vertex_range"][i]
                fa,fb=geometry["face_range"][i]
                tm=trimesh.Trimesh(
                    geometry["vertices"][va:vb],
                    geometry["faces"][fa:fb]-va,process=False)
                node=sc.add(pyrender.Mesh.from_trimesh(tm,material=material,smooth=True),
                            pose=world_pose)
                gripper_nodes.append(node)
                if version >= 10 and i in (2,4):
                    shell_nodes.append(node)
            elif geometry["shape_kind"][i] == 7:
                # Sensor plates are deliberately matte; geometry and model
                # attachment remain exactly the exported simulated boxes.
                if version >= 9:
                    trim_color=((.12,.38,.92) if body_index == 2 else (.95,.34,.08))
                    trim_extents=2*geometry["shape_scale"][i]
                    trim_extents[[0,2]]+=.002
                    trim_extents[1]-=.0002
                    trim_mat=pyrender.MetallicRoughnessMaterial(
                        baseColorFactor=(*trim_color,1),metallicFactor=0,roughnessFactor=.7)
                    trim_tm=trimesh.creation.box(extents=trim_extents)
                    sc.add(pyrender.Mesh.from_trimesh(trim_tm,material=trim_mat),pose=world_pose)
                    material=pyrender.MetallicRoughnessMaterial(
                        baseColorFactor=(.25,.25,.25,1),metallicFactor=0,roughnessFactor=1)
                else:
                    material=pyrender.MetallicRoughnessMaterial(
                        baseColorFactor=(.02,.02,.02,1),emissiveFactor=color*.72,
                        metallicFactor=0,roughnessFactor=1)
                tm=trimesh.creation.box(extents=2*geometry["shape_scale"][i])
                node=sc.add(pyrender.Mesh.from_trimesh(tm,material=material),pose=world_pose)
                gripper_nodes.append(node)
            else:
                raise RuntimeError(f"unsupported exported Panda shape kind {geometry['shape_kind'][i]}")
    else:
        # Exact physical pad boxes remain opaque blue simulated colliders.
        box(2*PAD_HALF,(.18,.31,.48),transform(bodies[2]))
        box(2*PAD_HALF,(.18,.31,.48),transform(bodies[3]))
    if version == 4:
        # Render-only Panda-hand-style shell: white housing behind the pads and
        # two slender dark fingers ending at the exact blue collider poses.
        box((.040,.145,.025),(.92,.93,.94),transform(bodies[1]),alpha=.40)
        capsule(.025,.040,(.95,.96,.97),transform(bodies[1]),alpha=.35)
        for i in (2,3):
            finger_pose=transform(bodies[i]) @ np.array(
                ((1,0,0,-.018),(0,1,0,0),(0,0,1,.038),(0,0,0,1)))
            box((.016,.011,.076),(.10,.12,.15),finger_pose,alpha=.45)
    elif version < 4:
        # Legacy render-only dressing, rigidly driven by recorded poses.
        box((.034,.14,.018),(.20,.23,.28),transform(bodies[1]),alpha=.35)
        for i in (2,3): box((.030,.014,.070),(.25,.28,.33),transform(bodies[i]) @ np.array(((1,0,0,0),(0,1,0,0),(0,0,1,.035),(0,0,0,1))),alpha=.35)
    # Ground, shadow/contact hint, and 5 cm grid.
    mid=sum(xlim)/2; ground=trimesh.creation.box((xlim[1]-xlim[0],.42,.002)); ground.apply_translation((mid,0,-.002))
    sc.add(pyrender.Mesh.from_trimesh(ground,material=pyrender.MetallicRoughnessMaterial(baseColorFactor=(.86,.88,.90,1),roughnessFactor=1)))
    for x in np.arange(np.floor(xlim[0]/.05)*.05,xlim[1]+.05,.05): box((.0007,.40,.0008),(.62,.66,.70),np.array(((1,0,0,x),(0,1,0,0),(0,0,1,.0003),(0,0,0,1))))
    shadow=trimesh.creation.cylinder(radius=.045,height=.0006); shadow.apply_scale((1.8,.65,1)); shadow.apply_translation((q[:,0].mean(),q[:,1].mean(),.0005))
    sc.add(pyrender.Mesh.from_trimesh(shadow,material=pyrender.MetallicRoughnessMaterial(baseColorFactor=(.12,.14,.16,.18),alphaMode="BLEND")))
    sc.add(pyrender.PerspectiveCamera(yfov=np.deg2rad(34)),pose=camera)
    light=pyrender.DirectionalLight(color=np.ones(3),intensity=4.5); sc.add(light,pose=look_at(camera[:3,3],np.array((mid,0,.04))))
    r=pyrender.OffscreenRenderer(WIDTH,HEIGHT); rgb,_=r.render(sc,flags=pyrender.RenderFlags.RGBA)
    masks=None
    if return_masks:
        shell_rgb,_=r.render(sc,flags=pyrender.RenderFlags.SEG,
                             seg_node_map={node:(255,255,255) for node in shell_nodes})
        grip_rgb,_=r.render(sc,flags=pyrender.RenderFlags.SEG,
                            seg_node_map={node:(255,255,255) for node in gripper_nodes})
        masks=(shell_rgb[:,:,0]>0,grip_rgb[:,:,0]>0)
    r.delete()
    composed=compose_overlays(rgb[:,:,:3], q, bodies, boundary, scene, t, meta, version)
    return (composed,*masks) if return_masks else composed


def mpl_frame(q,bodies,t,scene,meta,boundary,tets,inv_dm,camera,xlim,version=2):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    fig=plt.figure(figsize=(12.8,7.2),dpi=100,facecolor="#f3f5f7"); ax=fig.add_subplot(projection="3d",facecolor="#f3f5f7")
    tri=q[boundary]; depth=tri.mean(1)@np.array((.2,-1,.3)); order=np.argsort(depth)
    if scene=="damage":
        s=vertex_strain(q,tets,inv_dm); c=damage_colors(s[boundary].mean(1))
    else: c=np.tile((.91,.66,.25,1),(len(tri),1))
    ax.add_collection3d(Poly3DCollection(tri[order],facecolors=c[order],edgecolors="none"))
    display_pads=[offset_pose(bodies[i], (0,0,PAD_MOUNT_Z_OFFSET))
                  if version >= 5 else bodies[i] for i in (2,3)]
    boxes=[(display_pads[0],PAD_HALF,"#2e507a",1.0),
           (display_pads[1],PAD_HALF,"#2e507a",1.0)]
    if version >= 5:
        raise RuntimeError("v5 requires EGL/pyrender to draw the authored Franka meshes")
    elif version >= 4:
        boxes.append((bodies[1],np.array((.020,.0725,.0125)),"#f0f2f4",.40))
        boxes.extend((bodies[i],np.array((.008,.0055,.038)),"#1a1f26",.45)
                     for i in (2,3))
    else:
        boxes.append((bodies[1],np.array((.017,.07,.009)),"#353b45",.35))
    for pose,half,col,alpha in boxes:
        signs=np.array([(a,b,c) for a in (-1,1) for b in (-1,1) for c in (-1,1)]); corners=signs*half@rotation(pose[3:]).T+pose[:3]
        ax.add_collection3d(Poly3DCollection(corners[BOX_FACES],facecolors=col,
                                             edgecolors="#222",alpha=alpha))
    for x in np.arange(np.floor(xlim[0]/.05)*.05,xlim[1]+.05,.05): ax.plot([x,x],[-.2,.2],[0,0],color="#aeb4ba",lw=.6)
    ax.set(xlim=xlim,ylim=(-.20,.20),zlim=(0,.18)); ax.view_init(22,-70); ax.set_box_aspect((xlim[1]-xlim[0],.4,.18)); ax.set_axis_off()
    fig.canvas.draw(); rgb=np.asarray(fig.canvas.buffer_rgba())[:,:,:3].copy(); plt.close(fig)
    return compose_overlays(rgb, q, bodies, boundary, scene, t, meta, version)


CARD_TEXT = {
    "intact": ("W3 - INTACT",
               "Same grip (1.2 N), slow transport (realized 0.7 m/s2) - safe",
               "OUTCOME: SAFE"),
    "slip": ("W3 - SLIP",
             "Same grip (1.2 N), fast transport (realized 19.8 m/s2) - ejected 0.1 s after motion starts",
             "OUTCOME: EJECTED"),
    "damage": ("W3 - DAMAGE",
               "Excessive grip (2.0 N) - material damage",
               "OUTCOME: DAMAGED"),
}


def message_card(text, subtitle=None):
    im=Image.new("RGB",(WIDTH,HEIGHT),(239,242,245)); d=ImageDraw.Draw(im)
    try:
        title=ImageFont.truetype("DejaVuSans-Bold.ttf",54 if len(text)<25 else 40)
        sub=ImageFont.truetype("DejaVuSans.ttf",25)
    except OSError: title=sub=ImageFont.load_default()
    lines=textwrap.wrap(text,width=55)
    boxes=[d.textbbox((0,0),line,font=title) for line in lines]
    line_height=max(box[3]-box[1] for box in boxes)+12
    y=330-line_height*len(lines)/2
    for line,box in zip(lines,boxes):
        d.text(((WIDTH-(box[2]-box[0]))/2,y),line,font=title,fill=(25,34,45))
        y+=line_height
    if subtitle:
        subtitle_lines=textwrap.wrap(subtitle,width=76)
        y=405
        for line in subtitle_lines:
            box=d.textbbox((0,0),line,font=sub)
            d.text(((WIDTH-(box[2]-box[0]))/2,y),line,font=sub,fill=(65,75,86))
            y+=36
    return np.asarray(im)


def render_scene(scene, smoke=False, version=2):
    global BODY_REORDER
    manifest_path=(CLIPS/"panda"/"w3_force40_manifest.json" if version >= 8 else
                   (CLIPS/"panda"/"w3_panda_manifest.json"
                    if version >= 5 else CLIPS/"w3_manifest.json"))
    manifest=json.loads(manifest_path.read_text())
    scenes=manifest["scenes"]
    meta=scenes[scene] if isinstance(scenes,dict) else next(x for x in scenes if x["scene"]==scene)
    if version >= 8:
        base_manifest=json.loads((CLIPS/"panda"/"w3_panda_manifest.json").read_text())
        base_meta=next(x for x in base_manifest["scenes"] if x["scene"]==scene)
        meta={**base_meta,**meta}
        manifest={**base_manifest,**manifest}
    expected=meta.get("expected_label",meta.get("expected",
                      meta.get("source_final_band_label",meta.get("source_label"))))
    if not meta.get("label_reproduced") or meta["rerun_label"] != expected:
        raise RuntimeError("frozen label audit failed")
    if version >= 5:
        body_rows=meta.get("body_rows") or manifest.get("body_rows")
        raw_map=(meta.get("body_index_to_label") or meta.get("body_index_label_map")
                 or manifest.get("body_index_to_label") or manifest.get("body_index_label_map"))
        if raw_map is None:
            raw_map=["carriage","fr3_hand","fr3_leftfinger","fr3_rightfinger"]
        index_labels=({i:str(label) for i,label in enumerate(raw_map)}
                      if isinstance(raw_map,list)
                      else {int(index):str(label) for index,label in raw_map.items()})
        def body_index(label):
            matches=[i for i,value in index_labels.items()
                     if value==label or value.endswith("/"+label)]
            if len(matches)!=1:
                raise RuntimeError(f"Panda body map has {len(matches)} matches for {label}")
            return matches[0]
        BODY_REORDER=(np.array([int(body_rows[label]) for label in
                               ("carriage","fr3_hand","fr3_leftfinger","fr3_rightfinger")])
                      if body_rows else
                      np.array([body_index(label) for label in
                                ("carriage","fr3_hand","fr3_leftfinger","fr3_rightfinger")]))
        meta.setdefault("realized_accel",{"intact":.7,"slip":19.8,"damage":3.2}[scene])
    else:
        BODY_REORDER=None
    files=snapshots(scene,version); first,_,_=load_frame(files[0])
    settle_index=None
    if version >= 5 and scene == "slip":
        if load_frame(files[-1])[2] <= float(meta["drop_t"]):
            raise RuntimeError("Panda slip capture does not extend past ejection")
        settle_index=slip_settle_index(
            files,float(meta["drop_t"]),radius=.02 if version >= 8 else .01)
    elif version >= 3 and scene == "slip":
        capture_dir="w3_slip_dense_v4" if version >= 4 else "w3_slip_dense_ext"
        capture_meta_path=CLIPS/capture_dir/"capture_meta.json"
        if not capture_meta_path.exists():
            raise RuntimeError("extended slip capture is incomplete: capture_meta.json missing")
        capture_meta=json.loads(capture_meta_path.read_text())
        actual_last=load_frame(files[-1])[2]
        drop_t=float(capture_meta["drop_t"])
        stated_last=float(capture_meta.get("t_last",capture_meta.get("global_t_last")))
        if (not capture_meta.get("ejected") or capture_meta.get("label") != "slip"
                or capture_meta.get("source_label") != "slip"
                or actual_last <= drop_t + 1.0 or abs(actual_last-stated_last) > 1e-6):
            raise RuntimeError("extended slip capture lacks validated post-ejection frames")
        if version >= 4:
            slow_meta_path=CLIPS/"w3_slip_slowmo_v4"/"capture_meta.json"
            if not slow_meta_path.exists():
                raise RuntimeError("v4 slow-motion capture is incomplete: capture_meta.json missing")
            slow_meta=json.loads(slow_meta_path.read_text())
            slow_files=sorted((slow_meta_path.parent).glob("f_*.npz"))
            if (not slow_files or not slow_meta.get("ejected")
                    or slow_meta.get("label") != "slip"
                    or slow_meta.get("source_label") != "slip"
                    or float(slow_meta["slowmo_t_first"]) > 9.21
                    or float(slow_meta["slowmo_t_last"]) < 9.59):
                raise RuntimeError("v4 slow-motion capture failed provenance/window validation")
    with np.load(CLIPS/"tofu_topology.npz") as d: tets=np.array(d["tet_idx"]); n=int(d["n_particles"])
    if len(first)!=n: raise ValueError("topology/trajectory particle mismatch")
    boundary=boundary_triangles(tets,first)
    if len(boundary)!=768: raise ValueError(f"expected 768 boundary triangles, got {len(boundary)}")
    inv_dm=rest_poses(tets,first)
    egl,status=egl_available()
    camera,xlim,eye,target=camera_for(files,scene,version)
    wide_view=(camera,xlim,eye,target)
    slip_cut_t=None
    if version >= 10 and scene == "slip":
        slip_cut_t=float(meta["drop_t"])+.2
        _,grip_bodies,_=min((load_frame(path) for path in files),
                            key=lambda frame: abs(frame[2]-1.8))
        grip_mid=float(grip_bodies[1,0])
        target=np.array((grip_mid,0,.14))
        eye=target+np.array((-.48,-.58,.24))
        xlim=(grip_mid-.18,grip_mid+.18)
    aftermath=None
    if 3 <= version < 5 and scene=="slip":
        (camera,xlim,eye,target),aftermath=slip_v3_cameras(files)
    grip_frame=min((load_frame(path) for path in files),key=lambda frame: abs(frame[2]-1.8))
    grip_y=float((grip_frame[1][2,1]+grip_frame[1][3,1])/2)
    def view_at(t,bodies):
        moving_eye,moving_target,bounds=eye.copy(),target.copy(),xlim
        if version >= 10 and scene == "slip" and t > slip_cut_t:
            _,bounds,moving_eye,moving_target=wide_view
        if aftermath is not None and t > 9.55:
            _,wide_xlim,wide_eye,wide_target=aftermath
            u=np.clip((t-9.55)/.40,0,1); u=u*u*(3-2*u)
            moving_eye=eye+(wide_eye-eye)*u; moving_target=target+(wide_target-target)*u
            bounds=(xlim[0]+(wide_xlim[0]-xlim[0])*u,xlim[1]+(wide_xlim[1]-xlim[1])*u)
        if version == 4:
            drift=float((bodies[2,1]+bodies[3,1])/2)-grip_y
            moving_eye[1]+=drift; moving_target[1]+=drift
        return look_at(moving_eye,moving_target),bounds
    renderer=pyrender_frame if egl else mpl_frame
    if smoke == "gate":
        if not egl:
            raise RuntimeError("v10 still gate requires EGL/pyrender segmentation")
        ending_t=(min(load_frame(files[settle_index])[2],float(meta["drop_t"])+1.0)
                  if scene=="slip" else 10.60)
        check_times={"grasp":1.80,"mid_motion":9.40,"ending":ending_t}
        results={}
        out_dir=CLIPS/"panda"
        for name,requested_t in check_times.items():
            path=min(files,key=lambda p:abs(load_frame(p)[2]-requested_t))
            q,b,t=load_frame(path); frame_camera,frame_xlim=view_at(t,b)
            frame,shell_mask,grip_mask=pyrender_frame(
                q,b,t,scene,meta,boundary,tets,inv_dm,frame_camera,frame_xlim,version,
                return_masks=True)
            still_path=out_dir/f"w3_{scene}_v{version}_{name}.png"
            Image.fromarray(frame).save(still_path)
            usable=shell_mask.copy()
            usable[:215,:660]=False
            usable[HEIGHT-205:,:]=False
            luminance=(.2126*frame[:,:,0]+.7152*frame[:,:,1]+.0722*frame[:,:,2])
            shell_mean=float(luminance[usable].mean()) if usable.any() else 0.0
            ys,xs=np.nonzero(grip_mask)
            fully=bool(len(xs) and xs.min()>2 and xs.max()<WIDTH-3 and ys.min()>2 and ys.max()<HEIGHT-3)
            panel_y=HEIGHT-18-180
            panel_presence=[
                float((frame[panel_y:panel_y+180,x: x+180].mean(axis=2)>180).mean())>.25
                for x in (WIDTH-18-2*180-12,WIDTH-18-180)]
            mirrored=bool(all(panel_presence))
            results[name]={"time_s":t,"shell_luminance_mean":shell_mean,
                           "shell_pixel_count":int(usable.sum()),
                           "gripper_fully_in_frame":fully,
                           "insets_present_mirrored":mirrored,
                           "still":str(still_path.relative_to(ROOT))}
        return results
    if smoke:
        target_time = 9.8 if scene == "damage" else 9.3
        smoke_files=files
        if version == 4 and scene=="slip":
            smoke_files=sorted((CLIPS/"w3_slip_slowmo_v4").glob("f_*.npz"))
        index=min(range(len(smoke_files)),key=lambda i: abs(load_frame(smoke_files[i])[2]-target_time))
        mode=f"v{version}" if version >= 3 else "pro"
        out=CLIPS/f"w3_{scene}_{mode}_smoke.png"
        q,b,t=load_frame(smoke_files[index]); frame_camera,frame_xlim=view_at(t,b)
        composed=renderer(q,b,t,scene,meta,boundary,tets,inv_dm,frame_camera,frame_xlim,version)
        Image.fromarray(composed).save(out)
        if version >= 3:
            Image.fromarray(message_card(CARD_TEXT[scene][0],CARD_TEXT[scene][1])).save(CLIPS/f"w3_{scene}_{mode}_intro_smoke.png")
            Image.fromarray(message_card(CARD_TEXT[scene][2])).save(CLIPS/f"w3_{scene}_{mode}_end_smoke.png")
            if scene=="slip":
                aftermath_path=files[settle_index] if version >= 5 else files[-1]
                q2,b2,t2=load_frame(aftermath_path); cam2,limits2=view_at(t2,b2)
                aftermath_frame=renderer(q2,b2,t2,scene,meta,boundary,tets,inv_dm,cam2,limits2,version)
                Image.fromarray(aftermath_frame).save(CLIPS/f"w3_slip_{mode}_aftermath_smoke.png")
        # Exercise the same key-image write path: key PNG is the fully composed
        # video frame, including HUD and tactile insets.
        key_dir=CLIPS/f"w3_{scene}_{mode}_keys"; key_dir.mkdir(exist_ok=True)
        Image.fromarray(composed).save(key_dir/("dwell.png" if scene == "damage" else "hold.png"))
        return out,status,xlim,eye,target
    render_paths=[]
    if version >= 10:
        last_index=settle_index if scene=="slip" else len(files)-1
        for i,p in enumerate(files[:last_index+1]):
            _,_,t=load_frame(p)
            stride=16 if 2.2 <= t < 8.7 else 2
            if i%stride==0:
                render_paths.extend([p]*(4 if scene=="slip" and 9.20<=t<=float(meta["drop_t"]) else 1))
    elif version >= 5 and scene=="slip":
        for i,p in enumerate(files[:settle_index+1]):
            _,_,t=load_frame(p)
            if i%2==0:
                render_paths.extend([p]*(4 if 9.20<=t<=9.60 else 1))
    elif version >= 4 and scene=="slip":
        slow_files=sorted((CLIPS/"w3_slip_slowmo_v4").glob("f_*.npz"))
        if not slow_files: raise RuntimeError("missing v4 slow-motion substep states")
        stride=max(1,int(round(len(slow_files)/48)))
        render_paths += [p for i,p in enumerate(files) if i%2==0 and load_frame(p)[2]<9.20]
        render_paths += slow_files[::stride]
        render_paths += [p for i,p in enumerate(files) if i%2==0 and load_frame(p)[2]>9.60]
    else:
        for i,p in enumerate(files):
            _,_,t=load_frame(p)
            if i%2==0:
                slow=scene=="slip" and version >= 3 and 9.25<=t<=9.55
                render_paths.extend([p]*(4 if slow else 1))
    if version >= 3 and scene=="slip":
        freeze_path=files[settle_index] if version >= 5 else files[-1]
        render_paths.extend([freeze_path]*30)
    import imageio.v2 as imageio
    mode=f"v{version}" if version >= 3 else "pro"
    stem=f"w3_{scene}_{mode}"
    output_dir=CLIPS/"panda" if version >= 10 else CLIPS
    out=output_dir/f"{stem}.mp4"; keys=output_dir/f"{stem}_keys"; keys.mkdir(exist_ok=True)
    writer=imageio.get_writer(out,fps=FPS,codec="libx264",quality=8,macro_block_size=None)
    times=[]
    try:
        if version >= 3:
            intro=message_card(CARD_TEXT[scene][0],CARD_TEXT[scene][1])
            for _ in range(45): writer.append_data(intro)
        for path in render_paths:
            q,b,t=load_frame(path); frame_camera,frame_xlim=view_at(t,b)
            frame=renderer(q,b,t,scene,meta,boundary,tets,inv_dm,frame_camera,frame_xlim,version); writer.append_data(frame); times.append(t)
            for name,kt in KEY_TIMES.items():
                kp=keys/f"{name}.png"
                if abs(t-kt)<=1/60+.0001: Image.fromarray(frame).save(kp)
        if version >= 3:
            ending=message_card(CARD_TEXT[scene][2])
            for _ in range(30): writer.append_data(ending)
    finally: writer.close()
    # Always rewrite every key from a fully composed nearest real frame. This
    # prevents stale pre-inset keys surviving a later video re-encode.
    for name,kt in KEY_TIMES.items():
        kp=keys/f"{name}.png"
        q,b,t=load_frame(files[int(np.argmin([abs(load_frame(p)[2]-kt) for p in files]))])
        frame_camera,frame_xlim=view_at(t,b)
        Image.fromarray(renderer(q,b,t,scene,meta,boundary,tets,inv_dm,frame_camera,frame_xlim,version)).save(kp)
    return out,status,xlim,eye,target


def main():
    ap=argparse.ArgumentParser(); g=ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--render",action="store_true"); g.add_argument("--scene",choices=SCENES)
    g.add_argument("--smoke",action="store_true",help="one composed frame per scene")
    g.add_argument("--smoke-scene",choices=SCENES,help="one composed frame for one scene")
    ap.add_argument("--v3",action="store_true",help="v3 cards, taxels, and extended slip")
    ap.add_argument("--v4",action="store_true",help="v4 drift tracking, shell, and substep slow motion")
    ap.add_argument("--v5",action="store_true",help="real Panda-hand P-rig meshes and captures")
    ap.add_argument("--v6",action="store_true",help="draw exact visible shapes exported from PandaRig")
    ap.add_argument("--v7",action="store_true",help="v7: vertical hand pose, three-quarter camera, matte pads (ships to user)")
    ap.add_argument("--v8",action="store_true",help="v8: validated iter-40 normal/shear force insets")
    ap.add_argument("--v9",action="store_true",help="v9: cropped Panda appearance and tactile polish")
    ap.add_argument("--v10",action="store_true",help="v10: render-only hand materials, cameras, and timing")
    ap.add_argument("--v11",action="store_true",help="v11: corrected hand-shell axes and intact camera")
    a=ap.parse_args()
    chosen=SCENES if a.render or a.smoke else ((a.smoke_scene,) if a.smoke_scene else (a.scene,))
    smoke=a.smoke or bool(a.smoke_scene)
    failures=[]
    version=11 if a.v11 else (10 if a.v10 else (9 if a.v9 else (8 if a.v8 else (7 if a.v7 else (6 if a.v6 else (5 if a.v5 else (4 if a.v4 else (3 if a.v3 else 2))))))))
    if version >= 10 and not smoke:
        checks={}
        try:
            for scene in chosen:
                checks[scene]=render_scene(scene,"gate",version)
            payload={
                "version":version,
                "render_only":True,
                "simulation_rerun":False,
                "labels_untouched":True,
                "criteria":{"shell_luminance_mean_gt":200,
                            "gripper_fully_in_frame":True,
                            "insets_present_mirrored":True},
                "scenes":checks,
            }
            if version >= 11:
                payload["shell_attachment_3d"]=v11_shell_attachment_checks()
            all_rows=[row for scene_rows in checks.values() for row in scene_rows.values()]
            payload["passed"]=all(
                row["shell_luminance_mean"]>200 and row["gripper_fully_in_frame"]
                and row["insets_present_mirrored"] for row in all_rows)
            (CLIPS/"panda"/f"v{version}_stillcheck.json").write_text(
                json.dumps(payload,indent=2)+"\n",encoding="ascii")
            if not payload["passed"]:
                print("ERROR v10 still-check gate failed; encoding stopped",file=sys.stderr)
                return True
        except Exception as exc:
            print(f"ERROR v10 still-check gate: {exc}",file=sys.stderr)
            return True
    for scene in chosen:
        try:
            result=render_scene(scene,smoke,version); print(f"{scene}: {result}")
        except Exception as exc:
            failures.append(scene); print(f"ERROR {scene}: {exc}",file=sys.stderr)
    return bool(failures)

if __name__=="__main__": raise SystemExit(main())
