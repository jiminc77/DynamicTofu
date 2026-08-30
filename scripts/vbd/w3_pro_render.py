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
SOFT_CONTACT_MARGIN = 1e-3  # frozen src/frozen_config.py value
# The stored surface sits about 1.85 mm proud of the collider face even under
# firm grip. 3 mm is the smallest millimetre band yielding a stable footprint.
CONTACT_PROXIMITY_BAND = 3e-3
TAXEL_COLOR_CEILING = 0.6e-3
BOX_FACES = np.array(((0,1,3,2),(4,6,7,5),(0,4,5,1),(2,3,7,6),(0,2,6,4),(1,5,7,3)))
WIDTH, HEIGHT, FPS = 1280, 720, 30


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


def look_at(eye, target):
    # OpenGL camera looks down local -Z.
    z=(eye-target); z/=np.linalg.norm(z); x=np.cross(np.array((0.,0.,1.)),z); x/=np.linalg.norm(x)
    y=np.cross(z,x); m=np.eye(4); m[:3,:3]=np.column_stack((x,y,z)); m[:3,3]=eye; return m


def snapshots(scene, v3=False):
    suffix = "slip_dense_ext" if v3 and scene == "slip" else f"{scene}_dense"
    files=sorted((CLIPS/f"w3_{suffix}").glob("f_*.npz"))
    if not files: raise FileNotFoundError(f"missing frozen dense trajectory for {scene}")
    return files


def load_frame(path):
    with np.load(path) as d: return np.array(d["particle_q"]),np.array(d["body_q"]),float(d["t"])


def camera_for(files, scene):
    xs=[]
    for p in files:
        with np.load(p) as d: xs.append(float(d["body_q"][1,0]))
    # Damage has a longer actual track than the other scenes, so the old fixed
    # 10 cm padding made it tiny. Preserve its whole recorded excursion but use
    # only presentation clearance around that used segment.
    margin = .025 if scene == "damage" else .10
    lo,hi=min(xs)-margin,max(xs)+margin; mid=(lo+hi)/2; span=hi-lo
    target=np.array((mid,0,.050))
    distance = max(.34, span * (.90 if scene == "damage" else 1.35))
    eye=target+np.array((.12,-distance,.20))
    return look_at(eye,target), (lo,hi), eye, target


def slip_v3_cameras(files):
    """Tight event camera plus a wide aftermath camera for the ejected tofu."""
    primary=camera_for(snapshots("slip",False),"slip")
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


def add_hud(rgb, scene, t, meta, slow):
    im=Image.fromarray(rgb); d=ImageDraw.Draw(im,"RGBA")
    try: font=ImageFont.truetype("DejaVuSans.ttf",24); bold=ImageFont.truetype("DejaVuSans-Bold.ttf",28)
    except OSError: font=bold=ImageFont.load_default()
    d.rounded_rectangle((25,22,650,153),12,fill=(255,255,255,218),outline=(35,45,55,90),width=2)
    d.text((45,34),f"W3 - {scene.upper()}     t={t:.2f} s",font=bold,fill=(24,31,40,255))
    d.text((45,78),f"commanded a={meta['a']:g} m/s2 / realized a={meta['realized_accel']:g} m/s2",font=font,fill=(30,38,48,255))
    d.text((45,113),f"F_g={meta['F']:g} N",font=font,fill=(30,38,48,255))
    if slow:
        d.rounded_rectangle((1000,28,1248,78),10,fill=(185,38,45,235))
        d.text((1018,39),"SLOW MOTION x4",font=font,fill="white")
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


def pad_taxel_depth(vertices, pad_pose, inner_face_sign,
                    band=CONTACT_PROXIMITY_BAND):
    """Return an 8x8 max proximity-depth grid and selected vertex count."""
    local = (np.asarray(vertices) - pad_pose[:3]) @ rotation(pad_pose[3:])
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


def add_tactile_insets(rgb, q, bodies, boundary, taxels=False):
    """Post-composite two geometry-proxy tactile panels onto RGB."""
    im = Image.fromarray(rgb); draw = ImageDraw.Draw(im, "RGBA")
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", 13)
        small = ImageFont.truetype("DejaVuSans.ttf", 11)
    except OSError:
        font = small = ImageFont.load_default()
    surface_vertices = q[np.unique(boundary)]
    tofu_center = surface_vertices.mean(axis=0)
    panel_size, edge_margin, gap = 180, 18, 12
    right_x = WIDTH - edge_margin - panel_size
    left_x = right_x - gap - panel_size
    for label, body_index, x0 in (("L", 2, left_x), ("R", 3, right_x)):
        pose = bodies[body_index]
        tofu_local = (tofu_center - pose[:3]) @ rotation(pose[3:])
        inner_face_sign = 1.0 if tofu_local[1] >= 0 else -1.0
        points, centroid = pad_contact_footprint(surface_vertices, pose, inner_face_sign)
        y0, size = HEIGHT - edge_margin - panel_size, panel_size
        draw.rounded_rectangle((x0, y0, x0 + size, y0 + size), 9,
                               fill=(255, 255, 255, 232), outline=(45, 55, 65, 180), width=2)
        title = f"{label} penetration depth" if taxels else f"{label} pad contact footprint"
        draw.text((x0 + 8, y0 + 7), title, font=font, fill=(20, 28, 36, 255))
        caption = "(geometry proxy)" if taxels else "(geometry proxy)"
        draw.text((x0 + 8, y0 + 24), caption, font=small, fill=(75, 82, 90, 255))
        if taxels:
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
        if taxels:
            grid, count = pad_taxel_depth(surface_vertices, pose, inner_face_sign)
            # Compact viridis-like perceptually increasing blue-to-yellow ramp.
            stops = np.array(((.267,.005,.329),(.190,.407,.556),(.208,.719,.473),(.993,.906,.144)))
            def color(value):
                u=np.clip(value/TAXEL_COLOR_CEILING,0,1)*(len(stops)-1)
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


def compose_overlays(rgb, q, bodies, boundary, scene, t, meta, v3=False):
    slow = scene == "slip" and ((9.25 <= t <= 9.55) if v3 else (9.20 <= t <= 9.40))
    rgb = add_hud(rgb, scene, t, meta, slow)
    return add_tactile_insets(rgb, q, bodies, boundary, taxels=v3)


def pyrender_frame(q,bodies,t,scene,meta,boundary,tets,inv_dm,camera,xlim,v3=False):
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
    # Exact physical pad boxes.
    box(2*PAD_HALF,(.18,.31,.48),transform(bodies[2])); box(2*PAD_HALF,(.18,.31,.48),transform(bodies[3]))
    # Render-only dressing: palm housing and brackets, rigidly driven by recorded poses.
    box((.034,.14,.018),(.20,.23,.28),transform(bodies[1]),alpha=.35)
    for i in (2,3): box((.030,.014,.070),(.25,.28,.33),transform(bodies[i]) @ np.array(((1,0,0,0),(0,1,0,0),(0,0,1,.035),(0,0,0,1))),alpha=.35)
    # Ground, shadow/contact hint, and 5 cm grid.
    mid=sum(xlim)/2; ground=trimesh.creation.box((xlim[1]-xlim[0],.42,.002)); ground.apply_translation((mid,0,-.002))
    sc.add(pyrender.Mesh.from_trimesh(ground,material=pyrender.MetallicRoughnessMaterial(baseColorFactor=(.86,.88,.90,1),roughnessFactor=1)))
    for x in np.arange(np.floor(xlim[0]/.05)*.05,xlim[1]+.05,.05): box((.0007,.40,.0008),(.62,.66,.70),np.array(((1,0,0,x),(0,1,0,0),(0,0,1,.0003),(0,0,0,1))))
    shadow=trimesh.creation.cylinder(radius=.045,height=.0006); shadow.apply_scale((1.8,.65,1)); shadow.apply_translation((q[:,0].mean(),q[:,1].mean(),.0005))
    sc.add(pyrender.Mesh.from_trimesh(shadow,material=pyrender.MetallicRoughnessMaterial(baseColorFactor=(.12,.14,.16,.18),alphaMode="BLEND")))
    sc.add(pyrender.PerspectiveCamera(yfov=np.deg2rad(34)),pose=camera)
    light=pyrender.DirectionalLight(color=np.ones(3),intensity=3.0); sc.add(light,pose=look_at(camera[:3,3],np.array((mid,0,.04))))
    r=pyrender.OffscreenRenderer(WIDTH,HEIGHT); rgb,_=r.render(sc,flags=pyrender.RenderFlags.RGBA); r.delete()
    return compose_overlays(rgb[:,:,:3], q, bodies, boundary, scene, t, meta, v3)


def mpl_frame(q,bodies,t,scene,meta,boundary,tets,inv_dm,camera,xlim,v3=False):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    fig=plt.figure(figsize=(12.8,7.2),dpi=100,facecolor="#f3f5f7"); ax=fig.add_subplot(projection="3d",facecolor="#f3f5f7")
    tri=q[boundary]; depth=tri.mean(1)@np.array((.2,-1,.3)); order=np.argsort(depth)
    if scene=="damage":
        s=vertex_strain(q,tets,inv_dm); c=damage_colors(s[boundary].mean(1))
    else: c=np.tile((.91,.66,.25,1),(len(tri),1))
    ax.add_collection3d(Poly3DCollection(tri[order],facecolors=c[order],edgecolors="none"))
    for pose,half,col,alpha in ((bodies[2],PAD_HALF,"#2e507a",1.0),
                                (bodies[3],PAD_HALF,"#2e507a",1.0),
                                (bodies[1],np.array((.017,.07,.009)),"#353b45",.35)):
        signs=np.array([(a,b,c) for a in (-1,1) for b in (-1,1) for c in (-1,1)]); corners=signs*half@rotation(pose[3:]).T+pose[:3]
        ax.add_collection3d(Poly3DCollection(corners[BOX_FACES],facecolors=col,
                                             edgecolors="#222",alpha=alpha))
    for x in np.arange(np.floor(xlim[0]/.05)*.05,xlim[1]+.05,.05): ax.plot([x,x],[-.2,.2],[0,0],color="#aeb4ba",lw=.6)
    ax.set(xlim=xlim,ylim=(-.20,.20),zlim=(0,.18)); ax.view_init(22,-70); ax.set_box_aspect((xlim[1]-xlim[0],.4,.18)); ax.set_axis_off()
    fig.canvas.draw(); rgb=np.asarray(fig.canvas.buffer_rgba())[:,:,:3].copy(); plt.close(fig)
    return compose_overlays(rgb, q, bodies, boundary, scene, t, meta, v3)


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


def render_scene(scene, smoke=False, v3=False):
    manifest=json.loads((CLIPS/"w3_manifest.json").read_text()); meta=next(x for x in manifest["scenes"] if x["scene"]==scene)
    if not meta.get("label_reproduced") or meta["rerun_label"] != meta["source_final_band_label"]: raise RuntimeError("frozen label audit failed")
    files=snapshots(scene,v3); first,_,_=load_frame(files[0])
    if v3 and scene == "slip":
        capture_meta_path=CLIPS/"w3_slip_dense_ext"/"capture_meta.json"
        if not capture_meta_path.exists():
            raise RuntimeError("extended slip capture is incomplete: capture_meta.json missing")
        capture_meta=json.loads(capture_meta_path.read_text())
        actual_last=load_frame(files[-1])[2]
        drop_t=float(capture_meta["drop_t"]); stated_last=float(capture_meta["t_last"])
        if (not capture_meta.get("ejected") or capture_meta.get("label") != "slip"
                or capture_meta.get("source_label") != "slip"
                or actual_last <= drop_t + 1.0 or abs(actual_last-stated_last) > 1e-6):
            raise RuntimeError("extended slip capture lacks validated post-ejection frames")
    with np.load(CLIPS/"tofu_topology.npz") as d: tets=np.array(d["tet_idx"]); n=int(d["n_particles"])
    if len(first)!=n: raise ValueError("topology/trajectory particle mismatch")
    boundary=boundary_triangles(tets,first)
    if len(boundary)!=768: raise ValueError(f"expected 768 boundary triangles, got {len(boundary)}")
    inv_dm=rest_poses(tets,first)
    egl,status=egl_available()
    camera,xlim,eye,target=camera_for(files,scene)
    aftermath=None
    if v3 and scene=="slip":
        (camera,xlim,eye,target),aftermath=slip_v3_cameras(files)
    def view_at(t):
        if aftermath is None or t <= 9.55: return camera,xlim
        _,wide_xlim,wide_eye,wide_target=aftermath
        u=np.clip((t-9.55)/.40,0,1); u=u*u*(3-2*u)
        moving_eye=eye+(wide_eye-eye)*u; moving_target=target+(wide_target-target)*u
        bounds=(xlim[0]+(wide_xlim[0]-xlim[0])*u,xlim[1]+(wide_xlim[1]-xlim[1])*u)
        return look_at(moving_eye,moving_target),bounds
    renderer=pyrender_frame if egl else mpl_frame
    if smoke:
        target = 9.8 if scene == "damage" else 9.3
        index=min(range(len(files)),key=lambda i: abs(load_frame(files[i])[2]-target))
        out=CLIPS/f"w3_{scene}_{'v3' if v3 else 'pro'}_smoke.png"
        q,b,t=load_frame(files[index]); frame_camera,frame_xlim=view_at(t)
        composed=renderer(q,b,t,scene,meta,boundary,tets,inv_dm,frame_camera,frame_xlim,v3)
        Image.fromarray(composed).save(out)
        if v3:
            Image.fromarray(message_card(CARD_TEXT[scene][0],CARD_TEXT[scene][1])).save(CLIPS/f"w3_{scene}_v3_intro_smoke.png")
            Image.fromarray(message_card(CARD_TEXT[scene][2])).save(CLIPS/f"w3_{scene}_v3_end_smoke.png")
            if scene=="slip":
                q2,b2,t2=load_frame(files[-1]); cam2,limits2=view_at(t2)
                aftermath_frame=renderer(q2,b2,t2,scene,meta,boundary,tets,inv_dm,cam2,limits2,v3)
                Image.fromarray(aftermath_frame).save(CLIPS/"w3_slip_v3_aftermath_smoke.png")
        # Exercise the same key-image write path: key PNG is the fully composed
        # video frame, including HUD and tactile insets.
        key_dir=CLIPS/f"w3_{scene}_{'v3' if v3 else 'pro'}_keys"; key_dir.mkdir(exist_ok=True)
        Image.fromarray(composed).save(key_dir/("dwell.png" if scene == "damage" else "hold.png"))
        return out,status,xlim,eye,target
    # Dense 60 Hz -> 30 fps. Slip window repeats every selected real frame 4 times.
    indices=[]
    for i,p in enumerate(files):
        _,_,t=load_frame(p)
        if i%2==0:
            slow = scene=="slip" and ((9.25<=t<=9.55) if v3 else (9.20<=t<=9.40))
            indices.extend([i] * (4 if slow else 1))
    if v3 and scene=="slip": indices.extend([len(files)-1]*30)
    import imageio.v2 as imageio
    stem=f"w3_{scene}_{'v3' if v3 else 'pro'}"
    out=CLIPS/f"{stem}.mp4"; keys=CLIPS/f"{stem}_keys"; keys.mkdir(exist_ok=True)
    writer=imageio.get_writer(out,fps=FPS,codec="libx264",quality=8,macro_block_size=None)
    times=[]
    try:
        if v3:
            intro=message_card(CARD_TEXT[scene][0],CARD_TEXT[scene][1])
            for _ in range(45): writer.append_data(intro)
        for i in indices:
            q,b,t=load_frame(files[i]); frame_camera,frame_xlim=view_at(t)
            frame=renderer(q,b,t,scene,meta,boundary,tets,inv_dm,frame_camera,frame_xlim,v3); writer.append_data(frame); times.append(t)
            for name,kt in KEY_TIMES.items():
                kp=keys/f"{name}.png"
                if abs(t-kt)<=1/60+.0001: Image.fromarray(frame).save(kp)
        if v3:
            ending=message_card(CARD_TEXT[scene][2])
            for _ in range(30): writer.append_data(ending)
    finally: writer.close()
    # Always rewrite every key from a fully composed nearest real frame. This
    # prevents stale pre-inset keys surviving a later video re-encode.
    for name,kt in KEY_TIMES.items():
        kp=keys/f"{name}.png"
        q,b,t=load_frame(files[int(np.argmin([abs(load_frame(p)[2]-kt) for p in files]))])
        frame_camera,frame_xlim=view_at(t)
        Image.fromarray(renderer(q,b,t,scene,meta,boundary,tets,inv_dm,frame_camera,frame_xlim,v3)).save(kp)
    return out,status,xlim,eye,target


def main():
    ap=argparse.ArgumentParser(); g=ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--render",action="store_true"); g.add_argument("--scene",choices=SCENES)
    g.add_argument("--smoke",action="store_true",help="one composed frame per scene")
    g.add_argument("--smoke-scene",choices=SCENES,help="one composed frame for one scene")
    ap.add_argument("--v3",action="store_true",help="v3 cards, taxels, and extended slip")
    a=ap.parse_args()
    chosen=SCENES if a.render or a.smoke else ((a.smoke_scene,) if a.smoke_scene else (a.scene,))
    smoke=a.smoke or bool(a.smoke_scene)
    failures=[]
    for scene in chosen:
        try:
            result=render_scene(scene,smoke,a.v3); print(f"{scene}: {result}")
        except Exception as exc:
            failures.append(scene); print(f"ERROR {scene}: {exc}",file=sys.stderr)
    return bool(failures)

if __name__=="__main__": raise SystemExit(main())
