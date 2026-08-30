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


def snapshots(scene):
    files=sorted((CLIPS/f"w3_{scene}_dense").glob("f_*.npz"))
    if not files: raise FileNotFoundError(f"missing frozen dense trajectory for {scene}")
    return files


def load_frame(path):
    with np.load(path) as d: return np.array(d["particle_q"]),np.array(d["body_q"]),float(d["t"])


def camera_for(files):
    xs=[]
    for p in files:
        with np.load(p) as d: xs.append(float(d["body_q"][1,0]))
    lo,hi=min(xs)-.10,max(xs)+.10; mid=(lo+hi)/2; span=hi-lo
    target=np.array((mid,0,.050)); eye=target+np.array((.12,-max(.34,span*1.35),.20))
    return look_at(eye,target), (lo,hi), eye, target


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
    d.text((45,34),f"W3 — {scene.upper()}     t={t:.2f} s",font=bold,fill=(24,31,40,255))
    d.text((45,78),f"commanded a={meta['a']:g} m/s² / realized a={meta['realized_accel']:g} m/s²",font=font,fill=(30,38,48,255))
    d.text((45,113),f"F_g={meta['F']:g} N",font=font,fill=(30,38,48,255))
    if slow:
        d.rounded_rectangle((1000,28,1248,78),10,fill=(185,38,45,235))
        d.text((1018,39),"SLOW MOTION x4",font=font,fill="white")
    return np.asarray(im)


def pyrender_frame(q,bodies,t,scene,meta,boundary,tets,inv_dm,camera,xlim):
    import pyrender, trimesh
    sc=pyrender.Scene(bg_color=(.95,.96,.97,1),ambient_light=(.42,.42,.42))
    if scene=="damage":
        s=np.clip(vertex_strain(q,tets,inv_dm)/.30,0,1); import matplotlib
        colors=(matplotlib.colormaps["coolwarm"](s)*255).astype(np.uint8)
        tm=trimesh.Trimesh(q,boundary,vertex_colors=colors,process=False)
        mesh=pyrender.Mesh.from_trimesh(tm,smooth=True)
    else:
        tm=trimesh.Trimesh(q,boundary,process=False)
        mat=pyrender.MetallicRoughnessMaterial(baseColorFactor=(.91,.66,.25,1),metallicFactor=.05,roughnessFactor=.38)
        mesh=pyrender.Mesh.from_trimesh(tm,material=mat,smooth=True)
    sc.add(mesh)
    def box(ext,color,pose):
        tm=trimesh.creation.box(extents=ext); mat=pyrender.MetallicRoughnessMaterial(baseColorFactor=(*color,1),metallicFactor=.22,roughnessFactor=.3)
        sc.add(pyrender.Mesh.from_trimesh(tm,material=mat),pose=pose)
    # Exact physical pad boxes.
    box(2*PAD_HALF,(.18,.31,.48),transform(bodies[2])); box(2*PAD_HALF,(.18,.31,.48),transform(bodies[3]))
    # Render-only dressing: palm housing and brackets, rigidly driven by recorded poses.
    box((.034,.14,.018),(.20,.23,.28),transform(bodies[1]))
    for i in (2,3): box((.030,.014,.070),(.25,.28,.33),transform(bodies[i]) @ np.array(((1,0,0,0),(0,1,0,0),(0,0,1,.035),(0,0,0,1))))
    # Ground, shadow/contact hint, and 5 cm grid.
    mid=sum(xlim)/2; ground=trimesh.creation.box((xlim[1]-xlim[0],.42,.002)); ground.apply_translation((mid,0,-.002))
    sc.add(pyrender.Mesh.from_trimesh(ground,material=pyrender.MetallicRoughnessMaterial(baseColorFactor=(.86,.88,.90,1),roughnessFactor=1)))
    for x in np.arange(np.floor(xlim[0]/.05)*.05,xlim[1]+.05,.05): box((.0007,.40,.0008),(.62,.66,.70),np.array(((1,0,0,x),(0,1,0,0),(0,0,1,.0003),(0,0,0,1))))
    shadow=trimesh.creation.cylinder(radius=.045,height=.0006); shadow.apply_scale((1.8,.65,1)); shadow.apply_translation((q[:,0].mean(),q[:,1].mean(),.0005))
    sc.add(pyrender.Mesh.from_trimesh(shadow,material=pyrender.MetallicRoughnessMaterial(baseColorFactor=(.12,.14,.16,.18),alphaMode="BLEND")))
    sc.add(pyrender.PerspectiveCamera(yfov=np.deg2rad(34)),pose=camera)
    light=pyrender.DirectionalLight(color=np.ones(3),intensity=3.0); sc.add(light,pose=look_at(camera[:3,3],np.array((mid,0,.04))))
    r=pyrender.OffscreenRenderer(WIDTH,HEIGHT); rgb,_=r.render(sc,flags=pyrender.RenderFlags.RGBA); r.delete()
    return add_hud(rgb[:,:,:3],scene,t,meta,scene=="slip" and 9.20<=t<=9.40)


def mpl_frame(q,bodies,t,scene,meta,boundary,tets,inv_dm,camera,xlim):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    fig=plt.figure(figsize=(12.8,7.2),dpi=100,facecolor="#f3f5f7"); ax=fig.add_subplot(projection="3d",facecolor="#f3f5f7")
    tri=q[boundary]; depth=tri.mean(1)@np.array((.2,-1,.3)); order=np.argsort(depth)
    if scene=="damage":
        s=vertex_strain(q,tets,inv_dm); c=plt.get_cmap("coolwarm")(np.clip(s[boundary].mean(1)/.3,0,1))
    else: c=np.tile((.91,.66,.25,1),(len(tri),1))
    ax.add_collection3d(Poly3DCollection(tri[order],facecolors=c[order],edgecolors="none"))
    for pose,half,col in ((bodies[2],PAD_HALF,"#2e507a"),(bodies[3],PAD_HALF,"#2e507a"),(bodies[1],np.array((.017,.07,.009)),"#353b45")):
        signs=np.array([(a,b,c) for a in (-1,1) for b in (-1,1) for c in (-1,1)]); corners=signs*half@rotation(pose[3:]).T+pose[:3]
        ax.add_collection3d(Poly3DCollection(corners[BOX_FACES],facecolors=col,edgecolors="#222"))
    for x in np.arange(np.floor(xlim[0]/.05)*.05,xlim[1]+.05,.05): ax.plot([x,x],[-.2,.2],[0,0],color="#aeb4ba",lw=.6)
    ax.set(xlim=xlim,ylim=(-.20,.20),zlim=(0,.18)); ax.view_init(22,-70); ax.set_box_aspect((xlim[1]-xlim[0],.4,.18)); ax.set_axis_off()
    fig.canvas.draw(); rgb=np.asarray(fig.canvas.buffer_rgba())[:,:,:3].copy(); plt.close(fig)
    return add_hud(rgb,scene,t,meta,scene=="slip" and 9.20<=t<=9.40)


def render_scene(scene, smoke=False):
    manifest=json.loads((CLIPS/"w3_manifest.json").read_text()); meta=next(x for x in manifest["scenes"] if x["scene"]==scene)
    if not meta.get("label_reproduced") or meta["rerun_label"] != meta["source_final_band_label"]: raise RuntimeError("frozen label audit failed")
    files=snapshots(scene); first,_,_=load_frame(files[0])
    with np.load(CLIPS/"tofu_topology.npz") as d: tets=np.array(d["tet_idx"]); n=int(d["n_particles"])
    if len(first)!=n: raise ValueError("topology/trajectory particle mismatch")
    boundary=boundary_triangles(tets,first)
    if len(boundary)!=768: raise ValueError(f"expected 768 boundary triangles, got {len(boundary)}")
    inv_dm=rest_poses(tets,first); camera,xlim,eye,target=camera_for(files); egl,status=egl_available()
    renderer=pyrender_frame if egl else mpl_frame
    if smoke:
        indices=[min(len(files)-1,len(files)//2)]; out=CLIPS/f"w3_{scene}_pro_smoke.png"
        q,b,t=load_frame(files[indices[0]]); Image.fromarray(renderer(q,b,t,scene,meta,boundary,tets,inv_dm,camera,xlim)).save(out)
        return out,status,xlim,eye,target
    # Dense 60 Hz -> 30 fps. Slip window repeats every selected real frame 4 times.
    indices=[]
    for i,p in enumerate(files):
        _,_,t=load_frame(p)
        if i%2==0: indices.extend([i]* (4 if scene=="slip" and 9.20<=t<=9.40 else 1))
    import imageio.v2 as imageio
    out=CLIPS/f"w3_{scene}_pro.mp4"; keys=CLIPS/f"w3_{scene}_pro_keys"; keys.mkdir(exist_ok=True)
    writer=imageio.get_writer(out,fps=FPS,codec="libx264",quality=8,macro_block_size=None)
    times=[]
    try:
        for i in indices:
            q,b,t=load_frame(files[i]); frame=renderer(q,b,t,scene,meta,boundary,tets,inv_dm,camera,xlim); writer.append_data(frame); times.append(t)
            for name,kt in KEY_TIMES.items():
                kp=keys/f"{name}.png"
                if not kp.exists() and abs(t-kt)<=1/60+.0001: Image.fromarray(frame).save(kp)
    finally: writer.close()
    # Ejected slip ends before later phases: use its final real frame for unavailable keys.
    for name,kt in KEY_TIMES.items():
        kp=keys/f"{name}.png"
        if not kp.exists():
            q,b,t=load_frame(files[int(np.argmin([abs(load_frame(p)[2]-kt) for p in files]))]); Image.fromarray(renderer(q,b,t,scene,meta,boundary,tets,inv_dm,camera,xlim)).save(kp)
    return out,status,xlim,eye,target


def main():
    ap=argparse.ArgumentParser(); g=ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--render",action="store_true"); g.add_argument("--scene",choices=SCENES); g.add_argument("--smoke",action="store_true",help="one composed frame per scene")
    a=ap.parse_args(); chosen=SCENES if a.render or a.smoke else (a.scene,); failures=[]
    for scene in chosen:
        try:
            result=render_scene(scene,a.smoke); print(f"{scene}: {result}")
        except Exception as exc:
            failures.append(scene); print(f"ERROR {scene}: {exc}",file=sys.stderr)
    return bool(failures)

if __name__=="__main__": raise SystemExit(main())
