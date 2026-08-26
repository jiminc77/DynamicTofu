"""Analytic, rebuild-free transport profile family."""
import numpy as np

PROFILE_IDS = ("trapz_reversal_default", "single_leg_no_reversal", "sharp_reversal", "const_speed_arc")


def _leg(distance, a_peak, dt, direction=1.0):
    # Quintic smoothstep: q=10u^3-15u^4+6u^5. Determine duration from its exact acceleration peak.
    u0 = (3.0-np.sqrt(3.0))/6.0
    q2max = abs(60*u0 - 180*u0**2 + 120*u0**3)
    duration = np.sqrt(abs(distance) * q2max / a_peak)
    n = max(101, int(np.ceil(duration/dt))+1)
    u = np.linspace(0, 1, n); t = u*duration
    q = 10*u**3-15*u**4+6*u**5
    q1 = 30*u**2-60*u**3+30*u**4
    q2 = 60*u-180*u**2+120*u**3
    q3 = 60-360*u+360*u**2
    d = direction*abs(distance)
    return t, d*q, d*q1/duration, d*q2/duration**2, abs(d)*np.max(abs(q3))/duration**3


def generate(profile_id, a_peak, dt=0.002):
    if profile_id not in PROFILE_IDS: raise ValueError(f"unknown profile_id: {profile_id}")
    if a_peak <= 0 or dt <= 0: raise ValueError("a_peak and dt must be positive")
    if profile_id == "const_speed_arc":
        radius = 0.3
        v = np.sqrt(a_peak*radius)
        duration = np.pi*radius/v
        n = max(101, int(np.ceil(duration/dt))+1)
        t = np.linspace(0, duration, n); theta = np.pi*t/duration
        pos = np.column_stack((radius*np.sin(theta), radius*(1-np.cos(theta))))
        vel = np.column_stack((v*np.cos(theta), v*np.sin(theta)))
        acc = np.column_stack((-a_peak*np.sin(theta), a_peak*np.cos(theta)))
        j_max = a_peak*v/radius
        return _result(profile_id, a_peak, t, pos, vel, acc, j_max,
                       {"leg_start": 0., "leg_end": duration}, radius=radius, speed=v)
    dwell = 0.3 if profile_id == "trapz_reversal_default" else 0.0
    t1,p1,v1,a1,j1 = _leg(.3, a_peak, dt, 1)
    if profile_id == "single_leg_no_reversal":
        return _result(profile_id,a_peak,t1,p1[:,None],v1[:,None],a1[:,None],j1,
                       {"leg_out_start":0.,"leg_out_end":float(t1[-1])})
    nd = int(np.ceil(dwell/dt))
    td = np.linspace(t1[-1], t1[-1]+dwell, nd+1)[1:] if nd else np.empty(0)
    t2,p2,v2,a2,j2 = _leg(.3, a_peak, dt, -1)
    offset = t1[-1]+dwell
    t = np.r_[t1, td, offset+t2[1:]]
    p = np.r_[p1, np.full(td.size,.3), .3+p2[1:]]
    vel = np.r_[v1, np.zeros(td.size), v2[1:]]
    acc = np.r_[a1, np.zeros(td.size), a2[1:]]
    phases={"leg_out_start":0.,"leg_out_end":float(t1[-1]),"reversal_time":float(offset),
            "dwell_start":float(t1[-1]),"dwell_end":float(offset),"leg_back_end":float(t[-1])}
    result = _result(profile_id,a_peak,t,p[:,None],vel[:,None],acc[:,None],max(j1,j2),phases)
    result["dwell_s"] = dwell; result["decel_gain"] = 2.0 if profile_id == "sharp_reversal" else 1.0
    return result


def _result(pid, requested, t, pos, vel, acc, jmax, phases, **extra):
    realized=float(np.max(np.linalg.norm(acc,axis=1)))
    return {"profile_id":pid,"t":t,"pos":pos,"vel":vel,"acc":acc,
            "commanded_peak_accel":float(requested),"realized_peak_accel":realized,
            "j_max":float(jmax),"endpoint_positions":pos[[0,-1]].copy(),
            "phase_timestamps":phases, **extra}


generate_profile = generate
