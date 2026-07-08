"""evaluate.py - the single evaluation harness: rollouts + closed-loop metrics.

Both scripts/report_stats.py (which prints the README table) and tests/test_design.py
(the CI gates) import from HERE, so the numbers in the docs and the numbers CI asserts
on come from one code path - the same single-source-of-truth rule the repo applies to K
(designed once in lqr_design) and the plant (generated once from robot_params).

Nothing in here renders, so it runs headless on CI with no GL/display.
"""
import numpy as np
import mujoco

from balancer.params import robot_params as rp
from balancer.sim.lqr_design import Q, R, compute_K, linearize
from balancer.sim.plant import plant_xml
from balancer.sim.balance import make_controller, design_gains, encoder_state, pitch_from_quat, CONFIG

DT = 0.002
SHOVE = (1.5, 1.56, 2.2)   # (t_start, t_end, force_x N) - the disturbance the GIF shows


def rollout(K, initial_deg=3.0, pushes=(), cfg=CONFIG, seconds=9.0):
    """Fly K on the contact plant from an initial tilt, optionally shoved.

    Returns (history, peak_torque) where history rows are [t, |pitch_deg|, |x_mm|].
    """
    m = mujoco.MjModel.from_xml_string(plant_xml())
    d = mujoco.MjData(m)
    mujoco.mj_resetData(m, d)
    ip = np.radians(initial_deg)
    d.qpos[3:7] = [np.cos(ip / 2), 0, np.sin(ip / 2), 0]
    cid = m.body("chassis").id
    control = make_controller(cfg, K)

    peak_u, hist = 0.0, []
    for _ in range(int(seconds / DT)):
        u = control(d)
        d.ctrl[0] = d.ctrl[1] = u
        peak_u = max(peak_u, abs(u))
        d.xfrc_applied[cid, :] = 0.0
        for ts, te, fx in pushes:
            if ts <= d.time < te:
                d.xfrc_applied[cid, 0] = fx
        mujoco.mj_step(m, d)
        hist.append((d.time,
                     abs(np.degrees(pitch_from_quat(d.sensor("imu_quat").data))),
                     abs(d.qpos[0] * 1000)))
    return np.array(hist), peak_u


def settle(hist, col, band):
    """Last time |signal| exits the band (0.0 if it never does). col: 1=pitch, 2=x."""
    out = hist[hist[:, col] > band]
    return float(out[-1, 0]) if len(out) else 0.0


def poles():
    """Open- and closed-loop discrete pole magnitudes (descending) for the real robot."""
    p = rp.assemble()
    A, B = linearize(p["L"], p["pole_mass"], p["cart_mass"])
    K, _ = compute_K(p["L"], p["pole_mass"], p["cart_mass"], Q, R)
    ol = np.sort(np.abs(np.linalg.eigvals(A)))[::-1]
    cl = np.sort(np.abs(np.linalg.eigvals(A - B @ K)))[::-1]
    return ol, cl


def encoder_tracking_error(seconds=6.0):
    """Max |encoder-derived x - true x| (mm) while balancing - the sim-to-real gap."""
    m = mujoco.MjModel.from_xml_string(plant_xml())
    d = mujoco.MjData(m)
    mujoco.mj_resetData(m, d)
    ip = np.radians(3.0)
    d.qpos[3:7] = [np.cos(ip / 2), 0, np.sin(ip / 2), 0]
    control = make_controller(CONFIG, design_gains(phase=2))
    err = 0.0
    for _ in range(int(seconds / DT)):
        u = control(d)
        d.ctrl[0] = d.ctrl[1] = u
        mujoco.mj_step(m, d)
        x_enc, _ = encoder_state(d)
        err = max(err, abs(x_enc - d.qpos[0]) * 1000)
    return err


def summary(cfg=CONFIG):
    """Every headline metric as a dict - the one place they are computed."""
    p = rp.assemble()
    K = design_gains(phase=2)
    ol, cl = poles()
    tilt, u_tilt = rollout(K, 3.0, (), cfg)             # initial tilt, no push
    shove, u_shove = rollout(K, 3.0, (SHOVE,), cfg)     # settle, then a shove
    k1 = design_gains(phase=1)
    ph1, _ = rollout(k1, 3.0, (), cfg)                  # Phase-1 IMU-only

    return dict(
        ol_max=ol[0], cl_max=cl[0], cl=cl,
        pitch_recover_s=settle(tilt, 1, 1.0),
        recenter_s=settle(tilt, 2, 10),
        shove_peak_tilt=float(shove[:, 1].max()),
        shove_lunge_mm=float(shove[:, 2].max()),
        peak_torque=max(u_tilt, u_shove), stall=p["stall"],
        enc_err_mm=encoder_tracking_error(),
        phase1_peak_tilt=float(ph1[:, 1].max()),
        phase1_drift_mm=float(ph1[-1, 2]),
    )
