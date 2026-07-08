"""Print the closed-loop performance numbers used in the README Results table.

Everything here is derived live from robot_params -> lqr_design -> plant, so the
table can never drift from the actual design. Uses only project deps:

    uv run python scripts/report_stats.py
"""
import numpy as np
import mujoco
from scipy.linalg import solve_discrete_are

from balancer.params import robot_params as rp
from balancer.sim.lqr_design import Q, R, design_xml
from balancer.sim.plant import plant_xml
from balancer.sim.balance import design_gains, make_controller, CONFIG, pitch_from_quat

DT = 0.002
p = rp.assemble()
L, pole_mass, cart_mass, stall = p["L"], p["pole_mass"], p["cart_mass"], p["stall"]

# ---- open- vs closed-loop poles (discrete cart-pole the gains are designed on) ----
model = mujoco.MjModel.from_xml_string(design_xml(L, pole_mass, cart_mass))
data = mujoco.MjData(model)
mujoco.mj_resetData(model, data); mujoco.mj_forward(model, data)
A = np.zeros((2 * model.nv,) * 2); B = np.zeros((2 * model.nv, model.nu))
mujoco.mjd_transitionFD(model, data, 1e-6, True, A, B, None, None)
K = np.linalg.inv(B.T @ (P := solve_discrete_are(A, B, Q, R)) @ B + R) @ (B.T @ P @ A)
ol = np.sort(np.abs(np.linalg.eigvals(A)))[::-1]
cl = np.sort(np.abs(np.linalg.eigvals(A - B @ K)))[::-1]


def run(initial_deg, pushes):
    m = mujoco.MjModel.from_xml_string(plant_xml())
    d = mujoco.MjData(m); mujoco.mj_resetData(m, d)
    ip = np.radians(initial_deg)
    d.qpos[3:7] = [np.cos(ip / 2), 0, np.sin(ip / 2), 0]
    cid = m.body("chassis").id
    control = make_controller(CONFIG, design_gains())
    peak_u, hist = 0.0, []
    for _ in range(int(9.0 / DT)):
        u = control(d); d.ctrl[0] = d.ctrl[1] = u
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


def settle(arr, col, band):
    out = arr[arr[:, col] > band]
    return out[-1, 0] if len(out) else 0.0


a, ua = run(3.0, [])                       # 3 deg initial tilt, no push
b, ub = run(3.0, [(1.5, 1.56, 2.2)])       # settle, then a shove (matches the GIF)

print(f"open-loop  poles |z| = {np.round(ol, 4)}   (unstable: {ol[0]:.3f} > 1, gravity runaway)")
print(f"closed-loop poles |z| = {np.round(cl, 4)}   (all < 1 -> stable)")
print(f"pitch recovery (3 deg tilt): back within 1 deg in {settle(a, 1, 1.0):.2f} s")
print(f"position re-centering: within +/-10 mm in {settle(a, 2, 10):.2f} s")
print(f"disturbance (2.2 N shove): peak tilt {b[:,1].max():.1f} deg, lunge {b[:,2].max():.0f} mm, recovers")
print(f"peak control effort: {max(ua, ub):.3f} N*m of {stall:.3f} stall "
      f"({100*max(ua,ub)/stall:.0f} %, ~{100-100*max(ua,ub)/stall:.0f} % headroom)")
