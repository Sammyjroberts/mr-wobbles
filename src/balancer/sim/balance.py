"""
TWO-WHEEL SELF-BALANCER  -  LQR controller + simulation
========================================================
Run it:   uv run balancer-sim      (or: uv run python -m balancer.sim.balance)

A window opens; the stick balances and comes to a DEAD STILL stop (not the
wandering you saw before). Set controller_enabled=False to watch it fall.

WHY LQR (and not hand-tuned PD): hand-tuned gains found a "doesn't fall"
solution that was actually a permanent limit cycle - it orbited its target
forever and never settled. LQR instead solves for the gains that drive every
mode to rest with proper damping. We design it on a clean cart-pole stand-in
(linearizing the real robot THROUGH its wheel-ground contact gives a wrong,
gravity-free model), then map cart-force -> wheel-torque (tau = F * wheel_radius).

The controller is full-state feedback:   tau = -K . [x, pitch, x_dot, pitch_rate]
    x, x_dot     : from wheel ENCODERS (position + speed)
    pitch        : from the IMU (orientation)
    pitch_rate   : from the IMU gyro
"""
import time

import mujoco
import numpy as np

from balancer.params import robot_params as rp
from balancer.sim.lqr_design import Q, R, compute_K
from balancer.sim.plant import plant_xml

# ===== LQR gains =====
# Designed at startup from the CURRENT real-robot params, via the same compute_K()
# that lqr_design uses — one producer of K, so the sim can never fly stale gains.
WHEEL_RADIUS = rp.WHEEL_R   # m  -> maps cart force to wheel torque: tau = F * r
CONTROL_SIGN = +1.0         # found during validation (flip if your real robot drives the wrong way)


def design_gains():
    """K (force gains on [x, pitch, vx, pitch_rate]) for the real robot, designed now."""
    p = rp.assemble()
    K, _ = compute_K(p["L"], p["pole_mass"], p["cart_mass"], Q, R)
    return K.flatten()

CONFIG = {
    "torque_limit":   rp.MOTOR_STALL_TORQUE,  # N*m motor saturation (gearbox stall)
    # --- non-ideal knobs: raise toward real values, re-check it still settles ---
    "gyro_noise":     0.0,   # rad/s  (real MPU6050 ~0.01-0.03)
    "pitch_noise":    0.0,   # rad
    "ctrl_latency":   0,     # sim steps of delay (1 step = 2 ms)
    "torque_deadband":0.0,   # N*m below which the motor won't move (PWM deadband)
}


def pitch_from_quat(q):
    w, x, y, z = q
    return np.arctan2(2 * (w * y - z * x), 1 - 2 * (y * y + x * x))


def make_controller(cfg, K):
    buf = [0.0] * (cfg["ctrl_latency"] + 1)
    rng = np.random.default_rng(0)

    def control(d):
        x         = d.qpos[0]
        pitch     = pitch_from_quat(d.sensor("imu_quat").data) + rng.normal(0, cfg["pitch_noise"])
        x_dot     = d.qvel[0]
        pitch_rate= d.sensor("imu_gyro").data[1] + rng.normal(0, cfg["gyro_noise"])

        state = np.array([x, CONTROL_SIGN * pitch, x_dot, CONTROL_SIGN * pitch_rate])
        force = -(K @ state)                          # LQR: desired cart force
        u = force * WHEEL_RADIUS * CONTROL_SIGN        # -> wheel torque

        if abs(u) < cfg["torque_deadband"]:
            u = 0.0
        u = float(np.clip(u, -cfg["torque_limit"], cfg["torque_limit"]))
        buf.append(u)
        return buf.pop(0)

    return control


def simulate(headless=False, seconds=None, controller_enabled=True, initial_pitch=0.05):
    model = mujoco.MjModel.from_xml_string(plant_xml())
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    data.qpos[3:7] = [np.cos(initial_pitch / 2), 0, np.sin(initial_pitch / 2), 0]
    K = design_gains()
    print(f"flying K (cart force) = {np.round(K, 3)}")
    control = make_controller(CONFIG, K)

    def step():
        u = control(data) if controller_enabled else 0.0
        data.ctrl[0] = u           # both wheels same torque = drive straight
        data.ctrl[1] = u           # (differ them later to STEER)
        mujoco.mj_step(model, data)

    if headless:
        for _ in range(int((seconds or 5.0) / model.opt.timestep)):
            step()
        p = pitch_from_quat(data.sensor("imu_quat").data)
        print(f"headless {data.time:.1f}s: pitch={np.degrees(p):+.2f}deg  x={data.qpos[0]*1000:+.1f}mm")
        return

    from mujoco import viewer as mj_viewer
    with mj_viewer.launch_passive(model, data) as viewer:
        start = time.time()
        while viewer.is_running():
            if seconds and data.time > seconds:
                break
            step(); viewer.sync()
            lag = data.time - (time.time() - start)
            if lag > 0:
                time.sleep(lag)


def main():
    simulate(controller_enabled=True)


if __name__ == "__main__":
    main()
