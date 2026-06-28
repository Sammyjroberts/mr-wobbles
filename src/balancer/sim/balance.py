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
from pathlib import Path

import mujoco
import numpy as np

XML_PATH = Path(__file__).parent / "balancer.xml"

# ===== LQR gains (designed in lqr_design.py; re-run it if you change the robot) =====
K_LQR = np.array([-3.255, -28.478, -3.690, -2.505])   # force gains on [x, pitch, vx, pitch_rate]
WHEEL_RADIUS = 0.035        # m  -> maps cart force to wheel torque: tau = F * r
CONTROL_SIGN = +1.0         # found during validation (flip if your real robot drives the wrong way)

CONFIG = {
    "torque_limit":   0.31,  # N*m motor saturation (= your 3.2 kg*cm stall)
    # --- non-ideal knobs: raise toward real values, re-check it still settles ---
    "gyro_noise":     0.0,   # rad/s  (real MPU6050 ~0.01-0.03)
    "pitch_noise":    0.0,   # rad
    "ctrl_latency":   0,     # sim steps of delay (1 step = 2 ms)
    "torque_deadband":0.0,   # N*m below which the motor won't move (PWM deadband)
}


def pitch_from_quat(q):
    w, x, y, z = q
    return np.arctan2(2 * (w * y - z * x), 1 - 2 * (y * y + x * x))


def make_controller(cfg):
    buf = [0.0] * (cfg["ctrl_latency"] + 1)
    rng = np.random.default_rng(0)

    def control(d):
        x         = d.qpos[0]
        pitch     = pitch_from_quat(d.sensor("imu_quat").data) + rng.normal(0, cfg["pitch_noise"])
        x_dot     = d.qvel[0]
        pitch_rate= d.sensor("imu_gyro").data[1] + rng.normal(0, cfg["gyro_noise"])

        state = np.array([x, CONTROL_SIGN * pitch, x_dot, CONTROL_SIGN * pitch_rate])
        force = -(K_LQR @ state)                      # LQR: desired cart force
        u = force * WHEEL_RADIUS * CONTROL_SIGN        # -> wheel torque

        if abs(u) < cfg["torque_deadband"]:
            u = 0.0
        u = float(np.clip(u, -cfg["torque_limit"], cfg["torque_limit"]))
        buf.append(u)
        return buf.pop(0)

    return control


def simulate(headless=False, seconds=None, controller_enabled=True, initial_pitch=0.05):
    model = mujoco.MjModel.from_xml_path(str(XML_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    data.qpos[3:7] = [np.cos(initial_pitch / 2), 0, np.sin(initial_pitch / 2), 0]
    control = make_controller(CONFIG)

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
