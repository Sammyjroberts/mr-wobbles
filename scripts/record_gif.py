"""Render the balancing sim to assets/balance.gif: settle, take a push, recover.

Runs the real controller against the generated contact plant, applies a brief
horizontal shove mid-run, and records a side-on view so the pitch lean and the
catch-the-fall lunge are both visible.

    uv run --with imageio --with pillow python scripts/record_gif.py
"""
import numpy as np
import mujoco
import imageio

from balancer.params import robot_params as rp
from balancer.sim.plant import plant_xml
from balancer.sim.balance import design_gains, make_controller, CONFIG, pitch_from_quat

W, H, FPS = 640, 480, 25
DT = 0.002
STEPS_PER_FRAME = int((1.0 / FPS) / DT)
DURATION = 6.0
INITIAL_PITCH = 0.03
PUSHES = [(1.5, 1.56, 2.2)]   # (t_start, t_end, force_x N)
OUT = "assets/balance.gif"

model = mujoco.MjModel.from_xml_string(plant_xml())
data = mujoco.MjData(model)
mujoco.mj_resetData(model, data)
data.qpos[3:7] = [np.cos(INITIAL_PITCH / 2), 0, np.sin(INITIAL_PITCH / 2), 0]

control = make_controller(CONFIG, design_gains())
chassis_id = model.body("chassis").id

renderer = mujoco.Renderer(model, height=H, width=W)
cam = mujoco.MjvCamera()
cam.azimuth, cam.elevation, cam.distance = 90, -10, 0.62
cam.lookat[:] = [0.12, 0, 0.06]

frames, peak_pitch = [], 0.0
for i in range(int(DURATION / DT)):
    u = control(data)
    data.ctrl[0] = data.ctrl[1] = u

    data.xfrc_applied[chassis_id, :] = 0.0
    for ts, te, fx in PUSHES:
        if ts <= data.time < te:
            data.xfrc_applied[chassis_id, 0] = fx

    mujoco.mj_step(model, data)
    peak_pitch = max(peak_pitch, abs(np.degrees(pitch_from_quat(data.sensor("imu_quat").data))))

    if i % STEPS_PER_FRAME == 0:
        renderer.update_scene(data, camera=cam)
        frames.append(renderer.render())

imageio.mimsave(OUT, frames, fps=FPS, loop=0)
print(f"peak |pitch| during recovery = {peak_pitch:.1f} deg   wrote {OUT} ({len(frames)} frames)")
