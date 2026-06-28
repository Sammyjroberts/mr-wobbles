# Soma — Two-Wheel Self-Balancing Robot

A from-first-principles balancing robot: MuJoCo sim, LQR controller designed against
the *real* machine (parameters derived from the printed chassis + component datasheets),
and an IMU-first hardware bring-up.

## The key finding (why this repo exists)
The chassis plate is light and tall, but **two 100 g motors sit right on the wheel axle**,
which drags the center of mass down. Derived from the actual STL + datasheets:

| quantity                         | value      |
|----------------------------------|------------|
| plate mass (from STL, PETG)      | 105 g      |
| pole/body mass                   | 355 g      |
| **L (CoM height above axle)**    | **27.7 mm**|

That's ~3x shorter than a naive guess — a low CoM means a fast-falling, twitchy pendulum.
**Design takeaway:** when you add a battery, mount it HIGH. 85 g up top nearly doubles L
(28 -> 49 mm) and makes balancing far more forgiving. See `params/robot_params.py`.

## Layout
```
src/balancer/
  params/robot_params.py   # physical truth: derives mass/CoM/L from the STL + datasheet masses
  sim/lqr_design.py        # builds the linearized model from params, runs LQR -> K (gains)
  sim/balance.py           # real-time controller + MuJoCo viewer (u = -K . state)
  sim/balancer.xml         # full wheels-on-ground contact model (validation plant)
  paths.py                 # resolves the repo's data dirs (cad/, outputs/)
cad/balancer_chassis_v1.stl   # the printable chassis plate (PETG, prints flat)
cad/gen_chassis.py            # parametric generator for the plate
hardware/wiring_phase1.svg    # Phase-1 wiring (balance on IMU, no encoders)
hardware/chassis_drawing.svg  # dimensioned plate drawing
outputs/                      # computed K, summaries
```

## Run it
This project uses [uv](https://docs.astral.sh/uv/). One-time setup, then run anything:
```bash
uv sync                     # create the env + install deps from pyproject.toml

uv run balancer-params      # see the physical numbers / L
uv run balancer-design      # compute K for the real robot -> outputs/Kc_real.npy
uv run balancer-sim         # live balancing sim (viewer)
```
The console scripts above are equivalent to `uv run python -m balancer.sim.balance`, etc.

## Build phases (hardware)
- **Phase 1 (now):** Pico (USB) + TB6612 + 2 motors + IMU (STEMMA QT). Balances on the
  IMU alone — will drift/wander (no position feedback). Simplest wiring, no level shifter.
  See `hardware/wiring_phase1.svg`.
- **Phase 2 (later):** add the encoders for position hold. They run at 5 V and their
  outputs exceed the Pico's 3.3 V limit, so they need a **logic level shifter** (BSS138).

## Controller
`K` is computed offline by `lqr_design.py` (the Riccati solve). The robot only ever runs
`u = -K . state`, a dot product, with `state = [x, x_dot, pitch, pitch_rate]`.
For Phase 1 (IMU-only), use just the pitch and pitch-rate gains; x / x_dot need encoders.

## Notes / TODO
- `M_WHEEL`, `M_ELECTRONICS` in `robot_params.py` are estimates — refine if you weigh them.
- `balancer.xml` (contact model) currently holds the original assumed params; regenerate
  it from `robot_params.py` to validate the real-parameter K on the contact plant.
- `CONTROL_SIGN` in `balance.py`: flip if the motors drive into the fall instead of catching it.
