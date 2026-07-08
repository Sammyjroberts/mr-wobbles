# Mr. Wobbles: a self-balancing two-wheel robot

![Mr. Wobbles balancing in simulation, taking a shove and recovering](assets/balance.gif)

*MuJoCo sim: the robot settles upright, takes a sideways shove at ~1.5 s, lunges ~27 cm to
catch itself, and drives back to center. Peak tilt during recovery ~3.6°. The controller is
the LQR designed in this repo; render it yourself with `scripts/record_gif.py`.*

An inverted-pendulum robot built from first principles: the controller isn't tuned by
hand on the bench; it's **designed against the real machine**. Physical parameters are
derived from the printed chassis (mass and center of mass pulled straight from the STL)
plus component datasheets, fed into a linearized model, and an optimal (LQR) controller
is solved offline. The same parameters generate the MuJoCo validation plant, so the sim
and the design can never silently disagree.

Python does the physics and control design; the on-robot firmware (Rust + Embassy on an
RP2040) is the next phase and consumes the gains this repo produces.

---

## The finding that shaped the design

The chassis is a tall, light plate, so intuition says the center of mass is high and the
robot falls slowly. It's the opposite. **Two 100 g gearmotors sit right on the wheel axle**,
and they drag the CoM down hard. Computed from the actual STL and datasheet masses:

| quantity                          | value    |
|-----------------------------------|----------|
| plate mass (from STL, PETG)       | 105 g    |
| pole / body mass                  | 355 g    |
| total mass                        | 465 g    |
| **L, CoM height above the axle**  | **27.7 mm** |

A naive guess put L near 90 mm; this is **~3× shorter**. A low CoM means a short pendulum,
which means a *fast-falling, twitchy* robot that demands a quicker control loop and leaves
less margin. Concrete design takeaway baked into the params: **mount the battery high.**
An 85 g LiPo up top raises L from 28 mm to ~49 mm and makes balancing far more forgiving.

This is the whole point of deriving parameters instead of guessing them: the hard part of
the problem was invisible until the numbers were real.

---

## How it works

```
robot_params.py   physical truth  →  mass, CoM, and L straight from the STL + datasheets
      │
      ├──────────────► lqr_design.py   linearize (finite-difference A, B) → Riccati → K
      │                                    4 gains for state [x, ẋ, pitch, pitch_rate]
      │
      └──────────────► plant.py        generate the wheels-on-ground MuJoCo contact model
                                           from the *same* params (validation, not design)
                                        │
                             balance.py │  real-time loop: u = −K · state, live viewer
```

Two ideas do the heavy lifting:

- **Single source of truth.** Every physical number comes from `robot_params.py`. The gains
  `K` are solved at startup (never hardcoded) and the validation plant is generated from the
  same params (never a hand-written XML). Both of those were stale-parameter bugs earlier in
  the project; deriving everything from one place made that class of bug impossible.
- **The robot's job is trivial; the design is where the work is.** On hardware the controller
  is a single dot product, `u = −K · state`. All the intelligence lives in the offline
  Riccati solve that produces `K`.

---

## Results (in simulation)

Measured against the wheels-on-ground contact plant (`plant.py`), driven by the LQR from
`lqr_design.py`. Every number below is regenerated live from the design, so the table can't
drift from the code: `uv run python scripts/report_stats.py`.

| metric                         | value                          | reading |
|--------------------------------|--------------------------------|---------|
| open-loop unstable pole        | \|z\| = **1.049** (> 1)        | it genuinely wants to fall (gravity runaway) |
| closed-loop poles              | \|z\| = 0.72, 0.98, 0.998      | all inside the unit circle → **stabilized** |
| pitch recovery (from 3° tilt)  | back within 1° in **~0.1 s**   | the balancing loop is fast and aggressive |
| disturbance rejection          | **2.2 N** shove, peak tilt **3.7°** | lunges ~28 cm to catch itself, then recovers |
| position re-centering          | within ±10 mm in **~3.3 s**    | the slow mode (see note below) |
| peak control effort            | **0.19 N·m** of 0.31 N·m stall | ~**40 % torque headroom**, motors are sized right |

The GIF up top is exactly this: the 2.2 N shove and recovery.

**Honest caveat:** the position loop is intentionally soft (low position weight in `Q`), so
after re-centering a small sub-degree pitch ripple lingers before it's truly dead-still. That's
a deliberate trade: a stiffer position gain fights the balance loop. Tightening it (retuning
`Q` in `lqr_design.py`) is future work, and the kind of thing that only matters once it's on
real hardware.

---

## Quickstart

Uses [uv](https://docs.astral.sh/uv/).

```bash
uv sync                 # create the env + install deps

uv run balancer-params  # print the physical numbers (mass, CoM, L)
uv run balancer-design  # solve LQR → gains K → outputs/Kc_real.npy
uv run balancer-sim     # live balancing simulation (MuJoCo viewer)
```

The console scripts are thin wrappers around `python -m balancer.sim.balance`, etc.

---

## Layout

```
src/balancer/
  params/robot_params.py   physical truth: mass/CoM/L from the STL + datasheet masses
  sim/lqr_design.py        linearized model → LQR → K (the gains)
  sim/balance.py           real-time controller + MuJoCo viewer  (u = −K · state)
  sim/plant.py             wheels-on-ground contact model, generated from params
  paths.py                 resolves repo data dirs (cad/, outputs/)
cad/balancer_chassis_v2.stl   printable chassis plate (PETG, prints flat)
cad/gen_chassis.py            parametric generator for the plate
hardware/wiring_phase1.svg    Phase-1 wiring (balance on IMU, no encoders)
hardware/chassis_drawing.svg  dimensioned plate drawing
scripts/record_gif.py         renders assets/balance.gif from the sim
scripts/report_stats.py       prints the Results table numbers, live from the design
outputs/                      computed K, physical summary
```

---

## Hardware roadmap

- **Phase 1 (current):** Pico (USB) + TB6612 driver + 2 gearmotors + IMU (STEMMA QT).
  Balances on the IMU alone, so expect drift and wander, since there's no position feedback.
  Simplest wiring, no level shifter needed. See `hardware/wiring_phase1.svg`.
  **Build status:** validated in sim; physical assembly is **blocked on a wheel-to-motor
  adapter** (the 6 mm D-shaft to 70 mm scooter-wheel hub). Once it arrives, build + bring-up.
- **Phase 2:** add wheel encoders for position hold. They run at 5 V and their outputs
  exceed the Pico's 3.3 V limit, so this phase needs a **logic level shifter** (BSS138).
- **Firmware:** on-robot control loop in **Rust + Embassy** on the RP2040: async tasks with
  a fixed-interval `Ticker`, I²C to the IMU, PWM + direction GPIO to the TB6612. It reads the
  gains `K` produced by `balancer-design`. Designed, not yet built.

---

## Notes

- `M_WHEEL` and `M_ELECTRONICS` in `robot_params.py` are estimates; refine if you weigh the
  parts. The motors and plate dominate the CoM and are both known precisely.
- `plant.py`'s pole inertia is lumped/approximate (mass and CoM height are exact); swap in a
  distributed inertia for tighter validation.
- `CONTROL_SIGN` in `balance.py`: flip it if the motors drive *into* the fall instead of
  catching it; the usual first-run sign gotcha.

MIT licensed.
