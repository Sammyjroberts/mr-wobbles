"""
gen_chassis_v2.py — wider balancing-robot backplate.

Changes from v1:
  * WIDTH 80 -> 150 mm  so the two 65 mm (Pololu #4863 25Dx65L) motors sit
    end-to-end at the bottom and the wheels land ~146 mm apart (matches
    HALF_TRACK = 0.073 m in robot_params.py).
  * The dead M3 hole GRID is gone (plate isn't threaded; the screws don't fit
    the nuts). Replaced with ZIP-TIE SLOT PAIRS positioned to strap each motor
    body down to the plate.
  * Electronics zip-tie slots up top, spaced for a small ~40x70 mm board now,
    but the pattern also lets a BB830 (165x55) strap across later.
  * Central wire pass-through kept.

Coordinate frame: plate centered at origin. X = width (left-right, the axis the
wheels sit on), Y = height (up-down; motors live at the bottom = negative Y),
Z = thickness. Everything is a boolean-difference of "tool" solids from the base.
"""
from pathlib import Path

import trimesh
import numpy as np

OUT_PATH = Path(__file__).parent / "balancer_chassis_v2.stl"

# ---------------- plate ----------------
W, H, T = 150.0, 180.0, 6.0          # width x height x thickness (mm)
base = trimesh.creation.box(extents=[W, H, T])

tools = []
slots = []          # (x, y) log of every slot we cut
SLOT_W, SLOT_H = 3.0, 9.0            # zip-tie slot: 3 mm wide x 9 mm tall


def slot(x, y, w=SLOT_W, h=SLOT_H):
    """Cut one rectangular zip-tie slot centered at (x, y)."""
    s = trimesh.creation.box(extents=[w, h, T + 2])
    s.apply_translation([x, y, 0.0])
    tools.append(s)
    slots.append((round(x, 1), round(y, 1)))


# ---------------- motor straps (bottom zone) ----------------
# Two motors lie end-to-end across the bottom, bodies meeting near center,
# shafts pointing out to the left/right edges. Each motor body is ~25 mm dia
# and 65 mm long. We strap each body down with TWO zip ties (an inner pair and
# an outer pair), so each motor gets 4 slots (2 ties x 2 slots).
#
# A motor spans roughly x in [12, 77] on the right side (and mirror on left),
# body centered around x ~= 44. Put one tie near the gearbox end (inner, ~x=25)
# and one near the outer end (~x=63). Slots straddle the body so the tie loops
# OVER the motor: a slot just inside the body edge and one just outside won't
# work (body is round) -- instead we place BOTH slots of a pair straddling the
# body centerline in Y, i.e. one above and one below the motor at that X.
#
# Motor axis sits at AXLE_FROM_PLATE_BOTTOM = 40 mm above plate bottom.
# plate bottom = -H/2 = -90, so motor-axis Y ~= -90 + 40 = -50.
MOTOR_AXIS_Y = -50.0
BODY_HALF = 14.0        # ~ motor radius + a hair, for the strap to clear

# tie X positions along each motor body (measured from center)
tie_x = [25.0, 63.0]

for side in (+1, -1):                      # right motor, then left motor
    for tx in tie_x:
        x = side * tx
        # a zip tie loops over the round body: slot above + slot below the axis
        slot(x, MOTOR_AXIS_Y + BODY_HALF)
        slot(x, MOTOR_AXIS_Y - BODY_HALF)

# ---------------- electronics straps (upper zone) ----------------
# Strap pairs to cinch the breadboard flat. Small board ~40x70; give a few rows.
for y in [10.0, 40.0, 70.0]:
    slot(-22.0, y)
    slot(+22.0, y)

# ---------------- central wire pass-through ----------------
wc = trimesh.creation.box(extents=[18, 12, T + 2])
wc.apply_translation([0.0, -12.0, 0.0])
tools.append(wc)

# ---------------- boolean + export ----------------
cut = trimesh.util.concatenate(tools)
plate = trimesh.boolean.difference([base, cut])
plate.export(str(OUT_PATH))

print("OUT:", OUT_PATH.name)
print("watertight:", plate.is_watertight)
print("size mm:", np.round(plate.extents, 1))
vol_cm3 = plate.volume / 1000.0
print("volume cm3:", round(vol_cm3, 1), " (~", round(vol_cm3 * 1.27, 0), "g PETG)")
print("motor+elec slots:", len(slots))