"""
gen_chassis.py  -  v4: a square box body on K-shaped braced legs.

The traditional two-wheel-balancer silhouette, with a boxy body that's easy to pack:
a rectangular enclosure rides high on two braced K-legs, gearmotors cradled in the
feet, wheels out each side.

  * BODY: a closed, hollow box -- flat walls + a flat floor make it simple to seat a
    rectangular board and battery (standoffs on the floor, wire slot for the leads).
    It rides high, so the mass is high -- the fix for this robot's low-CoM twitchiness
    (see README). Prints as a two-piece clamshell split on the horizontal mid-plane:
    drop the electronics into the lower tub, cap with the upper half.
  * LEGS: one flat K-truss per side (a spine + two diagonals meeting at a knee), the
    classic braced-leg look. Feet straddle the wheel axle.
  * FEET: holder blocks cradle the 25Dx65L motors (bore along the axle).

Coordinate frame: the WHEEL AXLE is the origin (0,0,0). X = width (the axle the wheels
spin on), Y = up (body is +Y, above the axle), Z = fore-aft. A point's Y coordinate is
directly its height above the axle -- what robot_params.py wants.
"""
from pathlib import Path

import numpy as np
import trimesh

HERE = Path(__file__).parent
OUT_SHELL = HERE / "balancer_chassis_v4.stl"          # full shell: mass/CoM source
OUT_TOP = HERE / "balancer_chassis_v4_top.stl"        # print piece: upper clamshell
OUT_BOT = HERE / "balancer_chassis_v4_bottom.stl"     # print piece: lower tub + legs
ENGINE = "manifold"

# ---------------- parameters (mm) ----------------
BOX_W, BOX_H, BOX_D = 118.0, 88.0, 70.0     # outer width x height x depth
BOX_CY = 104.0                              # box center height above the axle (rides up on legs)
WALL = 2.0
SPLIT_Y = BOX_CY                            # clamshell split (through box mid-height)
BOX_BOTTOM = BOX_CY - BOX_H / 2.0

# motors: Pololu #4863 25Dx65L, coaxial along X at the axle (Y=0)
MOTOR_R = 12.75
HOLD_X0, HOLD_X1 = 15.0, 52.0
HOLD_Y0, HOLD_Y1 = -15.0, 9.0
HOLD_Z = 34.0

# K-legs: one flat truss per side, in the Y-Z plane, at |x| = LEG_X.
# Tops weld into the box floor; the two diagonals meet the spine at the knee -> a "K".
LEG_X = 50.0
LEG_THICK = 6.0
BAR_W = 11.0
_TOP = BOX_BOTTOM + 2.0                     # weld height (into the floor slab)
SPINE = ((-24.0, _TOP), (-4.0, 0.0))       # back -> foot
DIAG_UP = ((28.0, _TOP), (-9.0, 36.0))     # front -> knee
DIAG_LO = ((-9.0, 36.0), (26.0, 4.0))      # knee -> forward foot

# electronics standoffs on the floor (board ~60 x 50, M2.5)
STANDOFF_H, STANDOFF_R, STANDOFF_PILOT_R = 6.0, 3.0, 1.1
STANDOFF_XZ = [(28, 20), (28, -20), (-28, 20), (-28, -20)]


def box(w, h, d, center=(0, 0, 0)):
    m = trimesh.creation.box(extents=[w, h, d])
    m.apply_translation(center)
    return m


def x_cyl(radius, length, center=(0, 0, 0)):
    c = trimesh.creation.cylinder(radius=radius, height=length, sections=64)
    c.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
    c.apply_translation(center)
    return c


def y_cyl(radius, length, center=(0, 0, 0)):
    c = trimesh.creation.cylinder(radius=radius, height=length, sections=28)
    c.apply_translation(center)
    return c


def bar_yz(p0, p1, x, width=BAR_W, thick=LEG_THICK):
    """A flat strut in the Y-Z plane (thin along X), from (Z,Y) p0 to p1, at plane X=x."""
    (z0, y0), (z1, y1) = p0, p1
    dy, dz = y1 - y0, z1 - z0
    length = float(np.hypot(dy, dz))
    b = trimesh.creation.box(extents=[thick, length + width, width])   # long axis = Y
    b.apply_transform(trimesh.transformations.rotation_matrix(np.arctan2(dz, dy), [1, 0, 0]))
    b.apply_translation([x, (y0 + y1) / 2.0, (z0 + z1) / 2.0])
    return b


def build_shell():
    solids = [box(BOX_W, BOX_H, BOX_D, center=(0, BOX_CY, 0))]

    for side in (+1, -1):                       # motor holder blocks (feet)
        cx = side * (HOLD_X0 + HOLD_X1) / 2.0
        solids.append(box(HOLD_X1 - HOLD_X0, HOLD_Y1 - HOLD_Y0, HOLD_Z,
                          center=(cx, (HOLD_Y0 + HOLD_Y1) / 2.0, 0)))

    for side in (+1, -1):                        # K-legs
        for seg in (SPINE, DIAG_UP, DIAG_LO):
            solids.append(bar_yz(seg[0], seg[1], side * LEG_X))

    for (sx, sz) in STANDOFF_XZ:                 # standoffs on the floor
        solids.append(y_cyl(STANDOFF_R, STANDOFF_H,
                            center=(sx, BOX_BOTTOM + WALL + STANDOFF_H / 2.0, sz)))

    body = trimesh.boolean.union(solids, engine=ENGINE)

    tools = [box(BOX_W - 2 * WALL, BOX_H - 2 * WALL, BOX_D - 2 * WALL, center=(0, BOX_CY, 0))]
    tools.append(x_cyl(MOTOR_R, 2 * HOLD_X1 + 40, center=(0, 0, 0)))     # motor bore
    tools.append(box(24, 16, 22, center=(0, BOX_BOTTOM + WALL / 2.0, 0)))  # wire slot in floor
    for (sx, sz) in STANDOFF_XZ:                                          # pilot holes
        tools.append(y_cyl(STANDOFF_PILOT_R, STANDOFF_H + WALL + 2,
                           center=(sx, BOX_BOTTOM + WALL + STANDOFF_H / 2.0, sz)))

    cut = trimesh.boolean.union(tools, engine=ENGINE)
    return trimesh.boolean.difference([body, cut], engine=ENGINE)


def clamshell(shell):
    big = 500.0
    top = trimesh.boolean.intersection(
        [shell, box(big, big, big, center=(0, SPLIT_Y + big / 2.0, 0))], engine=ENGINE)
    bot = trimesh.boolean.intersection(
        [shell, box(big, big, big, center=(0, SPLIT_Y - big / 2.0, 0))], engine=ENGINE)
    return top, bot


def report(mesh, name):
    v = mesh.volume / 1000.0
    print(f"{name}: watertight={mesh.is_watertight}  size={np.round(mesh.extents,1)}  "
          f"vol={v:.1f}cm^3 (~{v*1.27:.0f}g)  CoM_mm={np.round(mesh.center_mass,1)}")


def main():
    shell = build_shell()
    shell.export(str(OUT_SHELL))
    report(shell, OUT_SHELL.name)
    top, bot = clamshell(shell)
    top.export(str(OUT_TOP))
    bot.export(str(OUT_BOT))
    report(top, OUT_TOP.name)
    report(bot, OUT_BOT.name)
    print(f"\nshell CoM height above axle: {shell.center_mass[1]:.1f} mm")


if __name__ == "__main__":
    main()
