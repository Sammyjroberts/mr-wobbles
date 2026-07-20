"""
gen_chassis.py  -  v4: a barrel-across-the-top chassis on K-shaped legs.

The "traditional" two-wheel-balancer silhouette: the body is a horizontal barrel
lying ACROSS the wheelbase (its axis parallel to the wheel axle), held up between
two braced K-shaped legs that carry the gearmotors + wheels at the bottom. Face is
embossed on the front curve.

  * BODY: a closed, hollow barrel, axis along X (left-right), face on the +Z front.
    Rides high, so the mass is high -- the fix for this robot's low-CoM twitchiness
    (see README). Prints as a two-piece clamshell split on the horizontal mid-plane:
    nest the electronics + battery in the lower half, cap with the upper half.
  * LEGS: one flat K-truss per side (a spine + two diagonals meeting at a knee),
    the classic braced-leg look. Each leg's foot straddles the wheel axle.
  * FEET: holder blocks cradle the 25Dx65L motors (bore along the axle); wheels sit
    out each side. Motor leads run up a center slot into the body.

Coordinate frame: the WHEEL AXLE is the origin (0,0,0). X = width (the axle the wheels
spin on, and now the barrel's long axis too), Y = up (body is +Y), Z = fore-aft (face on
+Z). A point's Y coordinate is directly its height above the axle -- what robot_params wants.
"""
from pathlib import Path

import numpy as np
import trimesh

HERE = Path(__file__).parent
OUT_SHELL = HERE / "balancer_chassis_v4.stl"          # full shell: mass/CoM source
OUT_TOP = HERE / "balancer_chassis_v4_top.stl"        # print piece: upper clamshell
OUT_BOT = HERE / "balancer_chassis_v4_bottom.stl"     # print piece: lower clamshell + legs
ENGINE = "manifold"

# ---------------- parameters (mm) ----------------
BODY_R = 38.0                # barrel radius
BODY_LX = 100.0             # barrel length, now along X (left-right)
BODY_CY = 108.0             # barrel axis height above the wheel axle (rides up on long legs)
WALL = 2.5                  # shell wall
SPLIT_Y = BODY_CY           # clamshell split plane (through the axis)

# motors: Pololu #4863 25Dx65L, coaxial along X at the axle (Y=0)
MOTOR_R = 12.75
HOLD_X0, HOLD_X1 = 15.0, 52.0
HOLD_Y0, HOLD_Y1 = -15.0, 9.0
HOLD_Z = 32.0

# K-legs: one flat truss per side, in the Y-Z plane, at |x| = LEG_X.
# The two diagonals meet the spine at the knee -> a braced "K"; tops attach to the
# wide lower flanks of the drum (near the axis height, where |Z| clearance is largest).
LEG_X = 46.0
LEG_THICK = 6.0            # plate thickness (along X)
BAR_W = 11.0              # strut width (in Z)
SPINE = ((-24.0, 94.0), (-4.0, 0.0))       # upper-back flank -> foot
DIAG_UP = ((28.0, 90.0), (-9.0, 45.0))     # upper-front flank -> knee
DIAG_LO = ((-9.0, 45.0), (26.0, 4.0))      # knee -> forward foot

# face on the front curve (+Z)
EYE_R, EYE_Y, EYE_X, EYE_DEPTH = 6.0, 120.0, 14.0, 1.3
MOUTH_Y, MOUTH_R, MOUTH_DEPTH = 94.0, 15.0, 1.3


def box(w, h, d, center=(0, 0, 0)):
    m = trimesh.creation.box(extents=[w, h, d])
    m.apply_translation(center)
    return m


def x_cyl(radius, length, center=(0, 0, 0), sections=96):
    c = trimesh.creation.cylinder(radius=radius, height=length, sections=sections)
    c.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
    c.apply_translation(center)
    return c


def z_cyl(radius, length, center=(0, 0, 0), sections=48):
    c = trimesh.creation.cylinder(radius=radius, height=length, sections=sections)
    c.apply_translation(center)
    return c


def bar_yz(p0, p1, x, width=BAR_W, thick=LEG_THICK):
    """A flat strut in the Y-Z plane (thin along X), from (Z,Y) p0 to p1, at plane X=x."""
    (z0, y0), (z1, y1) = p0, p1
    dy, dz = y1 - y0, z1 - z0
    length = float(np.hypot(dy, dz))
    b = trimesh.creation.box(extents=[thick, length + width, width])   # long axis = Y
    theta = np.arctan2(dz, dy)                                          # +Y -> (dy,dz)
    b.apply_transform(trimesh.transformations.rotation_matrix(theta, [1, 0, 0]))
    b.apply_translation([x, (y0 + y1) / 2.0, (z0 + z1) / 2.0])
    return b


def build_shell():
    solids = [x_cyl(BODY_R, BODY_LX, center=(0, BODY_CY, 0))]

    # motor holder blocks
    for side in (+1, -1):
        cx = side * (HOLD_X0 + HOLD_X1) / 2.0
        solids.append(box(HOLD_X1 - HOLD_X0, HOLD_Y1 - HOLD_Y0, HOLD_Z,
                          center=(cx, (HOLD_Y0 + HOLD_Y1) / 2.0, 0)))

    # K-legs
    for side in (+1, -1):
        for seg in (SPINE, DIAG_UP, DIAG_LO):
            solids.append(bar_yz(seg[0], seg[1], side * LEG_X))

    body = trimesh.boolean.union(solids, engine=ENGINE)

    # subtract: hollow, motor bore, wire slot, face
    tools = [x_cyl(BODY_R - WALL, BODY_LX - 2 * WALL, center=(0, BODY_CY, 0))]
    tools.append(x_cyl(MOTOR_R, 2 * HOLD_X1 + 40, center=(0, 0, 0)))
    tools.append(box(24, 16, 20, center=(0, BODY_CY - BODY_R + WALL, 0)))

    # face: blind recesses cut into the curved front, depth measured from the surface
    def surf_z(yf):
        return float(np.sqrt(max(BODY_R**2 - (yf - BODY_CY)**2, 1.0)))

    def emboss(radius, xf, yf, depth, length_out=12.0):
        return z_cyl(radius, length_out, center=(xf, yf, surf_z(yf) - depth + length_out / 2.0))

    for side in (+1, -1):
        tools.append(emboss(EYE_R, side * EYE_X, EYE_Y, EYE_DEPTH))
    sm = surf_z(MOUTH_Y)
    smile_a = z_cyl(MOUTH_R, 12, center=(0, MOUTH_Y, sm - MOUTH_DEPTH + 6))
    smile_b = z_cyl(MOUTH_R, 14, center=(0, MOUTH_Y + 7.0, sm - MOUTH_DEPTH + 7))
    tools.append(trimesh.boolean.difference([smile_a, smile_b], engine=ENGINE))

    cut = trimesh.boolean.union(tools, engine=ENGINE)
    return trimesh.boolean.difference([body, cut], engine=ENGINE)


def clamshell(shell):
    big = 400.0
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
