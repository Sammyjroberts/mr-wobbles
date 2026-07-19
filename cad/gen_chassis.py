"""
gen_chassis.py  -  v4: a friendly BARREL-on-legs chassis for Mr. Wobbles.

The look people picture for a cute balancer (and the reference the design targets):
a horizontal barrel BODY with a little face on the front, held up off the ground by
two splayed legs that carry the gearmotors + wheels at the bottom.

  * BODY: a closed, hollow barrel (a drum lying on its side, axis fore-aft), with a
    face embossed on the front cap. It rides high, so the mass is high -- exactly the
    fix for this robot's low-CoM twitchiness (see README). It prints as a two-piece
    clamshell split on the horizontal mid-plane: drop the electronics + battery into
    the lower half, cap it with the upper half.
  * LEGS: four rods splay down-and-out from the lower body to two motor-holder blocks.
  * FEET: the two holder blocks cradle the 25Dx65L motors (bore along the axle); the
    wheels sit out each side at the bottom. Motor leads run up a slot into the body.

Coordinate frame: the WHEEL AXLE is the origin (0,0,0). X = width (the axle the wheels
spin on), Y = up (the whole body is +Y, above the axle), Z = fore-aft (face on +Z). So a
point's Y coordinate is directly its height above the axle -- what robot_params.py wants.
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
BODY_LEN = 100.0            # barrel length (fore-aft, along Z, flat caps)
BODY_CY = 88.0              # body axis height above the wheel axle
WALL = 2.5                  # shell wall
FRONT_WALL = 4.0            # front cap thicker so the face embosses cleanly
SPLIT_Y = BODY_CY           # clamshell split plane (through the axis)
BODY_Z0 = -BODY_LEN / 2.0   # back cap plane
BODY_Z1 = BODY_LEN / 2.0    # front cap plane (face)

# motors: Pololu #4863 25Dx65L, coaxial along X at the axle (Y=0)
MOTOR_R = 12.75             # 25 mm dia + slip fit, /2
HOLD_X0, HOLD_X1 = 15.0, 52.0   # each holder block spans this |x| range
HOLD_Y0, HOLD_Y1 = -15.0, 9.0
HOLD_Z = 30.0              # holder depth

# legs: four rods, holder-top -> lower body surface (kept inside radius BODY_R)
LEG_R = 5.5
LEG_TOP = (34.0, 9.0)      # (|x|, y) where a leg meets its holder
LEG_BODY = (15.0, 54.0)    # (|x|, y) where a leg meets the body
LEG_TOP_Z = 13.0           # front/back rods at +/- this in Z at the foot
LEG_BODY_Z = 20.0          # ... and at the body

# face on the front cap (+Z = BODY_Z1)
EYE_R, EYE_Y, EYE_X, EYE_DEPTH = 6.0, 100.0, 13.0, 1.8
MOUTH_Y, MOUTH_R, MOUTH_DEPTH = 76.0, 15.0, 1.8


def box(w, h, d, center=(0, 0, 0)):
    m = trimesh.creation.box(extents=[w, h, d])
    m.apply_translation(center)
    return m


def z_cyl(radius, height, center=(0, 0, 0), sections=96):
    c = trimesh.creation.cylinder(radius=radius, height=height, sections=sections)
    c.apply_translation(center)
    return c


def x_cyl(radius, height, center=(0, 0, 0)):
    c = trimesh.creation.cylinder(radius=radius, height=height, sections=64)
    c.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
    c.apply_translation(center)
    return c


def rod(p0, p1, radius=LEG_R):
    return trimesh.creation.cylinder(radius=radius, segment=[p0, p1], sections=24)


def build_shell():
    # --- solid barrel body (flat caps) ---
    solids = [z_cyl(BODY_R, BODY_LEN, center=(0, BODY_CY, 0))]

    # --- motor holder blocks (the feet) ---
    for side in (+1, -1):
        cx = side * (HOLD_X0 + HOLD_X1) / 2.0
        solids.append(box(HOLD_X1 - HOLD_X0, HOLD_Y1 - HOLD_Y0, HOLD_Z,
                          center=(cx, (HOLD_Y0 + HOLD_Y1) / 2.0, 0)))

    # --- four splayed legs ---
    for side in (+1, -1):
        for zside in (+1, -1):
            top = (side * LEG_TOP[0], LEG_TOP[1], zside * LEG_TOP_Z)
            bot = (side * LEG_BODY[0], LEG_BODY[1], zside * LEG_BODY_Z)
            solids.append(rod(top, bot))

    body = trimesh.boolean.union(solids, engine=ENGINE)

    # --- subtract: hollow interior, motor bore, wire slot, face ---
    tools = []
    # hollow: leave FRONT_WALL at +Z, WALL at -Z and around
    cav_len = BODY_LEN - FRONT_WALL - WALL
    cav_zc = (BODY_Z0 + WALL + BODY_Z1 - FRONT_WALL) / 2.0
    tools.append(z_cyl(BODY_R - WALL, cav_len, center=(0, BODY_CY, cav_zc)))

    # motor bore through both feet along the axle
    tools.append(x_cyl(MOTOR_R, 2 * HOLD_X1 + 40, center=(0, 0, 0)))
    # wire pass-through slot in the lower body wall
    tools.append(box(22, 16, 26, center=(0, BODY_CY - BODY_R + WALL, 0)))

    # face on the front cap
    fz = BODY_Z1
    for side in (+1, -1):
        tools.append(z_cyl(EYE_R, EYE_DEPTH * 2, center=(side * EYE_X, EYE_Y, fz - EYE_DEPTH)))
    smile_a = z_cyl(MOUTH_R, MOUTH_DEPTH * 2, center=(0, MOUTH_Y, fz - MOUTH_DEPTH))
    smile_b = z_cyl(MOUTH_R, MOUTH_DEPTH * 4, center=(0, MOUTH_Y + 7.0, fz - MOUTH_DEPTH))
    tools.append(trimesh.boolean.difference([smile_a, smile_b], engine=ENGINE))

    cut = trimesh.boolean.union(tools, engine=ENGINE)
    return trimesh.boolean.difference([body, cut], engine=ENGINE)


def clamshell(shell):
    """Split on the horizontal mid-plane for support-free printing."""
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
