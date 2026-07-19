"""
gen_chassis.py  -  v3: an ENCLOSED box chassis for Mr. Wobbles.

v1/v2 were a single flat backplate with everything zip-tied to the open face.
v3 is the "normal" self-balancing-robot shape people picture: a closed box that
holds the electronics, riding up high on two wheels.

  * A 4-walled box (open top, + a separate snap-on lid) is the BODY. The Pico,
    TB6612 driver and IMU seat on standoffs on the box floor; the battery straps
    high against the inside of the back wall.
  * The two 25Dx65L gearmotors thread through circular bores in four bulkhead
    plates that hang UNDER the box floor, so the wheels sit at the bottom, out
    each side, and the whole body pivots above them.
  * A wire pass-through in the floor takes the motor leads up into the box.

Why a box (beyond looks): the project's own finding is that Mr. Wobbles has a LOW
center of mass (motors sit right on the axle), which makes it twitchy, and the
fix is "mount the battery high." An enclosed box does exactly that for free -- the
shell mass and the strapped-high battery both sit well above the axle -- so L (CoM
height above the axle) goes UP and the robot gets more forgiving. The number
regenerates from this STL via `uv run balancer-params`.

Coordinate frame: the WHEEL AXLE is the origin (0,0,0). X = width (the axle the
wheels spin on), Y = up (body is all +Y, above the axle), Z = depth (front-back).
So a point's Y coordinate IS its height above the axle -- which is exactly what
robot_params.py wants. Everything is unions + one boolean difference.
"""
from pathlib import Path

import numpy as np
import trimesh

HERE = Path(__file__).parent
OUT_SHELL = HERE / "balancer_chassis_v3.stl"
OUT_LID = HERE / "balancer_lid_v3.stl"

# ---------------- parameters (mm) ----------------
# enclosure (the electronics box) -- roughly square front face, sits up high
BOX_W, BOX_H, BOX_D = 112.0, 96.0, 62.0     # outer width x height x depth
WALL = 2.5                                   # wall / floor thickness
BOX_BOTTOM_Y = 14.0                          # box floor's outer underside, above the axle
                                             # (clears the motor bodies at Y=0)

# motors: Pololu #4863 25Dx65L, body ~25 mm dia, mounted coaxial along X at Y=0
MOTOR_R = 12.75                              # 25 mm dia + 0.5 mm slip fit, /2
BORE_X = [20.0, 50.0]                        # bulkhead plates per side (near center + outer)
PLATE_TX = 5.0                               # plate thickness along X
PLATE_D = 34.0                               # plate depth along Z
PLATE_TOP_Y = BOX_BOTTOM_Y + 3.0             # overlap up into the floor for a clean weld
PLATE_BOT_Y = -18.0                          # hangs below the axle; ground is at -35 (wheel r)

# electronics standoffs on the floor (board ~50 x 38, M2.5)
STANDOFF_H = 6.0
STANDOFF_R = 3.0
STANDOFF_PILOT_R = 1.1
STANDOFF_XY = [(22.0, 16.0), (22.0, -16.0), (-22.0, 16.0), (-22.0, -16.0)]  # (x, z)

# battery strap slots high on the back wall (holds an ~85 g LiPo up top)
STRAP_W, STRAP_H = 3.0, 10.0
STRAP_Y = 78.0
STRAP_X = 20.0

FLOOR_TOP_Y = BOX_BOTTOM_Y + WALL            # inside floor surface


def box(w, h, d, center=(0, 0, 0)):
    m = trimesh.creation.box(extents=[w, h, d])
    m.apply_translation(center)
    return m


def x_cylinder(radius, length, center=(0, 0, 0)):
    """A cylinder whose axis runs along X (motors + bores lie on the axle)."""
    c = trimesh.creation.cylinder(radius=radius, height=length, sections=64)
    c.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
    c.apply_translation(center)
    return c


def y_cylinder(radius, length, center=(0, 0, 0)):
    c = trimesh.creation.cylinder(radius=radius, height=length, sections=32)
    c.apply_translation(center)
    return c


def build_shell():
    box_cy = BOX_BOTTOM_Y + BOX_H / 2.0                      # box vertical center

    # --- the closed body: outer box minus an inner cavity that pokes out the top ---
    outer = box(BOX_W, BOX_H, BOX_D, center=(0, box_cy, 0))
    cavity = box(BOX_W - 2 * WALL, BOX_H, BOX_D - 2 * WALL,
                 center=(0, box_cy + WALL, 0))              # shifted up -> floor stays, top opens
    solids = [outer]

    # --- motor bulkheads under the floor (the wheels hang off these) ---
    plate_cy = (PLATE_TOP_Y + PLATE_BOT_Y) / 2.0
    plate_h = PLATE_TOP_Y - PLATE_BOT_Y
    for side in (+1, -1):
        for bx in BORE_X:
            solids.append(box(PLATE_TX, plate_h, PLATE_D, center=(side * bx, plate_cy, 0)))

    # --- electronics standoffs on the floor ---
    for (sx, sz) in STANDOFF_XY:
        solids.append(y_cylinder(STANDOFF_R, STANDOFF_H,
                                 center=(sx, FLOOR_TOP_Y + STANDOFF_H / 2.0, sz)))

    body = trimesh.boolean.union(solids, engine="manifold")

    # --- subtract everything at once ---
    tools = [cavity]
    # motor bore: one straight channel through all four bulkheads at the axle
    tools.append(x_cylinder(MOTOR_R, BOX_W + 40, center=(0, 0, 0)))
    # wire pass-through in the floor, center
    tools.append(box(16, WALL + 4, 12, center=(0, FLOOR_TOP_Y - WALL / 2.0, 0)))
    # standoff pilot holes
    for (sx, sz) in STANDOFF_XY:
        tools.append(y_cylinder(STANDOFF_PILOT_R, STANDOFF_H + WALL + 2,
                                center=(sx, FLOOR_TOP_Y + STANDOFF_H / 2.0, sz)))
    # battery strap slots through the back wall (-Z)
    back_z = -(BOX_D / 2.0)
    for side in (+1, -1):
        tools.append(box(STRAP_W, STRAP_H, WALL + 4, center=(side * STRAP_X, STRAP_Y, back_z)))

    cut = trimesh.boolean.union(tools, engine="manifold")
    shell = trimesh.boolean.difference([body, cut], engine="manifold")
    return shell


def build_lid():
    """Flat snap-on lid: a top plate + a lip that drops into the box opening.

    Modelled flat (prints face-down, no supports); on the robot it sits at the
    top of the box, ~BOX_H above the axle.
    """
    lip_clear = 0.4
    rim = 3.0
    top = box(BOX_W, 2.5, BOX_D, center=(0, 1.25, 0))
    lip_ow = BOX_W - 2 * WALL - 2 * lip_clear
    lip_od = BOX_D - 2 * WALL - 2 * lip_clear
    lip_outer = box(lip_ow, 5.0, lip_od, center=(0, -2.5, 0))
    lip_inner = box(lip_ow - 2 * rim, 7.0, lip_od - 2 * rim, center=(0, -2.5, 0))
    lip = trimesh.boolean.difference([lip_outer, lip_inner], engine="manifold")  # rim frame
    lid = trimesh.boolean.union([top, lip], engine="manifold")
    # two vent slots
    vents = [box(30, 6, 4, center=(0, 1.5, 12)), box(30, 6, 4, center=(0, 1.5, -12))]
    lid = trimesh.boolean.difference([lid, trimesh.boolean.union(vents, engine="manifold")],
                                     engine="manifold")
    return lid


def report(mesh, name):
    vol_cm3 = mesh.volume / 1000.0
    com = np.round(mesh.center_mass, 1)
    print(f"{name}: watertight={mesh.is_watertight}  size_mm={np.round(mesh.extents, 1)}")
    print(f"    volume={vol_cm3:.1f} cm^3  (~{vol_cm3 * 1.27:.0f} g PETG)  CoM_mm={com}")


def main():
    shell = build_shell()
    shell.export(str(OUT_SHELL))
    report(shell, OUT_SHELL.name)

    lid = build_lid()
    lid.export(str(OUT_LID))
    report(lid, OUT_LID.name)

    # the number that matters: CoM height (Y) of the shell above the axle
    print(f"\nshell CoM height above axle: {shell.center_mass[1]:.1f} mm")


if __name__ == "__main__":
    main()
