"""
robot_params.py  -  the physical truth of the real balancer.

Derives the chassis shell's mass/CoM straight from the printed STL, then assembles
it with datasheet component masses to compute the ONE number that decides how hard
this thing is to balance: L, the center-of-mass height above the wheel axle.

The v4 chassis mesh is modelled with the WHEEL AXLE at the origin (see cad/gen_chassis.py),
so a point's Y coordinate is directly its height above the axle -- the shell's own
CoM height needs no offset arithmetic. Edit the masses you can actually weigh; the
motors + shell dominate and are known.
"""
import numpy as np
import trimesh

from balancer.paths import STL_PATH, PHYS_SUMMARY_PATH

PETG_DENSITY = 1.27e-3        # g/mm^3  (1.27 g/cm^3)
G = 9.81

# ---------- component masses (kg) ----------
M_MOTOR       = 0.100   # Pololu #4863 25D MP 12V 20.4:1 w/ 48 CPR encoder (datasheet ~0.10 kg)
M_WHEEL       = 0.055   # 70 mm scooter wheel + 4 mm hub  (ESTIMATE - edit if you weigh it)
M_ELECTRONICS = 0.050   # Pico + TB6612 + IMU + wiring, inside the barrel  (ESTIMATE)
M_BATTERY     = 0.085   # 3S LiPo inside the barrel, high (replaces the tether)

# ---------- geometry (m) ----------
WHEEL_R            = 0.035   # 70 mm scooter wheel radius = axle height
HALF_TRACK         = 0.073   # half of ~146 mm wheel separation
MOTOR_STALL_TORQUE = 0.314   # N*m  (3.2 kg*cm gearbox-output stall @ 12 V)

# ---------- component heights above the axle (m) ----------
# The shell height comes from the STL; these are the loose parts nested inside the barrel.
Z_MOTOR       = 0.000   # motor body CoM sits on the axle line (bores at Y=0)
Z_WHEEL       = 0.000   # wheels at the axle
Z_ELECTRONICS = 0.058   # board resting in the lower half of the barrel
Z_BATTERY     = 0.085   # LiPo nested high in the barrel (near the body axis, BODY_CY)


def _mesh_props(path):
    """Mass (kg from PETG volume) and CoM (mm) of a printed part, straight from the mesh."""
    m = trimesh.load(path)
    vol_mm3 = float(m.volume)
    mass_kg = vol_mm3 * PETG_DENSITY / 1000.0
    return mass_kg, np.asarray(m.center_mass), vol_mm3


def plate_properties():
    """Back-compat alias: mass/CoM/volume of the main chassis shell."""
    return _mesh_props(STL_PATH)


def assemble():
    shell_mass, shell_com, vol = plate_properties()
    z_shell = shell_com[1] / 1000.0                 # axle at mesh origin -> Y is height above axle

    # POLE (everything that tilts with the body): motors + shell + electronics + battery
    pole = [("motors", 2*M_MOTOR, Z_MOTOR),
            ("shell",  shell_mass, z_shell),
            ("electronics", M_ELECTRONICS, Z_ELECTRONICS),
            ("battery", M_BATTERY, Z_BATTERY)]
    pole_mass = sum(m for _, m, _ in pole)
    L         = sum(m*z for _, m, z in pole) / pole_mass      # CoM height above axle

    cart_mass  = 2 * M_WHEEL                                  # the rolling wheels
    total_mass = pole_mass + cart_mass

    return dict(plate_mass=shell_mass, plate_vol_cm3=vol/1000.0, z_plate=z_shell,
                pole_mass=pole_mass, L=L, cart_mass=cart_mass, total_mass=total_mass,
                wheel_r=WHEEL_R, half_track=HALF_TRACK, stall=MOTOR_STALL_TORQUE,
                breakdown=pole)


def summarize(p):
    lines = [
        f"plate (from STL):  {p['plate_mass']*1000:6.1f} g   ({p['plate_vol_cm3']:.1f} cm^3 PETG)",
        f"pole / body mass:  {p['pole_mass']*1000:6.1f} g",
        f"cart (wheels):     {p['cart_mass']*1000:6.1f} g",
        f"total mass:        {p['total_mass']*1000:6.1f} g",
        f"--> L (CoM above axle): {p['L']*1000:6.1f} mm   ({p['L']:.4f} m)",
        "breakdown (mass @ height above axle):",
        *[f"    {name:12s} {m*1000:6.1f} g  @ {z*1000:6.1f} mm" for name, m, z in p["breakdown"]],
    ]
    return "\n".join(lines)


def main():
    text = summarize(assemble())
    print(text)
    PHYS_SUMMARY_PATH.parent.mkdir(exist_ok=True)
    PHYS_SUMMARY_PATH.write_text(text + "\n")   # regenerate the committed summary from code


if __name__ == "__main__":
    main()
