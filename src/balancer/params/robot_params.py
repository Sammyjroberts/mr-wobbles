"""
robot_params.py  -  the physical truth of the real balancer.

Derives the chassis plate's mass/CoM straight from the printed STL, then assembles
it with datasheet component masses to compute the ONE number that decides how hard
this thing is to balance: L, the center-of-mass height above the wheel axle.

Edit the masses you can actually weigh; the motors + plate dominate and are known.
"""
import numpy as np
import trimesh

from balancer.paths import STL_PATH

PETG_DENSITY = 1.27e-3        # g/mm^3  (1.27 g/cm^3)
G = 9.81

# ---------- component masses (kg) ----------
M_MOTOR       = 0.100   # Pololu #4863 25D MP 12V 20.4:1 w/ 48 CPR encoder (datasheet ~0.10 kg)
M_WHEEL       = 0.055   # 70 mm scooter wheel + 4 mm hub  (ESTIMATE - edit if you weigh it)
M_ELECTRONICS = 0.050   # Pico + TB6612 + IMU + breadboard + wiring  (ESTIMATE)
M_BATTERY     = 0.000   # tethered now; set ~0.085 when a top LiPo is added

# ---------- geometry (m) ----------
WHEEL_R                 = 0.035   # 70 mm scooter wheel radius = axle height
HALF_TRACK             = 0.073   # half of ~146 mm wheel separation
AXLE_FROM_PLATE_BOTTOM = 0.040   # where the axle sits up inside the lower hole grid
MOTOR_STALL_TORQUE     = 0.314   # N*m  (3.2 kg*cm gearbox-output stall @ 12 V)

# ---------- component heights above the axle (m) ----------
Z_MOTOR       = 0.000   # motor body CoM sits ~on the axle line
Z_WHEEL       = 0.000   # wheels at the axle
Z_ELECTRONICS = 0.090   # mounted mid-upper plate
Z_BATTERY     = 0.140   # if/when mounted up high


def plate_properties():
    """Mass, CoM, and inertia of the printed plate, straight from the mesh."""
    m = trimesh.load(STL_PATH)
    vol_mm3 = float(m.volume)
    mass_kg = vol_mm3 * PETG_DENSITY / 1000.0
    com_mm  = m.center_mass            # plate is modeled centered at origin, height = Y
    return mass_kg, np.asarray(com_mm), vol_mm3


def assemble():
    plate_mass, plate_com, vol = plate_properties()
    # plate-center is 90 mm above the plate bottom; axle is AXLE_FROM_PLATE_BOTTOM above it.
    plate_center_above_axle = 0.090 - AXLE_FROM_PLATE_BOTTOM            # m
    z_plate = plate_center_above_axle + plate_com[1] / 1000.0          # + real CoM offset from holes

    # POLE (everything that tilts with the body): motors + plate + electronics + battery
    pole = [("motors", 2*M_MOTOR, Z_MOTOR),
            ("plate",  plate_mass, z_plate),
            ("electronics", M_ELECTRONICS, Z_ELECTRONICS),
            ("battery", M_BATTERY, Z_BATTERY)]
    pole_mass = sum(m for _, m, _ in pole)
    L         = sum(m*z for _, m, z in pole) / pole_mass      # CoM height above axle

    cart_mass  = 2 * M_WHEEL                                  # the rolling wheels
    total_mass = pole_mass + cart_mass

    return dict(plate_mass=plate_mass, plate_vol_cm3=vol/1000.0, z_plate=z_plate,
                pole_mass=pole_mass, L=L, cart_mass=cart_mass, total_mass=total_mass,
                wheel_r=WHEEL_R, half_track=HALF_TRACK, stall=MOTOR_STALL_TORQUE,
                breakdown=pole)


def main():
    p = assemble()
    print(f"plate (from STL):  {p['plate_mass']*1000:6.1f} g   ({p['plate_vol_cm3']:.1f} cm^3 PETG)")
    print(f"pole / body mass:  {p['pole_mass']*1000:6.1f} g")
    print(f"cart (wheels):     {p['cart_mass']*1000:6.1f} g")
    print(f"total mass:        {p['total_mass']*1000:6.1f} g")
    print(f"--> L (CoM above axle): {p['L']*1000:6.1f} mm   ({p['L']:.4f} m)")
    print("breakdown (mass @ height above axle):")
    for name, m, z in p["breakdown"]:
        print(f"    {name:12s} {m*1000:6.1f} g  @ {z*1000:6.1f} mm")


if __name__ == "__main__":
    main()
