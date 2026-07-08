"""
plant.py - the contact-model "plant" (the validation physics), built from robot_params.

This is the full 3D, wheels-on-ground model the controller is VALIDATED against - as
opposed to the clean cart-pole stand-in K is DESIGNED on (see lqr_design.design_xml).
The two are deliberately different: you design on a clean model, then check the gains
hold on a messier, contact-rich one.

Generating it from robot_params means the validation plant always carries the real
robot's mass + CoM height (L). It can't drift back to "phantom" params the way the old
hand-written balancer.xml did - same single-source-of-truth fix we made for K.

Coordinate convention (this is what the controller assumes):
    x = forward/back   (drive + the direction it falls)
    y = left/right     (the wheel axle axis)
    z = up
    PITCH = rotation about y = the balancing axis. +pitch = nose tips forward (+x).

Inspect the generated XML with:  uv run balancer-plant
"""
from balancer.params import robot_params as rp


def plant_xml(p=None):
    """Build the contact-model MJCF from assembled robot params (defaults to rp.assemble())."""
    p = p or rp.assemble()
    L          = p["L"]              # CoM height above the axle (m) - sets the fall rate
    pole_mass  = p["pole_mass"]      # everything that tilts with the body (kg)
    wheel_mass = p["cart_mass"] / 2  # per wheel
    wheel_r    = p["wheel_r"]        # = axle height above floor
    half_track = p["half_track"]
    stall      = p["stall"]          # motor torque limit (N*m)
    body_h     = min(0.05, wheel_r + L)   # keep the body box visually above the floor

    return f"""<mujoco model="balancer">
  <compiler angle="radian" autolimits="true"/>

  <!-- 500 Hz physics. RK4 = accurate integrator, good for contact-light systems. -->
  <option timestep="0.002" integrator="RK4" gravity="0 0 -9.81"/>

  <default>
    <joint damping="0.001"/>
    <geom friction="1.0 0.005 0.0001"/>   <!-- sliding/torsional/rolling friction -->
  </default>

  <worldbody>
    <light pos="0 0 2"/>
    <geom name="floor" type="plane" size="0 0 0.05" rgba="0.3 0.3 0.35 1"/>

    <!-- chassis origin sits ON the axle line, wheel_r above the floor so wheels rest on it -->
    <body name="chassis" pos="0 0 {wheel_r}">
      <!-- freejoint = free in 3D; it falls in PITCH (about y), the two wheels stop it
           falling sideways (roll), and yaw is steerable by driving the wheels apart. -->
      <freejoint name="root"/>

      <!-- The tilting body, LUMPED into one box: it carries the whole pole mass with its
           center at z = L, the REAL CoM height above the axle (from robot_params). That
           mass + that height are what set the instability, and both are exact here.
           Collision is disabled (contype/conaffinity 0) so the body never spuriously
           touches the floor at low L - only the wheels contact ground, like the real robot.
           Its inertia (box + parallel-axis) is approximate; the real plate is distributed. -->
      <geom name="body" type="box" pos="0 0 {L:.6f}" size="0.02 0.04 {body_h:.6f}"
            mass="{pole_mass:.6f}" contype="0" conaffinity="0" rgba="0.85 0.4 0.2 1"/>

      <!-- IMU site: gyro/framequat are body-rigid, so its exact spot doesn't change the reading. -->
      <site name="imu" pos="0 0 {L:.6f}" size="0.005" rgba="0 1 0 1"/>

      <!-- ===== LEFT WHEEL ===== (at +y) -->
      <body name="left_wheel" pos="0 {half_track:.6f} 0">
        <joint name="left" type="hinge" axis="0 1 0"/>
        <geom name="left_tire" type="cylinder" size="{wheel_r:.6f} 0.0125" zaxis="0 1 0"
              mass="{wheel_mass:.6f}" rgba="0.1 0.1 0.1 1"/>
      </body>

      <!-- ===== RIGHT WHEEL ===== (at -y) -->
      <body name="right_wheel" pos="0 {-half_track:.6f} 0">
        <joint name="right" type="hinge" axis="0 1 0"/>
        <geom name="right_tire" type="cylinder" size="{wheel_r:.6f} 0.0125" zaxis="0 1 0"
              mass="{wheel_mass:.6f}" rgba="0.1 0.1 0.1 1"/>
      </body>
    </body>
  </worldbody>

  <!-- Direct-torque actuators; ctrlrange = wheel torque limit = the motor's gearbox stall. -->
  <actuator>
    <motor name="left_motor"  joint="left"  ctrlrange="{-stall:.4f} {stall:.4f}"/>
    <motor name="right_motor" joint="right" ctrlrange="{-stall:.4f} {stall:.4f}"/>
  </actuator>

  <!-- Sensor suite matching real hardware: IMU (gyro + accel + orientation) and wheel encoders. -->
  <sensor>
    <framequat   name="imu_quat" objtype="site" objname="imu"/>
    <gyro        name="imu_gyro" site="imu"/>
    <accelerometer name="imu_acc" site="imu"/>
    <jointpos name="left_enc"  joint="left"/>
    <jointpos name="right_enc" joint="right"/>
    <jointvel name="left_vel"  joint="left"/>
    <jointvel name="right_vel" joint="right"/>
  </sensor>
</mujoco>"""


def main():
    print(plant_xml())


if __name__ == "__main__":
    main()
