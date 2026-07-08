"""
lqr_design.py  -  computes K for the REAL robot, using params derived from the STL.

Pipeline: robot_params (STL + datasheets) -> linearized cart-pole model (A,B)
-> LQR cost (Q,R) -> Riccati -> K (four gains). Runs once, offline.
"""
import mujoco
import numpy as np
from scipy.linalg import solve_discrete_are

from balancer.params import robot_params as rp
from balancer.paths import KC_REAL_PATH

np.set_printoptions(precision=4, suppress=True)


def design_xml(L, pole_mass, cart_mass):
    h = max(L, 0.01)
    return f"""
<mujoco><option timestep="0.002" integrator="Euler" gravity="0 0 -9.81"/>
<worldbody>
  <body name="cart"><joint name="slide" type="slide" axis="1 0 0"/>
    <geom type="box" size="0.05 0.05 0.02" mass="{cart_mass}"/>
    <body name="pole"><joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom type="box" pos="0 0 {L}" size="0.01 0.01 {h}" mass="{pole_mass}"/>
    </body>
  </body>
</worldbody>
<actuator><motor joint="slide" gear="1"/></actuator></mujoco>"""


def compute_K(L, pole_mass, cart_mass, Q, R):
    # 1. BUILD the physics model from the XML description
    model = mujoco.MjModel.from_xml_string(design_xml(L, pole_mass, cart_mass))

    # 2. CREATE a data buffer and set the robot to its upright operating point
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)   # zero everything
    mujoco.mj_forward(model, data)     # compute physics at that state

    # 3. ALLOCATE empty A and B matrices of the right size
    n_states = 2 * model.nv            # nv=2 dofs -> state is [x, θ, ẋ, θ̇] = 4
    n_controls = model.nu              # one motor -> 1
    A = np.zeros((n_states, n_states)) # 4x4
    B = np.zeros((n_states, n_controls)) # 4x1

    # 4. LINEARIZE: fill A and B by finite differences (nudge, step, measure)
    mujoco.mjd_transitionFD(model, data, 1e-6, True, A, B, None, None)

    # 5. SOLVE the Riccati equation -> P, the "cost-to-go" matrix
    P = solve_discrete_are(A, B, Q, R)

    # 6. READ the optimal gains K off of P
    K = np.linalg.inv(B.T @ P @ B + R) @ (B.T @ P @ A)

    return K, np.linalg.eigvals(A)


# cost priorities: hate tilt most (500), position some (35), speeds a little; R = effort
Q = np.diag([35.0, 500.0, 2.0, 6.0]); R = np.array([[3.0]])


def main():
    p = rp.assemble()
    K_real, eig = compute_K(p["L"], p["pole_mass"], p["cart_mass"], Q, R)
    K_old, _    = compute_K(0.092, 0.75, 0.15, Q, R)   # the earlier guess, for contrast

    print(f"REAL robot:  L={p['L']*1000:.1f} mm  pole={p['pole_mass']:.3f} kg  cart={p['cart_mass']:.3f} kg")
    print(f"  eig(A) unstable mode: {max(abs(eig)):.4f}   (>1 = gravity runaway)")
    print(f"  K_real (cart force) = {np.round(K_real,3)}")
    tau = K_real.flatten() * p["wheel_r"]
    print(f"  K_real (wheel torque, tau=F*r) = {np.round(tau,4)}  N*m  [motor stall {p['stall']} N*m]")
    print()
    print(f"OLD guess (L=92mm, pole=0.75): K = {np.round(K_old,3)}")

    KC_REAL_PATH.parent.mkdir(exist_ok=True)
    np.save(KC_REAL_PATH, K_real)
    print(f"\nsaved {KC_REAL_PATH}")


if __name__ == "__main__":
    main()
