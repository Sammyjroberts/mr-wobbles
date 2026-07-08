"""Print the closed-loop performance numbers used in the README Results table.

Every number is derived live from the design via balancer.sim.evaluate - the same
harness the CI tests assert on - so the docs, the gates, and the code can't drift
apart. Uses only project deps:

    uv run python scripts/report_stats.py
"""
from balancer.sim import evaluate as ev
from balancer.sim.balance import CONFIG, REALISTIC

ideal = ev.summary(CONFIG)
real = ev.summary(REALISTIC)


def pct(u, stall):
    return f"{100 * u / stall:.0f}%"


print("open-loop  poles |z| = {:.3f} (unstable, gravity runaway)".format(ideal["ol_max"]))
print("closed-loop poles |z| = {}  (max {:.3f} < 1 -> stable)".format(
    [round(float(z), 3) for z in ideal["cl"]], ideal["cl_max"]))
print()
print("--- ideal sensors (encoder + IMU, Phase 2) ---")
print(f"  pitch recovery (3 deg tilt): within 1 deg in {ideal['pitch_recover_s']:.2f} s")
print(f"  position re-centering:       within +/-10 mm in {ideal['recenter_s']:.2f} s")
print(f"  2.2 N shove:                 peak tilt {ideal['shove_peak_tilt']:.1f} deg, "
      f"lunge {ideal['shove_lunge_mm']:.0f} mm, recovers")
print(f"  peak control effort:         {ideal['peak_torque']:.3f} N*m of {ideal['stall']:.3f} stall "
      f"({pct(ideal['peak_torque'], ideal['stall'])}, ~{100 - 100*ideal['peak_torque']/ideal['stall']:.0f}% headroom)")
print(f"  encoder vs true position:    tracks within {ideal['enc_err_mm']:.1f} mm while balancing")
print()
print("--- realistic: 0.02 rad/s gyro noise, 4 ms latency, PWM deadband ---")
print(f"  2.2 N shove:                 peak tilt {real['shove_peak_tilt']:.1f} deg "
      f"(vs {ideal['shove_peak_tilt']:.1f} ideal), peak torque {real['peak_torque']:.3f} N*m "
      f"({pct(real['peak_torque'], real['stall'])})")
print(f"  -> latency is the binding constraint: the low-CoM pendulum is twitchy, so a fast")
print(f"     control loop is a hard Phase-2 requirement, not a nicety.")
print()
print("--- Phase 1 (IMU only, no encoders) ---")
print(f"  balances but drifts: peak tilt {ideal['phase1_peak_tilt']:.1f} deg, "
      f"wanders {ideal['phase1_drift_mm']:.0f} mm in 9 s (~{ideal['phase1_drift_mm']/9:.0f} mm/s, no position feedback)")
