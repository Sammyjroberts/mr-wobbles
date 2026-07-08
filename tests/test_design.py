"""CI gates for the balancer design.

These are the release checks: they assert the invariants that make the robot flyable,
using the SAME evaluation harness (balancer.sim.evaluate) that report_stats.py prints to
the README. If a change to robot_params.py or the LQR cost drifts the design out of spec,
CI fails here before the gains ever reach hardware.

Nothing renders, so this runs headless on CI with no GL/display.
"""
import numpy as np
import pytest

from balancer.params import robot_params as rp
from balancer.sim.lqr_design import Q, R, compute_K, mask_phase1
from balancer.sim.balance import CONFIG, REALISTIC, design_gains
from balancer.sim import evaluate as ev
from balancer.paths import KC_REAL_PATH, KC_PHASE1_PATH


@pytest.fixture(scope="module")
def ideal():
    return ev.summary(CONFIG)


# ---- the problem is real, and we solve it -------------------------------------------
def test_open_loop_is_unstable():
    ol, _ = ev.poles()
    assert ol[0] > 1.0, "the upright equilibrium should be unstable (gravity runaway)"


def test_closed_loop_is_stable():
    _, cl = ev.poles()
    assert cl[0] < 1.0, f"all closed-loop poles must be inside the unit circle, got max {cl[0]:.4f}"


# ---- the gains on disk are the gains this code produces (release gate) ---------------
def test_flight_gains_match_golden():
    p = rp.assemble()
    K, _ = compute_K(p["L"], p["pole_mass"], p["cart_mass"], Q, R)
    K_golden = np.load(KC_REAL_PATH)
    np.testing.assert_allclose(
        K, K_golden, rtol=1e-4,
        err_msg="live gains differ from outputs/Kc_real.npy - params or Q/R changed? "
                "re-run `uv run balancer-design` and commit if the change was intended.")


def test_phase1_gains_match_golden():
    K_phase1 = mask_phase1(np.load(KC_REAL_PATH))
    np.testing.assert_allclose(K_phase1, np.load(KC_PHASE1_PATH), rtol=1e-4)


# ---- actuation stays within the motor (ideal design must not saturate) ---------------
def test_torque_headroom(ideal):
    assert ideal["peak_torque"] < 0.85 * ideal["stall"], (
        f"ideal control effort {ideal['peak_torque']:.3f} N*m should leave headroom "
        f"under the {ideal['stall']:.3f} N*m stall")


# ---- the encoder signal path (what the RP2040 will use) tracks truth -----------------
def test_encoder_tracks_truth():
    err = ev.encoder_tracking_error()
    assert err < 5.0, f"encoder-derived x drifted {err:.1f} mm from truth while balancing (sign/slip bug?)"


# ---- closed-loop behaviour: recovers a tilt, and stays upright under a shove ----------
def test_recovers_from_initial_tilt(ideal):
    assert ideal["pitch_recover_s"] < 1.0, "pitch should return within 1 deg quickly"


def test_survives_realistic_shove():
    s = ev.summary(REALISTIC)
    # thin margin under 4 ms latency is a documented finding; the gate is only that it
    # does not topple (a fall would blow well past 35 deg and never come back).
    assert s["shove_peak_tilt"] < 35.0, f"toppled under realistic conditions ({s['shove_peak_tilt']:.0f} deg)"


# ---- Phase 1 (IMU only) balances but wanders - the acceptance criterion for bring-up --
def test_phase1_balances_but_drifts():
    k1 = design_gains(phase=1)
    hist, _ = ev.rollout(k1, initial_deg=3.0)
    peak_tilt = hist[:, 1].max()
    drift = hist[-1, 2]
    assert peak_tilt < 10.0, f"Phase-1 should hold upright, peaked at {peak_tilt:.1f} deg"
    assert drift > 50.0, "Phase-1 has no position feedback, so it MUST drift - if it doesn't, the test is wrong"
