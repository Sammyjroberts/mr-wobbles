//! IMU mount configuration — maps the RAW IMU axes to the ROBOT frame.
//!
//! Shared by the IMU binaries via `#[path = "../imu_map.rs"] mod imu_map;`.
//!
//! Robot frame (matches src/balancer/sim/plant.py):
//!     +X_robot = FORWARD  (drive direction / the way it falls)
//!     +Y_robot = wheel AXLE (the pitch rotation axis)
//!     +Z_robot = UP
//!     pitch = rotation about Y;  +pitch = nose tips FORWARD;  upright = 0.
//!
//! ============================ HOW TO RECONFIGURE ============================
//! If you remount the IMU, run `imu_pitch_test`, then:
//!   1. Hold upright  -> note which raw axis reads ~±1g  => that's UP.
//!   2. Tip FORWARD   -> note which raw axis gains the tilt => that's FORWARD;
//!                        the axis that stays ~0 is the AXLE.
//!   3. Set the three ROBOT_* lines below to that (axis, sign). Pick signs so
//!      pitch reads ~0 upright and goes POSITIVE when tipped forward.
//!   4. Re-run imu_pitch_test to confirm pitch(+forward) and pitch_rate agree.
//! ===========================================================================

use libm::atan2f;

const RAD2DEG: f32 = 180.0 / core::f32::consts::PI;

#[derive(Clone, Copy)]
pub enum ImuAxis {
    X,
    Y,
    Z,
}

/// A robot-frame axis expressed as (which raw IMU axis, sign).
#[derive(Clone, Copy)]
pub struct MappedAxis {
    pub src: ImuAxis,
    pub sign: f32,
}

// ======================= CURRENT MOUNT (edit on remount) =======================
// Verified 2026-07-10: IMU X points DOWN when upright; tipping FORWARD moves
// gravity into IMU Z; IMU Y is the wheel axle. So:
//   robot FORWARD =  -IMU_Z     (tip forward -> az goes negative -> want +fwd)
//   robot UP      =  -IMU_X     (upright ax=-1g -> want +up)
//   robot AXLE    =   IMU_Y     (pitch-rate gyro axis)
// AXLE sign CORRECTED 2026-07-12: the gyro rate was sign-inverted vs the true
// angular velocity (measured corr -0.66 with d(raw pitch)/dt on live telemetry).
// That broke the complementary filter (estimate diverged opposite to reality)
// AND made the D-term anti-damping — the root cause of the limit cycle. Flipped
// -1.0 -> +1.0; after the fix the robot self-balances. (FWD/UP are accel-only.)
pub const ROBOT_FWD: MappedAxis = MappedAxis { src: ImuAxis::Z, sign: -1.0 };
pub const ROBOT_UP: MappedAxis = MappedAxis { src: ImuAxis::X, sign: -1.0 };
pub const ROBOT_AXLE: MappedAxis = MappedAxis { src: ImuAxis::Y, sign: 1.0 };

/// Fine trim (deg): subtracted from pitch so a *true* upright reads exactly 0.
/// Set this to whatever `pitch` shows when you hold the robot dead upright.
pub const PITCH_TRIM_DEG: f32 = 0.0;

/// Motor drive direction relative to the robot frame.
/// A positive controller command must roll the base FORWARD (toward a forward
/// lean, to catch the fall). Verified 2026-07-10: with +1.0 the base drives AWAY
/// from the lean (supported test: wheels accelerate a forward tilt), so the
/// catching sign is -1.0. (The earlier -1.0 "yeet" was correct direction but too
/// much authority — now tamed by a low MAX_DUTY, not a sign flip.)
pub const DRIVE_SIGN: f32 = -1.0;
// ===============================================================================

/// Select one raw component (x/y/z) and apply the mapped sign.
#[inline]
pub fn pick(a: MappedAxis, x: f32, y: f32, z: f32) -> f32 {
    let v = match a.src {
        ImuAxis::X => x,
        ImuAxis::Y => y,
        ImuAxis::Z => z,
    };
    a.sign * v
}

/// Robot-frame (forward, up) accel components from raw IMU accel.
#[inline]
pub fn accel_fwd_up(ax: f32, ay: f32, az: f32) -> (f32, f32) {
    (pick(ROBOT_FWD, ax, ay, az), pick(ROBOT_UP, ax, ay, az))
}

/// Pitch in degrees from the accel vector. +pitch = nose forward; upright ~= 0.
#[inline]
pub fn pitch_deg(ax: f32, ay: f32, az: f32) -> f32 {
    let (fwd, up) = accel_fwd_up(ax, ay, az);
    atan2f(fwd, up) * RAD2DEG - PITCH_TRIM_DEG
}

/// Pitch rate in deg/s from raw gyro (rotation about the wheel axle).
/// Sign matches pitch: tipping forward should give a POSITIVE rate.
#[inline]
pub fn pitch_rate_dps(gx: f32, gy: f32, gz: f32) -> f32 {
    pick(ROBOT_AXLE, gx, gy, gz)
}
