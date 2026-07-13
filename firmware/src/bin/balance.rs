#![no_std]
#![no_main]

//! balance — Phase-1 (IMU-only) self-balancing controller for Mr. Wobbles.
//!
//! Control law (from src/balancer/sim/balance.py, gains from outputs/Kc_phase1.npy):
//!     state  = [x, pitch, x_dot, pitch_rate]   (SI: rad, rad/s)
//!     force  = -K . state                       (desired cart force, N)
//!     torque = force * wheel_radius             (N*m) -> PWM duty
//! Phase 1 has no encoders, so x and x_dot are 0 and only the pitch / pitch_rate
//! gains act. It BALANCES but WANDERS (no position hold) — expected until encoders.
//!
//! Pitch comes from the IMU via the mount config in imu_map.rs (fused with a
//! complementary filter). +pitch = nose forward, upright = 0.
//!
//! SAFETY / behavior:
//!   * Boots DISARMED (motors off, STBY low). Arms only after it's held within
//!     ARM_ANGLE of upright for ARM_HOLD. So: plug in 12V, stand it up -> it goes.
//!   * If |pitch| exceeds FALL_ANGLE it DISARMS (motors off) and re-arms when
//!     stood back up.
//!   * Output is capped at MAX_DUTY.
//!
//! >>> FIRST TEST HELD IN HAND <<< The motor-direction signs below are unverified.
//! If CONTROL_SIGN is wrong the robot drives itself OVER when armed. Hold it, let
//! it arm, tip it slightly forward, and confirm the wheels spin to drive FORWARD
//! (under the lean). If they drive backward, flip CONTROL_SIGN. If the two wheels
//! fight each other, flip one of MOTOR_A_SIGN / MOTOR_B_SIGN.
//!
//! Run: cargo run --bin balance --release

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::block::ImageDef;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::i2c::{self, I2c};
use embassy_rp::peripherals::I2C0;
use embassy_rp::pwm::{Config as PwmConfig, Pwm};
use embassy_time::{Duration, Ticker, Timer};
use embedded_hal_async::i2c::I2c as _;
use {defmt_rtt as _, panic_probe as _};

#[path = "../imu_map.rs"]
mod imu_map;

// Gantry telemetry: opt-in, fully compiled out without `--features telemetry`.
#[cfg(feature = "telemetry")]
#[path = "../telemetry.rs"]
mod telemetry;

#[link_section = ".start_block"]
#[used]
static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

bind_interrupts!(struct Irqs {
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
});

// Separate binding for the USB peripheral, only when telemetry is enabled.
#[cfg(feature = "telemetry")]
bind_interrupts!(struct UsbIrqs {
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<embassy_rp::peripherals::USB>;
});

// ===================== Controller (Phase 1, IMU-only) =====================
// From outputs/Kc_phase1.npy = [0, -19.443, 0, -1.412] on [x, pitch, x_dot, pitch_rate].
const K_PITCH: f32 = -19.443; // N per rad
const K_PITCH_RATE: f32 = -1.412; // N per (rad/s)
const WHEEL_RADIUS_M: f32 = 0.035;
const STALL_TORQUE_NM: f32 = 0.314; // maps commanded torque -> PWM fraction

// ===================== Signs — VERIFY ON THE BENCH (held) =====================
// Drive direction lives in the mount config (imu_map::DRIVE_SIGN).
const MOTOR_A_SIGN: f32 = 1.0; // flip if LEFT wheel rolls opposite the robot's forward
const MOTOR_B_SIGN: f32 = 1.0; // flip if RIGHT wheel rolls opposite

// ===================== Loop / safety / filter =====================
const LOOP_HZ: u64 = 500; // 2 ms, matches the sim timestep; IMU runs 833 Hz so samples are fresh
const DT: f32 = 1.0 / LOOP_HZ as f32;
const ALPHA: f32 = 0.99; // complementary filter, retuned for 500 Hz (~0.2 s time constant)
const ARM_ANGLE_DEG: f32 = 4.0; // within this of upright to arm
const ARM_HOLD_CYCLES: u32 = (LOOP_HZ as u32) / 2; // ...held ~0.5 s
const FALL_ANGLE_DEG: f32 = 30.0; // past this = fallen -> disarm
const MAX_DUTY: f32 = 0.30; // gentle cap while verifying the catch direction; raise toward 0.85 once stable
const DEADBAND_DUTY: f32 = 0.12; // feedforward past motor stiction so small corrections still move the wheels (tune to your motors)
const CONTROL_DEADZONE: f32 = 0.04; // below this duty = coast (ignore sensor noise near upright, don't buzz the wheels)
const DEG2RAD: f32 = core::f32::consts::PI / 180.0;

// PWM: slice 0 drives GP16 (A) + GP17 (B). TOP=7499 -> ~20 kHz.
const PWM_TOP: u16 = 7499;

// LSM6DSOX registers.
const ADDR: u8 = 0x6A;
const EXPECTED_WHO_AM_I: u8 = 0x6C;
const REG_WHO_AM_I: u8 = 0x0F;
const REG_CTRL1_XL: u8 = 0x10;
const REG_CTRL2_G: u8 = 0x11;
const REG_CTRL3_C: u8 = 0x12;
const REG_OUTX_L_G: u8 = 0x22;
const CTRL1_XL_833HZ_4G: u8 = 0x78; // 833 Hz ODR, +-4 g (fresh samples for the 500 Hz loop)
const CTRL2_G_833HZ_500DPS: u8 = 0x74; // 833 Hz ODR, +-500 dps
const CTRL3_C_BDU_IFINC: u8 = 0x44;
const ACC_SENS_G_PER_LSB: f32 = 0.122e-3;
const GYR_SENS_DPS_PER_LSB: f32 = 17.50e-3;

/// Set one TB6612 channel from a signed duty in [-1,1]; returns the PWM compare.
/// duty>0 => IN1 high / IN2 low (forward); duty<0 => reverse; sign picks direction.
fn drive(duty: f32, in1: &mut Output<'static>, in2: &mut Output<'static>) -> u16 {
    let mag = if duty.abs() > MAX_DUTY { MAX_DUTY } else { duty.abs() };
    if duty >= 0.0 {
        in1.set_high();
        in2.set_low();
    } else {
        in1.set_low();
        in2.set_high();
    }
    (mag * (PWM_TOP as f32 + 1.0)) as u16
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("balance: booting (Phase 1, IMU-only)");

    // --- Gantry telemetry: init pipeline + spawn the USB-CDC drain task (feature-gated) ---
    #[cfg(feature = "telemetry")]
    {
        telemetry::init();
        let usb_driver = embassy_rp::usb::Driver::new(p.USB, UsbIrqs);
        _spawner.must_spawn(telemetry::telemetry_usb_task(usb_driver));
        info!("telemetry: gantry-tlm up, USB-CDC streaming as \"gantry-tlm\"");
    }

    // --- Motors: start fully disabled (STBY low, all pins low, 0 duty) ---
    let mut stby = Output::new(p.PIN_22, Level::Low);
    let mut ain1 = Output::new(p.PIN_18, Level::Low);
    let mut ain2 = Output::new(p.PIN_19, Level::Low);
    let mut bin1 = Output::new(p.PIN_20, Level::Low);
    let mut bin2 = Output::new(p.PIN_21, Level::Low);
    let mut pwm_cfg = PwmConfig::default();
    pwm_cfg.top = PWM_TOP;
    pwm_cfg.compare_a = 0;
    pwm_cfg.compare_b = 0;
    let mut pwm = Pwm::new_output_ab(p.PWM_SLICE0, p.PIN_16, p.PIN_17, pwm_cfg.clone());

    // --- IMU on I2C0 (GP4/GP5), 400 kHz for the control-loop read rate ---
    let mut i2c_cfg = i2c::Config::default();
    i2c_cfg.frequency = 400_000;
    let mut i2c = I2c::new_async(p.I2C0, p.PIN_5, p.PIN_4, Irqs, i2c_cfg);
    loop {
        let mut who = [0u8; 1];
        match i2c.write_read(ADDR, &[REG_WHO_AM_I], &mut who).await {
            Ok(()) if who[0] == EXPECTED_WHO_AM_I => break,
            Ok(()) => error!("WHO_AM_I=0x{:02X} (want 0x{:02X})", who[0], EXPECTED_WHO_AM_I),
            Err(e) => error!("IMU not acking: {:?} (check wiring). retrying...", e),
        }
        Timer::after(Duration::from_secs(1)).await;
    }
    unwrap!(i2c.write(ADDR, &[REG_CTRL3_C, CTRL3_C_BDU_IFINC]).await);
    unwrap!(i2c.write(ADDR, &[REG_CTRL1_XL, CTRL1_XL_833HZ_4G]).await);
    unwrap!(i2c.write(ADDR, &[REG_CTRL2_G, CTRL2_G_833HZ_500DPS]).await);
    info!(
        "IMU up. gains: Kp={=f32} Kd={=f32}. signs: drive={=f32} A={=f32} B={=f32}. Hold UPRIGHT to arm.",
        K_PITCH, K_PITCH_RATE, imu_map::DRIVE_SIGN, MOTOR_A_SIGN, MOTOR_B_SIGN
    );

    // --- Seed the complementary filter from the accelerometer ---
    let (ax, ay, az, _, _, _) = read_all(&mut i2c).await;
    let mut pitch = imu_map::pitch_deg(ax, ay, az); // deg

    let mut armed = false;
    let mut upright_cycles: u32 = 0;
    let mut log_div: u32 = 0;
    // Telemetry decimator: the loop runs at 500 Hz; stream at ~50 Hz (every 10th cycle).
    #[cfg(feature = "telemetry")]
    let mut tlm_div: u32 = 0;
    let mut ticker = Ticker::every(Duration::from_hz(LOOP_HZ));

    loop {
        ticker.next().await;
        let (ax, ay, az, gx, gy, gz) = read_all(&mut i2c).await;

        // --- Pitch: complementary filter (deg), + gyro pitch rate (deg/s) ---
        let pitch_acc = imu_map::pitch_deg(ax, ay, az);
        let pitch_rate = imu_map::pitch_rate_dps(gx, gy, gz);
        pitch = ALPHA * (pitch + pitch_rate * DT) + (1.0 - ALPHA) * pitch_acc;

        // Telemetry mirrors of the controller outputs (stay 0 while disarmed). cfg-gated so
        // the plain build is byte-identical — these lines don't exist without the feature.
        #[cfg(feature = "telemetry")]
        let mut tlm_force = 0.0f32;
        #[cfg(feature = "telemetry")]
        let mut tlm_duty = 0.0f32;

        // --- Arm / disarm state machine ---
        let upright = pitch.abs() < ARM_ANGLE_DEG;
        let fallen = pitch.abs() > FALL_ANGLE_DEG;
        if !armed {
            upright_cycles = if upright { upright_cycles + 1 } else { 0 };
            if upright_cycles >= ARM_HOLD_CYCLES {
                armed = true;
                stby.set_high(); // enable the driver
                info!("ARMED — balancing. (pitch={=f32}deg)", pitch);
            }
        } else if fallen {
            armed = false;
            upright_cycles = 0;
            // Coast everything off.
            ain1.set_low();
            ain2.set_low();
            bin1.set_low();
            bin2.set_low();
            pwm_cfg.compare_a = 0;
            pwm_cfg.compare_b = 0;
            pwm.set_config(&pwm_cfg);
            stby.set_low(); // disable driver
            warn!("FALL detected (pitch={=f32}deg) — DISARMED. Stand it up to re-arm.", pitch);
        }

        // --- Control law (only when armed) ---
        if armed {
            let pitch_rad = pitch * DEG2RAD;
            let rate_rad = pitch_rate * DEG2RAD;
            // force = -K.state (Phase 1: x, x_dot = 0). torque = force * r.
            let force = -(K_PITCH * pitch_rad + K_PITCH_RATE * rate_rad); // N
            let torque = force * WHEEL_RADIUS_M; // N*m
            let mut duty = torque / STALL_TORQUE_NM; // -> PWM fraction
            duty *= imu_map::DRIVE_SIGN;
            // Near-upright deadzone: ignore tiny commands (mostly sensor noise) so the
            // wheels don't buzz back and forth around vertical. Real corrections (above
            // the deadzone) get a feedforward kick past gearmotor stiction.
            if duty.abs() < CONTROL_DEADZONE {
                duty = 0.0;
            } else {
                duty += duty.signum() * DEADBAND_DUTY;
            }
            if duty > MAX_DUTY {
                duty = MAX_DUTY;
            } else if duty < -MAX_DUTY {
                duty = -MAX_DUTY;
            }
            pwm_cfg.compare_a = drive(duty * MOTOR_A_SIGN, &mut ain1, &mut ain2);
            pwm_cfg.compare_b = drive(duty * MOTOR_B_SIGN, &mut bin1, &mut bin2);
            pwm.set_config(&pwm_cfg);

            // Capture the commanded force/duty for telemetry (send happens below).
            #[cfg(feature = "telemetry")]
            {
                tlm_force = force;
                tlm_duty = duty;
            }

            log_div += 1;
            if log_div >= LOOP_HZ as u32 / 20 {
                // ~20 Hz
                log_div = 0;
                info!("bal pitch={=f32}deg rate={=f32}dps duty={=f32}", pitch, pitch_rate, duty);
            }
        } else {
            // Disarmed: log slowly so you can see it waiting.
            log_div += 1;
            if log_div >= LOOP_HZ as u32 {
                log_div = 0;
                info!("disarmed, waiting for upright. pitch={=f32}deg", pitch);
            }
        }

        // --- Telemetry: stream at ~50 Hz (decimated from the 500 Hz loop) ---
        #[cfg(feature = "telemetry")]
        {
            tlm_div += 1;
            if tlm_div >= LOOP_HZ as u32 / 50 {
                tlm_div = 0;
                telemetry::send_imu(pitch, pitch_rate);
                telemetry::send_drive(tlm_duty * MOTOR_A_SIGN, tlm_duty * MOTOR_B_SIGN);
                telemetry::send_balance(tlm_force, armed);
            }
        }
    }
}

async fn read_all(i2c: &mut I2c<'static, I2C0, i2c::Async>) -> (f32, f32, f32, f32, f32, f32) {
    let mut raw = [0u8; 12];
    let _ = i2c.write_read(ADDR, &[REG_OUTX_L_G], &mut raw).await;
    let gx = i16::from_le_bytes([raw[0], raw[1]]) as f32 * GYR_SENS_DPS_PER_LSB;
    let gy = i16::from_le_bytes([raw[2], raw[3]]) as f32 * GYR_SENS_DPS_PER_LSB;
    let gz = i16::from_le_bytes([raw[4], raw[5]]) as f32 * GYR_SENS_DPS_PER_LSB;
    let ax = i16::from_le_bytes([raw[6], raw[7]]) as f32 * ACC_SENS_G_PER_LSB;
    let ay = i16::from_le_bytes([raw[8], raw[9]]) as f32 * ACC_SENS_G_PER_LSB;
    let az = i16::from_le_bytes([raw[10], raw[11]]) as f32 * ACC_SENS_G_PER_LSB;
    (ax, ay, az, gx, gy, gz)
}
