#![no_std]
#![no_main]

//! imu_pitch_test — verify the IMU reports pitch correctly for the balance loop,
//! using the mount config in `imu_map.rs`.
//!
//! Controller convention (src/balancer/sim/plant.py):
//!     +pitch = nose tips FORWARD (+x); rotation about the wheel axle; upright = 0.
//!
//! At ~10 Hz it logs raw accel/gyro (so you can re-derive the mapping after a
//! remount) plus the ROBOT-FRAME pitch and pitch_rate from imu_map. To verify:
//!   * Hold UPRIGHT   -> pitch ~= 0  (adjust imu_map::PITCH_TRIM_DEG to trim).
//!   * Tip FORWARD    -> pitch goes POSITIVE, and pitch_rate is POSITIVE while
//!                       it's moving forward. If either is backwards, flip the
//!                       matching sign in imu_map.rs.
//!
//! Run: cargo run --bin imu_pitch_test --release

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::block::ImageDef;
use embassy_rp::i2c::{self, I2c};
use embassy_rp::peripherals::I2C0;
use embassy_time::{Duration, Ticker, Timer};
use embedded_hal_async::i2c::I2c as _;
use {defmt_rtt as _, panic_probe as _};

#[path = "../imu_map.rs"]
mod imu_map;

#[link_section = ".start_block"]
#[used]
static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

bind_interrupts!(struct Irqs {
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
});

// --- LSM6DSOX registers (same as imu_test) ---
const ADDR: u8 = 0x6A;
const EXPECTED_WHO_AM_I: u8 = 0x6C;
const REG_WHO_AM_I: u8 = 0x0F;
const REG_CTRL1_XL: u8 = 0x10;
const REG_CTRL2_G: u8 = 0x11;
const REG_CTRL3_C: u8 = 0x12;
const REG_OUTX_L_G: u8 = 0x22;
const CTRL1_XL_104HZ_4G: u8 = 0x48;
const CTRL2_G_104HZ_500DPS: u8 = 0x44;
const CTRL3_C_BDU_IFINC: u8 = 0x44;
const ACC_SENS_G_PER_LSB: f32 = 0.122e-3;
const GYR_SENS_DPS_PER_LSB: f32 = 17.50e-3;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("imu_pitch_test: booting");
    let mut i2c = I2c::new_async(p.I2C0, p.PIN_5, p.PIN_4, Irqs, i2c::Config::default());

    // WHO_AM_I retry.
    loop {
        let mut who = [0u8; 1];
        match i2c.write_read(ADDR, &[REG_WHO_AM_I], &mut who).await {
            Ok(()) if who[0] == EXPECTED_WHO_AM_I => {
                info!("WHO_AM_I=0x{:02X} -> PASS", who[0]);
                break;
            }
            Ok(()) => error!("WHO_AM_I=0x{:02X} (want 0x{:02X})", who[0], EXPECTED_WHO_AM_I),
            Err(e) => error!("WHO_AM_I read failed: {:?} (check wiring). retrying...", e),
        }
        Timer::after(Duration::from_secs(1)).await;
    }
    unwrap!(i2c.write(ADDR, &[REG_CTRL3_C, CTRL3_C_BDU_IFINC]).await);
    unwrap!(i2c.write(ADDR, &[REG_CTRL1_XL, CTRL1_XL_104HZ_4G]).await);
    unwrap!(i2c.write(ADDR, &[REG_CTRL2_G, CTRL2_G_104HZ_500DPS]).await);
    info!("configured. Hold UPRIGHT (pitch~0), then tip FORWARD (pitch should go +, pitch_rate + while moving).");

    // Report loop at ~10 Hz.
    let mut ticker = Ticker::every(Duration::from_millis(100));
    loop {
        ticker.next().await;
        let (ax, ay, az, gx, gy, gz) = read_all(&mut i2c).await;
        let pitch = imu_map::pitch_deg(ax, ay, az);
        let pitch_rate = imu_map::pitch_rate_dps(gx, gy, gz);
        info!(
            "raw acc=({=f32},{=f32},{=f32})g gyr=({=f32},{=f32},{=f32})dps | robot pitch={=f32}deg pitch_rate={=f32}dps",
            ax, ay, az, gx, gy, gz, pitch, pitch_rate
        );
    }
}

/// Burst-read gyro + accel, return (ax,ay,az [g], gx,gy,gz [dps]).
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
