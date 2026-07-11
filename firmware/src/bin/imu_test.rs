#![no_std]
#![no_main]

//! imu_test — LSM6DSOX bring-up over async I2C on the Pico 2 W (RP2350).
//!
//! What it does:
//!   1. Brings up I2C0 on SDA=GP4 / SCL=GP5 (async, interrupt-driven).
//!   2. Reads WHO_AM_I and logs PASS/FAIL vs the expected device id (0x6C).
//!      -> Confirm this says PASS before trusting anything below.
//!   3. Configures accel (+/-4 g) and gyro (+/-500 dps) at 104 Hz ODR.
//!   4. At ~50 Hz: reads accel + gyro, computes pitch theta from the accel
//!      (atan2), pitch rate theta_dot from the gyro, fuses them with a
//!      complementary filter, and logs everything so you can watch theta
//!      track the board as you physically tilt it.
//!
//! Run: cargo run --bin imu_test --release

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::block::ImageDef;
use embassy_rp::gpio::{Input, Pull};
use embassy_rp::i2c::{self, I2c};
use embassy_rp::peripherals::I2C0;
use embassy_rp::bind_interrupts;
use embassy_time::{Duration, Ticker, Timer};
use embedded_hal_async::i2c::I2c as _; // brings the async write / write_read methods into scope
#[path = "../imu_map.rs"]
mod imu_map;
use {defmt_rtt as _, panic_probe as _};

// RP2350 requires this image-definition block at the very start of flash.
#[link_section = ".start_block"]
#[used]
static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

// Wire the I2C0 interrupt to embassy-rp's async handler.
bind_interrupts!(struct Irqs {
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
});

// ---------------------------------------------------------------------------
// LSM6DSOX register map (only what we need)
// ---------------------------------------------------------------------------
const LSM6DSOX_ADDR: u8 = 0x6A; // 7-bit address (SDO/SA0 tied low). 0x6B if pulled high.
const EXPECTED_WHO_AM_I: u8 = 0x6C; // fixed device id for the LSM6DSOX

const REG_WHO_AM_I: u8 = 0x0F;
const REG_CTRL1_XL: u8 = 0x10; // accel: ODR | full-scale
const REG_CTRL2_G: u8 = 0x11; // gyro:  ODR | full-scale
const REG_CTRL3_C: u8 = 0x12; // control: BDU, auto-increment, etc.
const REG_OUTX_L_G: u8 = 0x22; // first of 12 consecutive output bytes (gyro XYZ, then accel XYZ)

// CTRL1_XL = 0b0100_10_0_0
//   [7:4]=0100 -> 104 Hz ODR
//   [3:2]=10   -> +/-4 g full scale
const CTRL1_XL_104HZ_4G: u8 = 0x48;
// CTRL2_G  = 0b0100_01_0_0
//   [7:4]=0100 -> 104 Hz ODR
//   [3:2]=01   -> +/-500 dps full scale
const CTRL2_G_104HZ_500DPS: u8 = 0x44;
// CTRL3_C  = BDU (bit6) + IF_INC (bit2). BDU keeps the low/high bytes of a
// sample consistent; IF_INC lets us burst-read across registers.
const CTRL3_C_BDU_IFINC: u8 = 0x44;

// Sensitivities (from the datasheet) for the ranges selected above.
const ACC_SENS_G_PER_LSB: f32 = 0.122e-3; // +/-4 g  -> 0.122 mg / LSB
const GYR_SENS_DPS_PER_LSB: f32 = 17.50e-3; // +/-500 dps -> 17.50 mdps / LSB

// ---------------------------------------------------------------------------
// Complementary filter tuning
// ---------------------------------------------------------------------------
const LOOP_HZ: u64 = 50;
const DT: f32 = 1.0 / LOOP_HZ as f32; // 0.02 s

// ALPHA weights the gyro (fast, drifts) against the accel (noisy, absolute).
// 0.98 => trust the integrated gyro for the short term, let the accel slowly
// pull out long-term drift. Raise toward 1.0 for smoother/slower correction.
const ALPHA: f32 = 0.98;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("imu_test: booting");

    // --- 0. Line-sense diagnostic (before we touch the I2C peripheral) ---
    // Read SDA/SCL as inputs with the INTERNAL PULL-DOWN engaged. The Adafruit
    // breakout has ~10k pull-ups to 3.3V; the internal pull-down is ~50k, so:
    //   line reads HIGH -> external pull-up wins  => breakout powered AND wired
    //   line reads LOW  -> nothing pulling up     => that wire is floating/dead
    // This isolates a wiring/power fault to the exact line before any I2C traffic.
    let mut sda = p.PIN_4;
    let mut scl = p.PIN_5;
    {
        let sda_in = Input::new(sda.reborrow(), Pull::Down);
        let scl_in = Input::new(scl.reborrow(), Pull::Down);
        Timer::after(Duration::from_millis(2)).await; // let the lines settle
        let sda_hi = sda_in.is_high();
        let scl_hi = scl_in.is_high();
        info!(
            "line sense: SDA(GP4)={=bool} SCL(GP5)={=bool}  (High = pull-up present => powered+wired; Low = that line is dead)",
            sda_hi, scl_hi
        );
        if !sda_hi || !scl_hi {
            warn!("  at least one I2C line is LOW: check that wire + shared GND + breakout power");
        }
    }

    // Async I2C on GP4 (SDA) / GP5 (SCL). Default config = 100 kHz standard mode,
    // which is plenty for 104 Hz sampling. Bump i2c::Config.frequency if you want.
    let mut i2c = I2c::new_async(p.I2C0, scl, sda, Irqs, i2c::Config::default());

    // --- 1. Find the sensor: scan + retry until WHO_AM_I passes ----------
    // We loop instead of panicking so you can fix wiring/power on the bench and
    // watch it flip to PASS with no reflash. Each attempt first scans the whole
    // 7-bit address space so you can see exactly what (if anything) is present.
    loop {
        // --- I2C bus scan (0x08..=0x77) ---
        // A 1-byte read that ACKs means a device lives at that address.
        info!("scanning I2C bus (SDA=GP4, SCL=GP5)...");
        let mut found = 0u32;
        for addr in 0x08u8..=0x77 {
            let mut b = [0u8; 1];
            if i2c.read(addr, &mut b).await.is_ok() {
                info!("  device ACKed at 0x{:02X}", addr);
                found += 1;
            }
        }
        if found == 0 {
            warn!("  no I2C devices responded on the bus");
        }

        // --- WHO_AM_I check ---
        let mut who = [0u8; 1];
        match i2c.write_read(LSM6DSOX_ADDR, &[REG_WHO_AM_I], &mut who).await {
            Ok(()) if who[0] == EXPECTED_WHO_AM_I => {
                info!("WHO_AM_I = 0x{:02X} @ 0x{:02X} -> PASS (LSM6DSOX responding)", who[0], LSM6DSOX_ADDR);
                break;
            }
            Ok(()) => error!(
                "WHO_AM_I = 0x{:02X} -> FAIL (expected 0x{:02X}); something is at 0x{:02X} but it isn't an LSM6DSOX",
                who[0], EXPECTED_WHO_AM_I, LSM6DSOX_ADDR
            ),
            // NoAcknowledge here => nothing at 0x6A: SDA/SCL swapped, no 3.3V to
            // the breakout, missing pull-ups, or the chip is at 0x6B (SDO/SA0 high).
            Err(e) => error!(
                "WHO_AM_I read failed at 0x{:02X}: {:?} -> check power/wiring (try 0x6B?). Retrying in 1s...",
                LSM6DSOX_ADDR, e
            ),
        }
        Timer::after(Duration::from_secs(1)).await;
    }

    // --- 2. Configure the sensor ----------------------------------------
    // Safe to unwrap now that we've confirmed the bus works.
    unwrap!(i2c.write(LSM6DSOX_ADDR, &[REG_CTRL3_C, CTRL3_C_BDU_IFINC]).await);
    unwrap!(i2c.write(LSM6DSOX_ADDR, &[REG_CTRL1_XL, CTRL1_XL_104HZ_4G]).await);
    unwrap!(i2c.write(LSM6DSOX_ADDR, &[REG_CTRL2_G, CTRL2_G_104HZ_500DPS]).await);
    info!("configured: accel +/-4g @104Hz, gyro +/-500dps @104Hz");

    // --- 3. Sample loop at ~50 Hz ---------------------------------------
    let mut ticker = Ticker::every(Duration::from_hz(LOOP_HZ));
    let mut pitch: f32 = 0.0; // fused pitch estimate (deg)
    let mut initialized = false;

    loop {
        ticker.next().await;

        // Burst-read 12 bytes starting at OUTX_L_G:
        //   [0..6)  = gyro  X,Y,Z (int16 LE)
        //   [6..12) = accel X,Y,Z (int16 LE)
        let mut raw = [0u8; 12];
        if let Err(e) = i2c
            .write_read(LSM6DSOX_ADDR, &[REG_OUTX_L_G], &mut raw)
            .await
        {
            warn!("sample read failed: {:?}", e);
            continue;
        }

        let gx = i16::from_le_bytes([raw[0], raw[1]]);
        let gy = i16::from_le_bytes([raw[2], raw[3]]);
        let gz = i16::from_le_bytes([raw[4], raw[5]]);
        let ax = i16::from_le_bytes([raw[6], raw[7]]);
        let ay = i16::from_le_bytes([raw[8], raw[9]]);
        let az = i16::from_le_bytes([raw[10], raw[11]]);

        // Convert to physical units.
        let ax_g = ax as f32 * ACC_SENS_G_PER_LSB;
        let ay_g = ay as f32 * ACC_SENS_G_PER_LSB;
        let az_g = az as f32 * ACC_SENS_G_PER_LSB;
        let gx_dps = gx as f32 * GYR_SENS_DPS_PER_LSB;
        let gy_dps = gy as f32 * GYR_SENS_DPS_PER_LSB;
        let gz_dps = gz as f32 * GYR_SENS_DPS_PER_LSB;

        // --- Pitch from the accelerometer (absolute, but noisy) ---------
        //
        // Robot-frame pitch comes from the mount config in imu_map.rs (verified
        // via imu_pitch_test): +pitch = nose forward, upright ~= 0. To re-map
        // after a remount, edit imu_map.rs — nothing here changes.
        let pitch_acc = imu_map::pitch_deg(ax_g, ay_g, az_g);

        // --- Pitch rate from the gyro (fast, but drifts) ----------------
        // Gyro about the wheel axle, same sign convention as pitch (imu_map.rs).
        let pitch_rate = imu_map::pitch_rate_dps(gx_dps, gy_dps, gz_dps);

        // --- Complementary filter ---------------------------------------
        // Integrate the gyro for the fast path, and blend in the accel angle to
        // cancel long-term drift:
        //     theta = ALPHA*(theta + theta_dot*dt) + (1-ALPHA)*theta_acc
        if !initialized {
            // Seed from the accel so we don't spend seconds converging at boot.
            pitch = pitch_acc;
            initialized = true;
        } else {
            pitch = ALPHA * (pitch + pitch_rate * DT) + (1.0 - ALPHA) * pitch_acc;
        }

        info!(
            "acc[g] x={=f32} y={=f32} z={=f32} | gyr[dps] x={=f32} y={=f32} z={=f32} | theta_acc={=f32} theta={=f32}deg theta_dot={=f32}dps",
            ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, pitch_acc, pitch, pitch_rate
        );
    }
}
