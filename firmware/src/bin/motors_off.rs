#![no_std]
#![no_main]

//! motors_off — safe state. Drives STBY LOW (TB6612 fully disabled) and all
//! channel A/B direction + PWM pins LOW, then idles. Nothing can move.
//! Flash this any time you need the driver guaranteed off.
//!
//! Run: cargo run --bin motors_off --release

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::block::ImageDef;
use embassy_rp::gpio::{Level, Output};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

#[link_section = ".start_block"]
#[used]
static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // STBY LOW = whole driver in standby (outputs high-Z, nothing drives).
    let _stby = Output::new(p.PIN_22, Level::Low);
    // All direction/PWM pins LOW too, for good measure.
    let _ain1 = Output::new(p.PIN_18, Level::Low);
    let _ain2 = Output::new(p.PIN_19, Level::Low);
    let _bin1 = Output::new(p.PIN_20, Level::Low);
    let _bin2 = Output::new(p.PIN_21, Level::Low);
    let _pwma = Output::new(p.PIN_16, Level::Low);
    let _pwmb = Output::new(p.PIN_17, Level::Low);

    info!("motors_off: STBY LOW, all pins LOW. Driver disabled.");
    loop {
        Timer::after(Duration::from_secs(5)).await;
        info!("motors_off: still disabled (safe)");
    }
}
