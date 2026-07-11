#![no_std]
#![no_main]

//! motor_diag — steady-state TB6612 diagnostic for a "motor won't move" bringup.
//!
//! Unlike motor_test (which cycles), this drives channel A FORWARD at 100% and
//! HOLDS it, so every control line sits at a stable DC level you can probe with
//! a multimeter. Expected readings (black probe on a common GND) while running:
//!
//!   VM   (TB6612 motor supply) .... ~12 V     <- your battery/supply + polarity
//!   VCC  (TB6612 logic supply) .... ~3.3 V     <- MUST be powered or nothing works
//!   STBY (GP22) ................... ~3.3 V
//!   AIN1 (GP18) ................... ~3.3 V
//!   AIN2 (GP19) ................... ~0 V
//!   PWMA (GP16) ................... ~3.3 V      (100% duty = held high)
//!   AO1 <-> AO2 (motor terminals) . ~11-12 V    <- this is what actually spins it
//!
//! Diagnosis:
//!   * VCC = 0            -> logic rail not connected. Wire TB6612 VCC to Pico 3V3.
//!   * a control pin = 0 AT THE TB6612 but 3.3 V at the Pico pin -> broken/missing wire.
//!   * all inputs correct, VM=12, VCC=3.3, but AO1-AO2 = 0 -> no common GND, or dead driver.
//!   * AO1-AO2 = 12 V but motor still still -> motor leads not making contact / dead motor.
//!
//! GROUND CHECK (do this with power OFF, meter in continuity/ohms):
//!   Pico GND <-> TB6612 GND must beep/read ~0 ohm. If not, that's the fault.
//!
//! Run: cargo run --bin motor_diag --release

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::block::ImageDef;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::pwm::{Config as PwmConfig, Pwm};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

#[link_section = ".start_block"]
#[used]
static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

const PWM_TOP: u16 = 7499; // ~20 kHz

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("motor_diag: holding channel A FORWARD @ 100% for steady-state probing");

    // Enable driver.
    let mut _stby = Output::new(p.PIN_22, Level::High); // STBY = HIGH -> enabled
    // Channel A direction: FORWARD = AIN1 high, AIN2 low.
    let mut _ain1 = Output::new(p.PIN_18, Level::High);
    let mut _ain2 = Output::new(p.PIN_19, Level::Low);

    // PWMA (GP16) held at 100% duty.
    let mut cfg = PwmConfig::default();
    cfg.top = PWM_TOP;
    cfg.compare_a = PWM_TOP + 1; // 100% -> output always high
    cfg.compare_b = 0;
    let _pwm = Pwm::new_output_ab(p.PWM_SLICE0, p.PIN_16, p.PIN_17, cfg);

    info!("STBY=GP22=HIGH  AIN1=GP18=HIGH  AIN2=GP19=LOW  PWMA=GP16=100%");
    info!("Probe now: VM~12V, VCC~3.3V, STBY/AIN1/PWMA~3.3V, AIN2~0V, motor terminals ~12V");
    info!("If motor terminals read 0V with inputs correct: check COMMON GROUND (Pico GND <-> TB6612 GND).");

    // Hold forever; heartbeat so you know it's alive.
    let mut n = 0u32;
    loop {
        Timer::after(Duration::from_secs(2)).await;
        n += 1;
        info!("still holding FORWARD @ 100% ({=u32}s)", n * 2);
    }
}
