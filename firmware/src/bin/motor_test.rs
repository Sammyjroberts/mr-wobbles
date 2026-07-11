#![no_std]
#![no_main]

//! motor_test — TB6612 bring-up on the Pico 2 W (RP2350).
//!
//! Drives channel A (left motor) through a repeating sequence so you can
//! confirm one motor spins and that "forward" is really forward before you
//! enable the second motor:
//!
//!     forward 30% (1s) -> coast (1s) -> reverse 30% (1s) -> coast (1s) -> repeat
//!
//! A ready-to-uncomment block does the same for channel B (right motor).
//!
//! TB6612 truth table (per channel, with STBY = HIGH and PWM applied):
//!     IN1  IN2   result
//!      H    L    forward  (motor speed set by PWM duty)
//!      L    H    reverse
//!      L    L    coast    (outputs high-Z, motor free-wheels)
//!      H    H    brake    (both outputs low, short-brake) -- unused here
//! STBY = LOW puts the whole driver in standby (both channels off).
//!
//! Pin map:
//!   Left / A:  PWMA=GP16  AIN1=GP18  AIN2=GP19
//!   Right / B: PWMB=GP17  BIN1=GP20  BIN2=GP21
//!   STBY=GP22 (drive HIGH to enable the driver)
//!
//! Note: on the RP2350, GP16 and GP17 are the two channels (A and B) of the
//! *same* PWM slice (slice 8), so a single Pwm drives both PWMA and PWMB.
//!
//! Run: cargo run --bin motor_test --release

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::block::ImageDef;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::pwm::{Config as PwmConfig, Pwm};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

// RP2350 requires this image-definition block at the very start of flash.
#[link_section = ".start_block"]
#[used]
static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

// PWM period. sys_clk = 150 MHz, divider = 1 (default), so the PWM frequency is
// 150 MHz / (TOP + 1). TOP = 7499 -> 20 kHz (above audible, easy on the TB6612).
// A "duty" value is a compare count in 0..=TOP+1; TOP+1 == 100%.
const PWM_TOP: u16 = 7499;

/// Convert a duty percentage (0..=100) into a PWM compare count.
fn duty(pct: f32) -> u16 {
    let c = (pct / 100.0) * (PWM_TOP as f32 + 1.0);
    c as u16
}

const DUTY_PCT: f32 = 30.0; // test speed
const PHASE: Duration = Duration::from_secs(1); // time per phase

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("motor_test: booting");

    // --- STBY: enable the whole driver ----------------------------------
    // Start LOW (driver disabled) then raise HIGH once everything is set up.
    let mut stby = Output::new(p.PIN_22, Level::Low);

    // --- Channel A (left) direction pins --------------------------------
    let mut ain1 = Output::new(p.PIN_18, Level::Low);
    let mut ain2 = Output::new(p.PIN_19, Level::Low);

    // --- PWM slice 0 drives both PWMA (GP16, chan A) and PWMB (GP17, chan B) ---
    // (embassy-rp names slices 0..7; GP16/GP17 fall on slice 0, channels A/B.)
    let mut pwm_cfg = PwmConfig::default();
    pwm_cfg.top = PWM_TOP;
    pwm_cfg.compare_a = 0; // GP16 / PWMA duty, start at 0
    pwm_cfg.compare_b = 0; // GP17 / PWMB duty, start at 0
    let mut pwm = Pwm::new_output_ab(p.PWM_SLICE0, p.PIN_16, p.PIN_17, pwm_cfg.clone());

    // ====================================================================
    // CHANNEL B (right) — ENABLED.
    let mut bin1 = Output::new(p.PIN_20, Level::Low);
    let mut bin2 = Output::new(p.PIN_21, Level::Low);
    // ====================================================================

    // Enable the driver.
    stby.set_high();
    info!("STBY high: driver enabled. Looping A+B: fwd -> coast -> rev -> coast");

    loop {
        // ---- FORWARD ----------------------------------------------------
        info!("A+B: FORWARD {}%", DUTY_PCT as u32);
        ain1.set_high();
        ain2.set_low();
        pwm_cfg.compare_a = duty(DUTY_PCT);
        // CHANNEL B forward:
        bin1.set_high();
        bin2.set_low();
        pwm_cfg.compare_b = duty(DUTY_PCT);
        pwm.set_config(&pwm_cfg);
        Timer::after(PHASE).await;

        // ---- COAST ------------------------------------------------------
        info!("A+B: COAST");
        ain1.set_low();
        ain2.set_low();
        pwm_cfg.compare_a = 0;
        // CHANNEL B coast:
        bin1.set_low();
        bin2.set_low();
        pwm_cfg.compare_b = 0;
        pwm.set_config(&pwm_cfg);
        Timer::after(PHASE).await;

        // ---- REVERSE ----------------------------------------------------
        info!("A+B: REVERSE {}%", DUTY_PCT as u32);
        ain1.set_low();
        ain2.set_high();
        pwm_cfg.compare_a = duty(DUTY_PCT);
        // CHANNEL B reverse:
        bin1.set_low();
        bin2.set_high();
        pwm_cfg.compare_b = duty(DUTY_PCT);
        pwm.set_config(&pwm_cfg);
        Timer::after(PHASE).await;

        // ---- COAST ------------------------------------------------------
        info!("A+B: COAST");
        ain1.set_low();
        ain2.set_low();
        pwm_cfg.compare_a = 0;
        // CHANNEL B coast:
        bin1.set_low();
        bin2.set_low();
        pwm_cfg.compare_b = 0;
        pwm.set_config(&pwm_cfg);
        Timer::after(PHASE).await;
    }
}
