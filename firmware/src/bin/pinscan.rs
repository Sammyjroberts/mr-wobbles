#![no_std]
#![no_main]

//! pinscan — "where is my I2C device actually wired?" diagnostic.
//!
//! Configures every GPIO GP0..=GP22 as an input with the INTERNAL PULL-DOWN
//! engaged, then reports which pins read HIGH. A pin reads HIGH only if
//! something external is pulling it up — e.g. the Adafruit breakout's ~10k
//! SDA/SCL pull-ups to 3.3V. So the two pins that light up HIGH are exactly
//! where your SDA/SCL wires actually land (and it proves the board is powered).
//!
//! Expected for the IMU on the intended pins: GP4 and GP5 read HIGH.
//! If instead GP2 and GP3 read HIGH, your wires are one pair too high on the
//! header (physical pins 4/5 instead of 6/7).
//!
//! Run: cargo run --bin pinscan --release

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::block::ImageDef;
use embassy_rp::gpio::{Input, Pull};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

#[link_section = ".start_block"]
#[used]
static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("pinscan: reading GP0..GP22 with internal pull-down. HIGH = external pull-up present.");

    // One Input per pin, held for the life of the program so we can poll them.
    // ( erases the pin type so they all fit in one array.)
    let inputs: [(u8, Input<'static>); 23] = [
        (0, Input::new(p.PIN_0, Pull::Down)),
        (1, Input::new(p.PIN_1, Pull::Down)),
        (2, Input::new(p.PIN_2, Pull::Down)),
        (3, Input::new(p.PIN_3, Pull::Down)),
        (4, Input::new(p.PIN_4, Pull::Down)),
        (5, Input::new(p.PIN_5, Pull::Down)),
        (6, Input::new(p.PIN_6, Pull::Down)),
        (7, Input::new(p.PIN_7, Pull::Down)),
        (8, Input::new(p.PIN_8, Pull::Down)),
        (9, Input::new(p.PIN_9, Pull::Down)),
        (10, Input::new(p.PIN_10, Pull::Down)),
        (11, Input::new(p.PIN_11, Pull::Down)),
        (12, Input::new(p.PIN_12, Pull::Down)),
        (13, Input::new(p.PIN_13, Pull::Down)),
        (14, Input::new(p.PIN_14, Pull::Down)),
        (15, Input::new(p.PIN_15, Pull::Down)),
        (16, Input::new(p.PIN_16, Pull::Down)),
        (17, Input::new(p.PIN_17, Pull::Down)),
        (18, Input::new(p.PIN_18, Pull::Down)),
        (19, Input::new(p.PIN_19, Pull::Down)),
        (20, Input::new(p.PIN_20, Pull::Down)),
        (21, Input::new(p.PIN_21, Pull::Down)),
        (22, Input::new(p.PIN_22, Pull::Down)),
    ];

    loop {
        Timer::after(Duration::from_millis(5)).await; // let lines settle
        let mut any = false;
        for (n, inp) in &inputs {
            if inp.is_high() {
                info!("  GP{=u8} = HIGH  <-- external pull-up here (device wired to this pin)", *n);
                any = true;
            }
        }
        if !any {
            warn!("  no pins pulled high: breakout unpowered, no shared GND, or SDA/SCL not connected to any GPIO");
        }
        info!("--- scan complete (expect the two I2C lines here); rescanning in 1s ---");
        Timer::after(Duration::from_secs(1)).await;
    }
}
