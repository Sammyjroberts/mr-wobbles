//! Gantry telemetry integration for Mr. Wobbles.
//!
//! This whole module is compiled ONLY with `--features telemetry` (the `mod telemetry`
//! declaration in each binary is itself `#[cfg(feature = "telemetry")]`). With the feature
//! off, none of this — the gantry-tlm pipeline, the USB-CDC stack, the packet structs — is
//! linked, and the firmware is byte-for-byte the plain build.
//!
//! Shared by the balancing binary via `#[path = "../telemetry.rs"] mod telemetry;`, mirroring
//! how `imu_map.rs` is shared.
//!
//! Wire path: `send_*()` (called from the control loop) -> gantry-tlm ring buffer ->
//! `telemetry_usb_task` drains framed records and streams them out the USB-CDC endpoint.
//! The `gantry-serial-agent` on the host opens the COM port and decodes the `gantry.v0`
//! wire format. See ai/gantry/docs/WIRE.md.

use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, Config};
use static_cell::StaticCell;

use gantry_tlm as tlm;

// ---------------------------------------------------------------------------------------------
// Packets — each field name/unit mirrors exactly what balance.rs already computes.
// ---------------------------------------------------------------------------------------------

/// Attitude the balancer acts on: complementary-filtered pitch (deg) and the mount-mapped
/// pitch rate (deg/s). Named `pitch_rate_dps`, not `gyro_y_dps`, because balance.rs feeds the
/// value through `imu_map::pitch_rate_dps()` (robot-axle rate), not a raw gyro-Y read.
#[derive(tlm::Telemetry)]
pub struct Imu {
    #[tlm(unit = "deg")]
    pub pitch_deg: f32,
    #[tlm(unit = "dps")]
    pub pitch_rate_dps: f32,
}

/// The per-wheel motor command. These are the SIGNED duty fractions the loop hands to
/// `drive()` (`duty * MOTOR_A_SIGN` / `MOTOR_B_SIGN`), so unit is duty in [-MAX_DUTY, MAX_DUTY].
/// Left = TB6612 channel A, Right = channel B (per the sign comments in balance.rs).
#[derive(tlm::Telemetry)]
pub struct Drive {
    #[tlm(unit = "duty")]
    pub left_cmd: f32,
    #[tlm(unit = "duty")]
    pub right_cmd: f32,
}

/// Controller state: the commanded cart force (N, pre-torque, = `-K . state`) and whether the
/// arm/disarm state machine currently has the motors live. `force_n` is 0 while disarmed.
#[derive(tlm::Telemetry)]
pub struct Balance {
    #[tlm(unit = "N")]
    pub force_n: f32,
    pub armed: bool,
}

// ---------------------------------------------------------------------------------------------
// Send helpers — keep the call sites in balance.rs to one readable line each.
// ---------------------------------------------------------------------------------------------

#[inline]
pub fn send_imu(pitch_deg: f32, pitch_rate_dps: f32) {
    tlm::send(&Imu { pitch_deg, pitch_rate_dps });
}

#[inline]
pub fn send_drive(left_cmd: f32, right_cmd: f32) {
    tlm::send(&Drive { left_cmd, right_cmd });
}

#[inline]
pub fn send_balance(force_n: f32, armed: bool) {
    tlm::send(&Balance { force_n, armed });
}

// ---------------------------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------------------------

/// Monotonic clock for gantry-tlm timestamps (embassy tick units).
fn now_ticks() -> u64 {
    embassy_time::Instant::now().as_ticks()
}

/// Boot-time session id from the RP2350 ring oscillator's random bit. Sampled with a short
/// spin between reads so successive bits aren't correlated. The collector resets its per-stream
/// state whenever this changes, so a fresh value each boot is what we want.
fn rosc_session() -> u32 {
    let rosc = embassy_rp::pac::ROSC;
    let mut acc: u32 = 0;
    for _ in 0..32 {
        acc = (acc << 1) | (rosc.randombit().read().randombit() as u32);
        cortex_m::asm::delay(64); // let the ROSC advance between samples
    }
    if acc == 0 {
        0xA5A5_A5A5
    } else {
        acc
    }
}

/// Install the global telemetry pipeline. Call once at boot, before spawning the USB task.
/// ~8 KB ring lives in `.bss` (sized here, owned by the SDK thereafter).
pub fn init() {
    let session = rosc_session();
    tlm::init!(
        bytes = 8192,
        clock = now_ticks,
        tick_hz = embassy_time::TICK_HZ,
        device_id = "mr-wobbles",
        session = session,
    );
}

// ---------------------------------------------------------------------------------------------
// USB-CDC transport task
// ---------------------------------------------------------------------------------------------

/// Max USB-CDC bulk packet (full-speed): 64 bytes.
const USB_PACKET: usize = 64;

/// Drain gantry-tlm's ring and stream the framed wire records out a USB-CDC (virtual COM) port.
///
/// Runs the USB device stack and the drain loop concurrently. When no host has the port open
/// (`wait_connection` — DTR de-asserted), we simply don't drain; the SDK ring drops oldest
/// records on its own, so there is no unbounded buildup and no blocking of the control loop.
#[embassy_executor::task]
pub async fn telemetry_usb_task(driver: Driver<'static, USB>) {
    // USB device identity. 0x16c0/0x27dd is the pid.codes generic test VID/PID (fine for
    // bring-up; not for a shipping product). Product string is how the agent finds the port.
    let mut config = Config::new(0x16c0, 0x27dd);
    config.manufacturer = Some("mr-wobbles");
    config.product = Some("gantry-tlm");
    config.serial_number = Some("wobbles-0001");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Builder scratch + CDC state, promoted to 'static from .bss via StaticCell.
    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 0]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static STATE: StaticCell<State> = StaticCell::new();

    let config_desc = CONFIG_DESC.init([0; 256]);
    let bos_desc = BOS_DESC.init([0; 256]);
    let msos_desc = MSOS_DESC.init([0; 0]);
    let control_buf = CONTROL_BUF.init([0; 64]);
    let state = STATE.init(State::new());

    let mut builder = Builder::new(driver, config, config_desc, bos_desc, msos_desc, control_buf);
    let mut class = CdcAcmClass::new(&mut builder, state, USB_PACKET as u16);
    let mut usb = builder.build();

    // Run the USB stack.
    let usb_fut = usb.run();

    // Drain -> write loop.
    let drain_fut = async {
        let mut buf = [0u8; 256];
        loop {
            // Block until a host opens the port (asserts DTR); no draining when nobody listens.
            class.wait_connection().await;
            loop {
                let n = tlm::drain(&mut buf);
                if n > 0 {
                    // Emit in <=64-byte USB packets. Bail to reconnect-wait on any endpoint
                    // error (host closed the port); records still queued are handled by the
                    // ring's drop-oldest, so we never block the sender.
                    let mut off = 0;
                    let mut ok = true;
                    while off < n {
                        let end = core::cmp::min(off + USB_PACKET, n);
                        if class.write_packet(&buf[off..end]).await.is_err() {
                            ok = false;
                            break;
                        }
                        off = end;
                    }
                    if !ok {
                        break;
                    }
                }
                Timer::after(Duration::from_millis(10)).await;
            }
        }
    };

    embassy_futures::join::join(usb_fut, drain_fut).await;
}
