#![allow(unsafe_code)]

use std::sync::atomic::{AtomicBool, Ordering};

use crate::buzz::BuzzOsc;
use crate::capture::{Capture, PendingStart, PendingStop};
use crate::curves::AxisRing;
use crate::dynamics::DynamicsModel;
use crate::ffi;
use crate::mailbox::MailboxWorker;
use crate::scale::CountMap;
use crate::sensorless::SensorlessBank;
use crate::server::FrameServer;
use crate::stream_halt::StreamHalt;
use crate::torque::TorqueGate;

mod bringup;
mod commands;
mod cycle;

pub use bringup::bringup;

static SIGTERM_RECEIVED: AtomicBool = AtomicBool::new(false);

pub struct EndpointCtx {
    server: FrameServer,

    num_slaves: usize,
    counts_per_mm: Vec<f64>,
    invert: Vec<bool>,
    cmd_counts_per_mm: Vec<f64>,
    rotation_distance: Vec<f64>,
    slave_axes: Vec<u8>,
    velocity_ff: Vec<bool>,
    torque_clamp_tenths: Vec<i16>,
    ff_lead_ns: Vec<u64>,
    jump_log_counts: Vec<i64>,
    cycle_ns: i64,
    telemetry_period: u64,
    dynamics: Option<DynamicsModel>,
    run_limits: Vec<(u32, u16)>,

    rings: Vec<AxisRing>,
    buzz: BuzzOsc,
    cmaps: Vec<Option<CountMap>>,
    last_counts: Vec<Option<i32>>,
    report_anchor: Vec<Option<(i32, f64)>>,
    last_streamed_target: Vec<Option<i32>>,
    last_sent_retired: u32,
    heartbeat_sent: bool,
    gate: TorqueGate,
    capture: Capture,
    cycle_index: u64,
    mailbox: MailboxWorker,
    pending_starts: Vec<(u32, String, PendingStart)>,
    pending_stops: Vec<(u32, PendingStop)>,
    capture_slots: Vec<u8>,
    prdiv: u64,
    ff_saturation: u32,
    wkc_consecutive: u8,
    latched_drive_err: u16,
    sensorless: SensorlessBank,
    stream_halt: StreamHalt,
}

pub fn run(ctx: &mut EndpointCtx) {
    'dc: loop {
        if SIGTERM_RECEIVED.load(Ordering::Acquire) {
            eprintln!("ec-rt: SIGTERM received — disabling drive and exiting");
            tracing::warn!(
                subsystem = "ethercat",
                event = "endpoint_exit",
                reason = "sigterm",
                "endpoint exiting: SIGTERM received — disabling drive"
            );
            break;
        }
        if ctx.server.session_ended() {
            eprintln!("ec-rt: bridge disconnected — disabling drive and exiting");
            tracing::warn!(
                subsystem = "ethercat",
                event = "endpoint_exit",
                reason = "bridge_disconnected",
                "endpoint exiting: bridge (klippy) disconnected — disabling drive (downstream of a host-side abort)"
            );
            break;
        }

        if commands::dispatch_commands(ctx).is_break() {
            break 'dc;
        }
        commands::drain_pending_starts(ctx);
        commands::drain_pending_stops(ctx);
        commands::drain_mailbox_replies(ctx);

        if cycle::run_cycle(ctx).is_break() {
            break;
        }
    }

    unsafe {
        for s in 0..ctx.num_slaves {
            ffi::ec_rt_disable(s as std::os::raw::c_int);
        }
        ffi::ec_rt_shutdown();
    }
    eprintln!("ec-rt: shutdown complete");
}
