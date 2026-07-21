use std::sync::atomic::{AtomicBool, Ordering};

use crate::buzz::BuzzOsc;
use crate::capture::{Capture, PendingStart, PendingStop};
use crate::curves::AxisRing;
use crate::damper::DiffDamperBank;
use crate::dynamics::DynamicsModel;
use crate::live_tap::LiveTap;
use crate::mailbox::MailboxWorker;
use crate::scale::CountMap;
use crate::sensorless::SensorlessBank;
use crate::server::FrameServer;
use crate::strain_comp::StrainCompBank;
use crate::stream_halt::StreamHalt;
use crate::torque::TorqueGate;
use crate::trim::DiffTrimBank;
use crate::wire::status_heartbeat_frame;

#[cfg(feature = "hw")]
mod bringup;
mod commands;
mod cycle;
mod drive;
#[cfg(test)]
mod tests;

#[cfg(feature = "hw")]
pub use bringup::bringup;

use drive::DriveChain;

static SIGTERM_RECEIVED: AtomicBool = AtomicBool::new(false);

pub struct EndpointCtx {
    server: FrameServer,
    drive: Box<dyn DriveChain>,

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
    damper: DiffDamperBank,
    trim: DiffTrimBank,
    comp: StrainCompBank,
    cmaps: Vec<Option<CountMap>>,
    last_counts: Vec<Option<i32>>,
    last_written_offset: Vec<i32>,
    report_anchor: Vec<Option<(i32, f64)>>,
    last_streamed_target: Vec<Option<i32>>,
    last_sent_retired: u32,
    heartbeat_sent: bool,
    gate: TorqueGate,
    capture: Capture,
    live_tap: LiveTap,
    tap_slots: Vec<u8>,
    cycle_index: u64,
    mailbox: MailboxWorker,
    pending_starts: Vec<(u32, String, PendingStart)>,
    pending_stops: Vec<(u32, PendingStop)>,
    pending_seed: Option<commands::PendingSeed>,
    capture_slots: Vec<u8>,
    prdiv: u64,
    ff_saturation: u32,
    wkc_consecutive: u8,
    latched_drive_err: u16,
    sensorless: SensorlessBank,
    stream_halt: StreamHalt,
    late_tolerance_ns: Option<i64>,
    timing_armed: bool,
    baseline_reanchor_count: u32,
    late_frames: u32,
    late_max_ns: i64,
    skip_count_policed: u32,
    late_frames_total: u32,
    last_lateness_ns: i64,
    last_dispatch_ns: i64,
    last_pre_work_ns: i64,
    prev_exchange_ns: i64,
    last_wake_late_ns: i64,
    last_recv_ns: i64,
    last_process_ns: i64,
    last_send_ns: i64,
    wake_late_max_ns: i64,
    recv_max_ns: i64,
    process_max_ns: i64,
    send_max_ns: i64,
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

        let pre_work = std::time::Instant::now();
        if commands::dispatch_commands(ctx).is_break() {
            break 'dc;
        }
        let dispatch_ns = pre_work.elapsed().as_nanos() as i64;
        commands::drain_pending_starts(ctx);
        commands::drain_pending_stops(ctx);
        commands::drain_pending_seed(ctx);
        commands::drain_mailbox_replies(ctx);
        ctx.last_dispatch_ns = dispatch_ns;
        ctx.last_pre_work_ns = pre_work.elapsed().as_nanos() as i64;

        if cycle::run_cycle(ctx).is_break() {
            break;
        }
    }

    ctx.drive.disable_all();
    ctx.drive.shutdown();
    eprintln!("ec-rt: shutdown complete");
}

pub(super) fn respond_fault_heartbeat(
    ctx: &mut EndpointCtx,
    engine_state: u8,
    error_code: u16,
) -> Vec<u32> {
    let retired: Vec<u32> = ctx.rings.iter().map(|r| r.retired_count()).collect();
    ctx.server.respond(&status_heartbeat_frame(
        engine_state,
        error_code,
        &retired,
        ctx.ff_saturation,
    ));
    retired
}

pub(super) fn discard_motion(ctx: &mut EndpointCtx) {
    for r in &mut ctx.rings {
        r.reset();
    }
    for c in &mut ctx.cmaps {
        *c = None;
    }
    for lc in &mut ctx.last_counts {
        *lc = None;
    }
    ctx.buzz.clear();
}
