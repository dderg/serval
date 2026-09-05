use std::sync::atomic::{AtomicBool, Ordering};

use crate::capture::{Capture, PendingStart, PendingStop};
use crate::damper::DiffDamperBank;
use crate::dynamics::DynamicsModel;
use crate::live_tap::LiveTap;
use crate::mailbox::MailboxWorker;
use crate::sensorless::SensorlessBank;
use crate::server::FrameServer;
use crate::setpoint::{SampleGrid, SetpointEntry, SetpointRing};
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
    jump_log_counts: Vec<i64>,
    cycle_ns: i64,
    group_delay_ns: u64,
    telemetry_period: u64,
    dynamics: Option<DynamicsModel>,
    /// Per-mode pin-rotor oscillator state + precomputed transition
    /// coefficients, sized to the installed dynamics model (empty when no
    /// mode is pinned).
    pin: cycle::PinState,
    /// Drive-frame sign per slot (`cmd_counts_per_mm.signum()`), fixed at
    /// bringup — the dynamics profile is fitted in the drive frame.
    drive_dirs: Vec<f32>,
    /// Drive-frame accel/velocity/following-error scratch, reused every
    /// cycle (sized at bringup) so the DC path allocates nothing.
    drive_scratch: cycle::DriveScratch,
    run_limits: Vec<(u32, u16)>,

    sp_rings: Vec<SetpointRing>,
    grid: SampleGrid,
    /// Drive-frame counts that a lane's `pos_counts == 0` maps to, latched at
    /// the first entry of each anchor epoch.
    ring_origin: Vec<Option<i32>>,
    /// Reused per-cycle buffer of the entries played this cycle, so the DC
    /// path allocates nothing.
    sp_play_scratch: Vec<Option<SetpointEntry>>,
    /// Reused decode buffer for one lane block of a `PushSampleRuns` fill, so
    /// the command path allocates nothing per frame.
    sp_fill_scratch: Vec<SetpointEntry>,
    /// Consumed `PushSampleRuns` payloads leave the DC thread here: their
    /// vectors were allocated by the socket reader, so freeing them inline
    /// would take that thread's allocator arena lock.
    reclaim: crate::reclaim::Reclaim,
    last_grid_index: u64,
    last_grid_clock: u64,
    damper: DiffDamperBank,
    trim: DiffTrimBank,
    comp: StrainCompBank,
    last_counts: Vec<Option<i32>>,
    last_written_offset: Vec<i32>,
    report_anchor: Vec<Option<(i32, f64)>>,
    last_streamed_target: Vec<Option<i32>>,
    /// Per-slot homing freeze: a StepperSuppress-ed slot holds its last
    /// commanded target and discards stream samples until ResumeStream.
    suppressed: Vec<bool>,
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
    /// Instant the previous DC exchange returned; the span from here to the
    /// next exchange entry is every non-exchange nanosecond of the loop —
    /// the region the stage clocks above do not cover.
    prev_exchange_return: Option<std::time::Instant>,
    last_pre_cycle_ns: i64,
    last_post_cycle_ns: i64,
    last_inter_exchange_ns: i64,
    pre_cycle_max_ns: i64,
    post_cycle_max_ns: i64,
    inter_exchange_max_ns: i64,
    last_nivcsw: i64,
    /// Sub-spans of the post-exchange region — reported on the next cycle's
    /// fault events so an overrun names the exact call that ate the time.
    last_fault_ns: i64,
    last_capture_ns: i64,
    last_wkc_ns: i64,
    last_heartbeat_ns: i64,
    last_telemetry_ns: i64,
    fault_max_ns: i64,
    capture_max_ns: i64,
    wkc_max_ns: i64,
    heartbeat_max_ns: i64,
    telemetry_max_ns: i64,
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

/// Per-lane progress the host paces its stream against: played ring cycles.
pub(super) fn lane_progress(ctx: &EndpointCtx) -> Vec<u32> {
    ctx.sp_rings
        .iter()
        .map(SetpointRing::played_count)
        .collect()
}

/// Trajectory clock the DC grid stamps `index` with, off the same
/// `(grid_index, grid_clock)` pair every `PushSampleRunsResponse` echoes, so
/// the host's span clocks and this endpoint's cursor share one domain.
fn grid_clock_of(ctx: &EndpointCtx, index: u64) -> u64 {
    let interval_ns = ctx.grid.interval_ns();
    if index >= ctx.last_grid_index {
        ctx.last_grid_clock + (index - ctx.last_grid_index) * interval_ns
    } else {
        ctx.last_grid_clock
            .saturating_sub((ctx.last_grid_index - index) * interval_ns)
    }
}

/// Per-lane playback clock the heartbeat carries beside the progress counts:
/// the exclusive cursor in trajectory nanoseconds, so the host retires a run
/// exactly when its `end_clock` has been consumed. Zero until a lane plays.
pub(super) fn lane_playback_clocks(ctx: &EndpointCtx) -> Vec<u64> {
    ctx.sp_rings
        .iter()
        .map(|ring| {
            ring.played_cursor()
                .map_or(0, |index| grid_clock_of(ctx, index))
        })
        .collect()
}

pub(super) fn all_lanes_idle(ctx: &EndpointCtx) -> bool {
    ctx.sp_rings.iter().all(SetpointRing::is_empty)
}

pub(super) fn respond_fault_heartbeat(
    ctx: &mut EndpointCtx,
    engine_state: u8,
    error_code: u16,
) -> Vec<u32> {
    let retired = lane_progress(ctx);
    let playback = lane_playback_clocks(ctx);
    ctx.server.respond(&status_heartbeat_frame(
        engine_state,
        error_code,
        &retired,
        &playback,
        ctx.ff_saturation,
    ));
    retired
}

pub(super) fn discard_motion(ctx: &mut EndpointCtx) {
    for r in &mut ctx.sp_rings {
        r.reset();
    }
    for o in &mut ctx.ring_origin {
        *o = None;
    }
    for lc in &mut ctx.last_counts {
        *lc = None;
    }
    // A discard re-anchors the commanded frame (homing trip, stop, sensorless
    // trip), so the pin oscillator's predicted deflection is stale — restart
    // it clean.
    ctx.pin.reset();
}
