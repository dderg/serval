// Sample-stream transport sink.
//
// Where the stepcompress sink turns clocked motor spans into step/dir volleys,
// this sink turns the same spans into `SampleRun`s: uniformly spaced absolute
// positions on the lane's own fixed-point grid. Every value on the wire is one
// `eval_at_clock` of the active span at a device clock the lane owns, taken
// once per `sample_period_cycles` tick and quantized with
// `quantize_step_delta` against the lane's position quantum — so a phase lane
// driven from samples lands on exactly the microsteps the step path would have
// pulsed. What the sink drops is the sub-sample crossing arithmetic: the
// executor receives positions, not edges, and interpolates them itself.
//
// A span is consumed once every sample inside it has been converted and its
// view released, and retired only once the mcu's own playback clock has
// carried past its end. Barriers remain for the one thing that needs a
// receipt — reconciling a re-anchor cut whose samples already reached the wire.

use std::collections::{HashMap, VecDeque};
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

use crossbeam_channel::Sender;
use host_rt::host_io::parser::ArgValue;
use runtime::sample_run::{
    SAMPLE_RUN_COUNT_MAX, SAMPLE_RUN_DATA_MAX, SampleRunBuf, SampleRunError, delta_bytes,
    encode_deltas,
};
use runtime::sample_wire::{
    SAMPLE_ANCHOR_NAME, SAMPLE_BARRIER_NAME, SAMPLE_OVERLAY_NAME, SAMPLE_RUN_NAME,
};
use runtime::stepping_state::MAX_AXES as HEARTBEAT_AXES;
use runtime::sub_sample_timing::quantize_step_delta;
use trajectory::{BuzzProfile, ClockedMotorSpan, ContinuousAxis, MotorGroup, MotorSpan, MotorTerm};

use super::barrier_ledger::{AckFault, BarrierId, BarrierLedger};
use super::pump_loop::pump_past_guard_secs;
use super::stepcompress_sink::{
    BARRIER_ACK_DEADLINE_SECONDS, ClockSource, FrameEgress, SEND_LEAD_SECONDS,
};
use super::{AxisFrame, BuzzLane, HeartbeatMsg, PumpMsg, SendError};
use crate::lock_ext::LockExt;

/// Spans one phase lane may have staged in the pump at a time. A sample lane
/// keeps no span ring on the mcu — a span is consumed the moment it has been
/// sampled through — so this is the host's own staging window, not a firmware
/// depth. The firmware depth the wire must respect is `phase_ring_depth`,
/// which the mcu advertises as `SAMPLE_RUNS_PER_LANE`.
pub const SAMPLE_LANE_PIECE_WINDOW: u32 = 64;

/// A phase lane's mcu ring is measured in single-digit runs, so the top-up
/// interval has to be a fraction of the ring's playback time rather than the
/// move-queue pacer's [`PACER_TICK`]: a retirement credit is worth nothing
/// until the next flush reads it.
pub const SAMPLE_PACER_TICK: std::time::Duration = std::time::Duration::from_millis(2);

/// A backlog this deep means the transport is not draining: every run past it
/// is lead the mcu will never receive in time.
pub const SAMPLE_BACKLOG_CEILING_RUNS: usize = 4096;
/// One coil write per sample commutates cleanly while the per-sample advance
/// stays within a quarter electrical cycle: MSCNT spans 1024 microsteps per
/// cycle (four full steps at the 256 microsteps klippy enforces for phase
/// stepping), so 256 units. Past that the commanded field angle leads the
/// rotor enough to shed torque — a demand beyond it is a fault, not a faster
/// move.
pub const QUARTER_ELECTRICAL_CYCLE_UNITS: u32 = 256;

/// Reads back a lane's executed position: `sample_get_position` answered by
/// `sample_position`, as `(clock, position)`.
pub type SamplePositionQuery = Arc<dyn Fn(u32) -> Result<(u64, i32), String> + Send + Sync>;

const SAMPLE_POSITION_QUERY_TIMEOUT: std::time::Duration = std::time::Duration::from_millis(250);

fn host_io_sample_position_query(
    mcu_id: u32,
    host_io: std::sync::Weak<host_rt::host_io::McuHostIo>,
) -> SamplePositionQuery {
    Arc::new(move |oid| {
        let io = host_io.upgrade().ok_or_else(|| {
            format!("sample mcu {mcu_id}: McuHostIo detached during position readback")
        })?;
        let params = io
            .call_args(
                runtime::sample_wire::SAMPLE_GET_POSITION_NAME,
                &[("oid".to_string(), ArgValue::Int(i64::from(oid)))],
                runtime::sample_wire::SAMPLE_POSITION_NAME,
                SAMPLE_POSITION_QUERY_TIMEOUT,
            )
            .map_err(|e| format!("sample_get_position failed for mcu {mcu_id} oid {oid}: {e:?}"))?;
        let clock = params.try_get_u32("clock").ok_or_else(|| {
            format!("sample_position from mcu {mcu_id} oid {oid} carries no `clock` field")
        })?;
        let position = params.try_get_i32("position").ok_or_else(|| {
            format!("sample_position from mcu {mcu_id} oid {oid} carries no `position` field")
        })?;
        Ok((u64::from(clock), position))
    })
}

/// Build the sample endpoint for one mcu's phase lanes. The rate is the
/// firmware's own `MOTION_SAMPLE_RATE_HZ`, carried on the topology, and the
/// lane's position quantum is its microstep distance — the LUT phase quantum
/// the mcu's phase executor counts in. `measured_clock_freq` is the same
/// measured estimate the pulse lanes' shim uses, so two lane kinds on one board
/// cannot disagree about the clock.
pub fn build_sample_endpoint(
    cfg: &crate::mcu_config::McuAxisConfig,
    host_io: std::sync::Weak<host_rt::host_io::McuHostIo>,
    pump_control: Sender<PumpMsg>,
    measured_clock_freq: f64,
    clock_of: ClockSource,
) -> Result<SampleEndpoint, String> {
    if !measured_clock_freq.is_finite() || measured_clock_freq <= 0.0 {
        return Err(format!(
            "sample mcu {}: clock estimate {measured_clock_freq} Hz is not a positive rate",
            cfg.mcu_id
        ));
    }
    let rate = cfg.phase_sample_rate;
    if !rate.is_finite() || rate <= 0.0 || rate > f64::from(u32::MAX) {
        return Err(format!(
            "sample mcu {}: phase sample rate {rate} Hz is not a representable positive rate",
            cfg.mcu_id
        ));
    }
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let sample_rate_hz = rate as u32;
    let mut lanes = Vec::new();
    for (lane, &axis) in cfg.axes.iter().enumerate() {
        if !cfg.phase_capable(lane) {
            continue;
        }
        let motor = cfg.motor_range(lane).start;
        let quantum = cfg.microstep_distance[motor];
        if !quantum.is_finite() || quantum <= 0.0 {
            return Err(format!(
                "sample mcu {} axis {axis}: position quantum {quantum} mm is not a positive length",
                cfg.mcu_id
            ));
        }
        // The lane cap models what one coil write per sample can commutate,
        // not the pulse executor's per-step ISR budget: MSCNT spans 1024
        // microsteps per electrical cycle (klippy enforces microsteps: 256
        // for phase stepping), and past a quarter cycle per sample the field
        // angle leads the rotor far enough to shed torque. The pulse-derived
        // step ceiling still applies when it is the larger bound (coarse
        // microstepping at high sample rates).
        let velocity_ceiling = cfg.motor_velocity_ceiling(axis);
        let units_per_sample = (velocity_ceiling / quantum / rate).ceil();
        if !units_per_sample.is_finite() || units_per_sample > f64::from(u32::MAX) {
            return Err(format!(
                "sample mcu {} axis {axis}: {velocity_ceiling} mm/s over {quantum} mm quanta at \
                 {rate} Hz needs {units_per_sample} units per sample, which the wire cannot carry",
                cfg.mcu_id
            ));
        }
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let max_units_per_sample = (units_per_sample as u32).max(QUARTER_ELECTRICAL_CYCLE_UNITS);
        #[allow(clippy::cast_possible_truncation)]
        lanes.push(SampleLaneConfig {
            axis: axis as u8,
            oid: cfg.stepper_oids[motor],
            cycles_per_second: measured_clock_freq,
            sample_rate_hz,
            position_quantum_mm: quantum as f32,
            max_units_per_sample,
            ring_depth: cfg.phase_ring_depth,
        });
    }
    if lanes.is_empty() {
        return Err(format!(
            "sample mcu {}: no phase lanes to stream to; a sample endpoint was built for an \
             mcu whose every lane is a pulse lane",
            cfg.mcu_id
        ));
    }
    let mut endpoint = SampleEndpoint::new(
        cfg.mcu_id,
        &lanes,
        super::stepcompress_sink::host_io_egress(cfg.mcu_id, host_io.clone()),
        clock_of,
        pump_control,
    )
    .map_err(|e| format!("sample mcu {}: {e}", cfg.mcu_id))?;
    endpoint.set_position_query(host_io_sample_position_query(cfg.mcu_id, host_io));
    Ok(endpoint)
}

/// One motor's sample lane. The position quantum is the fixed point the lane
/// counts in — a phase lane counts LUT phase quanta, an EtherCAT lane counts
/// drive counts — declared once, at config time.
#[derive(Clone, Copy, Debug)]
pub struct SampleLaneConfig {
    pub axis: u8,
    pub oid: u32,
    pub cycles_per_second: f64,
    pub sample_rate_hz: u32,
    pub position_quantum_mm: f32,
    pub max_units_per_sample: u32,
    /// Runs the mcu's ring for this lane holds, as the firmware advertised it.
    pub ring_depth: u32,
}

impl SampleLaneConfig {
    fn sample_period_cycles(&self) -> Result<u32, SendError> {
        let cycles = self.cycles_per_second / f64::from(self.sample_rate_hz);
        if !(1.0..=f64::from(u32::MAX)).contains(&cycles) {
            return Err(SendError::Fatal(format!(
                "sample lane axis {} oid {}: sample_rate_hz {} is not representable against \
                 cycles_per_second {}",
                self.axis, self.oid, self.sample_rate_hz, self.cycles_per_second
            )));
        }
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        Ok(cycles.round() as u32)
    }
}

/// A span carrying a `motor_mask` is relativized to start at zero, so its
/// samples ride against their own frame and leave the lane's absolute frame
/// where the last kinematic span left it — the same split the step path keeps
/// between `last_step_count` and `overlay_step_frame`, and the reason overlay
/// runs leave as `sample_overlay` rather than on the lane's abutting stream.
#[derive(Clone, Copy, Debug)]
struct OverlayFrame {
    p_prev: f32,
    position: i64,
    step_phase: f32,
}

/// A nonzero motor mask is what marks a view as an overlay: the lane
/// relativizes it to zero and ships it as `sample_overlay` instead of on the
/// absolute stream — which is exactly what a buzz is, a displacement around
/// wherever the trajectory left the lane.
const BUZZ_OVERLAY_MASK: u8 = 1;

/// The sweep the endpoint currently holds on its lanes. `end_clock` is the
/// only thing that proves it played out: overlay runs ride the lane's
/// separate overlay ring, so the heartbeat's retirement count never counts
/// them and only the mcu's own playback clock can carry past the sweep.
struct ActiveBuzz {
    lanes: Vec<usize>,
    end_clock: u64,
}

/// A seam the sink must handle when the marked span reaches it, in stream
/// order. `Cut` is a fresh anchor: stop sampling, re-slope the lane clock,
/// re-anchor the mcu. `Gap` is a rejoin — a stationary stream-time hole, which
/// on a sample lane is exactly a sanctioned re-anchor at the same position.
#[derive(Clone, Copy, Debug)]
enum PendingSeam {
    Cut { at: u64, epoch_freq: Option<f64> },
    Gap { at: u64 },
}

impl PendingSeam {
    fn at(self) -> u64 {
        match self {
            Self::Cut { at, .. } | Self::Gap { at } => at,
        }
    }
}

/// A cut whose samples already reached the wire: the lane is parked until the
/// mcu returns the barrier and the readback says where it actually stopped.
struct PendingSampleCut {
    barrier: BarrierId,
    cut_at: u64,
    epoch_freq: f64,
    expected_position: i64,
    held: Vec<ClockedMotorSpan>,
}

/// What the mcu's last `StatusHeartbeat` said about each lane ring: the
/// monotonic count of sample runs it has retired, and the clock it last
/// evaluated playback at. The heartbeat lands on the transport's reactor
/// thread while the pacer holds the endpoint, so both cross threads through
/// atomics rather than the endpoint lock.
///
/// The two prove ring room independently. The count is the round trip: it only
/// moves when a heartbeat carrying a fresh retirement arrives. The clock is a
/// fact about the mcu's own playback — a run whose window closed at or before
/// it has left the ring whatever the count says — and because it comes from
/// the mcu it can never run ahead of what the mcu actually played.
#[derive(Debug)]
pub struct RetiredRuns {
    per_axis: [AtomicU32; HEARTBEAT_AXES],
    playback_clock: [AtomicU64; HEARTBEAT_AXES],
}

impl RetiredRuns {
    fn new() -> Self {
        Self {
            per_axis: [const { AtomicU32::new(0) }; HEARTBEAT_AXES],
            playback_clock: [const { AtomicU64::new(0) }; HEARTBEAT_AXES],
        }
    }

    /// `counts` and `clocks` are the heartbeat's per-axis vectors, indexed the
    /// way the mcu's engine indexes its sample lanes: by axis.
    pub fn record(&self, counts: &[u32], clocks: &[u64]) {
        for (cell, &count) in self.per_axis.iter().zip(counts) {
            cell.store(count, Ordering::Relaxed);
        }
        for (cell, &clock) in self.playback_clock.iter().zip(clocks) {
            cell.store(clock, Ordering::Relaxed);
        }
    }

    fn of_axis(&self, axis: u8) -> Result<u32, SendError> {
        self.per_axis
            .get(usize::from(axis))
            .map(|cell| cell.load(Ordering::Relaxed))
            .ok_or_else(|| self.past_heartbeat_axes(axis))
    }

    fn playback_clock_of_axis(&self, axis: u8) -> Result<u64, SendError> {
        self.playback_clock
            .get(usize::from(axis))
            .map(|cell| cell.load(Ordering::Relaxed))
            .ok_or_else(|| self.past_heartbeat_axes(axis))
    }

    fn past_heartbeat_axes(&self, axis: u8) -> SendError {
        SendError::Fatal(format!(
            "sample lane axis {axis} is past the {HEARTBEAT_AXES} axes the mcu's retirement \
             heartbeat carries"
        ))
    }
}

enum Outbound {
    Anchor {
        oid: u32,
        clock: u64,
        position: i32,
    },
    Run {
        oid: u32,
        interval: u32,
        count: u8,
        data: Vec<u8>,
    },
    Overlay {
        oid: u32,
        clock: u64,
        interval: u32,
        count: u8,
        data: Vec<u8>,
    },
    Barrier(BarrierId),
}

impl Outbound {
    fn kind(&self) -> &'static str {
        match self {
            Self::Anchor { .. } => SAMPLE_ANCHOR_NAME,
            Self::Run { .. } => SAMPLE_RUN_NAME,
            Self::Overlay { .. } => SAMPLE_OVERLAY_NAME,
            Self::Barrier(_) => SAMPLE_BARRIER_NAME,
        }
    }

    /// Whether the frame takes a slot in the mcu's main lane ring — the ring
    /// whose depth the firmware advertises and whose retirement the heartbeat
    /// reports. An overlay run lands in the lane's separate overlay ring and an
    /// anchor or barrier takes no slot at all.
    fn occupies_lane_ring(&self) -> bool {
        matches!(self, Self::Run { .. })
    }

    /// An anchor heads the run behind it and clears the mcu's ring on arrival,
    /// so it may only leave when that run can leave with it.
    fn needs_ring_room(&self) -> bool {
        matches!(self, Self::Run { .. } | Self::Anchor { .. })
    }

    /// The clock the mcu's lane cursor reaches once this frame has played out:
    /// the same `start_clock + interval * count` the executor's ring header
    /// computes, so the two ends agree on when a slot frees.
    fn end_clock(&self, start_clock: u64) -> u64 {
        match self {
            Self::Run {
                interval, count, ..
            }
            | Self::Overlay {
                interval, count, ..
            } => start_clock.wrapping_add(u64::from(*interval) * u64::from(*count)),
            Self::Anchor { .. } | Self::Barrier(_) => start_clock,
        }
    }
}

struct OutboundRun {
    frame: Outbound,
    lane: usize,
    start_clock: u64,
    end_clock: u64,
    enqueue_order: u64,
}

fn frame_args(frame: &Outbound) -> (&'static str, Vec<(String, ArgValue)>) {
    let int = |name: &str, value: i64| (name.to_string(), ArgValue::Int(value));
    match frame {
        Outbound::Anchor {
            oid,
            clock,
            position,
        } => (
            SAMPLE_ANCHOR_NAME,
            vec![
                int("oid", i64::from(*oid)),
                int("clock", *clock as i64),
                int("position", i64::from(*position)),
            ],
        ),
        Outbound::Run {
            oid,
            interval,
            count,
            data,
        } => (
            SAMPLE_RUN_NAME,
            vec![
                int("oid", i64::from(*oid)),
                int("interval", i64::from(*interval)),
                int("count", i64::from(*count)),
                ("data".to_string(), ArgValue::Bytes(data.clone())),
            ],
        ),
        Outbound::Overlay {
            oid,
            clock,
            interval,
            count,
            data,
        } => (
            SAMPLE_OVERLAY_NAME,
            vec![
                int("oid", i64::from(*oid)),
                int("clock", *clock as i64),
                int("interval", i64::from(*interval)),
                int("count", i64::from(*count)),
                ("data".to_string(), ArgValue::Bytes(data.clone())),
            ],
        ),
        Outbound::Barrier(id) => (
            SAMPLE_BARRIER_NAME,
            vec![int("oid", i64::from(id.oid)), int("seq", i64::from(id.seq))],
        ),
    }
}

/// A run the sampler closed: everything the wire needs, resolved.
struct ClosedRun {
    start_clock: u64,
    interval: u32,
    positions: Vec<i32>,
    base: i32,
    overlay: bool,
    anchor: bool,
}

struct SampleLane {
    cfg: SampleLaneConfig,
    sample_period_cycles: u32,
    views: VecDeque<ClockedMotorSpan>,
    /// The one view the lane is converting right now. It leaves only when
    /// every sample inside it has been encoded, which is also when the lane
    /// counts it consumed.
    active: Option<ClockedMotorSpan>,
    overlay: Option<OverlayFrame>,
    p_prev: f32,
    step_phase: f32,
    position: i64,
    prev_sample: u64,
    origin_clock: Option<u64>,
    positioned: bool,
    resume_floor: u64,
    /// The absolute position the lane's next run encodes its deltas against,
    /// and the clock that run must start at. A `None` clock means the lane owes
    /// the mcu a `sample_anchor` before any run.
    wire_position: i64,
    wire_next_clock: Option<u64>,
    run: SampleRunBuf<SAMPLE_RUN_COUNT_MAX>,
    run_bytes: usize,
    run_is_overlay: bool,
    /// Monotonic count of runs handed to the wire for this lane, paired with
    /// what the mcu has proven retired: their difference is what the lane's
    /// ring holds, and it must never reach `cfg.ring_depth`.
    runs_sent: u32,
    /// End clocks of the runs the mcu has not yet proven retired, oldest first,
    /// and how many the mcu's reported playback clock has already carried past.
    /// The ring plays in order, so counting from the front is exact.
    in_flight_end_clocks: VecDeque<u64>,
    clock_retired: u32,
    /// The `now` at which this lane was first seen with a full ring and work
    /// waiting behind it. Delivery that never resumes is a wedged lane, not a
    /// slow one.
    saturated_since: Option<u64>,
    seams: VecDeque<PendingSeam>,
    cut: Option<PendingSampleCut>,
    /// Monotonic count of views fully converted and released, and the end
    /// clocks of the released views the mcu's playback has not yet carried
    /// past. A view is retired only against that clock, so a cut that abandons
    /// unresolved views credits neither odometer for them.
    consumed: u32,
    unretired_view_ends: VecDeque<u64>,
    retired: u32,
}

impl SampleLane {
    fn new(cfg: SampleLaneConfig) -> Result<Self, SendError> {
        let sample_period_cycles = cfg.sample_period_cycles()?;
        Ok(Self {
            cfg,
            sample_period_cycles,
            views: VecDeque::new(),
            active: None,
            overlay: None,
            p_prev: 0.0,
            step_phase: 0.0,
            position: 0,
            prev_sample: 0,
            origin_clock: None,
            positioned: false,
            resume_floor: 0,
            wire_position: 0,
            wire_next_clock: None,
            run: SampleRunBuf::new(0, sample_period_cycles),
            run_bytes: 0,
            run_is_overlay: false,
            runs_sent: 0,
            in_flight_end_clocks: VecDeque::new(),
            clock_retired: 0,
            saturated_since: None,
            seams: VecDeque::new(),
            cut: None,
            consumed: 0,
            unretired_view_ends: VecDeque::new(),
            retired: 0,
        })
    }

    /// A slot is free once the mcu has proven the run in it gone, and either
    /// report proves it on its own: the retirement count, or the reported
    /// playback clock having passed the run's end clock. Neither can over-free
    /// — both are the mcu's own observations — so the stronger of the two is
    /// the truth, and the count remains the floor whenever the clock report is
    /// the older of the two.
    fn retired_proven(&self, retired: &RetiredRuns) -> Result<u32, SendError> {
        Ok(retired.of_axis(self.cfg.axis)?.max(self.clock_retired))
    }

    fn outstanding_runs(&self, retired: &RetiredRuns) -> Result<u32, SendError> {
        Ok(self.runs_sent.wrapping_sub(self.retired_proven(retired)?))
    }

    /// Consume the mcu's latest reports: drop every in-flight run the reported
    /// playback clock has carried past, drop the ones the count alone proves
    /// gone so the queue never outgrows the ring, and retire every released
    /// view whose window that same playback clock has closed.
    fn absorb_mcu_reports(&mut self, retired: &RetiredRuns) -> Result<(), SendError> {
        let credit = retired.of_axis(self.cfg.axis)?;
        let playback_clock = retired.playback_clock_of_axis(self.cfg.axis)?;
        while self
            .in_flight_end_clocks
            .front()
            .is_some_and(|&end| end <= playback_clock)
        {
            self.in_flight_end_clocks.pop_front();
            self.clock_retired = self.clock_retired.wrapping_add(1);
        }
        let credit_lead = credit.wrapping_sub(self.clock_retired);
        if credit_lead != 0 && credit_lead <= u32::MAX / 2 {
            for _ in 0..credit_lead.min(self.in_flight_end_clocks.len() as u32) {
                self.in_flight_end_clocks.pop_front();
            }
            self.clock_retired = credit;
        }
        while self
            .unretired_view_ends
            .front()
            .is_some_and(|&end| end <= playback_clock)
        {
            self.unretired_view_ends.pop_front();
            self.retired = self.retired.wrapping_add(1);
        }
        Ok(())
    }

    /// How far past the mcu clock a run may be handed over. The mcu's ring is
    /// a playback queue, not a buffer for the host's whole planning lead: a run
    /// parked in it does not retire until its own window arrives, so filling
    /// the ring with lead would freeze retirement and starve the lane behind
    /// it. One ring's worth of full runs is exactly the residency the ring can
    /// turn over.
    fn send_horizon_cycles(&self) -> u64 {
        u64::from(self.cfg.ring_depth)
            * SAMPLE_RUN_COUNT_MAX as u64
            * u64::from(self.sample_period_cycles)
    }

    fn open_run_start(&self) -> Option<u64> {
        (!self.run.is_empty()).then(|| self.run.header().start_clock)
    }

    fn reset_to(&mut self, position: i64, resume_floor: u64) {
        self.abandon_views();
        self.overlay = None;
        self.position = position;
        #[allow(clippy::cast_precision_loss)]
        {
            self.p_prev = position as f32 * self.cfg.position_quantum_mm;
        }
        self.step_phase = 0.0;
        self.prev_sample = 0;
        self.origin_clock = None;
        self.positioned = true;
        self.resume_floor = resume_floor;
        self.run.reset(0, self.sample_period_cycles);
        self.run_bytes = 0;
        self.run_is_overlay = false;
        self.wire_position = position;
        self.wire_next_clock = None;
    }

    /// Sample every view the lane holds whose samples land at or before
    /// `sample_to`. Closed runs queue in the endpoint's backlog; the mcu's ring
    /// depth paces what leaves it, never what the lane samples.
    fn sample_until(&mut self, sample_to: u64, out: &mut Vec<ClosedRun>) -> Result<(), SendError> {
        loop {
            if self.active.is_none() && !self.activate_next_view(out)? {
                return Ok(());
            }
            let view_end = self.active_window_end()?;
            loop {
                let next_sample = self.prev_sample + u64::from(self.sample_period_cycles);
                if next_sample > sample_to {
                    return Ok(());
                }
                if next_sample > view_end {
                    break;
                }
                self.emit_sample(next_sample, out)?;
                self.prev_sample = next_sample;
            }
            self.release_active_view()?;
        }
    }

    /// Where the active view's samples stop: its own end clock, or the start
    /// of the view staged behind it when that lands first.
    fn active_window_end(&self) -> Result<u64, SendError> {
        let active = self
            .active
            .as_ref()
            .ok_or_else(|| self.fatal("the lane has no active view to sample"))?;
        Ok(match self.views.front() {
            Some(next) => active.end_clock.min(next.start_clock),
            None => active.end_clock,
        })
    }

    /// Take the next staged view as the lane's one active view: switch the
    /// open run's frame with it, and adopt the lane origin when the lane has
    /// none yet.
    fn activate_next_view(&mut self, out: &mut Vec<ClosedRun>) -> Result<bool, SendError> {
        let Some(view) = self.views.pop_front() else {
            return Ok(false);
        };
        let overlay_view = view.signal.motor_mask != 0;
        if overlay_view != self.run_is_overlay {
            self.close_run(out)?;
            self.run_is_overlay = overlay_view;
        }
        self.overlay = overlay_view.then_some(OverlayFrame {
            p_prev: 0.0,
            position: 0,
            step_phase: 0.0,
        });
        if self.origin_clock.is_none() {
            let begin = view.start_clock.max(self.resume_floor);
            self.prev_sample = begin;
            let position = self.eval_view_at(&view, begin)?;
            if !self.positioned {
                self.p_prev = position;
                self.positioned = true;
            }
            self.origin_clock = Some(begin);
        } else {
            let next_sample = self.prev_sample + u64::from(self.sample_period_cycles);
            if view.start_clock > next_sample {
                return Err(self.fatal(&format!(
                    "view at clock {} leaves a {} tick hole after the sample at {} — a hole in \
                     the stream needs an explicit re-anchor, never a padded start",
                    view.start_clock,
                    view.start_clock - self.prev_sample,
                    self.prev_sample
                )));
            }
        }
        self.active = Some(view);
        Ok(true)
    }

    /// The active view is spent: it leaves the lane counted consumed, and its
    /// end clock waits for the mcu's playback to prove it retired.
    fn release_active_view(&mut self) -> Result<(), SendError> {
        let view = self
            .active
            .take()
            .ok_or_else(|| self.fatal("the lane released a view it was not converting"))?;
        self.consumed = self.consumed.wrapping_add(1);
        self.unretired_view_ends.push_back(view.end_clock);
        Ok(())
    }

    /// Drop everything the lane has not converted. An abandoned view proves
    /// nothing about the mcu's playback, so neither odometer moves for it.
    fn abandon_views(&mut self) {
        self.views.clear();
        self.active = None;
    }

    #[allow(clippy::cast_possible_truncation)]
    fn eval_view_at(&self, view: &ClockedMotorSpan, clock: u64) -> Result<f32, SendError> {
        let pva = view.eval_at_clock(clock).map_err(|error| {
            self.fatal(&format!(
                "the view spanning clocks [{}, {}] does not evaluate at clock {clock}: {error}",
                view.start_clock, view.end_clock
            ))
        })?;
        Ok(pva.position as f32)
    }

    fn eval_active(&self, clock: u64) -> Result<f32, SendError> {
        let view = self
            .active
            .as_ref()
            .ok_or_else(|| self.fatal("a sample was demanded with no active view"))?;
        self.eval_view_at(view, clock)
    }

    fn emit_sample(&mut self, now: u64, out: &mut Vec<ClosedRun>) -> Result<(), SendError> {
        let p_end = self.eval_active(now)?;
        let (prev, p_start, step_phase_start) = match self.overlay {
            Some(frame) => (frame.position, frame.p_prev, frame.step_phase),
            None => (self.position, self.p_prev, self.step_phase),
        };
        let step_phase_end = step_phase_start + (p_end - p_start);
        let quantum = self.cfg.position_quantum_mm;
        let step_delta = quantize_step_delta(step_phase_end, quantum);
        #[allow(clippy::cast_precision_loss)]
        let next_step_phase = step_phase_end - step_delta as f32 * quantum;
        let units = step_delta.unsigned_abs();
        if units > self.cfg.max_units_per_sample {
            return Err(self.fatal(&format!(
                "advance of {units} units in one {} Hz sample exceeds the lane cap {}",
                self.cfg.sample_rate_hz, self.cfg.max_units_per_sample
            )));
        }
        let target = prev + i64::from(step_delta);
        let position = i32::try_from(target).map_err(|_| {
            self.fatal(&format!(
                "position {target} at clock {now} does not fit the lane's fixed point"
            ))
        })?;
        self.push_sample(now, position, out)?;
        match &mut self.overlay {
            Some(frame) => {
                frame.p_prev = p_end;
                frame.position = target;
                frame.step_phase = next_step_phase;
            }
            None => {
                self.p_prev = p_end;
                self.position = target;
                self.step_phase = next_step_phase;
            }
        }
        Ok(())
    }

    /// The absolute position an empty run encodes its first delta against. An
    /// overlay run is relativized to zero by construction.
    fn run_base(&self) -> Result<i32, SendError> {
        if self.run_is_overlay {
            return Ok(0);
        }
        i32::try_from(self.wire_position)
            .map_err(|_| self.fatal("the lane's wire position no longer fits its fixed point"))
    }

    /// Append one sample to the open run, closing it first when the sample
    /// cannot join: a clock that does not abut, a full run, or a delta that
    /// would overrun the wire payload.
    fn push_sample(
        &mut self,
        clock: u64,
        position: i32,
        out: &mut Vec<ClosedRun>,
    ) -> Result<(), SendError> {
        if !self.run.is_empty() && self.run.next_clock() != clock {
            self.close_run(out)?;
        }
        if !self.run.is_empty() {
            let base = self
                .run
                .last_position()
                .ok_or_else(|| self.fatal("a non-empty run has no last position"))?;
            let cost = delta_bytes(base, position).map_err(|e| self.sample_fatal(e))?;
            if self.run.is_full() || self.run_bytes + cost > SAMPLE_RUN_DATA_MAX {
                self.close_run(out)?;
            }
        }
        if self.run.is_empty() {
            self.run.reset(clock, self.sample_period_cycles);
            self.run_bytes = 0;
        }
        let base = match self.run.last_position() {
            Some(last) => last,
            None => self.run_base()?,
        };
        self.run_bytes += delta_bytes(base, position).map_err(|e| self.sample_fatal(e))?;
        self.run.push(position).map_err(|e| self.sample_fatal(e))?;
        Ok(())
    }

    fn close_run(&mut self, out: &mut Vec<ClosedRun>) -> Result<(), SendError> {
        if self.run.is_empty() {
            return Ok(());
        }
        let header = self.run.header();
        let positions = self.run.samples().to_vec();
        let first = *positions
            .first()
            .ok_or_else(|| self.fatal("a non-empty run has no first sample"))?;
        let last = *positions
            .last()
            .ok_or_else(|| self.fatal("a non-empty run has no last sample"))?;
        let overlay = self.run_is_overlay;
        let anchor = !overlay && self.wire_next_clock.is_none();
        if let Some(expected) = self.wire_next_clock.filter(|_| !overlay) {
            if expected != header.start_clock {
                return Err(self.fatal(&format!(
                    "run at clock {} does not abut the previous run's end {expected} — a hole \
                     in the stream is a producer bug, not something to pad over",
                    header.start_clock
                )));
            }
        }
        let base = if overlay {
            0
        } else if anchor {
            first
        } else {
            self.run_base()?
        };
        if !overlay {
            self.wire_position = i64::from(last);
            self.wire_next_clock = Some(header.end_clock());
        }
        out.push(ClosedRun {
            start_clock: header.start_clock,
            interval: header.interval_ticks,
            positions,
            base,
            overlay,
            anchor,
        });
        self.run.reset(0, self.sample_period_cycles);
        self.run_bytes = 0;
        Ok(())
    }

    fn fatal(&self, message: &str) -> SendError {
        SendError::Fatal(format!(
            "sample lane axis {} oid {}: {message}",
            self.cfg.axis, self.cfg.oid
        ))
    }

    fn sample_fatal(&self, error: SampleRunError) -> SendError {
        SendError::Fatal(format!(
            "sample lane axis {} oid {}: {} ({error:?}, fault {})",
            self.cfg.axis,
            self.cfg.oid,
            error.as_str(),
            error.fault_code()
        ))
    }
}

pub struct SampleEndpoint {
    mcu_id: u32,
    lanes: Vec<SampleLane>,
    by_axis: HashMap<u8, usize>,
    egress: FrameEgress,
    clock_of: ClockSource,
    pump_control: Sender<PumpMsg>,
    position_query: Option<SamplePositionQuery>,
    barriers: BarrierLedger,
    backlog: VecDeque<OutboundRun>,
    next_outbound_order: u64,
    mcu_retired: Arc<RetiredRuns>,
    buzz: Option<ActiveBuzz>,
    fatal: Option<String>,
}

impl SampleEndpoint {
    pub fn new(
        mcu_id: u32,
        lanes: &[SampleLaneConfig],
        egress: FrameEgress,
        clock_of: ClockSource,
        pump_control: Sender<PumpMsg>,
    ) -> Result<Self, SendError> {
        let mut by_axis = HashMap::new();
        let mut built = Vec::with_capacity(lanes.len());
        for (index, cfg) in lanes.iter().enumerate() {
            if by_axis.insert(cfg.axis, index).is_some() {
                return Err(SendError::Fatal(format!(
                    "sample endpoint mcu {mcu_id}: axis {} is configured twice",
                    cfg.axis
                )));
            }
            built.push(SampleLane::new(*cfg)?);
        }
        Ok(Self {
            mcu_id,
            lanes: built,
            by_axis,
            egress,
            clock_of,
            pump_control,
            position_query: None,
            barriers: BarrierLedger::new(),
            backlog: VecDeque::new(),
            next_outbound_order: 0,
            mcu_retired: Arc::new(RetiredRuns::new()),
            fatal: None,
            buzz: None,
        })
    }

    pub fn set_position_query(&mut self, query: SamplePositionQuery) {
        self.position_query = Some(query);
    }

    pub fn is_fatal(&self) -> bool {
        self.fatal.is_some()
    }

    pub fn retired_counts(&self) -> Vec<u32> {
        self.lanes.iter().map(|lane| lane.retired).collect()
    }

    /// Views the lanes have fully converted and released, whatever the mcu has
    /// since played. This is the odometer that frees the pump's staging
    /// window; [`SampleEndpoint::retired_counts`] is the one playback proves.
    pub fn consumed_counts(&self) -> Vec<u32> {
        self.lanes.iter().map(|lane| lane.consumed).collect()
    }

    pub fn lane_positions(&self) -> Vec<i64> {
        self.lanes.iter().map(|lane| lane.position).collect()
    }

    /// The cell the mcu's retirement heartbeat credits. The transport attaches
    /// its heartbeat callback to this, so a run leaves the backlog as soon as
    /// the ring has room — no barrier, no timer.
    pub fn mcu_retired(&self) -> Arc<RetiredRuns> {
        Arc::clone(&self.mcu_retired)
    }

    pub fn outstanding_runs(&self) -> Result<Vec<u32>, SendError> {
        self.lanes
            .iter()
            .map(|lane| lane.outstanding_runs(&self.mcu_retired))
            .collect()
    }

    pub fn backlog_len(&self) -> usize {
        self.backlog.len()
    }

    /// Whether this endpoint owns `axis` as one of its phase lanes. The pump
    /// routes by this, so a lane's transport is a membership fact rather than
    /// a configured mode.
    pub fn drives_axis(&self, axis: u8) -> bool {
        self.by_axis.contains_key(&axis)
    }

    pub fn owns_oid(&self, oid: u32) -> bool {
        self.lanes.iter().any(|lane| lane.cfg.oid == oid)
    }

    fn reject_latched_fatal(&self) -> Result<(), SendError> {
        self.fatal
            .as_ref()
            .map_or(Ok(()), |message| Err(SendError::Fatal(message.clone())))
    }

    /// Latch a fatal, tell the pump once, then refuse to run again so the
    /// endpoint stops ticking instead of retrying a broken stream.
    fn escalate(&mut self, error: SendError) -> SendError {
        let SendError::Fatal(message) = &error else {
            return error;
        };
        if self.fatal.is_none() {
            self.fatal = Some(message.clone());
            let _ = self.pump_control.send(PumpMsg::StepcompressFatal {
                mcu_id: self.mcu_id,
                error: message.clone(),
            });
        }
        error
    }

    fn lane_of(&self, axis: u8) -> Result<usize, SendError> {
        self.by_axis.get(&axis).copied().ok_or_else(|| {
            SendError::Fatal(format!(
                "sample endpoint mcu {}: no lane configured for axis {axis}",
                self.mcu_id
            ))
        })
    }

    fn lane_mut(&mut self, index: usize) -> Result<&mut SampleLane, SendError> {
        let mcu_id = self.mcu_id;
        self.lanes.get_mut(index).ok_or_else(|| {
            SendError::Fatal(format!("sample endpoint mcu {mcu_id}: no lane {index}"))
        })
    }

    fn lane_ref(&self, index: usize) -> Result<&SampleLane, SendError> {
        let mcu_id = self.mcu_id;
        self.lanes.get(index).ok_or_else(|| {
            SendError::Fatal(format!("sample endpoint mcu {mcu_id}: no lane {index}"))
        })
    }

    fn clock_now(&self) -> Result<(u64, f64), SendError> {
        (self.clock_of)(self.mcu_id).ok_or_else(|| {
            SendError::Fatal(format!(
                "sample endpoint mcu {}: no clock estimate yet",
                self.mcu_id
            ))
        })
    }

    fn queue_outbound(&mut self, lane: usize, frame: Outbound, start_clock: u64) {
        let enqueue_order = self.next_outbound_order;
        self.next_outbound_order = self.next_outbound_order.wrapping_add(1);
        let end_clock = frame.end_clock(start_clock);
        self.backlog.push_back(OutboundRun {
            frame,
            lane,
            start_clock,
            end_clock,
            enqueue_order,
        });
    }

    fn order_backlog_by_deadline(&mut self) {
        let backlog = self.backlog.make_contiguous();
        if backlog.windows(2).any(|pair| {
            (pair[0].start_clock, pair[0].enqueue_order)
                > (pair[1].start_clock, pair[1].enqueue_order)
        }) {
            backlog.sort_by_key(|out| (out.start_clock, out.enqueue_order));
        }
    }

    pub fn reset_position(&mut self, positions: &[i64]) -> Result<(), SendError> {
        if positions.len() != self.lanes.len() {
            let error = SendError::Fatal(format!(
                "sample endpoint mcu {}: position seed has {} entries for {} configured lanes",
                self.mcu_id,
                positions.len(),
                self.lanes.len()
            ));
            return Err(self.escalate(error));
        }
        let (now, _) = self.clock_now()?;
        for (index, &position) in positions.iter().enumerate() {
            let lane = self.lane_mut(index)?;
            lane.abandon_views();
            lane.seams.clear();
            lane.cut = None;
            lane.reset_to(position, now);
        }
        self.backlog.clear();
        Ok(())
    }

    /// Nothing staged, nothing unretired, no cut in flight: the precondition
    /// for handing a lane's motor over to the classic step queue.
    pub fn transport_quiescent(&self) -> Result<bool, SendError> {
        if !self.backlog.is_empty() {
            return Ok(false);
        }
        for lane in &self.lanes {
            if lane.cut.is_some()
                || !lane.views.is_empty()
                || lane.active.is_some()
                || !lane.seams.is_empty()
                || lane.open_run_start().is_some()
                || lane.outstanding_runs(&self.mcu_retired)? != 0
            {
                return Ok(false);
            }
        }
        Ok(true)
    }

    /// Nothing the lane owes the mcu, and nothing the mcu still owes back: the
    /// precondition for arming a sweep on it, and the same one for calling the
    /// sweep played out.
    fn lane_idle_for_buzz(&self, index: usize) -> Result<bool, SendError> {
        let lane = self.lane_ref(index)?;
        Ok(lane.cut.is_none()
            && lane.views.is_empty()
            && lane.active.is_none()
            && lane.seams.is_empty()
            && lane.open_run_start().is_none()
            && lane.outstanding_runs(&self.mcu_retired)? == 0
            && !self.backlog.iter().any(|out| out.lane == index))
    }

    /// Arm one resonance sweep across `lanes`. Every lane is cleared and every
    /// overlay view built before any lane is touched, so a refusal leaves the
    /// endpoint exactly as it was; the seam gap at the anchor makes the sweep's
    /// start a sanctioned re-anchor rather than a hole in the lane's stream,
    /// and one egress burst carries every lane so the axes stay in phase.
    pub fn arm_buzz(
        &mut self,
        lanes: &[BuzzLane],
        profile: &Arc<BuzzProfile>,
        anchor_clock: u64,
        clock_freq_hz: f64,
    ) -> Result<(), SendError> {
        self.reject_latched_fatal()?;
        if lanes.is_empty() {
            return Err(SendError::Fatal(format!(
                "sample endpoint mcu {}: a resonance buzz names no lane to drive",
                self.mcu_id
            )));
        }
        if self.buzz.is_some() {
            return Err(SendError::Fatal(format!(
                "sample endpoint mcu {}: a resonance buzz is already armed on these lanes",
                self.mcu_id
            )));
        }
        if !clock_freq_hz.is_finite() || clock_freq_hz <= 0.0 {
            return Err(SendError::Fatal(format!(
                "sample endpoint mcu {}: a resonance buzz cannot be clocked at {clock_freq_hz} Hz",
                self.mcu_id
            )));
        }
        let (now, _) = self.clock_now()?;
        if anchor_clock < now {
            return Err(SendError::Fatal(format!(
                "sample endpoint mcu {}: resonance buzz anchor clock {anchor_clock} is already \
                 behind the mcu clock {now}",
                self.mcu_id
            )));
        }
        let mut indexes: Vec<usize> = Vec::with_capacity(lanes.len());
        for lane in lanes {
            if !lane.sign.is_finite() || lane.sign == 0.0 {
                return Err(SendError::Fatal(format!(
                    "sample endpoint mcu {}: resonance buzz axis {} is driven with sign {}",
                    self.mcu_id, lane.axis, lane.sign
                )));
            }
            let index = self.lane_of(lane.axis)?;
            if indexes.contains(&index) {
                return Err(SendError::Fatal(format!(
                    "sample endpoint mcu {}: resonance buzz names axis {} twice",
                    self.mcu_id, lane.axis
                )));
            }
            if !self.lane_idle_for_buzz(index)? {
                return Err(SendError::Fatal(format!(
                    "sample endpoint mcu {}: resonance buzz axis {} still carries trajectory the \
                     lane has not converted and seen played",
                    self.mcu_id, lane.axis
                )));
            }
            indexes.push(index);
        }
        let mut frames = Vec::with_capacity(lanes.len());
        let mut end_clock = 0;
        for lane in lanes {
            let view = self.buzz_overlay_view(*lane, profile, anchor_clock, clock_freq_hz)?;
            end_clock = end_clock.max(view.end_clock);
            frames.push(AxisFrame {
                axis: lane.axis,
                spans: vec![view],
                new_head: 0,
                room: 0,
                guard_recorded_ns: 0,
                guard_mcu_clock: 0,
            });
        }
        for &index in &indexes {
            self.lane_mut(index)?
                .seams
                .push_back(PendingSeam::Gap { at: anchor_clock });
        }
        self.buzz = Some(ActiveBuzz {
            lanes: indexes,
            end_clock,
        });
        self.send_frames_inner(self.mcu_id, &frames)
            .map_err(|e| self.escalate(e))
    }

    fn buzz_overlay_view(
        &self,
        lane: BuzzLane,
        profile: &Arc<BuzzProfile>,
        anchor_clock: u64,
        clock_freq_hz: f64,
    ) -> Result<ClockedMotorSpan, SendError> {
        let axis = lane.axis;
        let signal = MotorSpan::try_new(
            Arc::from([MotorGroup::Independent(MotorTerm {
                source_axis: usize::from(axis),
                axis: ContinuousAxis::Buzz {
                    base_position: 0.0,
                    sign: lane.sign,
                    profile: Arc::clone(profile),
                },
                scale: 1.0,
            })]),
            profile.t_start(),
            profile.t_end(),
            BUZZ_OVERLAY_MASK,
            u32::MAX,
            false,
        )
        .map_err(|error| {
            SendError::Fatal(format!(
                "sample endpoint mcu {} axis {axis}: the resonance sweep signal is not \
                 dispatchable: {error}",
                self.mcu_id
            ))
        })?;
        #[allow(clippy::cast_precision_loss)]
        let anchor = anchor_clock as f64;
        ClockedMotorSpan::try_new(
            Arc::new(signal),
            profile.t_start(),
            profile.t_end(),
            profile.t_start(),
            profile.t_end(),
            anchor,
            clock_freq_hz,
        )
        .map_err(|error| {
            SendError::Fatal(format!(
                "sample endpoint mcu {} axis {axis}: the resonance sweep view is not clockable: \
                 {error}",
                self.mcu_id
            ))
        })
    }

    fn buzz_played_out(&self, buzz: &ActiveBuzz) -> Result<bool, SendError> {
        for &index in &buzz.lanes {
            if !self.lane_idle_for_buzz(index)? {
                return Ok(false);
            }
            let axis = self.lane_ref(index)?.cfg.axis;
            if self.mcu_retired.playback_clock_of_axis(axis)? < buzz.end_clock {
                return Ok(false);
            }
        }
        Ok(true)
    }

    /// Whether the armed sweep has left the machine. An unarmed endpoint is
    /// trivially complete; the armed one releases its lanes only once the mcu's
    /// playback clock has carried past the sweep's end, and settles each lane
    /// on its own position so the next absolute span issues a fresh anchor
    /// instead of trying to abut the overlay.
    pub fn buzz_complete(&mut self) -> Result<bool, SendError> {
        self.reject_latched_fatal()?;
        let Some(buzz) = self.buzz.take() else {
            return Ok(true);
        };
        let settled = self
            .clock_now()
            .and_then(|(now, _)| Ok(self.buzz_played_out(&buzz)?.then_some(now)));
        let now = match settled {
            Ok(Some(now)) => now,
            Ok(None) => {
                self.buzz = Some(buzz);
                return Ok(false);
            }
            Err(error) => {
                self.buzz = Some(buzz);
                return Err(self.escalate(error));
            }
        };
        for &index in &buzz.lanes {
            let lane = self.lane_mut(index)?;
            let position = lane.position;
            lane.reset_to(position, now);
        }
        Ok(true)
    }

    /// The mcu's own executed position for `axis` in lane units, cross-checked
    /// against the host's lane counter. A quiesced lane whose two counters
    /// disagree has lost samples, and handing that position to the other
    /// transport would bake the loss into the machine position.
    ///
    /// A lane that still owes the mcu an anchor has never told it this origin —
    /// a host-side `set_position` re-origins the lane locally and the anchor
    /// carrying it rides the next run — so there the host's counter is the only
    /// truth and the readback would report the origin before it.
    pub fn executed_position(&self, axis: u8) -> Result<i64, SendError> {
        let index = self.lane_of(axis)?;
        let lane = self.lane_ref(index)?;
        if lane.wire_next_clock.is_none() {
            return Ok(lane.position);
        }
        let oid = lane.cfg.oid;
        let query = self.position_query.as_ref().ok_or_else(|| {
            SendError::Fatal(format!(
                "sample endpoint mcu {} axis {axis}: no sample_get_position readback",
                self.mcu_id
            ))
        })?;
        let (executed_clock, executed) = query(oid).map_err(|error| {
            SendError::Fatal(format!(
                "sample endpoint mcu {} axis {axis} oid {oid}: sample_get_position failed: \
                 {error}",
                self.mcu_id
            ))
        })?;
        let executed = i64::from(executed);
        if executed != lane.position {
            return Err(SendError::Fatal(format!(
                "sample endpoint mcu {} axis {axis} oid {oid}: mcu reported {executed} lane \
                 units at clock {executed_clock} but the host holds {}, delta {}",
                self.mcu_id,
                lane.position,
                executed - lane.position
            )));
        }
        Ok(executed)
    }

    /// Re-origin one lane so its next run anchors at the position the other
    /// transport just left the motor at. The lane owes the mcu a fresh
    /// `sample_anchor`, which carries the position absolutely.
    pub fn reset_axis_position(&mut self, axis: u8, position: i64) -> Result<(), SendError> {
        let index = self.lane_of(axis)?;
        let (now, _) = self.clock_now()?;
        self.backlog.retain(|out| out.lane != index);
        let lane = self.lane_mut(index)?;
        lane.abandon_views();
        lane.seams.clear();
        lane.cut = None;
        lane.reset_to(position, now);
        Ok(())
    }

    pub fn mark_reanchor(
        &mut self,
        axis: u8,
        at_start_clock: u64,
        epoch_freq: Option<f64>,
    ) -> Result<(), SendError> {
        let index = self.lane_of(axis)?;
        self.lane_mut(index)?.seams.push_back(PendingSeam::Cut {
            at: at_start_clock,
            epoch_freq,
        });
        Ok(())
    }

    pub fn mark_seam_gap(&mut self, axis: u8, at_start_clock: u64) -> Result<(), SendError> {
        let index = self.lane_of(axis)?;
        self.lane_mut(index)?
            .seams
            .push_back(PendingSeam::Gap { at: at_start_clock });
        Ok(())
    }

    pub fn send_frames(&mut self, mcu_id: u32, frames: &[AxisFrame]) -> Result<(), SendError> {
        self.reject_latched_fatal()?;
        self.send_frames_inner(mcu_id, frames)
            .map_err(|e| self.escalate(e))
    }

    fn next_seam(
        &self,
        index: usize,
        rest: &[ClockedMotorSpan],
    ) -> Result<Option<(PendingSeam, usize)>, SendError> {
        let lane = self.lane_ref(index)?;
        let Some(seam) = lane.seams.front().copied() else {
            return Ok(None);
        };
        let at = seam.at();
        Ok(rest
            .iter()
            .position(|view| view.start_clock >= at || view.end_clock > at)
            .map(|split| (seam, split)))
    }

    fn send_frames_inner(&mut self, mcu_id: u32, frames: &[AxisFrame]) -> Result<(), SendError> {
        if mcu_id != self.mcu_id {
            return Err(SendError::Fatal(format!(
                "sample endpoint for mcu {} received frames addressed to mcu {mcu_id}",
                self.mcu_id
            )));
        }
        let (now, freq) = self.clock_now()?;
        for frame in frames {
            let index = self.lane_of(frame.axis)?;
            if let Some(buzz) = self.buzz.as_ref() {
                if buzz.lanes.contains(&index) {
                    if let Some(absolute) =
                        frame.spans.iter().find(|view| view.signal.motor_mask == 0)
                    {
                        return Err(SendError::Fatal(format!(
                            "sample endpoint mcu {}: axis {} carries an armed resonance buzz, so \
                             the absolute span at clock {} cannot be staged — the sweep has to \
                             complete before the lane re-anchors",
                            self.mcu_id, frame.axis, absolute.start_clock
                        )));
                    }
                }
            }
            if self.lane_ref(index)?.cut.is_some() {
                let views = frame.spans.clone();
                if let Some(cut) = self.lane_mut(index)?.cut.as_mut() {
                    cut.held.extend(views);
                }
                continue;
            }
            let mut rest: &[ClockedMotorSpan] = &frame.spans;
            loop {
                let Some((seam, split)) = self.next_seam(index, rest)? else {
                    let tail = rest.to_vec();
                    self.lane_mut(index)?.views.extend(tail);
                    break;
                };
                let (head, tail) = rest.split_at(split);
                {
                    let head = head.to_vec();
                    let lane = self.lane_mut(index)?;
                    lane.views.extend(head);
                    lane.seams.pop_front();
                }
                if self.apply_seam(index, seam, tail, now, freq)? {
                    break;
                }
                rest = tail;
            }
        }
        self.drain_into_backlog(now, freq)?;
        self.flush(now, freq)
    }

    /// Returns whether the views past the seam must wait for a barrier.
    fn apply_seam(
        &mut self,
        index: usize,
        seam: PendingSeam,
        tail: &[ClockedMotorSpan],
        now: u64,
        freq: f64,
    ) -> Result<bool, SendError> {
        self.sample_lane_until(index, seam.at())?;
        let mut closed = Vec::new();
        self.lane_mut(index)?.close_run(&mut closed)?;
        self.emit_closed(index, closed)?;
        match seam {
            PendingSeam::Gap { at } => {
                tracing::info!(
                    subsystem = "motion",
                    event = "sample_seam_gap_accepted",
                    mcu = self.mcu_id,
                    lane = index,
                    at,
                    "[rejoin] forward seam gap sanctioned — the lane re-anchors"
                );
                let lane = self.lane_mut(index)?;
                lane.prev_sample = at;
                lane.origin_clock = Some(at);
                lane.resume_floor = at;
                lane.wire_next_clock = None;
                Ok(false)
            }
            PendingSeam::Cut { at, epoch_freq } => {
                let epoch_freq = epoch_freq.ok_or_else(|| {
                    SendError::Fatal(format!(
                        "sample endpoint mcu {} lane {index}: fresh epoch carried no clock \
                         slope; the lane cannot adopt the producer's timeline",
                        self.mcu_id
                    ))
                })?;
                self.drain_into_backlog(now, freq)?;
                let unretired = self.lane_ref(index)?.outstanding_runs(&self.mcu_retired)?;
                if unretired == 0 {
                    self.backlog.retain(|out| out.lane != index);
                    let lane = self.lane_mut(index)?;
                    lane.abandon_views();
                    lane.cfg.cycles_per_second = epoch_freq;
                    lane.sample_period_cycles = lane.cfg.sample_period_cycles()?;
                    let position = lane.position;
                    lane.reset_to(position, at);
                    return Ok(false);
                }
                if self.lane_ref(index)?.cut.is_some() {
                    return Err(SendError::Fatal(format!(
                        "sample endpoint mcu {} lane {index}: a reanchor cut is already awaiting \
                         mcu reconciliation at clock {at}",
                        self.mcu_id
                    )));
                }
                let oid = self.lane_ref(index)?.cfg.oid;
                let expected_position = self.lane_ref(index)?.position;
                let barrier = self.barriers.issue(oid);
                let lane = self.lane_mut(index)?;
                lane.abandon_views();
                lane.cut = Some(PendingSampleCut {
                    barrier,
                    cut_at: at,
                    epoch_freq,
                    expected_position,
                    held: tail.to_vec(),
                });
                self.queue_outbound(index, Outbound::Barrier(barrier), at);
                Ok(true)
            }
        }
    }

    fn sample_lane_until(&mut self, index: usize, sample_to: u64) -> Result<(), SendError> {
        let mut closed = Vec::new();
        self.lane_mut(index)?.sample_until(sample_to, &mut closed)?;
        self.emit_closed(index, closed)
    }

    fn emit_closed(&mut self, index: usize, closed: Vec<ClosedRun>) -> Result<(), SendError> {
        for run in closed {
            let oid = self.lane_ref(index)?.cfg.oid;
            let count = u8::try_from(run.positions.len()).map_err(|_| {
                SendError::Fatal(format!(
                    "sample endpoint mcu {}: a run of {} samples exceeds the wire count field",
                    self.mcu_id,
                    run.positions.len()
                ))
            })?;
            if run.anchor {
                self.queue_outbound(
                    index,
                    Outbound::Anchor {
                        oid,
                        clock: run.start_clock,
                        position: run.base,
                    },
                    run.start_clock,
                );
            }
            let mut data = vec![0u8; SAMPLE_RUN_DATA_MAX];
            let written = encode_deltas(run.base, &run.positions, &mut data).map_err(|e| {
                SendError::Fatal(format!(
                    "sample endpoint mcu {}: lane {index} run at {} does not encode: {} \
                     ({e:?}, fault {})",
                    self.mcu_id,
                    run.start_clock,
                    e.as_str(),
                    e.fault_code()
                ))
            })?;
            data.truncate(written);
            let frame = if run.overlay {
                Outbound::Overlay {
                    oid,
                    clock: run.start_clock,
                    interval: run.interval,
                    count,
                    data,
                }
            } else {
                Outbound::Run {
                    oid,
                    interval: run.interval,
                    count,
                    data,
                }
            };
            self.queue_outbound(index, frame, run.start_clock);
        }
        Ok(())
    }

    fn drain_into_backlog(&mut self, now: u64, freq: f64) -> Result<(), SendError> {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let lead = (freq * SEND_LEAD_SECONDS) as u64;
        let sample_to = now.saturating_add(lead);
        for index in 0..self.lanes.len() {
            self.sample_lane_until(index, sample_to)?;
            if self.lane_needs_its_open_run(index, now)? {
                let mut closed = Vec::new();
                self.lane_mut(index)?.close_run(&mut closed)?;
                self.emit_closed(index, closed)?;
            }
        }
        if self.backlog.len() > SAMPLE_BACKLOG_CEILING_RUNS {
            return Err(SendError::Fatal(format!(
                "sample endpoint mcu {}: {} outbound runs waiting on the lane window, above \
                 the {SAMPLE_BACKLOG_CEILING_RUNS} ceiling — the mcu is not consuming samples",
                self.mcu_id,
                self.backlog.len()
            )));
        }
        Ok(())
    }

    /// A partially filled run is worth holding open: every sample that joins it
    /// is one the mcu's shallow ring does not spend a slot on. It closes once
    /// its own window comes within the lane's send horizon, which is also how
    /// the last run of a move leaves.
    fn lane_needs_its_open_run(&self, index: usize, now: u64) -> Result<bool, SendError> {
        let lane = self.lane_ref(index)?;
        if lane.cut.is_some() {
            return Ok(false);
        }
        let Some(start) = lane.open_run_start() else {
            return Ok(false);
        };
        Ok(start <= now.saturating_add(lane.send_horizon_cycles()))
    }

    fn flush(&mut self, now: u64, freq: f64) -> Result<(), SendError> {
        self.order_backlog_by_deadline();
        let guard_secs = pump_past_guard_secs();
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let stale_by = (freq * guard_secs) as u64;

        self.audit_mcu_retirement()?;
        let mut room: Vec<u32> = Vec::with_capacity(self.lanes.len());
        let mut horizon: Vec<u64> = Vec::with_capacity(self.lanes.len());
        for lane in &mut self.lanes {
            lane.absorb_mcu_reports(&self.mcu_retired)?;
        }
        for lane in &self.lanes {
            let outstanding = lane.outstanding_runs(&self.mcu_retired)?;
            room.push(lane.cfg.ring_depth.saturating_sub(outstanding));
            horizon.push(now.saturating_add(lane.send_horizon_cycles()));
        }
        self.audit_lane_saturation(now, freq, &room)?;
        let mut parked = vec![false; self.lanes.len()];
        let mut selected = vec![false; self.backlog.len()];
        let mut burst: Vec<(&'static str, Vec<(String, ArgValue)>)> = Vec::new();
        let mut sent_runs: Vec<(usize, u64)> = Vec::new();
        let mut sent_barriers: Vec<BarrierId> = Vec::new();
        let mut stale: Option<SendError> = None;
        for (position, out) in self.backlog.iter().enumerate() {
            let lane_index = out.lane;
            let parked_lane = parked
                .get_mut(lane_index)
                .ok_or_else(|| self.no_lane(lane_index))?;
            if *parked_lane {
                continue;
            }
            if !matches!(out.frame, Outbound::Barrier(_))
                && out.start_clock.saturating_add(stale_by) < now
            {
                stale = Some(self.stale_fatal(out, now, freq, guard_secs));
                break;
            }
            let lane_room = room
                .get_mut(lane_index)
                .ok_or_else(|| self.no_lane(lane_index))?;
            let lane_horizon = horizon
                .get(lane_index)
                .copied()
                .ok_or_else(|| self.no_lane(lane_index))?;
            if out.start_clock > lane_horizon || (out.frame.needs_ring_room() && *lane_room == 0) {
                *parked_lane = true;
                continue;
            }
            if out.frame.occupies_lane_ring() {
                *lane_room -= 1;
                sent_runs.push((lane_index, out.end_clock));
            }
            if let Outbound::Barrier(id) = out.frame {
                sent_barriers.push(id);
            }
            burst.push(frame_args(&out.frame));
            selected[position] = true;
        }
        if !burst.is_empty() {
            (self.egress)(&burst)?;
            let mut position = 0;
            self.backlog.retain(|_| {
                let keep = !selected[position];
                position += 1;
                keep
            });
            for (lane_index, end_clock) in sent_runs {
                let lane = self.lane_mut(lane_index)?;
                lane.runs_sent = lane.runs_sent.wrapping_add(1);
                lane.in_flight_end_clocks.push_back(end_clock);
            }
            for id in sent_barriers {
                self.barriers.note_sent(id, now);
            }
        }
        if let Some(error) = stale {
            return Err(error);
        }
        self.post_heartbeat()
    }

    /// A lane whose ring the mcu reports full while runs queue behind it is
    /// either being drained or wedged. One ring residency is the whole time a
    /// healthy ring needs to turn over, so staying full past it means the
    /// delivery has stopped rather than slowed, and the stream is already
    /// unrecoverable — the silent multi-second drift this used to produce is a
    /// fault, not a symptom.
    fn audit_lane_saturation(
        &mut self,
        now: u64,
        freq: f64,
        room: &[u32],
    ) -> Result<(), SendError> {
        let mut waiting = vec![0u32; self.lanes.len()];
        for out in &self.backlog {
            if out.frame.needs_ring_room() {
                if let Some(count) = waiting.get_mut(out.lane) {
                    *count += 1;
                }
            }
        }
        let mut wedged: Option<(usize, u64, u32)> = None;
        for (index, lane) in self.lanes.iter_mut().enumerate() {
            let free = room.get(index).copied().unwrap_or(0);
            let queued = waiting.get(index).copied().unwrap_or(0);
            if free != 0 || queued == 0 {
                lane.saturated_since = None;
                continue;
            }
            let since = *lane.saturated_since.get_or_insert(now);
            if now.saturating_sub(since) > lane.send_horizon_cycles() {
                wedged = Some((index, now.saturating_sub(since), queued));
                break;
            }
        }
        let Some((index, saturated_for, queued)) = wedged else {
            return Ok(());
        };
        let lane = self.lane_ref(index)?;
        let credit = self.mcu_retired.of_axis(lane.cfg.axis)?;
        let playback_clock = self.mcu_retired.playback_clock_of_axis(lane.cfg.axis)?;
        let heartbeat_age_ms = 1e3 * now.saturating_sub(playback_clock) as f64 / freq;
        let saturated_for_ms = 1e3 * saturated_for as f64 / freq;
        Err(SendError::Fatal(format!(
            "sample endpoint mcu {} axis {}: lane ring has been full for {saturated_for_ms:.0} ms \
             with {queued} runs waiting — {} sent, mcu credits {credit} retired and reports \
             playback clock {playback_clock}, {heartbeat_age_ms:.0} ms behind the pump clock \
             {now}. The mcu has stopped consuming samples.",
            self.mcu_id, lane.cfg.axis, lane.runs_sent
        )))
    }

    /// The mcu cannot have retired a run the host never handed it. A count
    /// past what left the endpoint means the two ends disagree about which
    /// runs exist, and every clock the host derives from its own model is then
    /// fiction.
    fn audit_mcu_retirement(&self) -> Result<(), SendError> {
        for lane in &self.lanes {
            let retired = self.mcu_retired.of_axis(lane.cfg.axis)?;
            if lane.runs_sent.wrapping_sub(retired) > u32::MAX / 2 {
                return Err(SendError::Fatal(format!(
                    "sample endpoint mcu {} axis {}: the mcu reports {retired} retired sample \
                     runs but the host has only sent {}",
                    self.mcu_id, lane.cfg.axis, lane.runs_sent
                )));
            }
        }
        Ok(())
    }

    fn no_lane(&self, index: usize) -> SendError {
        SendError::Fatal(format!(
            "sample endpoint mcu {}: no lane {index}",
            self.mcu_id
        ))
    }

    fn stale_fatal(&self, out: &OutboundRun, now: u64, freq: f64, guard_secs: f64) -> SendError {
        #[allow(clippy::cast_precision_loss)]
        let late_us = (now - out.start_clock) as f64 * 1e6 / freq;
        let outstanding = self
            .lane_ref(out.lane)
            .and_then(|lane| lane.outstanding_runs(&self.mcu_retired));
        SendError::Fatal(format!(
            "sample endpoint mcu {}: {} at clock {} is {late_us:.0} us behind the projected \
             mcu clock {now}, past the {guard_secs} s floor margin. {SEND_LEAD_SECONDS} s of \
             lead was not delivered: {} runs backlogged, {outstanding:?} unretired on lane {}",
            self.mcu_id,
            out.frame.kind(),
            out.start_clock,
            self.backlog.len(),
            out.lane
        ))
    }

    fn post_heartbeat(&self) -> Result<(), SendError> {
        let consumed_counts = self.consumed_counts();
        let retired_counts = self.retired_counts();
        self.pump_control
            .send(PumpMsg::Heartbeat(HeartbeatMsg {
                mcu_id: self.mcu_id,
                axes: self.lanes.iter().map(|lane| lane.cfg.axis).collect(),
                consumed_counts: Some(consumed_counts),
                retired_counts,
                retired_by: super::messages::RetiredBy::Phase,
            }))
            .map_err(|_| {
                SendError::Fatal(format!(
                    "sample endpoint mcu {}: the pump control channel is closed",
                    self.mcu_id
                ))
            })
    }

    pub fn on_barrier_ack(&mut self, oid: u32, seq: u32) -> Result<(), SendError> {
        self.reject_latched_fatal()?;
        self.on_barrier_ack_inner(oid, seq)
            .map_err(|e| self.escalate(e))
    }

    fn on_barrier_ack_inner(&mut self, oid: u32, seq: u32) -> Result<(), SendError> {
        let mcu_id = self.mcu_id;
        if let Err(fault) = self.barriers.record_ack(oid, seq) {
            let detail = match fault {
                AckFault::Unknown => "no barrier was ever issued for this oid".to_string(),
                AckFault::Unissued { issued } => format!("the host has only issued up to {issued}"),
                AckFault::Regressed { high_water } => {
                    format!("it walks the high-water mark {high_water} backwards")
                }
            };
            return Err(SendError::Fatal(format!(
                "sample endpoint mcu {mcu_id}: barrier ack oid={oid} seq={seq} is bogus — \
                 {detail}. Ledger: {}",
                self.barriers.ledger_line()
            )));
        }
        self.barriers.prune_acked();
        let ready: Vec<usize> = self
            .lanes
            .iter()
            .enumerate()
            .filter(|(_, lane)| {
                lane.cut
                    .as_ref()
                    .is_some_and(|cut| self.barriers.is_acked(cut.barrier))
            })
            .map(|(index, _)| index)
            .collect();
        for index in ready {
            self.complete_cut(index)?;
        }
        Ok(())
    }

    fn complete_cut(&mut self, index: usize) -> Result<(), SendError> {
        let mcu_id = self.mcu_id;
        let cut = self.lane_mut(index)?.cut.take().ok_or_else(|| {
            SendError::Fatal(format!(
                "sample endpoint mcu {mcu_id}: cut completion on lane {index} has no pending cut"
            ))
        })?;
        let query = self.position_query.as_ref().ok_or_else(|| {
            SendError::Fatal(format!(
                "sample endpoint mcu {mcu_id}: the sent-run cut at {} has no \
                 sample_get_position readback",
                cut.cut_at
            ))
        })?;
        let (executed_clock, executed_position) = query(cut.barrier.oid).map_err(|error| {
            SendError::Fatal(format!(
                "sample endpoint mcu {mcu_id}: sample_get_position readback failed after \
                 barrier oid={} seq={}: {error}",
                cut.barrier.oid, cut.barrier.seq
            ))
        })?;
        if i64::from(executed_position) != cut.expected_position {
            return Err(SendError::Fatal(format!(
                "sample endpoint mcu {mcu_id} lane {index} oid {} reanchor position mismatch \
                 at clock {}: host expected {} lane units, mcu reported {executed_position} at \
                 clock {executed_clock}, delta {}",
                cut.barrier.oid,
                cut.cut_at,
                cut.expected_position,
                i64::from(executed_position) - cut.expected_position
            )));
        }
        self.backlog.retain(|out| out.lane != index);
        let lane = self.lane_mut(index)?;
        lane.cfg.cycles_per_second = cut.epoch_freq;
        lane.sample_period_cycles = lane.cfg.sample_period_cycles()?;
        lane.reset_to(i64::from(executed_position), cut.cut_at);
        lane.views.extend(cut.held);
        let (now, freq) = self.clock_now()?;
        self.drain_into_backlog(now, freq)?;
        self.flush(now, freq)
    }

    fn check_barrier_deadline(&mut self, now: u64, freq: f64) -> Result<(), SendError> {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let deadline_ticks = (freq * BARRIER_ACK_DEADLINE_SECONDS) as u64;
        let overdue = self.barriers.overdue(now, deadline_ticks);
        let Some((id, sent_clock)) = overdue.first().copied() else {
            return Ok(());
        };
        let lane = self
            .lanes
            .iter()
            .position(|lane| lane.cut.as_ref().is_some_and(|cut| cut.barrier == id));
        Err(SendError::Fatal(format!(
            "sample endpoint mcu {}: barrier oid={} seq={} sent at mcu clock {sent_clock} is \
             unacked {} ticks later, past the {BARRIER_ACK_DEADLINE_SECONDS} s ceiling — lane \
             {lane:?} would park forever. Ledger: {}",
            self.mcu_id,
            id.oid,
            id.seq,
            now.saturating_sub(sent_clock),
            self.barriers.ledger_line()
        )))
    }

    pub fn tick(&mut self) -> Result<(), SendError> {
        self.reject_latched_fatal()?;
        let (now, freq) = self.clock_now()?;
        self.check_barrier_deadline(now, freq)
            .and_then(|()| self.drain_into_backlog(now, freq))
            .and_then(|()| self.flush(now, freq))
            .map_err(|e| self.escalate(e))
    }
}

pub struct SamplePacer {
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl SamplePacer {
    pub fn spawn(endpoints: Vec<Arc<Mutex<SampleEndpoint>>>) -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let stop_for_thread = Arc::clone(&stop);
        let thread = std::thread::Builder::new()
            .name("sample-pacer".into())
            .spawn(move || {
                host_rt::thread_prio::elevate_current_thread(
                    host_rt::thread_prio::PUMP_RT_PRIORITY,
                    "sample-pacer",
                );
                let mut live = endpoints;
                while !stop_for_thread.load(Ordering::Relaxed) {
                    live.retain(|endpoint| match endpoint.lock_ok().tick() {
                        Ok(()) => true,
                        Err(SendError::Fatal(_)) => false,
                        Err(e) => {
                            tracing::error!(
                                subsystem = "pump",
                                event = "sample_pacer_error",
                                error = ?e,
                                "sample pacer tick failed"
                            );
                            true
                        }
                    });
                    if live.is_empty() {
                        return;
                    }
                    std::thread::sleep(SAMPLE_PACER_TICK);
                }
            })
            .expect("spawn sample-pacer thread");
        Self {
            stop,
            thread: Some(thread),
        }
    }
}

impl Drop for SamplePacer {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.thread.take() {
            let _ = handle.join();
        }
    }
}

#[cfg(test)]
#[path = "sample_sink_tests.rs"]
mod sample_sink_tests;
