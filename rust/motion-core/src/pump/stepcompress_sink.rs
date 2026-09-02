use super::barrier_ledger::{
    BarrierId, barrier_seq_after, barrier_seq_before, barrier_seq_covers, barrier_seq_seed,
};
use super::pump_loop::pump_past_guard_secs;
use super::{AxisFrame, HeartbeatMsg, PumpMsg, SendError};
use crate::lock_ext::LockExt;
use crate::mcu_config::{McuAxisConfig, StepcompressEncoder};
use crossbeam_channel::Sender;
use ethercat_rt::buzz::MAX_BUZZ_SLOTS;
use host_rt::host_io::McuHostIo;
use host_rt::host_io::parser::ArgValue;
use std::collections::VecDeque;
use std::ops::Range;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, Weak};
use std::thread::JoinHandle;
use std::time::Duration;
use step_shim::{MotorConfig, ShimError, StepEncoder, StepFrame, StepShim};
use trajectory::{BuzzProfile, ClockedMotorSpan, ContinuousAxis, MotorGroup, MotorSpan, MotorTerm};

pub const SHIM_RING_DEPTH: u32 = 64;
const RETIREMENT_BATCH: u32 = SHIM_RING_DEPTH;
const RETIREMENT_IDLE_TICKS: u32 = 10;

pub const MOVE_SLOT_RESERVE: u32 = 16;

/// The most wall time one send pass may spend in the step root search. Bulk
/// refills (rebuilding the 0.25s send lead after a reconcile cut) amortize
/// across pacer ticks; a pass that compressed the whole lead synchronously
/// consumed the resume volley's real delivery margin - the "Timer too
/// close" trips.
pub const DRAIN_PASS_BUDGET: std::time::Duration = std::time::Duration::from_millis(8);

pub const SEND_LEAD_SECONDS: f64 = 2.0 * (host_rt::host_io::rtt::MIN_RTO_MS as f64) / 1000.0;

pub const CONSUMED_MARGIN_SECONDS: f64 = 0.010;

/// Sending a motion frame with less execution margin than this is one host
/// hiccup away from the MCU's "Timer too close" shutdown; worth a warn even
/// when the send succeeds.
pub const SEND_MARGIN_WARN_FLOOR_SECS: f64 = 0.050;

/// How often the endpoint stamps its projected clock into a
/// `kalico_wire_probe`; the mcu's receipt delta measures host->mcu wire and
/// demux latency, the direction the barrier-ack clock echo cannot see.
pub const WIRE_PROBE_INTERVAL_SECS: f64 = 0.050;

/// A barrier the mcu never acks would otherwise park the retirement cohort —
/// and every drain waiting on it — forever. One transport RTO ceiling past the
/// send is far beyond any legitimate wait: a barrier only leaves the backlog
/// when its clock is within [`SEND_LEAD_SECONDS`] of the mcu clock, so the mcu
/// reaches it within that lead plus its own queue drain. Measured on the mcu
/// clock, not the host's: a slow mcu owes its acks against the clock it is
/// actually running, and the endpoint paces every send against that same
/// projection.
pub const BARRIER_ACK_DEADLINE_SECONDS: f64 =
    host_rt::host_io::rtt::MAX_RTO.as_millis() as f64 / 1000.0;

/// The simulated mcu's clock is driven by the harness and stops whenever the
/// world is paused, so the wire's RTO ceiling is not a bound on its acks.
const SIM_BARRIER_ACK_DEADLINE_SECONDS: f64 = 60.0;

/// A classic stepper spends two scheduler events on every step: the pulse at
/// the step clock and the unstep `step_pulse_ticks` later, from which
/// `stepper_load_next` re-arms one more `step_pulse_ticks` out. A queued move
/// whose first step lands inside that window is loaded behind the pending
/// unstep — see `src/stepper_classic.c`.
pub const STEP_REARM_PULSES: u64 = 2;

pub const BACKLOG_CEILING_FRAMES: usize = 8192;

/// How often the pacer tops the mcu's move queue back up to
/// [`SEND_LEAD_SECONDS`]. Every tick is a flush, so a shorter interval buys
/// nothing but fragments the burst.
pub const PACER_TICK: Duration = Duration::from_millis(10);

pub type ClockSource = Arc<dyn Fn(u32) -> Option<(u64, f64)> + Send + Sync>;
/// A burst of frames leaves the endpoint as one call so the transport can
/// pack it into full Klipper message blocks; framing each frame on its own
/// block spends the protocol's sixteen sequence numbers a command at a time.
pub type FrameEgress =
    Arc<dyn Fn(&[(&'static str, Vec<(String, ArgValue)>)]) -> Result<(), SendError> + Send + Sync>;

pub type StepCountQuery = Arc<dyn Fn(u32) -> Result<i64, String> + Send + Sync>;

const STEP_COUNT_QUERY_TIMEOUT: Duration = Duration::from_millis(250);

fn host_io_step_count_query(mcu_id: u32, host_io: Weak<McuHostIo>) -> StepCountQuery {
    Arc::new(move |oid| {
        let io = host_io.upgrade().ok_or_else(|| {
            format!("stepcompress mcu {mcu_id}: McuHostIo detached during step count readback")
        })?;
        let params = io
            .call_args(
                "stepper_get_position",
                &[("oid".to_string(), ArgValue::Int(i64::from(oid)))],
                "stepper_position",
                STEP_COUNT_QUERY_TIMEOUT,
            )
            .map_err(|e| {
                format!(
                    "stepper_get_position failed for stepcompress mcu {mcu_id} oid {oid}: {e:?}"
                )
            })?;
        params.try_get_i32("pos").map(i64::from).ok_or_else(|| {
            format!(
                "stepper_position from stepcompress mcu {mcu_id} oid {oid} carries no `pos` field"
            )
        })
    })
}

pub fn host_io_egress(mcu_id: u32, host_io: Weak<McuHostIo>) -> FrameEgress {
    Arc::new(move |frames: &[(&'static str, Vec<(String, ArgValue)>)]| {
        let io = host_io.upgrade().ok_or_else(|| {
            SendError::Fatal(format!("McuHostIo for stepcompress mcu {mcu_id} detached"))
        })?;
        io.send_args_batch(frames)
            .map_err(|e| SendError::Transient(format!("stepcompress mcu {mcu_id}: {e:?}")))
    })
}

pub fn build_endpoint(
    cfg: &McuAxisConfig,
    host_io: Weak<McuHostIo>,
    pump_control: Sender<PumpMsg>,
    measured_clock_freq: f64,
    clock_of: ClockSource,
) -> Result<StepcompressEndpoint, String> {
    if !measured_clock_freq.is_finite() || measured_clock_freq <= 0.0 {
        return Err(format!(
            "stepcompress mcu {}: clock estimate {measured_clock_freq} Hz is not a positive \
             rate; every span is clocked on the slope the host projects starts with",
            cfg.mcu_id
        ));
    }
    if cfg.move_queue_slots <= MOVE_SLOT_RESERVE {
        return Err(format!(
            "stepcompress mcu {}: mcu advertised {} move-queue slots, which leaves nothing \
             after the {MOVE_SLOT_RESERVE}-slot reserve for klippy's own scheduled commands",
            cfg.mcu_id, cfg.move_queue_slots
        ));
    }
    let budget = cfg.move_queue_slots - MOVE_SLOT_RESERVE;
    let cycles_per_second = measured_clock_freq;
    let classic_encoder = if cfg
        .stepcompress_encoders
        .contains(&StepcompressEncoder::Classic)
    {
        let max_error_ticks = (cfg.stepcompress_max_error_secs * measured_clock_freq).round();
        if !max_error_ticks.is_finite()
            || max_error_ticks < 1.0
            || max_error_ticks > u32::MAX as f64
        {
            return Err(format!(
                "stepcompress mcu {}: classic encoder max_error {} s at \
                 {measured_clock_freq} Hz does not resolve to a tick budget in [1, {}]",
                cfg.mcu_id,
                cfg.stepcompress_max_error_secs,
                u32::MAX
            ));
        }
        Some(StepEncoder::Classic {
            max_error_ticks: max_error_ticks as u32,
        })
    } else {
        None
    };
    let motor_count: usize = cfg
        .motor_counts
        .iter()
        .map(|&count| usize::from(count))
        .sum();
    let mut motors = Vec::with_capacity(motor_count);
    let mut pulse_lanes = Vec::with_capacity(motor_count);
    for (lane, &axis) in cfg.axes.iter().enumerate() {
        if !cfg.pulse_capable(lane) {
            continue;
        }
        let velocity_ceiling = cfg.motor_velocity_ceiling(axis);
        for motor in cfg.motor_range(lane) {
            let microstep_distance = cfg.microstep_distance[motor];
            let steps_per_second = velocity_ceiling / microstep_distance;
            if steps_per_second >= cycles_per_second {
                return Err(format!(
                    "stepcompress mcu {} axis {axis} motor {motor}: {velocity_ceiling} mm/s over \
                     {microstep_distance} mm microsteps needs {steps_per_second} steps per \
                     second, which no longer resolves on a {cycles_per_second} Hz step clock",
                    cfg.mcu_id
                ));
            }
            let step_pulse_seconds = cfg.step_pulse_seconds[motor];
            if !step_pulse_seconds.is_finite() || step_pulse_seconds < 0.0 {
                return Err(format!(
                    "stepcompress mcu {} axis {axis} motor {motor}: step pulse width \
                     {step_pulse_seconds} s is not a non-negative duration",
                    cfg.mcu_id
                ));
            }
            motors.push(MotorConfig {
                oid: cfg.stepper_oids[motor],
                microstep_distance,
                invert_dir: cfg.invert_dir[motor],
                cycles_per_second,
                min_rearm_cycles: STEP_REARM_PULSES
                    * (step_pulse_seconds * cycles_per_second) as u64,
                encoder: match cfg.stepcompress_encoders[motor] {
                    StepcompressEncoder::HighPrecision => StepEncoder::HighPrecision,
                    StepcompressEncoder::Classic => {
                        classic_encoder.expect("validated for every classic motor")
                    }
                },
            });
            pulse_lanes.push(StepLaneConfig {
                axis,
                oid: cfg.stepper_oids[motor],
            });
        }
    }
    if motors.is_empty() {
        return Err(format!(
            "stepcompress mcu {}: no pulse-capable lanes to stream to; a stepcompress endpoint \
             was built for an mcu whose every lane is phase-only",
            cfg.mcu_id
        ));
    }
    let simulated = std::env::var_os("MCU_SIM_SOCK_DIR").is_some();
    let query = host_io_step_count_query(cfg.mcu_id, host_io.clone());
    let link_health = host_io.upgrade().map(|io| io.link_health());
    let mut endpoint = StepcompressEndpoint::new(
        cfg.mcu_id,
        StepShim::new(motors, SHIM_RING_DEPTH),
        &pulse_lanes,
        host_io_egress(cfg.mcu_id, host_io),
        pump_control,
        clock_of,
        budget,
        query,
        link_health,
        if simulated {
            SIM_BARRIER_ACK_DEADLINE_SECONDS
        } else {
            BARRIER_ACK_DEADLINE_SECONDS
        },
    )?;
    if !simulated {
        endpoint.set_drain_pass_budget(DRAIN_PASS_BUDGET);
    }
    Ok(endpoint)
}

struct InFlight {
    reclaim_clock: u64,
}

struct PendingRetire {
    waits: Vec<BarrierId>,
    counts: Vec<u32>,
}

struct PendingCut {
    barrier: BarrierId,
    cut_at: u64,
    resume_clock: u64,
    epoch_freq: f64,
    expected_count: i64,
    held: Vec<ClockedMotorSpan>,
}

struct StepBuzz {
    signals: Vec<Option<Arc<MotorSpan>>>,
    anchor_clock_exact: f64,
    cycles_per_second: f64,
    stream_t_origin: f64,
    next_stream_t: f64,
    stream_t_end: f64,
}

impl StepBuzz {
    fn clock_at(&self, stream_t: f64) -> f64 {
        self.anchor_clock_exact + (stream_t - self.stream_t_origin) * self.cycles_per_second
    }
}

struct SentBarrier {
    id: BarrierId,
    sent_clock: u64,
}

/// The mcu's projected step clock and the slope it runs on, sampled together.
/// Every seconds<->ticks conversion in the endpoint goes through here, so a
/// call site cannot pair one pass's `now` with another's rate.
#[derive(Clone, Copy)]
struct McuClock {
    now: u64,
    freq: f64,
}

impl McuClock {
    fn ticks(self, secs: f64) -> u64 {
        (self.freq * secs) as u64
    }

    fn secs(self, ticks: u64) -> f64 {
        ticks as f64 / self.freq
    }

    fn secs_signed(self, ticks: i64) -> f64 {
        ticks as f64 / self.freq
    }

    fn micros(self, ticks: u64) -> f64 {
        ticks as f64 * 1e6 / self.freq
    }

    /// How far ahead of the mcu the endpoint keeps its move queue stocked.
    fn lead_horizon(self) -> u64 {
        self.now.saturating_add(self.ticks(SEND_LEAD_SECONDS))
    }
}

/// The endpoint's two clock maps, reachable from an integration test without
/// publishing the pacing state they belong to.
#[cfg(any(test, feature = "test-support"))]
pub mod clock_probe {
    use super::{McuClock, StepBuzz};

    #[must_use]
    pub fn mcu_ticks(freq: f64, secs: f64) -> u64 {
        McuClock { now: 0, freq }.ticks(secs)
    }

    #[must_use]
    pub fn mcu_secs(freq: f64, ticks: u64) -> f64 {
        McuClock { now: 0, freq }.secs(ticks)
    }

    #[must_use]
    pub fn buzz_clock_at(
        anchor_clock_exact: f64,
        cycles_per_second: f64,
        stream_t_origin: f64,
        stream_t: f64,
    ) -> f64 {
        StepBuzz {
            signals: Vec::new(),
            anchor_clock_exact,
            cycles_per_second,
            stream_t_origin,
            next_stream_t: stream_t_origin,
            stream_t_end: stream_t_origin,
        }
        .clock_at(stream_t)
    }
}

/// Whether a drain may open a fresh retirement cohort. Deferring is what the
/// seam ladder needs: the spans it is mid-way through pushing are not a
/// retirable unit until the seam is applied.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Retire {
    Publish,
    Defer,
}

enum Outbound {
    Step(StepFrame),
    Barrier(BarrierId),
}

impl Outbound {
    const fn kind(&self) -> OutboundKind {
        match self {
            Self::Step(StepFrame::ResetStepClock { .. }) => OutboundKind::Reset,
            Self::Step(StepFrame::SetNextStepDir { .. }) => OutboundKind::Dir,
            Self::Step(StepFrame::QueueStep { .. }) => OutboundKind::Step,
            Self::Step(StepFrame::QueueStepHp { .. }) => OutboundKind::StepHp,
            Self::Barrier(_) => OutboundKind::Barrier,
        }
    }
}

/// What the mcu does with an outbound frame, classified once per frame so the
/// send pass reads the answer instead of re-deriving it from the variant.
#[derive(Clone, Copy, PartialEq, Eq)]
enum OutboundKind {
    Reset,
    Dir,
    Step,
    StepHp,
    Barrier,
}

impl OutboundKind {
    const fn consumes_move_slot(self) -> bool {
        matches!(self, Self::Step | Self::StepHp | Self::Barrier)
    }

    /// Barriers are control receipts: their start clock is the last sent step
    /// clock, in the past by the delivery lead by design, so the past-clock
    /// guard does not apply to them. Every other frame is motion the mcu
    /// schedules and must therefore be punctual — `reset_step_clock` included,
    /// since the mcu accepts a past reset clock silently and then fires its
    /// first `stepper_event` immediately, starving the scheduler.
    const fn is_motion(self) -> bool {
        !matches!(self, Self::Barrier)
    }

    const fn tracks_sent_boundary(self) -> bool {
        matches!(self, Self::Step | Self::StepHp)
    }

    /// A late continuation is harmless while the mcu still holds queued motion
    /// past `now` on that lane: the stepper is armed and executing, so a
    /// catch-up `queue_step` extends the queue. Only a volley head re-arms an
    /// idle timer, which is what the mcu shuts down on.
    const fn rearms_an_idle_timer(self) -> bool {
        matches!(self, Self::Reset)
    }

    const fn wire_name(self) -> &'static str {
        match self {
            Self::Reset => "reset_step_clock",
            Self::Dir => "set_next_step_dir",
            Self::Step => "queue_step",
            Self::StepHp => "queue_step_hp",
            Self::Barrier => "stepcompress_barrier",
        }
    }
}

struct OutboundFrame {
    frame: Outbound,
    lane: usize,
    start_clock: u64,
    end_clock: u64,
    enqueue_order: u64,
    queued_clock: u64,
}

/// A seam the endpoint must handle when the marked piece reaches it, in
/// stream order. `Cut` is a fresh anchor: halt the shim, re-slope, reset the
/// mcu step clock. `Gap` is a rejoin: a stationary stream-time hole — no mcu
/// frames, only a sanctioned forward seam jump in the shim.
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

/// Which logical axis a motor belongs to and the oid it answers on. One entry
/// per pulse-capable motor, in the order the shim holds them.
#[derive(Clone, Copy, Debug)]
pub struct StepLaneConfig {
    pub axis: usize,
    pub oid: u32,
}

/// Everything the endpoint tracks per motor. The motor index is the shim's
/// motor index, so `lanes[i]`, `shim` motor `i` and `consumed_counts()[i]`
/// are the same lane. Absences that gate a fatal stay `Option`: a lane with
/// no `step_clock` has no anchored mcu timeline, and a lane with no
/// `next_barrier_seq` never issued a barrier.
struct Lane {
    axis: usize,
    oid: u32,
    step_clock: Option<u64>,
    last_sent_boundary: Option<u64>,
    seams: VecDeque<PendingSeam>,
    pending_cut: Option<PendingCut>,
    frozen: bool,
    next_barrier_seq: Option<u32>,
    acked_barrier_seq: Option<u32>,
    retirement_bias: u32,
    commanded_base: f64,
}

impl Lane {
    fn new(cfg: StepLaneConfig) -> Self {
        Self {
            axis: cfg.axis,
            oid: cfg.oid,
            step_clock: None,
            last_sent_boundary: None,
            seams: VecDeque::new(),
            pending_cut: None,
            frozen: false,
            next_barrier_seq: None,
            acked_barrier_seq: None,
            retirement_bias: 0,
            commanded_base: 0.0,
        }
    }
}

pub struct StepcompressEndpoint {
    mcu_id: u32,
    shim: StepShim,
    lanes: Vec<Lane>,
    /// One contiguous motor run per logical axis, in lane order. Built once,
    /// so routing an axis frame is a lookup rather than a filter.
    axis_runs: Vec<(usize, Range<usize>)>,
    egress: FrameEgress,
    pump_control: Sender<PumpMsg>,
    clock_of: ClockSource,
    budget: u32,
    backlog: VecDeque<OutboundFrame>,
    next_outbound_order: u64,
    in_flight: Vec<InFlight>,
    step_count_query: StepCountQuery,
    last_wire_probe_clock: u64,
    pending_retire: Option<PendingRetire>,
    deferred_retirement: bool,
    retirement_idle_ticks: u32,
    published: Vec<u32>,
    cohort_counts: Vec<u32>,
    barrier_seq_seed: u32,
    sent_barriers: VecDeque<SentBarrier>,
    barrier_ack_deadline_secs: f64,
    latched_fatal: Option<String>,
    buzz: Option<StepBuzz>,
    drain_pass_budget: Option<std::time::Duration>,
    link_health: Option<Arc<host_rt::host_io::link_health::LinkHealth>>,
}

fn shim_error_to_send_error(mcu_id: u32, error: ShimError) -> SendError {
    match error {
        ShimError::QueueFull { motor } => SendError::Transient(format!(
            "stepcompress mcu {mcu_id} motor {motor}: shim span queue full"
        )),
        other => SendError::Fatal(format!("stepcompress mcu {mcu_id}: {other:?}")),
    }
}

const fn step_frame_oid(frame: &StepFrame) -> u32 {
    match *frame {
        StepFrame::QueueStep { oid, .. }
        | StepFrame::QueueStepHp { oid, .. }
        | StepFrame::SetNextStepDir { oid, .. }
        | StepFrame::ResetStepClock { oid, .. } => oid,
    }
}

fn frame_args(frame: &Outbound) -> (&'static str, Vec<(String, ArgValue)>) {
    let args = match *frame {
        Outbound::Step(StepFrame::QueueStep {
            oid,
            interval,
            count,
            add,
        }) => vec![
            ("oid".to_string(), ArgValue::Int(i64::from(oid))),
            ("interval".to_string(), ArgValue::Int(i64::from(interval))),
            ("count".to_string(), ArgValue::Int(i64::from(count))),
            ("add".to_string(), ArgValue::Int(i64::from(add))),
        ],
        Outbound::Step(StepFrame::QueueStepHp {
            oid,
            interval,
            count,
            add,
            add2,
            shift,
            ..
        }) => vec![
            ("oid".to_string(), ArgValue::Int(i64::from(oid))),
            ("interval".to_string(), ArgValue::Int(i64::from(interval))),
            ("count".to_string(), ArgValue::Int(i64::from(count))),
            ("add".to_string(), ArgValue::Int(i64::from(add))),
            ("add2".to_string(), ArgValue::Int(i64::from(add2))),
            ("shift".to_string(), ArgValue::Int(i64::from(shift))),
        ],
        Outbound::Step(StepFrame::SetNextStepDir { oid, dir }) => vec![
            ("oid".to_string(), ArgValue::Int(i64::from(oid))),
            ("dir".to_string(), ArgValue::Int(i64::from(dir))),
        ],
        Outbound::Step(StepFrame::ResetStepClock { oid, clock }) => vec![
            ("oid".to_string(), ArgValue::Int(i64::from(oid))),
            ("clock".to_string(), ArgValue::Int(i64::from(clock))),
        ],
        Outbound::Barrier(BarrierId { oid, seq }) => vec![
            ("oid".to_string(), ArgValue::Int(i64::from(oid))),
            ("seq".to_string(), ArgValue::Int(i64::from(seq))),
        ],
    };
    (frame.kind().wire_name(), args)
}

fn expand_clock32(reference: u64, low: u32) -> u64 {
    let delta = i64::from(low.wrapping_sub(reference as u32) as i32);
    (reference as i64).saturating_add(delta).max(0) as u64
}

fn queue_step_span(interval: u32, count: u16, add: i16) -> i64 {
    let count = i64::from(count);
    i64::from(interval) * count + i64::from(add) * count * (count - 1) / 2
}

#[allow(clippy::cast_possible_truncation)]
fn stepper_sel_from_mask(mask: u8) -> Result<u8, ()> {
    if mask == 0 {
        return Ok(0);
    }
    if mask.count_ones() != 1 {
        return Err(());
    }
    Ok(mask.trailing_zeros() as u8 + 1)
}

/// Where the clock a `StepFrame` is paced against comes from. A
/// `set_next_step_dir` carries no clock on the wire — the mcu latches it on
/// receipt and applies it to the next `queue_step` — so the only coherent
/// guard clock for it is the clock of the step run it heads.
enum FrameClock {
    Stamped { start: u64, end: u64 },
    HeadsTheNextRun { dir: u8 },
}

impl StepcompressEndpoint {
    pub fn new(
        mcu_id: u32,
        shim: StepShim,
        lanes: &[StepLaneConfig],
        egress: FrameEgress,
        pump_control: Sender<PumpMsg>,
        clock_of: ClockSource,
        budget: u32,
        step_count_query: StepCountQuery,
        link_health: Option<Arc<host_rt::host_io::link_health::LinkHealth>>,
        barrier_ack_deadline_secs: f64,
    ) -> Result<Self, String> {
        let published = shim.consumed_counts();
        let cohort_counts = published.clone();
        let mut axis_runs: Vec<(usize, Range<usize>)> = Vec::new();
        let mut first = 0;
        while first < lanes.len() {
            let axis = lanes[first].axis;
            let end = lanes[first..]
                .iter()
                .position(|lane| lane.axis != axis)
                .map_or(lanes.len(), |offset| first + offset);
            if let Some((_, run)) = axis_runs.iter().find(|(seen, _)| *seen == axis) {
                return Err(format!(
                    "stepcompress mcu {mcu_id}: axis {axis} owns two separate motor runs \
                     ({run:?} and {:?}); one logical axis must occupy one contiguous run or \
                     its frame routing and its retirement credit disagree",
                    first..end
                ));
            }
            axis_runs.push((axis, first..end));
            first = end;
        }
        Ok(Self {
            mcu_id,
            shim,
            lanes: lanes.iter().copied().map(Lane::new).collect(),
            axis_runs,
            egress,
            pump_control,
            clock_of,
            budget,
            backlog: VecDeque::new(),
            next_outbound_order: 0,
            in_flight: Vec::new(),
            step_count_query,
            last_wire_probe_clock: 0,
            pending_retire: None,
            deferred_retirement: false,
            retirement_idle_ticks: 0,
            published,
            cohort_counts,
            barrier_seq_seed: barrier_seq_seed(),
            sent_barriers: VecDeque::new(),
            barrier_ack_deadline_secs,
            latched_fatal: None,
            buzz: None,
            drain_pass_budget: None,
            link_health,
        })
    }

    fn fatal(&self, message: &str) -> SendError {
        SendError::Fatal(format!("stepcompress mcu {}: {message}", self.mcu_id))
    }

    fn motor_fatal(&self, motor: usize, message: &str) -> SendError {
        SendError::Fatal(format!(
            "stepcompress mcu {} motor {motor}: {message}",
            self.mcu_id
        ))
    }

    fn axis_fatal(&self, axis: u8, message: &str) -> SendError {
        SendError::Fatal(format!(
            "stepcompress mcu {} axis {axis}: {message}",
            self.mcu_id
        ))
    }

    fn link_line(&self) -> String {
        self.link_health
            .as_ref()
            .map_or_else(|| "link: no vitals attached".to_string(), |l| l.describe())
    }

    /// Cap the wall time one send pass may spend in the step root search.
    /// Production sets [`DRAIN_PASS_BUDGET`]; tests leave the search
    /// unbounded so mock-clock scenarios drain deterministically.
    pub fn set_drain_pass_budget(&mut self, budget: std::time::Duration) {
        self.drain_pass_budget = Some(budget);
    }

    #[must_use]
    pub fn accepts_buzz_mask(&self, axis_mask: u8) -> bool {
        self.lanes
            .iter()
            .any(|lane| lane.axis < 8 && axis_mask & (1 << lane.axis) != 0)
    }

    /// How many motor slots one arming of this endpoint would occupy.
    #[must_use]
    pub fn buzz_slot_count(&self) -> usize {
        self.lanes.len()
    }

    /// Nothing staged, nothing on the wire, nothing the shim still owes.
    fn transport_idle(&self) -> bool {
        self.backlog.is_empty()
            && self.in_flight.is_empty()
            && self.shim.queued_spans() == 0
            && self.shim.pending_roots() == 0
    }

    #[must_use]
    pub fn buzz_complete(&self) -> bool {
        self.buzz.is_none()
            && self.transport_idle()
            && self.pending_retire.is_none()
            && self.sent_barriers.is_empty()
            && !self.deferred_retirement
    }

    #[allow(clippy::cast_precision_loss)]
    pub fn arm_buzz(
        &mut self,
        axis_mask: u8,
        sign_mask: u8,
        profile: &Arc<BuzzProfile>,
        start_clock: u64,
    ) -> Result<(), SendError> {
        if self.buzz.is_some() {
            return Err(self.fatal("resonance buzz is already active"));
        }
        if !self.transport_idle() || self.pending_retire.is_some() {
            return Err(self.fatal("resonance buzz rejected while trajectory remains queued"));
        }
        if self.lanes.len() > MAX_BUZZ_SLOTS {
            return Err(self.fatal(&format!(
                "{} motors exceed the {MAX_BUZZ_SLOTS}-motor buzz limit",
                self.lanes.len()
            )));
        }
        let mut signals: Vec<Option<Arc<MotorSpan>>> = Vec::with_capacity(self.lanes.len());
        let mut driven = 0usize;
        for motor in 0..self.lanes.len() {
            let axis = self.lanes[motor].axis;
            if axis >= 8 || axis_mask & (1 << axis) == 0 {
                signals.push(None);
                continue;
            }
            driven += 1;
            let sign = if sign_mask & (1 << axis) == 0 {
                1.0
            } else {
                -1.0
            };
            let signal =
                buzz_signal(profile, self.lanes[motor].commanded_base, sign).map_err(|error| {
                    self.fatal(&format!(
                        "resonance buzz signal is not dispatchable: {error}"
                    ))
                })?;
            signals.push(Some(Arc::new(signal)));
        }
        if driven == 0 {
            return Err(self.fatal(&format!(
                "resonance buzz axis mask 0x{axis_mask:02x} selects no motor"
            )));
        }
        for motor in 0..self.lanes.len() {
            if signals[motor].is_some() {
                self.shim
                    .detach_span_seam(motor)
                    .map_err(|e| shim_error_to_send_error(self.mcu_id, e))?;
            }
        }
        self.buzz = Some(StepBuzz {
            signals,
            anchor_clock_exact: start_clock as f64,
            cycles_per_second: self.shim.motor_cycles_per_second(0),
            stream_t_origin: profile.t_start(),
            next_stream_t: profile.t_start(),
            stream_t_end: profile.t_end(),
        });
        Ok(())
    }

    /// The contiguous motor run one logical axis owns.
    fn motors_of(&self, axis: u8) -> Result<Range<usize>, SendError> {
        match self
            .axis_runs
            .iter()
            .find(|(configured, _)| *configured == usize::from(axis))
        {
            Some((_, run)) => Ok(run.clone()),
            None => Err(self.fatal(&format!(
                "frame for axis {axis} but configured axes are {:?}",
                self.lanes.iter().map(|lane| lane.axis).collect::<Vec<_>>()
            ))),
        }
    }

    fn lane_of_oid(&self, oid: u32) -> Option<usize> {
        self.lanes.iter().position(|lane| lane.oid == oid)
    }

    fn motor_executed_position(&self, axis: u8, motor: usize) -> Result<i64, SendError> {
        let oid = self.lanes[motor].oid;
        let wire_count = (self.step_count_query)(oid).map_err(|error| {
            SendError::Fatal(format!(
                "stepcompress mcu {} axis {axis} oid {oid}: stepper_get_position failed: {error}",
                self.mcu_id
            ))
        })?;
        let executed = if self.shim.invert_dir(motor) {
            wire_count.saturating_neg()
        } else {
            wire_count
        };
        let commanded = self.shim.commanded_steps(motor);
        if executed != commanded {
            return Err(SendError::Fatal(format!(
                "stepcompress mcu {} axis {axis} oid {oid}: mcu executed {executed} trajectory \
                 steps (wire {wire_count}) but the host commanded {commanded}, delta {}",
                self.mcu_id,
                executed - commanded
            )));
        }
        Ok(executed)
    }

    fn queue_outbound(
        &mut self,
        lane: usize,
        frame: Outbound,
        start_clock: u64,
        end_clock: u64,
        queued: u64,
    ) {
        let enqueue_order = self.next_outbound_order;
        self.next_outbound_order = self
            .next_outbound_order
            .checked_add(1)
            .expect("stepcompress outbound order exhausted");
        self.backlog.push_back(OutboundFrame {
            frame,
            lane,
            start_clock,
            end_clock,
            enqueue_order,
            queued_clock: queued,
        });
    }

    fn order_backlog_by_deadline(&mut self) {
        let backlog = self.backlog.make_contiguous();
        if backlog.windows(2).any(|pair| {
            (pair[0].start_clock, pair[0].enqueue_order)
                > (pair[1].start_clock, pair[1].enqueue_order)
        }) {
            backlog.sort_unstable_by_key(|out| (out.start_clock, out.enqueue_order));
        }
    }

    pub fn ring_depth(&self) -> u32 {
        self.shim.queue_depth()
    }

    /// Whether this endpoint owns `axis` as one of its pulse lanes. The pump
    /// routes by this, so a lane's transport is a membership fact rather than
    /// a configured mode.
    pub fn drives_axis(&self, axis: u8) -> bool {
        self.axis_runs
            .iter()
            .any(|(configured, _)| *configured == usize::from(axis))
    }

    pub fn owns_oid(&self, oid: u32) -> bool {
        self.lane_of_oid(oid).is_some()
    }

    pub fn shim_mut(&mut self) -> &mut StepShim {
        &mut self.shim
    }

    pub fn reset_position(&mut self, pos_steps: &[i64]) -> Result<(), SendError> {
        if pos_steps.len() != self.lanes.len() {
            return Err(self.fatal(&format!(
                "position seed has {} entries for {} configured axes",
                pos_steps.len(),
                self.lanes.len()
            )));
        }
        self.abort_outbound();
        for (motor, &count) in pos_steps.iter().enumerate() {
            self.reset_motor_position(motor, count)
                .map_err(SendError::Fatal)?;
            let mcu_count = if self.shim.invert_dir(motor) {
                -count
            } else {
                count
            };
            self.seed_mcu_position(self.lanes[motor].oid, mcu_count)?;
        }
        self.post_heartbeat()
    }

    /// The mcu counts the step pulses it executed; the reconcile after an
    /// endstop trip compares that count against the host's own absolute
    /// bookkeeping, so both must share an origin.
    fn seed_mcu_position(&self, oid: u32, count: i64) -> Result<(), SendError> {
        let count = i32::try_from(count).map_err(|_| {
            self.fatal(&format!(
                "position seed {count} for oid {oid} does not fit the mcu's 32-bit step counter"
            ))
        })?;
        (self.egress)(&[(
            "stepcompress_set_position",
            vec![
                ("oid".to_string(), ArgValue::Int(i64::from(oid))),
                ("pos".to_string(), ArgValue::Int(i64::from(count))),
            ],
        )])
    }

    fn sync_retirement_baseline(&mut self) {
        self.published = self.shim.consumed_counts();
        self.cohort_counts.clone_from(&self.published);
        self.pending_retire = None;
        self.deferred_retirement = false;
    }

    pub fn reset_motor_position(&mut self, motor: usize, count: i64) -> Result<(), String> {
        self.shim
            .halt_at(motor, u64::MAX)
            .map_err(|e| format!("stepcompress mcu {}: {e}", self.mcu_id))?;
        self.shim.reset_position(motor, count);
        self.lanes[motor].commanded_base = self.shim.commanded_position(motor);
        self.sync_retirement_baseline();
        self.post_heartbeat().map_err(|e| e.to_string())
    }

    pub fn freeze_motor(&mut self, motor: usize, count: i64) -> Result<(), SendError> {
        if motor >= self.lanes.len() {
            return Err(self.fatal(&format!(
                "cannot freeze motor {motor}; only {} motors are configured",
                self.lanes.len()
            )));
        }
        let mcu_id = self.mcu_id;
        let oid = self.lanes[motor].oid;
        let mut cancelled_barriers = Vec::new();
        self.backlog.retain(|out| {
            if out.lane != motor {
                return true;
            }
            if let Outbound::Barrier(id) = out.frame {
                cancelled_barriers.push(id.seq);
            }
            false
        });
        for seq in cancelled_barriers {
            self.cancel_barrier(motor, seq);
        }
        let lane = &mut self.lanes[motor];
        lane.step_clock = None;
        lane.last_sent_boundary = None;
        lane.pending_cut = None;
        lane.seams.clear();
        lane.frozen = true;
        tracing::info!(
            subsystem = "pump",
            event = "stepcompress_motor_frozen",
            mcu = mcu_id,
            motor,
            oid,
            count,
            "motor frozen - frames for it are dropped until a reanchor"
        );
        self.reset_motor_position(motor, count)
            .map_err(SendError::Fatal)
    }

    /// Nothing staged, nothing on the wire, nothing awaiting a cut: the
    /// precondition for handing this lane's motor over to the other transport.
    pub fn transport_quiescent(&self) -> bool {
        self.transport_idle()
            && self
                .lanes
                .iter()
                .all(|lane| lane.pending_cut.is_none() && lane.seams.is_empty())
    }

    /// The mcu's own executed step count for `axis` in trajectory steps,
    /// cross-checked against the host's absolute bookkeeping on EVERY motor
    /// of the axis - AWD twins must agree, and a mismatch on a quiesced lane
    /// means the counters have diverged, which is exactly what a transport
    /// handover must not carry forward.
    pub fn executed_position(&self, axis: u8) -> Result<i64, SendError> {
        let mut positions = self
            .motors_of(axis)?
            .map(|motor| self.motor_executed_position(axis, motor));
        let first = positions.next().expect("motors_of guarantees non-empty")?;
        for position in positions {
            let position = position?;
            if position != first {
                return Err(self.axis_fatal(
                    axis,
                    &format!(
                        "coupled motors disagree on the executed position ({first} vs \
                         {position}); a transport handover cannot pick one"
                    ),
                ));
            }
        }
        Ok(first)
    }

    /// Re-origin one lane's counters — host shim and mcu counter together,
    /// for every motor of the axis — so the next stream starts from a
    /// position the other transport just left the motors at.
    pub fn reset_axis_position(&mut self, axis: u8, count: i64) -> Result<(), SendError> {
        let motors = self.motors_of(axis)?;
        self.abort_outbound();
        for motor in motors {
            self.reset_motor_position(motor, count)
                .map_err(SendError::Fatal)?;
            let mcu_count = if self.shim.invert_dir(motor) {
                -count
            } else {
                count
            };
            self.seed_mcu_position(self.lanes[motor].oid, mcu_count)?;
        }
        self.post_heartbeat()
    }

    pub fn abort_axes(&mut self, axes: &[u8]) -> Result<(), SendError> {
        let clock = self.clock_now()?;
        let motors = axes
            .iter()
            .map(|&axis| self.motors_of(axis))
            .collect::<Result<Vec<_>, _>>()?
            .into_iter()
            .flatten()
            .collect::<Vec<_>>();
        self.abort_outbound();
        for motor in motors {
            self.shim
                .halt_at(motor, clock.now)
                .map_err(|e| shim_error_to_send_error(self.mcu_id, e))?;
            self.lanes[motor].commanded_base = self.shim.commanded_position(motor);
        }
        self.sync_retirement_baseline();
        self.post_heartbeat()
    }

    /// Barriers still queued here never reach the mcu, so nothing will ever
    /// ack them — cancel them by advancing the high-water mark. Barriers
    /// already on the wire are acked even when the mcu halt discards them.
    pub fn abort_outbound(&mut self) {
        for index in 0..self.backlog.len() {
            let out = &self.backlog[index];
            if let Outbound::Barrier(id) = out.frame {
                let lane = out.lane;
                self.cancel_barrier(lane, id.seq);
            }
        }
        self.backlog.clear();
        self.in_flight.clear();
        for lane in &mut self.lanes {
            lane.step_clock = None;
            lane.last_sent_boundary = None;
            lane.pending_cut = None;
            lane.seams.clear();
        }
        self.pending_retire = None;
        self.sent_barriers.clear();
        self.deferred_retirement = false;
        self.retirement_idle_ticks = 0;
    }

    pub fn mark_reanchor(&mut self, axis: u8, at_start_clock: u64, epoch_freq: Option<f64>) {
        let motors = self
            .motors_of(axis)
            .unwrap_or_else(|error| panic!("mark_reanchor rejected its routed axis: {error}"));
        for motor in motors.clone() {
            if self.lanes[motor].frozen {
                self.lanes[motor].frozen = false;
                tracing::info!(
                    subsystem = "pump",
                    event = "stepcompress_motor_unfrozen",
                    mcu = self.mcu_id,
                    motor,
                    axis,
                    "reanchor thawed a frozen motor"
                );
                let snapshot = self.shim.consumed_counts();
                let target = motors
                    .clone()
                    .map(|peer| snapshot[peer].wrapping_add(self.lanes[peer].retirement_bias))
                    .max()
                    .expect("one logical axis has at least one motor");
                self.lanes[motor].retirement_bias = target.wrapping_sub(snapshot[motor]);
            }
            self.lanes[motor].seams.push_back(PendingSeam::Cut {
                at: at_start_clock,
                epoch_freq,
            });
        }
    }

    pub fn mark_seam_gap(&mut self, axis: u8, at_start_clock: u64) {
        let motors = self
            .motors_of(axis)
            .unwrap_or_else(|error| panic!("mark_seam_gap rejected its routed axis: {error}"));
        for motor in motors {
            self.lanes[motor]
                .seams
                .push_back(PendingSeam::Gap { at: at_start_clock });
        }
    }

    fn cut_stream_unsent(
        &mut self,
        motor: usize,
        freq: f64,
        cut_at: u64,
        now: u64,
    ) -> Result<(), SendError> {
        tracing::info!(
            subsystem = "motion",
            event = "reanchor_cut_unsent",
            mcu = self.mcu_id,
            motor,
            cut_at,
            "[reanchor] cutting unsent shim spans"
        );
        let (_expected, tail) = self
            .shim
            .halt_at(motor, cut_at)
            .map_err(|e| shim_error_to_send_error(self.mcu_id, e))?;
        self.queue_step_volley(now, tail)?;
        self.shim.set_motor_cycles_per_second(motor, freq);
        self.lanes[motor].commanded_base = self.shim.commanded_position(motor);
        Ok(())
    }

    fn begin_cut(
        &mut self,
        motor: usize,
        cut_at: u64,
        epoch_freq: f64,
        held: &[ClockedMotorSpan],
        now: u64,
    ) -> Result<(), SendError> {
        if self.lanes[motor].pending_cut.is_some() {
            return Err(self.motor_fatal(
                motor,
                &format!("reanchor cut already awaiting MCU reconciliation at clock {cut_at}"),
            ));
        }
        let oid = self.lanes[motor].oid;
        let Some(resume_clock) = self.lanes[motor].step_clock else {
            return Err(SendError::Fatal(format!(
                "stepcompress mcu {} motor {motor} oid {oid}: begin_cut has no step_clock \
                 cursor — abort_outbound cleared it without a subsequent drain resetting it; \
                 the host cannot determine where the MCU's stepper timeline is anchored",
                self.mcu_id
            )));
        };
        let barrier = self.issue_barrier(motor);
        self.queue_outbound(
            motor,
            Outbound::Barrier(barrier),
            resume_clock,
            resume_clock,
            now,
        );
        let expected_count = self.shim.expected_halt_count(motor, resume_clock);
        self.lanes[motor].pending_cut = Some(PendingCut {
            barrier,
            cut_at,
            resume_clock,
            epoch_freq,
            expected_count,
            held: held.to_vec(),
        });
        Ok(())
    }

    fn complete_cut(&mut self, motor: usize) -> Result<(), SendError> {
        let Some(cut) = self.lanes[motor].pending_cut.take() else {
            return Err(self.motor_fatal(motor, "cut completion has no pending cut"));
        };
        let wire_count = (self.step_count_query)(cut.barrier.oid).map_err(|error| {
            self.motor_fatal(
                motor,
                &format!(
                    "stepper_get_position readback failed after barrier oid={} seq={}: {error}",
                    cut.barrier.oid, cut.barrier.seq
                ),
            )
        })?;
        let executed_count = if self.shim.invert_dir(motor) {
            wire_count.saturating_neg()
        } else {
            wire_count
        };
        let delta = executed_count - cut.expected_count;
        if delta != 0 {
            return Err(SendError::Fatal(format!(
                "stepcompress mcu {} motor {} oid {} reanchor count mismatch at clock {}: \
                 host expected {} trajectory steps, MCU reported {} trajectory steps \
                 (wire {}), delta {}",
                self.mcu_id,
                motor,
                cut.barrier.oid,
                cut.cut_at,
                cut.expected_count,
                executed_count,
                wire_count,
                delta
            )));
        }
        let (expected, tail) = self
            .shim
            .halt_at_with_executed(motor, cut.resume_clock, executed_count)
            .map_err(|e| shim_error_to_send_error(self.mcu_id, e))?;
        if expected != cut.expected_count {
            return Err(self.motor_fatal(
                motor,
                &format!(
                    "host count changed while cut at {} awaited reconciliation (was {}, now {})",
                    cut.cut_at, cut.expected_count, expected
                ),
            ));
        }
        let clock = self.clock_now()?;
        self.queue_step_volley(clock.now, tail)?;
        self.flush(clock)?;
        self.shim.set_motor_cycles_per_second(motor, cut.epoch_freq);
        self.lanes[motor].commanded_base = self.shim.commanded_position(motor);
        if !cut.held.is_empty() {
            let clock = self.clock_now()?;
            #[allow(clippy::cast_possible_truncation)]
            let axis = self.lanes[motor].axis as u8;
            self.push_motor_spans(motor, &cut.held, axis, clock)?;
        }
        let clock = self.clock_now()?;
        let snapshot = self.shim.consumed_counts();
        self.publish_retirement(&snapshot, clock.now);
        self.drain_until(clock, clock.lead_horizon(), Retire::Publish)?;
        self.flush(clock)
    }

    fn retirement_batch_ready(&self, snapshot: &[u32]) -> bool {
        snapshot.iter().enumerate().any(|(motor, &after)| {
            let before = self.cohort_counts.get(motor).copied().unwrap_or(0);
            after.wrapping_sub(before) >= RETIREMENT_BATCH
        })
    }

    fn publish_retirement(&mut self, snapshot: &[u32], now: u64) {
        if self.pending_retire.is_some() {
            self.deferred_retirement = true;
            return;
        }
        self.deferred_retirement = false;
        let mut waits = Vec::new();
        for motor in 0..self.lanes.len() {
            let before = self.cohort_counts.get(motor).copied().unwrap_or(0);
            let after = snapshot.get(motor).copied().unwrap_or(0);
            if before == after {
                continue;
            }
            let id = self.issue_barrier(motor);
            let barrier_clock = self.lanes[motor].step_clock.unwrap_or(0);
            self.queue_outbound(
                motor,
                Outbound::Barrier(id),
                barrier_clock,
                barrier_clock,
                now,
            );
            waits.push(id);
        }
        if waits.is_empty() {
            return;
        }
        self.cohort_counts.clear();
        self.cohort_counts.extend_from_slice(snapshot);
        self.pending_retire = Some(PendingRetire {
            waits,
            counts: snapshot.to_vec(),
        });
    }

    fn issue_barrier(&mut self, motor: usize) -> BarrierId {
        let seed = self.barrier_seq_seed;
        let lane = &mut self.lanes[motor];
        let seq = *lane.next_barrier_seq.get_or_insert(seed);
        lane.next_barrier_seq = Some(seq.wrapping_add(1));
        BarrierId { oid: lane.oid, seq }
    }

    /// A barrier that will never reach the mcu earns no receipt, so retire it
    /// by hand: advance the lane's ack high-water mark past it.
    fn cancel_barrier(&mut self, motor: usize, seq: u32) {
        let acked = self.lanes[motor].acked_barrier_seq.get_or_insert(seq);
        if barrier_seq_after(seq, *acked) {
            *acked = seq;
        }
    }

    fn barrier_acked(&self, id: BarrierId) -> bool {
        self.lane_of_oid(id.oid)
            .and_then(|motor| self.lanes[motor].acked_barrier_seq)
            .is_some_and(|high_water| barrier_seq_covers(high_water, id.seq))
    }

    fn note_barrier_sent(&mut self, id: BarrierId, sent_clock: u64) {
        self.sent_barriers.push_back(SentBarrier { id, sent_clock });
    }

    fn prune_acked_barriers(&mut self) {
        let mut sent = std::mem::take(&mut self.sent_barriers);
        sent.retain(|entry| !self.barrier_acked(entry.id));
        self.sent_barriers = sent;
    }

    fn outstanding_barriers(&self) -> impl Iterator<Item = BarrierId> + '_ {
        let cohort = self
            .pending_retire
            .iter()
            .flat_map(|pending| pending.waits.iter().copied());
        let cuts = self
            .lanes
            .iter()
            .filter_map(|lane| lane.pending_cut.as_ref().map(|cut| cut.barrier));
        cohort.chain(cuts).filter(|&id| !self.barrier_acked(id))
    }

    fn barrier_ack_ledger(&self) -> String {
        let mut acked: Vec<(u32, u32)> = self
            .lanes
            .iter()
            .filter_map(|lane| lane.acked_barrier_seq.map(|seq| (lane.oid, seq)))
            .collect();
        acked.sort_unstable();
        acked
            .iter()
            .map(|(oid, seq)| format!("oid={oid} acked_through_seq={seq}"))
            .collect::<Vec<_>>()
            .join(", ")
    }

    /// Since when the endpoint has been owed this receipt, and what it is
    /// waiting on. Every unacked outstanding barrier is either on the wire or
    /// still in the backlog: it was queued at issue time, and the only paths
    /// that take one out of the backlog either record it as sent or force-ack
    /// it (which `outstanding_barriers` filters out).
    fn barrier_wait(&self, id: BarrierId) -> (u64, &'static str) {
        if let Some(sent) = self.sent_barriers.iter().find(|sent| sent.id == id) {
            return (sent.sent_clock, "sent but unacked");
        }
        let queued = self
            .backlog
            .iter()
            .find_map(|out| match out.frame {
                Outbound::Barrier(queued) if queued == id => {
                    Some(out.queued_clock.max(out.start_clock))
                }
                _ => None,
            })
            .expect("an unacked outstanding barrier is either sent or still backlogged");
        (queued, "backlogged, never sent")
    }

    /// A cohort barrier the mcu never acks parks `pending_retire` forever, and
    /// with it every drain waiting on the retirement it gates; a cut barrier
    /// parks the held spans of a whole lane. Wait no longer than
    /// [`BARRIER_ACK_DEADLINE_SECONDS`] of mcu clock, then name what is missing
    /// and what did come back.
    ///
    /// The wait is measured from the send when there is one and from the
    /// enqueue when the frame is still backlogged: a barrier that never reaches
    /// the wire earns no ack, and skipping it here left the only unbounded
    /// silent wedge in the endpoint — the lane parks, nothing executes, nothing
    /// retires, and the sole symptom is a drip cohort stalling at floor 0.
    fn check_barrier_deadline(&self, clock: McuClock) -> Result<(), SendError> {
        use std::fmt::Write as _;
        let deadline_ticks = clock.ticks(self.barrier_ack_deadline_secs);
        let mut missing = String::new();
        for id in self.outstanding_barriers() {
            let (since, state) = self.barrier_wait(id);
            let waited = clock.now.saturating_sub(since);
            if waited < deadline_ticks {
                continue;
            }
            if !missing.is_empty() {
                missing.push_str(", ");
            }
            write!(
                missing,
                "oid={} seq={} {state} for {:.3} s",
                id.oid,
                id.seq,
                clock.secs(waited)
            )
            .expect("formatting into a String cannot fail");
        }
        if missing.is_empty() {
            return Ok(());
        }
        Err(self.fatal(&format!(
            "barrier deadline of {:.3} s of mcu clock expired — [{missing}]; received acks: \
             [{}]. {} frames backlogged, {} move slots in flight of {}. The retirement cohort, \
             every cut awaiting reconciliation, and every drain behind them cannot be released. \
             {}",
            self.barrier_ack_deadline_secs,
            self.barrier_ack_ledger(),
            self.backlog.len(),
            self.in_flight.len(),
            self.budget,
            self.link_line()
        )))
    }

    fn release_retirements(&mut self) {
        let all_acked = self
            .pending_retire
            .as_ref()
            .is_some_and(|pending| pending.waits.iter().all(|&id| self.barrier_acked(id)));
        if all_acked {
            if let Some(done) = self.pending_retire.take() {
                self.published = done.counts;
            }
        }
    }

    pub fn on_barrier_ack(&mut self, oid: u32, seq: u32) -> Result<(), SendError> {
        self.reject_latched_fatal()?;
        self.on_barrier_ack_inner(oid, seq)
            .map_err(|e| self.escalate(e))
    }

    fn on_barrier_ack_inner(&mut self, oid: u32, seq: u32) -> Result<(), SendError> {
        let Some((motor, issued)) = self
            .lane_of_oid(oid)
            .and_then(|motor| self.lanes[motor].next_barrier_seq.map(|next| (motor, next)))
        else {
            return Err(self.fatal(&format!(
                "barrier ack oid={oid} seq={seq} but no barrier was ever issued for that oid"
            )));
        };
        let expected = self.lanes[motor]
            .acked_barrier_seq
            .map_or(self.barrier_seq_seed, |s| s.wrapping_add(1));
        if barrier_seq_before(seq, expected) {
            return Ok(());
        }
        if !barrier_seq_before(seq, issued) {
            return Err(self.fatal(&format!(
                "barrier ack oid={oid} seq={seq} is ahead of the {issued} barriers issued for \
                 that oid"
            )));
        }
        if seq != expected {
            return Err(self.fatal(&format!(
                "barrier ack oid={oid} seq={seq} out of order, expected seq={expected}"
            )));
        }
        self.lanes[motor].acked_barrier_seq = Some(seq);
        self.prune_acked_barriers();
        self.release_retirements();
        let acked = BarrierId { oid, seq };
        let cut_motor = self.lanes.iter().position(|lane| {
            lane.pending_cut
                .as_ref()
                .is_some_and(|cut| cut.barrier == acked)
        });
        if let Some(motor) = cut_motor {
            self.complete_cut(motor)?;
        }
        self.post_heartbeat()
    }

    fn clock_now(&self) -> Result<McuClock, SendError> {
        let (now, freq) = (self.clock_of)(self.mcu_id).ok_or_else(|| {
            self.fatal("no clock estimate — step sends cannot be paced against the mcu move queue")
        })?;
        if !freq.is_finite() || freq <= 0.0 {
            return Err(self.fatal(&format!("clock estimate reports {freq} Hz")));
        }
        Ok(McuClock { now, freq })
    }

    fn frame_clocks(&mut self, lane: usize, now: u64, frame: StepFrame) -> FrameClock {
        match frame {
            StepFrame::ResetStepClock { clock, .. } => {
                let expanded = expand_clock32(now, clock);
                self.lanes[lane].step_clock = Some(expanded);
                FrameClock::Stamped {
                    start: expanded,
                    end: expanded,
                }
            }
            StepFrame::SetNextStepDir { dir, .. } => FrameClock::HeadsTheNextRun { dir },
            StepFrame::QueueStep {
                interval,
                count,
                add,
                ..
            } => {
                let cursor = self.lanes[lane].step_clock.get_or_insert(now);
                let first_step = cursor.saturating_add(u64::from(interval));
                *cursor = cursor.saturating_add_signed(queue_step_span(interval, count, add));
                FrameClock::Stamped {
                    start: first_step,
                    end: *cursor,
                }
            }
            StepFrame::QueueStepHp {
                first_step,
                last_step,
                ..
            } => {
                let cursor = self.lanes[lane].step_clock.get_or_insert(now);
                let first = cursor.saturating_add(first_step);
                *cursor = cursor.saturating_add(last_step);
                FrameClock::Stamped {
                    start: first,
                    end: *cursor,
                }
            }
        }
    }

    /// Queue one volley the shim drained. The shim emits every
    /// `set_next_step_dir` immediately ahead of the step run it applies to,
    /// same motor, so the clock-less dir frame is stamped by the very next
    /// stamped frame of its own lane — one forward pass, no per-oid table.
    fn queue_step_volley(&mut self, now: u64, frames: Vec<StepFrame>) -> Result<(), SendError> {
        let mut unclocked_dir: Option<(usize, usize, u8)> = None;
        for frame in frames {
            let lane = self
                .lane_of_oid(step_frame_oid(&frame))
                .expect("the shim emits frames only for this endpoint's configured oids");
            match self.frame_clocks(lane, now, frame) {
                FrameClock::Stamped { start, end } => {
                    if let Some((slot, dir_lane, _)) = unclocked_dir {
                        if dir_lane == lane {
                            self.backlog[slot].start_clock = start;
                            self.backlog[slot].end_clock = start;
                            unclocked_dir = None;
                        }
                    }
                    self.queue_outbound(lane, Outbound::Step(frame), start, end, now);
                }
                FrameClock::HeadsTheNextRun { dir } => {
                    if let Some((_, stale_lane, stale_dir)) = unclocked_dir {
                        return Err(self.unstamped_dir_fatal(stale_lane, stale_dir));
                    }
                    self.queue_outbound(lane, Outbound::Step(frame), now, now, now);
                    unclocked_dir = Some((self.backlog.len() - 1, lane, dir));
                }
            }
        }
        if let Some((_, lane, dir)) = unclocked_dir {
            return Err(self.unstamped_dir_fatal(lane, dir));
        }
        Ok(())
    }

    fn unstamped_dir_fatal(&self, lane: usize, dir: u8) -> SendError {
        let oid = self.lanes[lane].oid;
        self.fatal(&format!(
            "set_next_step_dir oid={oid} dir={dir} was drained without the step run it \
             applies to — a clock-less frame cannot be paced against the mcu clock"
        ))
    }

    fn drain_until(
        &mut self,
        clock: McuClock,
        drain_to: u64,
        retire: Retire,
    ) -> Result<(), SendError> {
        let drain_started = std::time::Instant::now();
        let deadline = self
            .drain_pass_budget
            .map(|budget| std::time::Instant::now() + budget);
        let frames = self
            .shim
            .drain_budgeted(drain_to, deadline)
            .map_err(|e| shim_error_to_send_error(self.mcu_id, e))?;
        let drain_elapsed = drain_started.elapsed();
        if drain_elapsed > std::time::Duration::from_millis(5) {
            tracing::warn!(
                subsystem = "pump",
                event = "shim_drain_slow",
                mcu = self.mcu_id,
                elapsed_us = drain_elapsed.as_micros() as u64,
                frames = frames.len(),
                "step compression consumed this much real time inside one send pass"
            );
        }
        self.queue_step_volley(clock.now, frames)?;
        if self.backlog.len() > BACKLOG_CEILING_FRAMES {
            return Err(self.fatal(&format!(
                "{} outbound step frames waiting on move-queue budget, above the \
                 {BACKLOG_CEILING_FRAMES} ceiling — the mcu is not consuming moves. {}",
                self.backlog.len(),
                self.link_line()
            )));
        }
        let snapshot = self.shim.consumed_counts();
        if retire == Retire::Publish && self.retirement_batch_ready(&snapshot) {
            self.publish_retirement(&snapshot, clock.now);
        } else if snapshot != self.cohort_counts {
            self.deferred_retirement = true;
        }
        Ok(())
    }

    fn flush(&mut self, clock: McuClock) -> Result<(), SendError> {
        let fresh = self.clock_now()?;
        let worked_secs = clock.secs(fresh.now.saturating_sub(clock.now));
        if worked_secs > 0.050 {
            tracing::warn!(
                subsystem = "pump",
                event = "send_pass_worked_long",
                mcu = self.mcu_id,
                worked_us = (worked_secs * 1e6) as u64,
                "host work between clock sampling and egress consumed this much                  real margin in one send pass"
            );
        }
        let cutoff = clock
            .now
            .saturating_sub(clock.ticks(CONSUMED_MARGIN_SECONDS));
        self.in_flight.retain(|e| e.reclaim_clock > cutoff);
        self.order_backlog_by_deadline();
        let guard_secs = pump_past_guard_secs();
        let stale_by = clock.ticks(guard_secs);
        let mut burst: Vec<(&'static str, Vec<(String, ArgValue)>)> = Vec::new();
        let mut reclaim_clocks: Vec<u64> = Vec::new();
        let mut sent_boundaries: Vec<(usize, u64)> = Vec::new();
        let mut sent_barriers: Vec<(BarrierId, u64)> = Vec::new();
        let mut stale: Option<SendError> = None;
        let mut in_flight = self.in_flight.len() as u32;
        let mut worst_margin_clocks: Option<(i64, i64, u32)> = None;
        for out in &self.backlog {
            let kind = out.frame.kind();
            if kind.consumes_move_slot() && in_flight >= self.budget {
                break;
            }
            let late = kind.is_motion() && out.start_clock.saturating_add(stale_by) < clock.now;
            let covered_by_queued_motion = late
                && !kind.rearms_an_idle_timer()
                && self.lanes[out.lane]
                    .last_sent_boundary
                    .is_some_and(|sent_horizon| clock.now.saturating_add(stale_by) < sent_horizon);
            if late && !covered_by_queued_motion {
                let now = clock.now;
                let late_us = clock.micros(now - out.start_clock);
                let kind = kind.wire_name();
                let oid = self.lanes[out.lane].oid;
                let sent_horizon = self.lanes[out.lane].last_sent_boundary;
                stale = Some(SendError::Fatal(format!(
                    "stepcompress mcu {}: {kind} oid {oid} at clock {} is {late_us:.0} us \
                     behind the projected mcu clock {now} — a deficit of {late_us:.0} us past \
                     the {guard_secs} s floor margin (sent horizon {sent_horizon:?}, queued at \
                     clock {}). The mcu shuts down on any late stepper re-arm (\"Stepper too \
                     far in past\"/\"Rescheduled timer in the past\"). {SEND_LEAD_SECONDS} s \
                     of lead was not delivered: {} frames backlogged, {in_flight}/{} move \
                     slots in flight. {}",
                    self.mcu_id,
                    out.start_clock,
                    out.queued_clock,
                    self.backlog.len(),
                    self.budget,
                    self.link_line()
                )));
                break;
            }
            if kind.is_motion() {
                let margin = out.start_clock as i64 - clock.now as i64;
                if worst_margin_clocks.is_none_or(|(w, _, _)| margin < w) {
                    let entry_margin = out.start_clock as i64 - out.queued_clock as i64;
                    let oid = self.lanes[out.lane].oid;
                    worst_margin_clocks = Some((margin, entry_margin, oid));
                }
            }
            burst.push(frame_args(&out.frame));
            if kind.tracks_sent_boundary() {
                sent_boundaries.push((out.lane, out.end_clock));
            }
            if let Outbound::Barrier(id) = out.frame {
                sent_barriers.push((id, out.end_clock));
            }
            if kind.consumes_move_slot() {
                reclaim_clocks.push(out.start_clock);
                in_flight += 1;
            }
        }
        if let Some((margin, entry_margin, oid)) = worst_margin_clocks {
            let margin_secs = clock.secs_signed(margin);
            if margin_secs < SEND_MARGIN_WARN_FLOOR_SECS {
                tracing::warn!(
                    subsystem = "pump",
                    event = "step_send_margin_low",
                    mcu = self.mcu_id,
                    oid,
                    margin_us = (margin_secs * 1e6) as i64,
                    backlog_entry_margin_us = (clock.secs_signed(entry_margin) * 1e6) as i64,
                    backlog = self.backlog.len() as u64,
                    in_flight = self.in_flight.len() as u64,
                    budget = self.budget,
                    "a motion frame is being sent with almost no execution margin — one \
                     host hiccup from the mcu's Timer too close shutdown"
                );
            }
        }
        if !burst.is_empty() {
            let egress_started = std::time::Instant::now();
            (self.egress)(&burst)?;
            let egress_elapsed = egress_started.elapsed();
            if egress_elapsed > std::time::Duration::from_millis(5) {
                tracing::warn!(
                    subsystem = "pump",
                    event = "egress_slow",
                    mcu = self.mcu_id,
                    elapsed_us = egress_elapsed.as_micros() as u64,
                    burst = burst.len(),
                    "handing this burst to the transport blocked the send pass"
                );
            }
            self.backlog.drain(..burst.len());
            self.in_flight.extend(
                reclaim_clocks
                    .into_iter()
                    .map(|reclaim_clock| InFlight { reclaim_clock }),
            );
            for (lane, end_clock) in sent_boundaries {
                self.lanes[lane].last_sent_boundary = Some(end_clock);
            }
            for (id, execution_clock) in sent_barriers {
                self.note_barrier_sent(id, clock.now.max(execution_clock));
            }
        }
        if let Some(error) = stale {
            return Err(error);
        }
        let probe_interval = clock.ticks(WIRE_PROBE_INTERVAL_SECS);
        if clock.now >= self.last_wire_probe_clock.saturating_add(probe_interval) {
            self.last_wire_probe_clock = clock.now;
            #[allow(clippy::cast_possible_truncation)]
            (self.egress)(&[(
                "kalico_wire_probe",
                vec![(
                    "clock".to_string(),
                    ArgValue::Int(i64::from(clock.now as u32)),
                )],
            )])?;
        }
        self.release_retirements();
        self.post_heartbeat()
    }

    fn generate_buzz(&mut self, clock: McuClock) -> Result<(), SendError> {
        let Some(mut buzz) = self.buzz.take() else {
            return Ok(());
        };
        let lead_end = clock.lead_horizon() as f64;
        while buzz.next_stream_t < buzz.stream_t_end {
            let chunk_start = buzz.next_stream_t;
            if buzz.clock_at(chunk_start) >= lead_end {
                break;
            }
            let chunk_end = (chunk_start + trajectory::MAX_SPAN_SECS).min(buzz.stream_t_end);
            for motor in 0..self.lanes.len() {
                let Some(signal) = buzz.signals[motor].clone() else {
                    continue;
                };
                let view = ClockedMotorSpan::try_new(
                    signal,
                    chunk_start,
                    chunk_end,
                    chunk_start,
                    chunk_end,
                    buzz.clock_at(chunk_start),
                    buzz.cycles_per_second,
                )
                .map_err(|error| {
                    self.fatal(&format!(
                        "resonance buzz view {chunk_start}..{chunk_end} s cannot be clocked: \
                         {error}"
                    ))
                })?;
                let chunk_end_clock = view.end_clock;
                self.shim
                    .push_spans(motor, &[view])
                    .map_err(|e| shim_error_to_send_error(self.mcu_id, e))?;
                self.lanes[motor].retirement_bias =
                    self.lanes[motor].retirement_bias.wrapping_sub(1);
                let frames = self
                    .shim
                    .drain(chunk_end_clock)
                    .map_err(|e| shim_error_to_send_error(self.mcu_id, e))?;
                self.queue_step_volley(clock.now, frames)?;
            }
            buzz.next_stream_t = chunk_end;
        }
        if buzz.next_stream_t >= buzz.stream_t_end {
            for motor in 0..self.lanes.len() {
                if buzz.signals[motor].is_some() {
                    self.shim
                        .detach_span_seam(motor)
                        .map_err(|e| shim_error_to_send_error(self.mcu_id, e))?;
                }
            }
        } else {
            self.buzz = Some(buzz);
        }
        Ok(())
    }

    fn counts_by_axis(&self, counts: &[u32]) -> (Vec<u8>, Vec<u32>) {
        let mut axes = Vec::with_capacity(self.axis_runs.len());
        let mut logical_counts = Vec::with_capacity(self.axis_runs.len());
        for (axis, run) in &self.axis_runs {
            let biased =
                |motor: usize| counts[motor].wrapping_add(self.lanes[motor].retirement_bias);
            let count = run
                .clone()
                .filter(|&motor| !self.lanes[motor].frozen)
                .map(biased)
                .min()
                .unwrap_or_else(|| {
                    run.clone()
                        .map(biased)
                        .max()
                        .expect("one logical axis has at least one motor")
                });
            #[allow(clippy::cast_possible_truncation)]
            axes.push(*axis as u8);
            logical_counts.push(count);
        }
        (axes, logical_counts)
    }

    fn post_heartbeat(&self) -> Result<(), SendError> {
        let (axes, consumed) = self.counts_by_axis(&self.shim.consumed_counts());
        let (_, retired) = self.counts_by_axis(&self.published);
        self.pump_control
            .send(PumpMsg::Heartbeat(HeartbeatMsg {
                mcu_id: self.mcu_id,
                axes,
                consumed_counts: Some(consumed),
                retired_counts: retired,
                retired_by: super::messages::RetiredBy::Pulse,
            }))
            .map_err(|_| self.fatal("pump control channel closed"))
    }

    pub fn published_counts(&self) -> Vec<u32> {
        self.published.clone()
    }

    pub fn is_fatal(&self) -> bool {
        self.latched_fatal.is_some()
    }

    /// Every fatal this endpoint produces is unrecoverable by construction:
    /// the shim's timeline, the mcu's move queue, or the transport is already
    /// wrong, and the next attempt reproduces it. Retrying one silently — the
    /// pacer's tick loop only logged — froze the backlog and hung klippy's
    /// `wait_moves` behind a drain that could never release. Escalate to the
    /// pump once, then refuse to run again so the endpoint stops ticking.
    fn escalate(&mut self, error: SendError) -> SendError {
        let SendError::Fatal(message) = &error else {
            return error;
        };
        if self.latched_fatal.is_none() {
            self.latched_fatal = Some(message.clone());
            tracing::error!(
                subsystem = "pump",
                event = "stepcompress_endpoint_fatal",
                mcu = self.mcu_id,
                error = %message,
                "stepcompress endpoint went fatal — escalating to the pump"
            );
            let _ = self.pump_control.send(PumpMsg::StepcompressFatal {
                mcu_id: self.mcu_id,
                error: message.clone(),
            });
        }
        error
    }

    fn reject_latched_fatal(&self) -> Result<(), SendError> {
        self.latched_fatal
            .as_ref()
            .map_or(Ok(()), |message| Err(SendError::Fatal(message.clone())))
    }

    pub fn tick(&mut self) -> Result<(), SendError> {
        self.reject_latched_fatal()?;
        self.tick_inner().map_err(|e| self.escalate(e))
    }

    pub fn send_frames(&mut self, mcu_id: u32, frames: &[AxisFrame]) -> Result<(), SendError> {
        self.reject_latched_fatal()?;
        self.send_frames_inner(mcu_id, frames)
            .map_err(|e| self.escalate(e))
    }

    fn tick_inner(&mut self) -> Result<(), SendError> {
        if !self.sent_barriers.is_empty()
            || self.lanes.iter().any(|lane| lane.pending_cut.is_some())
            || self.pending_retire.is_some()
        {
            let clock = self.clock_now()?;
            self.check_barrier_deadline(clock)?;
        }
        if self.transport_idle()
            && self.pending_retire.is_none()
            && !self.deferred_retirement
            && self.buzz.is_none()
        {
            return Ok(());
        }
        let clock = self.clock_now()?;
        self.generate_buzz(clock)?;
        self.drain_until(clock, clock.lead_horizon(), Retire::Publish)?;
        let quiescent = self.transport_idle();
        self.retirement_idle_ticks = if quiescent {
            self.retirement_idle_ticks.saturating_add(1)
        } else {
            0
        };
        if self.pending_retire.is_none() && self.deferred_retirement && quiescent {
            let snapshot = self.shim.consumed_counts();
            if self.retirement_batch_ready(&snapshot)
                || self.retirement_idle_ticks >= RETIREMENT_IDLE_TICKS
            {
                self.publish_retirement(&snapshot, clock.now);
                self.retirement_idle_ticks = 0;
            }
        }
        self.flush(clock)
    }

    fn frame_motors(&self, frame: &AxisFrame) -> Result<Vec<usize>, SendError> {
        let motors = self.motors_of(frame.axis)?;
        let mask = frame.spans.first().map_or(0, |span| span.signal.motor_mask);
        if frame
            .spans
            .iter()
            .any(|span| span.signal.motor_mask != mask)
        {
            return Err(self.axis_fatal(frame.axis, "one frame mixes motor selectors"));
        }
        let selector = stepper_sel_from_mask(mask).map_err(|()| {
            self.axis_fatal(
                frame.axis,
                &format!("motor mask {mask:#010b} selects multiple motors"),
            )
        })?;
        if selector == 0 {
            let (kept, dropped): (Vec<usize>, Vec<usize>) =
                motors.partition(|&motor| !self.lanes[motor].frozen);
            if !dropped.is_empty() {
                tracing::warn!(
                    subsystem = "pump",
                    event = "stepcompress_frozen_motor_skipped",
                    mcu = self.mcu_id,
                    axis = frame.axis,
                    dropped = ?dropped,
                    "axis frame skipped frozen motors - they will not step"
                );
            }
            return Ok(kept);
        }
        let selected = usize::from(selector - 1);
        let Some(motor) = motors.clone().nth(selected) else {
            return Err(self.axis_fatal(
                frame.axis,
                &format!(
                    "motor selector {selector} exceeds its {} motors",
                    motors.len()
                ),
            ));
        };
        Ok((!self.lanes[motor].frozen)
            .then_some(motor)
            .into_iter()
            .collect())
    }

    fn credit_unselected_motors(&mut self, frame: &AxisFrame, selected: &[usize]) {
        let mask = frame.spans.first().map_or(0, |span| span.signal.motor_mask);
        if mask == 0 {
            return;
        }
        let count = u32::try_from(frame.spans.len())
            .expect("one axis frame cannot contain more than u32::MAX spans");
        let motors = self
            .motors_of(frame.axis)
            .unwrap_or_else(|error| panic!("frame motors were already validated: {error}"));
        for motor in motors {
            if !selected.contains(&motor) {
                self.lanes[motor].retirement_bias =
                    self.lanes[motor].retirement_bias.wrapping_add(count);
            }
        }
    }

    fn send_frames_inner(&mut self, mcu_id: u32, frames: &[AxisFrame]) -> Result<(), SendError> {
        if mcu_id != self.mcu_id {
            return Err(SendError::Fatal(format!(
                "stepcompress endpoint for mcu {} received frames addressed to mcu {mcu_id}",
                self.mcu_id
            )));
        }
        let clock = self.clock_now()?;
        for frame in frames {
            let frame_motors = self.frame_motors(frame)?;
            self.credit_unselected_motors(frame, &frame_motors);
            for motor in frame_motors {
                if let Some(cut) = self.lanes[motor].pending_cut.as_mut() {
                    cut.held.extend_from_slice(&frame.spans);
                    continue;
                }
                self.push_motor_spans(motor, &frame.spans, frame.axis, clock)?;
            }
        }
        self.drain_until(clock, clock.lead_horizon(), Retire::Publish)?;
        match self.flush(clock) {
            Err(SendError::Transient(error)) => {
                tracing::warn!(
                    subsystem = "pump",
                    event = "egress_backpressure_absorbed",
                    mcu = self.mcu_id,
                    backlog = self.backlog.len() as u64,
                    error = %error,
                    "egress pushed back after the spans were consumed into the shim - \
                     the compiled frames stay in the backlog for the pacer; failing the \
                     bundle would make the pump replay already-consumed spans"
                );
                Ok(())
            }
            other => other,
        }
    }

    /// Queue one run of lane views and carry the exact position the last of
    /// them commands. A masked view is an overlay that returns the motor to
    /// the lane it started from, so only unmasked views move the base; a
    /// resonance buzz then starts from the sampled base instead of the
    /// microstep lattice the shim happens to have reached.
    fn push_lane_spans(
        &mut self,
        motor: usize,
        spans: &[ClockedMotorSpan],
    ) -> Result<(), SendError> {
        self.shim
            .push_spans(motor, spans)
            .map_err(|e| shim_error_to_send_error(self.mcu_id, e))?;
        let Some(view) = spans.iter().rev().find(|view| view.signal.motor_mask == 0) else {
            return Ok(());
        };
        let pva = view.eval_at_clock(view.end_clock).map_err(|error| {
            self.motor_fatal(
                motor,
                &format!(
                    "queued view does not evaluate at its own end clock {}: {error}",
                    view.end_clock
                ),
            )
        })?;
        self.lanes[motor].commanded_base = pva.position;
        Ok(())
    }

    /// The first view of `rest` that reaches this lane's next marked seam,
    /// with the seam itself.
    fn seam_reached_by(
        &self,
        motor: usize,
        rest: &[ClockedMotorSpan],
    ) -> Option<(usize, PendingSeam)> {
        let seam = *self.lanes[motor].seams.front()?;
        let at = seam.at();
        let index = rest
            .iter()
            .position(|view| view.start_clock >= at || view.end_clock > at)?;
        Some((index, seam))
    }

    /// Push one motor's views through the pending-seam ladder: push up to each
    /// marked seam, apply the seam (cut or sanctioned gap), continue with the
    /// remainder as a fresh run. A sent-boundary cut defers the remainder into
    /// `PendingCut::held`; [`Self::complete_cut`] replays it through this same
    /// path, so seams marked while a cut awaited its barrier are applied
    /// instead of the held run being pushed as one contiguous stream.
    fn push_motor_spans(
        &mut self,
        motor: usize,
        spans: &[ClockedMotorSpan],
        axis: u8,
        clock: McuClock,
    ) -> Result<(), SendError> {
        let mut rest: &[ClockedMotorSpan] = spans;
        loop {
            let Some((index, seam)) = self.seam_reached_by(motor, rest) else {
                return self.push_lane_spans(motor, rest);
            };
            let (head, tail) = rest.split_at(index);
            self.push_lane_spans(motor, head)?;
            self.lanes[motor].seams.pop_front();
            match seam {
                PendingSeam::Cut { at, epoch_freq } => {
                    let Some(epoch_freq) = epoch_freq else {
                        return Err(self.axis_fatal(
                            axis,
                            "fresh epoch carried no clock slope; the shim cannot adopt the \
                             producer's timeline",
                        ));
                    };
                    self.drain_until(clock, at, Retire::Defer)?;
                    self.drain_until(clock, clock.lead_horizon(), Retire::Defer)?;
                    let sent = self.lanes[motor]
                        .last_sent_boundary
                        .is_some_and(|boundary| at <= boundary);
                    if sent {
                        return self.begin_cut(motor, at, epoch_freq, tail, clock.now);
                    }
                    self.cut_stream_unsent(motor, epoch_freq, at, clock.now)?;
                }
                PendingSeam::Gap { at } => {
                    tracing::info!(
                        subsystem = "motion",
                        event = "seam_gap_accepted",
                        mcu = self.mcu_id,
                        motor,
                        at,
                        "[rejoin] forward seam gap sanctioned — no mcu frames"
                    );
                    self.shim
                        .accept_forward_seam_gap(motor, at)
                        .map_err(|e| shim_error_to_send_error(self.mcu_id, e))?;
                }
            }
            rest = tail;
        }
    }
}

fn buzz_signal(
    profile: &Arc<BuzzProfile>,
    base_position: f64,
    sign: f64,
) -> Result<MotorSpan, trajectory::ContinuousError> {
    MotorSpan::try_new(
        Arc::from([MotorGroup::Independent(MotorTerm {
            source_axis: 0,
            axis: ContinuousAxis::Buzz {
                base_position,
                sign,
                profile: Arc::clone(profile),
            },
            scale: 1.0,
        })]),
        profile.t_start(),
        profile.t_end(),
        0,
        0,
        false,
    )
}

pub struct StepcompressPacer {
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl StepcompressPacer {
    pub fn spawn(endpoints: Vec<Arc<Mutex<StepcompressEndpoint>>>) -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let stop_for_thread = Arc::clone(&stop);
        let thread = std::thread::Builder::new()
            .name("stepcompress-pacer".into())
            .spawn(move || {
                host_rt::thread_prio::elevate_current_thread(
                    host_rt::thread_prio::PUMP_RT_PRIORITY,
                    "stepcompress-pacer",
                );
                let mut live = endpoints;
                while !stop_for_thread.load(Ordering::Relaxed) {
                    live.retain(|endpoint| match endpoint.lock_ok().tick() {
                        Ok(()) => true,
                        Err(SendError::Fatal(_)) => false,
                        Err(e) => {
                            tracing::error!(
                                subsystem = "pump",
                                event = "stepcompress_pacer_error",
                                error = ?e,
                                "stepcompress pacer tick failed"
                            );
                            true
                        }
                    });
                    if live.is_empty() {
                        return;
                    }
                    std::thread::sleep(PACER_TICK);
                }
            })
            .expect("spawn stepcompress-pacer thread");
        Self {
            stop,
            thread: Some(thread),
        }
    }
}

impl Drop for StepcompressPacer {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.thread.take() {
            let _ = handle.join();
        }
    }
}

#[cfg(test)]
#[path = "stepcompress_sink_tests.rs"]
mod stepcompress_sink_tests;

#[cfg(test)]
#[path = "nudge_pacing_tests.rs"]
mod nudge_pacing_tests;
