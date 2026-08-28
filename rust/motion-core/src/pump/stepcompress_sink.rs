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
use std::collections::{HashMap, HashSet, VecDeque};
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

pub const SEND_LEAD_SECONDS: f64 = 2.0 * (host_rt::host_io::rtt::MIN_RTO_MS as f64) / 1000.0;

pub const CONSUMED_MARGIN_SECONDS: f64 = 0.010;

/// Sending a motion frame with less execution margin than this is one host
/// hiccup away from the MCU's "Timer too close" shutdown; worth a warn even
/// when the send succeeds.
pub const SEND_MARGIN_WARN_FLOOR_SECS: f64 = 0.050;

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
    let mut pulse_axes = Vec::with_capacity(motor_count);
    let mut pulse_oids = Vec::with_capacity(motor_count);
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
            pulse_axes.push(axis);
            pulse_oids.push(cfg.stepper_oids[motor]);
        }
    }
    if motors.is_empty() {
        return Err(format!(
            "stepcompress mcu {}: no pulse-capable lanes to stream to; a stepcompress endpoint \
             was built for an mcu whose every lane is phase-only",
            cfg.mcu_id
        ));
    }
    let query = host_io_step_count_query(cfg.mcu_id, host_io.clone());
    let mut endpoint = StepcompressEndpoint::new(
        cfg.mcu_id,
        StepShim::new(motors, SHIM_RING_DEPTH),
        pulse_axes,
        pulse_oids,
        host_io_egress(cfg.mcu_id, host_io),
        pump_control,
        clock_of,
        budget,
    );
    endpoint.set_step_count_query(query);
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

/// Where an outstanding barrier actually is. `Queued` has not reached the wire
/// yet, so the mcu owes nothing for it; `Lost` means the endpoint is waiting on
/// a receipt that neither the backlog nor the transport is carrying.
enum BarrierWait {
    Sent(u64),
    Queued(u64),
    Lost,
}

enum Outbound {
    Step(StepFrame),
    Barrier(BarrierId),
}
impl Outbound {
    fn oid(&self) -> u32 {
        match self {
            Self::Step(
                StepFrame::QueueStep { oid, .. }
                | StepFrame::QueueStepHp { oid, .. }
                | StepFrame::SetNextStepDir { oid, .. }
                | StepFrame::ResetStepClock { oid, .. },
            ) => *oid,
            Self::Barrier(id) => id.oid,
        }
    }
}

struct OutboundFrame {
    frame: Outbound,
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

pub struct StepcompressEndpoint {
    mcu_id: u32,
    shim: StepShim,
    axes: Vec<usize>,
    oids: Vec<u32>,
    egress: FrameEgress,
    pump_control: Sender<PumpMsg>,
    clock_of: ClockSource,
    budget: u32,
    backlog: VecDeque<OutboundFrame>,
    next_outbound_order: u64,
    in_flight: Vec<InFlight>,
    step_count_query: Option<StepCountQuery>,
    last_sent_boundary: HashMap<u32, u64>,
    pending_cuts: HashMap<usize, PendingCut>,
    step_clock: HashMap<u32, u64>,
    pending_seams: HashMap<usize, VecDeque<PendingSeam>>,
    frozen_motors: HashSet<usize>,
    pending_retire: VecDeque<PendingRetire>,
    deferred_retirement: bool,
    retirement_idle_ticks: u32,
    published: Vec<u32>,
    cohort_counts: Vec<u32>,
    retirement_bias: Vec<u32>,
    next_barrier_seq: HashMap<u32, u32>,
    acked_barrier_seq: HashMap<u32, u32>,
    barrier_seq_seed: u32,
    sent_barriers: VecDeque<SentBarrier>,
    barrier_ack_deadline_secs: f64,
    fatal: Option<String>,
    buzz: Option<StepBuzz>,
    commanded_base: Vec<f64>,
}

fn shim_error_to_send_error(mcu_id: u32, error: ShimError) -> SendError {
    match error {
        ShimError::QueueFull { motor } => SendError::Transient(format!(
            "stepcompress mcu {mcu_id} motor {motor}: shim span queue full"
        )),
        other => SendError::Fatal(format!("stepcompress mcu {mcu_id}: {other:?}")),
    }
}

fn frame_args(frame: &Outbound) -> (&'static str, Vec<(String, ArgValue)>) {
    match *frame {
        Outbound::Step(StepFrame::QueueStep {
            oid,
            interval,
            count,
            add,
        }) => (
            "queue_step",
            vec![
                ("oid".to_string(), ArgValue::Int(i64::from(oid))),
                ("interval".to_string(), ArgValue::Int(i64::from(interval))),
                ("count".to_string(), ArgValue::Int(i64::from(count))),
                ("add".to_string(), ArgValue::Int(i64::from(add))),
            ],
        ),
        Outbound::Step(StepFrame::QueueStepHp {
            oid,
            interval,
            count,
            add,
            add2,
            shift,
            ..
        }) => (
            "queue_step_hp",
            vec![
                ("oid".to_string(), ArgValue::Int(i64::from(oid))),
                ("interval".to_string(), ArgValue::Int(i64::from(interval))),
                ("count".to_string(), ArgValue::Int(i64::from(count))),
                ("add".to_string(), ArgValue::Int(i64::from(add))),
                ("add2".to_string(), ArgValue::Int(i64::from(add2))),
                ("shift".to_string(), ArgValue::Int(i64::from(shift))),
            ],
        ),
        Outbound::Step(StepFrame::SetNextStepDir { oid, dir }) => (
            "set_next_step_dir",
            vec![
                ("oid".to_string(), ArgValue::Int(i64::from(oid))),
                ("dir".to_string(), ArgValue::Int(i64::from(dir))),
            ],
        ),
        Outbound::Step(StepFrame::ResetStepClock { oid, clock }) => (
            "reset_step_clock",
            vec![
                ("oid".to_string(), ArgValue::Int(i64::from(oid))),
                ("clock".to_string(), ArgValue::Int(i64::from(clock))),
            ],
        ),
        Outbound::Barrier(BarrierId { oid, seq }) => (
            "stepcompress_barrier",
            vec![
                ("oid".to_string(), ArgValue::Int(i64::from(oid))),
                ("seq".to_string(), ArgValue::Int(i64::from(seq))),
            ],
        ),
    }
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

impl StepcompressEndpoint {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        mcu_id: u32,
        shim: StepShim,
        axes: Vec<usize>,
        oids: Vec<u32>,
        egress: FrameEgress,
        pump_control: Sender<PumpMsg>,
        clock_of: ClockSource,
        budget: u32,
    ) -> Self {
        let published = shim.consumed_counts();
        let cohort_counts = published.clone();
        let retirement_bias = vec![0; published.len()];
        let commanded_base = vec![0.0; published.len()];
        let barrier_seq_seed = barrier_seq_seed();
        Self {
            mcu_id,
            shim,
            axes,
            oids,
            egress,
            pump_control,
            clock_of,
            budget,
            backlog: VecDeque::new(),
            next_outbound_order: 0,
            in_flight: Vec::new(),
            step_count_query: None,
            last_sent_boundary: HashMap::new(),
            pending_cuts: HashMap::new(),
            step_clock: HashMap::new(),
            pending_seams: HashMap::new(),
            frozen_motors: HashSet::new(),
            pending_retire: VecDeque::new(),
            published,
            cohort_counts,
            retirement_bias,
            deferred_retirement: false,
            retirement_idle_ticks: 0,
            next_barrier_seq: HashMap::new(),
            acked_barrier_seq: HashMap::new(),
            barrier_seq_seed,
            sent_barriers: VecDeque::new(),
            barrier_ack_deadline_secs: BARRIER_ACK_DEADLINE_SECONDS,
            fatal: None,
            buzz: None,
            commanded_base,
        }
    }

    fn set_step_count_query(&mut self, query: StepCountQuery) {
        self.step_count_query = Some(query);
    }

    #[must_use]
    pub fn accepts_buzz_mask(&self, axis_mask: u8) -> bool {
        self.axes
            .iter()
            .any(|&axis| axis < 8 && axis_mask & (1 << axis) != 0)
    }

    /// How many motor slots one arming of this endpoint would occupy.
    #[must_use]
    pub fn buzz_slot_count(&self) -> usize {
        self.oids.len()
    }

    #[must_use]
    pub fn buzz_complete(&self) -> bool {
        self.buzz.is_none()
            && self.backlog.is_empty()
            && self.in_flight.is_empty()
            && self.shim.queued_spans() == 0
            && self.shim.pending_roots() == 0
            && self.pending_retire.is_empty()
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
            return Err(SendError::Fatal(format!(
                "stepcompress mcu {}: resonance buzz is already active",
                self.mcu_id
            )));
        }
        if !self.backlog.is_empty()
            || !self.in_flight.is_empty()
            || self.shim.queued_spans() != 0
            || self.shim.pending_roots() != 0
            || !self.pending_retire.is_empty()
        {
            return Err(SendError::Fatal(format!(
                "stepcompress mcu {}: resonance buzz rejected while trajectory remains queued",
                self.mcu_id
            )));
        }
        if self.oids.len() > MAX_BUZZ_SLOTS {
            return Err(SendError::Fatal(format!(
                "stepcompress mcu {}: {} motors exceed the {MAX_BUZZ_SLOTS}-motor buzz limit",
                self.mcu_id,
                self.oids.len()
            )));
        }
        let mut signals: Vec<Option<Arc<MotorSpan>>> = Vec::with_capacity(self.axes.len());
        let mut driven = 0usize;
        for motor in 0..self.axes.len() {
            let axis = self.axes[motor];
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
                buzz_signal(profile, self.commanded_base[motor], sign).map_err(|error| {
                    SendError::Fatal(format!(
                        "stepcompress mcu {}: resonance buzz signal is not dispatchable: {error}",
                        self.mcu_id
                    ))
                })?;
            signals.push(Some(Arc::new(signal)));
        }
        if driven == 0 {
            return Err(SendError::Fatal(format!(
                "stepcompress mcu {}: resonance buzz axis mask 0x{axis_mask:02x} selects no motor",
                self.mcu_id
            )));
        }
        for motor in 0..self.axes.len() {
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

    fn motors_of(&self, axis: u8) -> Result<Vec<usize>, SendError> {
        let motors: Vec<usize> = self
            .axes
            .iter()
            .enumerate()
            .filter_map(|(motor, &configured)| (configured == usize::from(axis)).then_some(motor))
            .collect();
        if motors.is_empty() {
            return Err(SendError::Fatal(format!(
                "stepcompress mcu {}: frame for axis {axis} but configured axes are {:?}",
                self.mcu_id, self.axes
            )));
        }
        Ok(motors)
    }

    fn motor_executed_position(&self, axis: u8, motor: usize) -> Result<i64, SendError> {
        let oid = self.oids[motor];
        let query = self.step_count_query.as_ref().ok_or_else(|| {
            SendError::Fatal(format!(
                "stepcompress mcu {} axis {axis}: no stepper_get_position readback",
                self.mcu_id
            ))
        })?;
        let wire_count = query(oid).map_err(|error| {
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
    fn queue_outbound(&mut self, frame: Outbound, start_clock: u64, end_clock: u64, queued: u64) {
        let enqueue_order = self.next_outbound_order;
        self.next_outbound_order = self
            .next_outbound_order
            .checked_add(1)
            .expect("stepcompress outbound order exhausted");
        self.backlog.push_back(OutboundFrame {
            frame,
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
        self.axes.contains(&usize::from(axis))
    }

    pub fn owns_oid(&self, oid: u32) -> bool {
        self.oids.contains(&oid)
    }

    pub fn shim_mut(&mut self) -> &mut StepShim {
        &mut self.shim
    }

    pub fn reset_position(&mut self, pos_steps: &[i64]) -> Result<(), SendError> {
        if pos_steps.len() != self.axes.len() {
            return Err(SendError::Fatal(format!(
                "stepcompress mcu {}: position seed has {} entries for {} configured axes",
                self.mcu_id,
                pos_steps.len(),
                self.axes.len()
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
            self.seed_mcu_position(self.oids[motor], mcu_count)?;
        }
        self.post_heartbeat()
    }

    /// The mcu counts the step pulses it executed; the reconcile after an
    /// endstop trip compares that count against the host's own absolute
    /// bookkeeping, so both must share an origin.
    fn seed_mcu_position(&self, oid: u32, count: i64) -> Result<(), SendError> {
        let count = i32::try_from(count).map_err(|_| {
            SendError::Fatal(format!(
                "stepcompress mcu {}: position seed {count} for oid {oid} does not fit the \
                 mcu's 32-bit step counter",
                self.mcu_id
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
        self.pending_retire.clear();
        self.deferred_retirement = false;
    }

    pub fn reset_motor_position(&mut self, motor: usize, count: i64) -> Result<(), String> {
        self.shim
            .halt_at(motor, u64::MAX)
            .map_err(|e| format!("stepcompress mcu {}: {e}", self.mcu_id))?;
        self.shim.reset_position(motor, count);
        self.commanded_base[motor] = self.shim.commanded_position(motor);
        self.sync_retirement_baseline();
        self.post_heartbeat().map_err(|e| e.to_string())
    }
    pub fn freeze_motor(&mut self, motor: usize, count: i64) -> Result<(), SendError> {
        let oid = *self.oids.get(motor).ok_or_else(|| {
            SendError::Fatal(format!(
                "stepcompress mcu {}: cannot freeze motor {motor}; only {} motors are configured",
                self.mcu_id,
                self.oids.len()
            ))
        })?;
        let mut cancelled_barriers = Vec::new();
        self.backlog.retain(|out| {
            if out.frame.oid() != oid {
                return true;
            }
            if let Outbound::Barrier(id) = out.frame {
                cancelled_barriers.push(id);
            }
            false
        });
        for id in cancelled_barriers {
            let acked = self.acked_barrier_seq.entry(id.oid).or_insert(id.seq);
            if barrier_seq_after(id.seq, *acked) {
                *acked = id.seq;
            }
        }
        self.step_clock.remove(&oid);
        self.last_sent_boundary.remove(&oid);
        self.pending_cuts.remove(&motor);
        self.pending_seams.remove(&motor);
        tracing::info!(
            subsystem = "pump",
            event = "stepcompress_motor_frozen",
            mcu = self.mcu_id,
            motor,
            oid,
            count,
            "motor frozen - frames for it are dropped until a reanchor"
        );
        self.frozen_motors.insert(motor);
        self.reset_motor_position(motor, count)
            .map_err(SendError::Fatal)
    }

    /// Nothing staged, nothing on the wire, nothing awaiting a cut: the
    /// precondition for handing this lane's motor over to the other transport.
    pub fn transport_quiescent(&self) -> bool {
        self.backlog.is_empty()
            && self.in_flight.is_empty()
            && self.pending_cuts.is_empty()
            && self.pending_seams.values().all(VecDeque::is_empty)
            && self.shim.queued_spans() == 0
            && self.shim.pending_roots() == 0
    }

    /// The mcu's own executed step count for `axis` in trajectory steps,
    /// cross-checked against the host's absolute bookkeeping on EVERY motor
    /// of the axis - AWD twins must agree, and a mismatch on a quiesced lane
    /// means the counters have diverged, which is exactly what a transport
    /// handover must not carry forward.
    pub fn executed_position(&self, axis: u8) -> Result<i64, SendError> {
        let motors = self.motors_of(axis)?;
        let mut positions = motors
            .iter()
            .map(|&motor| self.motor_executed_position(axis, motor));
        let first = positions.next().expect("motors_of guarantees non-empty")?;
        for position in positions {
            let position = position?;
            if position != first {
                return Err(SendError::Fatal(format!(
                    "stepcompress mcu {} axis {axis}: coupled motors disagree on the executed \
                     position ({first} vs {position}); a transport handover cannot pick one",
                    self.mcu_id
                )));
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
            self.seed_mcu_position(self.oids[motor], mcu_count)?;
        }
        self.post_heartbeat()
    }

    pub fn abort_axes(&mut self, axes: &[u8]) -> Result<(), SendError> {
        let (now, _) = self.clock_now()?;
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
                .halt_at(motor, now)
                .map_err(|e| shim_error_to_send_error(self.mcu_id, e))?;
            self.commanded_base[motor] = self.shim.commanded_position(motor);
        }
        self.sync_retirement_baseline();
        self.post_heartbeat()
    }

    /// Barriers still queued here never reach the mcu, so nothing will ever
    /// ack them — cancel them by advancing the high-water mark. Barriers
    /// already on the wire are acked even when the mcu halt discards them.
    pub fn abort_outbound(&mut self) {
        for out in &self.backlog {
            if let Outbound::Barrier(id) = out.frame {
                let acked = self.acked_barrier_seq.entry(id.oid).or_insert(id.seq);
                if barrier_seq_after(id.seq, *acked) {
                    *acked = id.seq;
                }
            }
        }
        self.backlog.clear();
        self.in_flight.clear();
        self.step_clock.clear();
        self.last_sent_boundary.clear();
        self.pending_cuts.clear();
        self.pending_seams.clear();
        self.pending_retire.clear();
        self.sent_barriers.clear();
        self.deferred_retirement = false;
        self.retirement_idle_ticks = 0;
    }

    pub fn mark_reanchor(&mut self, axis: u8, at_start_clock: u64, epoch_freq: Option<f64>) {
        for motor in self
            .motors_of(axis)
            .unwrap_or_else(|error| panic!("mark_reanchor rejected its routed axis: {error}"))
        {
            if self.frozen_motors.remove(&motor) {
                tracing::info!(
                    subsystem = "pump",
                    event = "stepcompress_motor_unfrozen",
                    mcu = self.mcu_id,
                    motor,
                    axis,
                    "reanchor thawed a frozen motor"
                );
                let snapshot = self.shim.consumed_counts();
                let target = self
                    .motors_of(axis)
                    .expect("the routed axis was validated above")
                    .into_iter()
                    .map(|peer| snapshot[peer].wrapping_add(self.retirement_bias[peer]))
                    .max()
                    .expect("one logical axis has at least one motor");
                self.retirement_bias[motor] = target.wrapping_sub(snapshot[motor]);
            }
            self.pending_seams
                .entry(motor)
                .or_default()
                .push_back(PendingSeam::Cut {
                    at: at_start_clock,
                    epoch_freq,
                });
        }
    }

    pub fn mark_seam_gap(&mut self, axis: u8, at_start_clock: u64) {
        for motor in self
            .motors_of(axis)
            .unwrap_or_else(|error| panic!("mark_seam_gap rejected its routed axis: {error}"))
        {
            self.pending_seams
                .entry(motor)
                .or_default()
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
        self.commanded_base[motor] = self.shim.commanded_position(motor);
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
        if self.pending_cuts.contains_key(&motor) {
            return Err(SendError::Fatal(format!(
                "stepcompress mcu {} motor {motor}: reanchor cut already awaiting MCU \
                 reconciliation at clock {cut_at}",
                self.mcu_id
            )));
        }
        let oid = self.oids[motor];
        let resume_clock = self.step_clock.get(&oid).copied().unwrap_or(0);
        let seq = {
            let next = self
                .next_barrier_seq
                .entry(oid)
                .or_insert(self.barrier_seq_seed);
            let seq = *next;
            *next = next.wrapping_add(1);
            seq
        };
        let barrier = BarrierId { oid, seq };
        self.queue_outbound(Outbound::Barrier(barrier), resume_clock, resume_clock, now);
        self.pending_cuts.insert(
            motor,
            PendingCut {
                barrier,
                cut_at,
                resume_clock,
                epoch_freq,
                expected_count: self.shim.expected_halt_count(motor, resume_clock),
                held: held.to_vec(),
            },
        );
        Ok(())
    }

    fn complete_cut(&mut self, motor: usize) -> Result<(), SendError> {
        let cut = self.pending_cuts.remove(&motor).ok_or_else(|| {
            SendError::Fatal(format!(
                "stepcompress mcu {} motor {motor}: cut completion has no pending cut",
                self.mcu_id
            ))
        })?;
        let query = self.step_count_query.as_ref().ok_or_else(|| {
            SendError::Fatal(format!(
                "stepcompress mcu {} motor {motor}: sent-frame cut at {} has no \
                 stepper_get_position readback",
                self.mcu_id, cut.cut_at
            ))
        })?;
        let wire_count = query(cut.barrier.oid).map_err(|error| {
            SendError::Fatal(format!(
                "stepcompress mcu {} motor {motor}: stepper_get_position readback failed \
                 after barrier oid={} seq={}: {error}",
                self.mcu_id, cut.barrier.oid, cut.barrier.seq
            ))
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
            return Err(SendError::Fatal(format!(
                "stepcompress mcu {} motor {motor}: host count changed while cut at {} \
                 awaited reconciliation (was {}, now {})",
                self.mcu_id, cut.cut_at, cut.expected_count, expected
            )));
        }
        self.queue_step_volley(cut.resume_clock, tail)?;
        self.shim.set_motor_cycles_per_second(motor, cut.epoch_freq);
        self.commanded_base[motor] = self.shim.commanded_position(motor);
        if !cut.held.is_empty() {
            let (now, freq) = self.clock_now()?;
            #[allow(clippy::cast_possible_truncation)]
            let axis = self.axes[motor] as u8;
            self.push_motor_spans(motor, &cut.held, true, axis, now, freq)?;
        }
        let (now, freq) = self.clock_now()?;
        let snapshot = self.shim.consumed_counts();
        self.publish_retirement(&snapshot, now);
        self.drain_into_backlog(now, freq)?;
        self.flush(now, freq)
    }

    fn retirement_batch_ready(&self, snapshot: &[u32]) -> bool {
        snapshot.iter().enumerate().any(|(motor, &after)| {
            let before = self.cohort_counts.get(motor).copied().unwrap_or(0);
            after.wrapping_sub(before) >= RETIREMENT_BATCH
        })
    }
    fn publish_retirement(&mut self, snapshot: &[u32], now: u64) {
        if !self.pending_retire.is_empty() {
            self.deferred_retirement = true;
            return;
        }
        self.deferred_retirement = false;
        let mut waits = Vec::new();
        for motor in 0..self.oids.len() {
            let before = self.cohort_counts.get(motor).copied().unwrap_or(0);
            let after = snapshot.get(motor).copied().unwrap_or(0);
            if before == after {
                continue;
            }
            let oid = self.oids[motor];
            let seq = {
                let next = self
                    .next_barrier_seq
                    .entry(oid)
                    .or_insert(self.barrier_seq_seed);
                let seq = *next;
                *next = next.wrapping_add(1);
                seq
            };
            let barrier_clock = self.step_clock.get(&oid).copied().unwrap_or(0);
            let id = BarrierId { oid, seq };
            self.queue_outbound(Outbound::Barrier(id), barrier_clock, barrier_clock, now);
            waits.push(id);
        }
        if waits.is_empty() {
            return;
        }
        self.cohort_counts.clear();
        self.cohort_counts.extend_from_slice(snapshot);
        self.pending_retire.push_back(PendingRetire {
            waits,
            counts: snapshot.to_vec(),
        });
    }

    fn barrier_acked(&self, id: BarrierId) -> bool {
        self.acked_barrier_seq
            .get(&id.oid)
            .is_some_and(|&high_water| barrier_seq_covers(high_water, id.seq))
    }

    fn note_barrier_sent(&mut self, id: BarrierId, sent_clock: u64) {
        self.sent_barriers.push_back(SentBarrier { id, sent_clock });
    }

    fn prune_acked_barriers(&mut self) {
        let mut sent = std::mem::take(&mut self.sent_barriers);
        sent.retain(|entry| !self.barrier_acked(entry.id));
        self.sent_barriers = sent;
    }

    fn outstanding_barriers(&self) -> Vec<BarrierId> {
        let cohort = self
            .pending_retire
            .iter()
            .flat_map(|pending| pending.waits.iter().copied());
        let cuts = self.pending_cuts.values().map(|cut| cut.barrier);
        cohort
            .chain(cuts)
            .filter(|&id| !self.barrier_acked(id))
            .collect()
    }

    fn barrier_ack_ledger(&self) -> String {
        let mut acked: Vec<(u32, u32)> = self
            .acked_barrier_seq
            .iter()
            .map(|(&oid, &seq)| (oid, seq))
            .collect();
        acked.sort_unstable();
        acked
            .iter()
            .map(|(oid, seq)| format!("oid={oid} acked_through_seq={seq}"))
            .collect::<Vec<_>>()
            .join(", ")
    }

    fn barrier_wait(&self, id: BarrierId) -> BarrierWait {
        if let Some(sent) = self.sent_barriers.iter().find(|sent| sent.id == id) {
            return BarrierWait::Sent(sent.sent_clock);
        }
        self.backlog
            .iter()
            .find_map(|out| match out.frame {
                Outbound::Barrier(queued) if queued == id => {
                    Some(out.queued_clock.max(out.start_clock))
                }
                _ => None,
            })
            .map_or(BarrierWait::Lost, BarrierWait::Queued)
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
    fn check_barrier_deadline(&mut self, now: u64, freq: f64) -> Result<(), SendError> {
        let deadline_ticks = (freq * self.barrier_ack_deadline_secs) as u64;
        let overdue: Vec<(BarrierId, String)> = self
            .outstanding_barriers()
            .into_iter()
            .filter_map(|id| {
                let (since, state) = match self.barrier_wait(id) {
                    BarrierWait::Sent(sent_clock) => (sent_clock, "sent but unacked"),
                    BarrierWait::Queued(queued_clock) => (queued_clock, "backlogged, never sent"),
                    BarrierWait::Lost => {
                        return Some((id, "dropped from the backlog unsent".to_string()));
                    }
                };
                let waited = now.saturating_sub(since);
                (waited >= deadline_ticks)
                    .then(|| (id, format!("{state} for {:.3} s", waited as f64 / freq)))
            })
            .collect();
        if overdue.is_empty() {
            return Ok(());
        }
        let missing = overdue
            .iter()
            .map(|(id, state)| format!("oid={} seq={} {state}", id.oid, id.seq))
            .collect::<Vec<_>>()
            .join(", ");
        Err(SendError::Fatal(format!(
            "stepcompress mcu {}: barrier deadline of {:.3} s of mcu clock expired — [{missing}]; \
             received acks: [{}]. {} frames backlogged, {} move slots in flight of {}. The \
             retirement cohort, every cut awaiting reconciliation, and every drain behind them \
             cannot be released",
            self.mcu_id,
            self.barrier_ack_deadline_secs,
            self.barrier_ack_ledger(),
            self.backlog.len(),
            self.in_flight.len(),
            self.budget
        )))
    }

    fn release_retirements(&mut self) {
        while let Some(front) = self.pending_retire.front() {
            if !front.waits.iter().all(|&id| self.barrier_acked(id)) {
                break;
            }
            let done = self
                .pending_retire
                .pop_front()
                .expect("front was just observed");
            self.published = done.counts;
        }
    }

    pub fn on_barrier_ack(&mut self, oid: u32, seq: u32) -> Result<(), SendError> {
        if let Some(latched) = self.latched_fatal() {
            return Err(latched);
        }
        let result = self.on_barrier_ack_inner(oid, seq);
        result.map_err(|e| self.escalate(e))
    }

    fn on_barrier_ack_inner(&mut self, oid: u32, seq: u32) -> Result<(), SendError> {
        let mcu_id = self.mcu_id;
        let issued = self.next_barrier_seq.get(&oid).copied().ok_or_else(|| {
            SendError::Fatal(format!(
                "stepcompress mcu {mcu_id}: barrier ack oid={oid} seq={seq} but no barrier was \
                 ever issued for that oid"
            ))
        })?;
        let expected = self
            .acked_barrier_seq
            .get(&oid)
            .map_or(self.barrier_seq_seed, |&s| s.wrapping_add(1));
        if barrier_seq_before(seq, expected) {
            return Ok(());
        }
        if !barrier_seq_before(seq, issued) {
            return Err(SendError::Fatal(format!(
                "stepcompress mcu {mcu_id}: barrier ack oid={oid} seq={seq} is ahead of the \
                 {issued} barriers issued for that oid"
            )));
        }
        if seq != expected {
            return Err(SendError::Fatal(format!(
                "stepcompress mcu {mcu_id}: barrier ack oid={oid} seq={seq} out of order, \
                 expected seq={expected}"
            )));
        }
        self.acked_barrier_seq.insert(oid, seq);
        self.prune_acked_barriers();
        self.release_retirements();
        let cut_motor = self
            .pending_cuts
            .iter()
            .find_map(|(&motor, cut)| (cut.barrier == BarrierId { oid, seq }).then_some(motor));
        if let Some(motor) = cut_motor {
            self.complete_cut(motor)?;
        }
        self.post_heartbeat()
    }

    fn clock_now(&self) -> Result<(u64, f64), SendError> {
        let mcu_id = self.mcu_id;
        let (now, freq) = (self.clock_of)(mcu_id).ok_or_else(|| {
            SendError::Fatal(format!(
                "stepcompress mcu {mcu_id}: no clock estimate — step sends cannot be paced \
                 against the mcu move queue"
            ))
        })?;
        if !freq.is_finite() || freq <= 0.0 {
            return Err(SendError::Fatal(format!(
                "stepcompress mcu {mcu_id}: clock estimate reports {freq} Hz"
            )));
        }
        Ok((now, freq))
    }

    fn frame_clocks(&mut self, now: u64, frame: StepFrame) -> Option<(u64, u64)> {
        match frame {
            StepFrame::ResetStepClock { oid, clock } => {
                let expanded = expand_clock32(now, clock);
                self.step_clock.insert(oid, expanded);
                Some((expanded, expanded))
            }
            StepFrame::SetNextStepDir { .. } => None,
            StepFrame::QueueStep {
                oid,
                interval,
                count,
                add,
            } => {
                let cursor = self.step_clock.entry(oid).or_insert(now);
                let first_step = cursor.saturating_add(u64::from(interval));
                *cursor = cursor.saturating_add_signed(queue_step_span(interval, count, add));
                Some((first_step, *cursor))
            }
            StepFrame::QueueStepHp {
                oid,
                first_step,
                last_step,
                ..
            } => {
                let cursor = self.step_clock.entry(oid).or_insert(now);
                let first = cursor.saturating_add(first_step);
                *cursor = cursor.saturating_add(last_step);
                Some((first, *cursor))
            }
        }
    }

    /// `set_next_step_dir` carries no clock on the wire — the mcu latches it
    /// on receipt and applies it to the next `queue_step`. Its guard clock is
    /// therefore pure host bookkeeping, and the only coherent value is the
    /// clock of the step run it heads: the lane's cursor at the time the dir
    /// frame is emitted is where the *previous* run ended, which after a hold
    /// or a re-anchored volley is arbitrarily far in the past while the volley
    /// itself is punctual.
    fn queue_step_volley(&mut self, now: u64, frames: Vec<StepFrame>) -> Result<(), SendError> {
        let mut clocks: Vec<Option<(u64, u64)>> = frames
            .iter()
            .map(|&frame| self.frame_clocks(now, frame))
            .collect();
        let mut heads: HashMap<u32, u64> = HashMap::new();
        for (index, frame) in frames.iter().enumerate().rev() {
            match *frame {
                StepFrame::QueueStep { oid, .. } | StepFrame::QueueStepHp { oid, .. } => {
                    heads.insert(oid, clocks[index].expect("a step frame is clocked").0);
                }
                StepFrame::SetNextStepDir { oid, dir } => {
                    let head = heads.get(&oid).copied().ok_or_else(|| {
                        SendError::Fatal(format!(
                            "stepcompress mcu {}: set_next_step_dir oid={oid} dir={dir} was \
                             drained without the step run it applies to — a clock-less frame \
                             cannot be paced against the mcu clock",
                            self.mcu_id
                        ))
                    })?;
                    clocks[index] = Some((head, head));
                }
                StepFrame::ResetStepClock { .. } => {}
            }
        }
        for (frame, clocks) in frames.into_iter().zip(clocks) {
            let (start_clock, end_clock) = clocks.expect("every frame is stamped");
            self.queue_outbound(Outbound::Step(frame), start_clock, end_clock, now);
        }
        Ok(())
    }

    fn drain_into_backlog(&mut self, now: u64, freq: f64) -> Result<(), SendError> {
        let lead = (freq * SEND_LEAD_SECONDS) as u64;
        self.drain_until(now, now.saturating_add(lead))
    }

    fn drain_into_backlog_without_retirement(
        &mut self,
        now: u64,
        freq: f64,
    ) -> Result<(), SendError> {
        let lead = (freq * SEND_LEAD_SECONDS) as u64;
        self.drain_until_without_retirement(now, now.saturating_add(lead))
    }

    fn drain_until(&mut self, now: u64, drain_to: u64) -> Result<(), SendError> {
        self.drain_until_with_retirement(now, drain_to, true)
    }

    fn drain_until_without_retirement(&mut self, now: u64, drain_to: u64) -> Result<(), SendError> {
        self.drain_until_with_retirement(now, drain_to, false)
    }

    fn drain_until_with_retirement(
        &mut self,
        now: u64,
        drain_to: u64,
        publish: bool,
    ) -> Result<(), SendError> {
        let frames = self
            .shim
            .drain(drain_to)
            .map_err(|e| shim_error_to_send_error(self.mcu_id, e))?;
        self.queue_step_volley(now, frames)?;
        if self.backlog.len() > BACKLOG_CEILING_FRAMES {
            return Err(SendError::Fatal(format!(
                "stepcompress mcu {}: {} outbound step frames waiting on move-queue budget, \
                 above the {BACKLOG_CEILING_FRAMES} ceiling — the mcu is not consuming moves",
                self.mcu_id,
                self.backlog.len()
            )));
        }
        let snapshot = self.shim.consumed_counts();
        if publish && self.retirement_batch_ready(&snapshot) {
            self.publish_retirement(&snapshot, now);
        } else if snapshot != self.cohort_counts {
            self.deferred_retirement = true;
        }
        Ok(())
    }

    fn flush(&mut self, now: u64, freq: f64) -> Result<(), SendError> {
        let margin = (freq * CONSUMED_MARGIN_SECONDS) as u64;
        let cutoff = now.saturating_sub(margin);
        self.in_flight.retain(|e| e.reclaim_clock > cutoff);
        self.order_backlog_by_deadline();
        let guard_secs = pump_past_guard_secs();
        let stale_by = (freq * guard_secs) as u64;
        let mut burst: Vec<(&'static str, Vec<(String, ArgValue)>)> = Vec::new();
        let mut reclaim_clocks: Vec<u64> = Vec::new();
        let mut sent_boundaries: Vec<(u32, u64)> = Vec::new();
        let mut sent_barriers: Vec<(BarrierId, u64)> = Vec::new();
        let mut stale: Option<SendError> = None;
        let mut in_flight = self.in_flight.len() as u32;
        let mut worst_margin_clocks: Option<i64> = None;
        for out in &self.backlog {
            let consumes_slot = matches!(
                &out.frame,
                Outbound::Step(StepFrame::QueueStep { .. } | StepFrame::QueueStepHp { .. })
                    | Outbound::Barrier(_)
            );
            // Every motion frame's start clock is guarded, not just the
            // queue_step frames: reset_step_clock accepts a past clock
            // silently and the first stepper_event then fires immediately,
            // starting a catch-up that starves the scheduler (the MCU-side
            // "Rescheduled timer in the past"). The reset clock is the first
            // frame of a fresh volley, so it is the one that must never
            // reach the wire in the past. Barriers are control receipts —
            // their start clock is the last sent step clock, in the past by
            // the delivery lead by design — so they are exempt.
            let motion_frame = matches!(
                &out.frame,
                Outbound::Step(StepFrame::ResetStepClock { .. })
                    | Outbound::Step(StepFrame::SetNextStepDir { .. })
                    | Outbound::Step(StepFrame::QueueStep { .. })
                    | Outbound::Step(StepFrame::QueueStepHp { .. })
            );
            if consumes_slot && in_flight >= self.budget {
                break;
            }
            if motion_frame && out.start_clock.saturating_add(stale_by) < now {
                let late_us = (now - out.start_clock) as f64 * 1e6 / freq;
                let kind = match out.frame {
                    Outbound::Step(StepFrame::ResetStepClock { .. }) => "reset_step_clock",
                    Outbound::Step(StepFrame::SetNextStepDir { .. }) => "set_next_step_dir",
                    Outbound::Step(StepFrame::QueueStep { .. }) => "queue_step",
                    Outbound::Step(StepFrame::QueueStepHp { .. }) => "queue_step_hp",
                    Outbound::Barrier(_) => unreachable!("barriers are exempt above"),
                };
                stale = Some(SendError::Fatal(format!(
                    "stepcompress mcu {}: {kind} at clock {} is {late_us:.0} us behind the \
                     projected mcu clock {now} — a deficit of {late_us:.0} us past the \
                     {guard_secs} s floor margin. The mcu shuts down on any \
                     late stepper re-arm (\"Stepper too far in past\"/\"Rescheduled timer in \
                     the past\"). {SEND_LEAD_SECONDS} s of lead was not delivered: {} frames \
                     backlogged, {in_flight}/{} move slots in flight",
                    self.mcu_id,
                    out.start_clock,
                    self.backlog.len(),
                    self.budget
                )));
                break;
            }
            if motion_frame {
                let margin = out.start_clock as i64 - now as i64;
                if worst_margin_clocks.is_none_or(|w| margin < w) {
                    worst_margin_clocks = Some(margin);
                }
            }
            burst.push(frame_args(&out.frame));
            let oid = match &out.frame {
                Outbound::Step(StepFrame::QueueStep { oid, .. })
                | Outbound::Step(StepFrame::QueueStepHp { oid, .. }) => Some(*oid),
                Outbound::Step(StepFrame::ResetStepClock { .. })
                | Outbound::Step(StepFrame::SetNextStepDir { .. })
                | Outbound::Barrier(_) => None,
            };
            if let Some(oid) = oid {
                sent_boundaries.push((oid, out.end_clock));
            }
            if let Outbound::Barrier(id) = out.frame {
                sent_barriers.push((id, out.end_clock));
            }
            if consumes_slot {
                reclaim_clocks.push(out.start_clock);
                in_flight += 1;
            }
        }
        if let Some(margin) = worst_margin_clocks {
            let margin_secs = margin as f64 / freq;
            if margin_secs < SEND_MARGIN_WARN_FLOOR_SECS {
                tracing::warn!(
                    subsystem = "pump",
                    event = "step_send_margin_low",
                    mcu = self.mcu_id,
                    margin_us = (margin_secs * 1e6) as i64,
                    backlog = self.backlog.len() as u64,
                    in_flight = self.in_flight.len() as u64,
                    budget = self.budget,
                    "a motion frame is being sent with almost no execution margin — one \
                     host hiccup from the mcu's Timer too close shutdown"
                );
            }
        }
        if !burst.is_empty() {
            (self.egress)(&burst)?;
            self.backlog.drain(..burst.len());
            self.in_flight.extend(
                reclaim_clocks
                    .into_iter()
                    .map(|reclaim_clock| InFlight { reclaim_clock }),
            );
            for (oid, end_clock) in sent_boundaries {
                self.last_sent_boundary.insert(oid, end_clock);
            }
            for (id, execution_clock) in sent_barriers {
                self.note_barrier_sent(id, now.max(execution_clock));
            }
        }
        if let Some(error) = stale {
            return Err(error);
        }
        self.release_retirements();
        self.post_heartbeat()
    }
    fn generate_buzz(&mut self, now: u64, freq: f64) -> Result<(), SendError> {
        let Some(mut buzz) = self.buzz.take() else {
            return Ok(());
        };
        let lead_end = now.saturating_add((freq * SEND_LEAD_SECONDS) as u64) as f64;
        while buzz.next_stream_t < buzz.stream_t_end {
            let chunk_start = buzz.next_stream_t;
            if buzz.clock_at(chunk_start) >= lead_end {
                break;
            }
            let chunk_end = (chunk_start + trajectory::MAX_SPAN_SECS).min(buzz.stream_t_end);
            for motor in 0..self.oids.len() {
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
                    SendError::Fatal(format!(
                        "stepcompress mcu {}: resonance buzz view {chunk_start}..{chunk_end} s \
                         cannot be clocked: {error}",
                        self.mcu_id
                    ))
                })?;
                let chunk_end_clock = view.end_clock;
                self.shim
                    .push_spans(motor, &[view])
                    .map_err(|e| shim_error_to_send_error(self.mcu_id, e))?;
                self.retirement_bias[motor] = self.retirement_bias[motor].wrapping_sub(1);
                let frames = self
                    .shim
                    .drain(chunk_end_clock)
                    .map_err(|e| shim_error_to_send_error(self.mcu_id, e))?;
                self.queue_step_volley(now, frames)?;
            }
            buzz.next_stream_t = chunk_end;
        }
        if buzz.next_stream_t >= buzz.stream_t_end {
            for motor in 0..self.oids.len() {
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
        let mut axes = Vec::new();
        let mut logical_counts = Vec::new();
        let mut first = 0;
        while first < self.axes.len() {
            let axis = self.axes[first];
            let end = self.axes[first..]
                .iter()
                .position(|&candidate| candidate != axis)
                .map_or(self.axes.len(), |offset| first + offset);
            let active = (first..end).filter(|motor| !self.frozen_motors.contains(motor));
            let count = active
                .map(|motor| counts[motor].wrapping_add(self.retirement_bias[motor]))
                .min()
                .unwrap_or_else(|| {
                    (first..end)
                        .map(|motor| counts[motor].wrapping_add(self.retirement_bias[motor]))
                        .max()
                        .expect("one logical axis has at least one motor")
                });
            #[allow(clippy::cast_possible_truncation)]
            axes.push(axis as u8);
            logical_counts.push(count);
            first = end;
        }
        (axes, logical_counts)
    }

    fn post_heartbeat(&self) -> Result<(), SendError> {
        let mcu_id = self.mcu_id;
        let (axes, consumed) = self.counts_by_axis(&self.shim.consumed_counts());
        let (_, retired) = self.counts_by_axis(&self.published);
        self.pump_control
            .send(PumpMsg::Heartbeat(HeartbeatMsg {
                mcu_id,
                axes,
                consumed_counts: Some(consumed),
                retired_counts: retired,
                retired_by: super::messages::RetiredBy::Pulse,
            }))
            .map_err(|_| {
                SendError::Fatal(format!(
                    "stepcompress mcu {mcu_id}: pump control channel closed"
                ))
            })
    }

    pub fn published_counts(&self) -> Vec<u32> {
        self.published.clone()
    }

    pub fn is_fatal(&self) -> bool {
        self.fatal.is_some()
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
        if self.fatal.is_none() {
            self.fatal = Some(message.clone());
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

    fn latched_fatal(&self) -> Option<SendError> {
        self.fatal.clone().map(SendError::Fatal)
    }

    pub fn tick(&mut self) -> Result<(), SendError> {
        if let Some(latched) = self.latched_fatal() {
            return Err(latched);
        }
        let result = self.tick_inner();
        result.map_err(|e| self.escalate(e))
    }

    pub fn send_frames(&mut self, mcu_id: u32, frames: &[AxisFrame]) -> Result<(), SendError> {
        if let Some(latched) = self.latched_fatal() {
            return Err(latched);
        }
        let result = self.send_frames_inner(mcu_id, frames);
        result.map_err(|e| self.escalate(e))
    }

    fn tick_inner(&mut self) -> Result<(), SendError> {
        if !self.sent_barriers.is_empty()
            || !self.pending_cuts.is_empty()
            || !self.pending_retire.is_empty()
        {
            let (now, freq) = self.clock_now()?;
            self.check_barrier_deadline(now, freq)?;
        }
        if self.backlog.is_empty()
            && self.in_flight.is_empty()
            && self.shim.queued_spans() == 0
            && self.shim.pending_roots() == 0
            && self.pending_retire.is_empty()
            && !self.deferred_retirement
            && self.buzz.is_none()
        {
            return Ok(());
        }
        let (now, freq) = self.clock_now()?;
        self.generate_buzz(now, freq)?;
        self.drain_into_backlog(now, freq)?;
        if self.shim.queued_spans() == 0 && self.shim.pending_roots() > 0 {
            for motor in 0..self.oids.len() {
                let tail = self
                    .shim
                    .finish(motor)
                    .map_err(|e| shim_error_to_send_error(self.mcu_id, e))?;
                self.queue_step_volley(now, tail)?;
            }
            self.deferred_retirement = true;
        }
        let transport_quiescent = self.backlog.is_empty()
            && self.in_flight.is_empty()
            && self.shim.queued_spans() == 0
            && self.shim.pending_roots() == 0;
        self.retirement_idle_ticks = if transport_quiescent {
            self.retirement_idle_ticks.saturating_add(1)
        } else {
            0
        };
        if self.pending_retire.is_empty() && self.deferred_retirement && transport_quiescent {
            let snapshot = self.shim.consumed_counts();
            if self.retirement_batch_ready(&snapshot)
                || self.retirement_idle_ticks >= RETIREMENT_IDLE_TICKS
            {
                self.publish_retirement(&snapshot, now);
                self.retirement_idle_ticks = 0;
            }
        }
        self.flush(now, freq)
    }

    fn frame_motors(&self, frame: &AxisFrame) -> Result<Vec<usize>, SendError> {
        let motors = self.motors_of(frame.axis)?;
        let mask = frame.spans.first().map_or(0, |span| span.signal.motor_mask);
        if frame
            .spans
            .iter()
            .any(|span| span.signal.motor_mask != mask)
        {
            return Err(SendError::Fatal(format!(
                "stepcompress mcu {} axis {}: one frame mixes motor selectors",
                self.mcu_id, frame.axis
            )));
        }
        let selector = stepper_sel_from_mask(mask).map_err(|()| {
            SendError::Fatal(format!(
                "stepcompress mcu {} axis {}: motor mask {mask:#010b} selects multiple motors",
                self.mcu_id, frame.axis
            ))
        })?;
        if selector == 0 {
            let (kept, dropped): (Vec<usize>, Vec<usize>) = motors
                .into_iter()
                .partition(|motor| !self.frozen_motors.contains(motor));
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
        let motor = motors.get(selected).copied().ok_or_else(|| {
            SendError::Fatal(format!(
                "stepcompress mcu {} axis {}: motor selector {selector} exceeds its {} motors",
                self.mcu_id,
                frame.axis,
                motors.len()
            ))
        })?;
        Ok((!self.frozen_motors.contains(&motor))
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
                self.retirement_bias[motor] = self.retirement_bias[motor].wrapping_add(count);
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
        let (now, freq) = self.clock_now()?;
        for frame in frames {
            let frame_motors = self.frame_motors(frame)?;
            self.credit_unselected_motors(frame, &frame_motors);
            for motor in frame_motors {
                if let Some(cut) = self.pending_cuts.get_mut(&motor) {
                    cut.held.extend_from_slice(&frame.spans);
                    continue;
                }
                self.push_motor_spans(motor, &frame.spans, false, frame.axis, now, freq)?;
            }
        }
        self.drain_into_backlog(now, freq)?;
        self.flush(now, freq)
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
            SendError::Fatal(format!(
                "stepcompress mcu {} motor {motor}: queued view does not evaluate at its own \
                 end clock {}: {error}",
                self.mcu_id, view.end_clock
            ))
        })?;
        self.commanded_base[motor] = pva.position;
        Ok(())
    }

    /// Push one motor's views through the pending-seam ladder: validate and
    /// push up to each marked seam, apply the seam (cut or sanctioned gap),
    /// continue with the remainder as a fresh run. A sent-boundary cut defers
    /// the remainder into `pending_cuts.held`; [`Self::complete_cut`] replays
    /// it through this same path, so seams marked while a cut awaited its
    /// barrier are applied instead of the held run being validated as one
    /// contiguous stream.
    fn push_motor_spans(
        &mut self,
        motor: usize,
        spans: &[ClockedMotorSpan],
        mut fresh_head: bool,
        axis: u8,
        now: u64,
        freq: f64,
    ) -> Result<(), SendError> {
        let mcu_id = self.mcu_id;
        let mut rest: &[ClockedMotorSpan] = spans;
        loop {
            let seam = self
                .pending_seams
                .get(&motor)
                .and_then(VecDeque::front)
                .copied();
            let seam_index = seam.and_then(|s| {
                let at = s.at();
                rest.iter()
                    .position(|view| view.start_clock >= at || view.end_clock > at)
            });
            let Some(index) = seam_index else {
                if fresh_head {
                    self.shim
                        .validate_fresh_spans(motor, rest)
                        .map_err(|e| shim_error_to_send_error(mcu_id, e))?;
                }
                self.push_lane_spans(motor, rest)?;
                return Ok(());
            };
            let seam = seam.expect("seam_index implies a pending seam");
            let (head, tail) = rest.split_at(index);
            if fresh_head {
                self.shim
                    .validate_fresh_spans(motor, head)
                    .map_err(|e| shim_error_to_send_error(mcu_id, e))?;
            } else {
                self.shim
                    .validate_spans(motor, head)
                    .map_err(|e| shim_error_to_send_error(mcu_id, e))?;
            }
            self.push_lane_spans(motor, head)?;
            let mut defer_tail = false;
            match seam {
                PendingSeam::Cut { at, epoch_freq } => {
                    let epoch_freq = epoch_freq.ok_or_else(|| {
                        SendError::Fatal(format!(
                            "stepcompress mcu {mcu_id} axis {axis}: fresh epoch carried no \
                             clock slope; the shim cannot adopt the producer's timeline"
                        ))
                    })?;
                    self.drain_until_without_retirement(now, at)?;
                    self.drain_into_backlog_without_retirement(now, freq)?;
                    let sent = self
                        .last_sent_boundary
                        .get(&self.oids[motor])
                        .is_some_and(|&boundary| at <= boundary);
                    if sent {
                        self.begin_cut(motor, at, epoch_freq, tail, now)?;
                        defer_tail = true;
                    } else {
                        self.cut_stream_unsent(motor, epoch_freq, at, now)?;
                    }
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
                        .map_err(|e| shim_error_to_send_error(mcu_id, e))?;
                }
            }
            if let Some(q) = self.pending_seams.get_mut(&motor) {
                q.pop_front();
                if q.is_empty() {
                    self.pending_seams.remove(&motor);
                }
            }
            if defer_tail {
                return Ok(());
            }
            rest = tail;
            fresh_head = true;
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
