use super::pump_loop::pump_past_guard_secs;
use super::sched::SeamBasis;
use super::{AxisFrame, HeartbeatMsg, PumpMsg, SendError};
use crate::lock_ext::LockExt;
use crate::mcu_config::{McuAxisConfig, StepcompressEncoder};
use crossbeam_channel::Sender;
use host_rt::host_io::McuHostIo;
use host_rt::host_io::parser::ArgValue;
use runtime::piece_ring::PieceEntry;
use std::collections::{HashMap, VecDeque};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, Weak};
use std::thread::JoinHandle;
use std::time::Duration;
use step_shim::{MotorConfig, ShimError, StepEncoder, StepFrame, StepShim};

pub const SHIM_RING_DEPTH: u32 = 64;
const RETIREMENT_BATCH: u32 = SHIM_RING_DEPTH;
const RETIREMENT_IDLE_TICKS: u32 = 10;

pub const MOVE_SLOT_RESERVE: u32 = 16;

pub const SEND_LEAD_SECONDS: f64 = 2.0 * (host_rt::host_io::rtt::MIN_RTO_MS as f64) / 1000.0;

pub const CONSUMED_MARGIN_SECONDS: f64 = 0.010;

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
             rate; piece spans must use the slope the host projects piece starts with",
            cfg.mcu_id
        ));
    }
    let sample_rate_hz = cfg.stepcompress_sample_rate;
    if !sample_rate_hz.is_finite() || sample_rate_hz <= 0.0 {
        return Err(format!(
            "stepcompress mcu {}: sample rate {sample_rate_hz} Hz is not a positive rate",
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
    let encoder = match cfg.stepcompress_encoder {
        StepcompressEncoder::HighPrecision => StepEncoder::HighPrecision,
        StepcompressEncoder::Classic => {
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
            StepEncoder::Classic {
                max_error_ticks: max_error_ticks as u32,
            }
        }
    };
    let mut motors = Vec::with_capacity(cfg.axes.len());
    for (motor, &axis) in cfg.axes.iter().enumerate() {
        let microstep_distance = cfg.microstep_distance[motor];
        let velocity_ceiling = cfg.motor_velocity_ceiling(axis);
        let steps_per_sample = (velocity_ceiling / microstep_distance / sample_rate_hz).ceil();
        let cap = runtime::sub_sample_timing::MAX_STEPS_PER_SAMPLE as f64;
        if steps_per_sample > cap {
            return Err(format!(
                "stepcompress mcu {} axis {axis}: {velocity_ceiling} mm/s over \
                 {microstep_distance} mm microsteps needs {steps_per_sample} steps per \
                 {sample_rate_hz} Hz sample, above the {cap} the step timing kernel can hold",
                cfg.mcu_id
            ));
        }
        let step_pulse_seconds = cfg.step_pulse_seconds[motor];
        if !step_pulse_seconds.is_finite() || step_pulse_seconds < 0.0 {
            return Err(format!(
                "stepcompress mcu {} axis {axis}: step pulse width {step_pulse_seconds} s is \
                 not a non-negative duration",
                cfg.mcu_id
            ));
        }
        motors.push(MotorConfig {
            oid: cfg.stepper_oids[motor],
            microstep_distance: microstep_distance as f32,
            invert_dir: cfg.invert_dir[motor],
            max_steps_per_sample: steps_per_sample as u32,
            sample_rate_hz: sample_rate_hz as f32,
            cycles_per_second,
            min_rearm_cycles: STEP_REARM_PULSES * (step_pulse_seconds * cycles_per_second) as u64,
            encoder,
        });
    }
    let query = host_io_step_count_query(cfg.mcu_id, host_io.clone());
    let mut endpoint = StepcompressEndpoint::new(
        cfg.mcu_id,
        StepShim::new(motors, SHIM_RING_DEPTH),
        cfg.axes.clone(),
        cfg.stepper_oids.clone(),
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
    held: Vec<PieceEntry>,
}

#[derive(Clone, Copy, PartialEq, Eq)]
struct BarrierId {
    oid: u32,
    seq: u32,
}

enum Outbound {
    Step(StepFrame),
    Barrier(BarrierId),
}

struct OutboundFrame {
    frame: Outbound,
    start_clock: u64,
    end_clock: u64,
    enqueue_order: u64,
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
    pending_seams: HashMap<u8, VecDeque<PendingSeam>>,
    pending_retire: VecDeque<PendingRetire>,
    deferred_retirement: bool,
    retirement_idle_ticks: u32,
    published: Vec<u32>,
    cohort_counts: Vec<u32>,
    next_barrier_seq: HashMap<u32, u32>,
    acked_barrier_seq: HashMap<u32, u32>,
    barrier_seq_seed: u32,
}

fn shim_error_to_send_error(mcu_id: u32, error: ShimError) -> SendError {
    match error {
        ShimError::RingFull { motor } => SendError::Transient(format!(
            "stepcompress mcu {mcu_id} motor {motor}: shim ring full"
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

fn barrier_seq_seed() -> u32 {
    let elapsed = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .expect("system clock is before the Unix epoch");
    (elapsed.as_nanos() as u32) | 1
}

fn barrier_seq_after(candidate: u32, reference: u32) -> bool {
    let distance = candidate.wrapping_sub(reference);
    distance != 0 && distance < (1 << 31)
}

fn barrier_seq_before(candidate: u32, reference: u32) -> bool {
    barrier_seq_after(reference, candidate)
}

fn barrier_seq_covers(high_water: u32, seq: u32) -> bool {
    high_water == seq || barrier_seq_after(high_water, seq)
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
        let published = shim.retired_counts();
        let cohort_counts = published.clone();
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
            pending_retire: VecDeque::new(),
            published,
            cohort_counts,
            deferred_retirement: false,
            retirement_idle_ticks: 0,
            next_barrier_seq: HashMap::new(),
            acked_barrier_seq: HashMap::new(),
            barrier_seq_seed,
        }
    }

    fn set_step_count_query(&mut self, query: StepCountQuery) {
        self.step_count_query = Some(query);
    }

    fn motor_of(&self, axis: u8) -> Result<usize, SendError> {
        let mcu_id = self.mcu_id;
        self.axes
            .iter()
            .position(|&a| a == usize::from(axis))
            .ok_or_else(|| {
                SendError::Fatal(format!(
                    "stepcompress mcu {mcu_id}: frame for axis {axis} but configured axes are {:?}",
                    self.axes
                ))
            })
    }
    fn queue_outbound(&mut self, frame: Outbound, start_clock: u64, end_clock: u64) {
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
        self.shim.ring_depth()
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
        self.published = self.shim.retired_counts();
        self.cohort_counts.clone_from(&self.published);
        self.pending_retire.clear();
        self.deferred_retirement = false;
    }

    pub fn reset_motor_position(&mut self, motor: usize, count: i64) -> Result<(), String> {
        self.shim
            .halt_at(motor, u64::MAX)
            .map_err(|e| format!("stepcompress mcu {}: {e}", self.mcu_id))?;
        self.shim.reset_position(motor, count);
        self.sync_retirement_baseline();
        self.post_heartbeat().map_err(|e| e.to_string())
    }

    pub fn abort_axes(&mut self, axes: &[u8]) -> Result<(), SendError> {
        let (now, _) = self.clock_now()?;
        let motors = axes
            .iter()
            .map(|&axis| self.motor_of(axis))
            .collect::<Result<Vec<_>, _>>()?;
        self.abort_outbound();
        for motor in motors {
            self.shim
                .halt_at(motor, now)
                .map_err(|e| shim_error_to_send_error(self.mcu_id, e))?;
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
        self.deferred_retirement = false;
        self.retirement_idle_ticks = 0;
    }

    pub fn mark_reanchor(&mut self, axis: u8, at_start_clock: u64, epoch_freq: Option<f64>) {
        self.pending_seams
            .entry(axis)
            .or_default()
            .push_back(PendingSeam::Cut {
                at: at_start_clock,
                epoch_freq,
            });
    }

    pub fn mark_seam_gap(&mut self, axis: u8, at_start_clock: u64) {
        self.pending_seams
            .entry(axis)
            .or_default()
            .push_back(PendingSeam::Gap { at: at_start_clock });
    }

    /// How the shim will reproject this axis' piece ends once the pieces being
    /// staged now reach it: the epoch slope of a marked but not-yet-applied
    /// cut, otherwise the slope the shim currently holds. Frames already
    /// egressed carry clocks derived from it, so it cannot be retimed after
    /// the fact — upstream must adopt it, not the other way round.
    ///
    /// The merge spends at most half the shim's flat seam allowance, measured
    /// against the end of the piece it absorbs. The shim measures the merged
    /// piece against the start of the piece that follows, so it sees that
    /// spend plus the absorbed piece's own f32 reprojection — which its
    /// tolerance covers separately, scaled to the merged piece's span. Half
    /// the flat allowance therefore leaves the shim's check its meaning.
    pub fn seam_basis(&self, axis: u8) -> Option<SeamBasis> {
        let motor = self.axes.iter().position(|&a| a == usize::from(axis))?;
        let pending_cut_freq = self
            .pending_seams
            .get(&axis)
            .into_iter()
            .flatten()
            .rev()
            .find_map(|s| match *s {
                PendingSeam::Cut { epoch_freq, .. } => Some(epoch_freq),
                PendingSeam::Gap { .. } => None,
            });
        let freq = match pending_cut_freq {
            Some(Some(epoch_freq)) => epoch_freq,
            _ => self.shim.motor_cycles_per_second(motor),
        };
        Some(SeamBasis {
            freq,
            skew_budget_cycles: step_shim::MAX_SEAM_SKEW_CYCLES / 2,
        })
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
            "[reanchor] cutting unsent shim pieces"
        );
        let (_expected, tail) = self
            .shim
            .halt_at(motor, cut_at)
            .map_err(|e| shim_error_to_send_error(self.mcu_id, e))?;
        for frame in tail {
            let (start_clock, end_clock) = self.frame_clocks(now, frame);
            self.queue_outbound(Outbound::Step(frame), start_clock, end_clock);
        }
        self.shim.set_motor_cycles_per_second(motor, freq);
        let snapshot = self.shim.retired_counts();
        self.publish_retirement(&snapshot);
        Ok(())
    }

    fn begin_cut(
        &mut self,
        motor: usize,
        cut_at: u64,
        epoch_freq: f64,
        held: &[PieceEntry],
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
        self.queue_outbound(Outbound::Barrier(barrier), resume_clock, resume_clock);
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
        for frame in tail {
            let (start_clock, end_clock) = self.frame_clocks(cut.resume_clock, frame);
            self.queue_outbound(Outbound::Step(frame), start_clock, end_clock);
        }
        self.shim.set_motor_cycles_per_second(motor, cut.epoch_freq);
        if !cut.held.is_empty() {
            self.shim
                .validate_fresh_pieces(motor, &cut.held)
                .map_err(|e| shim_error_to_send_error(self.mcu_id, e))?;
            self.shim
                .push_pieces(motor, &cut.held)
                .map_err(|e| shim_error_to_send_error(self.mcu_id, e))?;
        }
        let snapshot = self.shim.retired_counts();
        self.publish_retirement(&snapshot);
        let (now, freq) = self.clock_now()?;
        self.drain_into_backlog(now, freq)?;
        self.flush(now, freq)
    }

    fn retirement_batch_ready(&self, snapshot: &[u32]) -> bool {
        snapshot.iter().enumerate().any(|(motor, &after)| {
            let before = self.cohort_counts.get(motor).copied().unwrap_or(0);
            after.wrapping_sub(before) >= RETIREMENT_BATCH
        })
    }
    fn publish_retirement(&mut self, snapshot: &[u32]) {
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
            self.queue_outbound(Outbound::Barrier(id), barrier_clock, barrier_clock);
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

    fn frame_clocks(&mut self, now: u64, frame: StepFrame) -> (u64, u64) {
        match frame {
            StepFrame::ResetStepClock { oid, clock } => {
                let expanded = expand_clock32(now, clock);
                self.step_clock.insert(oid, expanded);
                (expanded, expanded)
            }
            StepFrame::SetNextStepDir { oid, .. } => {
                let cursor = self.step_clock.get(&oid).copied().unwrap_or(now);
                (cursor, cursor)
            }
            StepFrame::QueueStep {
                oid,
                interval,
                count,
                add,
            } => {
                let cursor = self.step_clock.entry(oid).or_insert(now);
                let first_step = cursor.saturating_add(u64::from(interval));
                *cursor = cursor.saturating_add_signed(queue_step_span(interval, count, add));
                (first_step, *cursor)
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
                (first, *cursor)
            }
        }
    }

    fn drain_into_backlog(&mut self, now: u64, freq: f64) -> Result<(), SendError> {
        let lead = (freq * SEND_LEAD_SECONDS) as u64;
        self.drain_until(now, now.saturating_add(lead))
    }

    fn drain_until(&mut self, now: u64, drain_to: u64) -> Result<(), SendError> {
        let frames = self
            .shim
            .drain(drain_to)
            .map_err(|e| shim_error_to_send_error(self.mcu_id, e))?;
        for frame in frames {
            let (start_clock, end_clock) = self.frame_clocks(now, frame);
            self.queue_outbound(Outbound::Step(frame), start_clock, end_clock);
        }
        if self.backlog.len() > BACKLOG_CEILING_FRAMES {
            return Err(SendError::Fatal(format!(
                "stepcompress mcu {}: {} outbound step frames waiting on move-queue budget, \
                 above the {BACKLOG_CEILING_FRAMES} ceiling — the mcu is not consuming moves",
                self.mcu_id,
                self.backlog.len()
            )));
        }
        let snapshot = self.shim.retired_counts();
        if self.retirement_batch_ready(&snapshot) {
            self.publish_retirement(&snapshot);
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
        let stale_by = (freq * pump_past_guard_secs()) as u64;
        let mut burst: Vec<(&'static str, Vec<(String, ArgValue)>)> = Vec::new();
        let mut reclaim_clocks: Vec<u64> = Vec::new();
        let mut sent_boundaries: Vec<(u32, u64)> = Vec::new();
        let mut stale: Option<SendError> = None;
        let mut in_flight = self.in_flight.len() as u32;
        for out in &self.backlog {
            let consumes_slot = matches!(
                &out.frame,
                Outbound::Step(StepFrame::QueueStep { .. } | StepFrame::QueueStepHp { .. })
                    | Outbound::Barrier(_)
            );
            let queue_step = matches!(
                &out.frame,
                Outbound::Step(StepFrame::QueueStep { .. } | StepFrame::QueueStepHp { .. })
            );
            if consumes_slot && in_flight >= self.budget {
                break;
            }
            if queue_step && out.start_clock.saturating_add(stale_by) < now {
                let late_us = (now - out.start_clock) as f64 * 1e6 / freq;
                stale = Some(SendError::Fatal(format!(
                    "stepcompress mcu {}: queue_step first step at clock {} is {late_us:.0} us \
                     behind the mcu clock {now} — the mcu shuts down on any late idle-stepper \
                     re-arm (\"Stepper too far in past\"). {SEND_LEAD_SECONDS} s of lead was not \
                     delivered: {} frames backlogged, {in_flight}/{} move slots in flight",
                    self.mcu_id,
                    out.start_clock,
                    self.backlog.len(),
                    self.budget
                )));
                break;
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
            if consumes_slot {
                reclaim_clocks.push(out.start_clock);
                in_flight += 1;
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
        }
        if let Some(error) = stale {
            return Err(error);
        }
        self.release_retirements();
        self.post_heartbeat()
    }

    fn counts_by_axis(&self, counts: &[u32]) -> Vec<u32> {
        let max_axis = self.axes.iter().copied().max().unwrap_or(0);
        let mut out = vec![0u32; max_axis + 1];
        for (motor, &axis) in self.axes.iter().enumerate() {
            out[axis] = counts[motor];
        }
        out
    }

    fn post_heartbeat(&self) -> Result<(), SendError> {
        let mcu_id = self.mcu_id;
        let consumed = self.shim.retired_counts();
        self.pump_control
            .send(PumpMsg::Heartbeat(HeartbeatMsg {
                mcu_id,
                consumed_counts: Some(self.counts_by_axis(&consumed)),
                retired_counts: self.counts_by_axis(&self.published),
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

    pub fn tick(&mut self) -> Result<(), SendError> {
        if self.backlog.is_empty()
            && self.in_flight.is_empty()
            && self.shim.queued_pieces() == 0
            && self.shim.pending_steps() == 0
            && self.pending_retire.is_empty()
            && !self.deferred_retirement
        {
            return Ok(());
        }
        let (now, freq) = self.clock_now()?;
        self.drain_into_backlog(now, freq)?;
        if self.shim.queued_pieces() == 0 && self.shim.pending_steps() > 0 {
            for motor in 0..self.axes.len() {
                let tail = self
                    .shim
                    .finish(motor)
                    .map_err(|e| shim_error_to_send_error(self.mcu_id, e))?;
                for frame in tail {
                    let (start_clock, end_clock) = self.frame_clocks(now, frame);
                    self.queue_outbound(Outbound::Step(frame), start_clock, end_clock);
                }
            }
            self.deferred_retirement = true;
        }
        let transport_quiescent = self.backlog.is_empty()
            && self.in_flight.is_empty()
            && self.shim.queued_pieces() == 0
            && self.shim.pending_steps() == 0;
        self.retirement_idle_ticks = if transport_quiescent {
            self.retirement_idle_ticks.saturating_add(1)
        } else {
            0
        };
        if self.pending_retire.is_empty() && self.deferred_retirement && transport_quiescent {
            let snapshot = self.shim.retired_counts();
            if self.retirement_batch_ready(&snapshot)
                || self.retirement_idle_ticks >= RETIREMENT_IDLE_TICKS
            {
                self.publish_retirement(&snapshot);
                self.retirement_idle_ticks = 0;
            }
        }
        self.flush(now, freq)
    }

    pub fn send_frames(&mut self, mcu_id: u32, frames: &[AxisFrame]) -> Result<(), SendError> {
        if mcu_id != self.mcu_id {
            return Err(SendError::Fatal(format!(
                "stepcompress endpoint for mcu {} received frames addressed to mcu {mcu_id}",
                self.mcu_id
            )));
        }
        let (now, freq) = self.clock_now()?;
        for frame in frames {
            let motor = self.motor_of(frame.axis)?;
            if let Some(cut) = self.pending_cuts.get_mut(&motor) {
                cut.held.extend_from_slice(&frame.pieces);
                continue;
            }
            let mut rest: &[PieceEntry] = &frame.pieces;
            let mut fresh_head = false;
            loop {
                let seam = self
                    .pending_seams
                    .get(&frame.axis)
                    .and_then(VecDeque::front)
                    .copied();
                #[allow(clippy::cast_possible_truncation)]
                let cps = self.shim.motor_cycles_per_second(motor) as f32;
                let seam_index = seam.and_then(|s| {
                    let at = s.at();
                    rest.iter()
                        .position(|p| p.start_time >= at || p.end_time(cps) > at)
                });
                let Some(index) = seam_index else {
                    if fresh_head {
                        self.shim
                            .validate_fresh_pieces(motor, rest)
                            .map_err(|e| shim_error_to_send_error(mcu_id, e))?;
                    }
                    self.shim
                        .push_pieces(motor, rest)
                        .map_err(|e| shim_error_to_send_error(mcu_id, e))?;
                    break;
                };
                let seam = seam.expect("seam_index implies a pending seam");
                let (head, tail) = rest.split_at(index);
                if fresh_head {
                    self.shim
                        .validate_fresh_pieces(motor, head)
                        .map_err(|e| shim_error_to_send_error(mcu_id, e))?;
                } else {
                    self.shim
                        .validate_pieces_public(motor, head)
                        .map_err(|e| shim_error_to_send_error(mcu_id, e))?;
                }
                self.shim
                    .push_pieces(motor, head)
                    .map_err(|e| shim_error_to_send_error(mcu_id, e))?;
                let mut defer_tail = false;
                match seam {
                    PendingSeam::Cut { at, epoch_freq } => {
                        let epoch_freq = epoch_freq.ok_or_else(|| {
                            SendError::Fatal(format!(
                                "stepcompress mcu {mcu_id} axis {}: fresh epoch carried no \
                                 clock slope; the shim cannot adopt the producer's timeline",
                                frame.axis
                            ))
                        })?;
                        self.drain_until(now, at)?;
                        self.drain_into_backlog(now, freq)?;
                        let sent = self
                            .last_sent_boundary
                            .get(&self.oids[motor])
                            .is_some_and(|&boundary| at <= boundary);
                        if sent {
                            self.begin_cut(motor, at, epoch_freq, tail)?;
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
                if let Some(q) = self.pending_seams.get_mut(&frame.axis) {
                    q.pop_front();
                    if q.is_empty() {
                        self.pending_seams.remove(&frame.axis);
                    }
                }
                if defer_tail {
                    break;
                }
                rest = tail;
                fresh_head = true;
            }
        }
        self.drain_into_backlog(now, freq)?;
        self.flush(now, freq)
    }
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
                while !stop_for_thread.load(Ordering::Relaxed) {
                    for endpoint in &endpoints {
                        let result = endpoint.lock_ok().tick();
                        if let Err(e) = result {
                            tracing::error!(
                                subsystem = "pump",
                                event = "stepcompress_pacer_error",
                                error = ?e,
                                "stepcompress pacer tick failed"
                            );
                        }
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
