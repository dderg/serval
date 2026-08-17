// Sample-stream transport sink.
//
// Where the stepcompress sink turns lowered pieces into step/dir volleys, this
// sink turns the same pieces into `SampleRun`s: uniformly spaced absolute
// positions on the lane's own fixed-point grid. The quantization is the step
// path's, entry point for entry point — `arm_piece` + `eval_pos_vel` +
// `quantize_step_delta` against the lane's position quantum — so a phase lane
// driven from samples lands on exactly the microsteps the step path would have
// pulsed. What the sink drops is the sub-sample crossing arithmetic: the
// executor receives positions, not edges, and interpolates them itself.
//
// Retirement is simpler here than on the stepcompress path, and deliberately
// so: in sample mode the mcu never holds pieces, only samples, so a piece is
// retired once it has been sampled. Barriers remain for the one thing that
// needs a receipt — reconciling a re-anchor cut whose samples already reached
// the wire.

use std::collections::{HashMap, VecDeque};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

use crossbeam_channel::Sender;
use host_rt::host_io::parser::ArgValue;
use runtime::motion_core::{ArmedPiece, arm_piece};
use runtime::piece_ring::PieceEntry;
use runtime::sample_run::{
    SAMPLE_RUN_COUNT_MAX, SAMPLE_RUN_DATA_MAX, SampleRunBuf, SampleRunError, delta_bytes,
    encode_deltas,
};
use runtime::sample_wire::{SAMPLE_ANCHOR_NAME, SAMPLE_OVERLAY_NAME, SAMPLE_RUN_NAME};
use runtime::sub_sample_timing::quantize_step_delta;

use super::barrier_ledger::{AckFault, BarrierId, BarrierLedger};
use super::pump_loop::pump_past_guard_secs;
use super::stepcompress_sink::{
    BARRIER_ACK_DEADLINE_SECONDS, CONSUMED_MARGIN_SECONDS, ClockSource, FrameEgress, PACER_TICK,
    SEND_LEAD_SECONDS,
};
use super::{AxisFrame, HeartbeatMsg, PumpMsg, SendError};
use crate::lock_ext::LockExt;

/// Bounded in-flight sample window per lane: how many sent-but-unconsumed runs
/// a lane may have outstanding. At the 48-sample wire cap this is far more than
/// [`SEND_LEAD_SECONDS`] of lead at every rate the executors run, so the lead
/// paces the stream and this ceiling is what catches a lane whose consumption
/// has stopped.
pub const SAMPLE_WINDOW_RUNS: usize = 64;

/// A backlog this deep means the transport is not draining: every run past it
/// is lead the mcu will never receive in time.
pub const SAMPLE_BACKLOG_CEILING_RUNS: usize = 4096;

const SAMPLE_BARRIER_NAME: &str = "sample_barrier";

/// Reads back a lane's executed position: `sample_get_position` answered by
/// `sample_position`, as `(clock, position)`.
pub type SamplePositionQuery = Arc<dyn Fn(u32) -> Result<(u64, i32), String> + Send + Sync>;

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

/// A piece carrying a `motor_mask` is relativized to start at zero, so its
/// samples ride against their own frame and leave the lane's absolute frame
/// where the last kinematic piece left it — the same split the step path keeps
/// between `last_step_count` and `overlay_step_frame`, and the reason overlay
/// runs leave as `sample_overlay` rather than on the lane's abutting stream.
#[derive(Clone, Copy, Debug)]
struct OverlayFrame {
    p_prev: f32,
    position: i64,
    step_phase: f32,
}

/// A seam the sink must handle when the marked piece reaches it, in stream
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
    held: Vec<PieceEntry>,
}

struct InFlightRun {
    end_clock: u64,
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

    fn consumes_window(&self) -> bool {
        matches!(self, Self::Run { .. } | Self::Overlay { .. })
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
    pieces: VecDeque<PieceEntry>,
    armed: Option<ArmedPiece>,
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
    in_flight: VecDeque<InFlightRun>,
    seams: VecDeque<PendingSeam>,
    cut: Option<PendingSampleCut>,
    retired: u32,
}

impl SampleLane {
    fn new(cfg: SampleLaneConfig) -> Result<Self, SendError> {
        let sample_period_cycles = cfg.sample_period_cycles()?;
        Ok(Self {
            cfg,
            sample_period_cycles,
            pieces: VecDeque::new(),
            armed: None,
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
            in_flight: VecDeque::new(),
            seams: VecDeque::new(),
            cut: None,
            retired: 0,
        })
    }

    #[allow(clippy::cast_possible_truncation)]
    fn cycles_per_second_f32(&self) -> f32 {
        self.cfg.cycles_per_second as f32
    }

    fn window_full(&self) -> bool {
        self.in_flight.len() >= SAMPLE_WINDOW_RUNS
    }

    fn reset_to(&mut self, position: i64, resume_floor: u64) {
        self.armed = None;
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

    /// Sample every piece the lane holds whose samples land at or before
    /// `sample_to`, closing runs as the wire budget fills.
    fn sample_until(&mut self, sample_to: u64, out: &mut Vec<ClosedRun>) -> Result<(), SendError> {
        let cps = self.cycles_per_second_f32();
        while let Some(piece) = self.pieces.front().copied() {
            let piece_end = match self.pieces.get(1) {
                Some(next) => piece.end_time(cps).min(next.start_time),
                None => piece.end_time(cps),
            };
            if self
                .armed
                .is_none_or(|a| a.piece_start_cycles != piece.start_time)
            {
                self.arm(&piece, cps, out)?;
            }
            let armed = self
                .armed
                .ok_or_else(|| self.fatal("the piece armed above is gone"))?;
            loop {
                if self.window_full() {
                    return Ok(());
                }
                let next_sample = self.prev_sample + u64::from(self.sample_period_cycles);
                if next_sample > sample_to {
                    return Ok(());
                }
                if next_sample > piece_end {
                    break;
                }
                self.emit_sample(&armed, next_sample, out)?;
                self.prev_sample = next_sample;
            }
            self.pieces.pop_front();
            self.retired = self.retired.wrapping_add(1);
            self.armed = None;
        }
        Ok(())
    }

    fn arm(
        &mut self,
        piece: &PieceEntry,
        cps: f32,
        out: &mut Vec<ClosedRun>,
    ) -> Result<(), SendError> {
        let armed = arm_piece(piece, cps);
        let overlay_piece = piece.motor_mask != 0;
        if overlay_piece != self.run_is_overlay {
            self.close_run(out)?;
            self.run_is_overlay = overlay_piece;
        }
        self.overlay = overlay_piece.then_some(OverlayFrame {
            p_prev: 0.0,
            position: 0,
            step_phase: 0.0,
        });
        if self.origin_clock.is_none() {
            let begin = piece.start_time.max(self.resume_floor);
            self.prev_sample = begin;
            let (position, _velocity) = armed.eval_pos_vel(begin);
            if !self.positioned {
                self.p_prev = position;
                self.positioned = true;
            }
            self.origin_clock = Some(begin);
        } else {
            let next_sample = self.prev_sample + u64::from(self.sample_period_cycles);
            if piece.start_time > next_sample {
                return Err(self.fatal(&format!(
                    "piece at clock {} leaves a {} tick hole after the sample at {} — a hole in \
                     the stream needs an explicit re-anchor, never a padded start",
                    piece.start_time,
                    piece.start_time - self.prev_sample,
                    self.prev_sample
                )));
            }
        }
        self.armed = Some(armed);
        Ok(())
    }

    fn emit_sample(
        &mut self,
        armed: &ArmedPiece,
        now: u64,
        out: &mut Vec<ClosedRun>,
    ) -> Result<(), SendError> {
        let (p_end, _v_end) = armed.eval_pos_vel(now);
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
            fatal: None,
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

    pub fn lane_positions(&self) -> Vec<i64> {
        self.lanes.iter().map(|lane| lane.position).collect()
    }

    pub fn in_flight_runs(&self) -> Vec<usize> {
        self.lanes.iter().map(|lane| lane.in_flight.len()).collect()
    }

    pub fn backlog_len(&self) -> usize {
        self.backlog.len()
    }

    fn latched_fatal(&self) -> Option<SendError> {
        self.fatal.clone().map(SendError::Fatal)
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

    fn queue_outbound(&mut self, lane: usize, frame: Outbound, start_clock: u64, end_clock: u64) {
        let enqueue_order = self.next_outbound_order;
        self.next_outbound_order = self.next_outbound_order.wrapping_add(1);
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
            lane.pieces.clear();
            lane.seams.clear();
            lane.cut = None;
            lane.in_flight.clear();
            lane.reset_to(position, now);
        }
        self.backlog.clear();
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
        if let Some(latched) = self.latched_fatal() {
            return Err(latched);
        }
        self.send_frames_inner(mcu_id, frames)
            .map_err(|e| self.escalate(e))
    }

    fn next_seam(
        &self,
        index: usize,
        rest: &[PieceEntry],
    ) -> Result<Option<(PendingSeam, usize)>, SendError> {
        let lane = self.lane_ref(index)?;
        let Some(seam) = lane.seams.front().copied() else {
            return Ok(None);
        };
        let at = seam.at();
        let cps = lane.cycles_per_second_f32();
        Ok(rest
            .iter()
            .position(|piece| piece.start_time >= at || piece.end_time(cps) > at)
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
            if self.lane_ref(index)?.cut.is_some() {
                let pieces = frame.pieces.clone();
                if let Some(cut) = self.lane_mut(index)?.cut.as_mut() {
                    cut.held.extend(pieces);
                }
                continue;
            }
            let mut rest: &[PieceEntry] = &frame.pieces;
            loop {
                let Some((seam, split)) = self.next_seam(index, rest)? else {
                    let tail = rest.to_vec();
                    self.lane_mut(index)?.pieces.extend(tail);
                    break;
                };
                let (head, tail) = rest.split_at(split);
                {
                    let head = head.to_vec();
                    let lane = self.lane_mut(index)?;
                    lane.pieces.extend(head);
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

    /// Returns whether the pieces past the seam must wait for a barrier.
    fn apply_seam(
        &mut self,
        index: usize,
        seam: PendingSeam,
        tail: &[PieceEntry],
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
                if self.lane_ref(index)?.in_flight.is_empty() {
                    self.backlog.retain(|out| out.lane != index);
                    let lane = self.lane_mut(index)?;
                    lane.pieces.clear();
                    lane.cfg.cycles_per_second = epoch_freq;
                    lane.sample_period_cycles = lane.cfg.sample_period_cycles()?;
                    let position = lane.position;
                    lane.reset_to(position, at);
                    lane.pieces.extend(tail.iter().copied());
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
                lane.pieces.clear();
                lane.cut = Some(PendingSampleCut {
                    barrier,
                    cut_at: at,
                    epoch_freq,
                    expected_position,
                    held: tail.to_vec(),
                });
                self.queue_outbound(index, Outbound::Barrier(barrier), at, at);
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
            let end_clock = run
                .start_clock
                .saturating_add(u64::from(run.interval) * u64::from(count));
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
            self.queue_outbound(index, frame, run.start_clock, end_clock);
        }
        Ok(())
    }

    fn drain_into_backlog(&mut self, now: u64, freq: f64) -> Result<(), SendError> {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let lead = (freq * SEND_LEAD_SECONDS) as u64;
        let sample_to = now.saturating_add(lead);
        for index in 0..self.lanes.len() {
            self.sample_lane_until(index, sample_to)?;
            let mut closed = Vec::new();
            self.lane_mut(index)?.close_run(&mut closed)?;
            self.emit_closed(index, closed)?;
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

    fn flush(&mut self, now: u64, freq: f64) -> Result<(), SendError> {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let margin = (freq * CONSUMED_MARGIN_SECONDS) as u64;
        let cutoff = now.saturating_sub(margin);
        for lane in &mut self.lanes {
            while lane
                .in_flight
                .front()
                .is_some_and(|entry| entry.end_clock <= cutoff)
            {
                lane.in_flight.pop_front();
            }
        }
        self.order_backlog_by_deadline();
        let guard_secs = pump_past_guard_secs();
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let stale_by = (freq * guard_secs) as u64;

        let mut burst: Vec<(&'static str, Vec<(String, ArgValue)>)> = Vec::new();
        let mut committed: Vec<(usize, u64)> = Vec::new();
        let mut sent_barriers: Vec<BarrierId> = Vec::new();
        let mut extra_window: HashMap<usize, usize> = HashMap::new();
        let mut stale: Option<SendError> = None;
        for out in &self.backlog {
            let lane = self.lane_ref(out.lane)?;
            if out.frame.consumes_window() {
                let extra = extra_window.get(&out.lane).copied().unwrap_or(0);
                if lane.in_flight.len() + extra >= SAMPLE_WINDOW_RUNS {
                    break;
                }
            }
            if !matches!(out.frame, Outbound::Barrier(_))
                && out.start_clock.saturating_add(stale_by) < now
            {
                #[allow(clippy::cast_precision_loss)]
                let late_us = (now - out.start_clock) as f64 * 1e6 / freq;
                stale = Some(SendError::Fatal(format!(
                    "sample endpoint mcu {}: {} at clock {} is {late_us:.0} us behind the \
                     projected mcu clock {now}, past the {guard_secs} s floor margin. \
                     {SEND_LEAD_SECONDS} s of lead was not delivered: {} runs backlogged, \
                     {} in flight on lane {}",
                    self.mcu_id,
                    out.frame.kind(),
                    out.start_clock,
                    self.backlog.len(),
                    lane.in_flight.len(),
                    out.lane
                )));
                break;
            }
            burst.push(frame_args(&out.frame));
            if out.frame.consumes_window() {
                *extra_window.entry(out.lane).or_insert(0) += 1;
                committed.push((out.lane, out.end_clock));
            }
            if let Outbound::Barrier(id) = out.frame {
                sent_barriers.push(id);
            }
        }
        if !burst.is_empty() {
            (self.egress)(&burst)?;
            self.backlog.drain(..burst.len());
            for (lane_index, end_clock) in committed {
                self.lane_mut(lane_index)?
                    .in_flight
                    .push_back(InFlightRun { end_clock });
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

    fn post_heartbeat(&self) -> Result<(), SendError> {
        let retired_counts = self.retired_counts();
        self.pump_control
            .send(PumpMsg::Heartbeat(HeartbeatMsg {
                mcu_id: self.mcu_id,
                consumed_counts: Some(retired_counts.clone()),
                retired_counts,
            }))
            .map_err(|_| {
                SendError::Fatal(format!(
                    "sample endpoint mcu {}: the pump control channel is closed",
                    self.mcu_id
                ))
            })
    }

    pub fn on_barrier_ack(&mut self, oid: u32, seq: u32) -> Result<(), SendError> {
        if let Some(latched) = self.latched_fatal() {
            return Err(latched);
        }
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
        lane.in_flight.clear();
        lane.cfg.cycles_per_second = cut.epoch_freq;
        lane.sample_period_cycles = lane.cfg.sample_period_cycles()?;
        lane.reset_to(i64::from(executed_position), cut.cut_at);
        lane.pieces.extend(cut.held.iter().copied());
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
        if let Some(latched) = self.latched_fatal() {
            return Err(latched);
        }
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
                    std::thread::sleep(PACER_TICK);
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
