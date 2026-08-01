use super::sched::SeamBasis;
use super::{AxisFrame, HeartbeatMsg, PumpMsg, SendError};
use crate::lock_ext::LockExt;
use crate::mcu_config::McuAxisConfig;
use crossbeam_channel::Sender;
use host_rt::host_io::McuHostIo;
use host_rt::host_io::parser::ArgValue;
use std::collections::{HashMap, VecDeque};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, Weak};
use std::thread::JoinHandle;
use std::time::Duration;
use step_shim::{MotorConfig, ShimError, StepFrame, StepShim};

pub const SHIM_RING_DEPTH: u32 = 64;

pub const MOVE_SLOT_RESERVE: u32 = 16;

pub const SEND_LEAD_SECONDS: f64 = 0.250;

pub const CONSUMED_MARGIN_SECONDS: f64 = 0.010;

pub const BACKLOG_CEILING_FRAMES: usize = 8192;

pub const PACER_TICK: Duration = Duration::from_millis(2);

pub type ClockSource = Arc<dyn Fn(u32) -> Option<(u64, f64)> + Send + Sync>;
pub type FrameEgress =
    Arc<dyn Fn(&str, &[(String, ArgValue)]) -> Result<(), SendError> + Send + Sync>;

pub fn host_io_egress(mcu_id: u32, host_io: Weak<McuHostIo>) -> FrameEgress {
    Arc::new(move |name: &str, args: &[(String, ArgValue)]| {
        let io = host_io.upgrade().ok_or_else(|| {
            SendError::Fatal(format!("McuHostIo for stepcompress mcu {mcu_id} detached"))
        })?;
        io.send_args(name, args)
            .map_err(|e| SendError::Transient(format!("stepcompress mcu {mcu_id} {name}: {e:?}")))
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
        motors.push(MotorConfig {
            oid: cfg.stepper_oids[motor],
            microstep_distance: microstep_distance as f32,
            invert_dir: cfg.invert_dir[motor],
            max_steps_per_sample: steps_per_sample as u32,
            sample_rate_hz: sample_rate_hz as f32,
            cycles_per_second,
        });
    }
    Ok(StepcompressEndpoint::new(
        cfg.mcu_id,
        StepShim::new(motors, SHIM_RING_DEPTH),
        cfg.axes.clone(),
        cfg.stepper_oids.clone(),
        host_io_egress(cfg.mcu_id, host_io),
        pump_control,
        clock_of,
        budget,
    ))
}

struct InFlight {
    end_clock: u64,
}

struct PendingRetire {
    waits: Vec<BarrierId>,
    counts: Vec<u32>,
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
    end_clock: u64,
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
    in_flight: Vec<InFlight>,
    step_clock: HashMap<u32, u64>,
    pending_cut: HashMap<u8, (u64, Option<f64>)>,
    pending_retire: VecDeque<PendingRetire>,
    published: Vec<u32>,
    cohort_counts: Vec<u32>,
    next_barrier_seq: HashMap<u32, u32>,
    acked_barrier_seq: HashMap<u32, u32>,
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
            in_flight: Vec::new(),
            step_clock: HashMap::new(),
            pending_cut: HashMap::new(),
            pending_retire: VecDeque::new(),
            published,
            cohort_counts,
            next_barrier_seq: HashMap::new(),
            acked_barrier_seq: HashMap::new(),
        }
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
        (self.egress)(
            "stepcompress_set_position",
            &[
                ("oid".to_string(), ArgValue::Int(i64::from(oid))),
                ("pos".to_string(), ArgValue::Int(i64::from(count))),
            ],
        )
    }

    fn sync_retirement_baseline(&mut self) {
        self.published = self.shim.retired_counts();
        self.cohort_counts.clone_from(&self.published);
    }

    pub fn reset_motor_position(&mut self, motor: usize, count: i64) -> Result<(), String> {
        self.shim
            .halt_at(motor, u64::MAX)
            .map_err(|e| format!("stepcompress mcu {}: {e}", self.mcu_id))?;
        self.shim.reset_position(motor, count);
        self.sync_retirement_baseline();
        Ok(())
    }

    /// Barriers still queued here never reach the mcu, so nothing will ever
    /// ack them — cancel them by advancing the high-water mark. Barriers
    /// already on the wire are acked even when the mcu halt discards them.
    pub fn abort_outbound(&mut self) {
        for out in &self.backlog {
            if let Outbound::Barrier(id) = out.frame {
                let acked = self.acked_barrier_seq.entry(id.oid).or_insert(id.seq);
                *acked = (*acked).max(id.seq);
            }
        }
        self.backlog.clear();
        self.in_flight.clear();
        self.step_clock.clear();
        self.pending_cut.clear();
        self.pending_retire.clear();
    }

    pub fn mark_reanchor(&mut self, axis: u8, at_start_clock: u64, epoch_freq: Option<f64>) {
        self.pending_cut.insert(axis, (at_start_clock, epoch_freq));
    }

    /// How the shim will reproject this axis' piece ends once the pieces being
    /// staged now reach it: the epoch slope of a marked but not-yet-applied
    /// cut, otherwise the slope the shim currently holds. Frames already
    /// egressed carry clocks derived from it, so it cannot be retimed after
    /// the fact — upstream must adopt it, not the other way round.
    ///
    /// Half the shim's seam tolerance is budgeted to duration rewrites; the
    /// other half stays for the producer's own projection rounding, so the
    /// check the shim runs keeps its meaning.
    pub fn seam_basis(&self, axis: u8) -> Option<SeamBasis> {
        let motor = self.axes.iter().position(|&a| a == usize::from(axis))?;
        let freq = match self.pending_cut.get(&axis) {
            Some(&(_, Some(epoch_freq))) => epoch_freq,
            _ => self.shim.motor_cycles_per_second(motor),
        };
        Some(SeamBasis {
            freq,
            skew_budget_cycles: step_shim::MAX_SEAM_SKEW_CYCLES / 2,
        })
    }

    fn cut_stream(&mut self, motor: usize, freq: f64, now: u64) -> Result<(), SendError> {
        let emit_cursor = self.step_clock.get(&self.oids[motor]).copied().unwrap_or(0);
        tracing::info!(
            subsystem = "motion",
            event = "reanchor_cut",
            mcu = self.mcu_id,
            motor,
            emit_cursor,
            "[reanchor] cutting the shim piece stream"
        );
        let (_executed, tail) = self
            .shim
            .halt_at(motor, emit_cursor)
            .map_err(|e| shim_error_to_send_error(self.mcu_id, e))?;
        for frame in tail {
            let end_clock = self.frame_end_clock(now, frame);
            self.backlog.push_back(OutboundFrame {
                frame: Outbound::Step(frame),
                end_clock,
            });
        }
        self.shim.set_motor_cycles_per_second(motor, freq);
        let snapshot = self.shim.retired_counts();
        self.publish_retirement(snapshot);
        Ok(())
    }

    fn publish_retirement(&mut self, snapshot: Vec<u32>) {
        let mut waits = Vec::new();
        for motor in 0..self.oids.len() {
            let before = self.cohort_counts.get(motor).copied().unwrap_or(0);
            let after = snapshot.get(motor).copied().unwrap_or(0);
            if before == after {
                continue;
            }
            let oid = self.oids[motor];
            let seq = {
                let next = self.next_barrier_seq.entry(oid).or_insert(0);
                let seq = *next;
                *next += 1;
                seq
            };
            let end_clock = self.step_clock.get(&oid).copied().unwrap_or(0);
            let id = BarrierId { oid, seq };
            self.backlog.push_back(OutboundFrame {
                frame: Outbound::Barrier(id),
                end_clock,
            });
            waits.push(id);
        }
        if waits.is_empty() {
            return;
        }
        self.cohort_counts.clone_from(&snapshot);
        self.pending_retire.push_back(PendingRetire {
            waits,
            counts: snapshot,
        });
    }

    fn barrier_acked(&self, id: BarrierId) -> bool {
        self.acked_barrier_seq
            .get(&id.oid)
            .is_some_and(|&high_water| high_water >= id.seq)
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
        if seq >= issued {
            return Err(SendError::Fatal(format!(
                "stepcompress mcu {mcu_id}: barrier ack oid={oid} seq={seq} is ahead of the \
                 {issued} barriers issued for that oid"
            )));
        }
        let expected = self.acked_barrier_seq.get(&oid).map_or(0, |&s| s + 1);
        if seq < expected {
            // An abort cancelled this barrier by advancing the high-water
            // mark; the ack was already in flight. Nothing left to release.
            return Ok(());
        }
        if seq != expected {
            return Err(SendError::Fatal(format!(
                "stepcompress mcu {mcu_id}: barrier ack oid={oid} seq={seq} out of order, \
                 expected seq={expected}"
            )));
        }
        self.acked_barrier_seq.insert(oid, seq);
        self.release_retirements();
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

    fn frame_end_clock(&mut self, now: u64, frame: StepFrame) -> u64 {
        match frame {
            StepFrame::ResetStepClock { oid, clock } => {
                let expanded = expand_clock32(now, clock);
                self.step_clock.insert(oid, expanded);
                expanded
            }
            StepFrame::SetNextStepDir { oid, .. } => {
                self.step_clock.get(&oid).copied().unwrap_or(now)
            }
            StepFrame::QueueStep {
                oid,
                interval,
                count,
                add,
            } => {
                let cursor = self.step_clock.entry(oid).or_insert(now);
                *cursor = cursor.saturating_add_signed(queue_step_span(interval, count, add));
                *cursor
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
            let end_clock = self.frame_end_clock(now, frame);
            self.backlog.push_back(OutboundFrame {
                frame: Outbound::Step(frame),
                end_clock,
            });
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
        self.publish_retirement(snapshot);
        Ok(())
    }

    fn flush(&mut self, now: u64, freq: f64) -> Result<(), SendError> {
        let margin = (freq * CONSUMED_MARGIN_SECONDS) as u64;
        let cutoff = now.saturating_sub(margin);
        self.in_flight.retain(|e| e.end_clock > cutoff);
        let egress = Arc::clone(&self.egress);
        while let Some(front) = self.backlog.front() {
            let consumes_slot = matches!(front.frame, Outbound::Step(StepFrame::QueueStep { .. }));
            if consumes_slot && self.in_flight.len() as u32 >= self.budget {
                break;
            }
            let out = self
                .backlog
                .pop_front()
                .expect("backlog front was just observed");
            let (name, args) = frame_args(&out.frame);
            egress(name, &args)?;
            if consumes_slot {
                self.in_flight.push(InFlight {
                    end_clock: out.end_clock,
                });
            }
        }
        self.release_retirements();
        self.post_heartbeat()
    }

    /// The shim counts retirements per MOTOR, in this endpoint's `axes`
    /// order; the pump keys its queues by axis. A stepcompress mcu carrying
    /// only a follower lane (an extruder toolhead) has `axes == [3]`, so
    /// motor 0's count must land on axis 3 or that lane's ring never drains.
    fn retired_by_axis(&self) -> Vec<u32> {
        let max_axis = self.axes.iter().copied().max().unwrap_or(0);
        let mut out = vec![0u32; max_axis + 1];
        for (motor, &axis) in self.axes.iter().enumerate() {
            out[axis] = self.published[motor];
        }
        out
    }

    fn post_heartbeat(&self) -> Result<(), SendError> {
        let mcu_id = self.mcu_id;
        self.pump_control
            .send(PumpMsg::Heartbeat(HeartbeatMsg {
                mcu_id,
                retired_counts: self.retired_by_axis(),
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
                    let end_clock = self.frame_end_clock(now, frame);
                    self.backlog.push_back(OutboundFrame {
                        frame: Outbound::Step(frame),
                        end_clock,
                    });
                }
            }
            let snapshot = self.shim.retired_counts();
            self.publish_retirement(snapshot);
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
            #[allow(clippy::cast_possible_truncation)]
            let cps = self.shim.motor_cycles_per_second(motor) as f32;
            let cut_index = self.pending_cut.get(&frame.axis).and_then(|&(at, _)| {
                frame
                    .pieces
                    .iter()
                    .position(|p| p.start_time >= at || p.end_time(cps) > at)
            });
            match cut_index {
                Some(index) => {
                    let epoch_freq = self.pending_cut[&frame.axis].1.ok_or_else(|| {
                        SendError::Fatal(format!(
                            "stepcompress mcu {mcu_id} axis {}: fresh epoch carried no clock \
                             slope; the shim cannot adopt the producer's timeline",
                            frame.axis
                        ))
                    })?;
                    self.shim
                        .validate_pieces_public(motor, &frame.pieces[..index])
                        .map_err(|e| shim_error_to_send_error(mcu_id, e))?;
                    self.shim
                        .push_pieces(motor, &frame.pieces[..index])
                        .map_err(|e| shim_error_to_send_error(mcu_id, e))?;
                    let cut_clock = self.pending_cut[&frame.axis].0;
                    self.drain_until(now, cut_clock)?;
                    self.drain_into_backlog(now, freq)?;
                    self.cut_stream(motor, epoch_freq, now)?;
                    self.pending_cut.remove(&frame.axis);
                    self.shim
                        .validate_fresh_pieces(motor, &frame.pieces[index..])
                        .map_err(|e| shim_error_to_send_error(mcu_id, e))?;
                    self.shim
                        .push_pieces(motor, &frame.pieces[index..])
                        .map_err(|e| shim_error_to_send_error(mcu_id, e))?;
                }
                None => self
                    .shim
                    .push_pieces(motor, &frame.pieces)
                    .map_err(|e| shim_error_to_send_error(mcu_id, e))?,
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
