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

/// Move-pool slots the endpoint never claims. The MCU's `move_count` pool is
/// shared: klippy reserves slots through `request_move_queue_slot` for its own
/// scheduled commands, and `queue_digital_out` / `queue_pwm_out` allocate from
/// the same free list. Sizing the stepper budget at the full pool would let a
/// heater or fan update hit an empty pool and shut the printer down.
pub const MOVE_SLOT_RESERVE: u32 = 16;

/// How far ahead of the estimated MCU clock the shim is drained. Matches the
/// order of Klipper's step-generation lookahead: deep enough that a 2 ms pacer
/// tick plus serial wire time never starves the MCU, shallow enough that a
/// re-plan or a trip discards only a fraction of a second of committed steps.
pub const SEND_LEAD_SECONDS: f64 = 0.250;

/// A sent move is only counted as consumed once its last step clock is this
/// far behind the clock estimate. Covers the estimate's own skew and the
/// ack-driven staleness of `mcu_now` so the budget is never released early.
pub const CONSUMED_MARGIN_SECONDS: f64 = 0.010;

/// Outbound frames allowed to wait for budget. `SEND_LEAD_SECONDS` of steps
/// for a handful of motors is a few hundred frames; an order of magnitude of
/// headroom above that and the backlog is provably not draining.
pub const BACKLOG_CEILING_FRAMES: usize = 8192;

/// Pacer period. The pump only calls `send_frames` when new pieces arrive, so
/// this thread is what releases backlog as the MCU clock advances.
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
    watermark: u64,
    counts: Vec<u32>,
}

struct OutboundFrame {
    frame: StepFrame,
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
}

fn shim_error_to_send_error(mcu_id: u32, error: ShimError) -> SendError {
    match error {
        ShimError::RingFull { motor } => SendError::Transient(format!(
            "stepcompress mcu {mcu_id} motor {motor}: shim ring full"
        )),
        other => SendError::Fatal(format!("stepcompress mcu {mcu_id}: {other:?}")),
    }
}

fn frame_args(frame: StepFrame) -> (&'static str, Vec<(String, ArgValue)>) {
    match frame {
        StepFrame::QueueStep {
            oid,
            interval,
            count,
            add,
        } => (
            "queue_step",
            vec![
                ("oid".to_string(), ArgValue::Int(i64::from(oid))),
                ("interval".to_string(), ArgValue::Int(i64::from(interval))),
                ("count".to_string(), ArgValue::Int(i64::from(count))),
                ("add".to_string(), ArgValue::Int(i64::from(add))),
            ],
        ),
        StepFrame::SetNextStepDir { oid, dir } => (
            "set_next_step_dir",
            vec![
                ("oid".to_string(), ArgValue::Int(i64::from(oid))),
                ("dir".to_string(), ArgValue::Int(i64::from(dir))),
            ],
        ),
        StepFrame::ResetStepClock { oid, clock } => (
            "reset_step_clock",
            vec![
                ("oid".to_string(), ArgValue::Int(i64::from(oid))),
                ("clock".to_string(), ArgValue::Int(i64::from(clock))),
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

    /// Seeds motor step counters host-side for an externally set position.
    /// `pos_steps` is one entry per configured axis, in `axes` order. The
    /// committed stream described the old origin, so both the unsent backlog
    /// and the shim's queued pieces are discarded; the next drain re-anchors
    /// with `reset_step_clock` from the new counter.
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
        }
        self.published = self.shim.retired_counts();
        self.post_heartbeat()
    }

    /// One motor's counterpart of [`Self::reset_position`], for the homing
    /// reconcile that reseeds a single tripped lane. `halt_at` clears the
    /// queued pieces and the seam expectation too: the post-trip stream is a
    /// new timeline and must not be held contiguous with the aborted one.
    pub fn reset_motor_position(&mut self, motor: usize, count: i64) -> Result<(), String> {
        self.shim
            .halt_at(motor, u64::MAX)
            .map_err(|e| format!("stepcompress mcu {}: {e}", self.mcu_id))?;
        self.shim.reset_position(motor, count);
        Ok(())
    }

    /// Discards everything the endpoint has not yet put on the wire. Paired
    /// with `StepShim::halt_at` / `reset_position`: after a trip the committed
    /// step stream is void, so shipping the leftover backlog would drive the
    /// motor past the stop.
    pub fn abort_outbound(&mut self) {
        self.backlog.clear();
        self.in_flight.clear();
        self.step_clock.clear();
        self.pending_cut.clear();
    }

    /// Records that `axis`'s next fresh-epoch piece starts at
    /// `at_start_clock`. The cut is applied when that exact piece is pushed,
    /// not when the mark arrives: a bundle can carry the tail of the old
    /// epoch and the head of the new one, and cutting ahead of the whole
    /// bundle would re-create the very `PieceGap` it exists to prevent.
    pub fn mark_reanchor(&mut self, axis: u8, at_start_clock: u64, epoch_freq: Option<f64>) {
        self.pending_cut.insert(axis, (at_start_clock, epoch_freq));
    }

    /// Cuts one motor's piece stream at the last step already emitted, so the
    /// next piece may start at any clock instead of tripping the shim's
    /// `PieceGap` contiguity check against a timeline that no longer exists.
    ///
    /// Frames already emitted (sent or waiting in the backlog) still describe
    /// real steps and are kept, so the step counter stays exact; only the
    /// sampled-but-unemitted tail past the emit cursor is discarded, and
    /// `halt_at` subtracts exactly those steps. The freed ring slots are
    /// republished so the pump gets its credit back.
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
            self.backlog.push_back(OutboundFrame { frame, end_clock });
        }
        self.shim.set_motor_cycles_per_second(motor, freq);
        let snapshot = self.shim.retired_counts();
        self.publish_retirement(snapshot);
        Ok(())
    }

    /// Retirement is only observable once every frame it covers has been
    /// sent, so a snapshot rides on the last outbound frame and is published
    /// immediately only when nothing is waiting.
    fn publish_retirement(&mut self, snapshot: Vec<u32>) {
        let watermark = self
            .backlog
            .iter()
            .map(|f| f.end_clock)
            .chain(self.in_flight.iter().map(|f| f.end_clock))
            .max()
            .unwrap_or(0);
        self.pending_retire.push_back(PendingRetire {
            watermark,
            counts: snapshot,
        });
    }

    fn release_retirements(&mut self, cutoff: u64) {
        while let Some(front) = self.pending_retire.front() {
            let unsent = self.backlog.iter().any(|f| f.end_clock <= front.watermark);
            if unsent || front.watermark > cutoff {
                break;
            }
            let done = self
                .pending_retire
                .pop_front()
                .expect("front was just observed");
            self.published = done.counts;
        }
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
            self.backlog.push_back(OutboundFrame { frame, end_clock });
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
            let consumes_slot = matches!(front.frame, StepFrame::QueueStep { .. });
            if consumes_slot && self.in_flight.len() as u32 >= self.budget {
                break;
            }
            let out = self
                .backlog
                .pop_front()
                .expect("backlog front was just observed");
            let (name, args) = frame_args(out.frame);
            egress(name, &args)?;
            if consumes_slot {
                self.in_flight.push(InFlight {
                    end_clock: out.end_clock,
                });
            }
        }
        self.release_retirements(cutoff);
        self.post_heartbeat()
    }

    fn post_heartbeat(&self) -> Result<(), SendError> {
        let mcu_id = self.mcu_id;
        self.pump_control
            .send(PumpMsg::Heartbeat(HeartbeatMsg {
                mcu_id,
                retired_counts: self.published.clone(),
            }))
            .map_err(|_| {
                SendError::Fatal(format!(
                    "stepcompress mcu {mcu_id}: pump control channel closed"
                ))
            })
    }

    /// Periodic driver. The pump only calls `send_frames` when new pieces
    /// arrive, so this is what turns the advancing MCU clock into progress:
    /// it samples the pieces the widening drain horizon now covers, releases
    /// the frames the freed move budget now allows, and posts the heartbeat.
    /// Sampling here is what retires the last pieces of a stream — without it
    /// a finished move never reaches full retirement and the pump's
    /// `RETIREMENT_STALL_FATAL` watchdog fires.
    ///
    /// Does nothing (and needs no clock) while there is nothing outstanding.
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
                    self.backlog.push_back(OutboundFrame { frame, end_clock });
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
        let cps = freq as f32;
        for frame in frames {
            let motor = self.motor_of(frame.axis)?;
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
                        .validate_fresh_pieces(motor, &frame.pieces[index..])
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

/// Owns the pacer thread; stopping and joining it on drop keeps a pipeline
/// respawn from leaving the previous pacer behind.
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
