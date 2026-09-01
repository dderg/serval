pub mod compress;
pub mod compress_hp;
pub mod ring;
pub mod root_cursor;

use trajectory::{ClockedMotorSpan, ContinuousError};

use compress::compress_with_max_error;
use ring::SpanQueue;
use root_cursor::{StepRoot, StepRootCursor};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepEncoder {
    Classic { max_error_ticks: u32 },
    HighPrecision,
}

#[derive(Debug, Clone, Copy)]
pub struct MotorConfig {
    pub oid: u32,
    pub microstep_distance: f64,
    pub invert_dir: bool,
    pub cycles_per_second: f64,
    pub encoder: StepEncoder,
    /// How far the mcu's classic stepper needs between the last step of one
    /// queued move and the first step of the next. `stepper_event_full`
    /// schedules an unstep `step_pulse_ticks` after every step and
    /// `stepper_load_next` re-arms from that pending unstep, so a run that
    /// starts sooner is loaded behind it: `motion.step_load_late`, then
    /// "Stepper too far in past". Zero means the caller only owes strict
    /// monotonicity (both-edge drivers configure zero pulse ticks).
    pub min_rearm_cycles: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepFrame {
    ResetStepClock {
        oid: u32,
        clock: u32,
    },
    SetNextStepDir {
        oid: u32,
        dir: u8,
    },
    QueueStep {
        oid: u32,
        interval: u32,
        count: u16,
        add: i16,
    },
    QueueStepHp {
        oid: u32,
        interval: u32,
        count: u16,
        add: i16,
        add2: i16,
        shift: i8,
        /// Tick offset of the move's first step from the pre-move step
        /// clock, from the encoder's fixed-point walk. The sink anchors its
        /// emitted clock on this instead of the wire `interval`, which is
        /// only the fixed-point seed.
        first_step: u64,
        /// Tick offset of the move's last step from the pre-move step
        /// clock, from the same walk. Never sent on the wire.
        last_step: u64,
    },
}

#[derive(Debug)]
pub enum ShimError {
    QueueFull {
        motor: usize,
    },
    StepClockRegression {
        motor: usize,
        source_line: u32,
        previous_clock: u64,
        clock: u64,
        step_count: i64,
        advance: i8,
    },
    SpanGap {
        motor: usize,
        expected: u64,
        got: u64,
        tolerance: u64,
    },
    SpanClockDegenerate {
        motor: usize,
        start_clock: u64,
        end_clock: u64,
    },
    SpanFrequencyMismatch {
        motor: usize,
        expected: f64,
        got: f64,
    },
    SpanEval {
        motor: usize,
        error: ContinuousError,
    },
    /// A run's first step lands closer to the committed cursor than the mcu
    /// can re-arm its stepper. Reported instead of emitting it because the
    /// mcu answers such a stream with "Stepper too far in past".
    StepTooSoon {
        motor: usize,
        first: u64,
        committed: u64,
        min_rearm: u64,
    },
    CompressFailure {
        motor: usize,
        detail: String,
    },
}

impl std::fmt::Display for ShimError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::QueueFull { motor } => write!(f, "motor {motor}: span queue full"),
            Self::StepClockRegression {
                motor,
                source_line,
                previous_clock,
                clock,
                step_count,
                advance,
            } => write!(
                f,
                "motor {motor}: line {source_line} step root {clock} did not advance past \
                 {previous_clock} at step count {step_count} with advance {advance}"
            ),
            Self::SpanGap {
                motor,
                expected,
                got,
                tolerance,
            } => write!(
                f,
                "motor {motor}: span starts at {got}, expected {expected} \
                 (+/-{tolerance} cycles of clock-map rounding)"
            ),
            Self::SpanClockDegenerate {
                motor,
                start_clock,
                end_clock,
            } => write!(
                f,
                "motor {motor}: span clocks {start_clock}..{end_clock} do not strictly increase"
            ),
            Self::SpanFrequencyMismatch {
                motor,
                expected,
                got,
            } => write!(
                f,
                "motor {motor}: span carries clock slope {got} Hz but the lane runs at {expected} Hz"
            ),
            Self::SpanEval { motor, error } => {
                write!(f, "motor {motor}: span evaluation failed: {error}")
            }
            Self::StepTooSoon {
                motor,
                first,
                committed,
                min_rearm,
            } => write!(
                f,
                "motor {motor}: run starts at {first}, only {} cycles after the committed \
                 {committed} — the mcu needs {min_rearm} to re-arm after its pending unstep",
                first - committed
            ),
            Self::CompressFailure { motor, detail } => {
                write!(f, "motor {motor}: stepcompress failed: {detail}")
            }
        }
    }
}

impl std::error::Error for ShimError {}

/// The compressor a motor drives, plus whatever it carries between runs. The
/// classic packer starts every run from the base clock alone; the
/// high-precision one owns its least-squares scratch and the interval its next
/// run seeds from.
#[derive(Debug)]
enum Encoder {
    Classic {
        max_error_ticks: u32,
    },
    HighPrecision {
        scratch: compress_hp::HpScratch,
        carry: u32,
    },
}

impl Encoder {
    fn new(encoder: StepEncoder) -> Self {
        match encoder {
            StepEncoder::Classic { max_error_ticks } => Self::Classic { max_error_ticks },
            StepEncoder::HighPrecision => Self::HighPrecision {
                scratch: compress_hp::HpScratch::new(),
                carry: 0,
            },
        }
    }

    /// A volley that re-anchors the mcu's step clock starts from silence, so
    /// nothing a previous run carried applies to it.
    fn rearm(&mut self) {
        match self {
            Self::Classic { .. } => {}
            Self::HighPrecision { carry, .. } => *carry = 0,
        }
    }

    /// Packs the run into wire moves, appends them in emission order, and
    /// answers how many of `clocks` they cover plus the clock the last packed
    /// step lands on.
    fn encode(
        &mut self,
        oid: u32,
        clocks: &[u64],
        base_clock: u64,
        frames: &mut Vec<StepFrame>,
    ) -> Result<(usize, u64), compress::CompressError> {
        let mut clock = base_clock;
        let covered = match self {
            Self::Classic { max_error_ticks } => {
                let (moves, covered) =
                    compress_with_max_error(clocks, base_clock, *max_error_ticks)?;
                for mv in &moves {
                    frames.push(StepFrame::QueueStep {
                        oid,
                        interval: mv.interval,
                        count: mv.count,
                        add: mv.add,
                    });
                    clock = mv.last_clock(clock);
                }
                covered
            }
            Self::HighPrecision { scratch, carry } => {
                let (moves, covered, carry_out) =
                    compress_hp::compress_hp(scratch, clocks, base_clock, *carry)?;
                *carry = carry_out;
                for mv in &moves {
                    frames.push(StepFrame::QueueStepHp {
                        oid,
                        interval: mv.interval,
                        count: mv.count,
                        add: mv.add,
                        add2: mv.add2,
                        shift: mv.shift,
                        first_step: mv.first_step,
                        last_step: mv.last_step,
                    });
                    clock = mv.last_clock(clock);
                }
                covered
            }
        };
        Ok((covered, clock))
    }
}

#[derive(Debug)]
struct MotorState {
    cfg: MotorConfig,
    queue: SpanQueue,
    cursor: StepRootCursor,
    pending: Vec<StepRoot>,
    stepped_clock: Option<u64>,
    last_dir: Option<u8>,
    encoder: Encoder,
}

impl MotorState {
    fn new(cfg: MotorConfig, queue_depth: u32) -> Self {
        Self {
            cursor: StepRootCursor::new(&cfg),
            cfg,
            queue: SpanQueue::new(queue_depth),
            pending: Vec::new(),
            stepped_clock: None,
            last_dir: None,
            encoder: Encoder::new(cfg.encoder),
        }
    }

    fn emit(&mut self, motor: usize, frames: &mut Vec<StepFrame>) -> Result<(), ShimError> {
        let oid = self.cfg.oid;
        while let Some(first) = self.pending.first().copied() {
            let dir = first.dir;
            let run_len = self.pending.iter().take_while(|s| s.dir == dir).count();
            let clocks: Vec<u64> = self.pending[..run_len].iter().map(|s| s.clock).collect();

            let (committed, min_rearm) = match self.stepped_clock {
                Some(clock) => (clock, self.cfg.min_rearm_cycles),
                None => (
                    self.cursor
                        .origin_clock()
                        .expect("origin clock is set before any root is solved"),
                    0,
                ),
            };
            if clocks[0] <= committed {
                return Err(ShimError::CompressFailure {
                    motor,
                    detail: format!(
                        "step clock regression: first step of this run is at {} but the \
                         stream is already committed to {committed} \
                         (stepped_clock={:?}, run_len={run_len}, dir={dir})",
                        clocks[0], self.stepped_clock
                    ),
                });
            }
            if clocks[0] - committed < min_rearm {
                return Err(ShimError::StepTooSoon {
                    motor,
                    first: clocks[0],
                    committed,
                    min_rearm,
                });
            }

            let re_anchoring =
                self.stepped_clock.is_none() || clocks[0] - committed >= compress::CLOCK_DIFF_MAX;
            // `committed` is where the stream is guaranteed silent, not where
            // the volley starts: a lane that holds before it steps has no
            // stepped clock for the whole hold, so its committed clock is the
            // seam the hold began on — seconds or minutes behind the first
            // step. reset_step_clock heads the volley and the mcu shuts down
            // on a late stepper re-arm ("Rescheduled timer in the past"), so
            // a re-anchoring volley bases itself on its own first step.
            let base_clock = if re_anchoring {
                clocks[0] - 1
            } else {
                committed
            };
            if re_anchoring {
                frames.push(StepFrame::ResetStepClock {
                    oid,
                    clock: base_clock as u32,
                });
                self.encoder.rearm();
                if self.stepped_clock.is_none() {
                    self.last_dir = None;
                }
            }
            if self.last_dir != Some(dir) {
                frames.push(StepFrame::SetNextStepDir { oid, dir });
                self.last_dir = Some(dir);
            }
            let run_start = frames.len();
            let (covered, reconstructed) = self
                .encoder
                .encode(oid, &clocks, base_clock, frames)
                .map_err(|e| ShimError::CompressFailure {
                    motor,
                    detail: e.detail,
                })?;
            if covered == 0 {
                return Err(ShimError::CompressFailure {
                    motor,
                    detail: format!(
                        "encoder covered none of the {run_len} step roots of this run \
                         from base clock {base_clock}"
                    ),
                });
            }
            assert!(
                frames.len() > run_start,
                "motor {motor}: encoder covered {covered} of {run_len} step roots without \
                 emitting a move; a set_next_step_dir heading this run would reach the host \
                 unclocked"
            );

            self.stepped_clock = Some(reconstructed);
            self.pending.drain(..covered);
        }
        Ok(())
    }
}

fn drain_motor(
    motor: usize,
    state: &mut MotorState,
    up_to_clock: u64,
    deadline: Option<std::time::Instant>,
) -> Result<Vec<StepFrame>, ShimError> {
    let started = std::time::Instant::now();
    let queued_before = state.queue.len();
    let mut frames = Vec::new();
    let result = state
        .cursor
        .advance(
            motor,
            &state.cfg,
            &mut state.queue,
            up_to_clock,
            &mut state.pending,
            deadline,
        )
        .and_then(|()| state.emit(motor, &mut frames));
    let elapsed = started.elapsed();
    let evals = crate::root_cursor::EVAL_COUNT.with(std::cell::Cell::take);
    let bounds = crate::root_cursor::BOUNDS_COUNT.with(std::cell::Cell::take);
    let windows = crate::root_cursor::WINDOW_COUNT.with(std::cell::Cell::take);
    let cert_none = crate::root_cursor::CERT_NONE_COUNT.with(std::cell::Cell::take);
    let pruned = crate::root_cursor::PRUNE_COUNT.with(std::cell::Cell::take);
    if elapsed > std::time::Duration::from_millis(4) {
        tracing::warn!(
            subsystem = "pump",
            event = "shim_motor_drain_slow",
            motor,
            elapsed_us = elapsed.as_micros() as u64,
            queued_before,
            evals,
            bounds,
            windows,
            cert_none,
            pruned,
            "one motor's root search dominated the shim drain"
        );
    }
    result.map(|()| frames)
}

#[derive(Debug)]
pub struct StepShim {
    motors: Vec<MotorState>,
    queue_depth: u32,
    drain_rotation: usize,
}

impl StepShim {
    pub fn new(motors: Vec<MotorConfig>, queue_depth: u32) -> Self {
        Self {
            motors: motors
                .into_iter()
                .map(|cfg| MotorState::new(cfg, queue_depth))
                .collect(),
            queue_depth,
            drain_rotation: 0,
        }
    }

    /// Admit a whole run or none of it: the batch is validated against the
    /// motor's seam and remaining room before the first view is queued.
    pub fn push_spans(
        &mut self,
        motor: usize,
        views: &[ClockedMotorSpan],
    ) -> Result<(), ShimError> {
        let state = self.motor_mut(motor);
        state.queue.validate(motor, views)?;
        for view in views {
            state.queue.push(motor, view.clone())?;
        }
        Ok(())
    }

    /// Sanction a forward-only seam jump: the stream time crossed a
    /// drained-to-rest hole (a dwell) with no spans, so the next span for this
    /// motor starts later than the end clock of the previous one. No steps, no
    /// clock reset — only the seam expectation moves. A jump BACKWARD past the
    /// rounding tolerance is still an overlap and stays loud.
    pub fn accept_forward_seam_gap(
        &mut self,
        motor: usize,
        at_start_clock: u64,
    ) -> Result<(), ShimError> {
        self.motor_mut(motor)
            .queue
            .accept_forward_gap(motor, at_start_clock)
    }

    pub fn detach_span_seam(&mut self, motor: usize) -> Result<(), ShimError> {
        self.motor_mut(motor).queue.detach_seam(motor)
    }

    pub fn commanded_steps(&self, motor: usize) -> i64 {
        self.motors[motor].cursor.step_count()
    }

    pub fn commanded_position(&self, motor: usize) -> f64 {
        self.motors[motor].cursor.position()
    }

    pub fn invert_dir(&self, motor: usize) -> bool {
        self.motors[motor].cfg.invert_dir
    }

    pub fn motor_encoder(&self, motor: usize) -> StepEncoder {
        self.motors[motor].cfg.encoder
    }

    /// The clock slope this motor's spans must carry. A view clocked on any
    /// other slope belongs to a different epoch and is refused.
    pub fn motor_cycles_per_second(&self, motor: usize) -> f64 {
        self.motors[motor].cfg.cycles_per_second
    }

    pub fn pending_roots(&self) -> usize {
        self.motors.iter().map(|m| m.pending.len()).sum()
    }

    pub fn queued_spans(&self) -> usize {
        self.motors.iter().map(|m| m.queue.len()).sum()
    }

    pub fn drain(&mut self, up_to_clock: u64) -> Result<Vec<StepFrame>, ShimError> {
        self.drain_budgeted(up_to_clock, None)
    }

    /// Like [`Self::drain`], but the root search stops emitting new windows
    /// once `deadline` passes; the per-motor frontier persists, so later
    /// calls resume where this one stopped. A bulk refill (post-cut lead
    /// rebuild) then amortizes across pacer ticks instead of consuming the
    /// resume volley's delivery margin in one synchronous pass.
    pub fn drain_budgeted(
        &mut self,
        up_to_clock: u64,
        deadline: Option<std::time::Instant>,
    ) -> Result<Vec<StepFrame>, ShimError> {
        let parallel = deadline.is_some()
            && cfg!(not(target_arch = "wasm32"))
            && self.motors.len() > 1
            && std::thread::available_parallelism().is_ok_and(|cores| cores.get() > 1);
        if parallel {
            return std::thread::scope(|scope| {
                let handles = self
                    .motors
                    .iter_mut()
                    .enumerate()
                    .map(|(motor, state)| {
                        scope.spawn(move || drain_motor(motor, state, up_to_clock, deadline))
                    })
                    .collect::<Vec<_>>();
                let mut frames = Vec::new();
                for handle in handles {
                    frames.extend(handle.join().expect("motor root search thread panicked")?);
                }
                Ok(frames)
            });
        }

        let count = self.motors.len();
        let start = if count == 0 {
            0
        } else {
            self.drain_rotation = (self.drain_rotation + 1) % count;
            self.drain_rotation
        };
        let mut frames = Vec::new();
        for offset in 0..count {
            let motor = (start + offset) % count;
            frames.extend(drain_motor(
                motor,
                &mut self.motors[motor],
                up_to_clock,
                deadline,
            )?);
        }
        Ok(frames)
    }

    /// Views this motor has converted to roots and released, plus the views a
    /// cut abandoned. Both free their room upstream; only the converted ones
    /// ever reached the wire.
    pub fn consumed_counts(&self) -> Vec<u32> {
        self.motors.iter().map(|m| m.queue.released()).collect()
    }

    pub fn queue_depth(&self) -> u32 {
        self.queue_depth
    }

    pub fn halt_at(
        &mut self,
        motor: usize,
        clock: u64,
    ) -> Result<(i64, Vec<StepFrame>), ShimError> {
        let executed = self.derived_halt_count(motor, clock);
        let frames = self.halt_at_seeded(motor, clock, executed)?;
        Ok((executed, frames))
    }

    pub fn halt_at_with_executed(
        &mut self,
        motor: usize,
        clock: u64,
        executed: i64,
    ) -> Result<(i64, Vec<StepFrame>), ShimError> {
        let expected = self.derived_halt_count(motor, clock);
        let frames = self.halt_at_seeded(motor, clock, executed)?;
        Ok((expected, frames))
    }

    pub fn expected_halt_count(&self, motor: usize, clock: u64) -> i64 {
        self.derived_halt_count(motor, clock)
    }

    fn derived_halt_count(&self, motor: usize, clock: u64) -> i64 {
        let state = &self.motors[motor];
        let unexecuted: i64 = state
            .pending
            .iter()
            .filter(|root| root.clock > clock)
            .map(|root| i64::from(root.advance))
            .sum();
        state.cursor.step_count() - unexecuted
    }

    fn halt_at_seeded(
        &mut self,
        motor: usize,
        clock: u64,
        executed: i64,
    ) -> Result<Vec<StepFrame>, ShimError> {
        let state = self.motor_mut(motor);
        state.pending.retain(|root| root.clock <= clock);
        let mut frames = Vec::new();
        state.emit(motor, &mut frames)?;
        state.queue.abandon_all();
        state.cursor.reset_to(executed, clock.saturating_add(1));
        state.stepped_clock = None;
        state.last_dir = None;
        state.encoder.rearm();
        Ok(frames)
    }

    pub fn set_motor_cycles_per_second(&mut self, motor: usize, freq: f64) {
        let state = self.motor_mut(motor);
        if state.cfg.cycles_per_second == freq {
            return;
        }
        state.cfg.cycles_per_second = freq;
        state.cursor.retime(&state.cfg);
    }

    pub fn reset_position(&mut self, motor: usize, count: i64) {
        let state = self.motor_mut(motor);
        state.pending.clear();
        state.cursor.reset_to(count, 0);
        state.cursor.set_step_remainder(0.0);
        state.stepped_clock = None;
        state.last_dir = None;
        state.encoder.rearm();
    }

    fn motor_mut(&mut self, motor: usize) -> &mut MotorState {
        let count = self.motors.len();
        self.motors
            .get_mut(motor)
            .unwrap_or_else(|| panic!("motor index {motor} out of range for {count} motors"))
    }
}

#[cfg(test)]
#[path = "lib_tests.rs"]
mod tests;

#[cfg(test)]
#[path = "rearm_tests.rs"]
mod rearm_tests;
