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
                previous_clock,
                clock,
                step_count,
                advance,
            } => write!(
                f,
                "motor {motor}: step root {clock} did not advance past {previous_clock} \
                 at step count {step_count} with advance {advance}"
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

/// A run's compressed moves, in emission order. The classic and high-precision
/// encoders produce different move types; the emit loop dispatches on this so
/// the reset/dir/drain tail is shared.
enum Encoded {
    Classic(Vec<compress::StepMove>, usize),
    Hp(Vec<compress_hp::StepMoveHp>, usize, u32),
}

impl Encoded {
    fn covered(&self) -> usize {
        match self {
            Self::Classic(_, covered) | Self::Hp(_, covered, _) => *covered,
        }
    }
}

#[derive(Debug)]
struct MotorState {
    cfg: MotorConfig,
    queue: SpanQueue,
    cursor: StepRootCursor,
    pending: Vec<StepRoot>,
    last_step_clock: u64,
    needs_reset: bool,
    last_dir: Option<u8>,
    next_expected_interval: u32,
    hp_scratch: Option<compress_hp::HpScratch>,
}

impl MotorState {
    fn new(cfg: MotorConfig, queue_depth: u32) -> Self {
        Self {
            cursor: StepRootCursor::new(&cfg),
            cfg,
            queue: SpanQueue::new(queue_depth),
            pending: Vec::new(),
            last_step_clock: 0,
            needs_reset: true,
            last_dir: None,
            next_expected_interval: 0,
            hp_scratch: matches!(cfg.encoder, StepEncoder::HighPrecision)
                .then(compress_hp::HpScratch::new),
        }
    }

    fn emit(&mut self, motor: usize, frames: &mut Vec<StepFrame>) -> Result<(), ShimError> {
        let oid = self.cfg.oid;
        while let Some(first) = self.pending.first().copied() {
            let dir = first.dir;
            let run_len = self.pending.iter().take_while(|s| s.dir == dir).count();
            let clocks: Vec<u64> = self.pending[..run_len].iter().map(|s| s.clock).collect();

            let committed = if self.needs_reset {
                self.cursor
                    .origin_clock()
                    .expect("origin clock is set before any root is solved")
            } else {
                self.last_step_clock
            };
            if clocks[0] <= committed {
                return Err(ShimError::CompressFailure {
                    motor,
                    detail: format!(
                        "step clock regression: first step of this run is at {} but the \
                         stream is already committed to {committed} (needs_reset={}, \
                         last_step_clock={}, run_len={run_len}, dir={dir})",
                        clocks[0], self.needs_reset, self.last_step_clock
                    ),
                });
            }

            let min_rearm = if self.needs_reset {
                0
            } else {
                self.cfg.min_rearm_cycles
            };
            if clocks[0] - committed < min_rearm {
                return Err(ShimError::StepTooSoon {
                    motor,
                    first: clocks[0],
                    committed,
                    min_rearm,
                });
            }

            let out_of_reach = clocks[0] - committed >= compress::CLOCK_DIFF_MAX;
            let re_anchoring = self.needs_reset || out_of_reach;
            // `committed` is where the stream is guaranteed silent, not where
            // the volley starts: a lane that holds before it steps keeps
            // `needs_reset` for the whole hold, so its origin clock is the
            // seam the hold began on — seconds or minutes behind the first
            // step. reset_step_clock heads the volley and the mcu shuts down
            // on a late stepper re-arm ("Rescheduled timer in the past"), so
            // a re-anchoring volley bases itself on its own first step.
            let base_clock = if re_anchoring {
                clocks[0] - 1
            } else {
                committed
            };
            let hp_carry = if re_anchoring {
                0
            } else {
                self.next_expected_interval
            };
            let encoded = match self.cfg.encoder {
                StepEncoder::Classic { max_error_ticks } => {
                    let (moves, covered) =
                        compress_with_max_error(&clocks, base_clock, max_error_ticks).map_err(
                            |e| ShimError::CompressFailure {
                                motor,
                                detail: e.detail,
                            },
                        )?;
                    Encoded::Classic(moves, covered)
                }
                StepEncoder::HighPrecision => {
                    let scratch = self
                        .hp_scratch
                        .as_mut()
                        .expect("high-precision motors own their compressor scratch");
                    let (moves, covered, carry_out) = compress_hp::compress_hp(
                        scratch, &clocks, base_clock, hp_carry,
                    )
                    .map_err(|e| ShimError::CompressFailure {
                        motor,
                        detail: e.detail,
                    })?;
                    Encoded::Hp(moves, covered, carry_out)
                }
            };
            let covered = encoded.covered();
            if covered == 0 {
                break;
            }

            if re_anchoring {
                frames.push(StepFrame::ResetStepClock {
                    oid,
                    clock: base_clock as u32,
                });
                if self.needs_reset {
                    self.needs_reset = false;
                    self.last_dir = None;
                }
            }
            if self.last_dir != Some(dir) {
                frames.push(StepFrame::SetNextStepDir { oid, dir });
                self.last_dir = Some(dir);
            }
            let mut reconstructed = base_clock;
            match encoded {
                Encoded::Classic(moves, _) => {
                    for mv in &moves {
                        frames.push(StepFrame::QueueStep {
                            oid,
                            interval: mv.interval,
                            count: mv.count,
                            add: mv.add,
                        });
                        reconstructed = mv.last_clock(reconstructed);
                    }
                }
                Encoded::Hp(moves, _, carry_out) => {
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
                        reconstructed = reconstructed.wrapping_add(mv.last_step);
                    }
                    self.next_expected_interval = carry_out;
                }
            }

            self.last_step_clock = reconstructed;
            self.pending.drain(..covered);
            if covered < run_len {
                break;
            }
        }
        Ok(())
    }
}

#[derive(Debug)]
pub struct StepShim {
    motors: Vec<MotorState>,
    queue_depth: u32,
}

impl StepShim {
    pub fn new(motors: Vec<MotorConfig>, queue_depth: u32) -> Self {
        Self {
            motors: motors
                .into_iter()
                .map(|cfg| MotorState::new(cfg, queue_depth))
                .collect(),
            queue_depth,
        }
    }

    pub fn push_spans(
        &mut self,
        motor: usize,
        views: &[ClockedMotorSpan],
    ) -> Result<(), ShimError> {
        self.validate_spans(motor, views)?;
        let state = self.motor_mut(motor);
        for view in views {
            state.queue.push(motor, view.clone())?;
        }
        Ok(())
    }

    pub fn validate_fresh_spans(
        &self,
        motor: usize,
        views: &[ClockedMotorSpan],
    ) -> Result<(), ShimError> {
        self.motors[motor].queue.validate(motor, views, false)
    }

    pub fn validate_spans(
        &self,
        motor: usize,
        views: &[ClockedMotorSpan],
    ) -> Result<(), ShimError> {
        self.motors[motor].queue.validate(motor, views, true)
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

    /// The clock the last emitted step of this motor lands on. Every frame
    /// batch re-anchors from it; the sink mirrors it from the frame clocks.
    pub fn emitted_clock(&self, motor: usize) -> u64 {
        self.motors[motor].last_step_clock
    }

    /// The clock slope this motor's spans must carry. A view clocked on any
    /// other slope belongs to a different epoch and is refused.
    pub fn motor_cycles_per_second(&self, motor: usize) -> f64 {
        self.motors[motor].cfg.cycles_per_second
    }

    pub fn motor_microstep_distance(&self, motor: usize) -> f64 {
        self.motors[motor].cfg.microstep_distance
    }

    pub fn pending_roots(&self) -> usize {
        self.motors.iter().map(|m| m.pending.len()).sum()
    }

    pub fn queued_spans(&self) -> usize {
        self.motors.iter().map(|m| m.queue.len()).sum()
    }

    pub fn finish(&mut self, motor: usize) -> Result<Vec<StepFrame>, ShimError> {
        let state = self.motor_mut(motor);
        let mut frames = Vec::new();
        loop {
            let before = state.pending.len();
            if before == 0 {
                return Ok(frames);
            }
            state.emit(motor, &mut frames)?;
            if state.pending.len() == before {
                return Err(ShimError::CompressFailure {
                    motor,
                    detail: format!("{before} step roots cannot be compressed at stream end"),
                });
            }
        }
    }

    pub fn drain(&mut self, up_to_clock: u64) -> Result<Vec<StepFrame>, ShimError> {
        let mut frames = Vec::new();
        for motor in 0..self.motors.len() {
            let state = &mut self.motors[motor];
            state.cursor.advance(
                motor,
                &state.cfg,
                &mut state.queue,
                up_to_clock,
                &mut state.pending,
            )?;
            state.emit(motor, &mut frames)?;
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
        loop {
            let before = state.pending.len();
            if before == 0 {
                break;
            }
            state.emit(motor, &mut frames)?;
            if state.pending.len() == before {
                return Err(ShimError::CompressFailure {
                    motor,
                    detail: format!(
                        "{before} step roots at or before the cut clock {clock} cannot be \
                         compressed; cutting here would drop executed motion"
                    ),
                });
            }
        }
        let state = self.motor_mut(motor);
        state.pending.clear();
        state.queue.abandon_all();
        state.cursor.reset_to(executed, clock.saturating_add(1));
        state.last_step_clock = 0;
        state.needs_reset = true;
        state.last_dir = None;
        state.next_expected_interval = 0;
        Ok(frames)
    }

    pub fn set_motor_cycles_per_second(&mut self, motor: usize, freq: f64) {
        let state = self.motor_mut(motor);
        if state.cfg.cycles_per_second == freq {
            return;
        }
        let count = state.cursor.step_count();
        let floor = state.cursor.resume_floor();
        state.cfg.cycles_per_second = freq;
        state.cursor = StepRootCursor::new(&state.cfg);
        state.cursor.reset_to(count, floor);
    }

    pub fn reset_position(&mut self, motor: usize, count: i64) {
        let state = self.motor_mut(motor);
        state.pending.clear();
        state.cursor.reset_to(count, 0);
        state.last_step_clock = 0;
        state.needs_reset = true;
        state.last_dir = None;
        state.next_expected_interval = 0;
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
