pub mod compress;
pub mod ring;
pub mod sampler;

use runtime::piece_ring::PieceEntry;

use compress::compress;
use ring::PieceRing;
use sampler::{MotorSampler, PendingStep};

#[derive(Debug, Clone, Copy)]
pub struct MotorConfig {
    pub oid: u32,
    pub microstep_distance: f32,
    pub invert_dir: bool,
    pub max_steps_per_sample: u32,
    pub sample_rate_hz: f32,
    pub cycles_per_second: f64,
    /// How far the mcu's classic stepper needs between the last step of one
    /// queued move and the first step of the next. `stepper_event_full`
    /// schedules an unstep `step_pulse_ticks` after every step and
    /// `stepper_load_next` re-arms from that pending unstep, so a run that
    /// starts sooner is loaded behind it: `motion.step_load_late`, then
    /// "Stepper too far in past". Zero means the caller only owes strict
    /// monotonicity (both-edge drivers configure zero pulse ticks).
    pub min_rearm_cycles: u64,
}

/// What the producer's own anchoring is allowed to move a piece start by:
/// the integer rounding on each side of a seam plus the slop the segment
/// anchor may introduce when it re-times a stream origin. It does **not**
/// cover the f32 round trip through `duration` — that scales with the piece
/// and is added per seam by [`projection_slack_cycles`].
pub const MAX_SEAM_SKEW_CYCLES: u64 = 16;

/// How far [`PieceEntry::end_time`] — the arithmetic every consumer of a
/// piece runs, host sampler and mcu walker alike — can land from the clock
/// the producer projected the *next* piece's start onto.
///
/// The consumer computes `start + (fl32(duration) * fl32(freq)) as u64`
/// while the producer rounds an f64 projection of the same instant, so over
/// a piece spanning `span_cycles` the two are separated by
///
/// - `fl32(duration)`         — up to one f32 half-ulp of the span,
/// - `fl32(freq)`             — another,
/// - `fl32(duration * freq)`  — another,
/// - the truncation to `u64`  — below one cycle,
/// - the producer's `round()` — half a cycle.
///
/// A flat tolerance cannot express this: a merged hold spanning 2^27 cycles
/// (1.9 s at 72 MHz) already has an 8-cycle half-ulp, and merged holds run
/// to `MAX_MERGED_HOLD_SECS`. Anything wider than this bound is a broken
/// stream — a dropped piece, a stale epoch, a clock slope the producer and
/// the shim do not share — not rounding.
#[must_use]
pub fn projection_slack_cycles(span_cycles: u64) -> u64 {
    const F32_HALF_ULPS: u64 = 3;
    const INTEGER_ROUNDINGS: u64 = 2;
    let half_ulp_scale = 1_u64 << f32::MANTISSA_DIGITS;
    (F32_HALF_ULPS * span_cycles).div_ceil(half_ulp_scale) + INTEGER_ROUNDINGS
}

/// Where the previous piece was projected to end and how wide that
/// projection was, so the seam tolerance scales with the piece that produced
/// the seam rather than the one arriving.
#[derive(Debug, Clone, Copy)]
struct Seam {
    expected_start: u64,
    projected_span: u64,
}

impl Seam {
    fn after(piece: &PieceEntry, cycles_per_second: f32) -> Self {
        let expected_start = piece.end_time(cycles_per_second);
        Self {
            expected_start,
            projected_span: expected_start - piece.start_time,
        }
    }

    fn skew_tolerance(self) -> u64 {
        MAX_SEAM_SKEW_CYCLES + projection_slack_cycles(self.projected_span)
    }
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
}

#[derive(Debug)]
pub enum ShimError {
    RingFull {
        motor: usize,
    },
    StepRateExceeded {
        motor: usize,
        steps: u32,
        cap: u32,
    },
    StepClockRegression {
        motor: usize,
        previous_clock: u64,
        clock: u64,
        sample_clock: u64,
        piece_start_clock: u64,
        piece_end_clock: u64,
        previous_step_count: i64,
        target_step_count: i64,
        p_start: f32,
        p_end: f32,
        previous_advance: Option<i8>,
        advance: i8,
    },
    PieceGap {
        motor: usize,
        expected: u64,
        got: u64,
        tolerance: u64,
        projected_span: u64,
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
            Self::RingFull { motor } => write!(f, "motor {motor}: virtual piece ring full"),
            Self::StepRateExceeded { motor, steps, cap } => write!(
                f,
                "motor {motor}: {steps} steps in one sample exceeds cap {cap}"
            ),
            Self::StepClockRegression {
                motor,
                previous_clock,
                clock,
                sample_clock,
                piece_start_clock,
                piece_end_clock,
                previous_step_count,
                target_step_count,
                p_start,
                p_end,
                previous_advance,
                advance,
            } => write!(
                f,
                "motor {motor}: step clock {clock} did not advance past {previous_clock} \
                 at sample {sample_clock} in piece {piece_start_clock}..{piece_end_clock}; \
                 position {p_start} -> {p_end}, count {previous_step_count} -> \
                 {target_step_count}, advance {previous_advance:?} -> {advance}"
            ),
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
            Self::PieceGap {
                motor,
                expected,
                got,
                tolerance,
                projected_span,
            } => write!(
                f,
                "motor {motor}: piece starts at {got}, expected {expected} \
                 (+/-{tolerance} clock-domain skew, reprojected from a \
                 {projected_span}-cycle piece)"
            ),
            Self::CompressFailure { motor, detail } => {
                write!(f, "motor {motor}: stepcompress failed: {detail}")
            }
        }
    }
}

impl std::error::Error for ShimError {}

#[derive(Debug)]
struct MotorState {
    cfg: MotorConfig,
    ring: PieceRing,
    sampler: MotorSampler,
    pending: Vec<PendingStep>,
    last_step_clock: u64,
    needs_reset: bool,
    last_dir: Option<u8>,
    next_seam: Option<Seam>,
}

impl MotorState {
    fn new(cfg: MotorConfig, ring_depth: u32) -> Self {
        Self {
            sampler: MotorSampler::new(&cfg),
            cfg,
            ring: PieceRing::new(ring_depth),
            pending: Vec::new(),
            last_step_clock: 0,
            needs_reset: true,
            last_dir: None,
            next_seam: None,
        }
    }

    fn emit(&mut self, motor: usize, frames: &mut Vec<StepFrame>) -> Result<(), ShimError> {
        let oid = self.cfg.oid;
        while let Some(first) = self.pending.first().copied() {
            let dir = first.dir;
            let run_len = self.pending.iter().take_while(|s| s.dir == dir).count();
            let clocks: Vec<u64> = self.pending[..run_len].iter().map(|s| s.clock).collect();

            let committed = if self.needs_reset {
                self.sampler
                    .origin_clock()
                    .expect("origin clock is set before any step is sampled")
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
            let base_clock = if out_of_reach {
                clocks[0] - 1
            } else {
                committed
            };

            let (moves, covered) =
                compress(&clocks, base_clock).map_err(|e| ShimError::CompressFailure {
                    motor,
                    detail: e.detail,
                })?;
            if covered == 0 {
                break;
            }

            if self.needs_reset || out_of_reach {
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
            for mv in &moves {
                frames.push(StepFrame::QueueStep {
                    oid,
                    interval: mv.interval,
                    count: mv.count,
                    add: mv.add,
                });
                reconstructed = mv.last_clock(reconstructed);
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
    ring_depth: u32,
}

impl StepShim {
    pub fn new(motors: Vec<MotorConfig>, ring_depth: u32) -> Self {
        Self {
            motors: motors
                .into_iter()
                .map(|cfg| MotorState::new(cfg, ring_depth))
                .collect(),
            ring_depth,
        }
    }

    pub fn push_pieces(&mut self, motor: usize, pieces: &[PieceEntry]) -> Result<(), ShimError> {
        self.validate_pieces_public(motor, pieces)?;
        let state = self.motor_mut(motor);
        let cycles_per_second = state.cfg.cycles_per_second as f32;
        for piece in pieces {
            state.ring.push(motor, *piece)?;
            state.next_seam = Some(Seam::after(piece, cycles_per_second));
        }
        Ok(())
    }

    pub fn validate_fresh_pieces(
        &mut self,
        motor: usize,
        pieces: &[PieceEntry],
    ) -> Result<(), ShimError> {
        self.validate_from(motor, pieces, None)
    }

    /// Sanction a forward-only seam jump: the stream time crossed a
    /// drained-to-rest hole (a dwell) with no pieces, so the next piece for
    /// this motor starts later than the projected end of the previous one.
    /// No steps, no clock reset — only the seam expectation moves. A jump
    /// BACKWARD past the tolerance is still an overlap and stays loud.
    pub fn accept_forward_seam_gap(
        &mut self,
        motor: usize,
        at_start_clock: u64,
    ) -> Result<(), ShimError> {
        let state = self.motor_mut(motor);
        if let Some(s) = state.next_seam {
            if at_start_clock.saturating_add(s.skew_tolerance()) < s.expected_start {
                return Err(ShimError::PieceGap {
                    motor,
                    expected: s.expected_start,
                    got: at_start_clock,
                    tolerance: s.skew_tolerance(),
                    projected_span: s.projected_span,
                });
            }
        }
        state.next_seam = None;
        Ok(())
    }

    pub fn validate_pieces_public(
        &mut self,
        motor: usize,
        pieces: &[PieceEntry],
    ) -> Result<(), ShimError> {
        let seam = self.motor_mut(motor).next_seam;
        self.validate_from(motor, pieces, seam)
    }

    fn validate_from(
        &mut self,
        motor: usize,
        pieces: &[PieceEntry],
        mut seam: Option<Seam>,
    ) -> Result<(), ShimError> {
        let state = self.motor_mut(motor);
        let cycles_per_second = state.cfg.cycles_per_second as f32;
        let occupied = if seam.is_some() { state.ring.len() } else { 0 };
        if occupied + pieces.len() > state.ring.capacity() as usize {
            return Err(ShimError::RingFull { motor });
        }
        for piece in pieces {
            if let Some(s) = seam {
                let tolerance = s.skew_tolerance();
                if piece.start_time.abs_diff(s.expected_start) > tolerance {
                    return Err(ShimError::PieceGap {
                        motor,
                        expected: s.expected_start,
                        got: piece.start_time,
                        tolerance,
                        projected_span: s.projected_span,
                    });
                }
            }
            seam = Some(Seam::after(piece, cycles_per_second));
        }
        Ok(())
    }

    /// Pieces pushed but not yet sampled to completion. Zero means the shim
    /// has nothing left to turn into step frames as the clock advances.
    pub fn commanded_steps(&self, motor: usize) -> i64 {
        self.motors[motor].sampler.step_count()
    }

    pub fn invert_dir(&self, motor: usize) -> bool {
        self.motors[motor].cfg.invert_dir
    }

    /// The clock slope this motor's seam projection is frozen on. Anything
    /// upstream that rewrites a piece's `duration` from a tick span must use
    /// this exact value, or the rewritten piece projects to a different end
    /// clock than the one the next piece was planned to abut.
    pub fn motor_cycles_per_second(&self, motor: usize) -> f64 {
        self.motors[motor].cfg.cycles_per_second
    }

    pub fn pending_steps(&self) -> usize {
        self.motors.iter().map(|m| m.pending.len()).sum()
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
                    detail: format!("{before} sampled steps cannot be compressed at stream end"),
                });
            }
        }
    }

    pub fn queued_pieces(&self) -> usize {
        self.motors.iter().map(|m| m.ring.len()).sum()
    }

    pub fn drain(&mut self, up_to_clock: u64) -> Result<Vec<StepFrame>, ShimError> {
        let mut frames = Vec::new();
        for motor in 0..self.motors.len() {
            let state = &mut self.motors[motor];
            state.sampler.sample(
                motor,
                &state.cfg,
                &mut state.ring,
                up_to_clock,
                &mut state.pending,
            )?;
            state.emit(motor, &mut frames)?;
        }
        Ok(frames)
    }

    pub fn retired_counts(&self) -> Vec<u32> {
        self.motors.iter().map(|m| m.ring.retired()).collect()
    }

    pub fn ring_depth(&self) -> u32 {
        self.ring_depth
    }

    pub fn halt_at(
        &mut self,
        motor: usize,
        clock: u64,
    ) -> Result<(i64, Vec<StepFrame>), ShimError> {
        let state = self.motor_mut(motor);
        let unexecuted: i64 = state
            .pending
            .iter()
            .filter(|s| s.clock > clock)
            .map(|s| i64::from(s.advance))
            .sum();
        let executed = state.sampler.step_count() - unexecuted;
        state.pending.retain(|s| s.clock <= clock);
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
                        "{before} sampled steps at or before the cut clock {clock} cannot be \
                         compressed; cutting here would drop executed motion"
                    ),
                });
            }
        }
        let state = self.motor_mut(motor);
        state.pending.clear();
        state.ring.retire_all();
        let cfg = state.cfg;
        state.sampler.reset_to(executed, &cfg, clock);
        state.next_seam = None;
        state.last_step_clock = 0;
        state.needs_reset = true;
        state.last_dir = None;
        Ok((executed, frames))
    }

    pub fn set_motor_cycles_per_second(&mut self, motor: usize, freq: f64) {
        let state = self.motor_mut(motor);
        if state.cfg.cycles_per_second == freq {
            return;
        }
        let count = state.sampler.step_count();
        let floor = state.sampler.resume_floor();
        state.cfg.cycles_per_second = freq;
        state.sampler = crate::sampler::MotorSampler::new(&state.cfg);
        let cfg = state.cfg;
        state.sampler.reset_to(count, &cfg, floor);
    }

    pub fn reset_position(&mut self, motor: usize, count: i64) {
        let state = self.motor_mut(motor);
        state.pending.clear();
        let cfg = state.cfg;
        state.sampler.reset_to(count, &cfg, 0);
        state.last_step_clock = 0;
        state.needs_reset = true;
        state.last_dir = None;
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
