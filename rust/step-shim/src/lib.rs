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
}

const MAX_SEAM_SKEW_CYCLES: u64 = 16;

/// Where the previous piece ended and how long it was, so the seam tolerance
/// scales with the piece that produced the seam rather than the one arriving.
#[derive(Debug, Clone, Copy)]
struct Seam {
    expected_start: u64,
}

impl Seam {
    fn skew_tolerance(self) -> u64 {
        MAX_SEAM_SKEW_CYCLES
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
    PieceGap {
        motor: usize,
        expected: u64,
        got: u64,
        tolerance: u64,
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
            Self::PieceGap {
                motor,
                expected,
                got,
                tolerance,
            } => write!(
                f,
                "motor {motor}: piece starts at {got}, expected {expected} \
                 (+/-{tolerance} clock-domain skew)"
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

            let reset_clock = if self.needs_reset {
                Some(
                    self.sampler
                        .origin_clock()
                        .expect("origin clock is set before any step is sampled"),
                )
            } else {
                None
            };
            let base_clock = reset_clock.unwrap_or(self.last_step_clock);
            if clocks[0] <= base_clock {
                return Err(ShimError::CompressFailure {
                    motor,
                    detail: format!(
                        "step clock regression: first step of this run is at {} but the \
                         stream is already committed to {base_clock} (reset={:?}, \
                         last_step_clock={}, run_len={run_len}, dir={dir})",
                        clocks[0], reset_clock, self.last_step_clock
                    ),
                });
            }

            let (moves, covered) =
                compress(&clocks, base_clock).map_err(|e| ShimError::CompressFailure {
                    motor,
                    detail: e.detail,
                })?;
            if covered == 0 {
                break;
            }

            if let Some(clock) = reset_clock {
                frames.push(StepFrame::ResetStepClock {
                    oid,
                    clock: clock as u32,
                });
                self.needs_reset = false;
                self.last_dir = None;
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
            if let Some(seam) = state.next_seam {
                let tolerance = seam.skew_tolerance();
                if piece.start_time.abs_diff(seam.expected_start) > tolerance {
                    return Err(ShimError::PieceGap {
                        motor,
                        expected: seam.expected_start,
                        got: piece.start_time,
                        tolerance,
                    });
                }
            }
            let end = piece.end_time(cycles_per_second);
            state.ring.push(motor, *piece)?;
            state.next_seam = Some(Seam {
                expected_start: end,
            });
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
                    });
                }
            }
            let end = piece.end_time(cycles_per_second);
            seam = Some(Seam {
                expected_start: end,
            });
        }
        Ok(())
    }

    /// Pieces pushed but not yet sampled to completion. Zero means the shim
    /// has nothing left to turn into step frames as the clock advances.
    pub fn commanded_steps(&self, motor: usize) -> i64 {
        self.motors[motor].sampler.step_count()
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
