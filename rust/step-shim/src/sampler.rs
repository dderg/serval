use runtime::dispatch_stepper::DISPLACEMENT_THRESHOLD_MM;
use runtime::motion_core::{ArmedPiece, arm_piece};
use runtime::sub_sample_timing::{
    QuadStepTimeInputs, StepTimeInputs, StepTimingResult, compute_step_times,
    compute_step_times_quadratic, quantize_step_delta,
};

use crate::ring::PieceRing;
use crate::{MotorConfig, ShimError, StepEncoder};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PendingStep {
    pub clock: u64,
    pub dir: u8,
    pub advance: i8,
}

/// A per-motor overlay run: pieces carrying a motor_mask (nudges /
/// FORCE_MOVE) are relativized to start at zero by the enqueue stage, so
/// they step against their own frame and leave the lane's absolute frame
/// where the last kinematic piece left it — the same split the on-MCU
/// dispatcher keeps between `last_step_count` and `overlay_step_frame`.
#[derive(Debug, Clone, Copy)]
struct OverlayFrame {
    p_prev: f32,
    step_count: i64,
    step_phase: f32,
}

#[derive(Debug)]
pub struct MotorSampler {
    armed: Option<ArmedPiece>,
    overlay: Option<OverlayFrame>,
    p_prev: f32,
    v_prev: f32,
    step_count: i64,
    step_phase: f32,
    prev_sample: u64,
    origin_clock: Option<u64>,
    positioned: bool,
    resume_floor: Option<u64>,
    last_step_clock: Option<u64>,
    sample_period_cycles: u32,
    sample_period_sec: f32,
    cycles_per_second: f32,
}

impl MotorSampler {
    pub fn new(cfg: &MotorConfig) -> Self {
        let sample_period_cycles = sample_period_cycles(cfg);
        let cycles_per_second = cfg.cycles_per_second as f32;
        Self {
            armed: None,
            overlay: None,
            p_prev: 0.0,
            v_prev: 0.0,
            step_count: 0,
            step_phase: 0.0,
            prev_sample: 0,
            origin_clock: None,
            positioned: false,
            resume_floor: None,
            last_step_clock: None,
            sample_period_cycles,
            sample_period_sec: sample_period_cycles as f32 / cycles_per_second,
            cycles_per_second,
        }
    }

    pub fn step_count(&self) -> i64 {
        self.step_count
    }

    pub fn resume_floor(&self) -> u64 {
        self.resume_floor.unwrap_or(0)
    }

    pub fn origin_clock(&self) -> Option<u64> {
        self.origin_clock
    }

    pub fn reset_to(&mut self, count: i64, cfg: &MotorConfig, resume_floor: u64) {
        self.armed = None;
        self.step_count = count;
        self.p_prev = count as f32 * cfg.microstep_distance;
        #[allow(clippy::cast_precision_loss)]
        let physical_position = count as f64 * f64::from(cfg.microstep_distance);
        #[allow(clippy::cast_possible_truncation)]
        {
            self.step_phase = (f64::from(self.p_prev) - physical_position) as f32;
        }
        self.prev_sample = 0;
        self.origin_clock = None;
        self.positioned = true;
        self.resume_floor = Some(resume_floor);
        self.last_step_clock = None;
    }

    pub fn sample(
        &mut self,
        motor: usize,
        cfg: &MotorConfig,
        ring: &mut PieceRing,
        up_to_clock: u64,
        out: &mut Vec<PendingStep>,
    ) -> Result<(), ShimError> {
        while let Some(piece) = ring.front().copied() {
            let piece_end = match ring.next_start() {
                Some(next) => piece.end_time(self.cycles_per_second).min(next),
                None => piece.end_time(self.cycles_per_second),
            };
            if self
                .armed
                .is_none_or(|a| a.piece_start_cycles != piece.start_time)
            {
                let armed = arm_piece(&piece, self.cycles_per_second);
                self.overlay = (piece.motor_mask != 0).then_some(OverlayFrame {
                    p_prev: 0.0,
                    step_count: 0,
                    step_phase: 0.0,
                });
                if self.origin_clock.is_none() {
                    let begin = piece.start_time.max(self.resume_floor.unwrap_or(0));
                    self.prev_sample = begin;
                    let (position, velocity) = armed.eval_pos_vel(begin);
                    if !self.positioned {
                        self.p_prev = position;
                        self.positioned = true;
                    }
                    self.v_prev = velocity;
                    self.origin_clock = Some(begin);
                }
                self.armed = Some(armed);
            }
            let armed = self.armed.expect("piece armed above");
            loop {
                let next_sample = self.prev_sample + u64::from(self.sample_period_cycles);
                if next_sample > up_to_clock {
                    return Ok(());
                }
                if next_sample > piece_end {
                    break;
                }
                self.emit_sample(motor, cfg, &armed, next_sample, out)?;
                self.prev_sample = next_sample;
            }
            ring.retire_front();
            self.armed = None;
        }
        Ok(())
    }

    fn emit_sample(
        &mut self,
        motor: usize,
        cfg: &MotorConfig,
        armed: &ArmedPiece,
        now: u64,
        out: &mut Vec<PendingStep>,
    ) -> Result<(), ShimError> {
        let (p_end, v_end) = armed.eval_pos_vel(now);
        let (prev, p_start, step_phase_start) = match self.overlay {
            Some(frame) => (frame.step_count, frame.p_prev, frame.step_phase),
            None => (self.step_count, self.p_prev, self.step_phase),
        };
        let step_phase_end = step_phase_start + (p_end - p_start);
        let step_delta = quantize_step_delta(step_phase_end, cfg.microstep_distance);
        #[allow(clippy::cast_precision_loss)]
        let next_step_phase = step_phase_end - step_delta as f32 * cfg.microstep_distance;
        let target = prev + i64::from(step_delta);
        let signed_steps = i64::from(step_delta);
        if signed_steps == 0 {
            self.commit_frame(p_end, v_end, prev, next_step_phase);
            return Ok(());
        }
        let abs_steps = u32::try_from(signed_steps.unsigned_abs()).unwrap_or(u32::MAX);
        if abs_steps > cfg.max_steps_per_sample {
            return Err(ShimError::StepRateExceeded {
                motor,
                steps: abs_steps,
                cap: cfg.max_steps_per_sample,
            });
        }

        let inputs = StepTimeInputs {
            p_start: step_phase_start,
            p_end: step_phase_end,
            step_delta,
            microstep_distance: cfg.microstep_distance,
            sample_period_sec: self.sample_period_sec,
            sample_start_cycles: self.prev_sample as u32,
            cycles_per_second: self.cycles_per_second,
            displacement_threshold: DISPLACEMENT_THRESHOLD_MM,
        };
        let times = match cfg.encoder {
            StepEncoder::Classic { .. } => compute_step_times(&inputs),
            StepEncoder::HighPrecision => compute_step_times_quadratic(&QuadStepTimeInputs {
                p_start: step_phase_start,
                p_end: step_phase_end,
                v_start: self.v_prev,
                v_end,
                step_delta,
                microstep_distance: cfg.microstep_distance,
                sample_period_sec: self.sample_period_sec,
                sample_start_cycles: self.prev_sample as u32,
                cycles_per_second: self.cycles_per_second,
                displacement_threshold: DISPLACEMENT_THRESHOLD_MM,
            }),
        };
        let times = match times {
            StepTimingResult::SecantSlope(t) | StepTimingResult::Uniform(t) => t,
            StepTimingResult::NoSteps => {
                self.commit_frame(p_end, v_end, prev, next_step_phase);
                return Ok(());
            }
        };

        let forward = signed_steps > 0;
        let dir = u8::from(forward != cfg.invert_dir);
        let advance: i8 = if forward { 1 } else { -1 };
        let window_start_lo = self.prev_sample as u32;
        for cycle_lo in times.iter().copied() {
            let clock = self.prev_sample + u64::from(cycle_lo.wrapping_sub(window_start_lo));
            if let Some(previous_clock) = self.last_step_clock.filter(|&last| clock <= last) {
                return Err(ShimError::StepClockRegression {
                    motor,
                    previous_clock,
                    clock,
                    sample_clock: now,
                    piece_start_clock: armed.piece_start_cycles,
                    piece_end_clock: armed.piece_end_cycles,
                    previous_step_count: prev,
                    target_step_count: target,
                    p_start,
                    p_end,
                    previous_advance: out.last().map(|step| step.advance),
                    advance,
                });
            }
            self.last_step_clock = Some(clock);
            out.push(PendingStep {
                clock,
                dir,
                advance,
            });
        }
        self.commit_frame(p_end, v_end, target, next_step_phase);
        Ok(())
    }

    fn commit_frame(&mut self, p_end: f32, v_end: f32, step_count: i64, step_phase: f32) {
        match &mut self.overlay {
            Some(frame) => {
                frame.p_prev = p_end;
                frame.step_count = step_count;
                frame.step_phase = step_phase;
            }
            None => {
                self.p_prev = p_end;
                self.step_count = step_count;
                self.step_phase = step_phase;
            }
        }
        self.v_prev = v_end;
    }
}

fn sample_period_cycles(cfg: &MotorConfig) -> u32 {
    let cycles = cfg.cycles_per_second / f64::from(cfg.sample_rate_hz);
    assert!(
        cycles >= 1.0 && cycles <= f64::from(u32::MAX),
        "sample_rate_hz {} is not representable against cycles_per_second {}",
        cfg.sample_rate_hz,
        cfg.cycles_per_second
    );
    cycles.round() as u32
}

#[cfg(test)]
#[path = "sampler_tests.rs"]
mod tests;
