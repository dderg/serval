use trajectory::{ClockedMotorSpan, Pva};

use crate::ring::SpanQueue;
use crate::{MotorConfig, ShimError};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StepRoot {
    pub clock: u64,
    pub dir: u8,
    pub advance: i8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Slope {
    Rising,
    Falling,
}

impl Slope {
    fn advance(self) -> i8 {
        match self {
            Self::Rising => 1,
            Self::Falling => -1,
        }
    }

    fn reached(self, position: f64, level: f64) -> bool {
        match self {
            Self::Rising => position >= level,
            Self::Falling => position <= level,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct Lattice {
    origin_mm: f64,
    step_count: i64,
}

impl Lattice {
    fn nominal_position(self, microstep_mm: f64) -> f64 {
        self.origin_mm + self.step_count as f64 * microstep_mm
    }

    fn threshold(self, microstep_mm: f64, slope: Slope) -> f64 {
        self.origin_mm + (self.step_count + i64::from(slope.advance())) as f64 * microstep_mm
    }
}

const BISECTION_SAFEGUARD_PERIOD: u32 = 3;

#[derive(Debug)]
pub struct StepRootCursor {
    microstep_mm: f64,
    lane: Lattice,
    overlay: Option<Lattice>,
    overlay_signal_id: Option<usize>,
    positioned: bool,
    resume_floor: Option<u64>,
    origin_clock: Option<u64>,
    frontier: Option<u64>,
    last_root_clock: Option<u64>,
}

impl StepRootCursor {
    pub fn new(cfg: &MotorConfig) -> Self {
        Self {
            microstep_mm: cfg.microstep_distance,
            lane: Lattice {
                origin_mm: 0.0,
                step_count: 0,
            },
            overlay: None,
            overlay_signal_id: None,
            positioned: false,
            resume_floor: None,
            origin_clock: None,
            frontier: None,
            last_root_clock: None,
        }
    }

    pub fn step_count(&self) -> i64 {
        self.lane.step_count
    }

    pub fn position(&self) -> f64 {
        self.lane.nominal_position(self.microstep_mm)
    }

    pub fn resume_floor(&self) -> u64 {
        self.resume_floor.unwrap_or(0)
    }

    pub fn origin_clock(&self) -> Option<u64> {
        self.origin_clock
    }

    pub fn reset_to(&mut self, count: i64, resume_floor: u64) {
        self.lane = Lattice {
            origin_mm: 0.0,
            step_count: count,
        };
        self.overlay = None;
        self.overlay_signal_id = None;
        self.positioned = true;
        self.resume_floor = Some(resume_floor);
        self.origin_clock = resume_floor.checked_sub(1);
        self.last_root_clock = None;
        self.frontier = None;
    }

    pub fn advance(
        &mut self,
        motor: usize,
        cfg: &MotorConfig,
        queue: &mut SpanQueue,
        up_to_clock: u64,
        out: &mut Vec<StepRoot>,
    ) -> Result<(), ShimError> {
        while let Some(view) = queue.active().cloned() {
            if view.clock_freq_hz != cfg.cycles_per_second {
                return Err(ShimError::SpanFrequencyMismatch {
                    motor,
                    expected: cfg.cycles_per_second,
                    got: view.clock_freq_hz,
                });
            }
            let signal_start = view.start_clock.max(self.resume_floor());
            let begin = signal_start.max(self.frontier.unwrap_or(0));
            if begin > view.end_clock {
                queue.release_active();
                continue;
            }
            let last_clock = view.end_clock.min(up_to_clock);
            if last_clock < begin {
                return Ok(());
            }
            self.enter(motor, &view, signal_start, begin)?;
            self.emit_roots(motor, cfg, &view, begin, last_clock, out)?;
            self.frontier = Some(last_clock + 1);
            if last_clock < view.end_clock {
                return Ok(());
            }
            queue.release_active();
        }
        Ok(())
    }

    fn enter(
        &mut self,
        motor: usize,
        view: &ClockedMotorSpan,
        signal_start: u64,
        begin: u64,
    ) -> Result<(), ShimError> {
        if view.signal.motor_mask == 0 {
            self.overlay = None;
            self.overlay_signal_id = None;
        } else {
            let signal_id = std::sync::Arc::as_ptr(&view.signal) as usize;
            if self.overlay_signal_id != Some(signal_id) {
                let position = self.eval(motor, view, signal_start)?.position;
                self.overlay = Some(Lattice {
                    origin_mm: position,
                    step_count: 0,
                });
                self.overlay_signal_id = Some(signal_id);
            }
        }
        if self.origin_clock.is_none() {
            if !self.positioned {
                let position = self.eval(motor, view, begin)?.position;
                self.lane.origin_mm = position - self.lane.step_count as f64 * self.microstep_mm;
                self.positioned = true;
            }
            self.origin_clock = Some(begin);
        }
        Ok(())
    }

    fn emit_roots(
        &mut self,
        motor: usize,
        cfg: &MotorConfig,
        view: &ClockedMotorSpan,
        begin: u64,
        last_clock: u64,
        out: &mut Vec<StepRoot>,
    ) -> Result<(), ShimError> {
        if begin == last_clock {
            return self.emit_interval(motor, cfg, view, begin, last_clock, out);
        }
        let mut boundaries = vec![view.start_clock, view.end_clock];
        let breakpoints = &view.signal.breakpoints;
        let first = breakpoints.partition_point(|&t| t <= view.stream_t_start);
        let last = breakpoints.partition_point(|&t| t < view.stream_t_end);
        for &t in &breakpoints[first..last] {
            boundaries.push(
                view.clock_at_stream_time(t)
                    .map_err(|error| ShimError::SpanEval { motor, error })?,
            );
        }
        boundaries.retain(|clock| *clock >= begin && *clock <= last_clock);
        boundaries.extend([begin, last_clock]);
        boundaries.sort_unstable();
        boundaries.dedup();
        for window in boundaries.windows(2) {
            self.emit_interval(motor, cfg, view, window[0], window[1], out)?;
        }
        Ok(())
    }

    fn emit_run(
        &mut self,
        motor: usize,
        cfg: &MotorConfig,
        view: &ClockedMotorSpan,
        lo: u64,
        hi: u64,
        slope: Slope,
        out: &mut Vec<StepRoot>,
    ) -> Result<(), ShimError> {
        let run_end = self.eval(motor, view, hi)?.position;
        let mut search_from = lo;
        loop {
            let level = self.frame().threshold(self.microstep_mm, slope);
            if !slope.reached(run_end, level) {
                return Ok(());
            }
            let clock = self.solve_crossing(motor, view, search_from, hi, level, slope)?;
            self.push_root(motor, cfg, clock, slope, out)?;
            search_from = clock;
        }
    }

    fn solve_crossing(
        &self,
        motor: usize,
        view: &ClockedMotorSpan,
        lo: u64,
        hi: u64,
        level: f64,
        slope: Slope,
    ) -> Result<u64, ShimError> {
        let mut low = lo;
        let mut low_pva = self.eval(motor, view, low)?;
        if slope.reached(low_pva.position, level) {
            return Ok(low);
        }
        let mut high = hi;
        let mut iteration = 0_u32;
        while high - low > 1 {
            let bisection = low + (high - low) / 2;
            let safeguarded =
                iteration % BISECTION_SAFEGUARD_PERIOD == BISECTION_SAFEGUARD_PERIOD - 1;
            let candidate = if safeguarded {
                bisection
            } else {
                newton_candidate(low, high, low_pva, level, view.clock_freq_hz).unwrap_or(bisection)
            };
            let pva = self.eval(motor, view, candidate)?;
            if slope.reached(pva.position, level) {
                high = candidate;
            } else {
                low = candidate;
                low_pva = pva;
            }
            iteration = iteration.wrapping_add(1);
        }
        Ok(high)
    }

    fn push_root(
        &mut self,
        motor: usize,
        cfg: &MotorConfig,
        clock: u64,
        slope: Slope,
        out: &mut Vec<StepRoot>,
    ) -> Result<(), ShimError> {
        let advance = slope.advance();
        if let Some(previous_clock) = self.last_root_clock.filter(|&last| clock <= last) {
            return Err(ShimError::StepClockRegression {
                motor,
                previous_clock,
                clock,
                step_count: self.frame().step_count,
                advance,
            });
        }
        self.last_root_clock = Some(clock);
        let forward = advance > 0;
        let dir = u8::from(forward != cfg.invert_dir);
        self.frame_mut().step_count += i64::from(advance);
        out.push(StepRoot {
            clock,
            dir,
            advance,
        });
        Ok(())
    }

    fn emit_interval(
        &mut self,
        motor: usize,
        cfg: &MotorConfig,
        view: &ClockedMotorSpan,
        from: u64,
        to: u64,
        out: &mut Vec<StepRoot>,
    ) -> Result<(), ShimError> {
        if let Some(slope) = self.certified_slope(motor, view, from, to)? {
            return self.emit_run(motor, cfg, view, from, to, slope, out);
        }
        if !self.interval_can_reach_next_lattice(motor, view, from, to)? {
            return Ok(());
        }
        if to - from <= 1 {
            let rise =
                self.eval(motor, view, to)?.position - self.eval(motor, view, from)?.position;
            let slope = if rise >= 0.0 {
                Slope::Rising
            } else {
                Slope::Falling
            };
            return self.emit_run(motor, cfg, view, from, to, slope, out);
        }
        let mid = from + (to - from) / 2;
        self.emit_interval(motor, cfg, view, from, mid, out)?;
        self.emit_interval(motor, cfg, view, mid, to, out)
    }

    fn interval_can_reach_next_lattice(
        &self,
        motor: usize,
        view: &ClockedMotorSpan,
        from: u64,
        to: u64,
    ) -> Result<bool, ShimError> {
        let t_from = self.stream_time(motor, view, from)?;
        let t_to = self.stream_time(motor, view, to)?;
        let bounds = view
            .signal
            .pva_bounds(t_from, t_to)
            .map_err(|error| ShimError::SpanEval { motor, error })?;
        let from_position = self.eval(motor, view, from)?.position;
        let to_position = self.eval(motor, view, to)?.position;
        let duration = (to - from) as f64 / view.clock_freq_hz;
        let scale = from_position
            .abs()
            .max(to_position.abs())
            .max(self.microstep_mm)
            .max(1.0);
        let slack = 256.0 * f64::EPSILON * scale;
        let lower = from_position
            .min(to_position)
            .min(from_position + bounds.velocity_min.min(0.0) * duration)
            - slack;
        let upper = from_position
            .max(to_position)
            .max(from_position + bounds.velocity_max.max(0.0) * duration)
            + slack;
        let nominal = self.frame().nominal_position(self.microstep_mm);
        Ok(lower <= nominal - self.microstep_mm || upper >= nominal + self.microstep_mm)
    }

    fn certified_slope(
        &self,
        motor: usize,
        view: &ClockedMotorSpan,
        from: u64,
        to: u64,
    ) -> Result<Option<Slope>, ShimError> {
        let t_from = self.stream_time(motor, view, from)?;
        let t_to = self.stream_time(motor, view, to)?;
        let bounds = view
            .signal
            .pva_bounds(t_from, t_to)
            .map_err(|error| ShimError::SpanEval { motor, error })?;
        let from_pva = self.eval(motor, view, from)?;
        let to_pva = self.eval(motor, view, to)?;
        let duration = (to - from) as f64 / view.clock_freq_hz;
        let velocity_scale = bounds
            .velocity_min
            .abs()
            .max(bounds.velocity_max.abs())
            .max(from_pva.velocity.abs())
            .max(to_pva.velocity.abs())
            .max(bounds.acceleration_abs_max * duration)
            .max(1.0);
        let tolerance = 256.0 * f64::EPSILON * velocity_scale;
        if bounds.velocity_min >= -tolerance && to_pva.position >= from_pva.position {
            return Ok(Some(Slope::Rising));
        }
        if bounds.velocity_max <= tolerance && to_pva.position <= from_pva.position {
            return Ok(Some(Slope::Falling));
        }
        let mean_velocity = 0.5 * (from_pva.velocity + to_pva.velocity);
        let dip = 0.5 * bounds.acceleration_abs_max * duration;
        if mean_velocity - dip >= -tolerance {
            return Ok(Some(Slope::Rising));
        }
        if mean_velocity + dip <= tolerance {
            return Ok(Some(Slope::Falling));
        }
        Ok(None)
    }

    fn frame(&self) -> Lattice {
        self.overlay.unwrap_or(self.lane)
    }

    fn frame_mut(&mut self) -> &mut Lattice {
        match &mut self.overlay {
            Some(overlay) => overlay,
            None => &mut self.lane,
        }
    }

    fn eval(&self, motor: usize, view: &ClockedMotorSpan, clock: u64) -> Result<Pva, ShimError> {
        view.eval_at_clock(clock)
            .map_err(|error| ShimError::SpanEval { motor, error })
    }

    fn stream_time(
        &self,
        motor: usize,
        view: &ClockedMotorSpan,
        clock: u64,
    ) -> Result<f64, ShimError> {
        view.stream_time_at_clock(clock)
            .map_err(|error| ShimError::SpanEval { motor, error })
    }
}

fn newton_candidate(
    low: u64,
    high: u64,
    low_pva: Pva,
    level: f64,
    clock_freq_hz: f64,
) -> Option<u64> {
    let clocks = (level - low_pva.position) / low_pva.velocity * clock_freq_hz;
    if !clocks.is_finite() || clocks <= 0.0 {
        return None;
    }
    let span = high - low;
    if clocks >= span as f64 {
        return None;
    }
    let offset = (clocks.ceil() as u64).clamp(1, span - 1);
    Some(low + offset)
}
