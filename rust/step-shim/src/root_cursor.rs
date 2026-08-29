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

    fn deficit(self, position: f64, level: f64) -> f64 {
        match self {
            Self::Rising => level - position,
            Self::Falling => position - level,
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

thread_local! {
    pub static EVAL_COUNT: std::cell::Cell<u64> = const { std::cell::Cell::new(0) };
    pub static BOUNDS_COUNT: std::cell::Cell<u64> = const { std::cell::Cell::new(0) };
    pub static WINDOW_COUNT: std::cell::Cell<u64> = const { std::cell::Cell::new(0) };
    pub static CERT_NONE_COUNT: std::cell::Cell<u64> = const { std::cell::Cell::new(0) };
    pub static PRUNE_COUNT: std::cell::Cell<u64> = const { std::cell::Cell::new(0) };
}

#[derive(Debug)]
pub struct StepRootCursor {
    drain_deadline: Option<std::time::Instant>,
    drain_halted: bool,
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
            drain_deadline: None,
            drain_halted: false,
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

    pub fn drain_base(&self) -> Option<u64> {
        self.frontier
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
        deadline: Option<std::time::Instant>,
    ) -> Result<(), ShimError> {
        self.drain_deadline = deadline;
        self.drain_halted = false;
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
            if self.drain_halted {
                self.drain_deadline = None;
                return Ok(());
            }
            self.frontier = Some(last_clock + 1);
            if last_clock < view.end_clock {
                return Ok(());
            }
            queue.release_active();
        }
        self.drain_deadline = None;
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
                let position = self.position_at(motor, view, signal_start)?;
                self.overlay = Some(Lattice {
                    origin_mm: position,
                    step_count: 0,
                });
                self.overlay_signal_id = Some(signal_id);
            }
        }
        if self.origin_clock.is_none() {
            if !self.positioned {
                let position = self.position_at(motor, view, begin)?;
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
            self.emit_interval(motor, cfg, view, begin, last_clock, out)?;
            self.frontier = Some(last_clock + 1);
            return Ok(());
        }
        let mut boundaries = vec![view.start_clock, view.end_clock];
        let breakpoints = &view.signal.breakpoints;
        let stream_t_at = |clock: u64| {
            (view.stream_t_start + (clock as f64 - view.start_clock_exact) / view.clock_freq_hz)
                .clamp(view.stream_t_start, view.stream_t_end)
        };
        let t_begin = stream_t_at(begin);
        let t_last = stream_t_at(last_clock);
        let view_first = breakpoints.partition_point(|&t| t <= view.stream_t_start);
        let view_last = breakpoints.partition_point(|&t| t < view.stream_t_end);
        let first = breakpoints
            .partition_point(|&t| t < t_begin)
            .saturating_sub(1)
            .max(view_first);
        let last = (breakpoints.partition_point(|&t| t <= t_last) + 1).min(view_last);
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
        WINDOW_COUNT.with(|c| c.set(c.get() + boundaries.len() as u64 - 1));
        let mut index = 0;
        while index + 1 < boundaries.len() {
            let from = boundaries[index];
            let mut end_index = index + 1;
            match self.certified_slope(motor, view, from, boundaries[end_index])? {
                Some(mut slope) => {
                    let mut stride = 1;
                    while end_index < boundaries.len() - 1 {
                        let probe = (end_index + stride).min(boundaries.len() - 1);
                        match self.certified_slope(motor, view, from, boundaries[probe])? {
                            Some(merged) => {
                                slope = merged;
                                end_index = probe;
                                stride *= 2;
                            }
                            None => break,
                        }
                    }
                    self.emit_run(motor, cfg, view, from, boundaries[end_index], slope, out)?;
                }
                None => {
                    self.emit_interval(motor, cfg, view, from, boundaries[end_index], out)?;
                }
            }
            if self.drain_halted {
                return Ok(());
            }
            self.frontier = Some(boundaries[end_index] + 1);
            index = end_index;
        }
        self.frontier = Some(last_clock + 1);
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
        let run_end = self.position_at(motor, view, hi)?;
        let mut search_from = lo;
        let mut search_position = self.position_at(motor, view, lo)?;
        let mut roots_since_check = 0_u32;
        loop {
            let level = self.frame().threshold(self.microstep_mm, slope);
            if !slope.reached(run_end, level) {
                return Ok(());
            }
            let (clock, position) = self.solve_crossing(
                motor,
                view,
                (search_from, search_position),
                (hi, run_end),
                level,
                slope,
            )?;
            self.push_root(motor, cfg, clock, slope, out)?;
            search_from = clock;
            search_position = position;
            roots_since_check += 1;
            if roots_since_check == 32 {
                roots_since_check = 0;
                if self
                    .drain_deadline
                    .is_some_and(|d| std::time::Instant::now() >= d)
                    && clock < hi
                {
                    self.drain_halted = true;
                    self.frontier = Some(clock + 1);
                    return Ok(());
                }
            }
        }
    }

    fn solve_crossing(
        &self,
        motor: usize,
        view: &ClockedMotorSpan,
        (lo, lo_position): (u64, f64),
        (hi, hi_position): (u64, f64),
        level: f64,
        slope: Slope,
    ) -> Result<(u64, f64), ShimError> {
        if slope.reached(lo_position, level) {
            return Ok((lo, lo_position));
        }
        let mut low = lo;
        let mut deficit_low = slope.deficit(lo_position, level);
        let mut high = hi;
        let mut surplus_high = -slope.deficit(hi_position, level);
        let mut high_position = hi_position;
        let mut previous_side: Option<bool> = None;
        let mut iteration = 0_u32;
        while high - low > 1 {
            let span = high - low;
            let bisection = low + span / 2;
            let safeguarded =
                iteration % BISECTION_SAFEGUARD_PERIOD == BISECTION_SAFEGUARD_PERIOD - 1;
            let candidate = if safeguarded {
                bisection
            } else {
                let fraction = deficit_low / (deficit_low + surplus_high);
                if fraction.is_finite() && fraction > 0.0 && fraction < 1.0 {
                    low + ((fraction * span as f64).ceil() as u64).clamp(1, span - 1)
                } else {
                    bisection
                }
            };
            let position = self.position_at(motor, view, candidate)?;
            let reached = slope.reached(position, level);
            if reached {
                high = candidate;
                high_position = position;
                surplus_high = -slope.deficit(position, level);
                if previous_side == Some(true) {
                    deficit_low *= 0.5;
                }
            } else {
                low = candidate;
                deficit_low = slope.deficit(position, level);
                if previous_side == Some(false) {
                    surplus_high *= 0.5;
                }
            }
            previous_side = Some(reached);
            iteration = iteration.wrapping_add(1);
        }
        Ok((high, high_position))
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
        CERT_NONE_COUNT.with(|c| c.set(c.get() + 1));
        if !self.interval_can_reach_next_lattice(motor, view, from, to)? {
            PRUNE_COUNT.with(|c| c.set(c.get() + 1));
            return Ok(());
        }
        if to - from <= 1 {
            let rise = self.position_at(motor, view, to)? - self.position_at(motor, view, from)?;
            let slope = if rise >= 0.0 {
                Slope::Rising
            } else {
                Slope::Falling
            };
            return self.emit_run(motor, cfg, view, from, to, slope, out);
        }
        let mid = from + (to - from) / 2;
        self.emit_interval(motor, cfg, view, from, mid, out)?;
        if self.drain_halted {
            return Ok(());
        }
        self.emit_interval(motor, cfg, view, mid, to, out)
    }

    fn interval_can_reach_next_lattice(
        &self,
        motor: usize,
        view: &ClockedMotorSpan,
        from: u64,
        to: u64,
    ) -> Result<bool, ShimError> {
        BOUNDS_COUNT.with(|c| c.set(c.get() + 1));
        let t_from = self.stream_time(motor, view, from)?;
        let t_to = self.stream_time(motor, view, to)?;
        let bounds = view
            .signal
            .pva_bounds(t_from, t_to)
            .map_err(|error| ShimError::SpanEval { motor, error })?;
        let from_position = self.position_at(motor, view, from)?;
        let to_position = self.position_at(motor, view, to)?;
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
        BOUNDS_COUNT.with(|c| c.set(c.get() + 1));
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
        EVAL_COUNT.with(|c| c.set(c.get() + 1));
        view.eval_at_clock(clock)
            .map_err(|error| ShimError::SpanEval { motor, error })
    }

    fn position_at(
        &self,
        motor: usize,
        view: &ClockedMotorSpan,
        clock: u64,
    ) -> Result<f64, ShimError> {
        EVAL_COUNT.with(|c| c.set(c.get() + 1));
        view.position_at_clock(clock)
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
