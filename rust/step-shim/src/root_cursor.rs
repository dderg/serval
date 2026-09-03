use std::sync::Arc;

use trajectory::{ClockedMotorSpan, LocalPolynomial, MotorSpan, Pva};

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

/// A motor-local signal's own step lattice, tied to the signal that opened it:
/// the overlay walks that lattice instead of the lane's while the signal is
/// active, and a different signal opens a fresh one.
#[derive(Debug)]
struct Overlay {
    signal: Arc<MotorSpan>,
    lattice: Lattice,
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
    overlay: Option<Overlay>,
    /// The sub-microstep remainder the last overlay left unstepped: its final
    /// continuous position minus the last lattice threshold it actually
    /// crossed. A relative overlay signal restarts its coordinate frame at
    /// zero, but the rotor holds where the previous overlay stepped it - a
    /// fresh lattice anchored at the raw signal origin would silently discard
    /// this remainder at every seam, and motors_sync-style buzz waveforms
    /// (dozens of fractional-amplitude nudges) integrate the discards into
    /// real drift.
    overlay_carry_mm: f64,
    positioned: bool,
    resume_floor: u64,
    origin_clock: Option<u64>,
    frontier: u64,
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
            overlay_carry_mm: 0.0,
            positioned: false,
            resume_floor: 0,
            origin_clock: None,
            frontier: 0,
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
        self.resume_floor
    }

    pub fn origin_clock(&self) -> Option<u64> {
        self.origin_clock
    }

    /// The unstepped sub-microstep remainder of the last completed overlay.
    /// It survives [`Self::reset_to`]: a cut re-anchors the bookkeeping while
    /// the rotor stays put, so the remainder is still physically real. Only an
    /// external position reseed (homing, counter handover) redefines the
    /// rotor's truth and clears it via [`Self::set_step_remainder`].
    pub fn step_remainder(&self) -> f64 {
        self.overlay_carry_mm
    }

    pub fn set_step_remainder(&mut self, carry_mm: f64) {
        self.overlay_carry_mm = carry_mm;
    }

    pub fn reset_to(&mut self, count: i64, resume_floor: u64) {
        self.lane = Lattice {
            origin_mm: 0.0,
            step_count: count,
        };
        self.overlay = None;
        self.positioned = true;
        self.resume_floor = resume_floor;
        self.origin_clock = resume_floor.checked_sub(1);
        self.last_root_clock = None;
        self.frontier = 0;
    }

    /// The lane's clock slope changed epoch, so every clock the cursor holds
    /// belongs to the old one. The step lattice and its unstepped remainder are
    /// physical and survive.
    pub fn retime(&mut self, cfg: &MotorConfig) {
        let count = self.lane.step_count;
        let resume_floor = self.resume_floor;
        let carry = self.overlay_carry_mm;
        *self = Self::new(cfg);
        self.reset_to(count, resume_floor);
        self.overlay_carry_mm = carry;
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
            let begin = signal_start.max(self.frontier);
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
            self.frontier = last_clock + 1;
            if last_clock < view.end_clock {
                return Ok(());
            }
            if let Some(overlay) = &self.overlay {
                let nominal = overlay.lattice.nominal_position(self.microstep_mm);
                let end_position = self.position_at(motor, &view, view.end_clock)?;
                self.overlay_carry_mm = end_position - nominal;
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
        } else {
            let continues_signal = self
                .overlay
                .as_ref()
                .is_some_and(|overlay| Arc::ptr_eq(&overlay.signal, &view.signal));
            if !continues_signal {
                let position = self.position_at(motor, view, signal_start)?;
                self.overlay = Some(Overlay {
                    signal: Arc::clone(&view.signal),
                    lattice: Lattice {
                        origin_mm: position - self.overlay_carry_mm,
                        step_count: 0,
                    },
                });
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
            return self.emit_window(motor, cfg, view, begin, last_clock, None, out);
        }
        let mut boundaries = vec![begin, last_clock];
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
            boundaries.push(last_clock_at_or_before(view, t));
        }
        boundaries.retain(|clock| *clock >= begin && *clock <= last_clock);
        boundaries.sort_unstable();
        boundaries.dedup();
        WINDOW_COUNT.with(|c| c.set(c.get() + boundaries.len() as u64 - 1));
        let mut models = vec![WindowModel::Unbuilt; boundaries.len() - 1];
        let mut index = 0;
        while index + 1 < boundaries.len() {
            let from = boundaries[index];
            let model = self.window_model(motor, view, &boundaries, &mut models, index)?;
            match self.certified_slope(motor, view, from, boundaries[index + 1], model.as_ref())? {
                Some(slope) => {
                    let mut end_index = index + 1;
                    while end_index + 1 < boundaries.len() {
                        let next =
                            self.window_model(motor, view, &boundaries, &mut models, end_index)?;
                        let piece = self.certified_slope(
                            motor,
                            view,
                            boundaries[end_index],
                            boundaries[end_index + 1],
                            next.as_ref(),
                        )?;
                        if piece != Some(slope) {
                            break;
                        }
                        end_index += 1;
                    }
                    self.emit_run(
                        motor,
                        cfg,
                        view,
                        from,
                        boundaries[end_index],
                        slope,
                        model.as_ref(),
                        out,
                    )?;
                    index = end_index;
                }
                None => {
                    self.subdivide(
                        motor,
                        cfg,
                        view,
                        from,
                        boundaries[index + 1],
                        model.as_ref(),
                        out,
                    )?;
                    index += 1;
                }
            }
            if self.drain_halted {
                return Ok(());
            }
        }
        Ok(())
    }

    /// The polynomial for the window starting at boundary `index`, built once
    /// and kept for the merge probes and the run that follow.
    fn window_model(
        &self,
        motor: usize,
        view: &ClockedMotorSpan,
        boundaries: &[u64],
        models: &mut [WindowModel],
        index: usize,
    ) -> Result<Option<LocalPolynomial>, ShimError> {
        if let WindowModel::Built(model) = models[index] {
            return Ok(model);
        }
        let model = self.model_at(motor, view, boundaries[index])?;
        models[index] = WindowModel::Built(model);
        Ok(model)
    }

    fn emit_run(
        &mut self,
        motor: usize,
        cfg: &MotorConfig,
        view: &ClockedMotorSpan,
        lo: u64,
        hi: u64,
        slope: Slope,
        model: Option<&LocalPolynomial>,
        out: &mut Vec<StepRoot>,
    ) -> Result<(), ShimError> {
        if self.halt_if_past_deadline(lo) {
            return Ok(());
        }
        let run_end = self.position_at(motor, view, hi)?;
        let mut search_from = lo;
        let mut search_position = self.position_at(motor, view, lo)?;
        let mut predictor =
            Predictor::for_run(view, lo, search_position, run_end, self.microstep_mm, model);
        let mut roots_since_check = 0_u32;
        loop {
            let level = self.frame().threshold(self.microstep_mm, slope);
            if !slope.reached(run_end, level) {
                let nominal = self.frame().nominal_position(self.microstep_mm);
                if (run_end - nominal).abs() >= self.microstep_mm {
                    return Err(ShimError::LatticeDrift {
                        motor,
                        source_line: view.signal.source_line,
                        clock: hi,
                        position: run_end,
                        nominal,
                    });
                }
                return Ok(());
            }
            let (clock, position) = self.solve_crossing(
                motor,
                view,
                (search_from, search_position),
                (hi, run_end),
                level,
                slope,
                predictor.as_mut(),
            )?;
            self.push_root(motor, cfg, view, clock, slope, out)?;
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
                    self.frontier = clock + 1;
                    return Ok(());
                }
            }
        }
    }

    /// The first clock in `(lo, hi]` whose position reaches `level`, by
    /// shrinking the bracket `(low, high)` until it is one clock wide. With
    /// a predictor the exact local polynomial names the crossing to a
    /// fraction of a clock; when it also places that clock and the one
    /// before it on opposite sides of the level by more than its noise band,
    /// the answer is settled without touching the carrier. Otherwise the
    /// carrier is probed there, and a safeguarded secant takes over when no
    /// prediction is available.
    fn solve_crossing(
        &self,
        motor: usize,
        view: &ClockedMotorSpan,
        (lo, lo_position): (u64, f64),
        (hi, hi_position): (u64, f64),
        level: f64,
        slope: Slope,
        mut predictor: Option<&mut Predictor>,
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
        let mut last_probe = (lo, lo_position);
        while high - low > 1 {
            let span = high - low;
            let safeguarded =
                iteration % BISECTION_SAFEGUARD_PERIOD == BISECTION_SAFEGUARD_PERIOD - 1;
            let predicted = if safeguarded {
                None
            } else {
                predictor
                    .as_deref_mut()
                    .and_then(|predictor| predictor.crossing_clock(last_probe, level, low, high))
            };
            let candidate = match predicted {
                Some(candidate) => {
                    if let Some(settled) = predictor
                        .as_deref()
                        .and_then(|predictor| predictor.settles(candidate, level, slope, low))
                    {
                        return Ok(settled);
                    }
                    candidate
                }
                None if safeguarded => low + span / 2,
                None => {
                    let fraction = deficit_low / (deficit_low + surplus_high);
                    low + ((fraction * span as f64).ceil() as u64).clamp(1, span - 1)
                }
            };
            let position = self.position_at(motor, view, candidate)?;
            last_probe = (candidate, position);
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
        view: &ClockedMotorSpan,
        clock: u64,
        slope: Slope,
        out: &mut Vec<StepRoot>,
    ) -> Result<(), ShimError> {
        let advance = slope.advance();
        if let Some(previous_clock) = self.last_root_clock.filter(|&last| clock <= last) {
            return Err(ShimError::StepClockRegression {
                motor,
                source_line: view.signal.source_line,
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

    fn emit_window(
        &mut self,
        motor: usize,
        cfg: &MotorConfig,
        view: &ClockedMotorSpan,
        lo: u64,
        hi: u64,
        model: Option<&LocalPolynomial>,
        out: &mut Vec<StepRoot>,
    ) -> Result<(), ShimError> {
        if lo == hi {
            return self.emit_single_clock(motor, cfg, view, lo, out);
        }
        match self.certified_slope(motor, view, lo, hi, model)? {
            Some(slope) => self.emit_run(motor, cfg, view, lo, hi, slope, model, out),
            None => self.subdivide(motor, cfg, view, lo, hi, model, out),
        }
    }

    /// One clock carries no rise and no signed duration, so neither the
    /// velocity bounds nor the endpoint difference can name its direction: the
    /// lattice threshold the position has reached is the only thing that can.
    fn emit_single_clock(
        &mut self,
        motor: usize,
        cfg: &MotorConfig,
        view: &ClockedMotorSpan,
        clock: u64,
        out: &mut Vec<StepRoot>,
    ) -> Result<(), ShimError> {
        let position = self.position_at(motor, view, clock)?;
        let lattice = self.frame();
        let slope = if position >= lattice.threshold(self.microstep_mm, Slope::Rising) {
            Slope::Rising
        } else if position <= lattice.threshold(self.microstep_mm, Slope::Falling) {
            Slope::Falling
        } else {
            return Ok(());
        };
        self.emit_run(motor, cfg, view, clock, clock, slope, None, out)
    }

    /// The window carries no certified slope, so it is halved until one half
    /// does — or until a single clock is left and the rise decides.
    fn subdivide(
        &mut self,
        motor: usize,
        cfg: &MotorConfig,
        view: &ClockedMotorSpan,
        lo: u64,
        hi: u64,
        model: Option<&LocalPolynomial>,
        out: &mut Vec<StepRoot>,
    ) -> Result<(), ShimError> {
        if self.halt_if_past_deadline(lo) {
            return Ok(());
        }
        CERT_NONE_COUNT.with(|c| c.set(c.get() + 1));
        if !self.interval_can_reach_next_lattice(motor, view, lo, hi, model)? {
            PRUNE_COUNT.with(|c| c.set(c.get() + 1));
            return Ok(());
        }
        if hi - lo <= 1 {
            let rise = self.position_at(motor, view, hi)? - self.position_at(motor, view, lo)?;
            let slope = if rise >= 0.0 {
                Slope::Rising
            } else {
                Slope::Falling
            };
            return self.emit_run(motor, cfg, view, lo, hi, slope, model, out);
        }
        let mid = lo + (hi - lo) / 2;
        self.emit_window(motor, cfg, view, lo, mid, model, out)?;
        if self.drain_halted {
            return Ok(());
        }
        self.emit_window(motor, cfg, view, mid, hi, model, out)
    }

    fn interval_can_reach_next_lattice(
        &self,
        motor: usize,
        view: &ClockedMotorSpan,
        from: u64,
        to: u64,
        model: Option<&LocalPolynomial>,
    ) -> Result<bool, ShimError> {
        BOUNDS_COUNT.with(|c| c.set(c.get() + 1));
        let t_from = self.stream_time(motor, view, from)?;
        let t_to = self.stream_time(motor, view, to)?;
        let nominal = self.frame().nominal_position(self.microstep_mm);
        if let Some((hull_from, hull_to)) = clocks_covered(model, view, t_from, t_to) {
            let (mut lower, mut upper) = model
                .expect("a covered window has a model")
                .position_range(hull_from, hull_to);
            if hull_from > t_from {
                let from_position = self.position_at(motor, view, from)?;
                lower = lower.min(from_position);
                upper = upper.max(from_position);
            }
            return Ok(lower <= nominal - self.microstep_mm || upper >= nominal + self.microstep_mm);
        }
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
        Ok(lower <= nominal - self.microstep_mm || upper >= nominal + self.microstep_mm)
    }

    fn certified_slope(
        &self,
        motor: usize,
        view: &ClockedMotorSpan,
        from: u64,
        to: u64,
        model: Option<&LocalPolynomial>,
    ) -> Result<Option<Slope>, ShimError> {
        BOUNDS_COUNT.with(|c| c.set(c.get() + 1));
        let t_from = self.stream_time(motor, view, from)?;
        let t_to = self.stream_time(motor, view, to)?;
        let duration = (to - from) as f64 / view.clock_freq_hz;
        if let Some((hull_from, hull_to)) = clocks_covered(model, view, t_from, t_to) {
            let model = model.expect("a covered window has a model");
            let (velocity_min, velocity_max) = model.velocity_range(hull_from, hull_to);
            let (position_min, position_max) = model.position_range(hull_from, hull_to);
            let position_scale = position_min
                .abs()
                .max(position_max.abs())
                .max(self.microstep_mm);
            let position_rate_scale = position_scale / duration.max(view.clock_freq_hz.recip());
            let velocity_scale = velocity_min
                .abs()
                .max(velocity_max.abs())
                .max(position_rate_scale)
                .max(1.0);
            let tolerance = 256.0 * f64::EPSILON * velocity_scale;
            let first_rise = if hull_from > t_from {
                self.position_at(motor, view, from + 1)? - self.position_at(motor, view, from)?
            } else {
                0.0
            };
            let tick_tolerance = tolerance / view.clock_freq_hz;
            if velocity_min >= -tolerance && first_rise >= -tick_tolerance {
                return Ok(Some(Slope::Rising));
            }
            if velocity_max <= tolerance && first_rise <= tick_tolerance {
                return Ok(Some(Slope::Falling));
            }
            return Ok(None);
        }
        let bounds = view
            .signal
            .pva_bounds(t_from, t_to)
            .map_err(|error| ShimError::SpanEval { motor, error })?;
        let from_pva = self.eval(motor, view, from)?;
        let to_pva = self.eval(motor, view, to)?;
        let position_scale = from_pva
            .position
            .abs()
            .max(to_pva.position.abs())
            .max(self.microstep_mm);
        let position_rate_scale = position_scale / duration.max(view.clock_freq_hz.recip());
        let velocity_scale = bounds
            .velocity_min
            .abs()
            .max(bounds.velocity_max.abs())
            .max(from_pva.velocity.abs())
            .max(to_pva.velocity.abs())
            .max(bounds.acceleration_abs_max * duration)
            .max(position_rate_scale)
            .max(1.0);
        let tolerance = 256.0 * f64::EPSILON * velocity_scale;
        if bounds.velocity_min >= -tolerance && to_pva.position >= from_pva.position {
            return Ok(Some(Slope::Rising));
        }
        if bounds.velocity_max <= tolerance && to_pva.position <= from_pva.position {
            return Ok(Some(Slope::Falling));
        }
        if !bounds.velocity_continuous {
            return Ok(None);
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

    /// The signal from `clock` to its next breakpoint as one polynomial, when
    /// its carriers admit one. A window cut at a breakpoint starts on the
    /// last clock before it, so a polynomial ending within one clock is the
    /// previous piece and the one beginning at that breakpoint is taken.
    fn model_at(
        &self,
        motor: usize,
        view: &ClockedMotorSpan,
        clock: u64,
    ) -> Result<Option<LocalPolynomial>, ShimError> {
        let t = self.stream_time(motor, view, clock)?;
        let Some(model) = view.signal.local_polynomial(t) else {
            return Ok(None);
        };
        if model.valid_until() - t < view.clock_freq_hz.recip() {
            return Ok(view.signal.local_polynomial(model.valid_until()));
        }
        Ok(Some(model))
    }

    fn halt_if_past_deadline(&mut self, resume_clock: u64) -> bool {
        if self
            .drain_deadline
            .is_some_and(|deadline| std::time::Instant::now() >= deadline)
        {
            self.drain_halted = true;
            self.frontier = resume_clock;
            return true;
        }
        false
    }

    fn frame(&self) -> Lattice {
        self.overlay
            .as_ref()
            .map_or(self.lane, |overlay| overlay.lattice)
    }

    fn frame_mut(&mut self) -> &mut Lattice {
        match &mut self.overlay {
            Some(overlay) => &mut overlay.lattice,
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

/// A window's polynomial, built the first time the window is certified or
/// run so a drain halted early never pays for windows it did not reach.
#[derive(Clone, Copy)]
enum WindowModel {
    Unbuilt,
    Built(Option<LocalPolynomial>),
}

/// Roots a run is expected to carry before building its polynomial pays for
/// itself: the build costs about as much as ten signal evaluations and each
/// predicted root saves about three.
const PREDICTOR_MIN_STEPS: f64 = 4.0;
const PREDICTOR_NEWTON_ITERATIONS: u32 = 16;
const PREDICTOR_KNOT_HOPS: u32 = 8;

/// The signal's exact local polynomial, moved along the run knot by knot, so
/// each crossing is predicted by Newton on a cheap Horner form instead of
/// bracketed on the spline.
struct Predictor<'a> {
    view: &'a ClockedMotorSpan,
    model: LocalPolynomial,
}

impl<'a> Predictor<'a> {
    fn for_run(
        view: &'a ClockedMotorSpan,
        lo: u64,
        lo_position: f64,
        hi_position: f64,
        microstep_mm: f64,
        model: Option<&LocalPolynomial>,
    ) -> Option<Self> {
        let t_lo = view.stream_time_at_clock(lo).ok()?;
        if let Some(model) = model.filter(|model| t_lo < model.valid_until()) {
            return Some(Self {
                view,
                model: *model,
            });
        }
        if (hi_position - lo_position).abs() < PREDICTOR_MIN_STEPS * microstep_mm {
            return None;
        }
        let model = view.signal.local_polynomial(t_lo)?;
        Some(Self { view, model })
    }

    fn stream_time(&self, clock: u64) -> f64 {
        self.view.stream_t_start
            + (clock as f64 - self.view.start_clock_exact) / self.view.clock_freq_hz
    }

    fn first_clock_at_or_after(&self, t: f64) -> f64 {
        (self.view.start_clock_exact + (t - self.view.stream_t_start) * self.view.clock_freq_hz)
            .ceil()
    }

    /// Move the polynomial forward over the knots `t` has passed.
    fn follow(&mut self, t: f64, hops: &mut u32) -> Option<()> {
        while t >= self.model.valid_until() {
            if *hops == PREDICTOR_KNOT_HOPS {
                return None;
            }
            *hops += 1;
            self.model = self
                .view
                .signal
                .local_polynomial(self.model.valid_until())?;
        }
        Some(())
    }

    /// The crossing at `candidate` when the polynomial places that clock and
    /// the one before it on opposite sides of the level by more than its
    /// noise band; `low` is already known to fall short exactly.
    fn settles(&self, candidate: u64, level: f64, slope: Slope, low: u64) -> Option<(u64, f64)> {
        let t = self.stream_time(candidate);
        if !(self.model.t0() <= t && t < self.model.valid_until()) {
            return None;
        }
        let position = self.model.position(t);
        if slope.deficit(position, level) > -self.model.noise_band(t) {
            return None;
        }
        if candidate - 1 > low {
            let t_before = self.stream_time(candidate - 1);
            if t_before < self.model.t0() {
                return None;
            }
            let before = self.model.position(t_before);
            if slope.deficit(before, level) < self.model.noise_band(t_before) {
                return None;
            }
        }
        Some((candidate, position))
    }

    /// The first clock strictly inside `(low, high)` at or after the level
    /// crossing, refined from an exact probe `(clock, position)` by Newton
    /// steps on the polynomial. `None` leaves the probe to the caller's
    /// secant when the polynomial cannot name it.
    fn crossing_clock(
        &mut self,
        (probe_clock, probe_position): (u64, f64),
        level: f64,
        low: u64,
        high: u64,
    ) -> Option<u64> {
        let t_low = self.stream_time(low);
        let t_high = self.stream_time(high);
        let settled = 1e-3 / self.view.clock_freq_hz;
        let mut t = self.stream_time(probe_clock);
        let mut residual = probe_position - level;
        let mut hops = 0;
        for _ in 0..PREDICTOR_NEWTON_ITERATIONS {
            self.follow(t, &mut hops)?;
            let step = -residual / self.model.velocity(t);
            if !step.is_finite() {
                return None;
            }
            t += step;
            if !(t_low..=t_high).contains(&t) {
                return None;
            }
            if step.abs() <= settled {
                break;
            }
            self.follow(t, &mut hops)?;
            residual = self.model.position(t) - level;
        }
        let candidate = self.first_clock_at_or_after(t);
        if !candidate.is_finite() {
            return None;
        }
        Some((candidate as u64).clamp(low + 1, high - 1))
    }
}

/// The last clock whose stream time is at or before `t`, so a window cut at a
/// breakpoint never carries a clock past it.
fn last_clock_at_or_before(view: &ClockedMotorSpan, t: f64) -> u64 {
    let exact = view.start_clock_exact + (t - view.stream_t_start) * view.clock_freq_hz;
    (exact.floor() as u64).clamp(view.start_clock, view.end_clock)
}

/// The stream interval on which `model` speaks for every clock after
/// `t_from` up to `t_to`: the polynomial may begin up to one clock after
/// `t_from` — no clock lies in that sliver but `from` itself — and must
/// reach `t_to`. `None` when the window is not the polynomial's.
fn clocks_covered(
    model: Option<&LocalPolynomial>,
    view: &ClockedMotorSpan,
    t_from: f64,
    t_to: f64,
) -> Option<(f64, f64)> {
    let model = model?;
    let tick = view.clock_freq_hz.recip();
    let roundoff = 1e-6 * tick;
    (model.t0() <= t_from + tick && t_to <= model.valid_until() + roundoff && t_from <= t_to).then(
        || {
            (
                t_from.max(model.t0()).min(t_to),
                t_to.min(model.valid_until()),
            )
        },
    )
}

#[cfg(test)]
#[path = "root_cursor_tests.rs"]
mod tests;
