use nurbs::ScalarNurbs;
use nurbs::bezier::bezier_pieces_to_nurbs;
use trajectory::{AxisChainSet, ChainStage, ShapedSegment};

use crate::PostProcessError;
use crate::shaper::{TrackSignal, apply_pressure_advance_to_track, fit_axis_from_signal};

const INTEGRAL_TOL_MM: f64 = 1e-10;
const INTEGRAL_MAX_DEPTH: u32 = 24;
const GRID_DEDUP_EPS_S: f64 = 1e-12;
const SPAN_MIN_LEN_MM: f64 = 1e-12;
const SPAN_LOOKUP_SLACK_MM: f64 = 1e-6;

/// Rebuild every projected-follower track from its leaders' *shaped* motion.
///
/// The raw move stream defines an extrusion-per-path-distance profile: each
/// spatial segment contributes a span of raw arc length carrying its follower
/// demand's ratio (zero for travel). The projection extrudes each raw
/// millimeter when the *shaped* path traverses it: the follower's velocity is
/// `r(s_shaped(t)) · |v_shaped(t)|` and its position the running integral.
/// This keeps the follower moving through rest holds (the kernel's creep is
/// real traversal of the raw path's tail) and continuous across every seam,
/// and it makes the extruded amount track the shaped path's true distance —
/// permanently short of the commanded total by exactly the corner-cut length.
/// Extrude-only moves ride no spatial path; their raw track adds in directly.
/// The follower's own chain (pressure advance) applies after the projection.
pub(crate) fn project_followers(
    base: &[ShapedSegment],
    out: &mut [ShapedSegment],
    chains: &AxisChainSet,
    states: &mut Vec<FollowerState>,
) -> Result<(), PostProcessError> {
    if states.len() < chains.n_axes() {
        states.resize_with(chains.n_axes(), FollowerState::default);
    }
    for (axis, leaders) in chains.projected_followers() {
        let chain = &chains.chains[axis];
        assert!(
            !chain
                .stages
                .iter()
                .any(|s| matches!(s, ChainStage::SmoothKernel(_))),
            "follower axis {axis} declares a smoothing kernel; a follower is \
             projected onto its leaders' shaped motion instead"
        );
        let leaders_shaped = leaders.iter().any(|&l| !chains.chains[l].is_empty());
        let state = &mut states[axis];
        let projecting = leaders_shaped || state.active;
        if !projecting && chain.is_empty() {
            continue;
        }
        if projecting {
            state.active = true;
            for seg in base {
                state.ingest(seg, axis, leaders);
            }
        }
        for i in 0..out.len() {
            let raw = &base[i];
            if axis >= raw.axes.len() {
                return Err(PostProcessError::AxisCountMismatch {
                    expected: chains.n_axes(),
                    got: raw.axes.len(),
                });
            }
            let raw_track = &raw.axes[axis];
            let projected = if projecting {
                let raw_start = nurbs::eval::eval(raw_track, raw.t_start);
                let e_start = state.e_end.unwrap_or(raw_start - state.carried_deficit);
                let (track, s_end, e_end) = {
                    let sig = FollowerSignal::new(&out[i], raw, axis, leaders, &*state, e_start);
                    let track = fit_axis_from_signal(axis, raw_track, &sig)?;
                    (track, sig.s_end(), sig.eval(raw.t_end))
                };
                state.s_shaped = s_end;
                state.e_end = Some(e_end);
                state.carried_deficit = nurbs::eval::eval(raw_track, raw.t_end) - e_end;
                track
            } else {
                raw_track.clone()
            };
            let mut track = projected;
            for stage in &chain.stages {
                if let ChainStage::LinearPressureAdvance { k } = stage {
                    track = apply_pressure_advance_to_track(&track, *k);
                }
            }
            if !track.control_points().iter().all(|v| v.is_finite()) {
                return Err(PostProcessError::NonFiniteSample {
                    axis,
                    t: raw.t_start,
                });
            }
            out[i].axes[axis] = track;
        }
        if projecting {
            state.prune_spans();
        }
    }
    Ok(())
}

/// One raw segment's stretch of path: raw arc length `[s0, s1]` carrying a
/// linearly ramped follower ratio, with `e0` the cumulative projected
/// extrusion at its start.
#[derive(Debug, Clone, Copy)]
struct RatioSpan {
    s0: f64,
    s1: f64,
    r0: f64,
    r1: f64,
    e0: f64,
}

impl RatioSpan {
    fn ratio_at(&self, s: f64) -> f64 {
        self.r0 + (self.r1 - self.r0) * (s - self.s0) / (self.s1 - self.s0)
    }

    fn ratio_slope(&self) -> f64 {
        (self.r1 - self.r0) / (self.s1 - self.s0)
    }

    fn e_at(&self, s: f64) -> f64 {
        let ds = s - self.s0;
        self.e0 + self.r0 * ds + 0.5 * (self.r1 - self.r0) * ds * ds / (self.s1 - self.s0)
    }
}

/// Per-follower streaming state: the raw extrusion-per-distance table and the
/// shaped path odometer, carried across emit windows so the projected track
/// stays continuous through window boundaries, resets, and chain swaps.
#[derive(Debug, Default)]
pub(crate) struct FollowerState {
    active: bool,
    spans: Vec<RatioSpan>,
    ingested_through_t: Option<f64>,
    s_raw_end: f64,
    s_shaped: f64,
    e_end: Option<f64>,
    carried_deficit: f64,
}

impl FollowerState {
    /// A `Reset` restarts the timeline and relabels positions; the physical
    /// gap between commanded and projected extrusion survives it, so the next
    /// stream re-anchors at the new raw position minus the carried deficit.
    pub(crate) fn reset_timeline(&mut self) {
        self.spans.clear();
        self.ingested_through_t = None;
        self.s_raw_end = 0.0;
        self.s_shaped = 0.0;
        self.e_end = None;
    }

    pub(crate) fn is_active(&self) -> bool {
        self.active
    }

    fn ingest(&mut self, seg: &ShapedSegment, axis: usize, leaders: &[usize]) {
        if let Some(through) = self.ingested_through_t {
            if seg.t_end <= through + GRID_DEDUP_EPS_S {
                return;
            }
            assert!(
                seg.t_start >= through - GRID_DEDUP_EPS_S,
                "follower span ingestion saw an out-of-order segment: \
                 t_start {} before ingested-through {}",
                seg.t_start,
                through
            );
        }
        self.ingested_through_t = Some(seg.t_end);
        let leader_d1: Vec<ScalarNurbs> = leaders
            .iter()
            .map(|&l| derivative_or_zero(&seg.axes[l]))
            .collect();
        let speed = |t: f64| {
            leader_d1
                .iter()
                .map(|c| nurbs::eval::eval(c, t).powi(2))
                .sum::<f64>()
                .sqrt()
        };
        let grid = knot_grid(&leader_d1, seg.t_start, seg.t_end);
        let mut ds = 0.0;
        for w in grid.windows(2) {
            ds += integrate(&speed, w[0], w[1]);
        }
        if ds <= SPAN_MIN_LEN_MM {
            return;
        }
        let (r0, r1) = seg
            .followers
            .iter()
            .find(|f| f.axis_index == axis)
            .filter(|_| seg.spatial_path)
            .map_or((0.0, 0.0), |f| (f.ratio, f.ratio_end));
        let e0 = self.spans.last().map_or(0.0, |span| span.e_at(span.s1));
        self.spans.push(RatioSpan {
            s0: self.s_raw_end,
            s1: self.s_raw_end + ds,
            r0,
            r1,
            e0,
        });
        self.s_raw_end += ds;
    }

    fn prune_spans(&mut self) {
        let keep_from = self
            .spans
            .partition_point(|span| span.s1 < self.s_shaped - SPAN_LOOKUP_SLACK_MM);
        self.spans.drain(..keep_from);
    }

    /// Cumulative projected spatial extrusion at shaped path distance `s`.
    /// A kernel with negative lobes can overshoot the ingested path by a
    /// whisker at a terminal flush; the terminal ratio extends it.
    fn spans_e(&self, s: f64) -> f64 {
        let Some(first) = self.spans.first() else {
            return 0.0;
        };
        assert!(
            s >= first.s0 - SPAN_LOOKUP_SLACK_MM,
            "shaped path odometer {s} fell behind the pruned span table \
             starting at {}",
            first.s0
        );
        let idx = self.spans.partition_point(|span| span.s1 < s);
        match self.spans.get(idx) {
            Some(span) => span.e_at(s.max(span.s0)),
            None => {
                let last = self.spans.last().expect("non-empty spans");
                last.e_at(last.s1) + last.r1 * (s - last.s1)
            }
        }
    }

    fn ratio_and_slope(&self, s: f64) -> (f64, f64) {
        let idx = self.spans.partition_point(|span| span.s1 < s);
        match self.spans.get(idx) {
            Some(span) if s >= span.s0 => (span.ratio_at(s), span.ratio_slope()),
            Some(span) => (span.r0, 0.0),
            None => (self.spans.last().map_or(0.0, |span| span.r1), 0.0),
        }
    }
}

/// A constant track (a held axis) differentiates to zero; `nurbs` refuses
/// degree-0 input, so hold the zero explicitly over the same span.
fn derivative_or_zero(curve: &ScalarNurbs) -> ScalarNurbs {
    if curve.degree() >= 1 {
        return nurbs::eval::derivative(curve);
    }
    let knots = curve.knots();
    bezier_pieces_to_nurbs(&[nurbs::bezier::BezierPiece {
        u_start: *knots.first().expect("curve has a span"),
        u_end: *knots.last().expect("curve has a span"),
        coeffs: vec![0.0],
    }])
}

fn knot_grid(curves: &[ScalarNurbs], t0: f64, t1: f64) -> Vec<f64> {
    let mut grid: Vec<f64> = vec![t0, t1];
    for curve in curves {
        grid.extend(curve.knots().iter().copied().filter(|&k| k > t0 && k < t1));
    }
    grid.sort_by(f64::total_cmp);
    grid.dedup_by(|a, b| (*a - *b).abs() <= GRID_DEDUP_EPS_S);
    grid
}

/// One committed segment's projected follower as an evaluable signal:
/// `e(t) = e_start + E(s_shaped(t)) − E(s_start) + raw_e_delta(t)`, where `E`
/// is the spans table's cumulative extrusion and the raw-delta term carries
/// extrude-only motion (segments with no spatial path).
pub(crate) struct FollowerSignal<'a> {
    state: &'a FollowerState,
    e_start: f64,
    s_start: f64,
    e_spans_start: f64,
    t0: f64,
    t1: f64,
    shaped_d1: Vec<ScalarNurbs>,
    shaped_d2: Vec<ScalarNurbs>,
    raw_delta: Option<(ScalarNurbs, ScalarNurbs, ScalarNurbs, f64)>,
    grid: Vec<f64>,
    cumulative: Vec<f64>,
}

impl<'a> FollowerSignal<'a> {
    fn new(
        shaped: &ShapedSegment,
        raw: &ShapedSegment,
        axis: usize,
        leaders: &[usize],
        state: &'a FollowerState,
        e_start: f64,
    ) -> Self {
        let (t0, t1) = (raw.t_start, raw.t_end);
        let shaped_d1: Vec<ScalarNurbs> = leaders
            .iter()
            .map(|&l| derivative_or_zero(&shaped.axes[l]))
            .collect();
        let shaped_d2: Vec<ScalarNurbs> = shaped_d1.iter().map(derivative_or_zero).collect();
        let raw_delta = (!raw.spatial_path).then(|| {
            let track = raw.axes[axis].clone();
            let d1 = derivative_or_zero(&track);
            let d2 = derivative_or_zero(&d1);
            let at_start = nurbs::eval::eval(&track, t0);
            (track, d1, d2, at_start)
        });
        let grid = knot_grid(&shaped_d1, t0, t1);

        let mut sig = Self {
            state,
            e_start,
            s_start: state.s_shaped,
            e_spans_start: state.spans_e(state.s_shaped),
            t0,
            t1,
            shaped_d1,
            shaped_d2,
            raw_delta,
            grid,
            cumulative: Vec::new(),
        };
        let mut cumulative = Vec::with_capacity(sig.grid.len());
        let mut acc = 0.0;
        cumulative.push(0.0);
        for w in sig.grid.clone().windows(2) {
            acc += integrate(&|t| sig.shaped_speed(t), w[0], w[1]);
            cumulative.push(acc);
        }
        sig.cumulative = cumulative;
        sig
    }

    fn shaped_speed(&self, t: f64) -> f64 {
        self.shaped_d1
            .iter()
            .map(|c| nurbs::eval::eval(c, t).powi(2))
            .sum::<f64>()
            .sqrt()
    }

    fn shaped_speed_deriv(&self, t: f64) -> f64 {
        let speed = self.shaped_speed(t);
        if speed <= 1e-9 {
            return 0.0;
        }
        self.shaped_d1
            .iter()
            .zip(&self.shaped_d2)
            .map(|(v, a)| nurbs::eval::eval(v, t) * nurbs::eval::eval(a, t))
            .sum::<f64>()
            / speed
    }

    fn s_at(&self, t: f64) -> f64 {
        let t = t.clamp(self.t0, self.t1);
        let idx = self
            .grid
            .partition_point(|&g| g <= t)
            .saturating_sub(1)
            .min(self.grid.len() - 1);
        self.s_start
            + self.cumulative[idx]
            + integrate(&|u| self.shaped_speed(u), self.grid[idx], t)
    }

    fn s_end(&self) -> f64 {
        self.s_start + self.cumulative.last().copied().expect("cumulative seeded")
    }
}

impl TrackSignal for FollowerSignal<'_> {
    fn eval(&self, t: f64) -> f64 {
        let t = t.clamp(self.t0, self.t1);
        let spans = self.state.spans_e(self.s_at(t)) - self.e_spans_start;
        let raw = self
            .raw_delta
            .as_ref()
            .map_or(0.0, |(track, _, _, at_start)| {
                nurbs::eval::eval(track, t) - at_start
            });
        self.e_start + spans + raw
    }

    fn deriv(&self, t: f64) -> f64 {
        let t = t.clamp(self.t0, self.t1);
        let (ratio, _) = self.state.ratio_and_slope(self.s_at(t));
        let raw = self
            .raw_delta
            .as_ref()
            .map_or(0.0, |(_, d1, _, _)| nurbs::eval::eval(d1, t));
        ratio * self.shaped_speed(t) + raw
    }

    fn second_deriv(&self, t: f64) -> f64 {
        let t = t.clamp(self.t0, self.t1);
        let speed = self.shaped_speed(t);
        let (ratio, slope) = self.state.ratio_and_slope(self.s_at(t));
        let raw = self
            .raw_delta
            .as_ref()
            .map_or(0.0, |(_, _, d2, _)| nurbs::eval::eval(d2, t));
        slope * speed * speed + ratio * self.shaped_speed_deriv(t) + raw
    }
}

fn integrate(f: &impl Fn(f64) -> f64, a: f64, b: f64) -> f64 {
    if b - a <= 0.0 {
        return 0.0;
    }
    let m = 0.5 * (a + b);
    let (fa, fm, fb) = (f(a), f(m), f(b));
    let whole = (b - a) / 6.0 * (fa + 4.0 * fm + fb);
    adaptive_simpson(
        f,
        a,
        b,
        fa,
        fm,
        fb,
        whole,
        INTEGRAL_TOL_MM,
        INTEGRAL_MAX_DEPTH,
    )
}

#[allow(clippy::too_many_arguments)]
fn adaptive_simpson(
    f: &impl Fn(f64) -> f64,
    a: f64,
    b: f64,
    fa: f64,
    fm: f64,
    fb: f64,
    whole: f64,
    tol: f64,
    depth: u32,
) -> f64 {
    let m = 0.5 * (a + b);
    let (lm, rm) = (0.5 * (a + m), 0.5 * (m + b));
    let (flm, frm) = (f(lm), f(rm));
    let left = (m - a) / 6.0 * (fa + 4.0 * flm + fm);
    let right = (b - m) / 6.0 * (fm + 4.0 * frm + fb);
    let delta = left + right - whole;
    if depth == 0 || delta.abs() <= 15.0 * tol {
        return left + right + delta / 15.0;
    }
    adaptive_simpson(f, a, m, fa, flm, fm, left, 0.5 * tol, depth - 1)
        + adaptive_simpson(f, m, b, fm, frm, fb, right, 0.5 * tol, depth - 1)
}
