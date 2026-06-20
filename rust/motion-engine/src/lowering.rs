use geometry::path::lowering::PositionProfile;
use geometry::{Move, MoveVelocity, VelSample};
use nurbs::ScalarNurbs;
use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs};
use trajectory::ShapedSegment;

const MIN_PIECE_DURATION_S: f64 = 1e-9;
const MAX_SUBDIVISION_DEPTH: u32 = 22;

/// Minimum duration of an emitted cubic piece. The MCU consumes pieces against
/// a fixed step-sample clock (tens of µs); a piece shorter than several samples
/// makes the MCU evaluate one cubic across a sample boundary and extrapolate it
/// — corrupting the dispatched polynomial (Neptune steps-per-sample fault). The
/// velocity profile can carry thousands of samples per move, so we fit *coarse*
/// pieces to tolerance and never subdivide below this floor.
const MIN_FIT_PIECE_S: f64 = 5e-4;

/// Interior fractions sampled when testing a candidate cubic against the true
/// trajectory over a span; a span passes only if every sampled axis is within
/// `fit_tol_mm`.
const RESIDUAL_PROBES: [f64; 3] = [0.25, 0.5, 0.75];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoweringError {
    SourceMismatch,
    EmptyProfile,
    DegeneratePhase,
    FollowerAxisOutOfRange { axis_index: usize },
}

impl std::fmt::Display for LoweringError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::SourceMismatch => write!(f, "geometry and velocity move sources differ"),
            Self::EmptyProfile => write!(f, "velocity move has fewer than two samples"),
            Self::DegeneratePhase => write!(f, "velocity profile phase is degenerate"),
            Self::FollowerAxisOutOfRange { axis_index } => {
                write!(
                    f,
                    "follower axis {axis_index} exceeds the registry start vector"
                )
            }
        }
    }
}

impl std::error::Error for LoweringError {}

struct Phase {
    t0: f64,
    dt: f64,
    s0: f64,
    v0: f64,
    accel: f64,
    s_end: f64,
}

impl Phase {
    fn s_v_at(&self, t_local: f64) -> (f64, f64) {
        let tau = (t_local - self.t0).clamp(0.0, self.dt);
        let s = self
            .v0
            .mul_add(tau, 0.5 * self.accel * tau * tau + self.s0)
            .clamp(self.s0, self.s_end);
        let v = self.accel.mul_add(tau, self.v0).max(0.0);
        (s, v)
    }
}

fn build_phases(samples: &[VelSample]) -> Result<(Vec<Phase>, f64), LoweringError> {
    if samples.len() < 2 {
        return Err(LoweringError::EmptyProfile);
    }
    let mut phases = Vec::with_capacity(samples.len() - 1);
    let mut t_acc = 0.0;
    for w in samples.windows(2) {
        let (s0, v0) = (w[0].s, w[0].v);
        let (s1, v1) = (w[1].s, w[1].v);
        let ds = s1 - s0;
        let v_sum = v0 + v1;
        if !(ds.is_finite() && ds > 0.0 && v_sum > 0.0) {
            return Err(LoweringError::DegeneratePhase);
        }
        let accel = (v1 * v1 - v0 * v0) / (2.0 * ds);
        let dt = 2.0 * ds / v_sum;
        if !(dt.is_finite() && dt > 0.0) {
            return Err(LoweringError::DegeneratePhase);
        }
        phases.push(Phase {
            t0: t_acc,
            dt,
            s0,
            v0,
            accel,
            s_end: s1,
        });
        t_acc += dt;
    }
    Ok((phases, t_acc))
}

/// Power-basis cubic on `[u_start, u_end]` matching position and velocity at
/// both ends (Hermite). `coeffs[k]` multiplies `(u − u_start)^k` — the
/// representation `BezierPiece::evaluate` expects. A quadratic input (the line
/// case, where position is exactly quadratic in time) yields `coeffs[3] == 0`
/// and is reproduced exactly.
fn hermite_cubic(u_start: f64, u_end: f64, p0: f64, v0: f64, p1: f64, v1: f64) -> BezierPiece<f64> {
    let h = u_end - u_start;
    if h <= 0.0 {
        return BezierPiece {
            u_start,
            u_end,
            coeffs: vec![p0, 0.0, 0.0, 0.0],
        };
    }
    let a = p1 - p0 - v0 * h;
    let b = v1 - v0;
    let a2 = (3.0 * a - b * h) / (h * h);
    let a3 = (b * h - 2.0 * a) / (h * h * h);
    BezierPiece {
        u_start,
        u_end,
        coeffs: vec![p0, v0, a2, a3],
    }
}

struct Sampler<'a> {
    phases: &'a [Phase],
    spatial: Option<&'a geometry::path::Segment>,
    start_pos: &'a [f64],
    followers: &'a [geometry::FollowerDemand],
}

impl Sampler<'_> {
    /// Absolute (position, velocity) for one registry axis at move-local time
    /// `t`. Spatial axes ride the path; followers pay out `start + ratio·s`;
    /// every other axis holds at its start position.
    fn axis_state(&self, axis: usize, t: f64) -> (f64, f64) {
        let (s, v) = locate(self.phases, t).s_v_at(t);
        if axis < 3 {
            match self.spatial {
                Some(seg) => (seg.point_at(s)[axis], seg.heading_at(s)[axis] * v),
                None => (self.start_pos.get(axis).copied().unwrap_or(0.0), 0.0),
            }
        } else if let Some(f) = self.followers.iter().find(|f| f.axis_index == axis) {
            (f.ratio.mul_add(s, self.start_pos[axis]), f.ratio * v)
        } else {
            (self.start_pos.get(axis).copied().unwrap_or(0.0), 0.0)
        }
    }

    /// Worst-axis deviation of the candidate cubic over `[ta, tb]` from the true
    /// trajectory, sampled at the interior probes.
    fn span_residual(&self, driven: &[usize], ta: f64, tb: f64) -> f64 {
        let mut worst = 0.0_f64;
        for &axis in driven {
            let (pa, va) = self.axis_state(axis, ta);
            let (pb, vb) = self.axis_state(axis, tb);
            let piece = hermite_cubic(ta, tb, pa, va, pb, vb);
            for &frac in &RESIDUAL_PROBES {
                let tm = frac.mul_add(tb - ta, ta);
                let (truth, _) = self.axis_state(axis, tm);
                worst = worst.max((piece.evaluate(tm) - truth).abs());
            }
        }
        worst
    }
}

fn locate(phases: &[Phase], t: f64) -> &Phase {
    phases
        .iter()
        .find(|p| t < p.t0 + p.dt)
        .unwrap_or(&phases[phases.len() - 1])
}

/// Adaptively split `[ta, tb]` into spans each fittable by a single cubic within
/// `tol_mm`, never going below `MIN_FIT_PIECE_S`. Appends the right edge of each
/// accepted span to `out`. This fits *coarse* pieces that span many velocity
/// samples — the MCU step clock cannot consume sub-sample pieces.
fn refine_span(
    sampler: &Sampler<'_>,
    driven: &[usize],
    tol_mm: f64,
    ta: f64,
    tb: f64,
    depth: u32,
    out: &mut Vec<f64>,
) {
    let h = tb - ta;
    let accept = depth >= MAX_SUBDIVISION_DEPTH
        || h <= 2.0 * MIN_FIT_PIECE_S
        || sampler.span_residual(driven, ta, tb) <= tol_mm;
    if accept {
        out.push(tb);
    } else {
        let tm = 0.5 * (ta + tb);
        refine_span(sampler, driven, tol_mm, ta, tm, depth + 1, out);
        refine_span(sampler, driven, tol_mm, tm, tb, depth + 1, out);
    }
}

/// Lower a single planned move into a per-axis position-vs-time [`ShapedSegment`]
/// in the planner's absolute time domain, starting at `t_start` and from the
/// absolute registry positions `start_pos` (index = registry axis: 0..3
/// spatial, then followers). The result feeds `enqueue_segment` unchanged. The
/// trajectory is C1 (velocity-continuous across phases; acceleration may step),
/// matching the velocity planner's own continuity.
pub fn lower_move(
    gm: &Move,
    vm: &MoveVelocity,
    t_start: f64,
    start_pos: &[f64],
    fit_tol_mm: f64,
) -> Result<ShapedSegment, LoweringError> {
    if gm.source != vm.source {
        return Err(LoweringError::SourceMismatch);
    }
    let (phases, total_t) = build_phases(&vm.samples)?;
    let spatial = gm.segment.spatial.as_ref();

    let n_axes = start_pos.len().max(3);
    for f in &gm.segment.followers {
        if f.axis_index >= n_axes {
            return Err(LoweringError::FollowerAxisOutOfRange {
                axis_index: f.axis_index,
            });
        }
    }

    let sampler = Sampler {
        phases: &phases,
        spatial,
        start_pos,
        followers: &gm.segment.followers,
    };
    let mut driven: Vec<usize> = (0..3).collect();
    driven.extend(gm.segment.followers.iter().map(|f| f.axis_index));

    // Coarse time grid shared by every axis: fit cubic pieces to tolerance over
    // the whole move (spanning many velocity samples), never below the
    // minimum-duration floor. One piece per velocity sample would emit thousands
    // of sub-sample pieces the MCU cannot consume.
    let mut bounds = vec![0.0];
    refine_span(&sampler, &driven, fit_tol_mm, 0.0, total_t, 0, &mut bounds);

    let mut axes_pieces: Vec<Vec<BezierPiece<f64>>> = vec![Vec::new(); n_axes];
    for w in bounds.windows(2) {
        let (ta, tb) = (w[0], w[1]);
        if tb - ta <= MIN_PIECE_DURATION_S {
            continue;
        }
        let (ua, ub) = (t_start + ta, t_start + tb);
        for (axis, pieces) in axes_pieces.iter_mut().enumerate() {
            let (pa, va) = sampler.axis_state(axis, ta);
            let (pb, vb) = sampler.axis_state(axis, tb);
            pieces.push(hermite_cubic(ua, ub, pa, va, pb, vb));
        }
    }

    let axes: Vec<ScalarNurbs<f64>> = axes_pieces
        .iter()
        .map(|p| bezier_pieces_to_nurbs(p))
        .collect();

    Ok(ShapedSegment {
        axes,
        followers: gm.segment.followers.clone(),
        t_start,
        t_end: t_start + total_t,
        motor_mask: 0,
    })
}

#[cfg(test)]
mod tests;
