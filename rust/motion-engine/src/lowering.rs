use geometry::path::lowering::PositionProfile;
use geometry::{Move, MoveVelocity, VelSample};
use nurbs::ScalarNurbs;
use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs};
use trajectory::ShapedSegment;

const MIN_PIECE_DURATION_S: f64 = 1e-9;
const MAX_SUBDIVISION_DEPTH: u32 = 16;

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

fn refine_bounds(
    phase: &Phase,
    spatial: Option<&geometry::path::Segment>,
    tol_mm: f64,
    ta: f64,
    tb: f64,
    depth: u32,
    out: &mut Vec<f64>,
) {
    let h = tb - ta;
    let split = depth < MAX_SUBDIVISION_DEPTH
        && h > 2.0 * MIN_PIECE_DURATION_S
        && spatial.is_some_and(|seg| hermite_residual(phase, seg, ta, tb) > tol_mm);
    if split {
        let tm = 0.5 * (ta + tb);
        refine_bounds(phase, spatial, tol_mm, ta, tm, depth + 1, out);
        refine_bounds(phase, spatial, tol_mm, tm, tb, depth + 1, out);
    } else {
        out.push(tb);
    }
}

fn hermite_residual(phase: &Phase, seg: &geometry::path::Segment, ta: f64, tb: f64) -> f64 {
    let (sa, va) = phase.s_v_at(ta);
    let (sb, vb) = phase.s_v_at(tb);
    let tm = 0.5 * (ta + tb);
    let (sm, _) = phase.s_v_at(tm);
    let pa = seg.point_at(sa);
    let ha = seg.heading_at(sa);
    let pb = seg.point_at(sb);
    let hb = seg.heading_at(sb);
    let truth = seg.point_at(sm);
    (0..3)
        .map(|axis| {
            let piece = hermite_cubic(ta, tb, pa[axis], ha[axis] * va, pb[axis], hb[axis] * vb);
            (piece.evaluate(tm) - truth[axis]).abs()
        })
        .fold(0.0_f64, f64::max)
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

    let mut axes_pieces: Vec<Vec<BezierPiece<f64>>> = vec![Vec::new(); n_axes];
    let mut driven = vec![false; n_axes];
    for axis in 0..3 {
        driven[axis] = true;
    }
    for f in &gm.segment.followers {
        driven[f.axis_index] = true;
    }

    for phase in &phases {
        let mut bounds = vec![phase.t0];
        refine_bounds(
            phase,
            spatial,
            fit_tol_mm,
            phase.t0,
            phase.t0 + phase.dt,
            0,
            &mut bounds,
        );
        for w in bounds.windows(2) {
            let (ta, tb) = (w[0], w[1]);
            if tb - ta <= MIN_PIECE_DURATION_S {
                continue;
            }
            let (sa, va) = phase.s_v_at(ta);
            let (sb, vb) = phase.s_v_at(tb);
            let (ua, ub) = (t_start + ta, t_start + tb);

            for axis in 0..3 {
                let piece = match spatial {
                    Some(seg) => {
                        let pa = seg.point_at(sa);
                        let ha = seg.heading_at(sa);
                        let pb = seg.point_at(sb);
                        let hb = seg.heading_at(sb);
                        hermite_cubic(ua, ub, pa[axis], ha[axis] * va, pb[axis], hb[axis] * vb)
                    }
                    None => {
                        let hold = start_pos.get(axis).copied().unwrap_or(0.0);
                        hermite_cubic(ua, ub, hold, 0.0, hold, 0.0)
                    }
                };
                axes_pieces[axis].push(piece);
            }

            for f in &gm.segment.followers {
                let base = start_pos[f.axis_index];
                let piece = hermite_cubic(
                    ua,
                    ub,
                    f.ratio.mul_add(sa, base),
                    f.ratio * va,
                    f.ratio.mul_add(sb, base),
                    f.ratio * vb,
                );
                axes_pieces[f.axis_index].push(piece);
            }
        }
    }

    for axis in 0..n_axes {
        if !driven[axis] {
            let hold = start_pos[axis];
            axes_pieces[axis].push(hermite_cubic(
                t_start,
                t_start + total_t,
                hold,
                0.0,
                hold,
                0.0,
            ));
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
