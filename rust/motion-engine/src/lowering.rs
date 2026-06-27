use geometry::path::lowering::PositionProfile;
use geometry::{Move, MoveVelocity, VelSample};
use nurbs::ScalarNurbs;
use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs};
use trajectory::{ChainStage, CompiledChain, ShapedSegment};

const MIN_PIECE_DURATION_S: f64 = 1e-9;
const MAX_SUBDIVISION_DEPTH: u32 = 22;

const MIN_FIT_PIECE_S: f64 = 5e-4;

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

/// Cubic Hermite piece in monomial form matching position and velocity at both
/// endpoints. Used by the input-shaping post-processor, where only the
/// convolution-smoothed position and its derivative are available.
pub(crate) fn hermite_cubic(
    u_start: f64,
    u_end: f64,
    p0: f64,
    v0: f64,
    p1: f64,
    v1: f64,
) -> BezierPiece<f64> {
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
    axis_chains: &'a [CompiledChain],
}

impl Sampler<'_> {
    fn axis_base_state(&self, axis: usize, t: f64) -> (f64, f64, f64) {
        let phase = locate(self.phases, t);
        let (s, v) = phase.s_v_at(t);
        if axis < 3 {
            match self.spatial {
                Some(seg) => {
                    // Exact per-axis acceleration `a_t·ĥ + v²·(dĥ/ds)`: the planner's
                    // tangential accel `a_t = phase.accel` along the heading, plus the
                    // centripetal term from the path curving. No finite difference, so
                    // it stays smooth where a difference of headings would jitter.
                    let accel =
                        phase.accel * seg.heading_at(s)[axis] + v * v * seg.dheading_ds(s)[axis];
                    (seg.point_at(s)[axis], seg.heading_at(s)[axis] * v, accel)
                }
                None => (self.start_pos.get(axis).copied().unwrap_or(0.0), 0.0, 0.0),
            }
        } else if let Some(f) = self.followers.iter().find(|f| f.axis_index == axis) {
            let pos = f.ratio.mul_add(s, self.start_pos[axis]);
            let e_dot = f.ratio * v;
            let e_ddot = f.ratio * phase.accel;
            (pos, e_dot, e_ddot)
        } else {
            (self.start_pos.get(axis).copied().unwrap_or(0.0), 0.0, 0.0)
        }
    }

    fn axis_state(&self, axis: usize, t: f64, apply_zero_support: bool) -> (f64, f64) {
        let (mut pos, mut vel, mut accel) = self.axis_base_state(axis, t);
        if apply_zero_support {
            if let Some(chain) = self.axis_chains.get(axis) {
                for stage in &chain.stages {
                    match stage {
                        ChainStage::LinearPressureAdvance { k } => {
                            pos = k.mul_add(vel, pos);
                            vel = k.mul_add(accel, vel);
                            accel = 0.0;
                        }
                        ChainStage::SmoothKernel(_) => break,
                    }
                }
            }
        }
        (pos, vel)
    }

    fn span_residual(&self, driven: &[usize], ta: f64, tb: f64) -> f64 {
        let mut worst = 0.0_f64;
        for &axis in driven {
            let (pa, va) = self.axis_state(axis, ta, false);
            let (pb, vb) = self.axis_state(axis, tb, false);
            let piece = hermite_cubic(ta, tb, pa, va, pb, vb);
            for &frac in &RESIDUAL_PROBES {
                let tm = frac.mul_add(tb - ta, ta);
                let (truth, _) = self.axis_state(axis, tm, false);
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

pub fn lower_move(
    gm: &Move,
    vm: &MoveVelocity,
    t_start: f64,
    start_pos: &[f64],
    fit_tol_mm: f64,
    axis_chains: &[CompiledChain],
) -> Result<ShapedSegment, LoweringError> {
    let (axes_pieces, total_t) =
        lower_move_pieces(gm, vm, t_start, start_pos, fit_tol_mm, axis_chains)?;
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
        source_line: 0,
    })
}

/// Lower a move to per-axis cubic Bézier pieces in monomial form
/// (`pos = c0 + c1·τ + c2·τ² + c3·τ³`, `τ = t − u_start`) — the trajectory the
/// firmware executes, before it is packed into NURBS. Returns the per-axis pieces
/// and the move duration.
pub fn lower_move_pieces(
    gm: &Move,
    vm: &MoveVelocity,
    t_start: f64,
    start_pos: &[f64],
    fit_tol_mm: f64,
    axis_chains: &[CompiledChain],
) -> Result<(Vec<Vec<BezierPiece<f64>>>, f64), LoweringError> {
    if gm.source != vm.source {
        return Err(LoweringError::SourceMismatch);
    }
    if !vm.phases.is_empty() {
        return lower_straight_from_phases(gm, vm, t_start, start_pos, axis_chains);
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
        axis_chains,
    };
    let mut driven: Vec<usize> = (0..3).collect();
    driven.extend(gm.segment.followers.iter().map(|f| f.axis_index));

    let mut coarse_fit_grid = vec![0.0];
    refine_span(
        &sampler,
        &driven,
        fit_tol_mm,
        0.0,
        total_t,
        0,
        &mut coarse_fit_grid,
    );
    let bounds = coarse_fit_grid;

    let mut axes_pieces: Vec<Vec<BezierPiece<f64>>> = vec![Vec::new(); n_axes];
    for w in bounds.windows(2) {
        let (ta, tb) = (w[0], w[1]);
        if tb - ta <= MIN_PIECE_DURATION_S {
            continue;
        }
        let (ua, ub) = (t_start + ta, t_start + tb);
        for (axis, pieces) in axes_pieces.iter_mut().enumerate() {
            let (pa, va) = sampler.axis_state(axis, ta, true);
            let (pb, vb) = sampler.axis_state(axis, tb, true);
            pieces.push(hermite_cubic(ua, ub, pa, va, pb, vb));
        }
    }

    Ok((axes_pieces, total_t))
}

/// Linear pressure advance is `pos += k * vel`, exact on a cubic: it maps the
/// monomial coefficients onto another cubic (`SmoothKernel` is a downstream
/// convolution, so it stops the per-piece transform exactly as the sampled path
/// does). Mirrors the `ChainStage` semantics in [`Sampler::axis_state`].
fn apply_axis_chain(coeffs: &mut [f64; 4], chain: &CompiledChain) {
    for stage in &chain.stages {
        match stage {
            ChainStage::LinearPressureAdvance { k } => {
                let [c0, c1, c2, c3] = *coeffs;
                *coeffs = [
                    k.mul_add(c1, c0),
                    k.mul_add(2.0 * c2, c1),
                    k.mul_add(3.0 * c3, c2),
                    c3,
                ];
            }
            ChainStage::SmoothKernel(_) => break,
        }
    }
}

/// Lower a straight constant-ceiling move from its closed-form jerk phases: one
/// exact cubic per phase. Each axis is the arc-length profile scaled by its fixed
/// share of the motion (a spatial axis by the heading, a follower by its ratio),
/// so position, velocity, acceleration, and jerk are all exact — no grid fitting,
/// no acceleration overshoot, and the traversal time is the profile's own.
fn lower_straight_from_phases(
    gm: &Move,
    vm: &MoveVelocity,
    t_start: f64,
    start_pos: &[f64],
    axis_chains: &[CompiledChain],
) -> Result<(Vec<Vec<BezierPiece<f64>>>, f64), LoweringError> {
    let n_axes = start_pos.len().max(3);
    for f in &gm.segment.followers {
        if f.axis_index >= n_axes {
            return Err(LoweringError::FollowerAxisOutOfRange {
                axis_index: f.axis_index,
            });
        }
    }
    let spatial = gm
        .segment
        .spatial
        .as_ref()
        .map(|seg| (seg.point_at(0.0), seg.heading_at(0.0)));

    let axis_scale_base = |axis: usize| -> (f64, f64) {
        if axis < 3 {
            match spatial {
                Some((origin, heading)) => (heading[axis], origin[axis]),
                None => (0.0, start_pos.get(axis).copied().unwrap_or(0.0)),
            }
        } else if let Some(f) = gm.segment.followers.iter().find(|f| f.axis_index == axis) {
            (f.ratio, start_pos[axis])
        } else {
            (0.0, start_pos.get(axis).copied().unwrap_or(0.0))
        }
    };

    // One shared cumulative-time bounds array, exactly as the grid path: every
    // axis reads the same breakpoint float, so consecutive pieces are bit-exactly
    // contiguous (`u_end == u_start`) and `t_start + total_t` is the final bound —
    // the contiguity the NURBS assembler asserts, within a move and across seams.
    let mut bounds = Vec::with_capacity(vm.phases.len() + 1);
    bounds.push(0.0);
    for p in &vm.phases {
        bounds.push(bounds.last().unwrap() + p.dt);
    }
    let total_t = *bounds.last().unwrap();

    let mut axes_pieces: Vec<Vec<BezierPiece<f64>>> = vec![Vec::new(); n_axes];
    for (axis, pieces) in axes_pieces.iter_mut().enumerate() {
        let (scale, base) = axis_scale_base(axis);
        for (i, p) in vm.phases.iter().enumerate() {
            let mut coeffs = [
                scale.mul_add(p.s0, base),
                scale * p.v0,
                scale * 0.5 * p.a0,
                scale * p.j / 6.0,
            ];
            if let Some(chain) = axis_chains.get(axis) {
                apply_axis_chain(&mut coeffs, chain);
            }
            pieces.push(BezierPiece {
                u_start: t_start + bounds[i],
                u_end: t_start + bounds[i + 1],
                coeffs: coeffs.to_vec(),
            });
        }
    }

    Ok((axes_pieces, total_t))
}

#[cfg(test)]
mod tests;
