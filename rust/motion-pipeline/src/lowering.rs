use geometry::path::lowering::PositionProfile;
use geometry::{Move, MoveVelocity, VelSample};
use nurbs::ScalarNurbs;
use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs};
use trajectory::{ChainStage, CompiledChain, ShapedSegment};

const MIN_PIECE_DURATION_S: f64 = 1e-9;
const MAX_SUBDIVISION_DEPTH: u32 = 22;

const MIN_FIT_PIECE_S: f64 = 1e-4;

const RESIDUAL_PROBES: [f64; 3] = [0.25, 0.5, 0.75];

/// Absolute acceleration-error budget for a fitted piece, probed at the knots
/// as well as the interior. The positional weighting alone (`accel_err·h²/8`)
/// lets a short piece end with an acceleration hundreds of mm/s² off the
/// profile — positionally invisible, but adjacent pieces then step by twice
/// that error at their shared knot, and the dispatched trajectory carries a
/// jerk spike the planner never asked for.
const FIT_TOL_ACCEL_MM_S2: f64 = 50.0;

const ACCEL_PROBES: [f64; 5] = [0.0, 0.25, 0.5, 0.75, 1.0];

const PROFILE_OVERSHOOT_EPS: f64 = 1e-6;
const PROFILE_VELOCITY_FLOOR: f64 = 1e-9;

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

/// Quintic Hermite piece for `s(t)` over one sample window, in local time `τ`. It
/// matches `(s, v, a)` at both knots, so adjacent windows — which share those knot
/// values — join C2, and the whole move's `s(t)` is C2 by construction.
struct QuinticWindow {
    dt: f64,
    coeffs: [f64; 6],
    s0: f64,
    s1: f64,
}

impl QuinticWindow {
    fn state_at(&self, tau: f64) -> (f64, f64, f64) {
        let c = &self.coeffs;
        let s = (((((c[5] * tau) + c[4]) * tau + c[3]) * tau + c[2]) * tau + c[1]) * tau + c[0];
        let v = ((((5.0 * c[5] * tau) + 4.0 * c[4]) * tau + 3.0 * c[3]) * tau + 2.0 * c[2]) * tau
            + c[1];
        let a = ((20.0 * c[5] * tau) + 12.0 * c[4]) * tau * tau + 6.0 * c[3] * tau + 2.0 * c[2];
        (s, v, a)
    }
}

/// Monomial coefficients `c0..c5` of the quintic matching `(s0, v0, a0)` at `τ = 0`
/// and `(s1, v1, a1)` at `τ = h`.
fn quintic_hermite_coeffs(
    s0: f64,
    v0: f64,
    a0: f64,
    s1: f64,
    v1: f64,
    a1: f64,
    h: f64,
) -> [f64; 6] {
    let ds = s1 - s0;
    let h2 = h * h;
    let h3 = h2 * h;
    let c3 = (20.0 * ds - (8.0 * v1 + 12.0 * v0) * h - (3.0 * a0 - a1) * h2) / (2.0 * h3);
    let c4 =
        (-30.0 * ds + (14.0 * v1 + 16.0 * v0) * h + (3.0 * a0 - 2.0 * a1) * h2) / (2.0 * h3 * h);
    let c5 = (12.0 * ds - 6.0 * (v1 + v0) * h - (a0 - a1) * h2) / (2.0 * h3 * h2);
    [s0, v0, 0.5 * a0, c3, c4, c5]
}

/// C2 scalar arc-length profile `s(t)` for a curved move, one quintic Hermite per
/// sample window over the dense `(s, v, a)` grid.
struct ScalarProfile {
    windows: Vec<QuinticWindow>,
    knot_t: Vec<f64>,
}

impl ScalarProfile {
    fn locate(&self, t: f64) -> (usize, f64) {
        let count = self.knot_t.partition_point(|&kt| kt <= t);
        let idx = count.saturating_sub(1).min(self.windows.len() - 1);
        let tau = (t - self.knot_t[idx]).clamp(0.0, self.windows[idx].dt);
        (idx, tau)
    }

    fn state_at(&self, t: f64) -> (f64, f64, f64) {
        let (idx, tau) = self.locate(t);
        let w = &self.windows[idx];
        let (s, v, a) = w.state_at(tau);
        let margin = PROFILE_OVERSHOOT_EPS * (1.0 + (w.s1 - w.s0).abs());
        debug_assert!(
            s >= w.s0 - margin && s <= w.s1 + margin,
            "quintic profile position {s} escaped window [{}, {}]",
            w.s0,
            w.s1
        );
        debug_assert!(
            v >= -PROFILE_VELOCITY_FLOOR,
            "quintic profile velocity {v} below zero"
        );
        (s, v, a)
    }
}

fn build_profile(samples: &[VelSample]) -> Result<(ScalarProfile, f64), LoweringError> {
    if samples.len() < 2 {
        return Err(LoweringError::EmptyProfile);
    }
    let mut windows = Vec::with_capacity(samples.len() - 1);
    let mut knot_t = Vec::with_capacity(samples.len());
    knot_t.push(0.0);
    let mut t_acc = 0.0;
    for w in samples.windows(2) {
        let (s0, v0, a0) = (w[0].s, w[0].v, w[0].a);
        let (s1, v1, a1) = (w[1].s, w[1].v, w[1].a);
        let ds = s1 - s0;
        let v_sum = v0 + v1;
        let finite = v0.is_finite() && v1.is_finite() && a0.is_finite() && a1.is_finite();
        if !(ds.is_finite() && ds > 0.0 && v_sum > 0.0 && finite) {
            return Err(LoweringError::DegeneratePhase);
        }
        let dt = 2.0 * ds / v_sum;
        if !(dt.is_finite() && dt > 0.0) {
            return Err(LoweringError::DegeneratePhase);
        }
        windows.push(QuinticWindow {
            dt,
            coeffs: quintic_hermite_coeffs(s0, v0, a0, s1, v1, a1, dt),
            s0,
            s1,
        });
        t_acc += dt;
        knot_t.push(t_acc);
    }
    Ok((ScalarProfile { windows, knot_t }, t_acc))
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
    profile: &'a ScalarProfile,
    spatial: Option<&'a geometry::path::Segment>,
    start_pos: &'a [f64],
    followers: &'a [geometry::FollowerDemand],
    s_len: f64,
    axis_chains: &'a [CompiledChain],
}

impl Sampler<'_> {
    fn axis_base_state(&self, axis: usize, t: f64) -> (f64, f64, f64) {
        let (s, v, a_t) = self.profile.state_at(t);
        if axis < 3 {
            match self.spatial {
                Some(seg) => {
                    let accel = a_t * seg.heading_at(s)[axis] + v * v * seg.dheading_ds(s)[axis];
                    (seg.point_at(s)[axis], seg.heading_at(s)[axis] * v, accel)
                }
                None => (self.start_pos.get(axis).copied().unwrap_or(0.0), 0.0, 0.0),
            }
        } else if let Some(f) = self.followers.iter().find(|f| f.axis_index == axis) {
            if f.is_ramped() {
                let len = self.s_len;
                let r = f.ratio_at(s, len);
                let pos = self.start_pos[axis] + f.offset_at(s, len);
                let accel = f.ratio_slope(len).mul_add(v * v, r * a_t);
                (pos, r * v, accel)
            } else {
                let pos = f.ratio.mul_add(s, self.start_pos[axis]);
                (pos, f.ratio * v, f.ratio * a_t)
            }
        } else {
            (self.start_pos.get(axis).copied().unwrap_or(0.0), 0.0, 0.0)
        }
    }

    fn axis_state_full(&self, axis: usize, t: f64, apply_zero_support: bool) -> (f64, f64, f64) {
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
        (pos, vel, accel)
    }

    fn axis_state(&self, axis: usize, t: f64, apply_zero_support: bool) -> (f64, f64) {
        let (pos, vel, _) = self.axis_state_full(axis, t, apply_zero_support);
        (pos, vel)
    }

    fn axis_accel(&self, axis: usize, t: f64, apply_zero_support: bool) -> f64 {
        self.axis_state_full(axis, t, apply_zero_support).2
    }

    fn span_fits(&self, driven: &[usize], ta: f64, tb: f64, tol_mm: f64) -> bool {
        let h = tb - ta;
        let accel_scale = h * h / 8.0;
        for &axis in driven {
            let (pa, va) = self.axis_state(axis, ta, false);
            let (pb, vb) = self.axis_state(axis, tb, false);
            let piece = hermite_cubic(ta, tb, pa, va, pb, vb);
            let (c2, c3) = (piece.coeffs[2], piece.coeffs[3]);
            for &frac in &ACCEL_PROBES {
                let tm = frac.mul_add(h, ta);
                let accel_fit = 6.0_f64.mul_add(c3 * (tm - ta), 2.0 * c2);
                let accel_err = (accel_fit - self.axis_accel(axis, tm, false)).abs();
                if accel_err > FIT_TOL_ACCEL_MM_S2 || accel_err * accel_scale > tol_mm {
                    return false;
                }
            }
            for &frac in &RESIDUAL_PROBES {
                let tm = frac.mul_add(h, ta);
                let (truth, _) = self.axis_state(axis, tm, false);
                if (piece.evaluate(tm) - truth).abs() > tol_mm {
                    return false;
                }
            }
        }
        true
    }
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
        || sampler.span_fits(driven, ta, tb, tol_mm);
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
    // The closed-form phase path expresses each axis as one constant scale times
    // the arc-length profile; a ramped follower's ratio varies along the move, so
    // route those through the sampled fit instead. Constant followers (every
    // straight slicer move) keep the exact phase path.
    let ramped = gm.segment.followers.iter().any(|f| f.is_ramped());
    if !vm.phases.is_empty() && !ramped {
        return lower_straight_from_phases(gm, vm, t_start, start_pos, axis_chains);
    }
    let (profile, total_t) = build_profile(&vm.samples)?;
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
        profile: &profile,
        spatial,
        start_pos,
        followers: &gm.segment.followers,
        s_len: gm.segment.s_len(),
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
fn apply_pressure_advance(coeffs: &mut [f64; 4], chain: &CompiledChain) {
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
                apply_pressure_advance(&mut coeffs, chain);
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
