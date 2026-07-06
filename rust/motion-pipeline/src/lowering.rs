use geometry::path::lowering::PositionProfile;
use geometry::{Move, MoveVelocity, VelSample};
use nurbs::ScalarNurbs;
use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs};
use nurbs::chebyshev::{
    chebyshev_to_monomial_tau, monomial_u_to_chebyshev, taylor_shift,
    truncate_chebyshev_c2_anchored,
};
use trajectory::{ChainStage, CompiledChain, ShapedSegment};

/// Duplicated from `runtime::piece_ring::MAX_PIECE_COEFFS` (this crate must
/// not depend on the MCU runtime); equality is enforced by the cross-crate
/// const test in motion-engine.
pub const MAX_PIECE_COEFFS: usize = 8;

const MIN_PIECE_DURATION_S: f64 = 1e-9;
const MAX_SUBDIVISION_DEPTH: u32 = 22;

const MIN_FIT_PIECE_S: f64 = 1e-4;

/// Fit acceptance budgets for one lowered piece, probed at the interior
/// Chebyshev nodes (endpoints are matched exactly by construction).
#[derive(Debug, Clone, Copy)]
pub struct FitTol {
    pub pos_mm: f64,
    /// Absolute acceleration-error budget for a fitted piece. The positional
    /// weighting alone (`accel_err·h²/8`) lets a short piece end with an
    /// acceleration hundreds of mm/s² off the profile — positionally
    /// invisible, but adjacent pieces then step by twice that error at their
    /// shared knot, and the dispatched trajectory carries a jerk spike the
    /// planner never asked for. Only accel-feedforward consumers (EtherCAT
    /// servos) can tell; steppers just follow positions.
    pub accel_mm_s2: f64,
}

/// Interior probe nodes `cos(kπ/8)` on u ∈ [−1, 1] — endpoints are matched
/// exactly by construction, so all probing is interior.
pub(crate) const LADDER_PROBES_U: [f64; 7] = [
    -0.923_879_532_511_286_7,
    -std::f64::consts::FRAC_1_SQRT_2,
    -0.382_683_432_365_089_8,
    0.0,
    0.382_683_432_365_089_8,
    std::f64::consts::FRAC_1_SQRT_2,
    0.923_879_532_511_286_7,
];

/// `(1 − u²)³` — triple zeros at ±1, so adding it preserves endpoint p/v/a.
const BUMP6: [f64; 7] = [1.0, 0.0, -3.0, 0.0, 3.0, 0.0, -1.0];
/// `u·(1 − u²)³`.
const BUMP7: [f64; 8] = [0.0, 1.0, 0.0, -3.0, 0.0, 3.0, 0.0, -1.0];

/// Post-fit Chebyshev truncation budgets: position at a tenth of the fit
/// tolerance, endpoint velocity/acceleration bounded so collapsing a piece can
/// never step the seam feedforward.
pub(crate) const FIT_TRUNC_POS_FACTOR: f64 = 0.1;
pub(crate) const FIT_TRUNC_VEL_MM_S: f64 = 0.05;
pub(crate) const FIT_TRUNC_ACC_MM_S2: f64 = 0.25;

pub(crate) fn eval_mono(c: &[f64], x: f64) -> f64 {
    c.iter().rev().fold(0.0, |acc, &ck| acc * x + ck)
}

pub(crate) fn eval_mono_dd(c: &[f64], x: f64) -> f64 {
    let mut acc = 0.0;
    for (k, &ck) in c.iter().enumerate().skip(2).rev() {
        acc = acc * x + (k * (k - 1)) as f64 * ck;
    }
    acc
}

/// Monomial-in-u quintic matching `(p, v, a)` — time-domain derivatives — at
/// both ends of a span of duration `h`. Fitting in u keeps the coefficients
/// O(piece amplitude): the conditioning win over monomial-τ.
pub(crate) fn quintic_in_u(sa: (f64, f64, f64), sb: (f64, f64, f64), h: f64) -> Vec<f64> {
    let s = 0.5 * h;
    let q = quintic_hermite_coeffs(
        sa.0,
        sa.1 * s,
        sa.2 * s * s,
        sb.0,
        sb.1 * s,
        sb.2 * s * s,
        2.0,
    );
    taylor_shift(&q, 1.0)
}

/// Degree-`degree` ladder candidate: the quintic base plus `(1−u²)³`-shaped
/// corrections whose coefficients come from interior residuals (u = 0 for
/// degree 6; u = ±½ with exact 27/64 denominators for degree 7).
pub(crate) fn ladder_candidate(
    base: &[f64],
    degree: usize,
    truth_p: &dyn Fn(f64) -> f64,
) -> Vec<f64> {
    let mut c = base.to_vec();
    match degree {
        5 => {}
        6 => {
            let r0 = truth_p(0.0) - eval_mono(base, 0.0);
            c.resize(7, 0.0);
            for (ci, &w) in c.iter_mut().zip(&BUMP6) {
                *ci += r0 * w;
            }
        }
        7 => {
            let rp = truth_p(0.5) - eval_mono(base, 0.5);
            let rm = truth_p(-0.5) - eval_mono(base, -0.5);
            let q0 = (rp + rm) * (32.0 / 27.0);
            let q1 = (rp - rm) * (64.0 / 27.0);
            c.resize(8, 0.0);
            for (ci, &w) in c.iter_mut().zip(&BUMP6) {
                *ci += q0 * w;
            }
            for (ci, &w) in c.iter_mut().zip(&BUMP7) {
                *ci += q1 * w;
            }
        }
        _ => panic!("ladder degree {degree} outside 5..=7"),
    }
    c
}

fn candidate_ok(
    mono_u: &[f64],
    h: f64,
    tol: FitTol,
    truth_p: &dyn Fn(f64) -> f64,
    truth_a: &dyn Fn(f64) -> f64,
) -> bool {
    let dd_scale = (2.0 / h) * (2.0 / h);
    LADDER_PROBES_U.iter().all(|&u| {
        (eval_mono(mono_u, u) - truth_p(u)).abs() <= tol.pos_mm
            && (eval_mono_dd(mono_u, u) * dd_scale - truth_a(u)).abs() <= tol.accel_mm_s2
    })
}

/// Endpoint acceleration reads the wire's f32 coefficients with weight
/// `k²(k²−1)/3 · (2/h)²` — on a short piece a degree-6/7 coefficient's f32
/// rounding alone steps the seam accel by tens of mm/s². Below this span the
/// ladder stops at the quintic (whose position error already scales as h⁶).
pub(crate) const MIN_HIGH_DEGREE_SPAN_S: f64 = 5e-4;

pub(crate) fn ladder_degrees(h: f64) -> &'static [usize] {
    if h < MIN_HIGH_DEGREE_SPAN_S {
        &[5]
    } else {
        &[5, 6, 7]
    }
}

/// First ladder degree (5 → 6 → 7, span-capped) whose interior position and
/// acceleration residuals pass; `None` asks the caller to bisect.
pub(crate) fn ladder_fit(
    base: &[f64],
    h: f64,
    tol: FitTol,
    truth_p: &dyn Fn(f64) -> f64,
    truth_a: &dyn Fn(f64) -> f64,
) -> Option<Vec<f64>> {
    ladder_degrees(h).iter().find_map(|&degree| {
        let c = ladder_candidate(base, degree, truth_p);
        candidate_ok(&c, h, tol, truth_p, truth_a).then_some(c)
    })
}

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

/// One sample window's duration: the trapezoid estimate `2·ds/(v0+v1)`,
/// Newton-refined against the constant-jerk kinematics through the window's
/// own endpoint states. The trapezoid value alone is off by O(ds³), and the
/// quintic window pinned to six endpoint constraints swallows that duration
/// error as interior wiggle whose acceleration amplitude grows as 1/h² — on
/// short low-speed windows it exceeds the fit budget and the piece fit chases
/// phantom jerk. The refined duration is consistent with the endpoint states,
/// so the interior stays as smooth as the plan itself.
fn window_duration(ds: f64, v0: f64, v1: f64, a0: f64, a1: f64) -> f64 {
    let trapezoid = 2.0 * ds / (v0 + v1);
    let mut dt = trapezoid;
    for _ in 0..3 {
        let j = (a1 - a0) / dt;
        let residual = dt * (v0 + dt * (0.5 * a0 + dt * j / 6.0)) - ds;
        let slope = v0 + dt * (a0 + dt * 0.5 * j);
        let next = dt - residual / slope;
        if !(next.is_finite() && next > 0.0) {
            return trapezoid;
        }
        dt = next;
    }
    if (dt - trapezoid).abs() > 0.5 * trapezoid {
        return trapezoid;
    }
    dt
}

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
        let dt = window_duration(ds, v0, v1, a0, a1);
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

    fn span_fits(&self, driven: &[usize], ta: f64, tb: f64, tol: FitTol) -> bool {
        driven
            .iter()
            .all(|&axis| self.ladder_fit_axis(axis, ta, tb, tol, false).is_some())
    }

    fn ladder_fit_axis(
        &self,
        axis: usize,
        ta: f64,
        tb: f64,
        tol: FitTol,
        apply_zero_support: bool,
    ) -> Option<Vec<f64>> {
        let h = tb - ta;
        let sa = self.axis_state_full(axis, ta, apply_zero_support);
        let sb = self.axis_state_full(axis, tb, apply_zero_support);
        let base = quintic_in_u(sa, sb, h);
        let t_of = |u: f64| (0.5 * (u + 1.0)).mul_add(h, ta);
        let truth_p = |u: f64| self.axis_state(axis, t_of(u), apply_zero_support).0;
        let truth_a = |u: f64| self.axis_accel(axis, t_of(u), apply_zero_support);
        ladder_fit(&base, h, tol, &truth_p, &truth_a)
    }

    /// Fitted output piece for one accepted span: ladder fit on the
    /// chain-transformed signal, Chebyshev truncation to the true degree, back
    /// to monomial-τ for the NURBS carrier. A dead-end span (bisection floor
    /// under a curvature discontinuity — nothing fits) falls back to the
    /// quintic: endpoint-exact, so seams stay C² even where the interior
    /// cannot meet tolerance.
    fn fitted_piece(
        &self,
        axis: usize,
        ta: f64,
        tb: f64,
        ua: f64,
        ub: f64,
        tol: FitTol,
    ) -> BezierPiece<f64> {
        let h = tb - ta;
        let mono_u = self
            .ladder_fit_axis(axis, ta, tb, tol, true)
            .unwrap_or_else(|| {
                let sa = self.axis_state_full(axis, ta, true);
                let sb = self.axis_state_full(axis, tb, true);
                quintic_in_u(sa, sb, h)
            });
        truncated_piece(&mono_u, ua, ub, h, FIT_TRUNC_POS_FACTOR * tol.pos_mm)
    }
}

/// Chebyshev-truncate an accepted monomial-in-u fit to its true degree, then
/// back to monomial-τ for the NURBS carrier. `h` is the span duration the fit
/// was built with (not recomputed from the bounds — they may live on a shifted
/// timeline, and the truncation weights must match the fit's own scaling).
pub(crate) fn truncated_piece(
    mono_u: &[f64],
    u_start: f64,
    u_end: f64,
    h: f64,
    pos_budget_mm: f64,
) -> BezierPiece<f64> {
    let cheb = truncate_chebyshev_c2_anchored(
        &monomial_u_to_chebyshev(mono_u),
        h,
        pos_budget_mm,
        FIT_TRUNC_VEL_MM_S,
        FIT_TRUNC_ACC_MM_S2,
    );
    BezierPiece {
        u_start,
        u_end,
        coeffs: chebyshev_to_monomial_tau(&cheb, h),
    }
}

/// Zero-pad every piece of every axis to the move's maximum degree — both
/// `bezier_pieces_to_nurbs` (uniform degree per curve) and `lane_curve`'s
/// cross-axis addition for CoreXY mixing require it. Enqueue's Chebyshev
/// truncation recovers each piece's true degree at the wire.
fn pad_to_uniform_degree(axes_pieces: &mut [Vec<BezierPiece<f64>>]) {
    let max_len = axes_pieces
        .iter()
        .flatten()
        .map(|p| p.coeffs.len())
        .max()
        .unwrap_or(1);
    assert!(
        max_len <= MAX_PIECE_COEFFS,
        "fitted piece degree {} exceeds the wire maximum",
        max_len - 1
    );
    for piece in axes_pieces.iter_mut().flatten() {
        piece.coeffs.resize(max_len, 0.0);
    }
}

fn refine_span(
    sampler: &Sampler<'_>,
    driven: &[usize],
    tol: FitTol,
    ta: f64,
    tb: f64,
    depth: u32,
    out: &mut Vec<f64>,
) {
    let h = tb - ta;
    let accept = depth >= MAX_SUBDIVISION_DEPTH
        || h <= 2.0 * MIN_FIT_PIECE_S
        || sampler.span_fits(driven, ta, tb, tol);
    if accept {
        out.push(tb);
    } else {
        let tm = 0.5 * (ta + tb);
        refine_span(sampler, driven, tol, ta, tm, depth + 1, out);
        refine_span(sampler, driven, tol, tm, tb, depth + 1, out);
    }
}

pub fn lower_move(
    gm: &Move,
    vm: &MoveVelocity,
    t_start: f64,
    start_pos: &[f64],
    fit_tol: FitTol,
    axis_chains: &[CompiledChain],
) -> Result<ShapedSegment, LoweringError> {
    let (axes_pieces, total_t) =
        lower_move_pieces(gm, vm, t_start, start_pos, fit_tol, axis_chains)?;
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
    fit_tol: FitTol,
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
    // A jerk-regime change is a curvature kink in the arc-length profile: no
    // polynomial spans it within the acceleration budget, so blind bisection
    // cascades down to the floor around each one. Pinning a grid knot at every
    // regime boundary lets both sides fit their full smooth spans instead.
    let mut knots = regime_knot_times(&profile, fit_tol);
    knots.retain(|&t| t > MIN_PIECE_DURATION_S && total_t - t > MIN_PIECE_DURATION_S);
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
    // Each axis refines its own grid: an axis only pays for the knots its own
    // signal needs (a follower is blind to path curvature, z to planar motion),
    // and the guarantee is unchanged — every piece of every axis is probed
    // against the same tolerances.
    let mut axes_pieces: Vec<Vec<BezierPiece<f64>>> = vec![Vec::new(); n_axes];
    let no_knots: Vec<f64> = Vec::new();
    for (axis, pieces) in axes_pieces.iter_mut().enumerate() {
        let driven = [axis];
        // A follower sees the profile's jerk edges scaled by its ratio, far
        // below the acceleration budget — seeding them would only cut its
        // near-linear track into spatial-sized pieces.
        let axis_knots = if axis < 3 { &knots } else { &no_knots };
        let mut bounds = vec![0.0];
        let mut prev = 0.0;
        for &k in axis_knots.iter().chain(std::iter::once(&total_t)) {
            if k - prev > MIN_PIECE_DURATION_S {
                refine_span(&sampler, &driven, fit_tol, prev, k, 0, &mut bounds);
                prev = k;
            }
        }
        for w in bounds.windows(2) {
            let (ta, tb) = (w[0], w[1]);
            if tb - ta <= MIN_PIECE_DURATION_S {
                continue;
            }
            let (ua, ub) = (t_start + ta, t_start + tb);
            pieces.push(sampler.fitted_piece(axis, ta, tb, ua, ub, fit_tol));
        }
    }
    pad_to_uniform_degree(&mut axes_pieces);

    Ok((axes_pieces, total_t))
}

/// The span the knot placement is sized to keep fittable. A jerk step `Δj`
/// inside a span of `h` leaves an acceleration residual no polynomial removes,
/// on the order of `Δj·h/8`; steps that stay under the acceleration budget at
/// this span are absorbed by the fit, bigger ones get a knot.
const KNOT_TARGET_SPAN_S: f64 = 0.02;

/// Window boundaries where the profile's leading jerk takes one isolated step
/// larger than the budget — a jerk-phase edge in the plan, flanked by steady
/// windows. The isolation requirement is what separates a regime edge from a
/// numerically-ridden section whose jerk chatters at every boundary: chatter
/// gets no knots and falls back to plain bisection, which the windowed
/// reconstruction already smooths well enough to fit.
fn regime_knot_times(profile: &ScalarProfile, tol: FitTol) -> Vec<f64> {
    let threshold = 8.0 * tol.accel_mm_s2 / KNOT_TARGET_SPAN_S;
    let jerks: Vec<f64> = profile.windows.iter().map(|w| 6.0 * w.coeffs[3]).collect();
    let steps: Vec<f64> = jerks.windows(2).map(|j| (j[1] - j[0]).abs()).collect();
    let mut out: Vec<f64> = Vec::new();
    for (i, &step) in steps.iter().enumerate() {
        let isolated = (i == 0 || steps[i - 1] < 0.25 * threshold)
            && (i + 1 >= steps.len() || steps[i + 1] < 0.25 * threshold);
        if step > threshold
            && isolated
            && out
                .last()
                .is_none_or(|&k| profile.knot_t[i + 1] - k > MIN_FIT_PIECE_S)
        {
            out.push(profile.knot_t[i + 1]);
        }
    }
    out
}

/// Linear pressure advance is `pos += k * vel`, exact on a polynomial of any
/// degree: `c′_i = c_i + k·(i+1)·c_{i+1}` (`SmoothKernel` is a downstream
/// convolution, so it stops the per-piece transform exactly as the sampled path
/// does). Mirrors the `ChainStage` semantics in [`Sampler::axis_state`].
fn apply_pressure_advance(coeffs: &mut [f64], chain: &CompiledChain) {
    for stage in &chain.stages {
        match stage {
            ChainStage::LinearPressureAdvance { k } => {
                for i in 0..coeffs.len().saturating_sub(1) {
                    coeffs[i] = k.mul_add((i + 1) as f64 * coeffs[i + 1], coeffs[i]);
                }
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
            let mut coeffs = vec![scale.mul_add(p.s0, base), scale * p.v0, scale * 0.5 * p.a0];
            if p.j != 0.0 {
                coeffs.push(scale * p.j / 6.0);
            }
            if let Some(chain) = axis_chains.get(axis) {
                apply_pressure_advance(&mut coeffs, chain);
            }
            pieces.push(BezierPiece {
                u_start: t_start + bounds[i],
                u_end: t_start + bounds[i + 1],
                coeffs,
            });
        }
    }
    pad_to_uniform_degree(&mut axes_pieces);

    Ok((axes_pieces, total_t))
}

#[cfg(test)]
mod tests;
