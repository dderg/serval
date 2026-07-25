use geometry::path::lowering::PositionProfile;
use geometry::{Move, SurfaceTransform};
use nurbs::bezier::BezierPiece;
use trajectory::{ChainStage, CompiledChain};

use super::ladder::{FIT_TRUNC_POS_FACTOR, ladder_fit, quintic_in_u, truncated_piece};
use super::profile::{KnotSide, ScalarProfile};
use super::{FitTol, MAX_SUBDIVISION_DEPTH, MIN_FIT_PIECE_S};

/// How the bed surface transform applies to this move's Z axis. `Constant`
/// is the flat-region/faded-out fast form: the correction varies by less
/// than [`WARP_CONST_EPS_MM`] over the move, so one offset stands in for the
/// surface and the closed-form phase path stays available.
#[derive(Clone, Copy)]
pub(super) enum ZWarp<'a> {
    None,
    Constant(f64),
    Surface(&'a SurfaceTransform),
}

/// Correction treated as constant over a move when it cannot vary more than
/// this. Each move's constant is evaluated at its own gcode start, so the
/// error does not accumulate: consecutive machine-space Z steps at seams stay
/// under this bound.
const WARP_CONST_EPS_MM: f64 = 2e-3;
const WARP_BBOX_SAMPLES: usize = 8;

pub(super) fn z_warp_mode<'a>(
    mesh: Option<&'a SurfaceTransform>,
    gm: &Move,
    start_pos: &[f64],
) -> ZWarp<'a> {
    let Some(t) = mesh else {
        return ZWarp::None;
    };
    let Some(seg) = gm.segment.spatial.as_ref() else {
        let p = [
            start_pos.first().copied().unwrap_or(0.0),
            start_pos.get(1).copied().unwrap_or(0.0),
            start_pos.get(2).copied().unwrap_or(0.0),
        ];
        return ZWarp::Constant(t.correction_at(p[0], p[1], p[2]));
    };
    let s_len = gm.segment.s_len();
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for k in 0..=WARP_BBOX_SAMPLES {
        let p = seg.point_at(s_len * k as f64 / WARP_BBOX_SAMPLES as f64);
        for axis in 0..3 {
            lo[axis] = lo[axis].min(p[axis]);
            hi[axis] = hi[axis].max(p[axis]);
        }
    }
    // Between samples the curve deviates from the chord by at most the
    // sagitta bound κ·Δs²/8 — exactly zero for straight moves, so their
    // sampled bbox is tight.
    let ds = s_len / WARP_BBOX_SAMPLES as f64;
    let pad = {
        use geometry::path::CurvatureProfile;
        seg.kappa_peak().1.abs() * ds * ds / 8.0
    };
    let spread = t.correction_spread_over(
        lo[0] - pad,
        hi[0] + pad,
        lo[1] - pad,
        hi[1] + pad,
        lo[2] - pad,
        hi[2] + pad,
    );
    if spread <= WARP_CONST_EPS_MM {
        let p0 = seg.point_at(0.0);
        ZWarp::Constant(t.correction_at(p0[0], p0[1], p0[2]))
    } else {
        ZWarp::Surface(t)
    }
}

pub(super) struct Sampler<'a> {
    pub(super) profile: &'a ScalarProfile,
    pub(super) spatial: Option<&'a geometry::path::Segment>,
    pub(super) start_pos: &'a [f64],
    pub(super) followers: &'a [geometry::FollowerDemand],
    pub(super) s_len: f64,
    pub(super) axis_chains: &'a [CompiledChain],
    pub(super) z_warp: ZWarp<'a>,
}

impl Sampler<'_> {
    /// Apply the surface warp to the Z axis's `(pos, vel, accel)` base state,
    /// chain-rule velocity and acceleration through the moving XY.
    fn warped_z(
        &self,
        seg: &geometry::path::Segment,
        s: f64,
        v: f64,
        a_t: f64,
        base: (f64, f64, f64),
    ) -> (f64, f64, f64) {
        match self.z_warp {
            ZWarp::None => base,
            ZWarp::Constant(c) => (base.0 + c, base.1, base.2),
            ZWarp::Surface(t) => {
                let p = seg.point_at(s);
                let h = seg.heading_at(s);
                let dh = seg.dheading_ds(s);
                let vel = [h[0] * v, h[1] * v, h[2] * v];
                let acc = [
                    a_t * h[0] + v * v * dh[0],
                    a_t * h[1] + v * v * dh[1],
                    a_t * h[2] + v * v * dh[2],
                ];
                let w = t.warp(p[0], p[1], p[2]);
                let w_dot = w.wx * vel[0] + w.wy * vel[1] + w.wz * vel[2];
                let w_ddot = w.wx * acc[0]
                    + w.wy * acc[1]
                    + w.wz * acc[2]
                    + w.wxx * vel[0] * vel[0]
                    + w.wyy * vel[1] * vel[1]
                    + 2.0
                        * (w.wxy * vel[0] * vel[1]
                            + w.wxz * vel[0] * vel[2]
                            + w.wyz * vel[1] * vel[2]);
                (base.0 + w.w, base.1 + w_dot, base.2 + w_ddot)
            }
        }
    }

    fn axis_base_state(&self, axis: usize, t: f64, side: KnotSide) -> (f64, f64, f64) {
        let (s, v, a_t) = self.profile.state_at_side(t, side);
        if axis < 3 {
            match self.spatial {
                Some(seg) => {
                    let accel = a_t * seg.heading_at(s)[axis] + v * v * seg.dheading_ds(s)[axis];
                    let base = (seg.point_at(s)[axis], seg.heading_at(s)[axis] * v, accel);
                    if axis == 2 {
                        self.warped_z(seg, s, v, a_t, base)
                    } else {
                        base
                    }
                }
                None => {
                    let hold = self.start_pos.get(axis).copied().unwrap_or(0.0);
                    let offset = match self.z_warp {
                        ZWarp::Constant(c) if axis == 2 => c,
                        ZWarp::Surface(_) => {
                            unreachable!("z_warp_mode returns Constant for virtual moves")
                        }
                        _ => 0.0,
                    };
                    (hold + offset, 0.0, 0.0)
                }
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

    pub(super) fn axis_state_full(
        &self,
        axis: usize,
        t: f64,
        apply_zero_support: bool,
    ) -> (f64, f64, f64) {
        self.axis_state_side(axis, t, apply_zero_support, KnotSide::Begin)
    }

    fn axis_state_side(
        &self,
        axis: usize,
        t: f64,
        apply_zero_support: bool,
        side: KnotSide,
    ) -> (f64, f64, f64) {
        let (mut pos, mut vel, mut accel) = self.axis_base_state(axis, t, side);
        if apply_zero_support {
            if let Some(chain) = self.axis_chains.get(axis) {
                for stage in &chain.stages {
                    match stage {
                        ChainStage::DerivativeGains { k1, k2 } => {
                            assert!(
                                *k2 == 0.0,
                                "DerivativeGains k2={k2} reached the sampled lowering path: \
                                 the sampler carries (pos, vel, accel) only, so the transformed \
                                 velocity `vel + k1*accel + k2*jerk` cannot be computed here; \
                                 propagate jerk through axis_base_state before using k2 != 0 \
                                 on a sampled axis"
                            );
                            pos = k1.mul_add(vel, pos);
                            vel = k1.mul_add(accel, vel);
                            accel = 0.0;
                        }
                        ChainStage::SmoothKernel(_) => break,
                    }
                }
            }
        }
        (pos, vel, accel)
    }

    pub(super) fn axis_state(&self, axis: usize, t: f64, apply_zero_support: bool) -> (f64, f64) {
        let (pos, vel, _) = self.axis_state_full(axis, t, apply_zero_support);
        (pos, vel)
    }

    pub(super) fn axis_accel(&self, axis: usize, t: f64, apply_zero_support: bool) -> f64 {
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
        let sa = self.axis_state_side(axis, ta, apply_zero_support, KnotSide::Begin);
        let sb = self.axis_state_side(axis, tb, apply_zero_support, KnotSide::End);
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
    pub(super) fn fitted_piece(
        &self,
        axis: usize,
        ta: f64,
        tb: f64,
        ua: f64,
        ub: f64,
        tol: FitTol,
        deriv_scale: f64,
    ) -> BezierPiece {
        let h = tb - ta;
        let mono_u = self
            .ladder_fit_axis(axis, ta, tb, tol, true)
            .unwrap_or_else(|| {
                let sa = self.axis_state_side(axis, ta, true, KnotSide::Begin);
                let sb = self.axis_state_side(axis, tb, true, KnotSide::End);
                quintic_in_u(sa, sb, h)
            });
        truncated_piece(
            &mono_u,
            ua,
            ub,
            h,
            FIT_TRUNC_POS_FACTOR * tol.pos_mm,
            deriv_scale,
        )
    }
}

pub(super) fn refine_span(
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

/// The span the knot placement is sized to keep fittable. A jerk step `Δj`
/// inside a span of `h` leaves an acceleration residual no polynomial removes,
/// on the order of `Δj·h/8`; steps that stay under the acceleration budget at
/// this span are absorbed by the fit, bigger ones get a knot.
const KNOT_TARGET_SPAN_S: f64 = 0.02;

/// Knots for a phase-built profile: every interior window joint whose
/// acceleration or jerk steps past the fit budget. Phase windows are the plan
/// itself, so a step at a joint is a genuine regime corner — under unlimited
/// jerk the merged chain steps its acceleration at each one — and a span must
/// never straddle it: the quintic base would smear the step into interior
/// wiggle no ladder degree removes. Joints that land inside a still-tiny gap
/// merge into the previous knot, exactly as the closed-form straight path
/// merges sub-floor phases.
pub(super) fn phase_knot_times(profile: &ScalarProfile, tol: FitTol) -> Vec<f64> {
    let jerk_threshold = 8.0 * tol.accel_mm_s2 / KNOT_TARGET_SPAN_S;
    let mut out: Vec<f64> = Vec::new();
    for (i, pair) in profile.windows.windows(2).enumerate() {
        let (w, next) = (&pair[0], &pair[1]);
        let a_end = 2.0 * w.coeffs[2] + 6.0 * w.coeffs[3] * w.dt;
        let a_next = 2.0 * next.coeffs[2];
        let j_step = 6.0 * (next.coeffs[3] - w.coeffs[3]).abs();
        let discontinuous =
            (a_next - a_end).abs() > 0.5 * tol.accel_mm_s2 || j_step > jerk_threshold;
        if discontinuous
            && out
                .last()
                .is_none_or(|&k| profile.knot_t[i + 1] - k > MIN_FIT_PIECE_S)
        {
            out.push(profile.knot_t[i + 1]);
        }
    }
    out
}

/// Window boundaries where the profile's leading jerk takes one isolated step
/// larger than the budget — a jerk-phase edge in the plan, flanked by steady
/// windows. The isolation requirement is what separates a regime edge from a
/// numerically-ridden section whose jerk chatters at every boundary: chatter
/// gets no knots and falls back to plain bisection, which the windowed
/// reconstruction already smooths well enough to fit.
pub(super) fn regime_knot_times(profile: &ScalarProfile, tol: FitTol) -> Vec<f64> {
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
