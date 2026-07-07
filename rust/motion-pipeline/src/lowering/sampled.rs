use geometry::path::lowering::PositionProfile;
use nurbs::bezier::BezierPiece;
use trajectory::{ChainStage, CompiledChain};

use super::ladder::{FIT_TRUNC_POS_FACTOR, ladder_fit, quintic_in_u, truncated_piece};
use super::profile::ScalarProfile;
use super::{FitTol, MAX_SUBDIVISION_DEPTH, MIN_FIT_PIECE_S};

pub(super) struct Sampler<'a> {
    pub(super) profile: &'a ScalarProfile,
    pub(super) spatial: Option<&'a geometry::path::Segment>,
    pub(super) start_pos: &'a [f64],
    pub(super) followers: &'a [geometry::FollowerDemand],
    pub(super) s_len: f64,
    pub(super) axis_chains: &'a [CompiledChain],
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

    pub(super) fn axis_state_full(
        &self,
        axis: usize,
        t: f64,
        apply_zero_support: bool,
    ) -> (f64, f64, f64) {
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
    pub(super) fn fitted_piece(
        &self,
        axis: usize,
        ta: f64,
        tb: f64,
        ua: f64,
        ub: f64,
        tol: FitTol,
    ) -> BezierPiece {
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
