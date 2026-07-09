use std::f64::consts::PI;

const COLLINEAR_EPS_RAD: f64 = 1e-3;
const EXTRUSION_RAMP_REL_TOL: f64 = 0.25;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CornerFitConfig {
    pub theta_min_rad: f64,
    pub theta_max_rad: f64,
    /// Above this relative difference in per-axis extrusion ratio across a
    /// corner, the junction is left unblended (a full stop) rather than blended
    /// with a mid-corner ramp — see [`UnblendReason::ExtrusionStep`]. The same
    /// band bounds how far an arc run's facet ratios may spread around the
    /// single linear ramp the reconstruction carries.
    pub extrusion_ramp_rel_tol: f64,
    /// Second moment (s²) of the widest smoothing kernel among the spatial
    /// post-processor chains. A convolution kernel pulls the path inward by
    /// ≈ (σ²/2)·a wherever it curves, so the fitter deducts that share from
    /// each junction's corner-deviation budget before spending the remainder
    /// on blend geometry. Zero when no spatial chain carries a kernel.
    /// TODO: direction-dependent refinement — use the blended axes' own σ²
    /// instead of the worst spatial axis.
    pub kernel_variance_s2: f64,
    /// `max_extrude_only_accel` — worst-case budget for the extruder
    /// acceleration a fitter-created ramp adds on top of the G-code's own
    /// constant-ratio flow. The planner deliberately applies the
    /// `max_extrude_only_*` limits to extrude-only moves alone (coupling the
    /// follower into print-move velocity planning would make every plan
    /// iteration solve a joint ODE), so the fitter instead proves each ramp
    /// feasible in closed form — see [`ramps_admitted`].
    pub ramp_accel_budget_mm_s2: f64,
}

impl Default for CornerFitConfig {
    fn default() -> Self {
        Self {
            theta_min_rad: COLLINEAR_EPS_RAD,
            theta_max_rad: PI - COLLINEAR_EPS_RAD,
            extrusion_ramp_rel_tol: EXTRUSION_RAMP_REL_TOL,
            kernel_variance_s2: 0.0,
            ramp_accel_budget_mm_s2: f64::INFINITY,
        }
    }
}
