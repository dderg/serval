use std::f64::consts::PI;

const COLLINEAR_EPS_RAD: f64 = 1e-3;

/// The largest per-axis velocity step a junction left unblended may take:
/// passing a turn `θ` at speed `v` steps the velocity by `v·θ`.
pub const SEAM_VELOCITY_STEP_MM_S: f64 = 0.1;
const EXTRUSION_RAMP_REL_TOL: f64 = 0.25;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CornerFitConfig {
    /// The widest turn a junction may ever pass unblended; at speed it is
    /// narrowed further to keep `v·θ` within `seam_velocity_step_mm_s`.
    pub theta_min_rad: f64,
    pub seam_velocity_step_mm_s: f64,
    pub theta_max_rad: f64,
    /// Above this relative difference in per-axis extrusion ratio across a
    /// corner, the junction is left unblended (a full stop) rather than blended
    /// with a mid-corner ramp — see [`UnblendReason::ExtrusionStep`]. The same
    /// band bounds how far an arc run's facet ratios may spread around the
    /// single linear ramp the reconstruction carries.
    pub extrusion_ramp_rel_tol: f64,
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
            seam_velocity_step_mm_s: SEAM_VELOCITY_STEP_MM_S,
            theta_max_rad: PI - COLLINEAR_EPS_RAD,
            extrusion_ramp_rel_tol: EXTRUSION_RAMP_REL_TOL,
            ramp_accel_budget_mm_s2: f64::INFINITY,
        }
    }
}
