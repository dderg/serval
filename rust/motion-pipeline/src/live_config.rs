//! The planner settings the live printer derives from `PlannerConfig`. The
//! snapshot path, replays and the bench analyzer plan through the same
//! derivation, so every offline plan is the plan the printer executes.

use planner_config::PlannerConfig;

/// Backstop only. The continuity commit drains at every blend, so the buffer
/// normally hovers near the open-tail length past the finality barrier; this
/// force-drain to rest fires solely when no clean seam exists within reach.
///
/// It must sit well above the open tail a *legitimate* dense stream carries,
/// or the backstop stops being a backstop: the open tail is the re-plan batch
/// plus the brake-to-rest tail past the finality barrier, measured in moves.
/// Braking 600 mm/s at 1200 mm/s² takes 150 mm — about 300 moves of
/// half-millimetre slicer output. At 128 the Voron 0 motion repro tripped
/// it 41 times in 4673 moves, i.e. once per window: every look-ahead ended in
/// a full stop. A `Move` is 192 bytes, so this ceiling costs 200 kB of window.
pub const STREAM_MAX_BUFFER_MOVES: usize = 1024;
/// Velocity-profile ODE/sampling tolerance for the streaming planner, in v²
/// units. The offline default (1e-7) drives the adaptive RK4 to a precision far
/// below the physical noise floor — ~0.015 mm/s velocity error at this value on
/// a 300 mm/s move — at ~9× the host cost. Streaming runs on the Pi against a
/// real-time playhead, so it is tuned to the hardware budget: at 1e-4 a 40-move
/// burst plans in ~0.4 s instead of ~3.8 s, with trajectory time unchanged to
/// five significant figures (the residual is non-monotonic integration noise,
/// not a slower path).
pub const STREAM_INTEGRATION_TOL: f64 = 1e-4;

pub fn require_unlimited_config_jerk(max_jerk: f64) -> Result<(), &'static str> {
    if max_jerk.is_finite() {
        return Err(
            "finite [printer] max_jerk is not supported by the continuous trajectory pipeline; set max_jerk: 0",
        );
    }
    Ok(())
}

pub fn stream_config(cfg: &PlannerConfig) -> Result<crate::StreamConfig, String> {
    let cart = cfg.cartesian;
    require_unlimited_config_jerk(cart.max_jerk)?;
    cfg.check_ratio_ramp_extrude_only_caps()?;
    let max_extrude_only_accel_mm_s2 = cfg.max_extrude_only_accel.unwrap_or(f64::INFINITY);
    Ok(crate::StreamConfig {
        corner: geometry::CornerFitConfig {
            ramp_accel_budget_mm_s2: max_extrude_only_accel_mm_s2,
            ..cfg.corner
        },
        integration_tol: STREAM_INTEGRATION_TOL,
        max_extrude_only_velocity_mm_s: cfg.max_extrude_only_velocity.unwrap_or(f64::INFINITY),
        max_extrude_only_accel_mm_s2,
        fit_tol_mm: cfg.fit_tolerance_mm,
        fit_tol_accel_mm_s2: cfg.fit_tolerance_accel_mm_s2,
        max_buffer_moves: STREAM_MAX_BUFFER_MOVES,
        limits: z_capped_limits(
            cfg,
            cart.max_velocity,
            cart.max_accel,
            cart.corner_deviation,
        )?,
    })
}

/// A G-code move's limits: the path caps, the z caps for the planner to
/// apply to whatever share of each fitted segment's motion lands on z, and
/// the caps of the extruder the move extrudes on.
pub fn move_limits(
    cfg: &PlannerConfig,
    extruder: Option<geometry::ExtruderCaps>,
) -> Result<geometry::VelocityLimits, String> {
    let (max_velocity, accel, corner_deviation) = cfg.effective_limits();
    Ok(
        z_capped_limits(cfg, max_velocity, accel, corner_deviation)?
            .with_extruder_caps(extruder)?,
    )
}

/// Every limited follower's extruder caps under the chains `cfg` runs.
pub fn extruder_caps(cfg: &PlannerConfig) -> Result<Vec<geometry::ExtruderCaps>, String> {
    let chains = cfg.compile_active_chains().map_err(|e| e.to_string())?;
    planner_config::extruder_caps(&cfg.axis_registry, &chains)
}

/// The caps of the follower axis `axis` among `caps`, if it has limits.
#[must_use]
pub fn extruder_caps_on(
    caps: &[geometry::ExtruderCaps],
    axis: usize,
) -> Option<geometry::ExtruderCaps> {
    caps.iter().copied().find(|caps| caps.axis_index == axis)
}

fn z_capped_limits(
    cfg: &PlannerConfig,
    max_velocity: f64,
    accel: f64,
    corner_deviation: f64,
) -> Result<geometry::VelocityLimits, String> {
    Ok(
        geometry::VelocityLimits::try_new(max_velocity, accel, corner_deviation, f64::INFINITY)?
            .with_z_caps(cfg.cartesian.max_z_velocity, cfg.cartesian.max_z_accel)?,
    )
}

#[cfg(test)]
mod tests;
