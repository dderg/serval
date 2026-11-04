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

/// The name of the axis a G-code E word drives.
pub const EXTRUSION_AXIS: &str = "e";

/// One G-code line move as the host hands it over: where the toolhead is,
/// how far it goes, the E it extrudes, its feedrate and its sequence number.
/// `start + delta` is the end, computed in exactly one place.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GcodeLine {
    pub start: [f64; 3],
    pub delta: [f64; 3],
    pub de: f64,
    pub feedrate_mm_s: f64,
    pub seq: u32,
}

impl GcodeLine {
    #[must_use]
    pub fn end(&self) -> [f64; 3] {
        [
            self.start[0] + self.delta[0],
            self.start[1] + self.delta[1],
            self.start[2] + self.delta[2],
        ]
    }
}

/// The only constructor of the planner's G-code moves. The live bridge, the
/// snapshot path, the replays and the bench reference all build every move
/// through it, so no two of them can plan the same G-code under different
/// limits: the config's path and z caps, the caps of the [`EXTRUSION_AXIS`]
/// on every move (a travel beside an extrusion is budgeted by them too), and
/// the runtime overrides in force when the move is submitted.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GcodeMoveBase {
    limits: geometry::VelocityLimits,
    extruder_axis: Option<usize>,
}

impl GcodeMoveBase {
    pub fn of(
        cfg: &PlannerConfig,
        extruder_caps: &[geometry::ExtruderCaps],
    ) -> Result<Self, String> {
        let extruder_axis = cfg.axis_registry.axis_index(EXTRUSION_AXIS).ok();
        let extruder = extruder_axis.and_then(|axis| {
            extruder_caps
                .iter()
                .copied()
                .find(|caps| caps.axis_index == axis)
        });
        let cart = cfg.cartesian;
        Ok(Self {
            limits: z_capped_limits(
                cfg,
                cart.max_velocity,
                cart.max_accel,
                cart.corner_deviation,
            )?
            .with_extruder_caps(extruder)?,
            extruder_axis,
        })
    }

    /// A base not read from a config, for planner tests that pick limits
    /// directly.
    #[must_use]
    pub fn from_parts(limits: geometry::VelocityLimits, extruder_axis: Option<usize>) -> Self {
        Self {
            limits,
            extruder_axis,
        }
    }

    #[must_use]
    pub fn limits(&self) -> geometry::VelocityLimits {
        self.limits
    }

    /// The config limits runtime overrides are clamped against.
    #[must_use]
    pub fn config_limits(&self) -> planner_config::CartesianLimits {
        planner_config::CartesianLimits {
            max_velocity: self.limits.max_velocity_mm_s,
            max_accel: self.limits.accel_mm_s2,
            max_jerk: self.limits.max_jerk_mm_s3,
            max_z_velocity: self.limits.max_z_velocity_mm_s,
            max_z_accel: self.limits.max_z_accel_mm_s2,
            corner_deviation: self.limits.corner_deviation_mm,
        }
    }

    pub fn limits_under(
        &self,
        runtime: &planner_config::RuntimeLimits,
    ) -> Result<geometry::VelocityLimits, &'static str> {
        let (velocity, accel, corner_deviation) = runtime.effective(&self.config_limits());
        geometry::VelocityLimits::try_new(
            velocity,
            accel,
            corner_deviation,
            self.limits.max_jerk_mm_s3,
        )?
        .with_z_caps(
            self.limits.max_z_velocity_mm_s,
            self.limits.max_z_accel_mm_s2,
        )?
        .with_extruder_caps(self.limits.extruder)
    }

    /// `None` for a line too short to move anything.
    pub fn line_move(
        &self,
        runtime: &planner_config::RuntimeLimits,
        line: GcodeLine,
    ) -> Result<Option<geometry::Move>, String> {
        if line.de != 0.0 && self.extruder_axis.is_none() {
            return Err(format!(
                "E word on a move but no [axis {EXTRUSION_AXIS}] is declared — declare the \
                 follower axis or stop sending E"
            ));
        }
        let ctx = geometry::MoveContext {
            extruder_axis: self.extruder_axis.unwrap_or_default(),
            feedrate_mm_s: line.feedrate_mm_s,
            limits: self.limits_under(runtime)?,
            source: geometry::SourceRange {
                start_line: line.seq,
                end_line: line.seq,
            },
        };
        match geometry::line_move(line.start, line.end(), line.de, ctx) {
            Ok(m) => Ok(Some(m)),
            Err(geometry::FrontendError::ZeroMotion { .. }) => Ok(None),
            Err(e) => Err(format!("{e:?}")),
        }
    }
}

/// Every limited follower's extruder caps under the chains `cfg` runs.
pub fn extruder_caps(cfg: &PlannerConfig) -> Result<Vec<geometry::ExtruderCaps>, String> {
    let chains = cfg.compile_active_chains().map_err(|e| e.to_string())?;
    planner_config::extruder_caps(&cfg.axis_registry, &chains)
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
