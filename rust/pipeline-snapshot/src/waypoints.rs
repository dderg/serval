//! G-code text → absolute waypoints, mirroring `snapshots/harness.py::parse_gcode`
//! move for move so the playground and the snapshot harness agree on what a
//! pasted file means. E rides as the fourth coordinate so retracts (E-only
//! moves) and extruding moves flow through the pipeline as followers; the
//! engine differences consecutive E to a per-move delta. Each waypoint carries
//! the runtime limit overrides in force for the move ending at it — the same
//! [`RuntimeLimits`] state the live engine holds when that move is submitted —
//! so the snapshot derives its effective limits through the live clamp.
//! Extruder mode is M82 (absolute) / M83 (relative), independent of the
//! G90/G91 flag that governs X/Y/Z; under G91 an undeclared extruder rides
//! along as relative, and an E word with no mode declared at all is refused
//! rather than guessed. G92 resets any axis's position (commonly `G92 E0`)
//! without emitting a move.

use planner_config::{CartesianLimits, RuntimeLimits};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Waypoint {
    pub position: [f64; 4],
    pub feedrate: f64,
    pub runtime: RuntimeLimits,
}

impl Waypoint {
    /// A waypoint reached with no runtime limit override in force.
    #[must_use]
    pub fn at(position: [f64; 4], feedrate: f64) -> Self {
        Self {
            position,
            feedrate,
            runtime: RuntimeLimits::default(),
        }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum WaypointError {
    #[error(transparent)]
    Lex(#[from] gcode::ParseError),
    #[error(
        "line {line}: G{major} arc command is not supported: the motion engine has no native arc ingestion yet, and silently linearizing it here would claim to exercise an arc while feeding the engine straight segments"
    )]
    UnsupportedArc { line: u32, major: u32 },
    #[error(
        "line {line}: E word before any M82/M83 (or G91) — the extruder mode is ambiguous, and guessing absolute turns relative-E slicer output into garbage extrusion ratios. Declare the mode (slicer excerpts printed with relative extrusion need an 'M83' line at the top)."
    )]
    AmbiguousExtruderMode { line: u32 },
    #[error("line {line}: {command} {param}={value}: {reason}")]
    InvalidParam {
        line: u32,
        command: Box<str>,
        param: Box<str>,
        value: Box<str>,
        reason: &'static str,
    },
    #[error(
        "line {line}: SET_VELOCITY_LIMIT: SQUARE_CORNER_VELOCITY and CORNER_DEVIATION are aliases for the same corner budget; set exactly one"
    )]
    CornerBudgetAliases { line: u32 },
    #[error(
        "line {line}: {source} — the live host keeps the old limit with a warning; a snapshot refuses rather than plan a limit the G-code never got"
    )]
    RuntimeLimit {
        line: u32,
        source: planner_config::RuntimeLimitError,
    },
}

#[derive(Clone, Copy)]
enum Bound {
    Any,
    Above(f64),
    AtLeast(f64),
}

/// An extended command's `KEY=value` arguments as klippy's `GCodeCommand`
/// reads them: keys case-insensitive, values parsed only when asked for.
struct ExtendedArgs<'a> {
    command: &'a str,
    line: u32,
    args: &'a [(Box<str>, Box<str>)],
}

impl ExtendedArgs<'_> {
    fn raw(&self, key: &str) -> Option<&str> {
        self.args
            .iter()
            .rev()
            .find(|(k, _)| k.eq_ignore_ascii_case(key))
            .map(|(_, v)| &**v)
    }

    fn float(&self, key: &str, bound: Bound) -> Result<Option<f64>, WaypointError> {
        let Some(raw) = self.raw(key) else {
            return Ok(None);
        };
        let invalid = |reason| WaypointError::InvalidParam {
            line: self.line,
            command: self.command.into(),
            param: key.into(),
            value: raw.into(),
            reason,
        };
        let value: f64 = raw.trim().parse().map_err(|_| invalid("not a number"))?;
        match bound {
            Bound::Above(min) if value <= min => Err(invalid("must be above the minimum")),
            Bound::AtLeast(min) if value < min => Err(invalid("must be at least the minimum")),
            _ => Ok(Some(value)),
        }
    }
}

/// `SET_VELOCITY_LIMIT`, as `Motion.cmd_SET_VELOCITY_LIMIT` handles it.
fn set_velocity_limit(
    args: &ExtendedArgs<'_>,
    config: &CartesianLimits,
    runtime: &mut RuntimeLimits,
) -> Result<(), WaypointError> {
    args.float("MINIMUM_CRUISE_RATIO", Bound::Any)?;
    args.float("ACCEL_TO_DECEL", Bound::Any)?;
    let velocity = args.float("VELOCITY", Bound::Above(0.0))?;
    let accel = args.float("ACCEL", Bound::Above(0.0))?;
    let square_corner_velocity = args.float("SQUARE_CORNER_VELOCITY", Bound::AtLeast(0.0))?;
    let corner_deviation = args.float("CORNER_DEVIATION", Bound::AtLeast(0.0))?;
    if square_corner_velocity.is_some() && corner_deviation.is_some() {
        return Err(WaypointError::CornerBudgetAliases { line: args.line });
    }
    let runtime_limit = |source| WaypointError::RuntimeLimit {
        line: args.line,
        source,
    };
    if velocity.is_some() {
        runtime.set_velocity_cap(velocity).map_err(runtime_limit)?;
    }
    if accel.is_some() {
        runtime.set_accel_cap(accel).map_err(runtime_limit)?;
    }
    let corner_deviation = square_corner_velocity
        .map(|scv| geometry::corner_deviation_from_scv(scv, config.max_accel))
        .or(corner_deviation);
    if corner_deviation.is_some() {
        runtime
            .set_corner_deviation(corner_deviation)
            .map_err(runtime_limit)?;
    }
    Ok(())
}

/// `RESET_VELOCITY_LIMIT`, as `Motion.cmd_RESET_VELOCITY_LIMIT` handles it.
fn reset_velocity_limit(runtime: &mut RuntimeLimits) {
    runtime
        .set_velocity_cap(None)
        .and_then(|()| runtime.set_accel_cap(None))
        .and_then(|()| runtime.set_corner_deviation(None))
        .expect("clearing a runtime limit is always valid");
}

/// `M204`, as `Motion.cmd_M204` handles it: `S`, else the lower of `P` and
/// `T`; with neither, klippy only reports the command invalid.
fn m204(
    params: &gcode::Params,
    line: u32,
    runtime: &mut RuntimeLimits,
) -> Result<(), WaypointError> {
    let above_zero = |letter: u8| match params.get(letter) {
        Some(value) if value <= 0.0 => Err(WaypointError::InvalidParam {
            line,
            command: "M204".into(),
            param: char::from(letter).to_string().into(),
            value: value.to_string().into(),
            reason: "must be above the minimum",
        }),
        value => Ok(value),
    };
    let accel = match above_zero(b'S')? {
        Some(s) => Some(s),
        None => match (above_zero(b'P')?, above_zero(b'T')?) {
            (Some(p), Some(t)) => Some(p.min(t)),
            _ => None,
        },
    };
    if accel.is_some() {
        runtime
            .set_accel_cap(accel)
            .map_err(|source| WaypointError::RuntimeLimit { line, source })?;
    }
    Ok(())
}

pub fn parse_gcode(text: &str, config: &CartesianLimits) -> Result<Vec<Waypoint>, WaypointError> {
    let max_velocity = config.max_velocity;
    let mut waypoints: Vec<Waypoint> = Vec::new();
    let (mut x, mut y, mut z, mut e) = (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64);
    let mut feedrate = max_velocity;
    let mut runtime = RuntimeLimits::default();
    let mut relative = false;
    let mut e_relative: Option<bool> = None;

    for tok in gcode::lex(text) {
        let tok = tok?;
        let (letter, major, minor, params, line_no) = match tok {
            gcode::Token::Command {
                letter,
                major,
                minor,
                params,
                line_no,
            } => (letter, major, minor, params, line_no),
            gcode::Token::Extended {
                name,
                args,
                line_no,
            } => {
                let args = ExtendedArgs {
                    command: &name,
                    line: line_no,
                    args: &args,
                };
                match &*name {
                    "SET_VELOCITY_LIMIT" => set_velocity_limit(&args, config, &mut runtime)?,
                    "RESET_VELOCITY_LIMIT" => reset_velocity_limit(&mut runtime),
                    _ => {}
                }
                continue;
            }
            _ => continue,
        };
        if minor.is_some() {
            continue;
        }
        match (letter, major) {
            (b'G', 90) => relative = false,
            (b'G', 91) => relative = true,
            (b'M', 82) => e_relative = Some(false),
            (b'M', 83) => e_relative = Some(true),
            (b'M', 204) => m204(&params, line_no, &mut runtime)?,
            (b'G', 92) => {
                x = params.x().unwrap_or(x);
                y = params.y().unwrap_or(y);
                z = params.z().unwrap_or(z);
                e = params.e().unwrap_or(e);
            }
            (b'G', 2 | 3) => {
                return Err(WaypointError::UnsupportedArc {
                    line: line_no,
                    major,
                });
            }
            (b'G', cmd @ (0 | 1)) => {
                let has_position =
                    params.x().is_some() || params.y().is_some() || params.z().is_some();

                let (nx, ny, nz) = if relative {
                    (
                        x + params.x().unwrap_or(0.0),
                        y + params.y().unwrap_or(0.0),
                        z + params.z().unwrap_or(0.0),
                    )
                } else {
                    (
                        params.x().unwrap_or(x),
                        params.y().unwrap_or(y),
                        params.z().unwrap_or(z),
                    )
                };

                let ne = match params.e() {
                    Some(ev) => {
                        if e_relative.is_none() && !relative {
                            return Err(WaypointError::AmbiguousExtruderMode { line: line_no });
                        }
                        if e_relative.unwrap_or(true) {
                            e + ev
                        } else {
                            ev
                        }
                    }
                    None => e,
                };

                if cmd == 1 {
                    feedrate = params.f().map_or(feedrate, |f| f / 60.0);
                }
                if !(has_position || ne != e) {
                    continue;
                }
                (x, y, z, e) = (nx, ny, nz, ne);
                let e_only_before_any_position = waypoints.is_empty() && !has_position;
                if e_only_before_any_position {
                    continue;
                }
                let move_feedrate = if cmd == 0 { max_velocity } else { feedrate };
                waypoints.push(Waypoint {
                    position: [x, y, z, e],
                    feedrate: move_feedrate,
                    runtime,
                });
            }
            _ => {}
        }
    }
    Ok(waypoints)
}

#[cfg(test)]
mod tests;
