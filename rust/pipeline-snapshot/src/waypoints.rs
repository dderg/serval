//! G-code text → absolute waypoints, mirroring `snapshots/harness.py::parse_gcode`
//! move for move so the playground and the snapshot harness agree on what a
//! pasted file means. E rides as the fourth coordinate so retracts (E-only
//! moves) and extruding moves flow through the pipeline as followers; the
//! engine differences consecutive E to a per-move delta. Each waypoint carries
//! the runtime limit overrides in force for the move ending at it — the same
//! [`RuntimeLimits`] state the live engine holds when that move is submitted —
//! so the snapshot derives its effective limits through the live clamp.
//! Coordinates follow klippy's `GCodeMove`: absolute by default, E relative
//! under M83 or under G91 (even after M82), and G92 moves the G-code origin,
//! never the toolhead.

use planner_config::{CartesianLimits, RuntimeLimits};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Waypoint {
    pub position: [f64; 4],
    pub feedrate: f64,
    pub runtime: RuntimeLimits,
    /// `Some(d)`: the toolhead comes to rest at the previous waypoint and
    /// holds there `d` seconds before the move to this one starts, as the
    /// live engine drains for `G4`, `M400`, `M18`/`M84` and
    /// `SET_KINEMATIC_POSITION`; `None`: the move blends from the previous
    /// one. A rest after the last move leaves the trajectory unchanged and
    /// has no waypoint to ride on.
    pub dwell_before_s: Option<f64>,
}

impl Waypoint {
    /// A waypoint reached with no runtime limit override in force, blending
    /// from the previous one.
    #[must_use]
    pub fn at(position: [f64; 4], feedrate: f64) -> Self {
        Self {
            position,
            feedrate,
            runtime: RuntimeLimits::default(),
            dwell_before_s: None,
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
    #[error("line {line}: G{command} F{feedrate}: invalid speed")]
    InvalidSpeed {
        line: u32,
        command: u32,
        feedrate: f64,
    },
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
    #[error(
        "line {line}: Must home axis first: {axis} lost its homing when the motors went off (M18/M84), so the live host refuses the move"
    )]
    Unhomed { line: u32, axis: char },
    #[error(
        "line {line}: SET_KINEMATIC_POSITION renames the toolhead position {from:?} as {to:?} without moving it; a snapshot trajectory is one continuous path and cannot plan the jump"
    )]
    PositionJump {
        line: u32,
        from: [f64; 3],
        to: [f64; 3],
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

/// klippy's default `GCodeMove.speed`, in mm/s, before any `F` word.
const LIVE_DEFAULT_SPEED_MM_S: f64 = 25.0;
/// klippy's `GCodeMove.speed_factor` at 100 %: `F` is mm/min.
const LIVE_SPEED_FACTOR: f64 = 1.0 / 60.0;
/// klippy's `Move`: below this the move is extrude-only.
const MOVE_EPSILON_MM: f64 = 0.000_000_001;

/// `stepper_enable.DISABLE_STALL_TIME`: `M18`/`M84` dwell this long before
/// the motors go off and hold the next move this long after.
const DISABLE_STALL_TIME_S: f64 = 0.1;

/// klippy's `Motion` as the snapshot sees it: the commanded position, the
/// runtime limit overrides in force, which axes are homed, and `Motion.move`
/// turning a target and a speed into the move it submits. The snapshot
/// starts at the first move that names a position — the anchor — so a stop
/// at rest before it is already where the plan begins.
struct Toolhead<'a> {
    config: &'a CartesianLimits,
    runtime: RuntimeLimits,
    commanded: [f64; 4],
    anchored: bool,
    homed: [bool; 3],
    rest_s: Option<f64>,
}

impl Toolhead<'_> {
    /// The engine drains to rest and holds `dwell_s` before the next move:
    /// `Motion.dwell`, `Motion.wait_moves` and every forced fence.
    fn rest(&mut self, dwell_s: f64) {
        self.rest_s = Some(self.rest_s.unwrap_or(0.0) + dwell_s);
    }

    /// `PrinterStepperEnable.motor_off` with the host ahead of the MCU:
    /// dwell, motors off, hold the next move past the disable, and the
    /// kinematics forget every axis's homing.
    fn motors_off(&mut self) {
        self.rest(2.0 * DISABLE_STALL_TIME_S);
        self.homed = [false; 3];
    }

    /// `ForceMove.cmd_SET_KINEMATIC_POSITION`: wait for the moves, rename
    /// the toolhead position and home every axis but the cleared ones.
    fn set_kinematic_position(&mut self, args: &ExtendedArgs<'_>) -> Result<(), WaypointError> {
        self.rest(0.0);
        let mut renamed = self.commanded;
        for (axis, key) in ["X", "Y", "Z"].into_iter().enumerate() {
            if let Some(value) = args.float(key, Bound::Any)? {
                renamed[axis] = value;
            }
        }
        let jumps =
            (0..3).any(|axis| (renamed[axis] - self.commanded[axis]).abs() >= MOVE_EPSILON_MM);
        if self.anchored && jumps {
            return Err(WaypointError::PositionJump {
                line: args.line,
                from: [self.commanded[0], self.commanded[1], self.commanded[2]],
                to: [renamed[0], renamed[1], renamed[2]],
            });
        }
        self.commanded = renamed;
        let clear = args.raw("CLEAR").unwrap_or("").to_ascii_uppercase();
        for (axis, name) in ['X', 'Y', 'Z'].into_iter().enumerate() {
            self.homed[axis] = !clear.contains(name);
        }
        Ok(())
    }

    /// `Motion.move(newpos, speed)`: `Move` caps a kinematic move at the
    /// effective velocity, the kinematics `check_move` refuses an unhomed
    /// axis and caps the z share at `max_z_velocity`, a pure-z move is capped
    /// outright; an extrude-only move keeps its speed. `None` when klippy
    /// submits nothing, and for the move onto the anchor, which the snapshot
    /// starts from instead.
    fn move_to(
        &mut self,
        target: [f64; 4],
        speed: f64,
        names_position: bool,
        line: u32,
    ) -> Result<Option<Waypoint>, WaypointError> {
        let start = self.commanded;
        let (max_velocity, _, _) = self.runtime.effective(self.config);
        let mut velocity = speed.min(max_velocity);
        let mut delta = [0.0; 4];
        for (axis, d) in delta.iter_mut().enumerate() {
            *d = target[axis] - start[axis];
        }
        let mut end = target;
        let mut move_d = (delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]).sqrt();
        let kinematic = move_d >= MOVE_EPSILON_MM;
        if !kinematic {
            end = [start[0], start[1], start[2], target[3]];
            delta[..3].fill(0.0);
            move_d = delta[3].abs();
            velocity = speed;
        }
        if kinematic {
            if let Some(axis) = (0..3).find(|&axis| delta[axis] != 0.0 && !self.homed[axis]) {
                return Err(WaypointError::Unhomed {
                    line,
                    axis: char::from(b"XYZ"[axis]),
                });
            }
        }
        if !self.anchored {
            if !names_position {
                self.commanded = end;
                return Ok(None);
            }
            self.anchored = true;
            self.commanded = target;
            self.rest_s = None;
            return Ok(Some(Waypoint {
                position: target,
                feedrate: speed,
                runtime: self.runtime,
                dwell_before_s: None,
            }));
        }
        if move_d == 0.0 {
            return Ok(None);
        }
        let mut min_move_t = move_d / velocity;
        let max_cruise_v2 = velocity * velocity;
        if kinematic && delta[2] != 0.0 {
            let z_speed = self.config.max_z_velocity * (move_d / delta[2].abs());
            if z_speed * z_speed < max_cruise_v2 {
                min_move_t = move_d / z_speed;
            }
        }
        let mut feedrate = move_d / min_move_t;
        let pure_z = delta[2].abs() > MOVE_EPSILON_MM
            && delta[0].abs() < MOVE_EPSILON_MM
            && delta[1].abs() < MOVE_EPSILON_MM;
        if pure_z {
            feedrate = feedrate.min(self.config.max_z_velocity);
        }
        self.commanded = end;
        Ok(Some(Waypoint {
            position: end,
            feedrate,
            runtime: self.runtime,
            dwell_before_s: self.rest_s.take(),
        }))
    }
}

/// klippy's `GCodeMove` coordinate state: `last_position` is the toolhead
/// position G-code has commanded, `base_position` the origin G-code
/// coordinates are measured from. `G92` moves only the origin, so a position
/// reset never moves the toolhead.
struct GCodeMove {
    absolute_coord: bool,
    absolute_extrude: bool,
    base_position: [f64; 4],
    last_position: [f64; 4],
    speed: f64,
}

impl GCodeMove {
    /// `GCodeMove.cmd_G92`.
    fn set_position(&mut self, params: &gcode::Params) {
        let offsets = [params.x(), params.y(), params.z(), params.e()];
        if offsets.iter().all(Option::is_none) {
            self.base_position = self.last_position;
            return;
        }
        for (axis, offset) in offsets.into_iter().enumerate() {
            if let Some(offset) = offset {
                self.base_position[axis] = self.last_position[axis] - offset;
            }
        }
    }

    /// `GCodeMove.cmd_G1` up to the `move_with_transform` call: the new
    /// commanded position and speed.
    fn linear_move(
        &mut self,
        command: u32,
        params: &gcode::Params,
        line: u32,
    ) -> Result<(), WaypointError> {
        for (axis, value) in [params.x(), params.y(), params.z()].into_iter().enumerate() {
            if let Some(v) = value {
                self.last_position[axis] = if self.absolute_coord {
                    v + self.base_position[axis]
                } else {
                    self.last_position[axis] + v
                };
            }
        }
        if let Some(v) = params.e() {
            self.last_position[3] = if self.absolute_coord && self.absolute_extrude {
                v + self.base_position[3]
            } else {
                self.last_position[3] + v
            };
        }
        if let Some(f) = params.f() {
            if f <= 0.0 {
                return Err(WaypointError::InvalidSpeed {
                    line,
                    command,
                    feedrate: f,
                });
            }
            self.speed = f * LIVE_SPEED_FACTOR;
        }
        Ok(())
    }
}

/// `Motion.cmd_G4`: `P` milliseconds, at least zero.
fn g4_dwell_s(params: &gcode::Params, line: u32) -> Result<f64, WaypointError> {
    let milliseconds = params.get(b'P').unwrap_or(0.0);
    if milliseconds < 0.0 {
        return Err(WaypointError::InvalidParam {
            line,
            command: "G4".into(),
            param: "P".into(),
            value: milliseconds.to_string().into(),
            reason: "must be at least the minimum",
        });
    }
    Ok(milliseconds / 1000.0)
}

pub fn parse_gcode(text: &str, config: &CartesianLimits) -> Result<Vec<Waypoint>, WaypointError> {
    let mut waypoints: Vec<Waypoint> = Vec::new();
    let mut toolhead = Toolhead {
        config,
        runtime: RuntimeLimits::default(),
        commanded: [0.0; 4],
        anchored: false,
        homed: [true; 3],
        rest_s: None,
    };
    let mut gcode_move = GCodeMove {
        absolute_coord: true,
        absolute_extrude: true,
        base_position: [0.0; 4],
        last_position: [0.0; 4],
        speed: LIVE_DEFAULT_SPEED_MM_S,
    };

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
                    "SET_VELOCITY_LIMIT" => {
                        set_velocity_limit(&args, config, &mut toolhead.runtime)?;
                    }
                    "RESET_VELOCITY_LIMIT" => reset_velocity_limit(&mut toolhead.runtime),
                    "SET_KINEMATIC_POSITION" => {
                        toolhead.set_kinematic_position(&args)?;
                        gcode_move.last_position = toolhead.commanded;
                    }
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
            (b'G', 90) => gcode_move.absolute_coord = true,
            (b'G', 91) => gcode_move.absolute_coord = false,
            (b'M', 82) => gcode_move.absolute_extrude = true,
            (b'M', 83) => gcode_move.absolute_extrude = false,
            (b'M', 204) => m204(&params, line_no, &mut toolhead.runtime)?,
            (b'G', 92) => gcode_move.set_position(&params),
            (b'G', 4) => toolhead.rest(g4_dwell_s(&params, line_no)?),
            (b'M', 400) => toolhead.rest(0.0),
            (b'M', 18 | 84) => toolhead.motors_off(),
            (b'G', 2 | 3) => {
                return Err(WaypointError::UnsupportedArc {
                    line: line_no,
                    major,
                });
            }
            (b'G', cmd @ (0 | 1)) => {
                let has_position =
                    params.x().is_some() || params.y().is_some() || params.z().is_some();
                gcode_move.linear_move(cmd, &params, line_no)?;
                let waypoint = toolhead.move_to(
                    gcode_move.last_position,
                    gcode_move.speed,
                    has_position,
                    line_no,
                )?;
                waypoints.extend(waypoint);
            }
            _ => {}
        }
    }
    Ok(waypoints)
}

#[cfg(test)]
mod tests;
