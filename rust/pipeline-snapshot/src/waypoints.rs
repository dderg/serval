//! G-code text → absolute waypoints, mirroring what the live host submits
//! move for move, so the playground, the snapshot harness and the bench
//! analyzer agree with a printer on what a file means. E rides as the fourth
//! coordinate so retracts (E-only moves) and extruding moves flow through the
//! pipeline as followers; the engine differences consecutive E to a per-move
//! delta. Each waypoint carries the runtime limit overrides in force for the
//! move ending at it — the same [`RuntimeLimits`] state the live engine holds
//! when that move is submitted — so the snapshot derives its effective limits
//! through the live clamp.
//! Coordinates follow klippy's `GCodeMove`: absolute by default, E relative
//! under M83 or under G91 (even after M82), G92 and `SET_GCODE_OFFSET` move
//! the G-code origin, never the toolhead, M220/M221 scale speed and
//! extrusion, and `SAVE_GCODE_STATE`/`RESTORE_GCODE_STATE` keep all of it.
//! A command the snapshot cannot plan is refused by name (see `refused`);
//! any other is ignored, as the live host ignores it.

use std::collections::HashMap;

use planner_config::{CartesianLimits, RuntimeLimits};

mod refused;

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

/// The steepest extrusion any move between `waypoints` lays per millimetre
/// of travel; extrude-only moves lay none along a path.
#[must_use]
pub fn extrusion_ratio_bound(waypoints: &[Waypoint]) -> f64 {
    waypoints
        .windows(2)
        .filter_map(|pair| {
            let [from, to] = [pair[0].position, pair[1].position];
            let travel = (0..3)
                .map(|axis| (to[axis] - from[axis]).powi(2))
                .sum::<f64>()
                .sqrt();
            (travel > 0.0).then(|| (to[3] - from[3]).abs() / travel)
        })
        .fold(0.0, f64::max)
}

#[derive(Debug, thiserror::Error)]
pub enum WaypointError {
    #[error(transparent)]
    Lex(#[from] gcode::ParseError),
    #[error("line {line}: {command}: {reason}")]
    Unplannable {
        line: u32,
        command: Box<str>,
        reason: &'static str,
    },
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
    #[error("line {line}: Machine does not support G20 (inches) command")]
    InchUnits { line: u32 },
    #[error("line {line}: Unknown g-code state: {name}")]
    UnknownGcodeState { line: u32, name: Box<str> },
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

    fn int(&self, key: &str) -> Result<Option<i64>, WaypointError> {
        let Some(raw) = self.raw(key) else {
            return Ok(None);
        };
        raw.trim()
            .parse()
            .map(Some)
            .map_err(|_| WaypointError::InvalidParam {
                line: self.line,
                command: self.command.into(),
                param: key.into(),
                value: raw.into(),
                reason: "not an integer",
            })
    }
}

/// `SET_VELOCITY_LIMIT`, as `Motion.cmd_SET_VELOCITY_LIMIT` handles it.
fn set_velocity_limit(
    args: &ExtendedArgs<'_>,
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
    if square_corner_velocity.is_some() {
        runtime
            .set_square_corner_velocity(square_corner_velocity)
            .map_err(runtime_limit)?;
    }
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
/// starts at the anchor — the first position the G-code states: a
/// `SET_KINEMATIC_POSITION` or the first move that names a coordinate — so a
/// stop at rest before it is already where the plan begins.
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
    /// Before the anchor it states where the toolhead is, so it anchors the
    /// plan there.
    fn set_kinematic_position(
        &mut self,
        args: &ExtendedArgs<'_>,
        speed: f64,
    ) -> Result<Option<Waypoint>, WaypointError> {
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
        let clear = args.raw("CLEAR").unwrap_or("").to_ascii_uppercase();
        for (axis, name) in ['X', 'Y', 'Z'].into_iter().enumerate() {
            self.homed[axis] = !clear.contains(name);
        }
        if self.anchored {
            self.commanded = renamed;
            return Ok(None);
        }
        Ok(Some(self.anchor(renamed, speed)))
    }

    fn anchor(&mut self, position: [f64; 4], speed: f64) -> Waypoint {
        self.anchored = true;
        self.commanded = position;
        self.rest_s = None;
        Waypoint {
            position,
            feedrate: speed,
            runtime: self.runtime,
            dwell_before_s: None,
        }
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
            return Ok(Some(self.anchor(target, speed)));
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
/// coordinates are measured from. `G92` and `SET_GCODE_OFFSET` move only the
/// origin, so neither moves the toolhead unless asked to.
#[derive(Clone)]
struct GCodeState {
    absolute_coord: bool,
    absolute_extrude: bool,
    base_position: [f64; 4],
    last_position: [f64; 4],
    homing_position: [f64; 4],
    speed: f64,
    speed_factor: f64,
    extrude_factor: f64,
}

struct GCodeMove {
    state: GCodeState,
    saved_states: HashMap<Box<str>, GCodeState>,
}

/// A numbered command's `S` percentage, as `get_float("S", 100., above=0.)`.
fn percentage(params: &gcode::Params, command: &str, line: u32) -> Result<f64, WaypointError> {
    let value = params.get(b'S').unwrap_or(100.0);
    if value <= 0.0 {
        return Err(WaypointError::InvalidParam {
            line,
            command: command.into(),
            param: "S".into(),
            value: value.to_string().into(),
            reason: "must be above the minimum",
        });
    }
    Ok(value)
}

impl GCodeMove {
    fn new() -> Self {
        Self {
            state: GCodeState {
                absolute_coord: true,
                absolute_extrude: true,
                base_position: [0.0; 4],
                last_position: [0.0; 4],
                homing_position: [0.0; 4],
                speed: LIVE_DEFAULT_SPEED_MM_S,
                speed_factor: LIVE_SPEED_FACTOR,
                extrude_factor: 1.0,
            },
            saved_states: HashMap::new(),
        }
    }

    /// `GCodeMove.cmd_G92`.
    fn set_position(&mut self, params: &gcode::Params) {
        let s = &mut self.state;
        let offsets = [params.x(), params.y(), params.z(), params.e()];
        if offsets.iter().all(Option::is_none) {
            s.base_position = s.last_position;
            return;
        }
        for (axis, offset) in offsets.into_iter().enumerate() {
            if let Some(mut offset) = offset {
                if axis == 3 {
                    offset *= s.extrude_factor;
                }
                s.base_position[axis] = s.last_position[axis] - offset;
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
        let s = &mut self.state;
        for (axis, value) in [params.x(), params.y(), params.z()].into_iter().enumerate() {
            if let Some(v) = value {
                s.last_position[axis] = if s.absolute_coord {
                    v + s.base_position[axis]
                } else {
                    s.last_position[axis] + v
                };
            }
        }
        if let Some(e) = params.e() {
            let v = e * s.extrude_factor;
            s.last_position[3] = if s.absolute_coord && s.absolute_extrude {
                v + s.base_position[3]
            } else {
                s.last_position[3] + v
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
            s.speed = f * s.speed_factor;
        }
        Ok(())
    }

    /// `GCodeMove.cmd_M220`.
    fn set_speed_factor(&mut self, params: &gcode::Params, line: u32) -> Result<(), WaypointError> {
        let value = percentage(params, "M220", line)? / (60.0 * 100.0);
        let s = &mut self.state;
        s.speed = s.speed / s.speed_factor * value;
        s.speed_factor = value;
        Ok(())
    }

    /// `GCodeMove.cmd_M221`.
    fn set_extrude_factor(
        &mut self,
        params: &gcode::Params,
        line: u32,
    ) -> Result<(), WaypointError> {
        let new_extrude_factor = percentage(params, "M221", line)? / 100.0;
        let s = &mut self.state;
        let last_e_pos = s.last_position[3];
        let e_value = (last_e_pos - s.base_position[3]) / s.extrude_factor;
        s.base_position[3] = last_e_pos - e_value * new_extrude_factor;
        s.extrude_factor = new_extrude_factor;
        Ok(())
    }

    /// `MOVE=1` asks the toolhead onto the new position, at `MOVE_SPEED` or
    /// the current speed.
    fn requested_move_speed(&self, args: &ExtendedArgs<'_>) -> Result<Option<f64>, WaypointError> {
        if args.int("MOVE")?.unwrap_or(0) == 0 {
            return Ok(None);
        }
        Ok(Some(
            args.float("MOVE_SPEED", Bound::Above(0.0))?
                .unwrap_or(self.state.speed),
        ))
    }

    /// `GCodeMove.cmd_SET_GCODE_OFFSET`; `Some(speed)` when it moves.
    fn set_gcode_offset(&mut self, args: &ExtendedArgs<'_>) -> Result<Option<f64>, WaypointError> {
        let mut move_delta = [0.0; 4];
        for (axis, key) in ["X", "Y", "Z", "E"].into_iter().enumerate() {
            let homing = self.state.homing_position[axis];
            let offset = match args.float(key, Bound::Any)? {
                Some(offset) => offset,
                None => match args.float(&format!("{key}_ADJUST"), Bound::Any)? {
                    Some(adjust) => adjust + homing,
                    None => continue,
                },
            };
            let delta = offset - homing;
            move_delta[axis] = delta;
            self.state.base_position[axis] += delta;
            self.state.homing_position[axis] = offset;
        }
        let speed = self.requested_move_speed(args)?;
        if speed.is_some() {
            for (last, delta) in self.state.last_position.iter_mut().zip(move_delta) {
                *last += delta;
            }
        }
        Ok(speed)
    }

    fn state_name<'a>(args: &'a ExtendedArgs<'_>) -> &'a str {
        args.raw("NAME").unwrap_or("default")
    }

    /// `GCodeMove.cmd_SAVE_GCODE_STATE`.
    fn save_state(&mut self, args: &ExtendedArgs<'_>) {
        self.saved_states
            .insert(Self::state_name(args).into(), self.state.clone());
    }

    /// `GCodeMove.cmd_RESTORE_GCODE_STATE`; `Some(speed)` when it moves.
    fn restore_state(&mut self, args: &ExtendedArgs<'_>) -> Result<Option<f64>, WaypointError> {
        let name = Self::state_name(args);
        let saved = self
            .saved_states
            .get(name)
            .ok_or_else(|| WaypointError::UnknownGcodeState {
                line: args.line,
                name: name.into(),
            })?
            .clone();
        let last_position = self.state.last_position;
        self.state = GCodeState {
            last_position,
            ..saved.clone()
        };
        self.state.base_position[3] += last_position[3] - saved.last_position[3];
        let speed = self.requested_move_speed(args)?;
        if speed.is_some() {
            self.state.last_position[..3].copy_from_slice(&saved.last_position[..3]);
        }
        Ok(speed)
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

/// The live host's G-code handlers over the snapshot's toolhead: each
/// command a handler would act on is mirrored, each the snapshot cannot plan
/// is refused by name, and the rest are ignored as live ignores them —
/// their arguments are never read, so `M117 Hello` is as harmless here.
struct Interpreter<'a> {
    toolhead: Toolhead<'a>,
    gcode_move: GCodeMove,
    waypoints: Vec<Waypoint>,
}

impl Interpreter<'_> {
    fn submit(&mut self, speed: f64, names_position: bool, line: u32) -> Result<(), WaypointError> {
        let waypoint = self.toolhead.move_to(
            self.gcode_move.state.last_position,
            speed,
            names_position,
            line,
        )?;
        self.waypoints.extend(waypoint);
        Ok(())
    }

    fn numbered(
        &mut self,
        line: &gcode::Line<'_>,
        letter: u8,
        major: u32,
        minor: Option<u32>,
    ) -> Result<(), WaypointError> {
        let line_no = line.line_no;
        if let Some(reason) = refused::numbered(letter, major, minor, self.toolhead.anchored) {
            let minor = minor.map(|m| format!(".{m}")).unwrap_or_default();
            return Err(WaypointError::Unplannable {
                line: line_no,
                command: format!("{}{major}{minor}", char::from(letter)).into(),
                reason,
            });
        }
        let state = &mut self.gcode_move.state;
        match (letter, major, minor) {
            (b'G', 90, None) => state.absolute_coord = true,
            (b'G', 91, None) => state.absolute_coord = false,
            (b'M', 82, None) => state.absolute_extrude = true,
            (b'M', 83, None) => state.absolute_extrude = false,
            (b'G', 20, None) => return Err(WaypointError::InchUnits { line: line_no }),
            (b'M', 220, None) => self.gcode_move.set_speed_factor(&line.params()?, line_no)?,
            (b'M', 221, None) => self
                .gcode_move
                .set_extrude_factor(&line.params()?, line_no)?,
            (b'M', 204, None) => m204(&line.params()?, line_no, &mut self.toolhead.runtime)?,
            (b'G', 92, None) => self.gcode_move.set_position(&line.params()?),
            (b'G', 4, None) => self.toolhead.rest(g4_dwell_s(&line.params()?, line_no)?),
            (b'M', 400, None) => self.toolhead.rest(0.0),
            (b'M', 18 | 84, None) => self.toolhead.motors_off(),
            (b'G', command @ (0 | 1), None) => {
                let params = line.params()?;
                let names_position =
                    params.x().is_some() || params.y().is_some() || params.z().is_some();
                self.gcode_move.linear_move(command, &params, line_no)?;
                self.submit(self.gcode_move.state.speed, names_position, line_no)?;
            }
            _ => {}
        }
        Ok(())
    }

    fn extended(&mut self, line: &gcode::Line<'_>, name: &str) -> Result<(), WaypointError> {
        let line_no = line.line_no;
        if let Some(reason) = refused::extended(name, self.toolhead.anchored) {
            return Err(WaypointError::Unplannable {
                line: line_no,
                command: name.into(),
                reason,
            });
        }
        if !MIRRORED_EXTENDED.contains(&name) {
            return Ok(());
        }
        let raw_args = line.extended_args()?;
        let args = ExtendedArgs {
            command: name,
            line: line_no,
            args: &raw_args,
        };
        let move_speed = match name {
            "SET_VELOCITY_LIMIT" => {
                set_velocity_limit(&args, &mut self.toolhead.runtime)?;
                None
            }
            "RESET_VELOCITY_LIMIT" => {
                reset_velocity_limit(&mut self.toolhead.runtime);
                None
            }
            "SET_KINEMATIC_POSITION" => {
                let anchor = self
                    .toolhead
                    .set_kinematic_position(&args, self.gcode_move.state.speed)?;
                self.waypoints.extend(anchor);
                self.gcode_move.state.last_position = self.toolhead.commanded;
                None
            }
            "SET_GCODE_OFFSET" => self.gcode_move.set_gcode_offset(&args)?,
            "SAVE_GCODE_STATE" => {
                self.gcode_move.save_state(&args);
                None
            }
            "RESTORE_GCODE_STATE" => self.gcode_move.restore_state(&args)?,
            _ => unreachable!("{name} is listed as mirrored but has no handler"),
        };
        if let Some(speed) = move_speed {
            self.submit(speed, true, line_no)?;
        }
        Ok(())
    }
}

const MIRRORED_EXTENDED: [&str; 6] = [
    "SET_VELOCITY_LIMIT",
    "RESET_VELOCITY_LIMIT",
    "SET_KINEMATIC_POSITION",
    "SET_GCODE_OFFSET",
    "SAVE_GCODE_STATE",
    "RESTORE_GCODE_STATE",
];

pub fn parse_gcode(text: &str, config: &CartesianLimits) -> Result<Vec<Waypoint>, WaypointError> {
    let mut interpreter = Interpreter {
        toolhead: Toolhead {
            config,
            runtime: RuntimeLimits::default(),
            commanded: [0.0; 4],
            anchored: false,
            homed: [true; 3],
            rest_s: None,
        },
        gcode_move: GCodeMove::new(),
        waypoints: Vec::new(),
    };
    for line in gcode::lines(text) {
        let line = line?;
        match &line.head {
            gcode::Head::Numbered {
                letter,
                major,
                minor,
            } => interpreter.numbered(&line, *letter, *major, *minor)?,
            gcode::Head::Extended(name) => interpreter.extended(&line, name)?,
        }
    }
    Ok(interpreter.waypoints)
}

#[cfg(test)]
mod tests;
