//! Read the planner's config sections straight from a parsed config
//! [`Document`]: `[printer]` cartesian limits, `[axis *]`, `[limit *]`,
//! `[post_processor *]`, and the `[extruder]` extrude-only caps.
//!
//! This is the single source of the option names, defaults, bounds, and
//! error texts that klippy's Python `motion_setup` readers used to hold.
//! Every option consumed (including defaults taken) is reported so the
//! host can feed its access-tracking / unused-option accounting.

use config_doc::Document;

use crate::{AxisDecl, CartesianLimits, PostProcessorDecl};

const DEFAULT_SQUARE_CORNER_VELOCITY: f64 = 5.0;
const UNSUPPORTED_PRINTER_KEYS: [&str; 2] = ["max_accel_to_decel", "minimum_cruise_ratio"];
const LEGACY_STEPPER_AXES: [char; 5] = ['x', 'y', 'z', 'a', 'b'];
const LEGACY_SERVO_SECTIONS: [&str; 3] = ["servo_x", "servo_y", "servo_z"];
const MOTION_SLOT_COUNT: usize = 4;
const KINEMATICS_ROLES: [(&str, [(&str, &str); 3]); 2] = [
    (
        "cartesian",
        [
            ("x_motors", "axis_x"),
            ("y_motors", "axis_y"),
            ("z_motors", "axis_z"),
        ],
    ),
    (
        "corexy",
        [
            ("a_motors", "axis_x"),
            ("b_motors", "axis_y"),
            ("z_motors", "axis_z"),
        ],
    ),
];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Drive {
    Stepper,
    Servo,
}

impl Drive {
    pub fn as_str(self) -> &'static str {
        match self {
            Drive::Stepper => "stepper",
            Drive::Servo => "servo",
        }
    }
}

/// One kinematics lane: a motion slot bound to an `[axis <name>]` section
/// and driven by one or more same-drive motors.
#[derive(Debug, Clone)]
pub struct LaneDecl {
    pub lane_idx: usize,
    pub axis: String,
    pub motors: Vec<String>,
    pub drive: Drive,
}

/// An `[axis <name>]` with `motors:` that no kinematics role claims: it
/// rides a free motion slot (extruders and other followers).
#[derive(Debug, Clone)]
pub struct FollowerDecl {
    pub axis: String,
    pub motors: Vec<String>,
    pub slot: usize,
}

#[derive(Debug, Clone)]
pub struct KinematicsDecl {
    pub kind: String,
    pub lanes: Vec<LaneDecl>,
    pub followers: Vec<FollowerDecl>,
}

impl KinematicsDecl {
    pub fn claimed_axes(&self) -> Vec<String> {
        self.lanes.iter().map(|l| l.axis.clone()).collect()
    }
}

/// A `[limit <name>]` section with axes still by name; they resolve to
/// indices against the `AxisRegistry` at planner-init time.
#[derive(Debug, Clone)]
pub struct LimitDecl {
    pub name: String,
    pub axes: Vec<String>,
    pub max_velocity: Option<f64>,
    pub max_accel: Option<f64>,
    pub max_jerk: Option<f64>,
}

#[derive(Debug, Clone)]
pub struct MotionSettings {
    /// `None` when the config has no `[kinematics]` section — legal for
    /// planner-only harnesses (snapshots, viz); a live printer requires it.
    pub kinematics: Option<KinematicsDecl>,
    pub axes: Vec<AxisDecl>,
    pub limits: Vec<LimitDecl>,
    pub post_processors: Vec<PostProcessorDecl>,
    pub cartesian: CartesianLimits,
    pub fit_tolerance_mm: f64,
    pub fit_tolerance_accel_mm_s2: f64,
    pub max_extrude_only_velocity: Option<f64>,
    pub max_extrude_only_accel: Option<f64>,
}

impl MotionSettings {
    /// The lowest `[limit]` acceleration covering `axis`, if any covers it.
    pub fn axis_accel_cap(&self, axis: &str) -> Option<f64> {
        self.limits
            .iter()
            .filter(|l| l.max_accel.is_some() && l.axes.iter().any(|a| a == axis))
            .filter_map(|l| l.max_accel)
            .fold(None, |acc, a| Some(acc.map_or(a, |m: f64| m.min(a))))
    }
}

/// One option this reader consumed, echoed back for access tracking.
/// `value` carries the parsed value for float options and the raw text
/// otherwise, mirroring what klippy's `_get_wrapper` records.
#[derive(Debug, Clone, PartialEq)]
pub enum ConsumedValue {
    Float(f64),
    Text(String),
}

#[derive(Debug, Clone, PartialEq)]
pub struct ConsumedOption {
    pub section: String,
    pub option: String,
    pub value: ConsumedValue,
}

pub fn read_motion_settings(
    doc: &Document,
) -> Result<(MotionSettings, Vec<ConsumedOption>), String> {
    reject_unsupported_sections(doc)?;
    let mut reader = Reader {
        doc,
        consumed: Vec::new(),
    };

    let (cartesian, fit_tolerance_mm, fit_tolerance_accel_mm_s2) = reader.printer_section()?;
    let axes = reader.axis_sections()?;
    let kinematics = reader.kinematics_section(&axes)?;
    let limits = reader.limit_sections(&axes)?;
    let post_processors = reader.post_processor_sections(&axes)?;
    let (max_extrude_only_velocity, max_extrude_only_accel) = reader.extruder_caps()?;

    Ok((
        MotionSettings {
            kinematics,
            axes,
            limits,
            post_processors,
            cartesian,
            fit_tolerance_mm,
            fit_tolerance_accel_mm_s2,
            max_extrude_only_velocity,
            max_extrude_only_accel,
        },
        reader.consumed,
    ))
}

fn is_legacy_stepper_role_section(name: &str) -> bool {
    let Some(suffix) = name.strip_prefix("stepper_") else {
        return false;
    };
    let mut chars = suffix.chars();
    let Some(first) = chars.next() else {
        return false;
    };
    if !LEGACY_STEPPER_AXES.contains(&first) {
        return false;
    }
    let rest = chars.as_str();
    rest.is_empty() || rest.chars().all(|c| c.is_ascii_digit())
}

fn reject_unsupported_sections(doc: &Document) -> Result<(), String> {
    for name in doc.section_names() {
        if is_legacy_stepper_role_section(name) {
            return Err("role-encoding motor sections are not supported: name the \
                 motor freely (e.g. [motor a]) and assign it in [kinematics] \
                 role lists / [axis <name>] motors:"
                .to_owned());
        }
    }
    for name in LEGACY_SERVO_SECTIONS {
        if doc.has_section(name) {
            return Err("role-encoding servo sections are not supported: declare a \
                 [<motor>] section with 'drive: servo' and assign it in \
                 [kinematics]"
                .to_owned());
        }
    }
    if doc.has_section("firmware_retraction") {
        return Err("[firmware_retraction] is not supported: it presupposes an \
             extruder concept the motion system does not have"
            .to_owned());
    }
    if doc.has_section("input_shaper") {
        return Err("[input_shaper] is not supported: declare [post_processor \
             <name>] sections and reference them from [axis] \
             post_processors"
            .to_owned());
    }
    Ok(())
}

/// Sections with a `<prefix> ` name, as (verbatim-section-name, suffix)
/// pairs — the suffix is everything after the first whitespace run,
/// matching Python's `name.split(None, 1)[1]`.
fn prefix_sections<'d>(doc: &'d Document, prefix: &str) -> Vec<(&'d str, &'d str)> {
    doc.section_names()
        .filter(|n| n.starts_with(prefix))
        .filter_map(|n| {
            let (_, rest) = n.split_once(char::is_whitespace)?;
            Some((n, rest.trim_start()))
        })
        .collect()
}

fn split_list(value: &str) -> Vec<String> {
    if value.trim().is_empty() {
        return Vec::new();
    }
    value.split(',').map(|p| p.trim().to_owned()).collect()
}

struct Reader<'d> {
    doc: &'d Document,
    consumed: Vec<ConsumedOption>,
}

impl Reader<'_> {
    fn record(&mut self, section: &str, option: &str, value: ConsumedValue) {
        self.consumed.push(ConsumedOption {
            section: section.to_owned(),
            option: option.to_owned(),
            value,
        });
    }

    /// The interpolated value, with `${}` references recorded exactly like
    /// klippy's access tracking does (as-written names, resolved values).
    fn fetch(&mut self, section: &str, option: &str) -> Result<Option<String>, String> {
        if !self.doc.has_option(section, option) {
            return Ok(None);
        }
        let (value, refs) = self.doc.get(section, option).map_err(|e| e.to_string())?;
        for r in refs {
            self.record(&r.section, &r.option, ConsumedValue::Text(r.value));
        }
        Ok(Some(value))
    }

    fn get_str(&mut self, section: &str, option: &str) -> Result<Option<String>, String> {
        let Some(value) = self.fetch(section, option)? else {
            return Ok(None);
        };
        self.record(section, option, ConsumedValue::Text(value.clone()));
        Ok(Some(value))
    }

    fn get_list(&mut self, section: &str, option: &str) -> Result<Option<Vec<String>>, String> {
        Ok(self.get_str(section, option)?.map(|v| split_list(&v)))
    }

    fn getfloat(
        &mut self,
        section: &str,
        option: &str,
        bounds: Bounds,
    ) -> Result<Option<f64>, String> {
        let Some(raw) = self.fetch(section, option)? else {
            return Ok(None);
        };
        let v: f64 = raw
            .trim()
            .parse()
            .map_err(|_| format!("Unable to parse option '{option}' in section '{section}'"))?;
        if !v.is_finite() {
            return Err(format!(
                "Unable to parse option '{option}' in section '{section}'"
            ));
        }
        self.record(section, option, ConsumedValue::Float(v));
        bounds.check(section, option, v)?;
        Ok(Some(v))
    }

    fn getfloat_or(
        &mut self,
        section: &str,
        option: &str,
        default: f64,
        bounds: Bounds,
    ) -> Result<f64, String> {
        match self.getfloat(section, option, bounds)? {
            Some(v) => Ok(v),
            None => {
                self.record(section, option, ConsumedValue::Float(default));
                Ok(default)
            }
        }
    }

    fn getfloat_required(
        &mut self,
        section: &str,
        option: &str,
        bounds: Bounds,
    ) -> Result<f64, String> {
        self.getfloat(section, option, bounds)?
            .ok_or_else(|| format!("Option '{option}' in section '{section}' must be specified"))
    }

    fn printer_section(&mut self) -> Result<(CartesianLimits, f64, f64), String> {
        for key in UNSUPPORTED_PRINTER_KEYS {
            if self.doc.has_option("printer", key) {
                return Err(format!("[printer] {key} is not supported"));
            }
        }
        let max_velocity = self.getfloat_required("printer", "max_velocity", Bounds::above(0.0))?;
        let max_accel = self.getfloat_required("printer", "max_accel", Bounds::above(0.0))?;

        let scv = self.getfloat("printer", "square_corner_velocity", Bounds::min(0.0))?;
        let corner_deviation = self.getfloat("printer", "corner_deviation", Bounds::min(0.0))?;
        let corner_deviation = match (scv, corner_deviation) {
            (Some(_), Some(_)) => {
                return Err(
                    "[printer] square_corner_velocity and corner_deviation are both \
                     set — corner_deviation is the canonical corner budget and \
                     square_corner_velocity is its legacy alias; set exactly one"
                        .to_owned(),
                );
            }
            (_, Some(deviation)) => deviation,
            (scv, None) => {
                let scv = scv.unwrap_or(DEFAULT_SQUARE_CORNER_VELOCITY);
                geometry::corner_deviation_from_scv(scv, max_accel)
            }
        };

        let max_jerk =
            self.getfloat_or("printer", "max_jerk", max_accel * 2.0, Bounds::min(0.0))?;
        let max_jerk = if max_jerk > 0.0 {
            max_jerk
        } else {
            f64::INFINITY
        };

        let max_z_velocity = self.getfloat_or(
            "printer",
            "max_z_velocity",
            max_velocity,
            Bounds::above(0.0).max(max_velocity),
        )?;
        let max_z_accel = self.getfloat_or(
            "printer",
            "max_z_accel",
            max_accel,
            Bounds::above(0.0).max(max_accel),
        )?;
        let fit_tolerance_mm = self.getfloat_or(
            "printer",
            "max_path_deviation",
            0.005,
            Bounds::above(0.0).max(1.0),
        )?;
        let fit_tolerance_accel_mm_s2 =
            self.getfloat_or("printer", "max_accel_deviation", 50.0, Bounds::above(0.0))?;

        Ok((
            CartesianLimits {
                max_velocity,
                max_accel,
                max_jerk,
                max_z_velocity,
                max_z_accel,
                corner_deviation,
            },
            fit_tolerance_mm,
            fit_tolerance_accel_mm_s2,
        ))
    }

    fn get_str_required(&mut self, section: &str, option: &str) -> Result<String, String> {
        self.get_str(section, option)?
            .ok_or_else(|| format!("Option '{option}' in section '{section}' must be specified"))
    }

    fn motor_drive(&mut self, motor: &str, referenced_by: &str) -> Result<Drive, String> {
        let section = format!("motor {motor}");
        if !self.doc.has_section(&section) {
            return Err(format!(
                "{referenced_by} references motor '{motor}' but no [motor {motor}] \
                 section exists"
            ));
        }
        let drive = self.get_str_required(&section, "drive")?;
        match drive.as_str() {
            "stepper" => Ok(Drive::Stepper),
            "servo" => Ok(Drive::Servo),
            other => Err(format!(
                "Choice '{other}' for option 'drive' in section '{section}' is not a \
                 valid choice"
            )),
        }
    }

    fn kinematics_section(&mut self, axes: &[AxisDecl]) -> Result<Option<KinematicsDecl>, String> {
        if self.doc.has_option("printer", "kinematics") {
            return Err(
                "[printer] kinematics is not supported: declare a [kinematics] \
                 section (type + axis role bindings + motor lists)"
                    .to_owned(),
            );
        }
        if !self.doc.has_section("kinematics") {
            return Ok(None);
        }
        let kind = self.get_str_required("kinematics", "type")?;
        let roles = KINEMATICS_ROLES
            .iter()
            .find(|(k, _)| *k == kind)
            .map(|(_, roles)| roles)
            .ok_or_else(|| {
                format!(
                    "[kinematics] type '{kind}' is not supported (supported: {})",
                    KINEMATICS_ROLES.map(|(k, _)| k).join(", ")
                )
            })?;

        let mut lanes = Vec::new();
        for (lane_idx, (role_motors_key, axis_role_key)) in roles.iter().enumerate() {
            let axis = self.get_str_required("kinematics", axis_role_key)?;
            if !axes.iter().any(|a| a.name == axis) {
                return Err(format!(
                    "[kinematics] {axis_role_key} binds to axis '{axis}' but no \
                     [axis {axis}] section exists"
                ));
            }
            let motors = self
                .get_list("kinematics", role_motors_key)?
                .unwrap_or_default();
            if motors.is_empty() {
                return Err(format!(
                    "[kinematics] {role_motors_key} declares no motors (lane {lane_idx} \
                     needs at least one motor)"
                ));
            }
            let referenced_by = format!("[kinematics] {role_motors_key}");
            let mut drive = None;
            for motor in &motors {
                let motor_drive = self.motor_drive(motor, &referenced_by)?;
                if *drive.get_or_insert(motor_drive) != motor_drive {
                    return Err(format!(
                        "{referenced_by} mixes stepper and servo motors in one lane; a \
                         lane must be all-stepper or all-servo"
                    ));
                }
            }
            lanes.push(LaneDecl {
                lane_idx,
                axis,
                motors,
                drive: drive.expect("motors is non-empty"),
            });
        }

        let followers = self.follower_decls(axes, &lanes)?;
        self.reject_orphan_motors(axes, &lanes)?;
        Ok(Some(KinematicsDecl {
            kind,
            lanes,
            followers,
        }))
    }

    fn follower_decls(
        &mut self,
        axes: &[AxisDecl],
        lanes: &[LaneDecl],
    ) -> Result<Vec<FollowerDecl>, String> {
        let follower_axes: Vec<&AxisDecl> = axes
            .iter()
            .filter(|a| !lanes.iter().any(|l| l.axis == a.name) && !a.motors.is_empty())
            .collect();
        let free_slots: Vec<usize> = (0..MOTION_SLOT_COUNT)
            .filter(|slot| !lanes.iter().any(|l| l.lane_idx == *slot))
            .collect();
        if follower_axes.len() > free_slots.len() {
            return Err(format!(
                "{} follower axes declared but only {} motion slot(s) free of \
                 kinematics lanes",
                follower_axes.len(),
                free_slots.len()
            ));
        }
        let mut followers = Vec::new();
        for (axis, slot) in follower_axes.into_iter().zip(free_slots) {
            let referenced_by = format!("[axis {}] motors", axis.name);
            for motor in &axis.motors {
                let drive = self.motor_drive(motor, &referenced_by)?;
                if drive != Drive::Stepper {
                    return Err(format!(
                        "[axis {}] motors references '{motor}' with drive: {} — \
                         follower axes support stepper motors only",
                        axis.name,
                        drive.as_str()
                    ));
                }
            }
            followers.push(FollowerDecl {
                axis: axis.name.clone(),
                motors: axis.motors.clone(),
                slot,
            });
        }
        Ok(followers)
    }

    fn reject_orphan_motors(
        &mut self,
        axes: &[AxisDecl],
        lanes: &[LaneDecl],
    ) -> Result<(), String> {
        let consumed: std::collections::HashSet<&str> = lanes
            .iter()
            .flat_map(|l| l.motors.iter())
            .chain(axes.iter().flat_map(|a| a.motors.iter()))
            .map(String::as_str)
            .collect();
        let mut orphans: Vec<&str> = prefix_sections(self.doc, "motor ")
            .into_iter()
            .map(|(_, name)| name)
            .filter(|name| !consumed.contains(name))
            .collect();
        orphans.sort_unstable();
        if orphans.is_empty() {
            return Ok(());
        }
        Err(format!(
            "motor(s) {} declared but not assigned to any axis (reference them from a \
             [kinematics] role list or [axis <name>] motors:)",
            orphans
                .iter()
                .map(|m| format!("[motor {m}]"))
                .collect::<Vec<_>>()
                .join(", ")
        ))
    }

    fn axis_sections(&mut self) -> Result<Vec<AxisDecl>, String> {
        let mut axes = Vec::new();
        for (section, name) in prefix_sections(self.doc, "axis ") {
            let section = section.to_owned();
            let follows = self
                .get_list(&section, "follows")?
                .unwrap_or_default()
                .into_iter()
                .map(|a| a.to_lowercase())
                .collect();
            let motors = self.get_list(&section, "motors")?.unwrap_or_default();
            let post_processors = self
                .get_list(&section, "post_processors")?
                .unwrap_or_default();
            axes.push(AxisDecl {
                name: name.to_owned(),
                follows,
                motors,
                post_processors,
            });
        }
        Ok(axes)
    }

    fn limit_sections(&mut self, axes: &[AxisDecl]) -> Result<Vec<LimitDecl>, String> {
        let mut limits = Vec::new();
        for (section, name) in prefix_sections(self.doc, "limit ") {
            let section = section.to_owned();
            let limit_axes: Vec<String> = self
                .get_list(&section, "axes")?
                .ok_or_else(|| format!("Option 'axes' in section '{section}' must be specified"))?
                .into_iter()
                .map(|a| a.to_lowercase())
                .collect();
            for axis in &limit_axes {
                if !axes.iter().any(|a| &a.name == axis) {
                    return Err(format!(
                        "[limit] references undeclared axis '{axis}' (declare [axis {axis}])"
                    ));
                }
            }
            let max_velocity = self.getfloat(&section, "max_velocity", Bounds::above(0.0))?;
            let max_accel = self.getfloat(&section, "max_accel", Bounds::above(0.0))?;
            let max_jerk = self
                .getfloat(&section, "max_jerk", Bounds::min(0.0))?
                .map(|j| if j > 0.0 { j } else { f64::INFINITY });
            limits.push(LimitDecl {
                name: name.to_owned(),
                axes: limit_axes,
                max_velocity,
                max_accel,
                max_jerk,
            });
        }
        Ok(limits)
    }

    fn post_processor_sections(
        &mut self,
        axes: &[AxisDecl],
    ) -> Result<Vec<PostProcessorDecl>, String> {
        let mut decls = Vec::new();
        for (section, name) in prefix_sections(self.doc, "post_processor ") {
            let section = section.to_owned();
            let ty = self
                .get_str(&section, "type")?
                .ok_or_else(|| format!("Option 'type' in section '{section}' must be specified"))?;
            let mut params = Vec::new();
            for option in self.doc.options(&section).map_err(|e| e.to_string())? {
                if option == "type" {
                    continue;
                }
                let value = self
                    .getfloat(&section, &option, Bounds::none())?
                    .expect("iterating existing options");
                params.push((option, value));
            }
            decls.push(PostProcessorDecl {
                name: name.to_owned(),
                ty,
                params,
            });
        }
        for axis in axes {
            for reference in &axis.post_processors {
                if !decls.iter().any(|d| &d.name == reference) {
                    return Err(format!(
                        "[axis {}] references undeclared post_processor \
                         '{reference}' (declare [post_processor {reference}])",
                        axis.name
                    ));
                }
            }
        }
        Ok(decls)
    }

    fn extruder_caps(&mut self) -> Result<(Option<f64>, Option<f64>), String> {
        if !self.doc.has_section("extruder") {
            return Ok((None, None));
        }
        let velocity =
            self.getfloat("extruder", "max_extrude_only_velocity", Bounds::above(0.0))?;
        let accel = self.getfloat("extruder", "max_extrude_only_accel", Bounds::above(0.0))?;
        Ok((velocity, accel))
    }
}

/// klippy `_get_wrapper` bound checks, with its exact error wording.
#[derive(Default, Clone, Copy)]
struct Bounds {
    minval: Option<f64>,
    maxval: Option<f64>,
    above: Option<f64>,
}

impl Bounds {
    fn none() -> Self {
        Self::default()
    }
    fn min(v: f64) -> Self {
        Self {
            minval: Some(v),
            ..Self::default()
        }
    }
    fn above(v: f64) -> Self {
        Self {
            above: Some(v),
            ..Self::default()
        }
    }
    fn max(self, v: f64) -> Self {
        Self {
            maxval: Some(v),
            ..self
        }
    }

    fn check(self, section: &str, option: &str, v: f64) -> Result<(), String> {
        if let Some(minval) = self.minval {
            if v < minval {
                return Err(format!(
                    "Option '{option}' in section '{section}' must have minimum of {minval}"
                ));
            }
        }
        if let Some(maxval) = self.maxval {
            if v > maxval {
                return Err(format!(
                    "Option '{option}' in section '{section}' must have maximum of {maxval}"
                ));
            }
        }
        if let Some(above) = self.above {
            if v <= above {
                return Err(format!(
                    "Option '{option}' in section '{section}' must be above {above}"
                ));
            }
        }
        Ok(())
    }
}
