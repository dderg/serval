//! The planned extruder against the limits of the config it was planned
//! under. While the toolhead moves, e holds the `[axis e]` `max_velocity` /
//! `max_accel` its advance and ratio ramps are planned within; while it
//! rests, the extrude-only caps retracts and primes are planned under. Its
//! acceleration may leave the exact signal's by what the fit promises, and
//! its velocity steps only by rounding, bar what a kernel-free advance makes
//! of an unshaped leader's acceleration steps, which
//! [`motion_pipeline::follower_velocity_step_allowance`] bounds.

use std::path::Path;

use pipeline_snapshot::SampleSide;
use pipeline_snapshot::waypoints::extrusion_ratio_bound;

use super::{motion_settings, waypoints};

const E_AXIS: usize = 3;
const SPATIAL_AXES: [&str; 3] = ["x", "y", "z"];
const E_PROBES_PER_INTERVAL: usize = 16;
const TOOLHEAD_REST_MM_S: f64 = 1e-9;
const VELOCITY_SLACK: f64 = 1e-6;
const ROW_JOIN_S: f64 = 1e-9;
/// The shaped speed under which a speed minimum is a valley: excesses
/// within its reach belong to the ratio ramp's ends and near-reversals,
/// not to the path's turning.
const VALLEY_SPEED_MM_S: f64 = 2.0;
const VALLEY_PROBE_STEP_S: f64 = 2.5e-4;
const VALLEY_PROBES_EACH_SIDE: i32 = 60;
const VALLEY_REACH_FLOOR_S: f64 = 1e-3;
/// The shaped speed under which the path's heading, and so its turning,
/// is not read.
const HEADING_MIN_SPEED_MM_S: f64 = 1.0;

#[derive(Debug, Clone, Copy)]
struct Caps {
    velocity: f64,
    accel: f64,
}

impl Caps {
    const UNLIMITED: Self = Self {
        velocity: f64::INFINITY,
        accel: f64::INFINITY,
    };

    fn is_unlimited(self) -> bool {
        self.velocity.is_infinite() && self.accel.is_infinite()
    }
}

/// What the toolhead does while the extruder is probed. A rest transition
/// is a rest within the extruder chain's own window of motion, where its
/// kernel or ratio ramp moves the extruder for that motion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ToolheadPhase {
    Moving,
    RestTransition,
    Resting,
}

/// What the plan may ask of the extruder: the follower axis's caps while
/// the toolhead moves, rest transitions included, the extrude-only caps the
/// planner plans retracts and primes under while it rests, and the largest
/// velocity step it may take at once.
#[derive(Debug, Clone, Copy)]
struct ExtruderLimits {
    moving: Caps,
    extrude_only: Caps,
    accel_slack: AccelSlack,
    transition_window_s: f64,
    step_allowance: StepAllowance,
    advance: Option<KernelFreeLinearAdvance>,
}

/// A linear pressure advance no smoothing kernel follows: the extruder
/// lays `r·(s + k·ṡ)` of the shaped path `s`, so the path alone asks it
/// for `r·(s̈ + k·s⃛)`, with `s⃛ = j_t + a⊥²/v` along the path.
#[derive(Debug, Clone, Copy)]
struct KernelFreeLinearAdvance {
    k: f64,
    ratio_bound: f64,
}

/// What an acceleration excess while the toolhead moves is attributed to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AccelCause {
    /// The shaped path's turning under a kernel-free linear advance:
    /// `r·(a_t + k·(j_t + a⊥²/v))` leaves the cap at the ratio `r` the
    /// extruder's own velocity reads, `r·(a_t + k·j_t)` holds it, and no
    /// valley is within reach.
    PathTurning,
    Unattributed,
}

/// The acceleration the extruder's fit may add to the exact signal over a
/// piece, as the fit promises it: `max_accel_deviation`, and where the
/// motor reads a kernel-free advance's source fit, the acceleration that
/// spends the follower's position tolerance over the piece
/// (`AccelBudget::PositionEquivalent`), whichever is looser.
#[derive(Debug, Clone, Copy)]
struct AccelSlack {
    absolute: f64,
    position_equivalent_mm: Option<f64>,
}

impl AccelSlack {
    fn over_piece(self, duration_s: f64) -> f64 {
        self.position_equivalent_mm
            .map_or(self.absolute, |position| {
                self.absolute
                    .max(8.0 * position / (duration_s * duration_s))
            })
    }
}

/// The smallest share of `max_path_deviation` a follower's fit is held to,
/// motion-pipeline's `FOLLOWER_TOL_SCALE_MIN`: the strictest position
/// tolerance any piece's acceleration promise is read from.
const FOLLOWER_POSITION_SCALE_FLOOR: f64 = 1e-2;

/// A velocity step is an unbounded acceleration, so where the follower has
/// an acceleration limit only rounding may step its velocity, bar the steps
/// a kernel-free advance makes of an unshaped leader's acceleration steps.
#[derive(Debug, Clone, Copy)]
enum StepAllowance {
    Unlimited,
    Rounding,
    UnshapedLeaderStep(f64),
}

/// Where the plan's extruder leaves its limits.
#[derive(Debug, Clone, Copy)]
pub enum ExtruderExcess {
    Velocity {
        t: f64,
        value: f64,
        bound: f64,
        phase: ToolheadPhase,
    },
    Acceleration {
        t: f64,
        value: f64,
        bound: f64,
        phase: ToolheadPhase,
        cause: AccelCause,
    },
    VelocityStep {
        t: f64,
        step: f64,
        allowance: f64,
    },
}

impl ExtruderExcess {
    pub fn t(&self) -> f64 {
        match *self {
            Self::Velocity { t, .. }
            | Self::Acceleration { t, .. }
            | Self::VelocityStep { t, .. } => t,
        }
    }
}

/// A velocity step a kernel-free advance makes of an unshaped leader's
/// acceleration step, within the allowance
/// [`motion_pipeline::follower_velocity_step_allowance`] grants it: exempt
/// from the acceleration limit, which it breaks without bound, and held to
/// the allowance instead.
#[derive(Debug, Clone, Copy)]
pub struct UnshapedLeaderVelocityStep {
    pub t: f64,
    pub step: f64,
    pub allowance: f64,
}

#[derive(Debug, Default)]
pub struct ExtruderAudit {
    pub excesses: Vec<ExtruderExcess>,
    pub exempt_steps: Vec<UnshapedLeaderVelocityStep>,
    pub peak_velocity: f64,
    pub peak_accel: f64,
}

impl ExtruderAudit {
    pub fn summary(&self) -> String {
        let largest_step = self
            .exempt_steps
            .iter()
            .map(|step| step.step)
            .fold(0.0, f64::max);
        format!(
            "e peaks {:.2} mm/s, {:.0} mm/s²; {} unshaped-leader velocity steps within their \
             allowance, largest {largest_step:.3} mm/s",
            self.peak_velocity,
            self.peak_accel,
            self.exempt_steps.len()
        )
    }

    fn check(
        &mut self,
        limits: &ExtruderLimits,
        (t, phase): (f64, ToolheadPhase),
        accel_slack: f64,
        velocity: f64,
        accel: f64,
    ) {
        self.peak_velocity = self.peak_velocity.max(velocity);
        self.peak_accel = self.peak_accel.max(accel);
        let caps = match phase {
            ToolheadPhase::Moving | ToolheadPhase::RestTransition => limits.moving,
            ToolheadPhase::Resting => limits.extrude_only,
        };
        let velocity_bound = caps.velocity * (1.0 + VELOCITY_SLACK);
        if velocity > velocity_bound {
            self.excesses.push(ExtruderExcess::Velocity {
                t,
                value: velocity,
                bound: velocity_bound,
                phase,
            });
        }
        let accel_bound = caps.accel + accel_slack;
        if accel > accel_bound {
            self.excesses.push(ExtruderExcess::Acceleration {
                t,
                value: accel,
                bound: accel_bound,
                phase,
                cause: AccelCause::Unattributed,
            });
        }
    }

    fn check_step(&mut self, allowance: StepAllowance, t: f64, step: f64) {
        let rounding = geometry::SEAM_VELOCITY_STEP_MM_S;
        if step <= rounding {
            return;
        }
        match allowance {
            StepAllowance::Unlimited => {}
            StepAllowance::UnshapedLeaderStep(allowance) if step <= rounding + allowance => {
                self.exempt_steps.push(UnshapedLeaderVelocityStep {
                    t,
                    step,
                    allowance: rounding + allowance,
                });
            }
            StepAllowance::UnshapedLeaderStep(allowance) => {
                self.excesses.push(ExtruderExcess::VelocityStep {
                    t,
                    step,
                    allowance: rounding + allowance,
                });
            }
            StepAllowance::Rounding => self.excesses.push(ExtruderExcess::VelocityStep {
                t,
                step,
                allowance: rounding,
            }),
        }
    }
}

impl std::fmt::Display for ToolheadPhase {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Moving => "toolhead moving",
            Self::RestTransition => "rest transition",
            Self::Resting => "toolhead resting",
        })
    }
}

impl std::fmt::Display for ExtruderExcess {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match *self {
            Self::Velocity {
                t,
                value,
                bound,
                phase,
            } => write!(
                f,
                "e velocity {value:.3} mm/s at t={t:.4} ({phase}) exceeds {bound:.3}"
            ),
            Self::Acceleration {
                t,
                value,
                bound,
                phase,
                cause,
            } => {
                write!(
                    f,
                    "e acceleration {value:.1} mm/s² at t={t:.4} ({phase}) exceeds {bound:.1}"
                )?;
                match cause {
                    AccelCause::PathTurning => f.write_str(", asked by the path's turning"),
                    AccelCause::Unattributed => Ok(()),
                }
            }
            Self::VelocityStep { t, step, allowance } => write!(
                f,
                "e velocity steps by {step:.4} mm/s at t={t:.4}, beyond its {allowance:.4} mm/s \
                 allowance"
            ),
        }
    }
}

/// The chains the planner compiles for `settings`, its spatial axes
/// defaulted where the config leaves them undeclared, as the snapshot
/// pipeline does.
fn compiled_chains(
    settings: &planner_config::from_doc::MotionSettings,
) -> trajectory::AxisChainSet {
    let spatial = SPATIAL_AXES.iter().map(|name| {
        settings
            .axes
            .iter()
            .find(|decl| decl.name == *name)
            .cloned()
            .unwrap_or_else(|| planner_config::AxisDecl {
                name: (*name).to_string(),
                follows: Vec::new(),
                motors: Vec::new(),
                post_processors: Vec::new(),
                max_velocity: None,
                max_accel: None,
                instantaneous_corner_velocity: None,
            })
    });
    let followers = settings
        .axes
        .iter()
        .filter(|decl| !SPATIAL_AXES.contains(&decl.name.as_str()))
        .cloned();
    let registry = planner_config::AxisRegistry::try_new(spatial.chain(followers).collect())
        .expect("axes register");
    planner_config::PostProcessorSet::try_new(&registry, &settings.post_processors)
        .expect("post-processors build")
        .compile_active_chains(&registry, false)
        .expect("chains compile")
}

fn step_allowance(
    chains: &trajectory::AxisChainSet,
    moving: Caps,
    ratio_bound: f64,
    leader_accel_tol: f64,
) -> StepAllowance {
    if moving.accel.is_infinite() {
        return StepAllowance::Unlimited;
    }
    let allowance = motion_pipeline::follower_velocity_step_allowance(
        chains,
        E_AXIS,
        ratio_bound,
        leader_accel_tol,
    );
    if allowance > 0.0 {
        StepAllowance::UnshapedLeaderStep(allowance)
    } else {
        StepAllowance::Rounding
    }
}

fn extruder_limits(config: &Path, ratio_bound: f64) -> Option<ExtruderLimits> {
    let settings = motion_settings(config);
    let extrude_only = Caps {
        velocity: settings.max_extrude_only_velocity.unwrap_or(f64::INFINITY),
        accel: settings.max_extrude_only_accel.unwrap_or(f64::INFINITY),
    };
    let Some(axis) = settings.axes.iter().find(|decl| decl.name == "e") else {
        return (!extrude_only.is_unlimited()).then_some(ExtruderLimits {
            moving: Caps::UNLIMITED,
            extrude_only,
            accel_slack: AccelSlack {
                absolute: settings.fit_tolerance_accel_mm_s2,
                position_equivalent_mm: None,
            },
            transition_window_s: 0.0,
            step_allowance: StepAllowance::Unlimited,
            advance: None,
        });
    };
    let moving = Caps {
        velocity: axis.max_velocity.unwrap_or(f64::INFINITY),
        accel: axis.max_accel.unwrap_or(f64::INFINITY),
    };
    if moving.is_unlimited() && extrude_only.is_unlimited() {
        return None;
    }
    let chains = compiled_chains(&settings);
    let chain = &chains.chains[E_AXIS];
    let (kernel_back, kernel_forward) = chain.max_input_window();
    let advance = chain
        .kernel()
        .is_none()
        .then(|| linear_advance_k(&settings, axis))
        .flatten()
        .map(|k| KernelFreeLinearAdvance { k, ratio_bound });
    Some(ExtruderLimits {
        moving,
        extrude_only,
        accel_slack: AccelSlack {
            absolute: settings.fit_tolerance_accel_mm_s2,
            position_equivalent_mm: (chain.leading_transform().is_some()
                && chain.kernel().is_none())
            .then_some(settings.fit_tolerance_mm * FOLLOWER_POSITION_SCALE_FLOOR),
        },
        transition_window_s: kernel_forward.max(-kernel_back) + geometry::RATIO_RAMP_MAX_TIME_S,
        step_allowance: step_allowance(
            &chains,
            moving,
            ratio_bound,
            settings.fit_tolerance_accel_mm_s2,
        ),
        advance,
    })
}

fn toolhead_speed(trajectory: &pipeline_snapshot::ExactTrajectory, t: f64) -> f64 {
    (0..E_AXIS)
        .map(|axis| {
            trajectory
                .eval_axis(axis, t, SampleSide::Right)
                .unwrap_or_else(|e| panic!("axis {axis} at t={t} is not evaluable: {e}"))
                .velocity
        })
        .fold(0.0, |sum: f64, v| v.mul_add(v, sum))
        .sqrt()
}

/// The toolhead's rests, each a maximal run of pieces it holds still over.
fn toolhead_rests(trajectory: &pipeline_snapshot::ExactTrajectory) -> Vec<(f64, f64)> {
    let mut rests: Vec<(f64, f64)> = Vec::new();
    for piece in trajectory.breakpoints().windows(2) {
        let (t0, t1) = (piece[0], piece[1]);
        if t1 <= t0 || toolhead_speed(trajectory, 0.5 * (t0 + t1)) > TOOLHEAD_REST_MM_S {
            continue;
        }
        match rests.last_mut() {
            Some(rest) if rest.1 == t0 => rest.1 = t1,
            _ => rests.push((t0, t1)),
        }
    }
    rests
}

struct PhaseMap {
    rests: Vec<(f64, f64)>,
    window_s: f64,
    t_start: f64,
    t_end: f64,
}

impl PhaseMap {
    fn at(&self, t: f64) -> ToolheadPhase {
        let index = self.rests.partition_point(|rest| rest.1 < t);
        match self.rests.get(index) {
            Some(&(rest_start, rest_end)) if rest_start <= t => {
                let after_motion = rest_start > self.t_start && t - rest_start <= self.window_s;
                let before_motion = rest_end < self.t_end && rest_end - t <= self.window_s;
                if after_motion || before_motion {
                    ToolheadPhase::RestTransition
                } else {
                    ToolheadPhase::Resting
                }
            }
            _ => ToolheadPhase::Moving,
        }
    }
}

/// Every probed extruder state of the plan of `gcode` against the limits
/// of the config it was planned under, and every velocity step against its
/// allowance. `None` where the extruder has no limits.
pub fn extruder_audit(
    snapshot: &pipeline_snapshot::Snapshot,
    gcode: &str,
    config: &Path,
) -> Option<ExtruderAudit> {
    let limits = extruder_limits(config, extrusion_ratio_bound(&waypoints(gcode, config)))?;
    let trajectory = &snapshot.trajectory;
    let phases = PhaseMap {
        rests: toolhead_rests(trajectory),
        window_s: limits.transition_window_s,
        t_start: trajectory.t_start(),
        t_end: trajectory.t_end(),
    };
    let mut audit = ExtruderAudit::default();
    let rows = trajectory.rows(E_AXIS);
    let state = |row: usize, t: f64, side: SampleSide| {
        trajectory
            .eval_row(E_AXIS, row, t, side)
            .unwrap_or_else(|e| panic!("e at t={t} is not evaluable: {e}"))
    };
    for (index, row) in rows.iter().enumerate() {
        let breakpoints = trajectory.row_breakpoints(E_AXIS, index);
        for interval in breakpoints.windows(2) {
            let (t0, t1) = (interval[0], interval[1]);
            if t1 <= t0 {
                continue;
            }
            let accel_slack = limits.accel_slack.over_piece(t1 - t0);
            for k in 0..=E_PROBES_PER_INTERVAL {
                let t = t0 + (t1 - t0) * k as f64 / E_PROBES_PER_INTERVAL as f64;
                let side = if k == E_PROBES_PER_INTERVAL {
                    SampleSide::Left
                } else {
                    SampleSide::Right
                };
                let probed = state(index, t, side);
                audit.check(
                    &limits,
                    (t, phases.at(t)),
                    accel_slack,
                    probed.velocity.abs(),
                    probed.acceleration.abs(),
                );
            }
        }
        for &t in breakpoints.iter().filter(|&&t| t > row.t0 && t < row.t1) {
            let step = (state(index, t, SampleSide::Right).velocity
                - state(index, t, SampleSide::Left).velocity)
                .abs();
            audit.check_step(limits.step_allowance, t, step);
        }
        if let Some(next) = rows
            .get(index + 1)
            .filter(|next| (next.t0 - row.t1).abs() <= ROW_JOIN_S)
        {
            let step = (state(index + 1, next.t0, SampleSide::Right).velocity
                - state(index, row.t1, SampleSide::Left).velocity)
                .abs();
            audit.check_step(limits.step_allowance, row.t1, step);
        }
    }
    if let Some(advance) = limits.advance {
        for excess in &mut audit.excesses {
            if let ExtruderExcess::Acceleration {
                t,
                phase: ToolheadPhase::Moving,
                cause,
                ..
            } = excess
            {
                *cause = advance.cause_at(trajectory, *t, limits.moving.accel);
            }
        }
    }
    Some(audit)
}

fn linear_advance_k(
    settings: &planner_config::from_doc::MotionSettings,
    axis: &planner_config::AxisDecl,
) -> Option<f64> {
    axis.post_processors.iter().find_map(|name| {
        settings
            .post_processors
            .iter()
            .find(|decl| decl.name == *name && decl.ty == "linear_pressure_advance")
            .map(|decl| {
                decl.params
                    .iter()
                    .find(|(key, _)| key == "k")
                    .map(|&(_, k)| k)
                    .unwrap_or_else(|| panic!("[post_processor {name}] has no k"))
            })
    })
}

/// The shaped path's motion at an instant along its own heading: speed,
/// tangential acceleration and jerk, and the turning term `a⊥²/v`.
struct PathMotion {
    speed: f64,
    accel: f64,
    jerk: f64,
    turning: f64,
}

impl PathMotion {
    fn at(trajectory: &pipeline_snapshot::ExactTrajectory, t: f64) -> Self {
        let mut v = [0.0; E_AXIS];
        let mut a = [0.0; E_AXIS];
        let mut j = [0.0; E_AXIS];
        for axis in 0..E_AXIS {
            let state = trajectory
                .eval_axis(axis, t, SampleSide::Right)
                .unwrap_or_else(|e| panic!("axis {axis} at t={t} is not evaluable: {e}"));
            (v[axis], a[axis], j[axis]) = (state.velocity, state.acceleration, state.jerk);
        }
        let dot = |p: &[f64; E_AXIS], q: &[f64; E_AXIS]| -> f64 {
            p.iter().zip(q).map(|(p, q)| p * q).sum()
        };
        let speed = dot(&v, &v).sqrt();
        let accel = dot(&v, &a) / speed;
        Self {
            speed,
            accel,
            jerk: dot(&v, &j) / speed,
            turning: (dot(&a, &a) - accel * accel) / speed,
        }
    }
}

impl KernelFreeLinearAdvance {
    fn cause_at(
        self,
        trajectory: &pipeline_snapshot::ExactTrajectory,
        t: f64,
        cap: f64,
    ) -> AccelCause {
        let path = PathMotion::at(trajectory, t);
        if path.speed < HEADING_MIN_SPEED_MM_S {
            return AccelCause::Unattributed;
        }
        let e_velocity = trajectory
            .eval_axis(E_AXIS, t, SampleSide::Right)
            .unwrap_or_else(|e| panic!("e at t={t} is not evaluable: {e}"))
            .velocity;
        let ratio = e_velocity / (path.speed + self.k * path.accel);
        if !(ratio > 0.0 && ratio <= self.ratio_bound) {
            return AccelCause::Unattributed;
        }
        let without_turning = (ratio * (path.accel + self.k * path.jerk)).abs();
        let demand = (ratio * (path.accel + self.k * (path.jerk + path.turning))).abs();
        if demand > cap && without_turning <= cap && !near_valley(trajectory, t) {
            AccelCause::PathTurning
        } else {
            AccelCause::Unattributed
        }
    }
}

/// Whether a shaped speed minimum under [`VALLEY_SPEED_MM_S`] lies within
/// the time the path's peak deceleration nearby takes to brake from four
/// times that minimum's speed.
fn near_valley(trajectory: &pipeline_snapshot::ExactTrajectory, t: f64) -> bool {
    let speeds: Vec<(f64, f64)> = (-VALLEY_PROBES_EACH_SIDE..=VALLEY_PROBES_EACH_SIDE)
        .map(|i| {
            let at = (t + f64::from(i) * VALLEY_PROBE_STEP_S)
                .clamp(trajectory.t_start(), trajectory.t_end());
            (at, toolhead_speed(trajectory, at))
        })
        .collect();
    let peak_accel = speeds
        .windows(2)
        .map(|pair| ((pair[1].1 - pair[0].1) / VALLEY_PROBE_STEP_S).abs())
        .fold(f64::MIN_POSITIVE, f64::max);
    speeds.windows(3).any(|w| {
        let (at, v) = w[1];
        v <= w[0].1
            && v <= w[2].1
            && v < VALLEY_SPEED_MM_S
            && (at - t).abs() <= 4.0 * v / peak_accel + VALLEY_REACH_FLOOR_S
    })
}
