//! The planned extruder against the limits of the config it was planned
//! under. Which caps hold is the commanded move's, as the planner reads
//! it, never the shaped toolhead's motion: while e follows the toolhead,
//! its spill included, the `[axis e]` `max_velocity` / `max_accel` its
//! advance and ratio ramps are planned within; through an extrude-only
//! move, the limits the planner plans it under below its feed
//! ([`geometry::velocity::extrude_only_move_limits`]) plus the follower's
//! while toolhead motion that extrudes is within its window. Its
//! acceleration may leave those caps by the fit's acceleration tolerance
//! only, and its velocity steps only by rounding, bar the steps a
//! kernel-free advance makes where an unshaped leader steps its
//! acceleration, which [`motion_pipeline::follower_velocity_step_allowance`]
//! bounds.

use std::collections::HashMap;
use std::path::Path;

use motion_pipeline::live_config;
use pipeline_snapshot::waypoints::{Waypoint, extrusion_ratio_bound};
use pipeline_snapshot::{SampleSide, SegmentCommand, Snapshot};

use super::{motion_settings, planner, waypoints};

const E_AXIS: usize = 3;
const SPATIAL_AXES: [&str; 3] = ["x", "y", "z"];
const E_PROBES_PER_INTERVAL: usize = 16;
const VELOCITY_SLACK: f64 = 1e-6;
const ROW_JOIN_S: f64 = 1e-9;
const LEADER_ACCEL_STEP_ROUNDING_MM_S2: f64 = 1e-6;
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
    const STILL: Self = Self {
        velocity: 0.0,
        accel: 0.0,
    };

    fn is_unlimited(self) -> bool {
        self.velocity.is_infinite() && self.accel.is_infinite()
    }
}

/// The move the plan commands while the extruder is probed, which decides
/// the caps that hold it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Commanded {
    /// A toolhead move, or none: e follows the toolhead.
    Following,
    /// An extrude-only move: a retract, unretract or prime.
    ExtrudeOnly,
}

/// What the plan may ask of the extruder: the follower axis's caps while
/// it follows the toolhead, the extrude-only caps the planner plans
/// retracts and primes under, the fit's acceleration tolerance
/// (`max_accel_deviation`) on top of either, the window around a toolhead
/// move the follower moves e in for it, and the largest velocity step it
/// may take at once.
#[derive(Debug, Clone, Copy)]
struct ExtruderLimits {
    moving: Caps,
    extrude_only: Caps,
    accel_slack: f64,
    follow_window_s: f64,
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

/// A velocity step is an unbounded acceleration, so where the follower has
/// an acceleration limit only rounding may step its velocity, bar the steps
/// a kernel-free advance makes where one of its unshaped `leaders` steps
/// its acceleration.
#[derive(Debug, Clone, Copy)]
enum StepAllowance {
    Unlimited,
    Rounding,
    UnshapedLeaderStep {
        allowance: f64,
        leaders: [bool; E_AXIS],
    },
}

/// Where the plan's extruder leaves its limits.
#[derive(Debug, Clone, Copy)]
pub enum ExtruderExcess {
    Velocity {
        t: f64,
        value: f64,
        bound: f64,
        command: Commanded,
    },
    Acceleration {
        t: f64,
        value: f64,
        bound: f64,
        command: Commanded,
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

/// A velocity step a kernel-free advance makes where an unshaped leader
/// steps its acceleration, within the allowance
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
        accel_slack: f64,
        (t, command, caps): (f64, Commanded, Caps),
        velocity: f64,
        accel: f64,
    ) {
        self.peak_velocity = self.peak_velocity.max(velocity);
        self.peak_accel = self.peak_accel.max(accel);
        let velocity_bound = caps.velocity * (1.0 + VELOCITY_SLACK);
        if velocity > velocity_bound {
            self.excesses.push(ExtruderExcess::Velocity {
                t,
                value: velocity,
                bound: velocity_bound,
                command,
            });
        }
        let accel_bound = caps.accel + accel_slack;
        if accel > accel_bound {
            self.excesses.push(ExtruderExcess::Acceleration {
                t,
                value: accel,
                bound: accel_bound,
                command,
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
            StepAllowance::UnshapedLeaderStep { allowance, .. } if step <= rounding + allowance => {
                self.exempt_steps.push(UnshapedLeaderVelocityStep {
                    t,
                    step,
                    allowance: rounding + allowance,
                });
            }
            StepAllowance::UnshapedLeaderStep { allowance, .. } => {
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

impl StepAllowance {
    /// The allowance of a velocity step whose adjoining pieces span
    /// `adjoining`: an unshaped leader's only where one of them steps its
    /// acceleration within, rounding elsewhere.
    fn at(self, leader_steps: &LeaderAccelSteps, adjoining: (f64, f64)) -> Self {
        match self {
            Self::UnshapedLeaderStep { .. } if !leader_steps.within(adjoining) => Self::Rounding,
            allowance => allowance,
        }
    }
}

/// The instants, in order, where an unshaped leader of e steps its
/// acceleration.
struct LeaderAccelSteps(Vec<f64>);

impl LeaderAccelSteps {
    fn of(trajectory: &pipeline_snapshot::ExactTrajectory, allowance: StepAllowance) -> Self {
        let StepAllowance::UnshapedLeaderStep { leaders, .. } = allowance else {
            return Self(Vec::new());
        };
        let mut steps = Vec::new();
        for axis in (0..E_AXIS).filter(|&axis| leaders[axis]) {
            let accel = |row: usize, t: f64, side: SampleSide| {
                trajectory
                    .eval_row(axis, row, t, side)
                    .unwrap_or_else(|e| panic!("axis {axis} at t={t} is not evaluable: {e}"))
                    .acceleration
            };
            let steps_at = |(before, after): (f64, f64)| {
                (after - before).abs() > LEADER_ACCEL_STEP_ROUNDING_MM_S2
            };
            let rows = trajectory.rows(axis);
            for (index, row) in rows.iter().enumerate() {
                steps.extend(
                    trajectory
                        .row_breakpoints(axis, index)
                        .into_iter()
                        .filter(|&t| t > row.t0 && t < row.t1)
                        .filter(|&t| {
                            steps_at((
                                accel(index, t, SampleSide::Left),
                                accel(index, t, SampleSide::Right),
                            ))
                        }),
                );
                let joins_next = rows
                    .get(index + 1)
                    .filter(|next| (next.t0 - row.t1).abs() <= ROW_JOIN_S)
                    .is_some_and(|next| {
                        steps_at((
                            accel(index, row.t1, SampleSide::Left),
                            accel(index + 1, next.t0, SampleSide::Right),
                        ))
                    });
                if joins_next {
                    steps.push(row.t1);
                }
            }
        }
        steps.sort_by(f64::total_cmp);
        Self(steps)
    }

    fn within(&self, (from, to): (f64, f64)) -> bool {
        let first = self.0.partition_point(|&t| t < from);
        self.0.get(first).is_some_and(|&t| t <= to)
    }
}

impl std::fmt::Display for Commanded {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Following => "e following the toolhead",
            Self::ExtrudeOnly => "extrude-only move",
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
                command,
            } => write!(
                f,
                "e velocity {value:.3} mm/s at t={t:.4} ({command}) exceeds {bound:.3}"
            ),
            Self::Acceleration {
                t,
                value,
                bound,
                command,
                cause,
            } => {
                write!(
                    f,
                    "e acceleration {value:.1} mm/s² at t={t:.4} ({command}) exceeds {bound:.1}"
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
    if allowance == 0.0 {
        return StepAllowance::Rounding;
    }
    let unshaped: Vec<usize> = chains
        .projected_followers()
        .find(|&(follower, _)| follower == E_AXIS)
        .map(|(_, leaders)| {
            leaders
                .iter()
                .copied()
                .filter(|&leader| chains.chains[leader].kernel().is_none())
                .collect()
        })
        .unwrap_or_default();
    StepAllowance::UnshapedLeaderStep {
        allowance,
        leaders: std::array::from_fn(|axis| unshaped.contains(&axis)),
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
            accel_slack: settings.fit_tolerance_accel_mm_s2,
            follow_window_s: 0.0,
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
    let advance = kernel_free_advance_k(config).map(|k| KernelFreeLinearAdvance { k, ratio_bound });
    Some(ExtruderLimits {
        moving,
        extrude_only,
        accel_slack: settings.fit_tolerance_accel_mm_s2,
        follow_window_s: kernel_forward.max(-kernel_back) + geometry::RATIO_RAMP_MAX_TIME_S,
        step_allowance: step_allowance(
            &chains,
            moving,
            ratio_bound,
            settings.fit_tolerance_accel_mm_s2,
        ),
        advance,
    })
}

/// The `k` of the extruder's kernel-free linear advance; `None` where e has
/// a kernel or no linear advance.
pub fn kernel_free_advance_k(config: &Path) -> Option<f64> {
    let settings = motion_settings(config);
    let axis = settings.axes.iter().find(|decl| decl.name == "e")?;
    compiled_chains(&settings).chains[E_AXIS]
        .kernel()
        .is_none()
        .then(|| linear_advance_k(&settings, axis))
        .flatten()
}

/// Whether the extruder's chain smooths e through a kernel of its own.
pub fn e_has_kernel(config: &Path) -> bool {
    compiled_chains(&motion_settings(config)).chains[E_AXIS]
        .kernel()
        .is_some()
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

/// An extrude-only move the plan commands: when it runs and the limits the
/// planner plans it under.
struct ExtrudeOnlyRun {
    t_start: f64,
    t_end: f64,
    caps: Caps,
}

/// The commanded moves in time order: the extrude-only runs, and the
/// toolhead moves that extrude, which the follower moves e for within its
/// window around them.
struct CommandMap {
    extrude_only: Vec<ExtrudeOnlyRun>,
    extruding_motion: Vec<(f64, f64)>,
    moving: Caps,
    follow_window_s: f64,
}

impl CommandMap {
    fn new(
        snapshot: &Snapshot,
        waypoints: &[Waypoint],
        config: &Path,
        limits: &ExtruderLimits,
    ) -> Self {
        assert!(
            !snapshot.commands.is_empty() || snapshot.trajectory.is_empty(),
            "the audit reads the moves a plan made in process commands, which a serialized \
             snapshot does not keep"
        );
        if let Some(pair) = snapshot
            .commands
            .windows(2)
            .find(|pair| pair[1].t_start < pair[0].t_end)
        {
            panic!("the commanded segments run out of order: {pair:?}");
        }
        let planner = planner("extruder audit", config);
        let base = live_config::GcodeMoveBase::of(
            &planner,
            &live_config::extruder_caps(&planner).expect("the extruder chain compiles"),
        )
        .expect("the config's limits build a move base");
        let steps = pipeline_snapshot::plan_steps(waypoints, base).expect("the gcode plans");
        let extrude_only_moves: HashMap<u32, &geometry::Move> =
            pipeline_snapshot::plan_moves(&steps)
                .filter(|m| m.segment.spatial.is_none())
                .map(|m| (m.source.start_line, m))
                .collect();
        let drives_e = |command: &&SegmentCommand| {
            command.followers.iter().any(|follower| {
                follower.axis_index == E_AXIS
                    && (follower.ratio != 0.0 || follower.ratio_end != 0.0)
            })
        };
        let mut extrude_only = Vec::new();
        let mut extruding_motion = Vec::new();
        for command in snapshot.commands.iter().filter(drives_e) {
            if command.spatial_path {
                extruding_motion.push((command.t_start, command.t_end));
                continue;
            }
            let commanded = extrude_only_moves
                .get(&command.source_line)
                .unwrap_or_else(|| panic!("no extrude-only move runs {command:?}"));
            let (velocity, accel) = geometry::velocity::extrude_only_move_limits(
                (
                    commanded.limits.max_velocity_mm_s,
                    commanded.limits.accel_mm_s2,
                ),
                (limits.extrude_only.velocity, limits.extrude_only.accel),
            );
            extrude_only.push(ExtrudeOnlyRun {
                t_start: command.t_start,
                t_end: command.t_end,
                caps: Caps { velocity, accel },
            });
        }
        Self {
            extrude_only,
            extruding_motion,
            moving: limits.moving,
            follow_window_s: limits.follow_window_s,
        }
    }

    /// The move commanded at `t`, read from `side` where two meet, and the
    /// caps it holds e to: an extrude-only move's own plus, while toolhead
    /// motion that extrudes is within the follower's window, the
    /// follower's; the follower's otherwise.
    fn at(&self, t: f64, side: SampleSide) -> (Commanded, Caps) {
        let started = self.extrude_only.partition_point(|run| match side {
            SampleSide::Right => run.t_start <= t,
            SampleSide::Left => run.t_start < t,
        });
        let running = started
            .checked_sub(1)
            .map(|index| &self.extrude_only[index])
            .filter(|run| match side {
                SampleSide::Right => t < run.t_end,
                SampleSide::Left => t <= run.t_end,
            });
        let Some(run) = running else {
            return (Commanded::Following, self.moving);
        };
        let following = if self.extruding_motion_within_window(t) {
            self.moving
        } else {
            Caps::STILL
        };
        (
            Commanded::ExtrudeOnly,
            Caps {
                velocity: run.caps.velocity + following.velocity,
                accel: run.caps.accel + following.accel,
            },
        )
    }

    /// Whether a toolhead move that extrudes runs within the follower's
    /// window of `t`: only then may the follower still move e for it.
    fn extruding_motion_within_window(&self, t: f64) -> bool {
        let first = self
            .extruding_motion
            .partition_point(|&(_, end)| end < t - self.follow_window_s);
        self.extruding_motion
            .get(first)
            .is_some_and(|&(start, _)| start <= t + self.follow_window_s)
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
    let waypoints = waypoints(gcode, config);
    let limits = extruder_limits(config, extrusion_ratio_bound(&waypoints))?;
    let trajectory = &snapshot.trajectory;
    let commands = CommandMap::new(snapshot, &waypoints, config, &limits);
    let leader_steps = LeaderAccelSteps::of(trajectory, limits.step_allowance);
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
            for k in 0..=E_PROBES_PER_INTERVAL {
                let t = t0 + (t1 - t0) * k as f64 / E_PROBES_PER_INTERVAL as f64;
                let side = if k == E_PROBES_PER_INTERVAL {
                    SampleSide::Left
                } else {
                    SampleSide::Right
                };
                let probed = state(index, t, side);
                let (command, caps) = commands.at(t, side);
                audit.check(
                    limits.accel_slack,
                    (t, command, caps),
                    probed.velocity.abs(),
                    probed.acceleration.abs(),
                );
            }
        }
        for adjoining in breakpoints.windows(3) {
            let t = adjoining[1];
            let step = (state(index, t, SampleSide::Right).velocity
                - state(index, t, SampleSide::Left).velocity)
                .abs();
            let allowance = limits
                .step_allowance
                .at(&leader_steps, (adjoining[0], adjoining[2]));
            audit.check_step(allowance, t, step);
        }
        if let Some(next) = rows
            .get(index + 1)
            .filter(|next| (next.t0 - row.t1).abs() <= ROW_JOIN_S)
        {
            let step = (state(index + 1, next.t0, SampleSide::Right).velocity
                - state(index, row.t1, SampleSide::Left).velocity)
                .abs();
            let adjoining = (
                breakpoints[breakpoints.len() - 2],
                trajectory.row_breakpoints(E_AXIS, index + 1)[1],
            );
            let allowance = limits.step_allowance.at(&leader_steps, adjoining);
            audit.check_step(allowance, row.t1, step);
        }
    }
    if let Some(advance) = limits.advance {
        for excess in &mut audit.excesses {
            if let ExtruderExcess::Acceleration {
                t,
                command: Commanded::Following,
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
