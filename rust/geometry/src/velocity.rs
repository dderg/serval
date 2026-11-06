#[cfg(test)]
use std::collections::HashSet;
use std::collections::VecDeque;
use std::sync::{Condvar, Mutex, MutexGuard};

use crate::LENGTH_EPS_MM;
#[cfg(test)]
use crate::fitter::{FitOutcome, UnblendReason};
use crate::path::CurvatureProfile;
#[cfg(test)]
use crate::path::Segment;
use crate::segment::SourceRange;
use crate::shaped_corner::ShapedCorners;
use crate::step_capacity::{StepCapacity, StepCapacityOverload};

mod corner_caps;
mod disk;
mod extruder_caps;
mod extrusion_reach;
pub mod law;
mod rate_cap;
mod reconstruct;
mod step_capacity;
mod z_caps;

pub use extrusion_reach::{ExtrusionBudget, ExtrusionReach, extrusion_budgets};
pub use law::{LawSegment, RailPair, RailPairShape, ScalarLaw};

use disk::Kinematics;

const VELOCITY_EPS_MM_S: f64 = 1e-9;
const MIN_INTEGRATION_TOL: f64 = 1e-9;
const NEGATIVE_VELOCITY_TOL_MM_S: f64 = 1e-6;
const CONSISTENCY_TOL: f64 = 1e-6;
const RECONSTRUCT_WORKERS_MAX: usize = 4;

/// Where the planner runs its member profiles alongside the reverse pass.
pub trait FanOut {
    /// Runs `lead` on the calling thread while up to `participants - 1`
    /// other threads run `follow`, then runs `follow` on the calling thread
    /// too. `follow` must return once `lead` has finished, or unwound, and
    /// left it nothing to do. A panic on any thread resurfaces on the calling
    /// thread.
    fn lead_and_follow(&self, participants: usize, lead: impl FnOnce(), follow: impl Fn() + Sync);
}

/// Everything on the calling thread.
pub struct Serial;

impl FanOut for Serial {
    fn lead_and_follow(&self, _participants: usize, lead: impl FnOnce(), follow: impl Fn() + Sync) {
        lead();
        follow();
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VelSample {
    pub s: f64,
    pub v: f64,
    pub a: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct MoveVelocity {
    pub entry_v: f64,
    pub exit_v: f64,
    pub peak_v: f64,
    pub samples: Vec<VelSample>,
    pub phases: Vec<LawSegment>,
    pub accel: f64,
    pub length: f64,
    pub source: SourceRange,
    pub step_capacity_overload: Option<StepCapacityOverload>,
    /// The speed the corner budget capped the move at (a blend's apex
    /// speed on both its halves, a curve's on the curve), infinite where it
    /// capped none. A re-plan without the corner context holds the move to
    /// it ([`MoveVelocity::corner_held`]).
    pub corner_speed_mm_s: f64,
}

impl MoveVelocity {
    /// `m` with the corner budget's speed cap on it baked into its
    /// feedrate, for a re-plan that holds it without the corner context.
    #[must_use]
    pub fn corner_held(&self, mut m: crate::Move) -> crate::Move {
        m.feedrate_mm_s = m.feedrate_mm_s.min(self.corner_speed_mm_s);
        m
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct VelocityReport {
    pub stops: u32,
    pub curvature_bound: u32,
    pub feedrate_bound: u32,
    pub limit_ride: u32,
    /// Joins whose speed the follower's ratio ramp lowered: a ratio step
    /// too large for the longest ramp at the speed the path allows.
    pub ratio_step_bound: u32,
    pub traversal_time_s: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct VelocityProfile {
    pub moves: Vec<MoveVelocity>,
    pub report: VelocityReport,
    /// Seam index of the last finality barrier: the highest seam whose velocity
    /// meets the forward/ceiling-feasible profile (`min(v_forward, ceiling)` —
    /// acceleration pinned by the past, full cruise, or a curvature-limited corner
    /// peak) rather than being dragged below it by the buffer's tentative terminal
    /// rest. It is the reconvergence point of the backward sweep: appended moves
    /// are downstream and append-only streaming cannot lower an already-ceiling
    /// seam, so every seam at-or-before `barrier` is final and the suffix past it
    /// is the deferrable brake-to-rest. Seam index == committable move count, so
    /// the caller commits the latest clean seam `<= barrier`. `0` means nothing
    /// past the entry is final.
    pub barrier: usize,
    /// Velocity at `barrier`, used to size the flush-trigger watermark.
    pub v_barrier: f64,
    /// Continuation speed at each reconstructed move boundary, clamped to
    /// the analytic seam bound so it remains a valid warm-start entry.
    pub boundary_speeds: Vec<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VelocityError {
    FiniteJerkUnsupported {
        line_no: u32,
        jerk: f64,
    },
    Inconsistent {
        line_no: u32,
    },
    NonAlphabet {
        line_no: u32,
    },
    NonFinite {
        line_no: u32,
    },
    Diverged {
        line_no: u32,
    },
    OverCommitted {
        line_no: u32,
    },
    NegativeVelocity {
        line_no: u32,
        v: f64,
    },
    /// The seam plan handed a member an entry/exit pair its own exact disk
    /// reach cannot connect — a planning bug, not a numeric residue.
    Infeasible {
        line_no: u32,
        member: usize,
        entry_v: f64,
        exit_v: f64,
    },
    InvalidConfig,
}

impl std::fmt::Display for VelocityError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::FiniteJerkUnsupported { line_no, jerk } => write!(
                f,
                "line {line_no}: finite max_jerk {jerk} is not supported by the continuous trajectory pipeline; set [printer] max_jerk: 0"
            ),
            other => write!(f, "{other:?}"),
        }
    }
}

impl std::error::Error for VelocityError {}

fn pin_rest_anchor(sample: Option<&mut VelSample>) {
    if let Some(s) = sample {
        s.a = 0.0;
    }
}

struct MoveCaps {
    kin: Kinematics,
    kappa_peak: f64,
    step_capacity_overload: Option<StepCapacityOverload>,
    corner_speed: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VelocityPlanParams<'a> {
    pub integration_tol: f64,
    pub max_extrude_only_velocity_mm_s: f64,
    pub max_extrude_only_accel_mm_s2: f64,
    pub entry_v: f64,
    pub step_capacity: Option<&'a StepCapacity>,
    /// The shaper's rounding the corner budget is held to on the shaped
    /// path; `None` where no kernel shapes X or Y.
    pub shaped_corners: Option<&'static ShapedCorners>,
}

#[cfg(test)]
pub(crate) fn warm_start_params(integration_tol: f64) -> VelocityPlanParams<'static> {
    VelocityPlanParams {
        integration_tol,
        max_extrude_only_velocity_mm_s: f64::INFINITY,
        max_extrude_only_accel_mm_s2: f64::INFINITY,
        entry_v: 0.0,
        step_capacity: None,
        shaped_corners: None,
    }
}

#[cfg(test)]
pub(crate) fn plan_velocity_warm_start(
    outcome: &FitOutcome,
    params: VelocityPlanParams,
) -> Result<VelocityProfile, VelocityError> {
    let stop_lines: HashSet<u32> = outcome
        .report
        .unblended
        .iter()
        .filter(|u| u.reason != UnblendReason::Collinear)
        .map(|u| u.line_no)
        .collect();
    let stop_before: Vec<bool> = outcome
        .moves
        .iter()
        .map(|m| {
            stop_lines.contains(&m.source.start_line)
                && !matches!(m.segment.spatial, Some(Segment::Clothoid(_)))
        })
        .collect();
    plan_velocity_stops(&outcome.moves, &stop_before, params)
}

/// Plan over an already-fitted move sequence with explicit per-seam stop
/// anchors: `stop_before[k]` forces rest at the seam entering `moves[k]`.
/// `stop_before[0]` is ignored — the entry seam is anchored at
/// `params.entry_v`. Nothing comes before or after `moves`.
pub fn plan_velocity_stops(
    moves: &[crate::Move],
    stop_before: &[bool],
    params: VelocityPlanParams,
) -> Result<VelocityProfile, VelocityError> {
    plan_velocity_stops_reconstruct_prefix(moves, stop_before, params, moves.len())
}

pub fn plan_velocity_stops_reconstruct_prefix(
    moves: &[crate::Move],
    stop_before: &[bool],
    params: VelocityPlanParams,
    reconstruct_count: usize,
) -> Result<VelocityProfile, VelocityError> {
    plan_velocity_stops_select_prefix(
        moves,
        stop_before,
        &extrusion_reach::extrusion_budgets(
            moves,
            params.max_extrude_only_velocity_mm_s,
            params.max_extrude_only_accel_mm_s2,
        ),
        params,
        &Serial,
        |_| reconstruct_count,
    )
}

/// Plans the whole window and reconstructs the prefix `select_prefix` picks
/// from the finality barrier, building member profiles on `workers`.
/// `budgets[k]` is what `moves[k]`'s plan takes from the moves around it,
/// see [`extrusion_reach::ExtrusionReach`].
pub fn plan_velocity_stops_select_prefix<F, W>(
    moves: &[crate::Move],
    stop_before: &[bool],
    budgets: &[ExtrusionBudget],
    params: VelocityPlanParams,
    workers: &W,
    select_prefix: F,
) -> Result<VelocityProfile, VelocityError>
where
    F: FnOnce(usize) -> usize,
    W: FanOut,
{
    plan_span(
        moves,
        stop_before,
        budgets,
        SpanEnds::WINDOW,
        params,
        workers,
        select_prefix,
    )
}

/// Re-plans a span cut out of a settled plan between its two seam speeds:
/// entering at `params.entry_v` and leaving at `exit_v`, each move's
/// acceleration budget scaled by `accel_scales[k]` and each seam held at or
/// under `seam_ceilings[k]` (one per seam). The plan of the span is the one
/// a window holding it plans there whenever the scaled budgets leave both
/// seam speeds standing: a window's plan before a seam reads only that
/// seam's speed, and after it only that speed again. Ceilings at the seam
/// speeds the span was cut from keep a re-plan from raising any of them,
/// which lowered budgets would otherwise do where a seam's ceiling grows as
/// its budget falls (the ratio steps'). The returned boundary speeds show
/// whether both ends stand: an exit the span cannot reach comes back lower
/// than `exit_v`, an entry it cannot brake from fails with
/// [`VelocityError::OverCommitted`].
pub fn replan_span(
    moves: &[crate::Move],
    stop_before: &[bool],
    budgets: &[ExtrusionBudget],
    accel_scales: &[f64],
    seam_ceilings: &[f64],
    params: VelocityPlanParams,
    exit_v: f64,
) -> Result<VelocityProfile, VelocityError> {
    assert_eq!(accel_scales.len(), moves.len(), "one scale per move");
    assert_eq!(seam_ceilings.len(), moves.len() + 1, "one ceiling per seam");
    assert!(
        exit_v.is_finite() && exit_v >= 0.0,
        "a span's exit speed must be finite and non-negative, got {exit_v}"
    );
    plan_span(
        moves,
        stop_before,
        budgets,
        SpanEnds {
            accel_scales: Some(accel_scales),
            seam_ceilings: Some(seam_ceilings),
            exit_v,
        },
        params,
        &Serial,
        |_| moves.len(),
    )
}

/// What [`brake_horizon`] reads of a move: its kinematics on its
/// acceleration budget scaled by a fixed factor, and the ceiling of the seam
/// into it, `None` at the start of a span.
pub struct BrakeMove {
    kin: Kinematics,
    entry_ceiling: Option<f64>,
    line_no: u32,
}

impl BrakeMove {
    /// The fastest the move can be entered at and still leave at `exit`,
    /// or a bound under it. A line reads it off its budget exactly. On a
    /// curve the disk leaves at least `a/√2` of the budget `a` along the
    /// path while the turning `v²·κ` takes at most `a/√2`, which holds up
    /// to `v_c = √(a/(√2·κ_max))`, under every curvature cap the move
    /// rides; braking at `a/√2` up to `v_c` therefore never asks for more
    /// than the disk rail gives, and costs no rail integration.
    fn entry_speed(&self, exit: f64) -> f64 {
        let kin = &self.kin;
        let exit = exit.min(kin.flat_ceiling);
        if kin.is_straight() {
            let w = exit * exit + 2.0 * kin.accel * kin.length;
            return w.max(0.0).sqrt().min(kin.flat_ceiling);
        }
        let kappa_max = kin.kappa_abs(0.0).max(kin.kappa_abs(kin.length));
        let tangential = kin.accel * std::f64::consts::FRAC_1_SQRT_2;
        let turning_bound = disk::limit_speed(kappa_max, tangential);
        let exit = exit.min(turning_bound);
        (exit * exit + 2.0 * tangential * kin.length)
            .sqrt()
            .min(turning_bound)
            .min(kin.flat_ceiling)
            .min(disk::limit_speed(kin.kappa_abs(0.0), kin.accel))
    }
}

/// Move `m`'s [`BrakeMove`] on its budget scaled by `accel_scale`, entered
/// from `previous` (the move before it, its budget and its own brake move
/// under the same scale) or at the start of a span, at rest where
/// `stop_before`.
pub fn brake_move(
    m: &crate::Move,
    budget: &ExtrusionBudget,
    stop_before: bool,
    previous: Option<(&crate::Move, &ExtrusionBudget, &BrakeMove)>,
    accel_scale: f64,
    params: VelocityPlanParams,
) -> Result<BrakeMove, VelocityError> {
    let mut report = VelocityReport::default();
    let kin = build_move_caps(
        std::slice::from_ref(m),
        std::slice::from_ref(budget),
        Some(&[accel_scale]),
        params,
        &mut report,
    )?
    .pop()
    .expect("one move has one set of caps")
    .kin;
    let entry_ceiling = previous.map(|(up_move, up_budget, up)| {
        if stop_before {
            0.0
        } else {
            seam_ceiling(
                (up_move, m),
                (up_budget, budget),
                (&up.kin, &kin),
                &mut report,
            )
        }
    });
    Ok(BrakeMove {
        kin,
        entry_ceiling,
        line_no: m.source.start_line,
    })
}

/// The latest seam of `moves` (an index into `moves`, the seam entering
/// that move) whose speed in `entry_speeds` a rest at their far end could
/// not lower: there a brake envelope from that rest, through the moves'
/// ceilings and scaled budgets and never above the one the planner's disk
/// rails give ([`BrakeMove::entry_speed`]), reaches at least the seam's
/// speed. Any lowering of the plan at or past the far end then leaves every
/// seam up to it standing, the plan before a seam reading only that seam's
/// speed. `None` where no seam of `moves` stands clear.
pub fn brake_horizon(
    moves: &[BrakeMove],
    entry_speeds: impl DoubleEndedIterator<Item = f64> + ExactSizeIterator,
) -> Option<usize> {
    assert_eq!(entry_speeds.len(), moves.len(), "one entry speed per move");
    let mut brake = 0.0_f64;
    for (k, (brake_move, seam_speed)) in moves.iter().zip(entry_speeds).enumerate().rev() {
        let entry = brake_move.entry_speed(brake);
        brake = if k == 0 {
            entry
        } else {
            let ceiling = brake_move.entry_ceiling.unwrap_or_else(|| {
                panic!(
                    "brake horizon: line {} starts a span inside one",
                    brake_move.line_no
                )
            });
            entry.min(ceiling)
        };
        if seam_speed <= brake {
            return Some(k);
        }
    }
    None
}

/// How a plan ends and scales its budgets: a window brakes to rest at its
/// far end on its own budgets; a re-planned span leaves at a set speed and
/// holds its seams under ceilings.
#[derive(Clone, Copy)]
struct SpanEnds<'a> {
    accel_scales: Option<&'a [f64]>,
    seam_ceilings: Option<&'a [f64]>,
    exit_v: f64,
}

impl SpanEnds<'_> {
    const WINDOW: Self = Self {
        accel_scales: None,
        seam_ceilings: None,
        exit_v: 0.0,
    };
}

fn plan_span<F, W>(
    moves: &[crate::Move],
    stop_before: &[bool],
    budgets: &[ExtrusionBudget],
    ends: SpanEnds,
    params: VelocityPlanParams,
    workers: &W,
    select_prefix: F,
) -> Result<VelocityProfile, VelocityError>
where
    F: FnOnce(usize) -> usize,
    W: FanOut,
{
    let tol = params.integration_tol;
    let entry_v = params.entry_v;
    validate_config(params)?;

    let n = moves.len();
    assert_eq!(stop_before.len(), n, "one stop flag per move");
    assert_eq!(budgets.len(), n, "one extrusion budget per move");
    if let Some(m) = moves.iter().find(|m| m.limits.max_jerk_mm_s3.is_finite()) {
        return Err(VelocityError::FiniteJerkUnsupported {
            line_no: m.source.start_line,
            jerk: m.limits.max_jerk_mm_s3,
        });
    }
    if n == 0 {
        return Ok(VelocityProfile {
            moves: Vec::new(),
            report: VelocityReport::default(),
            barrier: 0,
            v_barrier: 0.0,
            boundary_speeds: vec![entry_v],
        });
    }

    let mut report = VelocityReport::default();
    let mut caps = build_move_caps(moves, budgets, ends.accel_scales, params, &mut report)?;
    let seam_ceilings = match (
        hold_corners(moves, &mut caps, stop_before, params),
        ends.seam_ceilings,
    ) {
        (Some(corners), Some(span)) => {
            Some(corners.iter().zip(span).map(|(a, b)| a.min(*b)).collect())
        }
        (Some(corners), None) => Some(corners),
        (None, span) => span.map(<[f64]>::to_vec),
    };
    check_entry_ceiling(moves, &caps, entry_v, tol)?;
    let mut plan = seed_seam_velocities(
        moves,
        &caps,
        budgets,
        stop_before,
        (entry_v, ends.exit_v),
        seam_ceilings.as_deref(),
        &mut report,
    );
    forward_pass(moves, &caps, &mut plan, tol)?;
    let shop = ProfileShop::new(&caps, &plan.is_anchor);
    let mut settled = None;
    workers.lead_and_follow(
        shop.participants,
        || {
            let _lead_done = LeadDone(&shop);
            let mut settler = Settler::new(&shop, select_prefix);
            settled = Some(settle_seams(
                moves,
                &caps,
                &mut plan,
                &mut settler,
                entry_v,
                tol,
            ));
        },
        || shop.follow(),
    );
    let (barrier, v_barrier, reconstruct_count) =
        settled.expect("the lead settles every seam or fails")?;
    let profiles = shop.into_profiles(reconstruct_count);
    let (out, boundaries) = reconstruct_runs(
        &moves[..reconstruct_count],
        &caps[..reconstruct_count],
        &budgets[..reconstruct_count],
        &plan,
        entry_v,
        profiles,
        &mut report,
    )?;

    Ok(VelocityProfile {
        moves: out,
        report,
        barrier,
        v_barrier,
        boundary_speeds: boundaries,
    })
}

fn validate_config(params: VelocityPlanParams) -> Result<(), VelocityError> {
    let VelocityPlanParams {
        integration_tol,
        max_extrude_only_velocity_mm_s,
        max_extrude_only_accel_mm_s2,
        entry_v,
        step_capacity: _,
        shaped_corners: _,
    } = params;
    if !(integration_tol.is_finite() && integration_tol >= MIN_INTEGRATION_TOL) {
        return Err(VelocityError::InvalidConfig);
    }
    if !(entry_v.is_finite() && entry_v >= 0.0) {
        return Err(VelocityError::InvalidConfig);
    }
    if !(max_extrude_only_velocity_mm_s > 0.0 && max_extrude_only_accel_mm_s2 > 0.0) {
        return Err(VelocityError::InvalidConfig);
    }
    Ok(())
}

fn build_move_caps(
    moves: &[crate::Move],
    budgets: &[ExtrusionBudget],
    accel_scales: Option<&[f64]>,
    params: VelocityPlanParams,
    report: &mut VelocityReport,
) -> Result<Vec<MoveCaps>, VelocityError> {
    let mut caps = Vec::with_capacity(moves.len());
    for (index, (m, budget)) in moves.iter().zip(budgets).enumerate() {
        let ratio_in_reach = budget.ratio_in_reach;
        let line_no = m.source.start_line;
        let path_ceiling = m.feedrate_mm_s.min(m.limits.max_velocity_mm_s);
        let (length, kappa0, sigma, kappa_peak, limits) = match &m.segment.spatial {
            Some(seg) => {
                let length = seg.s_len();
                validate_segment(seg, length, line_no, CONSISTENCY_TOL)?;
                let (kappa_start, _) = seg.kappa_endpoints();
                let sigma = seg.dkappa_ds(0.0);
                let (_, kappa_peak) = seg.kappa_peak();
                let z_capped = z_caps::z_capped(seg, &m.limits, kappa_peak, path_ceiling);
                let limits = match &m.limits.extruder {
                    Some(extruder) => {
                        extruder_caps::extruder_capped(seg, extruder, budget, kappa_peak, z_capped)
                    }
                    None => z_capped,
                };
                let limits = extruder_caps::moving_retract_capped(
                    &m.segment,
                    (
                        params.max_extrude_only_velocity_mm_s,
                        params.max_extrude_only_accel_mm_s2,
                    ),
                    limits,
                );
                (length, kappa_start, sigma, kappa_peak, limits)
            }
            None => {
                let length = m
                    .segment
                    .virtual_path_mm
                    .ok_or(VelocityError::NonFinite { line_no })?;
                if !(length.is_finite() && length > LENGTH_EPS_MM) {
                    return Err(VelocityError::NonFinite { line_no });
                }
                (
                    length,
                    0.0,
                    0.0,
                    0.0,
                    (
                        path_ceiling.min(params.max_extrude_only_velocity_mm_s),
                        m.limits
                            .accel_mm_s2
                            .min(params.max_extrude_only_accel_mm_s2),
                    ),
                )
            }
        };
        let ((flat_ceiling, accel), step_capacity_overload) = match params.step_capacity {
            Some(capacity) => step_capacity::step_capacity_capped(
                capacity,
                step_capacity::MotorDemand::of(
                    m,
                    m.segment.spatial.as_ref(),
                    ratio_in_reach,
                    capacity,
                ),
                length,
                kappa_peak,
                limits,
            ),
            None => (limits, None),
        };
        let accel = accel * accel_scales.map_or(1.0, |scales| scales[index]);

        if disk::limit_speed(kappa_peak, accel) < flat_ceiling {
            report.curvature_bound += 1;
        } else {
            report.feedrate_bound += 1;
        }
        caps.push(MoveCaps {
            kin: Kinematics {
                length,
                accel,
                kappa0,
                sigma,
                flat_ceiling,
            },
            kappa_peak,
            step_capacity_overload,
            corner_speed: f64::INFINITY,
        });
    }
    Ok(caps)
}

fn check_entry_ceiling(
    moves: &[crate::Move],
    caps: &[MoveCaps],
    entry_v: f64,
    tol: f64,
) -> Result<(), VelocityError> {
    let entry_ceiling = {
        let kin0 = &caps[0].kin;
        kin0.flat_ceiling
            .min(disk::limit_speed(kin0.kappa0.abs(), kin0.accel))
    };
    if entry_v > entry_ceiling + tol * (1.0 + entry_ceiling) {
        return Err(VelocityError::OverCommitted {
            line_no: moves[0].source.start_line,
        });
    }
    Ok(())
}

/// Seam speeds, which seams are anchors, and per member the rails the passes
/// read their reaches off: forward from the member's entry, reversed from its
/// exit.
struct SeamPlan {
    v: Vec<f64>,
    is_anchor: Vec<bool>,
    forward_rails: Vec<Option<LawSegment>>,
    backward_rails: Vec<Option<LawSegment>>,
}

impl SeamPlan {
    /// Drops member `j`'s rails once nothing reads them.
    fn release_rails(&mut self, j: usize) {
        self.forward_rails[j] = None;
        self.backward_rails[j] = None;
    }
}

/// Holds `caps` to the corner budget on the shaped path and returns each
/// seam's speed ceiling, see [`corner_caps::hold`]; none where X and Y are
/// unshaped.
fn hold_corners(
    moves: &[crate::Move],
    caps: &mut [MoveCaps],
    stop_before: &[bool],
    params: VelocityPlanParams,
) -> Option<Vec<f64>> {
    params
        .shaped_corners
        .map(|corners| corner_caps::hold(moves, caps, stop_before, corners))
}

fn seed_seam_velocities(
    moves: &[crate::Move],
    caps: &[MoveCaps],
    budgets: &[ExtrusionBudget],
    stop_before: &[bool],
    (entry_v, exit_v): (f64, f64),
    seam_ceilings: Option<&[f64]>,
    report: &mut VelocityReport,
) -> SeamPlan {
    let n = caps.len();
    let mut v = vec![0.0_f64; n + 1];
    v[0] = entry_v;
    v[n] = exit_v;
    let mut is_anchor = vec![false; n + 1];
    is_anchor[0] = true;
    is_anchor[n] = true;
    for k in 1..n {
        if stop_before[k] {
            report.stops += 1;
            is_anchor[k] = true;
        } else {
            v[k] = seam_ceiling(
                (&moves[k - 1], &moves[k]),
                (&budgets[k - 1], &budgets[k]),
                (&caps[k - 1].kin, &caps[k].kin),
                report,
            );
            if let Some(ceilings) = seam_ceilings {
                v[k] = v[k].min(ceilings[k]);
            }
        }
    }
    SeamPlan {
        v,
        is_anchor,
        forward_rails: vec![None; n],
        backward_rails: vec![None; n],
    }
}

/// The fastest a seam between two moves may run on their kinematics: their
/// feed ceilings, the curvature limits on either side, and the ratio step.
fn seam_ceiling(
    (up_move, dn_move): (&crate::Move, &crate::Move),
    (up_budget, dn_budget): (&ExtrusionBudget, &ExtrusionBudget),
    (up, dn): (&Kinematics, &Kinematics),
    report: &mut VelocityReport,
) -> f64 {
    let kappa_up = (up.kappa0 + up.sigma * up.length).abs();
    let kappa_dn = dn.kappa0.abs();
    let boundary_vlim =
        disk::limit_speed(kappa_up, up.accel).min(disk::limit_speed(kappa_dn, dn.accel));
    let ceiling = up.flat_ceiling.min(dn.flat_ceiling).min(boundary_vlim);
    let step_ceiling = extruder_caps::ratio_step_ceiling(
        up_move,
        dn_move,
        (up_budget, dn_budget),
        up.accel
            .max(dn.accel)
            .max(up_budget.strongest_accel_mm_s2)
            .max(dn_budget.strongest_accel_mm_s2),
    );
    if step_ceiling < ceiling {
        report.ratio_step_bound += 1;
        step_ceiling
    } else {
        ceiling
    }
}

fn forward_pass(
    moves: &[crate::Move],
    caps: &[MoveCaps],
    plan: &mut SeamPlan,
    tol: f64,
) -> Result<(), VelocityError> {
    let n = caps.len();
    let v = &mut plan.v;
    for k in 1..=n {
        let j = k - 1;
        let line_no = moves[j].source.start_line;
        let kin = &caps[j].kin;
        let disk = disk::disk_reach_v(kin, v[j], kin.length, tol)
            .ok_or(VelocityError::Diverged { line_no })?;
        if moves[j].limits.max_jerk_mm_s3 != f64::INFINITY {
            return Err(VelocityError::Diverged { line_no });
        }
        v[k] = v[k].min(disk.v);
        plan.forward_rails[j] = disk.rail;
    }
    Ok(())
}

/// The reverse pass and the entry brake check, settling every member's seam
/// speeds as it goes; returns the finality barrier, its speed, and how many
/// members the plan reconstructs.
fn settle_seams<F: FnOnce(usize) -> usize>(
    moves: &[crate::Move],
    caps: &[MoveCaps],
    plan: &mut SeamPlan,
    settler: &mut Settler<F>,
    entry_v: f64,
    tol: f64,
) -> Result<(usize, f64, usize), VelocityError> {
    let (barrier, v_barrier) = reverse_brake_envelope(moves, caps, plan, settler, tol)?;
    check_entry_brake(moves, caps, plan, entry_v, tol)?;
    settler.settle(plan, 0);
    let prefix = settler.prefix_or_select(plan, barrier);
    Ok((barrier, v_barrier, prefix))
}

fn reverse_brake_envelope<F: FnOnce(usize) -> usize>(
    moves: &[crate::Move],
    caps: &[MoveCaps],
    plan: &mut SeamPlan,
    settler: &mut Settler<F>,
    tol: f64,
) -> Result<(usize, f64), VelocityError> {
    let n = caps.len();
    let mut barrier = 0usize;
    for k in (1..n).rev() {
        let line_no = moves[k].source.start_line;
        let kin = &caps[k].kin;
        let disk = disk::disk_reach_v_rev(kin, plan.v[k + 1], kin.length, tol)
            .ok_or(VelocityError::Diverged { line_no })?;
        let forward_ceiling = plan.v[k];
        plan.v[k] = plan.v[k].min(disk.v);
        plan.backward_rails[k] = disk.rail;
        if barrier == 0 && !(plan.v[k] < forward_ceiling) {
            barrier = k;
            settler.select(plan, barrier);
        }
        settler.settle(plan, k);
    }
    Ok((barrier, plan.v[barrier]))
}

fn check_entry_brake(
    moves: &[crate::Move],
    caps: &[MoveCaps],
    plan: &mut SeamPlan,
    entry_v: f64,
    tol: f64,
) -> Result<(), VelocityError> {
    let entry_line_no = moves[0].source.start_line;
    let kin = &caps[0].kin;
    let entry_brake =
        disk::disk_reach_v_rev(kin, plan.v[1], kin.length, tol).ok_or(VelocityError::Diverged {
            line_no: entry_line_no,
        })?;
    plan.backward_rails[0] = entry_brake.rail;
    if entry_v > entry_brake.v + tol * (1.0 + entry_brake.v) {
        return Err(VelocityError::OverCommitted {
            line_no: entry_line_no,
        });
    }
    Ok(())
}

fn reconstruct_runs(
    moves: &[crate::Move],
    caps: &[MoveCaps],
    budgets: &[ExtrusionBudget],
    plan: &SeamPlan,
    entry_v: f64,
    profiles: Vec<MemberProfile>,
    report: &mut VelocityReport,
) -> Result<(Vec<MoveVelocity>, Vec<f64>), VelocityError> {
    let n = caps.len();
    let v = &plan.v;
    let is_anchor = &plan.is_anchor;
    let mut profiles = profiles.into_iter();
    let mut out: Vec<MoveVelocity> = Vec::with_capacity(n);
    let mut boundaries: Vec<f64> = Vec::with_capacity(n + 1);
    boundaries.push(entry_v);
    let mut run_start = 0;
    while run_start < n {
        let mut run_end = run_start + 1;
        while run_end < n && !is_anchor[run_end] {
            run_end += 1;
        }
        let run_start_line = moves[run_start].source.start_line;
        let reconstructed_phases = profiles
            .by_ref()
            .take(run_end - run_start)
            .map(|result| {
                result.map_err(|error| match error {
                    reconstruct::ReconstructError::Diverged => VelocityError::Diverged {
                        line_no: run_start_line,
                    },
                    reconstruct::ReconstructError::Infeasible {
                        member,
                        entry_v,
                        exit_v,
                    } => VelocityError::Infeasible {
                        line_no: run_start_line,
                        member,
                        entry_v,
                        exit_v,
                    },
                })
            })
            .collect::<Result<Vec<_>, _>>()?;
        let reconstructed: Vec<Vec<(f64, f64, f64)>> = reconstructed_phases
            .iter()
            .enumerate()
            .map(|(idx, segments)| {
                let mut samples = sample_segments(segments);
                let entry_v = v[run_start + idx];
                if let Some(first) = samples.first_mut() {
                    first.1 = entry_v;
                }
                if let Some(last) = samples.last_mut() {
                    last.1 = v[run_start + idx + 1];
                }
                samples
            })
            .collect();

        for (idx, j) in (run_start..run_end).enumerate() {
            let kin = &caps[j].kin;
            let m = &moves[j];
            let line_no = m.source.start_line;
            let mut samples: Vec<VelSample> = reconstructed[idx]
                .iter()
                .map(|&(s, v, a)| VelSample { s, v, a })
                .collect();
            if is_anchor[j] && v[j] <= VELOCITY_EPS_MM_S {
                pin_rest_anchor(samples.first_mut());
            }
            if is_anchor[j + 1] && v[j + 1] <= VELOCITY_EPS_MM_S {
                pin_rest_anchor(samples.last_mut());
            }
            let entry_v = samples.first().map_or(v[j], |s| s.v);
            let exit_v = samples.last().map_or(v[j + 1], |s| s.v);
            if let Some(v) = first_negative_velocity(&samples) {
                return Err(VelocityError::NegativeVelocity { line_no, v });
            }
            let peak_v = samples.iter().fold(0.0_f64, |acc, p| acc.max(p.v));
            let phases = held_at_rest(
                reconstructed_phases[idx].clone(),
                budgets[j].rest_holds_s,
                line_no,
            );
            assert!(
                !phases.is_empty(),
                "line {line_no}: non-zero-duration velocity move has no phases"
            );
            report.traversal_time_s += phases.iter().map(|p| p.dt).sum::<f64>();

            let curvature_ceiling = disk::limit_speed(caps[j].kappa_peak, kin.accel);
            if caps[j].kappa_peak > 0.0 && peak_v > curvature_ceiling + VELOCITY_EPS_MM_S {
                report.limit_ride += 1;
            }

            boundaries.push(if is_anchor[j + 1] && v[j + 1] <= VELOCITY_EPS_MM_S {
                0.0
            } else {
                v[j + 1]
            });
            out.push(MoveVelocity {
                entry_v,
                exit_v,
                peak_v,
                samples,
                phases,
                accel: kin.accel,
                length: kin.length,
                source: m.source,
                step_capacity_overload: caps[j].step_capacity_overload,
                corner_speed_mm_s: caps[j].corner_speed,
            });
        }
        run_start = run_end;
    }

    Ok((out, boundaries))
}

/// `phases` with the toolhead held still for `before` ahead of them and
/// `after` past them; only an extrude-only move, which runs from rest to
/// rest on constant accelerations, is held.
fn held_at_rest(
    phases: Vec<LawSegment>,
    [before, after]: [f64; 2],
    line_no: u32,
) -> Vec<LawSegment> {
    if before == 0.0 && after == 0.0 {
        return phases;
    }
    let first = phases
        .first()
        .expect("a member profile always carries a segment");
    let (t0, s0) = (first.t0, first.s0);
    let last = phases
        .last()
        .expect("a member profile always carries a segment");
    let (s_end, v_end, _) = last.end_state();
    let t_end = last.end_time();
    assert!(
        first.v0 <= VELOCITY_EPS_MM_S && v_end <= VELOCITY_EPS_MM_S,
        "line {line_no}: only a move from rest to rest holds at rest, it runs {} -> {v_end} mm/s",
        first.v0
    );
    let still = ScalarLaw::ConstAccel { a0: 0.0 };
    let mut held = Vec::with_capacity(phases.len() + 2);
    if before > 0.0 {
        held.push(LawSegment::new(t0, before, s0, 0.0, still));
    }
    held.extend(
        phases.iter().map(|phase| {
            LawSegment::new(phase.t0 + before, phase.dt, phase.s0, phase.v0, phase.law)
        }),
    );
    if after > 0.0 {
        held.push(LawSegment::new(t_end + before, after, s_end, 0.0, still));
    }
    held
}

type MemberProfile = Result<Vec<LawSegment>, reconstruct::ReconstructError>;

/// Settled members the reverse pass queues for other threads before it
/// builds the next one itself: enough to keep every participant busy, few
/// enough that their rails stay a small slice of the window's.
const SETTLED_BACKLOG_MAX: usize = 4 * RECONSTRUCT_WORKERS_MAX;

/// A member whose entry and exit seam speeds are final, with the rails its
/// profile may reuse.
struct SettledMember {
    j: usize,
    entry_v: f64,
    exit_v: f64,
    forward_rail: Option<LawSegment>,
    backward_rail: Option<LawSegment>,
}

#[derive(Default)]
struct Handoff {
    queue: VecDeque<SettledMember>,
    lead_done: bool,
    built: Vec<(usize, MemberProfile)>,
}

/// Builds member profiles while the reverse pass is still running. A
/// member's profile needs only its two seam speeds, final once the reverse
/// pass has passed its entry seam, so the pass hands each member over as it
/// settles and the other participants build it meanwhile; its rails are
/// spent the moment its profile exists rather than the whole window's rails
/// living until the end of the plan. Each profile is in member-local terms of
/// the run it belongs to (a run starts at each anchored seam).
struct ProfileShop<'a> {
    caps: &'a [MoveCaps],
    run_starts: Vec<usize>,
    participants: usize,
    handoff: Mutex<Handoff>,
    settled: Condvar,
}

impl<'a> ProfileShop<'a> {
    fn new(caps: &'a [MoveCaps], is_anchor: &[bool]) -> Self {
        let run_starts = (0..caps.len())
            .scan(0, |run_start, j| {
                if is_anchor[j] {
                    *run_start = j;
                }
                Some(*run_start)
            })
            .collect();
        let participants = if cfg!(not(target_arch = "wasm32")) {
            std::thread::available_parallelism()
                .map_or(1, |cores| cores.get())
                .min(RECONSTRUCT_WORKERS_MAX)
        } else {
            1
        };
        Self {
            caps,
            run_starts,
            participants,
            handoff: Mutex::new(Handoff::default()),
            settled: Condvar::new(),
        }
    }

    fn lock(&self) -> MutexGuard<'_, Handoff> {
        self.handoff
            .lock()
            .expect("member profile hand-off poisoned")
    }

    /// Queues `member` for another participant, or builds it on this thread
    /// when there is none or the backlog is full.
    fn hand_over(&self, member: SettledMember) {
        if self.participants > 1 {
            let mut handoff = self.lock();
            if handoff.queue.len() < SETTLED_BACKLOG_MAX {
                handoff.queue.push_back(member);
                drop(handoff);
                self.settled.notify_one();
                return;
            }
        }
        self.build(member);
    }

    fn build(&self, member: SettledMember) {
        let j = member.j;
        let run_member = disk::RunMember {
            kin: &self.caps[j].kin,
            forward_rail: member.forward_rail.as_ref(),
            backward_rail: member.backward_rail.as_ref(),
        };
        let profile = reconstruct::member_profile(
            j - self.run_starts[j],
            &run_member,
            member.entry_v,
            member.exit_v,
        );
        let mut handoff = self.lock();
        handoff.built.push((j, profile));
    }

    /// Builds handed-over members until the lead is done and none is left.
    fn follow(&self) {
        while let Some(member) = self.next_settled() {
            self.build(member);
        }
    }

    fn next_settled(&self) -> Option<SettledMember> {
        let mut handoff = self.lock();
        loop {
            if let Some(member) = handoff.queue.pop_front() {
                return Some(member);
            }
            if handoff.lead_done {
                return None;
            }
            handoff = self
                .settled
                .wait(handoff)
                .expect("member profile hand-off poisoned");
        }
    }

    /// Every profile of the reconstructed prefix, in member order.
    fn into_profiles(self, prefix: usize) -> Vec<MemberProfile> {
        let handoff = self
            .handoff
            .into_inner()
            .expect("member profile hand-off poisoned");
        let mut profiles: Vec<Option<MemberProfile>> = vec![None; prefix];
        for (j, profile) in handoff.built {
            profiles[j] = Some(profile);
        }
        profiles
            .into_iter()
            .enumerate()
            .map(|(j, profile)| {
                profile.unwrap_or_else(|| {
                    panic!("member {j} of the reconstructed prefix has no profile")
                })
            })
            .collect()
    }
}

/// Tells the followers the lead hands over nothing more, also when it
/// unwinds.
struct LeadDone<'s, 'a>(&'s ProfileShop<'a>);

impl Drop for LeadDone<'_, '_> {
    fn drop(&mut self) {
        self.0.lock().lead_done = true;
        self.0.settled.notify_all();
    }
}

/// The reverse pass's side of the shop: routes each member as its seams
/// settle — handed over when the plan reconstructs it, its rails spent when
/// it does not, held until the prefix is known otherwise.
struct Settler<'s, 'a, F> {
    shop: &'s ProfileShop<'a>,
    select_prefix: Option<F>,
    prefix: Option<usize>,
    awaiting_prefix: Vec<usize>,
}

impl<'s, 'a, F: FnOnce(usize) -> usize> Settler<'s, 'a, F> {
    fn new(shop: &'s ProfileShop<'a>, select_prefix: F) -> Self {
        Self {
            shop,
            select_prefix: Some(select_prefix),
            prefix: None,
            awaiting_prefix: Vec::new(),
        }
    }

    /// Fixes how many members the plan reconstructs from the finality
    /// barrier, and routes every member held for it.
    fn select(&mut self, plan: &mut SeamPlan, barrier: usize) {
        let select_prefix = self
            .select_prefix
            .take()
            .expect("the reconstructed prefix is selected once per plan");
        let prefix = select_prefix(barrier);
        let n = self.shop.caps.len();
        assert!(
            prefix <= n,
            "cannot reconstruct {prefix} moves from a {n}-move plan"
        );
        self.prefix = Some(prefix);
        for j in std::mem::take(&mut self.awaiting_prefix) {
            self.settle(plan, j);
        }
    }

    fn prefix_or_select(&mut self, plan: &mut SeamPlan, barrier: usize) -> usize {
        if self.prefix.is_none() {
            self.select(plan, barrier);
        }
        self.prefix.expect("select fixes the prefix")
    }

    /// Member `j`'s entry and exit seam speeds are final.
    fn settle(&mut self, plan: &mut SeamPlan, j: usize) {
        match self.prefix {
            None => self.awaiting_prefix.push(j),
            Some(prefix) if j < prefix => self.shop.hand_over(SettledMember {
                j,
                entry_v: plan.v[j],
                exit_v: plan.v[j + 1],
                forward_rail: plan.forward_rails[j].take(),
                backward_rail: plan.backward_rails[j].take(),
            }),
            Some(_) => plan.release_rails(j),
        }
    }
}

/// Dense-enough exact samples off a member's law segments: the segment
/// boundaries plus uniform interior points, every one evaluated from the law.
fn sample_segments(segments: &[LawSegment]) -> Vec<(f64, f64, f64)> {
    const INTERIOR: usize = 8;
    let mut out = Vec::with_capacity(segments.len() * (INTERIOR + 1) + 1);
    for seg in segments {
        for i in 0..=INTERIOR {
            let t = seg.t0 + seg.dt * (i as f64) / INTERIOR as f64;
            let (s, v, a) = seg.state_at(t);
            if out
                .last()
                .is_none_or(|&(prev_s, _, _): &(f64, f64, f64)| s > prev_s + 1e-12)
            {
                out.push((s, v, a));
            }
        }
    }
    out
}

fn first_negative_velocity(samples: &[VelSample]) -> Option<f64> {
    samples
        .iter()
        .map(|p| p.v)
        .find(|&v| v < -NEGATIVE_VELOCITY_TOL_MM_S)
}

#[cfg(test)]
fn traversal_time(samples: &[VelSample]) -> f64 {
    samples
        .windows(2)
        .map(|w| {
            let ds = w[1].s - w[0].s;
            let v_sum = w[0].v + w[1].v;
            if v_sum > 0.0 { 2.0 * ds / v_sum } else { 0.0 }
        })
        .sum()
}

fn validate_segment<P: CurvatureProfile>(
    seg: &P,
    length: f64,
    line_no: u32,
    tol: f64,
) -> Result<(), VelocityError> {
    if !(length.is_finite() && length > LENGTH_EPS_MM) {
        return Err(VelocityError::NonFinite { line_no });
    }
    let (s_peak, kappa_peak) = seg.kappa_peak();
    let sigma = seg.dkappa_ds(0.0);
    if !(kappa_peak.is_finite() && sigma.is_finite() && s_peak.is_finite()) {
        return Err(VelocityError::NonFinite { line_no });
    }
    let endpoint_tol = tol * length;
    let at_endpoint = s_peak.abs() <= endpoint_tol || (s_peak - length).abs() <= endpoint_tol;
    if !at_endpoint {
        return Err(VelocityError::NonAlphabet { line_no });
    }
    let (kappa_start, kappa_end) = seg.kappa_endpoints();
    let sigma_implied = (kappa_end - kappa_start) / length;
    if (sigma_implied - sigma).abs() > tol * sigma.abs().max(1.0) {
        return Err(VelocityError::Inconsistent { line_no });
    }
    Ok(())
}

#[cfg(test)]
mod plan_time_monotone_tests;
#[cfg(test)]
mod replan_tests;
#[cfg(test)]
mod tests;
