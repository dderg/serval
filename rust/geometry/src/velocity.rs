#[cfg(test)]
use std::collections::HashSet;

use crate::LENGTH_EPS_MM;
#[cfg(test)]
use crate::fitter::{FitOutcome, UnblendReason};
use crate::path::CurvatureProfile;
#[cfg(test)]
use crate::path::Segment;
use crate::segment::SourceRange;

mod certify;
mod chain;
mod compose;
mod curved;
mod disk;
mod profile;
mod scurve;

pub use profile::{
    BoundaryInfeasibility, Profile, StraightPhase, plan as plan_profile, straight_chain,
    straight_chain_between,
};

use curved::AccelWindow;
use disk::Kinematics;

const VELOCITY_EPS_MM_S: f64 = 1e-9;
const MIN_INTEGRATION_TOL: f64 = 1e-9;
const NEGATIVE_VELOCITY_TOL_MM_S: f64 = 1e-6;
const CONSISTENCY_TOL: f64 = 1e-6;

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
    /// Closed-form jerk phases in move-local time/arc-length: the plan itself.
    /// The lowering emits one exact cubic per phase for a straight move and
    /// fits axis positions against the phases' exact scalar profile for a
    /// curved one. `samples` is a derived dense `(s, v, a)` view of these same
    /// phases, never an independent reconstruction.
    pub phases: Vec<StraightPhase>,
    pub accel: f64,
    pub jerk: f64,
    pub length: f64,
    pub source: SourceRange,
}

/// The member the envelope solver could not plan between the two boundary
/// states its neighbours fixed, named so the failure is actionable rather than
/// a bare count.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UnreachableEntryState {
    pub move_index: usize,
    pub line_no: u32,
    pub entry: BoundaryState,
    pub exit: BoundaryState,
}

impl UnreachableEntryState {
    fn accel_magnitude(&self) -> f64 {
        self.entry.a.abs().max(self.exit.a.abs())
    }
}

/// Per-cause tally of the boundary infeasibilities a member's chain reported.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct InfeasibilityTally {
    pub length_too_short: u32,
    pub unwind_over_ceiling: u32,
    pub unwind_below_rest: u32,
    pub accel_over_limit: u32,
    pub speed_change_without_authority: u32,
    pub length_not_closed: u32,
    pub non_finite: u32,
    pub unbounded_jerk_with_accel_boundary: u32,
    pub uncertified_phase: u32,
    pub other: u32,
}

impl InfeasibilityTally {
    pub fn total(&self) -> u32 {
        self.length_too_short
            + self.unwind_over_ceiling
            + self.unwind_below_rest
            + self.accel_over_limit
            + self.speed_change_without_authority
            + self.length_not_closed
            + self.non_finite
            + self.unbounded_jerk_with_accel_boundary
            + self.uncertified_phase
            + self.other
    }

    fn record(&mut self, e: &VelocityError) {
        let slot = match e {
            VelocityError::UncertifiedPhase { .. } => &mut self.uncertified_phase,
            VelocityError::InfeasibleBoundary(why) => match why {
                BoundaryInfeasibility::LengthTooShort { .. } => &mut self.length_too_short,
                BoundaryInfeasibility::UnwindOverCeiling { .. } => &mut self.unwind_over_ceiling,
                BoundaryInfeasibility::UnwindBelowRest { .. } => &mut self.unwind_below_rest,
                BoundaryInfeasibility::AccelOverLimit { .. } => &mut self.accel_over_limit,
                BoundaryInfeasibility::SpeedChangeWithoutAuthority { .. } => {
                    &mut self.speed_change_without_authority
                }
                BoundaryInfeasibility::LengthNotClosed { .. } => &mut self.length_not_closed,
                BoundaryInfeasibility::NonFinite => &mut self.non_finite,
                BoundaryInfeasibility::UnboundedJerkWithAccelBoundary { .. } => {
                    &mut self.unbounded_jerk_with_accel_boundary
                }
            },
            _ => &mut self.other,
        };
        *slot += 1;
    }
}

/// Census of the envelope's per-member plans: how many members were planned
/// between their two boundary states, how many of those plans failed, and why.
/// A failed plan is now a planning error, so a run that returns at all carries a
/// census whose refusal counts are zero; the tallies are what names the cause
/// when one is not.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct EntryReachability {
    pub straight_members: u32,
    pub curved_members: u32,
    pub unreachable: u32,
    /// Members that no entry acceleration whatsoever would have let the envelope
    /// traverse to their exit: infeasible geometry at these limits rather than a
    /// state badly handed over. A subset of `unreachable`.
    pub no_admissible_entry: u32,
    pub straight: InfeasibilityTally,
    pub curved: InfeasibilityTally,
    /// Seams where the interval of entry accelerations the member admits and the
    /// interval its predecessor can deliver do not overlap: infeasible geometry
    /// at these limits, not a solver shortfall.
    pub accel_window_empty: u32,
}

impl EntryReachability {
    pub fn member_plans(&self) -> u32 {
        self.straight_members + self.curved_members
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct VelocityReport {
    pub stops: u32,
    pub curvature_bound: u32,
    pub feedrate_bound: u32,
    pub jerk_bound: u32,
    pub limit_ride: u32,
    pub traversal_time_s: f64,
    /// Seams whose boundary state carries a nonzero acceleration inherited from
    /// the member downstream of them: the brake a blend interior needs, carried
    /// into a seam that has no authority to build one.
    pub boundary_accel_seams: u32,
    pub worst_boundary_accel_mm_s2: f64,
    pub reachability: EntryReachability,
    pub worst_unreachable: Option<UnreachableEntryState>,
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
    /// Reconstructed profile state at every move boundary (`n + 1` entries,
    /// `boundaries[0]` mirrors the given entry). A streaming caller that cuts
    /// the window at seam `k` warm-starts the re-plan from `boundaries[k]`:
    /// the carried `(v, a)` is the profile's state at the seam (velocity
    /// clamped to the analytic node bound so a re-plan's entry checks accept
    /// it by construction), so the next window continues the same
    /// jerk-limited curve instead of re-anchoring at zero acceleration —
    /// which both bends the trajectory (an acceleration discontinuity at the
    /// cut) and can be outright infeasible when the profile crosses the seam
    /// mid-brake.
    pub boundaries: Vec<BoundaryState>,
}

/// The `(v, a)` state of a velocity profile at a move boundary: the full
/// state of the jerk-limited forward reconstruction, and therefore everything
/// a re-plan needs to continue the profile across a window cut.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoundaryState {
    pub v: f64,
    pub a: f64,
}

impl BoundaryState {
    pub const REST: Self = Self { v: 0.0, a: 0.0 };
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VelocityError {
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
    RestAnchorAccel {
        line_no: u32,
    },
    NegativeVelocity {
        line_no: u32,
        v: f64,
    },
    InvalidConfig,
    InfeasibleBoundary(BoundaryInfeasibility),
    UncertifiedPhase {
        s0: f64,
        v0: f64,
        a0: f64,
        j: f64,
        dt: f64,
    },
}

const REST_ANCHOR_ACCEL_EPS: f64 = 1e-3;

fn pin_rest_anchor(
    sample: Option<&mut VelSample>,
    line_no: u32,
    jerk: f64,
) -> Result<(), VelocityError> {
    if let Some(s) = sample {
        if jerk.is_finite() && s.a.abs() > REST_ANCHOR_ACCEL_EPS {
            return Err(VelocityError::RestAnchorAccel { line_no });
        }
        s.a = 0.0;
    }
    Ok(())
}

struct MoveCaps {
    kin: Kinematics,
    kappa_peak: f64,
}

#[cfg(test)]
pub(crate) fn plan_velocity_warm_start(
    outcome: &FitOutcome,
    integration_tol: f64,
    max_extrude_only_velocity_mm_s: f64,
    max_extrude_only_accel_mm_s2: f64,
    entry: BoundaryState,
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
    plan_velocity_stops(
        &outcome.moves,
        &stop_before,
        integration_tol,
        max_extrude_only_velocity_mm_s,
        max_extrude_only_accel_mm_s2,
        entry,
    )
}

/// Plan over an already-fitted move sequence with explicit per-seam stop
/// anchors: `stop_before[k]` forces rest at the seam entering `moves[k]`.
/// `stop_before[0]` is ignored — the entry seam is anchored at `entry`, the
/// profile state a streaming cut carried out of the previous window.
pub fn plan_velocity_stops(
    moves: &[crate::Move],
    stop_before: &[bool],
    integration_tol: f64,
    max_extrude_only_velocity_mm_s: f64,
    max_extrude_only_accel_mm_s2: f64,
    entry: BoundaryState,
) -> Result<VelocityProfile, VelocityError> {
    let tol = integration_tol;
    validate_config(
        tol,
        entry,
        max_extrude_only_velocity_mm_s,
        max_extrude_only_accel_mm_s2,
    )?;

    let n = moves.len();
    assert_eq!(stop_before.len(), n, "one stop flag per move");
    if n == 0 {
        return Ok(VelocityProfile {
            moves: Vec::new(),
            report: VelocityReport::default(),
            barrier: 0,
            v_barrier: 0.0,
            boundaries: vec![entry],
        });
    }

    let mut report = VelocityReport::default();
    let caps = build_move_caps(
        moves,
        max_extrude_only_velocity_mm_s,
        max_extrude_only_accel_mm_s2,
        &mut report,
    )?;
    check_entry_ceiling(moves, &caps, entry, tol)?;
    let mut plan = seed_seam_velocities(&caps, stop_before, entry, &mut report);
    let geo = compute_run_geometry(&caps, &plan);
    forward_pass(moves, &caps, &geo, &mut plan.v, tol)?;
    let (barrier, _) = reverse_brake_envelope(moves, &caps, &geo, &mut plan.v, tol)?;
    check_entry_brake(moves, &caps, &geo, &plan.v, entry, tol)?;
    let envelope = plan.v.clone();
    seam_accel_demands(&caps, &mut plan);
    settle_seam_states(moves, &caps, &envelope, &mut plan, &mut report)?;
    repair_stale_seam_accels(&caps, &envelope, &mut plan);
    let v_barrier = plan.v[barrier];
    let (out, boundaries) = reconstruct_runs(moves, &caps, &plan, &geo, entry, tol, &mut report)?;

    Ok(VelocityProfile {
        moves: out,
        report,
        barrier,
        v_barrier,
        boundaries,
    })
}

fn validate_config(
    tol: f64,
    entry: BoundaryState,
    max_extrude_only_velocity_mm_s: f64,
    max_extrude_only_accel_mm_s2: f64,
) -> Result<(), VelocityError> {
    if !(tol.is_finite() && tol >= MIN_INTEGRATION_TOL) {
        return Err(VelocityError::InvalidConfig);
    }
    if !(entry.v.is_finite() && entry.v >= 0.0 && entry.a.is_finite()) {
        return Err(VelocityError::InvalidConfig);
    }
    if !(max_extrude_only_velocity_mm_s > 0.0 && max_extrude_only_accel_mm_s2 > 0.0) {
        return Err(VelocityError::InvalidConfig);
    }
    Ok(())
}

fn build_move_caps(
    moves: &[crate::Move],
    max_extrude_only_velocity_mm_s: f64,
    max_extrude_only_accel_mm_s2: f64,
    report: &mut VelocityReport,
) -> Result<Vec<MoveCaps>, VelocityError> {
    let mut caps = Vec::with_capacity(moves.len());
    for m in moves {
        let line_no = m.source.start_line;
        let mut accel = m.limits.accel_mm_s2;
        let mut extrude_only_velocity_cap = f64::INFINITY;
        let (length, kappa0, sigma, kappa_peak) = match &m.segment.spatial {
            Some(seg) => {
                let length = seg.s_len();
                validate_segment(seg, length, line_no, CONSISTENCY_TOL)?;
                let (kappa_start, _) = seg.kappa_endpoints();
                let sigma = seg.dkappa_ds(0.0);
                let (_, kappa_peak) = seg.kappa_peak();
                // A corner's apex speed is `√(a/κ)`, so the acceleration spent
                // on curved geometry is what fixes the trajectory through it.
                // Capping it here leaves the straights the full budget: raising
                // `accel` then buys shorter ramps without ever speeding a corner
                // up, which is the whole point of having the two limits differ.
                if kappa_peak > 0.0 {
                    accel = accel.min(m.limits.corner_accel_mm_s2);
                }
                (length, kappa_start, sigma, kappa_peak)
            }
            None => {
                let length = m
                    .segment
                    .virtual_path_mm
                    .ok_or(VelocityError::NonFinite { line_no })?;
                if !(length.is_finite() && length > LENGTH_EPS_MM) {
                    return Err(VelocityError::NonFinite { line_no });
                }
                accel = accel.min(max_extrude_only_accel_mm_s2);
                extrude_only_velocity_cap = max_extrude_only_velocity_mm_s;
                (length, 0.0, 0.0, 0.0)
            }
        };

        let flat_ceiling = m
            .feedrate_mm_s
            .min(m.limits.max_velocity_mm_s)
            .min(extrude_only_velocity_cap);
        if disk::limit_speed(kappa_peak, accel) < flat_ceiling {
            report.curvature_bound += 1;
        } else {
            report.feedrate_bound += 1;
        }
        caps.push(MoveCaps {
            kin: Kinematics {
                length,
                accel,
                jerk: m.limits.max_jerk_mm_s3,
                kappa0,
                sigma,
                flat_ceiling,
            },
            kappa_peak,
        });
    }
    Ok(caps)
}

fn check_entry_ceiling(
    moves: &[crate::Move],
    caps: &[MoveCaps],
    entry: BoundaryState,
    tol: f64,
) -> Result<(), VelocityError> {
    let entry_ceiling = {
        let kin0 = &caps[0].kin;
        kin0.flat_ceiling
            .min(disk::limit_speed(kin0.kappa0.abs(), kin0.accel))
    };
    if entry.v > entry_ceiling + tol * (1.0 + entry_ceiling) {
        return Err(VelocityError::OverCommitted {
            line_no: moves[0].source.start_line,
        });
    }
    Ok(())
}

struct SeamPlan {
    v: Vec<f64>,
    a: Vec<f64>,
    is_anchor: Vec<bool>,
}

fn seed_seam_velocities(
    caps: &[MoveCaps],
    stop_before: &[bool],
    entry: BoundaryState,
    report: &mut VelocityReport,
) -> SeamPlan {
    let n = caps.len();
    let mut v = vec![0.0_f64; n + 1];
    v[0] = entry.v;
    let mut a = vec![0.0_f64; n + 1];
    a[0] = entry.a;
    let mut is_anchor = vec![false; n + 1];
    is_anchor[0] = true;
    is_anchor[n] = true;
    for k in 1..n {
        if stop_before[k] {
            report.stops += 1;
            is_anchor[k] = true;
        } else {
            v[k] = seam_ceiling(caps, k);
        }
    }
    SeamPlan { v, a, is_anchor }
}

/// Highest speed the geometry itself allows at seam `k`, before any envelope
/// sweep narrows it: the tighter of the two members' flat ceilings, the disk
/// limit their curvatures there impose, and the speed both can actually hold
/// there. Holding every seam to this keeps the forward and backward sweeps from
/// building an envelope above the jerk rail.
fn seam_ceiling(caps: &[MoveCaps], k: usize) -> f64 {
    let up = &caps[k - 1].kin;
    let dn = &caps[k].kin;
    let kappa_up = (up.kappa0 + up.sigma * up.length).abs();
    let kappa_dn = dn.kappa0.abs();
    let boundary_vlim =
        disk::limit_speed(kappa_up, up.accel).min(disk::limit_speed(kappa_dn, dn.accel));
    let ceiling = up.flat_ceiling.min(dn.flat_ceiling);
    ceiling
        .min(disk::notch_free_min(ceiling, boundary_vlim))
        .min(seam_hold_ceiling(caps, k))
}

/// The `(v, a)` a member requires at its entry to land on the exit state the
/// envelope has already fixed for it. Solved backward: a curved member by the
/// closed-form curved solver, a straight one by the reversed jerk-limited reach,
/// whose acceleration flips sign on the way back. Under unlimited jerk the
/// acceleration is free at every seam, so the member requires nothing.
fn required_entry_state(kin: &Kinematics, exit: BoundaryState) -> Option<BoundaryState> {
    if !kin.jerk.is_finite() {
        return None;
    }
    if kin.is_straight() {
        let (v, a) =
            scurve::reach_velocity_with_accel(exit.v, -exit.a, kin.length, kin.accel, kin.jerk)
                .ok()?;
        if v > kin.flat_ceiling {
            return Some(BoundaryState {
                v: kin.flat_ceiling,
                a: 0.0,
            });
        }
        Some(BoundaryState { v, a: -a })
    } else if disk::curved_solver_is_available(kin) {
        let (v, a) = curved::entry_requirement(kin, (exit.v, exit.a)).ok()?;
        Some(BoundaryState { v, a })
    } else {
        None
    }
}

/// How tightly a member's entry requirement must match the envelope velocity
/// before the required acceleration is adopted as the seam's boundary state.
const REQUIREMENT_BIND_REL: f64 = 1e-9;

/// Highest speed a member can be held at where it meets the seam: its curvature
/// *there*, not its peak. The disk envelope only bounds `kappa^2 v^4` against
/// the acceleration budget; a clothoid also owes `sigma v^3` of normal jerk with
/// no acceleration term to cancel it, so a blend seam has a speed cap the disk
/// never sees — and a blend entered at `kappa = 0` must not also pay its apex's
/// disk price at the seam, which is a speed the seam can hold perfectly well.
fn hold_ceiling_at(kin: &Kinematics, kappa_at_seam: f64) -> f64 {
    curved::top_speed_ceiling(&Kinematics {
        length: 0.0,
        accel: kin.accel,
        jerk: kin.jerk,
        kappa0: kappa_at_seam,
        sigma: kin.sigma,
        flat_ceiling: kin.flat_ceiling,
    })
}

fn seam_hold_ceiling(caps: &[MoveCaps], k: usize) -> f64 {
    let solvable = |c: &&MoveCaps| disk::curved_solver_is_available(&c.kin);
    let upstream = caps
        .get(k - 1)
        .filter(solvable)
        .map(|c| hold_ceiling_at(&c.kin, c.kin.kappa0 + c.kin.sigma * c.kin.length));
    let downstream = caps
        .get(k)
        .filter(solvable)
        .map(|c| hold_ceiling_at(&c.kin, c.kin.kappa0));
    upstream
        .into_iter()
        .chain(downstream)
        .fold(f64::INFINITY, f64::min)
}

/// The widest interval of entry accelerations the member downstream of seam `k`
/// admits, entered at `v_seam` and landing on the exit state already fixed for
/// it. `None` where no entry acceleration serves at all, or where the member has
/// no solver to ask.
fn seam_accel_window(
    caps: &[MoveCaps],
    k: usize,
    v_seam: f64,
    exit: BoundaryState,
) -> Option<AccelWindow> {
    let kin = &caps[k].kin;
    if !disk::curved_solver_is_available(kin) {
        return Some(AccelWindow { lo: 0.0, hi: 0.0 });
    }
    curved::entry_accel_window(kin, v_seam, (exit.v, exit.a)).ok()
}

/// Grid points the settlement sweeps across a seam's speed band looking for one
/// it can settle at. The band's own ends are excluded: the top is the speed that
/// already failed, and a seam at rest is a stop, not a settlement.
const SEAM_SLOWDOWN_PROBES: u32 = 16;

const SEAM_VELOCITY_BISECT_ITERS: u32 = 14;

/// Share of its speed a seam gives up when the seam after it cannot settle at
/// any speed: the successor's demand is unreachable because the *predecessor*
/// arrives too fast to shed what the seam needs shed, and only slowing the seam
/// before it opens the band.
const SEAM_RETREAT_SHRINK: f64 = 0.85;

/// Retreats the settlement may spend per member before it declares the run
/// infeasible at these limits. Bounded so the search cannot iterate forever on
/// geometry that will never settle.
const SEAM_RETREATS_PER_MEMBER: u32 = 8;

/// Walk the members right to left, each publishing the `(v, a)` it requires at
/// its entry as a boundary condition on its predecessor.
///
/// Where a member's fastest entry requirement is at or below the velocity
/// envelope the seam inherits the whole state — the brake the member's interior
/// needs is carried into the seam instead of being manufactured there, which a
/// `kappa = 0` seam has no authority to do. Otherwise the velocity stays the
/// envelope's and the acceleration comes from the interval the member admits at
/// that velocity, which is the query `entry_requirement` cannot answer: it names
/// one *fastest* `(v, a)` and leaves the acceleration at zero wherever the speed
/// does not bind, handing the predecessor a state the member's interior does not
/// actually accept.
///
/// The chain of demands this leaves is self-consistent by construction — every
/// member closes from `(v[k], a[k])` to `(v[k + 1], a[k + 1])` — everywhere
/// except at a run's own entry, which is an anchor the pass cannot move.
/// [`settle_seam_states`] repairs it from there forward.
fn seam_accel_demands(caps: &[MoveCaps], plan: &mut SeamPlan) {
    for k in (1..caps.len()).rev() {
        if plan.is_anchor[k] {
            continue;
        }
        let exit = BoundaryState {
            v: plan.v[k + 1],
            a: plan.a[k + 1],
        };
        let binds = required_entry_state(&caps[k].kin, exit)
            .filter(|required| required.v <= plan.v[k] * (1.0 + REQUIREMENT_BIND_REL));
        match binds {
            Some(required) => {
                plan.v[k] = required.v;
                plan.a[k] = required.a;
            }
            None => {
                let forward = (!plan.is_anchor[k + 1])
                    .then(|| curved::reachable_exit(&caps[k].kin, (plan.v[k], plan.a[k])))
                    .flatten()
                    .filter(|reached| {
                        (reached.0 - plan.v[k + 1]).abs()
                            <= REQUIREMENT_BIND_REL * (1.0 + plan.v[k + 1])
                    });
                if let Some((_, a)) = forward {
                    plan.a[k + 1] = a;
                    continue;
                }
                plan.a[k] = seam_accel_window(caps, k, plan.v[k], exit)
                    .map_or(0.0, |window| window.nearest_to(0.0));
            }
        }
    }
}

/// Largest seam velocity in `(lo, hi]` that `settles`.
///
/// The band of speeds a seam can settle at is bounded below as well as above: a
/// predecessor cannot brake to an arbitrarily low exit speed inside its own
/// length, so descending blindly walks straight out of the feasible band. The
/// search therefore sweeps the whole band from the top down for a grid point that
/// settles, then bisects between it and the point above it.
fn slowest_settling_seam<T>(
    lo: f64,
    hi: f64,
    settles: impl Fn(f64) -> Option<T>,
) -> Option<(f64, T)> {
    if !(hi > lo) {
        return None;
    }
    let step = (hi - lo) / f64::from(SEAM_SLOWDOWN_PROBES);
    let mut found = None;
    for i in (1..SEAM_SLOWDOWN_PROBES).rev() {
        let v = lo + step * f64::from(i);
        if let Some(settled) = settles(v) {
            found = Some((v, settled));
            break;
        }
    }
    let (mut good, mut settled) = found?;
    let mut bad = (good + step).min(hi);
    for _ in 0..SEAM_VELOCITY_BISECT_ITERS {
        let mid = 0.5 * (good + bad);
        if mid <= good || mid >= bad {
            break;
        }
        match settles(mid) {
            Some(next) => {
                good = mid;
                settled = next;
            }
            None => bad = mid,
        }
    }
    Some((good, settled))
}

/// Slowest seam velocity in `(lo, hi]` that `settles`, for a seam whose whole
/// demanded band is infeasible.
///
/// The backward requirement pass names each seam the speed its own member wants,
/// and the member before it may be unable to brake that far inside its own length.
/// Where nothing below the demand can be given up — a run's entry is an anchor no
/// pass may move — the seam has to give the demand up *upwards*, towards the
/// envelope the sweeps already proved it can hold. The sweep therefore climbs, and
/// the bisection pushes the answer back down towards the demand so the excess
/// speed carried into the next member is the least the predecessor forces.
fn slowest_undemanded_seam<T>(
    lo: f64,
    hi: f64,
    settles: impl Fn(f64) -> Option<T>,
) -> Option<(f64, T)> {
    if !(hi > lo) {
        return None;
    }
    let step = (hi - lo) / f64::from(SEAM_SLOWDOWN_PROBES);
    let probe = |i: u32| lo + step * f64::from(i);
    let (index, mut good, mut settled) = (1..=SEAM_SLOWDOWN_PROBES)
        .find_map(|i| settles(probe(i)).map(|settled| (i, probe(i), settled)))?;
    let mut bad = probe(index - 1);
    for _ in 0..SEAM_VELOCITY_BISECT_ITERS {
        let mid = 0.5 * (good + bad);
        if mid >= good || mid <= bad {
            break;
        }
        match settles(mid) {
            Some(next) => {
                good = mid;
                settled = next;
            }
            None => bad = mid,
        }
    }
    Some((good, settled))
}

/// Fastest exit speed a member can hand on, entered at `entry`. A curved member
/// answers from its own solver; a straight one from the jerk-limited reach, whose
/// exit acceleration is left running, which is what makes it an upper bound on
/// the next seam.
fn member_reach(kin: &Kinematics, entry: BoundaryState) -> f64 {
    if !kin.is_straight() && disk::curved_solver_is_available(kin) {
        let held = entry.v.min(hold_ceiling_at(kin, kin.kappa0));
        return curved::reachable_exit(kin, (held, entry.a))
            .or_else(|| curved::reachable_exit(kin, (held, 0.0)))
            .map_or(f64::INFINITY, |(v, _)| v);
    }
    scurve::reach_velocity_with_accel(
        entry.v,
        entry.a.clamp(-kin.accel, kin.accel),
        kin.length,
        kin.accel,
        kin.jerk,
    )
    .map_or(f64::INFINITY, |(v, _)| v)
}

/// Acceleration a seam can carry at speed `v`, or `None` when no acceleration
/// serves both members there.
///
/// Three conditions, two of them binding. The predecessor can only hand over
/// what its own — by now final — entry state lets it exit at. The successor can
/// only be entered at an acceleration it can be traversed from at all. The third
/// is a preference — the
/// successor would like the acceleration its backward requirement named, the
/// brake its interior needs carried in rather than manufactured at a seam with no
/// authority to build one — and it narrows the answer only when it can.
fn settled_seam_accel(
    caps: &[MoveCaps],
    k: usize,
    v: f64,
    exit: BoundaryState,
    entry: BoundaryState,
    demand: f64,
) -> Option<f64> {
    let upstream = &caps[k - 1].kin;
    let deliverable = curved::exit_accel_window(upstream, (entry.v, entry.a), v).ok()?;
    if deliverable.contains(demand)
        && curved::curved_chain(&caps[k].kin, (v, demand), (exit.v, exit.a)).is_ok()
    {
        return Some(demand);
    }
    if let Some(preferred) =
        seam_accel_window(caps, k, v, exit).and_then(|needed| deliverable.meet(&needed))
    {
        return Some(preferred.nearest_to(demand));
    }
    let traversable = curved::traversable_entry_accels(&caps[k].kin, v)?;
    Some(deliverable.meet(&traversable)?.nearest_to(demand))
}

/// Highest speed both members at seam `k` can be planned whole at. A seam may hold
/// more than the members it joins — its own curvature is the lowest either of them
/// has — but a state above what the solver can plan the whole member at is one the
/// emitter will refuse, so a seam raised off its demand may not go there.
fn seam_solver_ceiling(caps: &[MoveCaps], k: usize) -> f64 {
    let solvable = |c: &&MoveCaps| disk::curved_solver_is_available(&c.kin);
    [caps.get(k - 1), caps.get(k)]
        .into_iter()
        .flatten()
        .filter(solvable)
        .map(|c| curved::top_speed_ceiling(&c.kin))
        .fold(f64::INFINITY, f64::min)
}
/// Acceleration nearest `demand` at which both members at seam `k` close through
/// speed `v`: what the predecessor can deliver there, met with what the successor
/// needs to reach its own exit. `None` where no acceleration serves both.
fn seam_accel_closing_both(
    caps: &[MoveCaps],
    k: usize,
    v: f64,
    exit: (f64, f64),
    upstream: (f64, f64),
    demand: f64,
) -> Option<f64> {
    let needed = curved::entry_accel_window(&caps[k].kin, v, exit).ok()?;
    let deliverable = curved::exit_accel_window(&caps[k - 1].kin, upstream, v).ok()?;
    Some(needed.meet(&deliverable)?.nearest_to(demand))
}

/// Zero, if the predecessor can hand the anchor over at zero acceleration.
///
/// An anchor's speed is not the envelope's to give up — a stop is a stop, and a
/// window's terminal rest is where the buffer ends — and an anchor carries no
/// acceleration, so the only lever left is the speed of the seam before it.
fn anchored_seam(caps: &[MoveCaps], k: usize, v: f64, entry: BoundaryState) -> Option<f64> {
    curved::exit_accel_window(&caps[k - 1].kin, (entry.v, entry.a), v)
        .ok()
        .filter(|deliverable| deliverable.contains(0.0))
        .map(|_| 0.0)
}

/// Walk the members left to right, turning the backward pass's demands into seam
/// states the run can actually execute.
///
/// Two things are settled per seam, in this order. The velocity envelope is
/// tightened to what the predecessor can reach from its now-final entry state —
/// the earlier sweeps could only assume a zero-acceleration entry, and a seam
/// that gave up speed here drags its successors down with it. Then the seam
/// acceleration is chosen from the intersection of what the predecessor can
/// deliver and what the successor admits.
///
/// Where that intersection is empty the seam is being asked to hold a speed at
/// which no acceleration serves both members, and the envelope gives speed up —
/// never a member's requirement — until one does. Induction carries the run: the
/// seam downstream is handed an entry state its member was proved traversable
/// from, and draws its own acceleration from what that member can then deliver.
///
/// A seam that will not settle even after the seam before it has given up speed
/// is infeasible geometry at these limits. It keeps the nearest deliverable
/// acceleration, so nothing is emitted that is not provably traversable, and it
/// is counted so the census names it rather than the plan quietly failing.
/// Re-pick, right to left, the acceleration of any seam whose member the forward
/// settlement left unable to close.
///
/// The settlement chose each seam's acceleration against the *demand* standing at
/// the seam beyond it, and that demand can move afterwards — the seam beyond is
/// settled later, from its own predecessor's capability. Walking back the other
/// way makes both sides final at once: the exit state is already settled and the
/// entry the predecessor delivers no longer changes, so the intersection of what
/// the member needs and what the predecessor can give is exact. Only seams whose
/// member actually refuses are touched, so a run that settled cleanly pays one
/// closure test per seam and nothing else.
///
/// Where the two windows do not intersect no acceleration serves both members at
/// that speed, and the speed is what has to move — down first, and up towards the
/// envelope where the member before it cannot brake that far. Only then, with
/// nothing left to move, does the seam keep what the predecessor can deliver: the
/// predecessor's chain is planned to this very state and is never revisited, so
/// taking the successor's demand there would trade one refusing member for two.
fn repair_stale_seam_accels(caps: &[MoveCaps], envelope: &[f64], plan: &mut SeamPlan) {
    for k in (1..caps.len()).rev() {
        if plan.is_anchor[k] || !disk::curved_solver_is_available(&caps[k].kin) {
            continue;
        }
        let exit = (plan.v[k + 1], plan.a[k + 1]);
        let upstream = (plan.v[k - 1], plan.a[k - 1]);
        if curved::curved_chain(&caps[k].kin, (plan.v[k], plan.a[k]), exit).is_ok() {
            continue;
        }
        let demand = plan.a[k];
        let both = |v: f64| seam_accel_closing_both(caps, k, v, exit, upstream, demand);
        let hi = envelope[k].min(seam_solver_ceiling(caps, k)).max(plan.v[k]);
        let settled = both(plan.v[k])
            .map(|a| (plan.v[k], a))
            .or_else(|| slowest_settling_seam(0.0, plan.v[k], &both))
            .or_else(|| slowest_undemanded_seam(plan.v[k], hi, &both));
        if let Some((v, a)) = settled {
            plan.v[k] = v;
            plan.a[k] = a;
            continue;
        }
        let Ok(needed) = curved::entry_accel_window(&caps[k].kin, plan.v[k], exit) else {
            continue;
        };
        let deliverable = curved::exit_accel_window(&caps[k - 1].kin, upstream, plan.v[k]).ok();
        plan.a[k] = deliverable.unwrap_or(needed).nearest_to(plan.a[k]);
    }
}

fn settle_seam_states(
    moves: &[crate::Move],
    caps: &[MoveCaps],
    envelope: &[f64],
    plan: &mut SeamPlan,
    report: &mut VelocityReport,
) -> Result<(), VelocityError> {
    let n = caps.len();
    let mut retreats = 0u32;
    let budget = SEAM_RETREATS_PER_MEMBER * u32::try_from(n).unwrap_or(u32::MAX);
    let mut given_up_to = vec![0.0_f64; n + 1];
    let mut k = 1;
    while k <= n {
        let entry = BoundaryState {
            v: plan.v[k - 1],
            a: plan.a[k - 1],
        };
        let kin = &caps[k - 1].kin;
        let kappa_exit = kin.kappa0 + kin.sigma * kin.length;
        let keeps_inverse_trail = entry.a < 0.0 && kappa_exit.abs() > kin.kappa0.abs();
        let reach = if keeps_inverse_trail {
            plan.v[k]
        } else {
            member_reach(kin, entry)
        };
        plan.v[k] = plan.v[k].min(reach).max(given_up_to[k]);
        if !disk::curved_solver_is_available(&caps[k - 1].kin) {
            k += 1;
            continue;
        }
        let beyond = (k + 1).min(n);
        let exit = BoundaryState {
            v: plan.v[beyond],
            a: plan.a[beyond],
        };
        let demand = plan.a[k];
        let settles = |v: f64| settled_seam_accel(caps, k, v, exit, entry, demand);
        let settled = if plan.is_anchor[k] {
            anchored_seam(caps, k, plan.v[k], entry).map(|a| (plan.v[k], a))
        } else {
            settles(plan.v[k])
                .map(|a| (plan.v[k], a))
                .or_else(|| slowest_settling_seam(given_up_to[k], plan.v[k], &settles))
        };
        if let Some((v, a)) = settled {
            plan.v[k] = v;
            plan.a[k] = a;
            k += 1;
            continue;
        }
        let can_retreat = k > 1
            && !plan.is_anchor[k - 1]
            && retreats < budget
            && plan.v[k - 1] * SEAM_RETREAT_SHRINK >= given_up_to[k - 1];
        if can_retreat {
            plan.v[k - 1] *= SEAM_RETREAT_SHRINK;
            retreats += 1;
            k -= 1;
            continue;
        }
        let given_up = (!plan.is_anchor[k])
            .then(|| {
                let hi = envelope[k]
                    .min(reach)
                    .min(seam_solver_ceiling(caps, k))
                    .max(plan.v[k]);
                slowest_undemanded_seam(plan.v[k], hi, &settles)
            })
            .flatten();
        if let Some((v, a)) = given_up {
            given_up_to[k] = v;
            plan.v[k] = v;
            plan.a[k] = a;
            k += 1;
            continue;
        }
        if k == 1 || plan.is_anchor[k - 1] {
            return Err(VelocityError::OverCommitted {
                line_no: moves[k - 1].source.start_line,
            });
        }
        report.reachability.accel_window_empty += 1;
        if let Ok(deliverable) =
            curved::exit_accel_window(&caps[k - 1].kin, (entry.v, entry.a), plan.v[k])
        {
            plan.a[k] = deliverable.nearest_to(plan.a[k]);
        }
        k += 1;
    }
    for k in 1..=n {
        if plan.a[k] != 0.0 {
            report.boundary_accel_seams += 1;
            report.worst_boundary_accel_mm_s2 =
                report.worst_boundary_accel_mm_s2.max(plan.a[k].abs());
        }
    }
    Ok(())
}

struct RunGeometry {
    run_start_v: Vec<f64>,
    run_start_a: Vec<f64>,
    arc_from_run_start: Vec<f64>,
    arc_to_run_end: Vec<f64>,
}

fn compute_run_geometry(caps: &[MoveCaps], plan: &SeamPlan) -> RunGeometry {
    let n = caps.len();
    let mut run_start_v = vec![0.0_f64; n];
    let mut run_start_a = vec![0.0_f64; n];
    let mut arc_from_run_start = vec![0.0_f64; n];
    {
        let mut anchor_v = plan.v[0];
        let mut anchor_a = plan.a[0];
        let mut cum = 0.0;
        for j in 0..n {
            if plan.is_anchor[j] {
                anchor_v = plan.v[j];
                anchor_a = plan.a[j];
                cum = 0.0;
            }
            run_start_v[j] = anchor_v;
            run_start_a[j] = anchor_a;
            arc_from_run_start[j] = cum;
            cum += caps[j].kin.length;
        }
    }
    let mut arc_to_run_end = vec![0.0_f64; n];
    {
        let mut cum = 0.0;
        for j in (0..n).rev() {
            if plan.is_anchor[j + 1] {
                cum = 0.0;
            }
            arc_to_run_end[j] = cum;
            cum += caps[j].kin.length;
        }
    }
    RunGeometry {
        run_start_v,
        run_start_a,
        arc_from_run_start,
        arc_to_run_end,
    }
}

/// Speed a curved member can reach at its exit from a seam speed, or infinity
/// where there is no curved solver to ask. The entry is taken at zero
/// acceleration: the sweeps run before any seam acceleration is fixed, and a
/// zero-acceleration entry reaches no further than an accelerating one, so the
/// cap stays an upper bound.
fn curved_forward_cap(kin: &Kinematics, v_entry: f64) -> f64 {
    if kin.is_straight() || !disk::curved_solver_is_available(kin) {
        return f64::INFINITY;
    }
    let held = v_entry.min(hold_ceiling_at(kin, kin.kappa0));
    curved::reachable_exit(kin, (held, 0.0)).map_or(held, |(v, _)| v)
}

/// Speed a curved member may be entered at and still brought down to `v_exit`.
fn curved_brake_cap(kin: &Kinematics, v_exit: f64) -> f64 {
    if kin.is_straight() || !disk::curved_solver_is_available(kin) {
        return f64::INFINITY;
    }
    let landed = v_exit.min(hold_ceiling_at(kin, kin.kappa0 + kin.sigma * kin.length));
    curved::brake_reach(kin, landed).unwrap_or(landed)
}

fn forward_pass(
    moves: &[crate::Move],
    caps: &[MoveCaps],
    geo: &RunGeometry,
    v: &mut [f64],
    tol: f64,
) -> Result<(), VelocityError> {
    let n = caps.len();
    for k in 1..=n {
        let j = k - 1;
        let line_no = moves[j].source.start_line;
        let kin = &caps[j].kin;
        let disk = disk::disk_reach_v(kin, v[j], kin.length, tol)
            .ok_or(VelocityError::Diverged { line_no })?;
        let jerk = scurve::reach_velocity_with_accel(
            geo.run_start_v[j],
            geo.run_start_a[j].clamp(-kin.accel, kin.accel),
            geo.arc_from_run_start[j] + kin.length,
            kin.accel,
            kin.jerk,
        )
        .map(|(v, _)| v)
        .map_err(|_| VelocityError::Diverged { line_no })?;
        v[k] = v[k].min(disk).min(jerk).min(curved_forward_cap(kin, v[j]));
    }
    Ok(())
}

fn reverse_brake_envelope(
    moves: &[crate::Move],
    caps: &[MoveCaps],
    geo: &RunGeometry,
    v: &mut [f64],
    tol: f64,
) -> Result<(usize, f64), VelocityError> {
    let n = caps.len();
    let v_forward_ceiling = v.to_vec();
    for k in (1..n).rev() {
        let j = k;
        let line_no = moves[j].source.start_line;
        let kin = &caps[j].kin;
        let disk = disk::disk_reach_v_rev(kin, v[k + 1], kin.length, tol)
            .ok_or(VelocityError::Diverged { line_no })?;
        let jerk = scurve::reach_v(0.0, geo.arc_to_run_end[j] + kin.length, kin.accel, kin.jerk)
            .ok_or(VelocityError::Diverged { line_no })?;
        v[k] = v[k]
            .min(disk)
            .min(jerk)
            .min(curved_brake_cap(kin, v[k + 1]));
    }
    let mut barrier = 0usize;
    for k in 1..n {
        if !(v[k] < v_forward_ceiling[k]) {
            barrier = k;
        }
    }
    let v_barrier = v[barrier];
    Ok((barrier, v_barrier))
}

fn check_entry_brake(
    moves: &[crate::Move],
    caps: &[MoveCaps],
    geo: &RunGeometry,
    v: &[f64],
    entry: BoundaryState,
    tol: f64,
) -> Result<(), VelocityError> {
    let entry_line_no = moves[0].source.start_line;
    let entry_brake = {
        let kin = &caps[0].kin;
        let disk =
            disk::disk_reach_v_rev(kin, v[1], kin.length, tol).ok_or(VelocityError::Diverged {
                line_no: entry_line_no,
            })?;
        let jerk = scurve::reach_v(0.0, geo.arc_to_run_end[0] + kin.length, kin.accel, kin.jerk)
            .ok_or(VelocityError::Diverged {
                line_no: entry_line_no,
            })?;
        disk.min(jerk)
    };
    if entry.v > entry_brake + tol * (1.0 + entry_brake) {
        return Err(VelocityError::OverCommitted {
            line_no: entry_line_no,
        });
    }
    Ok(())
}

fn reconstruct_runs(
    moves: &[crate::Move],
    caps: &[MoveCaps],
    plan: &SeamPlan,
    geo: &RunGeometry,
    entry: BoundaryState,
    tol: f64,
    report: &mut VelocityReport,
) -> Result<(Vec<MoveVelocity>, Vec<BoundaryState>), VelocityError> {
    let n = caps.len();
    let v = &plan.v;
    let is_anchor = &plan.is_anchor;
    let mut out: Vec<MoveVelocity> = Vec::with_capacity(n);
    let mut boundaries: Vec<BoundaryState> = Vec::with_capacity(n + 1);
    boundaries.push(entry);
    let mut run_start = 0;
    while run_start < n {
        let mut run_end = run_start + 1;
        while run_end < n && !is_anchor[run_end] {
            run_end += 1;
        }
        let members: Vec<disk::RunMember> = (run_start..run_end)
            .map(|j| disk::RunMember {
                kin: &caps[j].kin,
                exit_v: v[j + 1],
                exit_a: plan.a[j + 1],
                exit_ceiling: if j + 1 < n {
                    seam_ceiling(caps, j + 1)
                } else {
                    caps[j].kin.flat_ceiling
                },
            })
            .collect();
        let run = disk::reconstruct_run(
            &members,
            geo.run_start_v[run_start],
            geo.run_start_a[run_start],
            tol,
        );
        report.reachability.straight_members += run.planned.straight;
        report.reachability.curved_members += run.planned.curved;
        for miss in &run.unreachable {
            let j = run_start + miss.index;
            let named = UnreachableEntryState {
                move_index: j,
                line_no: moves[j].source.start_line,
                entry: BoundaryState {
                    v: miss.entry.0,
                    a: miss.entry.1,
                },
                exit: BoundaryState {
                    v: miss.exit.0,
                    a: miss.exit.1,
                },
            };
            report.reachability.unreachable += 1;
            if disk::curved_solver_is_available(&caps[j].kin)
                && curved::entry_accel_window(&caps[j].kin, miss.entry.0, miss.exit).is_err()
            {
                report.reachability.no_admissible_entry += 1;
            }
            if caps[j].kin.is_straight() {
                report.reachability.straight.record(&miss.why);
            } else {
                report.reachability.curved.record(&miss.why);
            }
            if report
                .worst_unreachable
                .is_none_or(|worst| named.accel_magnitude() > worst.accel_magnitude())
            {
                report.worst_unreachable = Some(named);
            }
        }
        if let Some(miss) = run.unreachable.first() {
            return Err(miss.why);
        }

        for (idx, j) in (run_start..run_end).enumerate() {
            let kin = &caps[j].kin;
            let m = &moves[j];
            let line_no = m.source.start_line;
            let mut samples: Vec<VelSample> = run.samples[idx]
                .iter()
                .map(|&(s, v, a)| VelSample { s, v, a })
                .collect();
            if is_anchor[j] && v[j] <= VELOCITY_EPS_MM_S {
                pin_rest_anchor(samples.first_mut(), line_no, kin.jerk)?;
            }
            if is_anchor[j + 1] && v[j + 1] <= VELOCITY_EPS_MM_S {
                pin_rest_anchor(samples.last_mut(), line_no, kin.jerk)?;
            }
            let entry_v = samples.first().map_or(v[j], |s| s.v);
            let exit_v = samples.last().map_or(v[j + 1], |s| s.v);
            if let Some(v) = first_negative_velocity(&samples) {
                return Err(VelocityError::NegativeVelocity { line_no, v });
            }
            let peak_v = samples.iter().fold(0.0_f64, |acc, p| acc.max(p.v));
            let phases = run.phases[idx].clone();
            // A straight move's phases give the exact traversal time; the sampled
            // estimate mistimes the jerk-from-rest at v = 0 (the singularity the
            // closed-form profile avoids), so prefer the phases when present.
            report.traversal_time_s += if phases.is_empty() {
                traversal_time(&samples)
            } else {
                phases.iter().map(|p| p.dt).sum()
            };

            let disk_only = disk::disk_reach_v(kin, entry_v, kin.length, tol)
                .ok_or(VelocityError::Diverged { line_no })?;
            let jerk_only = scurve::reach_v(entry_v, kin.length, kin.accel, kin.jerk)
                .ok_or(VelocityError::Diverged { line_no })?;
            if jerk_only + VELOCITY_EPS_MM_S < disk_only {
                report.jerk_bound += 1;
            }
            let curvature_ceiling = disk::limit_speed(caps[j].kappa_peak, kin.accel);
            if caps[j].kappa_peak > 0.0 && peak_v > curvature_ceiling + VELOCITY_EPS_MM_S {
                report.limit_ride += 1;
            }

            boundaries.push(if is_anchor[j + 1] && v[j + 1] <= VELOCITY_EPS_MM_S {
                BoundaryState::REST
            } else {
                let (bv, ba) = run.exit_states[idx];
                // Grid integration can land the sample a hair above the
                // analytic node bound; a re-plan re-derives that bound (or a
                // looser one, by append monotonicity) as its entry check, so
                // clamping here is what makes every boundary a valid warm
                // start.
                BoundaryState {
                    v: bv.min(v[j + 1]),
                    a: ba,
                }
            });
            out.push(MoveVelocity {
                entry_v,
                exit_v,
                peak_v,
                samples,
                phases,
                accel: kin.accel,
                jerk: kin.jerk,
                length: kin.length,
                source: m.source,
            });
        }
        run_start = run_end;
    }

    Ok((out, boundaries))
}

fn first_negative_velocity(samples: &[VelSample]) -> Option<f64> {
    samples
        .iter()
        .map(|p| p.v)
        .find(|&v| v < -NEGATIVE_VELOCITY_TOL_MM_S)
}

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
mod jerk_audit_tests;

#[cfg(test)]
mod tests;

#[cfg(test)]
mod certify_tests;

#[cfg(test)]
mod curved_tests;
