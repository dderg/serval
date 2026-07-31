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
mod curved;
mod disk;
mod profile;
mod ride;
mod scurve;

pub use profile::{
    BoundaryInfeasibility, Profile, StraightPhase, plan as plan_profile, straight_chain,
    straight_chain_between,
};

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
    /// Closed-form jerk phases in move-local time/arc-length. Present for
    /// straight moves (the lowering emits one exact cubic per phase) and for
    /// curved moves planned without a jerk limit (the lowering fits axis
    /// positions against the phases' exact scalar profile instead of quintic
    /// windows over `samples`). Empty for finite-jerk curved moves.
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
    /// Members whose backward solve could not name an entry requirement at all.
    pub entry_requirement_unsolved: u32,
    /// Members the forward emission could not plan between their two envelope
    /// boundary states. Until this reaches zero the envelope chains cannot
    /// replace the marched profile.
    pub unreachable_entry_states: u32,
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
    let (barrier, v_barrier) = reverse_brake_envelope(moves, &caps, &geo, &mut plan.v, tol)?;
    check_entry_brake(moves, &caps, &geo, &plan.v, entry, tol)?;
    boundary_state_envelope(&caps, &mut plan, &mut report);
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
            let up = &caps[k - 1].kin;
            let dn = &caps[k].kin;
            let kappa_up = (up.kappa0 + up.sigma * up.length).abs();
            let kappa_dn = dn.kappa0.abs();
            let boundary_vlim =
                disk::limit_speed(kappa_up, up.accel).min(disk::limit_speed(kappa_dn, dn.accel));
            let ceiling = up.flat_ceiling.min(dn.flat_ceiling);
            v[k] = ceiling.min(disk::notch_free_min(ceiling, boundary_vlim));
        }
    }
    SeamPlan { v, a, is_anchor }
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

/// Highest speed both members meeting at a seam can actually hold there. The
/// disk envelope only bounds `kappa^2 v^4` against the acceleration budget; a
/// clothoid also owes `sigma v^3` of normal jerk with no acceleration term to
/// cancel it, so a blend seam has a speed cap the disk never sees.
fn seam_hold_ceiling(caps: &[MoveCaps], k: usize) -> f64 {
    [caps.get(k - 1), caps.get(k)]
        .into_iter()
        .flatten()
        .filter(|c| disk::curved_solver_is_available(&c.kin))
        .map(|c| curved::top_speed_ceiling(&c.kin))
        .fold(f64::INFINITY, f64::min)
}

/// Walk the members right to left, each publishing the `(v, a)` it requires at
/// its entry as a boundary condition on its predecessor. Where the requirement
/// is at or below the velocity envelope the seam inherits the whole state — the
/// brake the member's interior needs is carried into the seam instead of being
/// manufactured there, which a `kappa = 0` seam has no authority to do. Above
/// it the member has speed slack and pins nothing.
fn boundary_state_envelope(caps: &[MoveCaps], plan: &mut SeamPlan, report: &mut VelocityReport) {
    for k in 1..caps.len() {
        if !plan.is_anchor[k] {
            plan.v[k] = plan.v[k].min(seam_hold_ceiling(caps, k));
        }
    }
    for k in (1..caps.len()).rev() {
        if plan.is_anchor[k] {
            continue;
        }
        let exit = BoundaryState {
            v: plan.v[k + 1],
            a: plan.a[k + 1],
        };
        let Some(required) = required_entry_state(&caps[k].kin, exit) else {
            report.entry_requirement_unsolved += 1;
            continue;
        };
        if required.v > plan.v[k] * (1.0 + REQUIREMENT_BIND_REL) {
            continue;
        }
        plan.v[k] = required.v;
        plan.a[k] = required.a;
        if required.a != 0.0 {
            report.boundary_accel_seams += 1;
            report.worst_boundary_accel_mm_s2 =
                report.worst_boundary_accel_mm_s2.max(required.a.abs());
        }
    }
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
        v[k] = v[k].min(disk).min(jerk);
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
        v[k] = v[k].min(disk).min(jerk);
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
                fwd_s: geo.arc_from_run_start[j],
            })
            .collect();
        let run_start_line = moves[run_start].source.start_line;
        let run = disk::reconstruct_run(
            &members,
            geo.run_start_v[run_start],
            geo.run_start_a[run_start],
            tol,
        )
        .ok_or(VelocityError::Diverged {
            line_no: run_start_line,
        })?;
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
            report.unreachable_entry_states += 1;
            if report
                .worst_unreachable
                .is_none_or(|worst| named.accel_magnitude() > worst.accel_magnitude())
            {
                report.worst_unreachable = Some(named);
            }
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
