#[cfg(test)]
use std::collections::HashSet;

use crate::LENGTH_EPS_MM;
#[cfg(test)]
use crate::fitter::{FitOutcome, UnblendReason};
use crate::path::CurvatureProfile;
#[cfg(test)]
use crate::path::Segment;
use crate::segment::SourceRange;

mod disk;
pub mod law;
mod reconstruct;

pub use law::{LawSegment, ScalarLaw};

use disk::Kinematics;

const VELOCITY_EPS_MM_S: f64 = 1e-9;
const MIN_INTEGRATION_TOL: f64 = 1e-9;
const NEGATIVE_VELOCITY_TOL_MM_S: f64 = 1e-6;
const CONSISTENCY_TOL: f64 = 1e-6;
const RECONSTRUCT_WORKERS_MAX: usize = 4;

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
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct VelocityReport {
    pub stops: u32,
    pub curvature_bound: u32,
    pub feedrate_bound: u32,
    pub limit_ride: u32,
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
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VelocityPlanParams {
    pub integration_tol: f64,
    pub max_extrude_only_velocity_mm_s: f64,
    pub max_extrude_only_accel_mm_s2: f64,
    pub entry_v: f64,
}

#[cfg(test)]
pub(crate) fn warm_start_params(integration_tol: f64) -> VelocityPlanParams {
    VelocityPlanParams {
        integration_tol,
        max_extrude_only_velocity_mm_s: f64::INFINITY,
        max_extrude_only_accel_mm_s2: f64::INFINITY,
        entry_v: 0.0,
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
/// `params.entry_v`.
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
    plan_velocity_stops_select_prefix(moves, stop_before, params, |_| reconstruct_count)
}

pub fn plan_velocity_stops_select_prefix<F>(
    moves: &[crate::Move],
    stop_before: &[bool],
    params: VelocityPlanParams,
    select_prefix: F,
) -> Result<VelocityProfile, VelocityError>
where
    F: FnOnce(usize) -> usize,
{
    let tol = params.integration_tol;
    let entry_v = params.entry_v;
    validate_config(params)?;

    let n = moves.len();
    assert_eq!(stop_before.len(), n, "one stop flag per move");
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
    let caps = build_move_caps(moves, params, &mut report)?;
    check_entry_ceiling(moves, &caps, entry_v, tol)?;
    let mut plan = seed_seam_velocities(&caps, stop_before, entry_v, &mut report);
    forward_pass(moves, &caps, &mut plan.v, tol)?;
    let (barrier, v_barrier) = reverse_brake_envelope(moves, &caps, &mut plan.v, tol)?;
    check_entry_brake(moves, &caps, &plan.v, entry_v, tol)?;
    let reconstruct_count = select_prefix(barrier);
    assert!(
        reconstruct_count <= n,
        "cannot reconstruct {reconstruct_count} moves from a {n}-move plan"
    );
    let (out, boundaries) = reconstruct_runs(
        &moves[..reconstruct_count],
        &caps[..reconstruct_count],
        &plan,
        entry_v,
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
    params: VelocityPlanParams,
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
                accel = accel.min(params.max_extrude_only_accel_mm_s2);
                extrude_only_velocity_cap = params.max_extrude_only_velocity_mm_s;
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

struct SeamPlan {
    v: Vec<f64>,
    is_anchor: Vec<bool>,
}

fn seed_seam_velocities(
    caps: &[MoveCaps],
    stop_before: &[bool],
    entry_v: f64,
    report: &mut VelocityReport,
) -> SeamPlan {
    let n = caps.len();
    let mut v = vec![0.0_f64; n + 1];
    v[0] = entry_v;
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
    SeamPlan { v, is_anchor }
}

fn forward_pass(
    moves: &[crate::Move],
    caps: &[MoveCaps],
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
        if moves[j].limits.max_jerk_mm_s3 != f64::INFINITY {
            return Err(VelocityError::Diverged { line_no });
        }
        v[k] = v[k].min(disk);
    }
    Ok(())
}

fn reverse_brake_envelope(
    moves: &[crate::Move],
    caps: &[MoveCaps],
    v: &mut [f64],
    tol: f64,
) -> Result<(usize, f64), VelocityError> {
    let n = caps.len();
    let mut barrier = 0usize;
    for k in (1..n).rev() {
        let j = k;
        let line_no = moves[j].source.start_line;
        let kin = &caps[j].kin;
        let disk = disk::disk_reach_v_rev(kin, v[k + 1], kin.length, tol)
            .ok_or(VelocityError::Diverged { line_no })?;
        let forward_ceiling = v[k];
        v[k] = v[k].min(disk);
        if barrier == 0 && !(v[k] < forward_ceiling) {
            barrier = k;
        }
    }
    let v_barrier = v[barrier];
    Ok((barrier, v_barrier))
}

fn check_entry_brake(
    moves: &[crate::Move],
    caps: &[MoveCaps],
    v: &[f64],
    entry_v: f64,
    tol: f64,
) -> Result<(), VelocityError> {
    let entry_line_no = moves[0].source.start_line;
    let kin = &caps[0].kin;
    let entry_brake =
        disk::disk_reach_v_rev(kin, v[1], kin.length, tol).ok_or(VelocityError::Diverged {
            line_no: entry_line_no,
        })?;
    if entry_v > entry_brake + tol * (1.0 + entry_brake) {
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
    entry_v: f64,
    report: &mut VelocityReport,
) -> Result<(Vec<MoveVelocity>, Vec<f64>), VelocityError> {
    let n = caps.len();
    let v = &plan.v;
    let is_anchor = &plan.is_anchor;
    let mut out: Vec<MoveVelocity> = Vec::with_capacity(n);
    let mut boundaries: Vec<f64> = Vec::with_capacity(n + 1);
    boundaries.push(entry_v);
    let mut run_start = 0;
    while run_start < n {
        let mut run_end = run_start + 1;
        while run_end < n && !is_anchor[run_end] {
            run_end += 1;
        }
        let run_members: Vec<disk::RunMember> = (run_start..run_end)
            .map(|j| disk::RunMember {
                kin: &caps[j].kin,
                exit_v: v[j + 1],
            })
            .collect();
        let run_start_line = moves[run_start].source.start_line;
        let workers = if cfg!(not(target_arch = "wasm32")) {
            std::thread::available_parallelism()
                .map_or(1, |cores| cores.get())
                .min(RECONSTRUCT_WORKERS_MAX)
                .min(run_members.len())
        } else {
            1
        };
        let mut indexed_profiles = if workers > 1 {
            let next_member = std::sync::atomic::AtomicUsize::new(0);
            std::thread::scope(|scope| {
                let next_member = &next_member;
                let handles = (0..workers)
                    .map(|_| {
                        scope.spawn(|| {
                            let mut profiles = Vec::new();
                            loop {
                                let idx =
                                    next_member.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                                let Some(member) = run_members.get(idx) else {
                                    return profiles;
                                };
                                profiles.push((
                                    idx,
                                    reconstruct::member_profile(
                                        idx,
                                        member,
                                        v[run_start + idx],
                                        member.exit_v,
                                    ),
                                ));
                            }
                        })
                    })
                    .collect::<Vec<_>>();
                handles
                    .into_iter()
                    .flat_map(|handle| {
                        handle
                            .join()
                            .expect("velocity reconstruction thread panicked")
                    })
                    .collect::<Vec<_>>()
            })
        } else {
            run_members
                .iter()
                .enumerate()
                .map(|(idx, member)| {
                    (
                        idx,
                        reconstruct::member_profile(idx, member, v[run_start + idx], member.exit_v),
                    )
                })
                .collect()
        };
        indexed_profiles.sort_by_key(|(idx, _)| *idx);
        let reconstructed_phases = indexed_profiles
            .into_iter()
            .map(|(_, result)| {
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
                    last.1 = run_members[idx].exit_v;
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
            let phases = reconstructed_phases[idx].clone();
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
                let (_, boundary_v, _) = phases
                    .last()
                    .expect("a member profile always carries at least one segment")
                    .end_state();
                boundary_v.min(v[j + 1])
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
            });
        }
        run_start = run_end;
    }

    Ok((out, boundaries))
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
mod tests;
