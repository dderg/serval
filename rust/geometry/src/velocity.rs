use std::collections::HashSet;

use crate::fitter::{FitOutcome, UnblendReason};
use crate::path::{CurvatureProfile, Segment};
use crate::segment::SourceRange;

mod disk;
mod scurve;

use disk::Kinematics;

const LENGTH_EPS_MM: f64 = 1e-9;
const VELOCITY_EPS_MM_S: f64 = 1e-9;
const MIN_INTEGRATION_TOL: f64 = 1e-9;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VelocityConfig {
    pub consistency_tol: f64,
    pub max_jerk_mm_s3: f64,
    pub integration_tol: f64,
    pub max_extrude_only_velocity_mm_s: f64,
    pub max_extrude_only_accel_mm_s2: f64,
}

impl Default for VelocityConfig {
    fn default() -> Self {
        Self {
            consistency_tol: 1e-6,
            max_jerk_mm_s3: 100_000.0, // TODO: jerk-limit floor is an open tuning question (spec-motion-6)
            integration_tol: 1e-7,
            max_extrude_only_velocity_mm_s: f64::INFINITY,
            max_extrude_only_accel_mm_s2: f64::INFINITY,
        }
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
    pub accel: f64,
    pub jerk: f64,
    pub length: f64,
    pub source: SourceRange,
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct VelocityReport {
    pub stops: u32,
    pub curvature_bound: u32,
    pub feedrate_bound: u32,
    pub jerk_bound: u32,
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
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VelocityError {
    Inconsistent { line_no: u32 },
    NonAlphabet { line_no: u32 },
    NonFinite { line_no: u32 },
    Diverged { line_no: u32 },
    OverCommitted { line_no: u32 },
    RestAnchorAccel { line_no: u32 },
    InvalidConfig,
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

pub fn plan_velocity(
    outcome: &FitOutcome,
    config: VelocityConfig,
) -> Result<VelocityProfile, VelocityError> {
    plan_velocity_warm_start(outcome, config, 0.0)
}

pub fn plan_velocity_warm_start(
    outcome: &FitOutcome,
    config: VelocityConfig,
    entry_v: f64,
) -> Result<VelocityProfile, VelocityError> {
    let jerk = config.max_jerk_mm_s3;
    if jerk.is_nan() || jerk <= 0.0 {
        return Err(VelocityError::InvalidConfig);
    }
    let tol = config.integration_tol;
    if !(tol.is_finite() && tol >= MIN_INTEGRATION_TOL) {
        return Err(VelocityError::InvalidConfig);
    }
    if !(entry_v.is_finite() && entry_v >= 0.0) {
        return Err(VelocityError::InvalidConfig);
    }
    if !(config.max_extrude_only_velocity_mm_s > 0.0 && config.max_extrude_only_accel_mm_s2 > 0.0) {
        return Err(VelocityError::InvalidConfig);
    }

    let moves = &outcome.moves;
    let n = moves.len();
    if n == 0 {
        return Ok(VelocityProfile {
            moves: Vec::new(),
            report: VelocityReport::default(),
            barrier: 0,
            v_barrier: 0.0,
        });
    }

    let stop_lines: HashSet<u32> = outcome
        .report
        .unblended
        .iter()
        .filter(|u| u.reason != UnblendReason::Collinear)
        .map(|u| u.line_no)
        .collect();

    let mut report = VelocityReport::default();
    let mut caps = Vec::with_capacity(n);
    for m in moves {
        let line_no = m.source.start_line;
        let mut accel = m.limits.accel_mm_s2;
        let mut extrude_only_velocity_cap = f64::INFINITY;
        let (length, kappa0, sigma, kappa_peak) = match &m.segment.spatial {
            Some(seg) => {
                let length = seg.s_len();
                validate_segment(seg, length, line_no, config.consistency_tol)?;
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
                accel = accel.min(config.max_extrude_only_accel_mm_s2);
                extrude_only_velocity_cap = config.max_extrude_only_velocity_mm_s;
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
                jerk,
                kappa0,
                sigma,
                flat_ceiling,
            },
            kappa_peak,
        });
    }

    let entry_ceiling = {
        let kin0 = &caps[0].kin;
        kin0.flat_ceiling
            .min(disk::limit_speed(kin0.kappa0.abs(), kin0.accel))
    };
    if entry_v > entry_ceiling + VELOCITY_EPS_MM_S {
        return Err(VelocityError::OverCommitted {
            line_no: moves[0].source.start_line,
        });
    }

    let mut v = vec![0.0_f64; n + 1];
    v[0] = entry_v;
    let mut is_anchor = vec![false; n + 1];
    is_anchor[0] = true;
    is_anchor[n] = true;
    for k in 1..n {
        let downstream = &moves[k];
        let blend_half = matches!(downstream.segment.spatial, Some(Segment::Clothoid(_)));
        if stop_lines.contains(&downstream.source.start_line) && !blend_half {
            report.stops += 1;
            is_anchor[k] = true;
        } else {
            let up = &caps[k - 1].kin;
            let dn = &caps[k].kin;
            let kappa_up = (up.kappa0 + up.sigma * up.length).abs();
            let kappa_dn = dn.kappa0.abs();
            let boundary_vlim =
                disk::limit_speed(kappa_up, up.accel).min(disk::limit_speed(kappa_dn, dn.accel));
            v[k] = up.flat_ceiling.min(dn.flat_ceiling).min(boundary_vlim);
        }
    }

    let mut run_start_v = vec![0.0_f64; n];
    let mut arc_from_run_start = vec![0.0_f64; n];
    {
        let mut anchor_v = v[0];
        let mut cum = 0.0;
        for j in 0..n {
            if is_anchor[j] {
                anchor_v = v[j];
                cum = 0.0;
            }
            run_start_v[j] = anchor_v;
            arc_from_run_start[j] = cum;
            cum += caps[j].kin.length;
        }
    }
    let mut arc_to_run_end = vec![0.0_f64; n];
    {
        let mut cum = 0.0;
        for j in (0..n).rev() {
            if is_anchor[j + 1] {
                cum = 0.0;
            }
            arc_to_run_end[j] = cum;
            cum += caps[j].kin.length;
        }
    }

    for k in 1..=n {
        let j = k - 1;
        let line_no = moves[j].source.start_line;
        let kin = &caps[j].kin;
        let disk = disk::disk_reach_v(kin, v[j], kin.length, tol)
            .ok_or(VelocityError::Diverged { line_no })?;
        let jerk = scurve::reach_v(
            run_start_v[j],
            arc_from_run_start[j] + kin.length,
            kin.accel,
            kin.jerk,
        )
        .ok_or(VelocityError::Diverged { line_no })?;
        v[k] = v[k].min(disk).min(jerk);
    }
    let v_forward_ceiling = v.clone();
    for k in (1..n).rev() {
        let j = k;
        let line_no = moves[j].source.start_line;
        let kin = &caps[j].kin;
        let disk = disk::disk_reach_v_rev(kin, v[k + 1], kin.length, tol)
            .ok_or(VelocityError::Diverged { line_no })?;
        let jerk = scurve::reach_v(0.0, arc_to_run_end[j] + kin.length, kin.accel, kin.jerk)
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
    let entry_line_no = moves[0].source.start_line;
    let entry_brake = {
        let kin = &caps[0].kin;
        let disk =
            disk::disk_reach_v_rev(kin, v[1], kin.length, tol).ok_or(VelocityError::Diverged {
                line_no: entry_line_no,
            })?;
        let jerk = scurve::reach_v(0.0, arc_to_run_end[0] + kin.length, kin.accel, kin.jerk)
            .ok_or(VelocityError::Diverged {
                line_no: entry_line_no,
            })?;
        disk.min(jerk)
    };
    if entry_v > entry_brake + VELOCITY_EPS_MM_S {
        return Err(VelocityError::OverCommitted {
            line_no: entry_line_no,
        });
    }

    let mut out: Vec<MoveVelocity> = Vec::with_capacity(n);
    let mut run_start = 0;
    while run_start < n {
        let mut run_end = run_start + 1;
        while run_end < n && !is_anchor[run_end] {
            run_end += 1;
        }
        let members: Vec<disk::RunMember> = (run_start..run_end)
            .map(|j| disk::RunMember {
                kin: &caps[j].kin,
                entry_v: v[j],
                exit_v: v[j + 1],
                fwd_s: arc_from_run_start[j],
                bwd_s: arc_to_run_end[j],
            })
            .collect();
        let run_start_line = moves[run_start].source.start_line;
        let reconstructed = disk::reconstruct_run(&members, run_start_v[run_start], tol).ok_or(
            VelocityError::Diverged {
                line_no: run_start_line,
            },
        )?;

        for (idx, j) in (run_start..run_end).enumerate() {
            let kin = &caps[j].kin;
            let m = &moves[j];
            let line_no = m.source.start_line;
            let entry_v = v[j];
            let exit_v = v[j + 1];
            let mut samples: Vec<VelSample> = reconstructed[idx]
                .iter()
                .map(|&(s, v, a)| VelSample { s, v, a })
                .collect();
            if is_anchor[j] && entry_v <= VELOCITY_EPS_MM_S {
                pin_rest_anchor(samples.first_mut(), line_no, kin.jerk)?;
            }
            if is_anchor[j + 1] && exit_v <= VELOCITY_EPS_MM_S {
                pin_rest_anchor(samples.last_mut(), line_no, kin.jerk)?;
            }
            let peak_v = samples.iter().fold(0.0_f64, |acc, p| acc.max(p.v));
            report.traversal_time_s += traversal_time(&samples);

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

            out.push(MoveVelocity {
                entry_v,
                exit_v,
                peak_v,
                samples,
                accel: kin.accel,
                jerk: kin.jerk,
                length: kin.length,
                source: m.source,
            });
        }
        run_start = run_end;
    }

    Ok(VelocityProfile {
        moves: out,
        report,
        barrier,
        v_barrier,
    })
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
mod tests;
