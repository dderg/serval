use std::collections::HashSet;

use crate::fitter::{FitOutcome, UnblendReason};
use crate::path::{CurvatureProfile, Segment};
use crate::segment::SourceRange;

mod scurve;

const LENGTH_EPS_MM: f64 = 1e-9;
const VELOCITY_EPS_MM_S: f64 = 1e-9;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VelocityConfig {
    pub consistency_tol: f64,
    pub max_jerk_mm_s3: f64,
}

impl Default for VelocityConfig {
    fn default() -> Self {
        Self {
            consistency_tol: 1e-6,
            // TODO: jerk-limit floor is an open tuning question (spec-motion-6).
            max_jerk_mm_s3: 100_000.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MoveVelocity {
    pub start_v: f64,
    pub cruise_v: f64,
    pub end_v: f64,
    pub ceiling: f64,
    pub accel: f64,
    pub jerk: f64,
    pub length: f64,
    pub source: SourceRange,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct VelocityReport {
    pub stops: u32,
    pub curvature_bound: u32,
    pub feedrate_bound: u32,
    pub jerk_bound: u32,
}

#[derive(Debug, Clone, PartialEq)]
pub struct VelocityProfile {
    pub moves: Vec<MoveVelocity>,
    pub report: VelocityReport,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VelocityError {
    Inconsistent { line_no: u32 },
    NonAlphabet { line_no: u32 },
    NonFinite { line_no: u32 },
    InvalidConfig,
}

struct MoveCaps {
    length: f64,
    accel: f64,
    ceiling: f64,
}

pub fn plan_velocity(
    outcome: &FitOutcome,
    config: VelocityConfig,
) -> Result<VelocityProfile, VelocityError> {
    let jerk = config.max_jerk_mm_s3;
    if jerk.is_nan() || jerk <= 0.0 {
        return Err(VelocityError::InvalidConfig);
    }

    let moves = &outcome.moves;
    let n = moves.len();
    if n == 0 {
        return Ok(VelocityProfile {
            moves: Vec::new(),
            report: VelocityReport::default(),
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
        let accel = m.limits.accel_mm_s2;
        let (length, curvature_cap) = match &m.segment.spatial {
            Some(seg) => {
                let length = seg.s_len();
                validate_segment(seg, length, line_no, config.consistency_tol)?;
                let (_, kappa_peak) = seg.kappa_peak();
                let cap = if kappa_peak > 0.0 {
                    (accel / kappa_peak).sqrt()
                } else {
                    f64::INFINITY
                };
                (length, cap)
            }
            None => {
                let length = m
                    .segment
                    .virtual_path_mm
                    .ok_or(VelocityError::NonFinite { line_no })?;
                if !(length.is_finite() && length > LENGTH_EPS_MM) {
                    return Err(VelocityError::NonFinite { line_no });
                }
                (length, f64::INFINITY)
            }
        };

        let feed = m.feedrate_mm_s.min(m.limits.max_velocity_mm_s);
        let ceiling = feed.min(curvature_cap);
        if curvature_cap < feed {
            report.curvature_bound += 1;
        } else {
            report.feedrate_bound += 1;
        }
        caps.push(MoveCaps {
            length,
            accel,
            ceiling,
        });
    }

    let mut v = vec![0.0_f64; n + 1];
    for k in 1..n {
        let downstream = &moves[k];
        let blend_half = matches!(downstream.segment.spatial, Some(Segment::Clothoid(_)));
        if stop_lines.contains(&downstream.source.start_line) && !blend_half {
            report.stops += 1;
        } else {
            v[k] = caps[k - 1].ceiling.min(caps[k].ceiling);
        }
    }

    for k in 1..=n {
        let reachable =
            scurve::max_reachable_velocity(v[k - 1], caps[k - 1].length, caps[k - 1].accel, jerk);
        v[k] = v[k].min(reachable);
    }
    for k in (0..n).rev() {
        let reachable =
            scurve::max_reachable_velocity(v[k + 1], caps[k].length, caps[k].accel, jerk);
        v[k] = v[k].min(reachable);
    }

    let mut out = Vec::with_capacity(n);
    for (j, m) in moves.iter().enumerate() {
        let start_v = v[j];
        let end_v = v[j + 1];
        let accel_apex = caps[j].ceiling.min(
            (0.5 * (start_v * start_v + end_v * end_v) + caps[j].accel * caps[j].length).sqrt(),
        );
        let cruise_v = scurve::peak_velocity(
            start_v,
            end_v,
            caps[j].length,
            caps[j].accel,
            jerk,
            caps[j].ceiling,
        );
        if cruise_v + VELOCITY_EPS_MM_S < accel_apex {
            report.jerk_bound += 1;
        }
        out.push(MoveVelocity {
            start_v,
            cruise_v,
            end_v,
            ceiling: caps[j].ceiling,
            accel: caps[j].accel,
            jerk,
            length: caps[j].length,
            source: m.source,
        });
    }

    Ok(VelocityProfile { moves: out, report })
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
