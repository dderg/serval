//! Kinematic-invariant oracle over a lowered trajectory: takes the per-axis
//! polynomial pieces the firmware would execute plus the limits the plan was
//! made under, and reports every violation. `hard` violations are invariants
//! the pipeline must never break (finite coefficients, contiguous monotone
//! time, rows no narrower than the device's step-time resolution, C0 position
//! at seams and hold gaps); `target` violations break the intended smoothness
//! budget (C1/C2 seams, velocity limits, and — only when the plan is
//! jerk-limited — accel and jerk limits, since an infinite-jerk plan bounds
//! the scalar path's acceleration, not each axis' reconstruction of it).
//! Derivative maxima are probed at endpoints plus a dense interior grid, so a
//! reported violation is a certified lower bound on the true excess.

use motion_pipeline::StreamConfig;

use crate::TrajectoryPieces;

pub const AXIS_NAMES: [&str; 4] = ["x", "y", "z", "e"];

const INTERIOR_PROBES: usize = 16;
const TIME_CONTIGUITY_TOL_S: f64 = 1e-9;
const REST_VELOCITY_TOL_MM_S: f64 = 1e-9;

/// The device's step-time resolution: the narrowest window a row can mean
/// anything over. Differentiating a narrower row divides coefficient rounding
/// by a vanishing span, manufacturing derivative magnitudes no axis executes.
const MIN_ROW_SPAN_S: f64 = 2e-9;

/// An infinite-jerk plan bounds the scalar path's acceleration, not each
/// axis' reconstruction of it, so a per-axis rail crossing is expected — but
/// only within the order of magnitude the rails live in. Past this multiple a
/// row is reporting a numerical explosion, not motion.
const ACCEL_EXPLOSION_MULTIPLIER: f64 = 1e3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ViolationKind {
    NonFiniteCoefficient,
    NonPositiveSpan,
    SliverSpan,
    TimeGap,
    TimeOverlap,
    SeamPosition,
    SeamVelocity,
    SeamAccel,
    Velocity,
    Accel,
    AccelExplosion,
    Jerk,
}

#[derive(Debug, Clone, Copy)]
pub struct Violation {
    pub kind: ViolationKind,
    pub axis: usize,
    pub t: f64,
    pub value: f64,
    pub bound: f64,
}

impl std::fmt::Display for Violation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{:?} on {} at t={:.6}s: {:.6e} exceeds {:.6e}",
            self.kind, AXIS_NAMES[self.axis], self.t, self.value, self.bound
        )
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct AxisExtrema {
    pub max_velocity: f64,
    pub max_accel: f64,
    pub max_jerk: f64,
    pub min_piece_duration_s: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct AuditBudgets {
    pub seam_pos_mm: f64,
    pub seam_vel_mm_s: f64,
    pub seam_accel_mm_s2: f64,
    pub velocity_slack_mm_s: f64,
    pub accel_slack_mm_s2: f64,
    pub jerk_multiplier: f64,
    pub kinematic_axes: [bool; 4],
}

impl AuditBudgets {
    /// Budgets a correctly-lowered trajectory is expected to satisfy: seam
    /// steps within twice the lowering's own truncation budgets
    /// (`FIT_TRUNC_VEL_MM_S`/`FIT_TRUNC_ACC_MM_S2`), accel within the
    /// configured limit plus the fit tolerance the caller granted the
    /// lowerer, jerk within twice the configured limit.
    pub fn for_config(config: &StreamConfig) -> Self {
        Self {
            seam_pos_mm: 1e-4,
            seam_vel_mm_s: 0.1,
            seam_accel_mm_s2: 0.5,
            velocity_slack_mm_s: 1e-3 * config.limits.max_velocity_mm_s,
            accel_slack_mm_s2: config.fit_tol_accel_mm_s2,
            jerk_multiplier: 2.0,
            kinematic_axes: [true; 4],
        }
    }
}

#[derive(Debug, Default)]
pub struct AuditReport {
    pub hard: Vec<Violation>,
    pub target: Vec<Violation>,
    pub extrema: [AxisExtrema; 4],
}

impl AuditReport {
    pub fn hard_ok(&self) -> bool {
        self.hard.is_empty()
    }

    pub fn target_ok(&self) -> bool {
        self.target.is_empty()
    }
}

impl std::fmt::Display for AuditReport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "audit: {} hard, {} target violations",
            self.hard.len(),
            self.target.len()
        )?;
        for v in self.hard.iter().take(20) {
            writeln!(f, "  hard:   {v}")?;
        }
        for v in self.target.iter().take(20) {
            writeln!(f, "  target: {v}")?;
        }
        for (axis, e) in self.extrema.iter().enumerate() {
            writeln!(
                f,
                "  {}: max|v|={:.3} max|a|={:.1} max|j|={:.3e} min dt={:.3e}s",
                AXIS_NAMES[axis], e.max_velocity, e.max_accel, e.max_jerk, e.min_piece_duration_s
            )?;
        }
        Ok(())
    }
}

pub fn audit_trajectory(
    traj: &TrajectoryPieces,
    config: &StreamConfig,
    budgets: &AuditBudgets,
) -> AuditReport {
    let mut report = AuditReport::default();
    let axes = [&traj.x, &traj.y, &traj.z, &traj.e];
    for (axis, pieces) in axes.iter().enumerate() {
        audit_axis(axis, pieces, config, budgets, &mut report);
    }
    report
}

fn audit_axis(
    axis: usize,
    pieces: &[Vec<f64>],
    config: &StreamConfig,
    budgets: &AuditBudgets,
    report: &mut AuditReport,
) {
    let mut extrema = AxisExtrema {
        min_piece_duration_s: f64::INFINITY,
        ..Default::default()
    };

    for piece in pieces {
        audit_piece_structure(axis, piece, report);
    }
    let structurally_sound =
        |p: &Vec<f64>| p.len() >= 3 && p.iter().all(|c| c.is_finite()) && p[1] > p[0];

    let lane_is_one_row = pieces.len() == 1;
    for piece in pieces.iter().filter(|p| structurally_sound(p)) {
        let dt = piece[1] - piece[0];
        extrema.min_piece_duration_s = extrema.min_piece_duration_s.min(dt);
        if dt < MIN_ROW_SPAN_S && !lane_is_one_row {
            report.hard.push(Violation {
                kind: ViolationKind::SliverSpan,
                axis,
                t: piece[0],
                value: dt,
                bound: MIN_ROW_SPAN_S,
            });
        }
        let (max_v, max_a, max_j) = probed_derivative_maxima(&piece[2..], dt);
        extrema.max_velocity = extrema.max_velocity.max(max_v);
        extrema.max_accel = extrema.max_accel.max(max_a);
        extrema.max_jerk = extrema.max_jerk.max(max_j);

        if budgets.kinematic_axes[axis] {
            let limits = config.limits;
            let v_bound = limits.max_velocity_mm_s + budgets.velocity_slack_mm_s;
            if max_v > v_bound {
                report.target.push(Violation {
                    kind: ViolationKind::Velocity,
                    axis,
                    t: piece[0],
                    value: max_v,
                    bound: v_bound,
                });
            }
            let explosion_bound = limits.accel_mm_s2 * ACCEL_EXPLOSION_MULTIPLIER;
            if max_a > explosion_bound {
                report.hard.push(Violation {
                    kind: ViolationKind::AccelExplosion,
                    axis,
                    t: piece[0],
                    value: max_a,
                    bound: explosion_bound,
                });
            }
            if limits.max_jerk_mm_s3.is_finite() {
                let a_bound = limits.accel_mm_s2 + budgets.accel_slack_mm_s2;
                if max_a > a_bound {
                    report.target.push(Violation {
                        kind: ViolationKind::Accel,
                        axis,
                        t: piece[0],
                        value: max_a,
                        bound: a_bound,
                    });
                }
            }
            let j_bound = limits.max_jerk_mm_s3 * budgets.jerk_multiplier;
            if j_bound.is_finite() && max_j > j_bound {
                report.target.push(Violation {
                    kind: ViolationKind::Jerk,
                    axis,
                    t: piece[0],
                    value: max_j,
                    bound: j_bound,
                });
            }
        }
    }

    for w in pieces.windows(2) {
        let (left, right) = (&w[0], &w[1]);
        if !structurally_sound(left) || !structurally_sound(right) {
            continue;
        }
        audit_seam(axis, left, right, config, budgets, report);
    }

    if pieces.is_empty() {
        extrema.min_piece_duration_s = 0.0;
    }
    report.extrema[axis] = extrema;
}

fn audit_piece_structure(axis: usize, piece: &[f64], report: &mut AuditReport) {
    if piece.len() < 3 || piece.iter().any(|c| !c.is_finite()) {
        report.hard.push(Violation {
            kind: ViolationKind::NonFiniteCoefficient,
            axis,
            t: piece.first().copied().unwrap_or(f64::NAN),
            value: f64::NAN,
            bound: f64::NAN,
        });
        return;
    }
    if piece[1] <= piece[0] {
        report.hard.push(Violation {
            kind: ViolationKind::NonPositiveSpan,
            axis,
            t: piece[0],
            value: piece[1] - piece[0],
            bound: 0.0,
        });
    }
}

fn audit_seam(
    axis: usize,
    left: &[f64],
    right: &[f64],
    config: &StreamConfig,
    budgets: &AuditBudgets,
    report: &mut AuditReport,
) {
    let gap = right[0] - left[1];
    let (lp, lv, la) = state_at(&left[2..], left[1] - left[0]);
    let (rp, rv, ra) = state_at(&right[2..], 0.0);
    if gap < -TIME_CONTIGUITY_TOL_S {
        report.hard.push(Violation {
            kind: ViolationKind::TimeOverlap,
            axis,
            t: right[0],
            value: -gap,
            bound: TIME_CONTIGUITY_TOL_S,
        });
        return;
    }
    let gap_is_a_rest_hold =
        lv.abs() <= REST_VELOCITY_TOL_MM_S && rv.abs() <= REST_VELOCITY_TOL_MM_S;
    if gap > TIME_CONTIGUITY_TOL_S && !gap_is_a_rest_hold {
        report.hard.push(Violation {
            kind: ViolationKind::TimeGap,
            axis,
            t: left[1],
            value: gap,
            bound: TIME_CONTIGUITY_TOL_S,
        });
        return;
    }

    let dp = (rp - lp).abs();
    if dp > budgets.seam_pos_mm {
        report.hard.push(Violation {
            kind: ViolationKind::SeamPosition,
            axis,
            t: right[0],
            value: dp,
            bound: budgets.seam_pos_mm,
        });
    }
    let dv = (rv - lv).abs();
    if dv > budgets.seam_vel_mm_s {
        report.target.push(Violation {
            kind: ViolationKind::SeamVelocity,
            axis,
            t: right[0],
            value: dv,
            bound: budgets.seam_vel_mm_s,
        });
    }
    let da = (ra - la).abs();
    if config.limits.max_jerk_mm_s3.is_finite() && da > budgets.seam_accel_mm_s2 {
        report.target.push(Violation {
            kind: ViolationKind::SeamAccel,
            axis,
            t: right[0],
            value: da,
            bound: budgets.seam_accel_mm_s2,
        });
    }
}

fn differentiate(coeffs: &[f64]) -> Vec<f64> {
    coeffs
        .iter()
        .enumerate()
        .skip(1)
        .map(|(k, c)| k as f64 * c)
        .collect()
}

fn eval(coeffs: &[f64], tau: f64) -> f64 {
    coeffs.iter().rev().fold(0.0, |acc, c| acc * tau + c)
}

fn state_at(coeffs: &[f64], tau: f64) -> (f64, f64, f64) {
    let vel = differentiate(coeffs);
    let acc = differentiate(&vel);
    (eval(coeffs, tau), eval(&vel, tau), eval(&acc, tau))
}

fn probed_derivative_maxima(coeffs: &[f64], dt: f64) -> (f64, f64, f64) {
    let vel = differentiate(coeffs);
    let acc = differentiate(&vel);
    let jerk = differentiate(&acc);
    let mut max = [0.0_f64; 3];
    for k in 0..=INTERIOR_PROBES {
        let tau = dt * k as f64 / INTERIOR_PROBES as f64;
        for (m, c) in max.iter_mut().zip([&vel, &acc, &jerk]) {
            *m = m.max(eval(c, tau).abs());
        }
    }
    (max[0], max[1], max[2])
}

#[cfg(test)]
mod tests;
