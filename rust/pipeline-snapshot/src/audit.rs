//! Kinematic-invariant oracle over an exact trajectory: takes the carriers
//! the firmware would execute plus the limits the plan was made under, and
//! reports every violation. `hard` violations are invariants the pipeline
//! must never break (finite states, contiguous monotone time, rows no
//! narrower than the device's step-time resolution, evaluable carriers, C0
//! position at seams and hold gaps); `target` violations break the intended
//! smoothness budget (C1/C2 seams, velocity limits, and — only when the plan
//! is jerk-limited — accel and jerk limits, since an infinite-jerk plan
//! bounds the scalar path's acceleration, not each axis' reconstruction of
//! it). Every state is the carrier's own exact position/velocity/
//! acceleration/jerk, read one-sided at each discontinuity; derivative
//! maxima are probed at every breakpoint plus a dense interior grid, so a
//! reported violation is a certified lower bound on the true excess.

use motion_pipeline::StreamConfig;
use trajectory::continuous::Pvaj;

use crate::{CarrierRow, ExactTrajectory, SampleSide};

pub const AXIS_NAMES: [&str; 4] = ["x", "y", "z", "e"];

const INTERIOR_PROBES: usize = 16;
const TIME_CONTIGUITY_TOL_S: f64 = 1e-9;
const REST_VELOCITY_TOL_MM_S: f64 = 1e-9;

/// The device's step-time resolution: the narrowest window a row can mean
/// anything over.
const MIN_ROW_SPAN_S: f64 = 2e-9;

/// An infinite-jerk plan bounds the scalar path's acceleration, not each
/// axis' reconstruction of it, so a per-axis rail crossing is expected — but
/// only within the order of magnitude the rails live in. Past this multiple a
/// row is reporting a numerical explosion, not motion.
const ACCEL_EXPLOSION_MULTIPLIER: f64 = 1e3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ViolationKind {
    NonFiniteState,
    CarrierNotEvaluable,
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
    traj: &ExactTrajectory,
    config: &StreamConfig,
    budgets: &AuditBudgets,
) -> AuditReport {
    let mut report = AuditReport::default();
    for axis in 0..4 {
        audit_axis(traj, axis, config, budgets, &mut report);
    }
    report
}

fn audit_axis(
    traj: &ExactTrajectory,
    axis: usize,
    config: &StreamConfig,
    budgets: &AuditBudgets,
    report: &mut AuditReport,
) {
    let rows = traj.rows(axis);
    let mut extrema = AxisExtrema {
        min_piece_duration_s: if rows.is_empty() { 0.0 } else { f64::INFINITY },
        ..Default::default()
    };
    let lane_is_one_row = rows.len() == 1;

    for (index, row) in rows.iter().enumerate() {
        if !audit_row_structure(axis, row, report) {
            continue;
        }
        let dt = row.t1 - row.t0;
        extrema.min_piece_duration_s = extrema.min_piece_duration_s.min(dt);
        if dt < MIN_ROW_SPAN_S && !lane_is_one_row {
            report.hard.push(Violation {
                kind: ViolationKind::SliverSpan,
                axis,
                t: row.t0,
                value: dt,
                bound: MIN_ROW_SPAN_S,
            });
        }
        let Some((max_v, max_a, max_j)) = probed_maxima(traj, axis, index, report) else {
            continue;
        };
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
                    t: row.t0,
                    value: max_v,
                    bound: v_bound,
                });
            }
            let explosion_bound = limits.accel_mm_s2 * ACCEL_EXPLOSION_MULTIPLIER;
            if max_a > explosion_bound {
                report.hard.push(Violation {
                    kind: ViolationKind::AccelExplosion,
                    axis,
                    t: row.t0,
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
                        t: row.t0,
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
                    t: row.t0,
                    value: max_j,
                    bound: j_bound,
                });
            }
        }
    }

    for (index, row) in rows.iter().enumerate() {
        if !row_is_sound(row) {
            continue;
        }
        for t in traj.row_breakpoints(axis, index) {
            if t > row.t0 && t < row.t1 {
                audit_discontinuity(traj, axis, (index, t), (index, t), config, budgets, report);
            }
        }
        if rows.get(index + 1).is_some_and(row_is_sound) {
            audit_seam(traj, axis, index, config, budgets, report);
        }
    }

    report.extrema[axis] = extrema;
}

fn row_is_sound(row: &CarrierRow) -> bool {
    row.t0.is_finite() && row.t1.is_finite() && row.t1 > row.t0
}

fn audit_row_structure(axis: usize, row: &CarrierRow, report: &mut AuditReport) -> bool {
    if !(row.t0.is_finite() && row.t1.is_finite()) {
        report.hard.push(Violation {
            kind: ViolationKind::NonFiniteState,
            axis,
            t: row.t0,
            value: f64::NAN,
            bound: f64::NAN,
        });
        return false;
    }
    if row.t1 <= row.t0 {
        report.hard.push(Violation {
            kind: ViolationKind::NonPositiveSpan,
            axis,
            t: row.t0,
            value: row.t1 - row.t0,
            bound: 0.0,
        });
        return false;
    }
    true
}

/// Certified lower bounds on the row's own derivative maxima: the carrier is
/// evaluated on a dense grid of every interval between its breakpoints, from
/// the side that owns each station, so a jerk step at a phase joint is read
/// as the two values it really takes.
fn probed_maxima(
    traj: &ExactTrajectory,
    axis: usize,
    row: usize,
    report: &mut AuditReport,
) -> Option<(f64, f64, f64)> {
    let mut max = [0.0_f64; 3];
    for interval in traj.row_breakpoints(axis, row).windows(2) {
        let (t0, t1) = (interval[0], interval[1]);
        if t1 <= t0 {
            continue;
        }
        for step in 0..=INTERIOR_PROBES {
            let t = t0 + (t1 - t0) * step as f64 / INTERIOR_PROBES as f64;
            let side = if step == INTERIOR_PROBES {
                SampleSide::Left
            } else {
                SampleSide::Right
            };
            let state = probe(traj, axis, row, t, side, report)?;
            for (slot, value) in
                max.iter_mut()
                    .zip([state.velocity, state.acceleration, state.jerk])
            {
                *slot = slot.max(value.abs());
            }
        }
    }
    Some((max[0], max[1], max[2]))
}

fn probe(
    traj: &ExactTrajectory,
    axis: usize,
    row: usize,
    t: f64,
    side: SampleSide,
    report: &mut AuditReport,
) -> Option<Pvaj> {
    match traj.eval_row(axis, row, t, side) {
        Ok(state) => {
            if [
                state.position,
                state.velocity,
                state.acceleration,
                state.jerk,
            ]
            .into_iter()
            .all(f64::is_finite)
            {
                return Some(state);
            }
            report.hard.push(Violation {
                kind: ViolationKind::NonFiniteState,
                axis,
                t,
                value: f64::NAN,
                bound: f64::NAN,
            });
            None
        }
        Err(_) => {
            report.hard.push(Violation {
                kind: ViolationKind::CarrierNotEvaluable,
                axis,
                t,
                value: f64::NAN,
                bound: f64::NAN,
            });
            None
        }
    }
}

fn audit_seam(
    traj: &ExactTrajectory,
    axis: usize,
    left_row: usize,
    config: &StreamConfig,
    budgets: &AuditBudgets,
    report: &mut AuditReport,
) {
    let rows = traj.rows(axis);
    let (left, right) = (&rows[left_row], &rows[left_row + 1]);
    let gap = right.t0 - left.t1;
    if gap < -TIME_CONTIGUITY_TOL_S {
        report.hard.push(Violation {
            kind: ViolationKind::TimeOverlap,
            axis,
            t: right.t0,
            value: -gap,
            bound: TIME_CONTIGUITY_TOL_S,
        });
        return;
    }
    let Some(before) = probe(traj, axis, left_row, left.t1, SampleSide::Left, report) else {
        return;
    };
    let Some(after) = probe(
        traj,
        axis,
        left_row + 1,
        right.t0,
        SampleSide::Right,
        report,
    ) else {
        return;
    };
    let gap_is_a_rest_hold = before.velocity.abs() <= REST_VELOCITY_TOL_MM_S
        && after.velocity.abs() <= REST_VELOCITY_TOL_MM_S;
    if gap > TIME_CONTIGUITY_TOL_S && !gap_is_a_rest_hold {
        report.hard.push(Violation {
            kind: ViolationKind::TimeGap,
            axis,
            t: left.t1,
            value: gap,
            bound: TIME_CONTIGUITY_TOL_S,
        });
        return;
    }
    report_state_jump(axis, right.t0, before, after, config, budgets, report);
}

/// A phase joint, a knot or a profile boundary inside one row: both sides are
/// the same carrier's one-sided limits, so a step there is a real
/// discontinuity the axis executes.
fn audit_discontinuity(
    traj: &ExactTrajectory,
    axis: usize,
    left: (usize, f64),
    right: (usize, f64),
    config: &StreamConfig,
    budgets: &AuditBudgets,
    report: &mut AuditReport,
) {
    let Some(before) = probe(traj, axis, left.0, left.1, SampleSide::Left, report) else {
        return;
    };
    let Some(after) = probe(traj, axis, right.0, right.1, SampleSide::Right, report) else {
        return;
    };
    report_state_jump(axis, right.1, before, after, config, budgets, report);
}

fn report_state_jump(
    axis: usize,
    t: f64,
    before: Pvaj,
    after: Pvaj,
    config: &StreamConfig,
    budgets: &AuditBudgets,
    report: &mut AuditReport,
) {
    let dp = (after.position - before.position).abs();
    if dp > budgets.seam_pos_mm {
        report.hard.push(Violation {
            kind: ViolationKind::SeamPosition,
            axis,
            t,
            value: dp,
            bound: budgets.seam_pos_mm,
        });
    }
    let dv = (after.velocity - before.velocity).abs();
    if dv > budgets.seam_vel_mm_s {
        report.target.push(Violation {
            kind: ViolationKind::SeamVelocity,
            axis,
            t,
            value: dv,
            bound: budgets.seam_vel_mm_s,
        });
    }
    let da = (after.acceleration - before.acceleration).abs();
    if config.limits.max_jerk_mm_s3.is_finite() && da > budgets.seam_accel_mm_s2 {
        report.target.push(Violation {
            kind: ViolationKind::SeamAccel,
            axis,
            t,
            value: da,
            bound: budgets.seam_accel_mm_s2,
        });
    }
}

#[cfg(test)]
mod tests;
