//! Executed-trajectory limit utilization: how close to the kinematic limits the
//! committed per-axis time-polynomials actually ride, measured the way the MCU
//! steps them (finite differences at the 40 kHz sample rate).
//!
//! Unlike a grid-sampled solver diagnostic, this evaluates the exact committed
//! trajectory — the `FittedSegment`/emitted axes are polynomials in time, so
//! their velocity/accel/jerk are exact for any geometry and include the behavior
//! between solver grid points. By the maximum principle a time-optimal trajectory
//! rides some limit at (almost) every instant, so the peak utilization is both an
//! optimality signal (well below 1 ⇒ headroom left on the table) and a feasibility
//! check (above 1 ⇒ the executed motion exceeds a limit the grid never sampled).

use nurbs::bezier::extract_bezier_pieces;
use nurbs::ScalarNurbs;
use temporal::{restricted_norm, Limits};

/// The kinematic family a peak utilization was reached on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UtilFamily {
    Velocity,
    Accel,
    Jerk,
}

/// Peak limit utilization over a trajectory: the largest `|executed| / cap` ratio
/// reached on any axis-set and family, and which family it was.
#[derive(Debug, Clone, Copy)]
pub struct PeakUtilization {
    pub ratio: f64,
    pub family: UtilFamily,
}

/// MCU stepping period (40 kHz), matching `peak::peak_accel`.
const MCU_DT: f64 = 25e-6;

/// Peak utilization of one committed segment, sampling the per-axis time
/// polynomials at the MCU rate and forming `restricted_norm / cap` for each
/// spatial limit set and family. Returns `None` for a segment too short to carry
/// a jerk stencil or one whose caps are all non-finite.
#[must_use]
pub fn segment_peak_utilization(
    axes: &[ScalarNurbs<f64>],
    limits: &Limits,
) -> Option<PeakUtilization> {
    if axes.len() < 3 {
        return None;
    }
    let pieces = extract_bezier_pieces(&axes[0]);
    let t0 = pieces.first()?.u_start;
    let t1 = pieces.last()?.u_end;
    let dur = t1 - t0;
    if dur <= 4.0 * MCU_DT {
        return None;
    }

    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    let n = (dur / MCU_DT).ceil() as usize;
    let dt = dur / n as f64;

    let sample = |t: f64| -> [f64; 3] {
        [
            nurbs::eval::eval(&axes[0], t),
            nurbs::eval::eval(&axes[1], t),
            nurbs::eval::eval(&axes[2], t),
        ]
    };
    let x: Vec<[f64; 3]> = (0..=n).map(|i| sample(t0 + dt * i as f64)).collect();

    let mut best: Option<PeakUtilization> = None;
    let mut consider = |ratio: f64, family: UtilFamily| {
        if ratio.is_finite() && best.is_none_or(|b| ratio > b.ratio) {
            best = Some(PeakUtilization { ratio, family });
        }
    };

    for i in 2..=n - 2 {
        let mut v = [0.0_f64; 3];
        let mut a = [0.0_f64; 3];
        let mut j = [0.0_f64; 3];
        for k in 0..3 {
            v[k] = (x[i + 1][k] - x[i - 1][k]) / (2.0 * dt);
            a[k] = (x[i + 1][k] - 2.0 * x[i][k] + x[i - 1][k]) / (dt * dt);
            j[k] = (x[i + 2][k] - 2.0 * x[i + 1][k] + 2.0 * x[i - 1][k] - x[i - 2][k])
                / (2.0 * dt * dt * dt);
        }
        for (_, set) in limits.spatial_sets() {
            if set.v_max.is_finite() {
                consider(
                    restricted_norm(&v, set.axes) / set.v_max,
                    UtilFamily::Velocity,
                );
            }
            if set.a_max.is_finite() {
                consider(restricted_norm(&a, set.axes) / set.a_max, UtilFamily::Accel);
            }
            if set.j_max.is_finite() {
                consider(restricted_norm(&j, set.axes) / set.j_max, UtilFamily::Jerk);
            }
        }
    }

    best
}

/// Peak utilization across a window of committed segments, each checked against
/// its own true (un-derated) limits. `None` when no segment yields a sample.
#[must_use]
pub fn window_peak_utilization<'a>(
    segments: impl IntoIterator<Item = (&'a [ScalarNurbs<f64>], &'a Limits)>,
) -> Option<PeakUtilization> {
    let mut best: Option<PeakUtilization> = None;
    for (axes, limits) in segments {
        if let Some(u) = segment_peak_utilization(axes, limits) {
            if best.is_none_or(|b| u.ratio > b.ratio) {
                best = Some(u);
            }
        }
    }
    best
}

#[cfg(test)]
mod tests;
