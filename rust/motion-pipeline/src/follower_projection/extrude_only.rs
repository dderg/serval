//! A pressure advance models the nozzle pressure the flow along the path
//! builds. A follower whose kernel carries a linear advance gains its whole
//! projected input, so the extrude-only motion folded into that input — the
//! retracts and primes a segment without a spatial path commands — has its
//! gain taken back out: it keeps its own motion and the extrude-only caps it
//! was planned under, as on a ratio-ramped chain.

use nurbs::bezier::{BezierPiece, split_piece_at};
use trajectory::ContinuousAxis;

use super::axis_breakpoints;

/// Relative velocity mismatch at which an extrude-only stretch is not the
/// cubic its pieces are built as, on top of what rounding the two probe
/// times, deep into a stream's clock, moves the velocity by.
const CUBIC_CHECK_RELATIVE: f64 = 1e-9;
const CUBIC_CHECK_FLOOR_MM_S: f64 = 1e-9;
const PROBE_TIME_ROUNDING_ULPS: f64 = 8.0;

/// One stretch of a segment's extrude-only motion between the raw axis's
/// breakpoints, where its velocity is `velocity + accel·τ + jerk·τ²/2` in
/// `τ = t − t_start`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct ExtrudeOnlyPiece {
    t_start: f64,
    t_end: f64,
    velocity: f64,
    accel: f64,
    jerk: f64,
}

impl ExtrudeOnlyPiece {
    /// The gain `k1·ė + k2·ë` from `at` on, as power-basis coefficients in
    /// `τ = t − at`.
    fn gain_from(&self, at: f64, k1: f64, k2: f64) -> [f64; 3] {
        let d = at - self.t_start;
        let velocity = self.velocity + self.accel * d + 0.5 * self.jerk * d * d;
        let accel = self.accel + self.jerk * d;
        [
            k1 * velocity + k2 * accel,
            k1 * accel + k2 * self.jerk,
            0.5 * k1 * self.jerk,
        ]
    }
}

/// The extrude-only motion of `raw_axis` over `[t_start, t_end]`, empty for
/// a hold. Each stretch is read at its midpoint and must be a cubic: the
/// planner's extrude-only moves are, and any other motion fails loudly
/// rather than lose part of its gain.
pub(super) fn extrude_only_pieces(
    axis: usize,
    raw_axis: &ContinuousAxis,
    (t_start, t_end): (f64, f64),
) -> Vec<ExtrudeOnlyPiece> {
    if matches!(raw_axis, ContinuousAxis::Hold { .. }) {
        return Vec::new();
    }
    let mut cuts = vec![t_start];
    cuts.extend(
        axis_breakpoints(raw_axis)
            .into_iter()
            .filter(|&t| t > t_start && t < t_end),
    );
    cuts.push(t_end);
    cuts.dedup();
    let pvaj = |t: f64| {
        raw_axis.eval_pvaj(t).unwrap_or_else(|error| {
            panic!("follower axis {axis} extrude-only motion at t={t} is not evaluable: {error}")
        })
    };
    cuts.windows(2)
        .filter(|stretch| stretch[1] > stretch[0])
        .map(|stretch| {
            let (start, end) = (stretch[0], stretch[1]);
            let half = 0.5 * (end - start);
            let mid = pvaj(start + half);
            let piece = ExtrudeOnlyPiece {
                t_start: start,
                t_end: end,
                velocity: mid.velocity - mid.acceleration * half + 0.5 * mid.jerk * half * half,
                accel: mid.acceleration - mid.jerk * half,
                jerk: mid.jerk,
            };
            let check_at = start + 1.5 * half;
            let predicted = piece.gain_from(check_at, 1.0, 0.0)[0];
            let actual = pvaj(check_at).velocity;
            let time_rounding = PROBE_TIME_ROUNDING_ULPS * f64::EPSILON * check_at.abs().max(1.0);
            let tolerance = CUBIC_CHECK_RELATIVE * actual.abs().max(predicted.abs())
                + CUBIC_CHECK_FLOOR_MM_S
                + (mid.acceleration.abs() + mid.jerk.abs() * (end - start)) * time_rounding;
            assert!(
                (predicted - actual).abs() <= tolerance,
                "follower axis {axis} extrude-only motion over [{start}, {end}] is not a cubic: \
                 velocity {actual} mm/s at t={check_at} against {predicted} mm/s read from its \
                 midpoint"
            );
            piece
        })
        .collect()
}

/// `pieces`, a gained follower input in power basis, less the gain
/// `k1·ė + k2·ë` of the extrude-only motion inside them.
pub(super) fn without_extrude_only_gain<'a>(
    pieces: Vec<BezierPiece>,
    extrude_only: impl IntoIterator<Item = &'a ExtrudeOnlyPiece>,
    (k1, k2): (f64, f64),
) -> Vec<BezierPiece> {
    let mut extrude_only: Vec<&ExtrudeOnlyPiece> = extrude_only.into_iter().collect();
    if extrude_only.is_empty() {
        return pieces;
    }
    extrude_only.sort_by(|a, b| a.t_start.total_cmp(&b.t_start));
    let mut out = Vec::with_capacity(pieces.len() + 2 * extrude_only.len());
    let mut next = 0;
    for piece in pieces {
        while next < extrude_only.len() && extrude_only[next].t_end <= piece.u_start {
            next += 1;
        }
        let mut rest = piece;
        for stretch in &extrude_only[next..] {
            if stretch.t_start >= rest.u_end {
                break;
            }
            if stretch.t_start > rest.u_start {
                let (before, after) = split_piece_at(&rest, stretch.t_start);
                out.push(before);
                rest = after;
            }
            if stretch.t_end >= rest.u_end {
                subtract_gain(&mut rest, stretch, (k1, k2));
                break;
            }
            let (mut inside, after) = split_piece_at(&rest, stretch.t_end);
            subtract_gain(&mut inside, stretch, (k1, k2));
            out.push(inside);
            rest = after;
        }
        out.push(rest);
    }
    out
}

fn subtract_gain(piece: &mut BezierPiece, stretch: &ExtrudeOnlyPiece, (k1, k2): (f64, f64)) {
    let gain = stretch.gain_from(piece.u_start, k1, k2);
    if piece.coeffs.len() < gain.len() {
        piece.coeffs.resize(gain.len(), 0.0);
    }
    for (coeff, gain) in piece.coeffs.iter_mut().zip(gain) {
        *coeff -= gain;
    }
}

#[cfg(test)]
mod tests;
