//! Checks every piece a fit accepts against the exact signal it was fitted
//! to and keeps the ones that break the fit tolerance as resolved for the
//! piece's duration (`FitTol::for_piece`). Where the signal has exact
//! polynomials or an enclosure over the piece, the Bernstein hull's proven
//! bound on the error is the measure; elsewhere a dense scan of its resolved
//! stretch, edges included, and an exact search around every scanned peak,
//! never only at points the fit itself sampled. Where the fit's span has
//! exact polynomials, they are held to the signal's own evaluator at one
//! point of each sub-interval. What the fit does not resolve is not held to
//! it: pieces narrower than the cubic floor, and the slivers inside a
//! piece's edges that the ladder reads its anchoring state past, which seed
//! coalescing treats as carrying no integral. Compiled only for tests that
//! hold the shaper to its `max_path_deviation` / `max_accel_deviation`
//! promise; the live host never pays for it.

use std::convert::Infallible;
use std::sync::Mutex;

use crate::lowering::{FitTol, eval_mono, eval_mono_dd};
use trajectory::next_toward;

use crate::shaper::exact::ExactSpan;
use crate::shaper::{
    TrackSignal, golden_section_peak, proven_error_bounds, square_root_onset_on_edge,
};

/// Intervals each accepted piece's resolved stretch is scanned at, its
/// edges included. Prime, so no scan point but the edges lands on the fit's
/// own probes, nodes or onset check points: the audit never looks only
/// where the fit already looked.
const AUDIT_SCAN_INTERVALS: u32 = 97;

/// Share of the tolerance above which a local peak of the scanned miss is
/// searched for exactly between its scan neighbours.
const AUDIT_PEAK_SEARCH_SHARE: f64 = 0.25;

/// Share of a piece's tolerance by which the span's exact polynomials may
/// differ from the signal's evaluator: a verdict read off the one holds for
/// the other to within it.
const EXACT_EVALUATOR_AGREEMENT_SHARE: f64 = 1e-6;

/// Where across its stretch of the piece each exact sub-interval is held to
/// the evaluator: the golden section, which no probe ladder lands on.
const EXACT_EVALUATOR_CHECK_AT: f64 = 0.381_966_011_250_105_1;

/// A piece's miss of the exact signal at one point, over the tolerance.
#[derive(Clone, Copy)]
struct Miss {
    position: f64,
    acceleration: f64,
}

#[derive(Default)]
struct WorstMiss {
    position: f64,
    acceleration: f64,
    worst_u: f64,
}

impl WorstMiss {
    fn take(&mut self, u: f64, miss: Miss) {
        self.position = self.position.max(miss.position);
        if miss.acceleration > self.acceleration {
            self.acceleration = miss.acceleration;
            self.worst_u = u;
        }
    }
}

/// One accepted piece whose position or acceleration leaves the fit
/// tolerance somewhere inside it.
#[derive(Debug, Clone)]
pub struct FitViolation {
    pub fit_context: &'static str,
    pub axis: usize,
    pub t_start: f64,
    pub duration_s: f64,
    pub degree: usize,
    pub position_over_tolerance: f64,
    pub acceleration_over_tolerance: f64,
    pub worst_u: f64,
    pub accepted_at_floor: bool,
    /// The exact acceleration changes by more than the tolerance between
    /// one ulp inside an edge of the piece and the edge's inset: a break
    /// hidden in the sliver the ladder must not read its edge state from.
    /// A steep but smooth signal trips it too — the square-root approach of
    /// a disk rail to its curvature cap does, in materialized sources — so
    /// on its own it proves a break only where the signal is assembled from
    /// other tracks' pieces.
    pub break_in_edge_sliver: bool,
    pub exemption: Option<Exemption>,
}

static VIOLATIONS: Mutex<Vec<FitViolation>> = Mutex::new(Vec::new());

/// Every violation recorded since the previous call, by any thread: two
/// audited runs in one process take each other's.
pub fn take_violations() -> Vec<FitViolation> {
    std::mem::take(&mut *VIOLATIONS.lock().expect("fit audit poisoned"))
}

#[derive(Clone, Copy)]
pub(crate) struct AcceptedPiece<'a> {
    pub fit_context: &'static str,
    pub axis: usize,
    pub fit_tol: FitTol,
    pub t0: f64,
    pub t1: f64,
    pub mono_u: &'a [f64],
    pub accepted_at_floor: bool,
    pub cubic_floor: f64,
    /// How far inside each edge the ladder read the piece's anchoring
    /// state; what lies closer to an edge is not resolved by the fit.
    pub edge_insets: (f64, f64),
    /// The span's exact polynomials the fit read its truth off, where it
    /// had them.
    pub exact: Option<&'a ExactSpan>,
}

pub(crate) fn check<S: TrackSignal>(sig: &S, piece: AcceptedPiece) {
    let h = piece.t1 - piece.t0;
    if h < piece.cubic_floor {
        return;
    }
    let acceleration_scale = (2.0 / h) * (2.0 / h);
    let piece_tol = piece.fit_tol.for_piece(h);
    if let Some(exact) = piece.exact {
        hold_exact_to_evaluator(sig, exact, &piece, piece_tol);
    }
    let resolved = (
        piece.t0 + piece.edge_insets.0,
        piece.t1 - piece.edge_insets.1,
    );
    let floor_allowance = piece_tol.accel_mm_s2 * h;
    let proven = proven_error_bounds(
        sig,
        piece.exact.map(|exact| exact.over(resolved.0, resolved.1)),
        piece.mono_u,
        (piece.t0, piece.t1),
        resolved,
        [piece_tol.pos_mm, floor_allowance, piece_tol.accel_mm_s2],
    );
    if proven.is_some_and(|[position, _, acceleration]| {
        position <= piece_tol.pos_mm && acceleration <= piece_tol.accel_mm_s2
    }) {
        return;
    }
    let midpoint = 0.5 * (piece.t0 + piece.t1);
    let truth_at = |t: f64| match piece.exact {
        Some(exact) => exact.pva(t, if t < midpoint { piece.t1 } else { piece.t0 }),
        None => sig.eval_pva(t),
    };
    let miss_at = |u: f64| {
        let x = 2.0 * u - 1.0;
        let t = piece.t0 + h * u;
        let (truth_position, _, truth_acceleration) = truth_at(t);
        assert!(
            truth_position.is_finite() && truth_acceleration.is_finite(),
            "{} axis {}: the exact signal the fit accepted [{}, {}] against is not finite \
             at {t} (position {truth_position}, acceleration {truth_acceleration})",
            piece.fit_context,
            piece.axis,
            piece.t0,
            piece.t1
        );
        Miss {
            position: (eval_mono(piece.mono_u, x) - truth_position).abs() / piece_tol.pos_mm,
            acceleration: (eval_mono_dd(piece.mono_u, x) * acceleration_scale - truth_acceleration)
                .abs()
                / piece_tol.accel_mm_s2,
        }
    };
    let (resolved_from, resolved_to) = (piece.edge_insets.0 / h, 1.0 - piece.edge_insets.1 / h);
    let step = (resolved_to - resolved_from) / f64::from(AUDIT_SCAN_INTERVALS);
    let scan: [(f64, Miss); AUDIT_SCAN_INTERVALS as usize + 1] = std::array::from_fn(|k| {
        let u = if k == AUDIT_SCAN_INTERVALS as usize {
            resolved_to
        } else {
            (k as f64).mul_add(step, resolved_from)
        };
        (u, miss_at(u))
    });
    let mut worst = WorstMiss::default();
    for &(u, miss) in &scan {
        worst.take(u, miss);
    }
    let by_position: fn(&Miss) -> f64 = |miss| miss.position;
    let by_acceleration: fn(&Miss) -> f64 = |miss| miss.acceleration;
    for measure in [by_position, by_acceleration] {
        for (index, (u, miss)) in scan.iter().enumerate() {
            let value = measure(miss);
            let neighbour = |offset: Option<usize>| {
                offset
                    .and_then(|k| scan.get(k))
                    .map_or(0.0, |(_, miss)| measure(miss))
            };
            if value <= AUDIT_PEAK_SEARCH_SHARE
                || value < neighbour(index.checked_sub(1))
                || value < neighbour(Some(index + 1))
            {
                continue;
            }
            let Ok((peak_u, _)) = golden_section_peak(
                |u| Ok::<_, Infallible>(measure(&miss_at(u))),
                (u - step).max(resolved_from),
                (u + step).min(resolved_to),
            );
            worst.take(peak_u, miss_at(peak_u));
        }
    }
    let WorstMiss {
        mut position,
        mut acceleration,
        worst_u,
    } = worst;
    if let Some([position_bound, _, acceleration_bound]) = proven {
        position = position_bound / piece_tol.pos_mm;
        acceleration = acceleration_bound / piece_tol.accel_mm_s2;
    }
    if position > 1.0 || acceleration > 1.0 {
        let exemption = proven
            .filter(|_| position <= 1.0)
            .and_then(|[_, velocity_error, _]| onset_floor(sig, &piece, velocity_error));
        let edge_change = |edge: f64, inside: f64, past_sliver: f64| {
            (sig.eval_pva(next_toward(edge, inside)).2 - sig.eval_pva(past_sliver).2).abs()
                > piece.fit_tol.accel_mm_s2
        };
        let break_in_edge_sliver = edge_change(piece.t0, piece.t1, piece.t0 + piece.edge_insets.0)
            || edge_change(piece.t1, piece.t0, piece.t1 - piece.edge_insets.1);
        VIOLATIONS
            .lock()
            .expect("fit audit poisoned")
            .push(FitViolation {
                fit_context: piece.fit_context,
                axis: piece.axis,
                t_start: piece.t0,
                duration_s: h,
                degree: piece.mono_u.len() - 1,
                position_over_tolerance: position,
                acceleration_over_tolerance: acceleration,
                worst_u,
                accepted_at_floor: piece.accepted_at_floor,
                break_in_edge_sliver,
                exemption,
            });
    }
}

/// Holds every exact sub-interval reaching into the piece's resolved stretch
/// to the signal's evaluator at one point strictly inside it.
fn hold_exact_to_evaluator<S: TrackSignal>(
    sig: &S,
    exact: &ExactSpan,
    piece: &AcceptedPiece,
    piece_tol: FitTol,
) {
    let (from, to) = (
        piece.t0 + piece.edge_insets.0,
        piece.t1 - piece.edge_insets.1,
    );
    for polynomial in exact.over(from, to) {
        let (lo, hi) = (polynomial.lo.max(from), polynomial.hi.min(to));
        let t = (hi - lo).mul_add(EXACT_EVALUATOR_CHECK_AT, lo);
        if t <= lo || t >= hi {
            continue;
        }
        let (exact_position, _, exact_acceleration) = polynomial.eval_pva(t);
        let (position, _, acceleration) = sig.eval_pva(t);
        assert!(
            (exact_position - position).abs() <= EXACT_EVALUATOR_AGREEMENT_SHARE * piece_tol.pos_mm
                && (exact_acceleration - acceleration).abs()
                    <= EXACT_EVALUATOR_AGREEMENT_SHARE * piece_tol.accel_mm_s2,
            "{} axis {}: the exact polynomials the fit read [{}, {}] off disagree with the \
             signal's evaluator at {t}: position {exact_position} against {position}, \
             acceleration {exact_acceleration} against {acceleration}",
            piece.fit_context,
            piece.axis,
            piece.t0,
            piece.t1
        );
    }
}

/// Why a violating piece is not a fit failure.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Exemption {
    /// A piece at the fit floor on a declared square-root onset: no span
    /// shorter resolves the onset's shoulder, so the piece answers to the
    /// floor's contract — its position within the tolerance and its
    /// velocity within what the acceleration tolerance carries across it,
    /// `allowance = accel_tol·h`, both proven — not to its acceleration.
    OnsetFloor { velocity_error: f64, allowance: f64 },
}

/// The [`Exemption::OnsetFloor`] of a violating piece with its position
/// proven within the tolerance and its velocity error proven at most
/// `velocity_error`, where it holds.
fn onset_floor<S: TrackSignal>(
    sig: &S,
    piece: &AcceptedPiece,
    velocity_error: f64,
) -> Option<Exemption> {
    let h = piece.t1 - piece.t0;
    let allowance = piece.fit_tol.for_piece(h).accel_mm_s2 * h;
    let on_onset =
        square_root_onset_on_edge(sig, piece.t0) || square_root_onset_on_edge(sig, piece.t1);
    (piece.accepted_at_floor
        && h <= 2.0 * piece.cubic_floor
        && on_onset
        && velocity_error <= allowance)
        .then_some(Exemption::OnsetFloor {
            velocity_error,
            allowance,
        })
}

#[cfg(test)]
#[path = "fit_audit_tests.rs"]
mod tests;
