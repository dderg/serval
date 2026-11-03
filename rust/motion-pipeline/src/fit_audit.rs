//! Checks every piece a fit accepts against the exact signal it was fitted
//! to and keeps the ones that break the fit tolerance as resolved for the
//! piece's duration (`FitTol::for_piece`). Where the signal has exact
//! polynomials or an enclosure over the piece, the Bernstein hull's proven
//! bound on the error is the measure; elsewhere a dense scan of its resolved
//! stretch, edges included, and an exact search around every scanned peak,
//! never only at points the fit itself sampled. What the fit does not
//! resolve is not held to it: pieces narrower than the cubic floor, and the
//! slivers inside a piece's edges that the ladder reads its anchoring state
//! past, which seed coalescing treats as carrying no integral. Compiled only
//! for tests that hold the shaper to its `max_path_deviation` /
//! `max_accel_deviation` promise; the live host never pays for it.

use std::cell::RefCell;
use std::convert::Infallible;
use std::sync::Mutex;

use crate::lowering::{FitTol, eval_mono, eval_mono_dd};
use trajectory::next_toward;

use crate::shaper::{
    SQUARE_ROOT_ONSET_EDGE_SLACK_S, TrackSignal, golden_section_peak, proven_error_bounds,
};

/// Intervals each accepted piece's resolved stretch is scanned at, its
/// edges included. Prime, so no scan point but the edges lands on the fit's
/// own probes, nodes or onset check points: the audit never looks only
/// where the fit already looked.
const AUDIT_SCAN_INTERVALS: u32 = 97;

/// Share of the tolerance above which a local peak of the scanned miss is
/// searched for exactly between its scan neighbours.
const AUDIT_PEAK_SEARCH_SHARE: f64 = 0.25;

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
    /// The exact polynomials the fit proved the piece against, where the
    /// signal has them.
    pub exact: Option<&'a [crate::shaper::ExactPiece]>,
}

pub(crate) fn check<S: TrackSignal>(sig: &S, piece: AcceptedPiece) {
    let h = piece.t1 - piece.t0;
    if h < piece.cubic_floor {
        return;
    }
    let acceleration_scale = (2.0 / h) * (2.0 / h);
    let piece_tol = piece.fit_tol.for_piece(h);
    let resolved = (
        piece.t0 + piece.edge_insets.0,
        piece.t1 - piece.edge_insets.1,
    );
    let proven = proven_error_bounds(
        sig,
        piece.exact,
        piece.mono_u,
        (piece.t0, piece.t1),
        resolved,
        piece_tol,
    );
    if proven.is_some_and(|[position, acceleration]| {
        position <= piece_tol.pos_mm && acceleration <= piece_tol.accel_mm_s2
    }) {
        return;
    }
    let miss_at = |u: f64| {
        let x = 2.0 * u - 1.0;
        let t = piece.t0 + h * u;
        let (truth_position, _, truth_acceleration) = sig.eval_pva(t);
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
    if let Some([position_bound, acceleration_bound]) = proven {
        position = position_bound / piece_tol.pos_mm;
        acceleration = acceleration_bound / piece_tol.accel_mm_s2;
    }
    if position > 1.0 || acceleration > 1.0 {
        let exemption = (position <= 1.0)
            .then(|| cap_contact_onset(sig, &piece, acceleration))
            .flatten();
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

/// Why a violating piece is not a fit failure: what no representable span
/// could have held to the tolerance, with the measurements that bound it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Exemption {
    /// A floor piece abutting a disk rail's curvature-cap contact, where the
    /// exact acceleration rises as `onset_coefficient · √τ`: no polynomial
    /// whose acceleration has degree n misses it by less than
    /// `E_n · |B| · √h`. The rungs, anchored to the endpoint state, are not
    /// minimax: they are held to an empirical multiple of that floor.
    CapContactOnset {
        onset_coefficient: f64,
        error_mm_s2: f64,
        bound_mm_s2: f64,
    },
}

/// Minimax error of √x on [0, 1] by polynomials of degree n, by Remez.
const SQRT_MINIMAX_ERROR: [f64; 12] = [
    0.5, 0.125, 0.06762, 0.04593, 0.03469, 0.02785, 0.02325, 0.01995, 0.01747, 0.01554, 0.01399,
    0.01272,
];

/// Empirical ceiling on how far an endpoint-anchored rung exceeds the
/// minimax error at a square-root onset: measured 1.65–2.47× on every
/// onset floor piece of the dense slices, with margin. Not a proven bound —
/// the rungs are not interpolants at the probes, so no Lebesgue constant
/// applies to them.
pub const ANCHORED_RUNG_EXCESS_CEILING: f64 = 3.3;

thread_local! {
    static CAP_CONTACTS: RefCell<Vec<f64>> = const { RefCell::new(Vec::new()) };
}

/// The curvature-cap contact times of the source being fitted on this
/// thread, for as long as the scope lives.
pub(crate) struct CapContactScope;

impl CapContactScope {
    pub(crate) fn enter(contacts: &[f64]) -> Self {
        CAP_CONTACTS.with(|held| {
            let mut held = held.borrow_mut();
            held.clear();
            held.extend_from_slice(contacts);
        });
        Self
    }
}

impl Drop for CapContactScope {
    fn drop(&mut self) {
        CAP_CONTACTS.with(|held| held.borrow_mut().clear());
    }
}

fn cap_contact_onset<S: TrackSignal>(
    sig: &S,
    piece: &AcceptedPiece,
    acceleration_over_tolerance: f64,
) -> Option<Exemption> {
    let h = piece.t1 - piece.t0;
    if piece.fit_context != "materialize_source"
        || !piece.accepted_at_floor
        || h > 2.0 * piece.cubic_floor
    {
        return None;
    }
    let touches = |edge: f64| {
        CAP_CONTACTS.with(|held| {
            held.borrow()
                .iter()
                .any(|&contact| (contact - edge).abs() <= SQUARE_ROOT_ONSET_EDGE_SLACK_S)
        })
    };
    let (onset, far) = if touches(piece.t0) {
        (
            next_toward(piece.t0, piece.t1),
            next_toward(piece.t1, piece.t0),
        )
    } else if touches(piece.t1) {
        (
            next_toward(piece.t1, piece.t0),
            next_toward(piece.t0, piece.t1),
        )
    } else {
        return None;
    };
    let onset_coefficient = (sig.eval_pva(far).2 - sig.eval_pva(onset).2) / h.sqrt();
    let acceleration_degree = (piece.mono_u.len() - 1).checked_sub(2)?;
    let minimax = *SQRT_MINIMAX_ERROR.get(acceleration_degree)?;
    let bound_mm_s2 = ANCHORED_RUNG_EXCESS_CEILING * minimax * onset_coefficient.abs() * h.sqrt();
    let error_mm_s2 = acceleration_over_tolerance * piece.fit_tol.for_piece(h).accel_mm_s2;
    (error_mm_s2 <= bound_mm_s2).then_some(Exemption::CapContactOnset {
        onset_coefficient,
        error_mm_s2,
        bound_mm_s2,
    })
}

#[cfg(test)]
#[path = "fit_audit_tests.rs"]
mod tests;
