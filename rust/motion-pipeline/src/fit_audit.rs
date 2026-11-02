//! Checks every piece a fit accepts against the exact signal it was fitted
//! to, at dense interior points rather than only at the ladder's probes, and
//! keeps the ones that break the fit tolerance. What the fit does not resolve
//! is not held to it: pieces narrower than the cubic floor, and the slivers
//! inside a piece's edges that the ladder reads its anchoring state past,
//! which seed coalescing treats as carrying no integral. Compiled only for
//! tests that hold the shaper to its `max_path_deviation` /
//! `max_accel_deviation` promise; the live host never pays for it.

use std::sync::Mutex;

use crate::lowering::{FitTol, eval_mono, eval_mono_dd};
use trajectory::next_toward;

use crate::shaper::TrackSignal;

/// Interior points checked per accepted piece, spread uniformly so a feature
/// the ladder's nine probes straddle cannot hide between them.
const AUDIT_POINTS: u32 = 64;

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
}

static VIOLATIONS: Mutex<Vec<FitViolation>> = Mutex::new(Vec::new());

/// Every violation recorded since the previous call.
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
}

pub(crate) fn check<S: TrackSignal>(sig: &S, piece: AcceptedPiece) {
    let h = piece.t1 - piece.t0;
    if h < piece.cubic_floor {
        return;
    }
    let acceleration_scale = (2.0 / h) * (2.0 / h);
    let (mut position, mut acceleration, mut worst_u) = (0.0_f64, 0.0_f64, 0.0);
    let (start_inset, end_inset) = (piece.edge_insets.0 / h, piece.edge_insets.1 / h);
    for k in 0..AUDIT_POINTS {
        let u = start_inset
            + (1.0 - start_inset - end_inset) * (f64::from(k) + 0.5) / f64::from(AUDIT_POINTS);
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
        position = position
            .max((eval_mono(piece.mono_u, x) - truth_position).abs() / piece.fit_tol.pos_mm);
        let over = (eval_mono_dd(piece.mono_u, x) * acceleration_scale - truth_acceleration).abs()
            / piece.fit_tol.accel_mm_s2;
        if over > acceleration {
            acceleration = over;
            worst_u = u;
        }
    }
    if position > 1.0 || acceleration > 1.0 {
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
            });
    }
}
