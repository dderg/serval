//! The corner budget on the shaped path, at the acceleration each corner
//! is planned at ([`ShapedCorners`]): the speed each blended corner's apex
//! may be crossed at. A corner whose stop rounds past its budget, and a
//! curve, arrive with their slower acceleration and their speed baked into
//! their moves' limits already (the planner's approach stage).

use crate::frontend::Move;
use crate::shaped_corner::{ShapedCorners, blend_apex};

use super::MoveCaps;

/// Returns each seam's speed ceiling, `seam_ceilings[k]` at the seam
/// entering `moves[k]`: a blend's apex speed, held as a corner alone (as a
/// vertex of the curve its neighbouring corners trace it arrives capped
/// already). Each ceiling is also recorded as the speed limit of both
/// halves ([`MoveCaps::corner_speed`]).
pub(super) fn hold(
    moves: &[Move],
    caps: &mut [MoveCaps],
    stop_before: &[bool],
    corners: &ShapedCorners,
) -> Vec<f64> {
    let n = moves.len();
    let mut seam_ceilings = vec![f64::INFINITY; n + 1];
    let apexes: Vec<Apex> = (1..n)
        .filter(|&k| !stop_before[k])
        .filter_map(|k| Apex::of(moves, caps, k, corners))
        .collect();
    for apex in &apexes {
        let k = apex.seam;
        let kappa_peak = caps[k - 1].kappa_peak.max(caps[k].kappa_peak);
        let apex_speed = apex.ratio * (apex.accel / kappa_peak).sqrt();
        seam_ceilings[k] = apex_speed;
        for j in [k - 1, k] {
            caps[j].corner_speed = caps[j].corner_speed.min(apex_speed);
        }
    }
    seam_ceilings
}

/// A blended corner the corner budget holds at its apex: a clothoid pair
/// whose stop there fits its budget at the acceleration it is planned at.
/// A pair whose stop does not is no corner the fitter cut but a run of
/// facets it replaced, held as a curve instead.
struct Apex {
    seam: usize,
    accel: f64,
    ratio: f64,
}

impl Apex {
    fn of(moves: &[Move], caps: &[MoveCaps], k: usize, corners: &ShapedCorners) -> Option<Self> {
        let (theta, cut) = blend_apex(&moves[k - 1], &moves[k])?;
        let accel = caps[k - 1].kin.accel.max(caps[k].kin.accel);
        let budget = moves[k - 1].limits.corner.stricter(
            moves[k].limits.corner,
            caps[k - 1].kin.accel.min(caps[k].kin.accel),
        );
        let budget_mm = corners.budget_mm(budget, theta, accel);
        let ratio = corners.apex_speed_ratio(theta, cut, accel, budget_mm)?;
        Some(Self {
            seam: k,
            accel,
            ratio,
        })
    }
}
