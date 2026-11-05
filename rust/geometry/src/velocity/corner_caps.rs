//! The corner budget on the shaped path, at the acceleration each corner
//! is planned at ([`ShapedCorners`]): the speed each blended corner's apex
//! may be crossed at and the speed each curve may ride at. A corner whose
//! stop rounds past its budget arrives with its slower acceleration baked
//! into its moves' limits already (the planner's approach stage).

use crate::frontend::Move;
use crate::path::lowering::PositionProfile;
use crate::path::{CurvatureProfile, Segment};
use crate::shaped_corner::{ShapedCorners, blend_apex};
use crate::vec3::dot;

use super::MoveCaps;

/// Caps each curve's speed in `caps` and returns each seam's speed
/// ceiling, `seam_ceilings[k]` at the seam entering `moves[k]`. Each cap is
/// also recorded as the speed limit of its moves ([`MoveCaps::corner_speed`]):
/// a blend's apex speed caps both its halves there.
pub(super) fn hold(
    moves: &[Move],
    caps: &mut [MoveCaps],
    stop_before: &[bool],
    corners: &ShapedCorners,
) -> Vec<f64> {
    let n = moves.len();
    let mut seam_ceilings = vec![f64::INFINITY; n + 1];
    let mut apex_halves = vec![false; n];
    for k in 1..n {
        if stop_before[k] {
            continue;
        }
        let Some((theta, cut)) = blend_apex(&moves[k - 1], &moves[k]) else {
            continue;
        };
        let accel = caps[k - 1].kin.accel.max(caps[k].kin.accel);
        let budget = moves[k - 1].limits.corner.stricter(
            moves[k].limits.corner,
            caps[k - 1].kin.accel.min(caps[k].kin.accel),
        );
        let budget_mm = corners.budget_mm(budget, theta, accel);
        if cut >= budget_mm {
            continue;
        }
        let ratio = corners
            .apex_speed_ratio(theta, cut, accel, budget_mm)
            .unwrap_or_else(|| {
                panic!(
                    "line {}: a stop at the corner turning {theta} rad, cut {cut} mm, misses \
                     past its {budget_mm} mm budget at the {accel} mm/s² it is planned at",
                    moves[k].source.start_line
                )
            });
        let kappa_peak = caps[k - 1].kappa_peak.max(caps[k].kappa_peak);
        let apex_speed = ratio * (accel / kappa_peak).sqrt();
        seam_ceilings[k] = apex_speed;
        for j in [k - 1, k] {
            apex_halves[j] = true;
            caps[j].corner_speed = caps[j].corner_speed.min(apex_speed);
        }
    }
    for (k, m) in moves.iter().enumerate() {
        let Some(radius) = curve_radius(m).filter(|_| !apex_halves[k]) else {
            continue;
        };
        let room = corners.budget_mm(m.limits.corner, facet_turn(m), caps[k].kin.accel)
            - corners.arc_fit_tolerance_mm(m.limits.corner, m.limits.accel_mm_s2);
        let cap = corners.arc_speed_cap(radius, room);
        caps[k].kin.flat_ceiling = caps[k].kin.flat_ceiling.min(cap);
        caps[k].corner_speed = caps[k].corner_speed.min(cap);
    }
    seam_ceilings
}

/// The tightest radius a curved move that is no corner blend rides.
fn curve_radius(m: &Move) -> Option<f64> {
    let segment = m.segment.spatial.as_ref()?;
    if matches!(segment, Segment::Line(_)) {
        return None;
    }
    let (_, kappa_peak) = segment.kappa_peak();
    (kappa_peak.abs() > 0.0).then(|| 1.0 / kappa_peak.abs())
}

/// The turn at each G-code vertex a curved move stands in for: its sweep
/// shared over the lines it replaces.
fn facet_turn(m: &Move) -> f64 {
    let Some(segment) = &m.segment.spatial else {
        return 0.0;
    };
    let lines = f64::from(m.source.end_line.saturating_sub(m.source.start_line) + 1);
    let sweep = dot(segment.heading_at(0.0), segment.heading_at(segment.s_len()))
        .clamp(-1.0, 1.0)
        .acos();
    sweep / lines
}
