//! The corner budget on the shaped path: the apex speed each blended
//! corner may cross at, the speed each arc may ride at, and the slower
//! acceleration a corner gets where even a stop at its planned one would
//! round it past its budget ([`ShapedCorners`]).

use crate::frontend::Move;
use crate::path::lowering::PositionProfile;
use crate::path::{CurvatureProfile, Segment};
use crate::shaped_corner::ShapedCorners;
use crate::vec3::{dot, norm, sub};

use super::MoveCaps;

const TURN_EPS_RAD: f64 = 1e-6;

/// Lowers `caps` for the corner budget and returns each seam's speed
/// ceiling, `seam_ceilings[k]` at the seam entering `moves[k]`. Each
/// corner's budget reads the acceleration its moves are planned under
/// before any corner slows it.
pub(super) fn constrain(
    moves: &[Move],
    caps: &mut [MoveCaps],
    stop_before: &[bool],
    corners: &ShapedCorners,
) -> Vec<f64> {
    let n = moves.len();
    let planned: Vec<f64> = caps.iter().map(|cap| cap.kin.accel).collect();
    let mut accel = planned.clone();
    let mut seam_ceilings = vec![f64::INFINITY; n + 1];
    let apexes: Vec<usize> = (1..n)
        .filter(|&k| !stop_before[k] && apex_between(&moves[k - 1], &moves[k]).is_some())
        .collect();
    for (k, m) in moves.iter().enumerate() {
        let in_pair = apexes.iter().any(|&apex| apex == k || apex == k + 1);
        if let Some(radius) = curve_radius(m).filter(|_| !in_pair) {
            let theta = facet_turn(m);
            let room = corners.budget_mm(m.limits.corner, theta, planned[k])
                - corners.arc_fit_tolerance_mm(m.limits.corner, m.limits.accel_mm_s2);
            let cap = corners.arc_speed_cap(radius, room);
            caps[k].kin.flat_ceiling = caps[k].kin.flat_ceiling.min(cap);
        }
    }
    for &k in &apexes {
        let (theta, cut) = apex_between(&moves[k - 1], &moves[k]).expect("an apex seam");
        let budget = moves[k - 1]
            .limits
            .corner
            .stricter(moves[k].limits.corner, planned[k - 1].min(planned[k]));
        let a = planned[k - 1].max(planned[k]);
        let budget_mm = corners.budget_mm(budget, theta, a);
        let slowed = corners.corner_accel(theta, cut, a, budget_mm);
        if slowed < a {
            for j in reach_of_apex(k, n, stop_before) {
                accel[j] = accel[j].min(slowed);
            }
        }
        let ratio = corners
            .apex_speed_ratio(theta, cut, slowed, budget_mm)
            .expect("the slowed corner's stop fits its budget");
        let kappa_peak = caps[k - 1].kappa_peak.max(caps[k].kappa_peak);
        seam_ceilings[k] = ratio * (slowed / kappa_peak).sqrt();
    }
    for k in 1..n {
        if !stop_before[k] {
            continue;
        }
        let Some(theta) = stop_turn(&moves[k - 1], &moves[k]) else {
            continue;
        };
        let a = planned[k - 1].max(planned[k]);
        let budget = moves[k - 1]
            .limits
            .corner
            .stricter(moves[k].limits.corner, planned[k - 1].min(planned[k]));
        let budget_mm = corners.budget_mm(budget, theta, a);
        let slowed = corners.corner_accel(theta, 0.0, a, budget_mm);
        for j in [k - 1, k] {
            accel[j] = accel[j].min(slowed);
        }
    }
    for (cap, a) in caps.iter_mut().zip(accel) {
        cap.kin.accel = a;
    }
    seam_ceilings
}

/// The moves a slowed apex at seam `k` reads: the blend's halves and the
/// moves either side of them that reach it without a stop.
fn reach_of_apex(k: usize, n: usize, stop_before: &[bool]) -> impl Iterator<Item = usize> {
    let before = (k >= 2 && !stop_before[k - 1]).then(|| k - 2);
    let after = (k + 1 < n && !stop_before[k + 1]).then_some(k + 1);
    before.into_iter().chain([k - 1, k]).chain(after)
}

/// The turn and the apex's distance from the vertex of a clothoid pair
/// whose curvature peaks at the seam between `up` and `down`: the vertex is
/// where the pair's entry and exit tangents meet.
fn apex_between(up: &Move, down: &Move) -> Option<(f64, f64)> {
    let (Some(Segment::Clothoid(first)), Some(Segment::Clothoid(second))) =
        (&up.segment.spatial, &down.segment.spatial)
    else {
        return None;
    };
    let (up_start, up_end) = first.kappa_endpoints();
    let (down_start, down_end) = second.kappa_endpoints();
    let peaks =
        up_end.abs() >= up_start.abs() && down_start.abs() >= down_end.abs() && up_end.abs() > 0.0;
    if !peaks {
        return None;
    }
    let entry = first.point_at(0.0);
    let entry_heading = first.heading_at(0.0);
    let exit = second.point_at(second.s_len());
    let exit_heading = second.heading_at(second.s_len());
    let theta = dot(entry_heading, exit_heading).clamp(-1.0, 1.0).acos();
    if theta < TURN_EPS_RAD {
        return None;
    }
    let vertex = tangent_meet(entry, entry_heading, exit, exit_heading);
    Some((theta, norm(sub(first.point_at(first.s_len()), vertex))))
}

/// The point midway between the closest points of the lines through
/// `a` along `u` and through `b` along `w`.
fn tangent_meet(a: [f64; 3], u: [f64; 3], b: [f64; 3], w: [f64; 3]) -> [f64; 3] {
    let r = sub(a, b);
    let (uu, uw, ww) = (dot(u, u), dot(u, w), dot(w, w));
    let (ur, wr) = (dot(u, r), dot(w, r));
    let det = uu * ww - uw * uw;
    let s = (uw * wr - ww * ur) / det;
    let t = (uu * wr - uw * ur) / det;
    let p = [a[0] + s * u[0], a[1] + s * u[1], a[2] + s * u[2]];
    let q = [b[0] + t * w[0], b[1] + t * w[1], b[2] + t * w[2]];
    [
        0.5 * (p[0] + q[0]),
        0.5 * (p[1] + q[1]),
        0.5 * (p[2] + q[2]),
    ]
}

/// The turn a stop between two spatial moves makes.
fn stop_turn(before: &Move, after: &Move) -> Option<f64> {
    let (Some(incoming), Some(outgoing)) = (&before.segment.spatial, &after.segment.spatial) else {
        return None;
    };
    let theta = dot(
        incoming.heading_at(incoming.s_len()),
        outgoing.heading_at(0.0),
    )
    .clamp(-1.0, 1.0)
    .acos();
    (theta >= TURN_EPS_RAD).then_some(theta)
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
