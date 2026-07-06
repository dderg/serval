use crate::frontend::Move;
use crate::path::{Arc, CurvatureProfile, Line};
use crate::segment::FollowerDemand;

use super::circle::{facet_axes, facet_ratio};
use super::ease::EasedEnd;

/// Solve the whole reconstruction's extrusion as one rate-continuous chain:
/// up spiral (ramp `r_h → r_a`), arc (ramp `r_a → r_b`), down spiral (ramp
/// `r_b → r_t`). The outer seams anchor hard at the neighbor lines' own
/// ratios (`r_h`, `r_t`), so `ė = r·v` is continuous where the construct
/// meets them; the arc's slope is fixed to what the facets commanded
/// (`r_last − r_first` over the arc length); the one remaining degree of
/// freedom — a common offset on the arc's endpoint ratios — is solved so the
/// construct deposits exactly the E of the material it replaced: the facets'
/// covered footage plus the eased neighbors' trimmed footage. That offset
/// absorbs the footage-vs-arc-length mismatch today's constant-ratio version
/// buried in its single averaged rate.
pub(super) fn construct_followers(
    facets: &[Move],
    lines: &[&Line],
    head_consumption: f64,
    tail_consumption: f64,
    arc_len: f64,
    head: Option<&EasedEnd>,
    tail: Option<&EasedEnd>,
) -> (
    Vec<FollowerDemand>,
    Vec<FollowerDemand>,
    Vec<FollowerDemand>,
) {
    let mut axes = facet_axes(facets);
    for end in [head, tail].into_iter().flatten() {
        for f in end.neighbor_followers {
            if !axes.contains(&f.axis_index) {
                axes.push(f.axis_index);
            }
        }
    }

    let last = lines.len() - 1;
    let mut up = Vec::new();
    let mut arc = Vec::new();
    let mut down = Vec::new();
    for axis in axes {
        let r_first = facet_ratio(&facets[0], axis);
        let r_last = facet_ratio(facets.last().expect("run has facets"), axis);
        let delta = r_last - r_first;
        let neighbor_ratio = |end: &EasedEnd| {
            end.neighbor_followers
                .iter()
                .find(|f| f.axis_index == axis)
                .map_or(0.0, |f| {
                    assert!(
                        !f.is_ramped(),
                        "arc-run facets and neighbors must carry constant follower ratios"
                    );
                    f.ratio
                })
        };
        let r_h = head.map_or(0.0, &neighbor_ratio);
        let r_t = tail.map_or(0.0, &neighbor_ratio);
        let len_up = head.map_or(0.0, |e| e.spiral_len);
        let len_down = tail.map_or(0.0, |e| e.spiral_len);

        let mut e_total = 0.0;
        for (i, m) in facets.iter().enumerate() {
            let covered = if i == 0 {
                head_consumption
            } else if i == last {
                tail_consumption
            } else {
                lines[i].s_len()
            };
            e_total += facet_ratio(m, axis) * covered;
        }
        if let Some(end) = head {
            e_total += r_h * end.line_trim;
        }
        if let Some(end) = tail {
            e_total += r_t * end.line_trim;
        }

        let r_a =
            (e_total - 0.5 * r_h * len_up - 0.5 * delta * arc_len - 0.5 * (delta + r_t) * len_down)
                / (0.5 * len_up + arc_len + 0.5 * len_down);
        let r_b = r_a + delta;

        let push_nonzero = |v: &mut Vec<FollowerDemand>, d: FollowerDemand| {
            if d.max_abs_ratio() > 1e-12 {
                v.push(d);
            }
        };
        if head.is_some() {
            push_nonzero(&mut up, FollowerDemand::ramp(axis, r_h, r_a));
        }
        push_nonzero(&mut arc, FollowerDemand::ramp(axis, r_a, r_b));
        if tail.is_some() {
            push_nonzero(&mut down, FollowerDemand::ramp(axis, r_b, r_t));
        }
    }
    (up, arc, down)
}

pub(in crate::fitter) fn arc_len(arc: &Arc) -> f64 {
    arc.radius * arc.sweep.abs()
}
