use crate::frontend::Move;
use crate::path::Arc;
use crate::segment::FollowerDemand;

use super::circle::{facet_axes, facet_ratio};
use super::ease::EasedEnd;

/// Solve the whole reconstruction's extrusion as one rate-continuous chain:
/// up spiral (ramp `r_h → r_a`), arc (ramp `r_a → r_b`), down spiral (ramp
/// `r_b → r_t`). The outer seams anchor hard at the neighbor lines' own
/// ratios (`r_h`, `r_t`), so `ė = r·v` is continuous where the construct
/// meets them, and the arc anchors at the facets' own commanded rates
/// (`r_a = r_first`, `r_b = r_last`): the construct extrudes the commanded
/// `de/ds` over the path it actually travels. Total E is whatever that rate
/// integrates to — deliberately NOT the replaced footage's E, which would
/// deposit the footage-vs-arc-length mismatch as a rate excursion over the
/// reconstruction.
pub(super) fn construct_followers(
    facets: &[Move],
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

    let mut up = Vec::new();
    let mut arc = Vec::new();
    let mut down = Vec::new();
    for axis in axes {
        let r_a = facet_ratio(&facets[0], axis);
        let r_b = facet_ratio(facets.last().expect("run has facets"), axis);
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
