//! Planning a run of members as one composite instead of seam by seam.
//!
//! A seam-by-seam settlement fixes each member's exit state before the members
//! after it are planned, so every seam is handed the fastest state its
//! predecessor can reach there rather than the state the whole run wants to
//! pass through. Composition puts the seams back inside one solve: consecutive
//! members become the bands of a composite, `curved::composite_edges` places
//! the interior states with the same forward and backward marches a member's
//! own bands get, and each band is then planned between the states that solve
//! chose.
//!
//! Consecutive members that are one clothoid continued — the same curvature at
//! the join, the same rate of turn, which covers a sliced straight and a sliced
//! arc alike — fuse into a single band of their summed length, because the
//! closed form over that length *is* the exact answer their seam can only
//! approximate; the fused band's chain is cut back into its members by
//! [`clip_phases`]. Bands whose caps differ simply keep their own, which is what
//! the band machinery already does.
//!
//! A composite is a candidate, never an imposition: it is adopted only where it
//! takes materially less time than the settled plan it replaces.

use std::ops::Range;

use super::chain::clip_phases;
use super::curved;
use super::disk::{
    Kinematics, MemberPlan, RunMember, closed_form_is_available, curved_solver_is_available,
    member_chain,
};
use super::profile::StraightPhase;

/// Shortfall of a seam's own speed ceiling against what the members either side
/// hold everywhere anyway, relative, below which the seam constrains nothing
/// the composite does not already honour and may become an interior node.
const SEAM_CEILING_SLACK: f64 = 1e-9;

/// One band of a composite and the member ends inside it, ascending from zero.
struct Band {
    kin: Kinematics,
    cuts: Vec<f64>,
}

impl Band {
    fn spans_one_member(&self) -> bool {
        self.cuts.len() == 2
    }
}

fn plannable(kin: &Kinematics) -> bool {
    kin.length > 0.0 && (closed_form_is_available(kin) || curved_solver_is_available(kin))
}

/// Whether the seam between two members constrains anything the composite does
/// not already honour. The bands hold themselves to `top_speed_ceiling`
/// throughout, so a seam ceiling no tighter than those is a node the composite
/// may place freely; a tighter one is a genuine bottleneck the settlement owns.
fn seam_is_absorbable(up: &RunMember, dn: &RunMember) -> bool {
    plannable(up.kin)
        && plannable(dn.kin)
        && up.exit_ceiling
            >= curved::top_speed_ceiling(up.kin).min(curved::top_speed_ceiling(dn.kin))
                * (1.0 - SEAM_CEILING_SLACK)
}

/// Maximal stretches of two or more members whose interior seams are all
/// absorbable.
fn stretches(members: &[RunMember]) -> Vec<Range<usize>> {
    let mut out = Vec::new();
    let mut start = 0usize;
    for k in 1..members.len() {
        if !seam_is_absorbable(&members[k - 1], &members[k]) {
            if k - start >= 2 {
                out.push(start..k);
            }
            start = k;
        }
    }
    if members.len() - start >= 2 {
        out.push(start..members.len());
    }
    out
}

/// How closely the curvature the next member starts at, and the rate it turns
/// at, must continue the band's own for the two to be one clothoid.
const CLOTHOID_JOIN_REL: f64 = 1.0e-9;

/// Whether the next member is this band's own clothoid continued: same
/// curvature at the join, same rate of turn. Then the band's `kappa(s)` extends
/// over the next member's arc unchanged, and the two are one member the solver
/// has simply been handed in pieces.
fn continues(band: &Kinematics, next: &Kinematics) -> bool {
    let kappa_join = band.kappa0 + band.sigma * band.length;
    (next.kappa0 - kappa_join).abs() <= CLOTHOID_JOIN_REL * (1.0 + kappa_join.abs())
        && (next.sigma - band.sigma).abs() <= CLOTHOID_JOIN_REL * (1.0 + band.sigma.abs())
}

/// Two members of one clothoid as a single band: the length they share, and the
/// tightest limit either imposes, so every state of the band is feasible for
/// both.
fn fuse(a: &Kinematics, b: &Kinematics) -> Kinematics {
    Kinematics {
        length: a.length + b.length,
        accel: a.accel.min(b.accel),
        jerk: a.jerk.min(b.jerk),
        kappa0: a.kappa0,
        sigma: a.sigma,
        flat_ceiling: a.flat_ceiling.min(b.flat_ceiling),
    }
}

fn bands_of(members: &[RunMember]) -> Vec<Band> {
    let mut bands: Vec<Band> = Vec::new();
    for m in members {
        let fuses = bands.last().is_some_and(|b| continues(&b.kin, m.kin));
        match bands.last_mut() {
            Some(band) if fuses => {
                band.kin = fuse(&band.kin, m.kin);
                band.cuts.push(band.kin.length);
            }
            _ => bands.push(Band {
                kin: m.kin.clone(),
                cuts: vec![0.0, m.kin.length],
            }),
        }
    }
    bands
}

fn chain_time(chain: &[StraightPhase]) -> f64 {
    chain.iter().map(|p| p.dt).sum()
}

fn end_state(chain: &[StraightPhase], entry: (f64, f64)) -> (f64, f64) {
    chain.last().map_or(entry, |p| {
        let (_, v, a) = p.end_state();
        (v, a)
    })
}

/// Arc-length ends of a stretch's bands, so the settled seam states can be read
/// off the run's own boundary list at the members those ends fall on.
fn band_member_ends(bands: &[Band]) -> Vec<usize> {
    let mut ends = Vec::with_capacity(bands.len() + 1);
    ends.push(0);
    for band in bands {
        ends.push(ends[ends.len() - 1] + band.cuts.len() - 1);
    }
    ends
}

/// Per-member chains of one stretch planned as a composite whose band edges are
/// `edges`, or `None` where a band cannot be closed between them.
fn chains_at_edges(bands: &[Band], edges: &[(f64, f64)]) -> Option<Vec<Vec<StraightPhase>>> {
    let mut out = Vec::new();
    for (i, band) in bands.iter().enumerate() {
        let chain = member_chain(&band.kin, edges[i], edges[i + 1]).ok()?;
        if band.spans_one_member() {
            out.push(chain);
        } else {
            out.extend(clip_phases(&chain, &band.cuts));
        }
    }
    Some(out)
}

/// The quickest composite of one stretch, or `None` where the composite has
/// nothing to add or no edge set closes it.
///
/// Two edge sets are tried. The settled one leaves every band edge where the
/// envelope put it, so what a fused band buys is purely the summed length: two
/// straights planned as one ramp instead of two that each have to land exactly
/// on the mid-ramp state between them. The marched one places the interior
/// states itself, which is what a run whose settled seams are all mid-ramp
/// needs and what a fused band alone cannot reach.
fn composed_chains(
    members: &[RunMember],
    boundary: &[(f64, f64)],
) -> Option<Vec<Vec<StraightPhase>>> {
    let bands = bands_of(members);
    if bands.len() == 1 && bands[0].spans_one_member() {
        return None;
    }
    let kins: Vec<Kinematics> = bands.iter().map(|b| b.kin.clone()).collect();
    let settled: Vec<(f64, f64)> = band_member_ends(&bands)
        .into_iter()
        .map(|m| boundary[m])
        .collect();
    let marched = curved::composite_edges(&kins, settled[0], settled[settled.len() - 1]);
    [settled, marched]
        .into_iter()
        .filter_map(|edges| chains_at_edges(&bands, &edges))
        .min_by(|lhs, rhs| {
            total_time(lhs)
                .partial_cmp(&total_time(rhs))
                .expect("a planned chain has a finite duration")
        })
}

fn total_time(chains: &[Vec<StraightPhase>]) -> f64 {
    chains.iter().map(|c| chain_time(c)).sum()
}

fn settled_time(plans: &[MemberPlan]) -> f64 {
    plans
        .iter()
        .map(|p| {
            p.chain
                .as_ref()
                .map_or(f64::INFINITY, |chain| chain_time(chain))
        })
        .sum()
}

/// Time a composite must save against the settlement, relative, to be worth
/// adopting. A composite re-shapes the profile the lowering then has to fit in
/// cubics, and the shorter a fitted piece the larger the endpoint-derivative
/// residual its own degree truncation leaves — so a win under this is paid for
/// twice over at the wire.
const COMPOSE_TIME_WIN: f64 = 5.0e-3;

/// Replace the settled per-member plans of every composable stretch with the
/// composite's, wherever the composite is materially quicker. A stretch the
/// composite cannot close, or closes no faster, keeps the settlement it had.
pub(super) fn absorb_seams(
    members: &[RunMember],
    boundary: &[(f64, f64)],
    plans: &mut [MemberPlan],
) {
    for range in stretches(members) {
        let stretch_boundary = &boundary[range.start..=range.end];
        let Some(chains) = composed_chains(&members[range.clone()], stretch_boundary) else {
            continue;
        };
        let composed = total_time(&chains);
        assert!(
            composed.is_finite(),
            "composite over members {range:?} planned a chain of non-finite duration"
        );
        if composed >= settled_time(&plans[range.clone()]) * (1.0 - COMPOSE_TIME_WIN) {
            continue;
        }
        let last = range.end - 1;
        let exit = boundary[range.end];
        let mut state = boundary[range.start];
        for (offset, chain) in chains.into_iter().enumerate() {
            let index = range.start + offset;
            let reached = end_state(&chain, state);
            plans[index] = MemberPlan {
                entry: state,
                exit: if index == last { exit } else { reached },
                chain: Ok(chain),
            };
            state = reached;
        }
    }
}

#[cfg(test)]
mod tests;
