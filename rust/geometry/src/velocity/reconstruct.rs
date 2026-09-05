//! Member-local exact reconstruction.
//!
//! The seam plan already carries the exact forward/backward disk-reach sweeps
//! (`disk_reach_v` in both directions), so every member arrives here with a
//! feasible `(entry_v, exit_v)` pair and the time-optimal interior under
//! unlimited jerk is fully local: accelerate on the rail, cruise at the feed
//! ceiling if it binds, brake on the rail to land the exit exactly. `κ(s)` is
//! linear, so the curvature cap is lowest at a member end and the seam plan
//! has already capped both ends; a rail that meets the cap on its way to such
//! an end settles onto it inside the law itself, so no fourth regime exists.
//!
//! Each regime is one exact [`LawSegment`]; a member emits at most three.

use super::disk::VELOCITY_FLOOR;
use super::disk::{Kinematics, RunMember};
use super::law::{LawSegment, ScalarLaw};

const ONSET_BISECT_ITERS: u32 = 60;
/// Slack (relative to speed scale) for seam-plan feasibility checks: an
/// entry/exit pair outside the member's own reach by more than this is not a
/// float residue, it is a planning bug upstream.
const SEAM_SLACK_REL: f64 = 1e-6;

#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) enum ReconstructError {
    /// The seam plan handed a member an entry/exit pair its own disk reach
    /// cannot connect.
    Infeasible {
        member: usize,
        entry_v: f64,
        exit_v: f64,
    },
    /// A law integration degenerated (stall inside a span it should cover).
    Diverged,
}

fn member_law(kin: &Kinematics, local: f64, brake: bool) -> ScalarLaw {
    if kin.is_straight() {
        ScalarLaw::ConstAccel {
            a0: if brake { -kin.accel } else { kin.accel },
        }
    } else {
        ScalarLaw::DiskRail {
            accel: kin.accel,
            kappa0: kin.kappa0 + kin.sigma * local,
            sigma: kin.sigma,
            brake,
        }
    }
}

/// The member's exact profile as at most three law segments in member-local
/// time and arc: the forward rail from the entry, the reversed rail from the
/// exit, each integrated once over the whole member, and the cuts of those
/// two that meet at the onset. Every seam is closed on the interpolant the
/// onset was solved on, so no seam depends on re-integrating a rail over a
/// different grid.
pub(super) fn member_profile(
    idx: usize,
    m: &RunMember,
    entry_v: f64,
    exit_v: f64,
) -> Result<Vec<LawSegment>, ReconstructError> {
    let kin = m.kin;
    let len = kin.length;
    let ceiling = kin.flat_ceiling;
    let slack = SEAM_SLACK_REL * (1.0 + entry_v.max(exit_v));
    let infeasible = || ReconstructError::Infeasible {
        member: idx,
        entry_v,
        exit_v,
    };
    let joint_tol = |v: f64| SEAM_SLACK_REL * (1.0 + v) + 1e-6;
    let reversed = Kinematics {
        length: kin.length,
        accel: kin.accel,
        jerk: kin.jerk,
        kappa0: kin.kappa0 + kin.sigma * kin.length,
        sigma: -kin.sigma,
        flat_ceiling: kin.flat_ceiling,
    };
    let at_ceiling = entry_v >= ceiling * (1.0 - 1e-12);
    let forward = if at_ceiling {
        None
    } else {
        Some(
            LawSegment::until_arc(0.0, 0.0, entry_v, member_law(kin, 0.0, false), len)
                .ok_or(ReconstructError::Diverged)?,
        )
    };
    let backward = LawSegment::until_arc(0.0, 0.0, exit_v, member_law(&reversed, 0.0, false), len)
        .ok_or(ReconstructError::Diverged)?;
    let forward_at = |x: f64| -> Option<f64> {
        let Some(forward) = &forward else {
            return Some(ceiling);
        };
        let t = forward.time_at_distance(x)?;
        Some(forward.state_at(t).1.min(ceiling))
    };
    let backward_at = |x: f64| -> Option<f64> {
        let t = backward.time_at_distance(len - x)?;
        Some(backward.state_at(t).1)
    };
    let fwd_end = forward_at(len).ok_or(ReconstructError::Diverged)?;
    let bwd_start = backward_at(0.0).ok_or(ReconstructError::Diverged)?;
    if exit_v > fwd_end + slack || entry_v > bwd_start + slack {
        return Err(infeasible());
    }

    // Onset: the arc where the forward accelerate/cruise curve meets the
    // backward brake curve. `g` is monotonically increasing with a sign
    // change bracketed by the feasibility checks above. An onset within the
    // seam slack of a member end is integrator noise, not a bang-bang peak:
    // snapped, or it would mint a nanosecond accelerate/brake wedge whose
    // acceleration flip downstream fitters chase as a real feature.
    let g = |x: f64| -> Option<f64> { Some(forward_at(x)? - backward_at(x)?) };
    let onset = if g(len).ok_or(ReconstructError::Diverged)? <= joint_tol(exit_v) {
        len
    } else if g(0.0).ok_or(ReconstructError::Diverged)? >= -joint_tol(entry_v) {
        0.0
    } else {
        let (mut lo, mut hi) = (0.0_f64, len);
        for _ in 0..ONSET_BISECT_ITERS {
            let mid = 0.5 * (lo + hi);
            if g(mid).ok_or(ReconstructError::Diverged)? <= 0.0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        0.5 * (lo + hi)
    };

    let mut out: Vec<LawSegment> = Vec::with_capacity(3);
    fn push(out: &mut Vec<LawSegment>, seg: LawSegment) {
        if seg.dt > 0.0 {
            out.push(seg);
        }
    }

    // The forward rail up to the onset: cut where it first reaches the
    // ceiling when that comes before the onset, else at the onset itself. A
    // contact within a nanometre of the onset is the onset - there is no
    // cruise to start.
    let (accelerate, flat_contact) = match &forward {
        None => (None, 0.0),
        Some(forward) => {
            let contact = forward
                .cut_at_speed(ceiling)
                .filter(|seg| onset - seg.end_distance() >= 1e-9);
            match contact {
                Some(seg) => {
                    let contact = seg.end_distance();
                    (Some(seg), contact)
                }
                None => (
                    Some(
                        forward
                            .cut_at_arc(onset)
                            .ok_or(ReconstructError::Diverged)?,
                    ),
                    onset,
                ),
            }
        }
    };
    let mut t = 0.0_f64;
    let mut v = entry_v;
    if let Some(seg) = accelerate {
        let (_, v_end, _) = seg.end_state();
        t = seg.end_time();
        v = v_end;
        push(&mut out, seg);
    }
    if onset > flat_contact {
        v = ceiling;
        let seg = LawSegment::new(
            t,
            (onset - flat_contact) / v.max(VELOCITY_FLOOR),
            flat_contact,
            v,
            ScalarLaw::ConstAccel { a0: 0.0 },
        );
        t = seg.end_time();
        push(&mut out, seg);
    }
    if onset < len {
        let (seg, v0) = backward
            .flipped_cut(t, onset, member_law(kin, onset, true), len - onset)
            .ok_or(ReconstructError::Diverged)?;
        if (v0 - v).abs() > joint_tol(v) {
            return Err(infeasible());
        }
        push(&mut out, seg);
    }
    if out.is_empty() {
        push(
            &mut out,
            LawSegment::new(
                0.0,
                len / entry_v.max(VELOCITY_FLOOR),
                0.0,
                entry_v,
                ScalarLaw::ConstAccel { a0: 0.0 },
            ),
        );
    }
    Ok(out)
}

#[cfg(test)]
mod tests;
