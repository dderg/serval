//! Member-local exact reconstruction.
//!
//! The seam plan already carries the exact forward/backward disk-reach sweeps
//! (`disk_reach_v` in both directions), so every member arrives here with a
//! feasible `(entry_v, exit_v)` pair and the time-optimal interior under
//! unlimited jerk is fully local: accelerate on the rail, cruise at the feed
//! ceiling if it binds, brake on the rail to land the exit exactly. Curvature
//! caps never bind in a member's interior — `κ(s)` is linear, so `|κ|` peaks
//! at a member end and the seam plan has already capped both ends.
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

/// Speed of the accelerate-then-cruise profile from `(0, v0)` at local arc
/// `x`, capped by the member's feed ceiling.
fn forward_speed(kin: &Kinematics, v0: f64, x: f64) -> Option<f64> {
    if v0 >= kin.flat_ceiling {
        return Some(kin.flat_ceiling);
    }
    let v = LawSegment::reach_over(member_law(kin, 0.0, false), v0, x)?;
    Some(v.min(kin.flat_ceiling))
}

/// Entry speed at local arc `x` of the brake that lands `v_end` at the member
/// end.
fn backward_speed(kin: &Kinematics, v_end: f64, x: f64) -> Option<f64> {
    let reversed_local = 0.0;
    let rev = Kinematics {
        length: kin.length,
        accel: kin.accel,
        jerk: kin.jerk,
        kappa0: kin.kappa0 + kin.sigma * kin.length,
        sigma: -kin.sigma,
        flat_ceiling: kin.flat_ceiling,
    };
    LawSegment::reach_over(
        member_law(&rev, reversed_local, false),
        v_end,
        kin.length - x,
    )
}

/// The member's exact profile as at most three law segments in member-local
/// time and arc, plus the exit state.
pub(super) fn member_profile(
    idx: usize,
    m: &RunMember,
    entry_v: f64,
    exit_v: f64,
) -> Result<Vec<LawSegment>, ReconstructError> {
    let kin = m.kin;
    let len = kin.length;
    let slack = SEAM_SLACK_REL * (1.0 + entry_v.max(exit_v));
    let infeasible = || ReconstructError::Infeasible {
        member: idx,
        entry_v,
        exit_v,
    };
    let reversed = Kinematics {
        length: kin.length,
        accel: kin.accel,
        jerk: kin.jerk,
        kappa0: kin.kappa0 + kin.sigma * kin.length,
        sigma: -kin.sigma,
        flat_ceiling: kin.flat_ceiling,
    };
    let forward_profile = if kin.is_straight() || entry_v >= kin.flat_ceiling {
        None
    } else {
        Some(
            LawSegment::until_arc(0.0, 0.0, entry_v, member_law(kin, 0.0, false), len)
                .ok_or(ReconstructError::Diverged)?,
        )
    };
    let backward_profile = if kin.is_straight() {
        None
    } else {
        Some(
            LawSegment::until_arc(0.0, 0.0, exit_v, member_law(&reversed, 0.0, false), len)
                .ok_or(ReconstructError::Diverged)?,
        )
    };
    let forward_at = |x: f64| {
        if entry_v >= kin.flat_ceiling {
            return Some(kin.flat_ceiling);
        }
        match &forward_profile {
            Some(profile) => profile
                .time_at_distance(x)
                .map(|t| profile.state_at(t).1.min(kin.flat_ceiling)),
            None => forward_speed(kin, entry_v, x),
        }
    };
    let backward_at = |x: f64| match &backward_profile {
        Some(profile) => profile
            .time_at_distance(len - x)
            .map(|t| profile.state_at(t).1),
        None => backward_speed(kin, exit_v, x),
    };
    let fwd_end = forward_at(len).ok_or(ReconstructError::Diverged)?;
    let bwd_start = backward_at(0.0).ok_or(ReconstructError::Diverged)?;
    if exit_v > fwd_end + slack || entry_v > bwd_start + slack {
        return Err(infeasible());
    }

    let mut out: Vec<LawSegment> = Vec::with_capacity(3);
    fn push(out: &mut Vec<LawSegment>, seg: LawSegment) {
        if seg.dt > 0.0 {
            out.push(seg);
        }
    }

    // Onset: the arc where the forward accelerate/cruise curve meets the
    // backward brake curve. `g` is monotonically increasing with a sign
    // change bracketed by the feasibility checks above. An onset within the
    // seam slack of a member end is integrator noise, not a bang-bang peak:
    // snapped, or it would mint a nanosecond accelerate/brake wedge whose
    // acceleration flip downstream fitters chase as a real feature.
    let joint_tol = |v: f64| SEAM_SLACK_REL * (1.0 + v) + 1e-6;
    let g = |x: f64| -> Option<f64> { Some(forward_at(x)? - backward_at(x)?) };
    let onset = if g(len).ok_or(ReconstructError::Diverged)? <= 0.0 {
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
        let mut solved = 0.5 * (lo + hi);
        if !kin.is_straight() {
            let exact_g = |x: f64| -> Option<f64> {
                Some(forward_speed(kin, entry_v, x)? - backward_speed(kin, exit_v, x)?)
            };
            let candidate_forward =
                forward_speed(kin, entry_v, solved).ok_or(ReconstructError::Diverged)?;
            let candidate_backward =
                backward_speed(kin, exit_v, solved).ok_or(ReconstructError::Diverged)?;
            let candidate_residual = candidate_forward - candidate_backward;
            if candidate_residual.abs() > 0.25 * joint_tol(candidate_forward) {
                let radius = 1e-3 * (1.0 + len);
                let local_lo = (solved - radius).max(0.0);
                let local_hi = (solved + radius).min(len);
                let local_brackets = exact_g(local_lo).ok_or(ReconstructError::Diverged)? <= 0.0
                    && exact_g(local_hi).ok_or(ReconstructError::Diverged)? >= 0.0;
                let (mut exact_lo, mut exact_hi) = if local_brackets {
                    (local_lo, local_hi)
                } else {
                    (0.0, len)
                };
                // The onset is solved in arc but consumed as a speed seam, and
                // `dv/ds = a/v` converts one into the other: at a high budget
                // and a low speed an arc bracket that looks closed still leaves
                // the seam speeds a slack apart.
                let onset_arc_tol =
                    0.25 * joint_tol(candidate_forward) * candidate_forward.max(VELOCITY_FLOOR)
                        / kin.accel;
                for _ in 0..ONSET_BISECT_ITERS {
                    if exact_hi - exact_lo <= onset_arc_tol {
                        break;
                    }
                    let mid = 0.5 * (exact_lo + exact_hi);
                    if exact_g(mid).ok_or(ReconstructError::Diverged)? <= 0.0 {
                        exact_lo = mid;
                    } else {
                        exact_hi = mid;
                    }
                }
                solved = 0.5 * (exact_lo + exact_hi);
            }
        }
        if g(len).ok_or(ReconstructError::Diverged)? <= joint_tol(exit_v) {
            len
        } else {
            solved
        }
    };

    // Flat contact: where the accelerating rail reaches the feed ceiling
    // (before the onset, else there is no cruise).
    let flat_contact = if entry_v >= kin.flat_ceiling * (1.0 - 1e-12) {
        0.0
    } else if forward_at(onset).ok_or(ReconstructError::Diverged)?
        >= kin.flat_ceiling * (1.0 - 1e-12)
    {
        let reach = &forward_at;
        let (mut lo, mut hi) = (0.0_f64, onset);
        for _ in 0..ONSET_BISECT_ITERS {
            let mid = 0.5 * (lo + hi);
            if reach(mid).ok_or(ReconstructError::Diverged)? < kin.flat_ceiling {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let contact = 0.5 * (lo + hi);
        if onset - contact < 1e-9 {
            onset
        } else {
            contact
        }
    } else {
        onset
    };

    let mut t = 0.0_f64;
    let mut v = entry_v;
    let forward_gain =
        forward_speed(kin, entry_v, onset).ok_or(ReconstructError::Diverged)? - entry_v;
    if onset > 0.0 && forward_gain <= joint_tol(entry_v) {
        let seg = LawSegment::new(
            0.0,
            onset / entry_v.max(VELOCITY_FLOOR),
            0.0,
            entry_v,
            ScalarLaw::ConstAccel { a0: 0.0 },
        );
        t = seg.end_time();
        push(&mut out, seg);
        if onset < len {
            let (seg, v0) =
                LawSegment::brake_to(t, onset, member_law(kin, onset, true), len - onset, exit_v)
                    .ok_or(ReconstructError::Diverged)?;
            if (v0 - v).abs() > joint_tol(v) {
                return Err(infeasible());
            }
            push(&mut out, seg);
        }
        return Ok(out);
    }
    if flat_contact > 0.0 {
        let seg = LawSegment::until_arc(t, 0.0, v, member_law(kin, 0.0, false), flat_contact)
            .ok_or(ReconstructError::Diverged)?;
        let (_, v_end, _) = seg.end_state();
        t = seg.end_time();
        v = v_end;
        push(&mut out, seg);
    }
    if onset > flat_contact {
        v = kin.flat_ceiling;
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
        let (seg, v0) =
            LawSegment::brake_to(t, onset, member_law(kin, onset, true), len - onset, exit_v)
                .ok_or(ReconstructError::Diverged)?;
        if (v0 - v).abs() > SEAM_SLACK_REL * (1.0 + v) + 1e-6 {
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
