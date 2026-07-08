use std::sync::Arc;

use crossbeam_channel::{Receiver, Sender};
use geometry::path::lowering::PositionProfile;
use geometry::{Move, SurfaceTransform};
use trajectory::AxisChainSet;

use crate::lowering::{FitTol, lower_move};
use crate::{Control, LoweredItem, LoweredSegment, PlannedItem};

const REST_EPS_MM_S: f64 = 1e-9;

/// Third pipeline stage: lowers each planned move into a dispatchable
/// `ShapedSegment`. It is the persistent owner of the trajectory clock and
/// odometer: `Dwell` advances the clock without motion, `Reset` restarts the
/// timeline at rest at the given position, `SetAxisChains` swaps the chain
/// set future moves are lowered against, `SetMesh` swaps the bed surface
/// transform. The odometer and everything upstream are gcode space; the
/// emitted segments are machine space — this stage owns the warp.
pub fn run_lowerer(
    input: Receiver<PlannedItem>,
    output: Sender<LoweredItem>,
    fit_tol: FitTol,
    mut axis_chains: AxisChainSet,
    home_pos: Vec<f64>,
    t_start: f64,
) {
    let mut odometer = home_pos;
    let mut lower_chains = lowering_chains(&axis_chains);
    let mut t = t_start;
    let mut rest_hold_pending = true;
    let mut mesh: Option<Arc<SurfaceTransform>> = None;

    while let Ok(item) = input.recv() {
        let planned = match item {
            PlannedItem::Move(planned) => planned,
            PlannedItem::Drain => {
                rest_hold_pending = true;
                if output.send(LoweredItem::Drain).is_err() {
                    return;
                }
                continue;
            }
            PlannedItem::Control(ctrl) => {
                match &ctrl {
                    Control::Dwell { secs } => {
                        assert!(*secs >= 0.0, "lowerer: negative dwell {secs}");
                        t += secs;
                    }
                    Control::Reset { pos } => {
                        odometer.clone_from(pos);
                        t = 0.0;
                        rest_hold_pending = true;
                    }
                    Control::SetAxisChains(chains) => {
                        axis_chains = chains.clone();
                        lower_chains = lowering_chains(&axis_chains);
                    }
                    Control::SetMesh {
                        mesh: m,
                        gcode_z_rebase,
                    } => {
                        mesh = m.clone();
                        if let Some(z) = odometer.get_mut(2) {
                            *z = *gcode_z_rebase;
                        }
                    }
                    Control::Nudge { .. } | Control::Barrier(_) => {}
                }
                if output.send(LoweredItem::Control(ctrl)).is_err() {
                    return;
                }
                continue;
            }
        };
        let hold_pad = if rest_hold_pending {
            axis_chains.forward_support()
        } else {
            0.0
        };
        rest_hold_pending = false;
        let clock = crate::timing::stopwatch();
        let mut seg = lower_move(
            &planned.geometry,
            &planned.velocity,
            t + hold_pad,
            &odometer,
            fit_tol,
            &lower_chains,
            mesh.as_deref(),
        )
        .unwrap_or_else(|e| panic!("lowerer: line {}: {e}", planned.geometry.source.start_line));
        seg.source_line = planned.geometry.source.start_line;
        if hold_pad > 0.0 {
            let hold = rest_hold_segment(
                &odometer,
                rest_z_warp(mesh.as_deref(), &odometer),
                t,
                seg.t_start,
                seg.axes.len(),
                seg.source_line,
            );
            if output
                .send(LoweredItem::Seg(LoweredSegment {
                    seg: hold,
                    rest_at_end: true,
                }))
                .is_err()
            {
                return;
            }
        }

        t = seg.t_end;
        advance_odometer(&mut odometer, &planned.geometry);
        tracing::debug!(
            subsystem = "motion",
            event = "pipe_lower",
            line = seg.source_line,
            lower_us = clock.elapsed_us(),
            n_pieces = seg
                .axes
                .iter()
                .map(|a| nurbs::bezier::extract_bezier_pieces(a).len())
                .max()
                .unwrap_or(0),
            t_us = crate::timing::mono_us(),
            "[pipe] lower"
        );

        let rest_at_end = planned.velocity.exit_v <= REST_EPS_MM_S;
        if output
            .send(LoweredItem::Seg(LoweredSegment { seg, rest_at_end }))
            .is_err()
        {
            return;
        }
    }
}

/// The chains the lowerer bakes into raw tracks. A projected follower's whole
/// chain applies in the shaper after re-projection onto its leaders' shaped
/// motion, so its slot is emptied here — baking it against the raw profile
/// would double-apply it.
fn lowering_chains(axis_chains: &AxisChainSet) -> Vec<trajectory::CompiledChain> {
    axis_chains
        .chains
        .iter()
        .enumerate()
        .map(|(axis, chain)| {
            if axis_chains.is_projected_follower(axis) {
                trajectory::CompiledChain::default()
            } else {
                chain.clone()
            }
        })
        .collect()
}

/// The gcode-space odometer is warped like any commanded position when a
/// segment holds it as machine-space output.
fn rest_z_warp(mesh: Option<&SurfaceTransform>, odometer: &[f64]) -> f64 {
    mesh.map_or(0.0, |t| {
        t.correction_at(
            odometer.first().copied().unwrap_or(0.0),
            odometer.get(1).copied().unwrap_or(0.0),
            odometer.get(2).copied().unwrap_or(0.0),
        )
    })
}

/// The rest the shaper's drain flush clamped against, materialized as real
/// trajectory once the next move is known: the shaped output creeps toward
/// the resumed motion inside this window, and that creep must be emitted,
/// not skipped over by a bare clock jump.
fn rest_hold_segment(
    odometer: &[f64],
    z_warp: f64,
    t_start: f64,
    t_end: f64,
    n_axes: usize,
    source_line: u32,
) -> trajectory::ShapedSegment {
    use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs};
    let axes = (0..n_axes)
        .map(|axis| {
            let warp = if axis == 2 { z_warp } else { 0.0 };
            bezier_pieces_to_nurbs(&[BezierPiece {
                u_start: t_start,
                u_end: t_end,
                coeffs: vec![odometer.get(axis).copied().unwrap_or(0.0) + warp],
            }])
        })
        .collect();
    trajectory::ShapedSegment {
        axes,
        followers: Vec::new(),
        spatial_path: false,
        t_start,
        t_end,
        motor_mask: 0,
        source_line,
    }
}

pub fn dist3(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

pub fn advance_odometer(pos: &mut [f64], gm: &Move) {
    let s_len = gm.segment.s_len();
    if let Some(seg) = &gm.segment.spatial {
        let end = seg.point_at(s_len);
        for axis in 0..3.min(pos.len()) {
            pos[axis] = end[axis];
        }
    }
    for f in &gm.segment.followers {
        if let Some(slot) = pos.get_mut(f.axis_index) {
            *slot += f.delta_over(s_len);
        }
    }
}
