use geometry::path::lowering::PositionProfile;
use geometry::{Move, MoveVelocity};
use nurbs::bezier::BezierPiece;
use trajectory::{ChainStage, CompiledChain};

use super::{LoweringError, MIN_PHASE_PIECE_S, pad_to_uniform_degree};

/// Derivative gains are `pos += k1·vel + k2·accel`, exact on a polynomial of
/// any degree: `c′_i = c_i + k1·(i+1)·c_{i+1} + k2·(i+1)·(i+2)·c_{i+2}`
/// (`SmoothKernel` is a downstream convolution, so it stops the per-piece
/// transform exactly as the sampled path does). Mirrors the `ChainStage`
/// semantics in the sampled `axis_state`.
pub(crate) fn apply_derivative_gains(coeffs: &mut [f64], chain: &CompiledChain) {
    for stage in &chain.stages {
        match stage {
            ChainStage::DerivativeGains { k1, k2 } => {
                for i in 0..coeffs.len().saturating_sub(1) {
                    coeffs[i] = k1.mul_add((i + 1) as f64 * coeffs[i + 1], coeffs[i]);
                    if let Some(&c2) = coeffs.get(i + 2) {
                        coeffs[i] = k2.mul_add(((i + 1) * (i + 2)) as f64 * c2, coeffs[i]);
                    }
                }
            }
            ChainStage::SmoothKernel(_) => break,
            ChainStage::NonlinearAdvance(_) => unreachable!(
                "a pre-kernel nonlinear advance routes the move through the \
                 sampled lowering path, which samples the advance law"
            ),
        }
    }
}

/// A nonlinear advance ahead of the kernel is not a per-piece coefficient
/// transform — the advance law is not polynomial in the track — so a move
/// carrying one cannot take the closed-form path.
pub(super) fn has_pre_kernel_nonlinear_advance(chains: &[CompiledChain]) -> bool {
    chains.iter().any(|chain| {
        chain
            .stages
            .iter()
            .take_while(|stage| !matches!(stage, ChainStage::SmoothKernel(_)))
            .any(|stage| matches!(stage, ChainStage::NonlinearAdvance(_)))
    })
}

/// Lower a straight constant-ceiling move from its closed-form jerk phases: one
/// exact cubic per phase. Each axis is the arc-length profile scaled by its fixed
/// share of the motion (a spatial axis by the heading, a follower by its ratio),
/// so position, velocity, acceleration, and jerk are all exact — no grid fitting,
/// no acceleration overshoot, and the traversal time is the profile's own.
pub(super) fn lower_straight_from_phases(
    gm: &Move,
    vm: &MoveVelocity,
    t_start: f64,
    start_pos: &[f64],
    axis_chains: &[CompiledChain],
    z_offset: f64,
) -> Result<(Vec<Vec<BezierPiece>>, f64), LoweringError> {
    let n_axes = start_pos.len().max(3);
    for f in &gm.segment.followers {
        if f.axis_index >= n_axes {
            return Err(LoweringError::FollowerAxisOutOfRange {
                axis_index: f.axis_index,
            });
        }
    }
    let spatial = gm
        .segment
        .spatial
        .as_ref()
        .map(|seg| (seg.point_at(0.0), seg.heading_at(0.0)));

    // The surface warp is one constant offset on this closed-form path (the
    // sampled path handles a Z that varies with XY); it shifts the Z axis's
    // base position without touching its velocity scale.
    let axis_scale_base = |axis: usize| -> (f64, f64) {
        let warp = if axis == 2 { z_offset } else { 0.0 };
        if axis < 3 {
            match spatial {
                Some((origin, heading)) => (heading[axis], origin[axis] + warp),
                None => (0.0, start_pos.get(axis).copied().unwrap_or(0.0) + warp),
            }
        } else if let Some(f) = gm.segment.followers.iter().find(|f| f.axis_index == axis) {
            (f.ratio, start_pos[axis])
        } else {
            (0.0, start_pos.get(axis).copied().unwrap_or(0.0))
        }
    };

    // One shared cumulative-time span list, exactly as the grid path: every
    // axis reads the same breakpoint float, so consecutive pieces are bit-exactly
    // contiguous (`u_end == u_start`) and `t_start + total_t` is the final bound —
    // the contiguity the NURBS assembler asserts, within a move and across seams.
    //
    // A phase shorter than the floor is merged into its neighbor's span rather
    // than emitted: the Bézier round trip reconstructs a piece's monomials with
    // error growing as `ulp(pos)/h²` in acceleration and `ulp(pos)/h³` in jerk,
    // so a nanosecond piece — a jerk ramp across a micro-step between adjacent
    // move ceilings — comes back with garbage derivatives orders of magnitude
    // past the limits. Absorbing forward (extending the host's span past the
    // micro phase) only extrapolates the host's own cubic — an error of the
    // jerk difference over the micro duration cubed. Absorbing backward must
    // rebase the host's coefficients to the earlier span start (`lead`):
    // keeping them as-is time-shifts the whole host early, which puts a
    // `v · lead` position gap at the *next* joint — and the NURBS packing
    // welds joints by control point, turning that gap into a `6·gap/h²`
    // acceleration corruption of whatever short piece follows.
    let total_t: f64 = vm.phases.iter().map(|p| p.dt).sum();
    let mut spans: Vec<(&geometry::StraightPhase, f64, f64, f64)> = Vec::new();
    let mut t_acc = 0.0;
    let mut carry_start: Option<f64> = None;
    for p in &vm.phases {
        let t0 = carry_start.take().unwrap_or(t_acc);
        let lead = t_acc - t0;
        let t1 = t_acc + p.dt;
        t_acc = t1;
        if p.dt <= MIN_PHASE_PIECE_S {
            match spans.last_mut() {
                Some(last) => last.2 = t1,
                None => carry_start = Some(t0),
            }
            continue;
        }
        spans.push((p, t0, t1, lead));
    }
    if spans.is_empty() {
        if let Some(p) = vm.phases.first() {
            spans.push((p, 0.0, total_t, 0.0));
        }
    }

    let mut axes_pieces: Vec<Vec<BezierPiece>> = vec![Vec::new(); n_axes];
    for (axis, pieces) in axes_pieces.iter_mut().enumerate() {
        let (scale, base) = axis_scale_base(axis);
        for &(p, t0, t1, lead) in &spans {
            let (s0, v0, a0) = if lead > 0.0 {
                let b = -lead;
                (
                    p.s0 + b * (p.v0 + b * (0.5 * p.a0 + b * p.j / 6.0)),
                    p.v0 + b * (p.a0 + b * 0.5 * p.j),
                    p.a0 + b * p.j,
                )
            } else {
                (p.s0, p.v0, p.a0)
            };
            let mut coeffs = vec![scale.mul_add(s0, base), scale * v0, scale * 0.5 * a0];
            if p.j != 0.0 {
                coeffs.push(scale * p.j / 6.0);
            }
            if let Some(chain) = axis_chains.get(axis) {
                apply_derivative_gains(&mut coeffs, chain);
            }
            pieces.push(BezierPiece {
                u_start: t_start + t0,
                u_end: t_start + t1,
                coeffs,
            });
        }
    }
    pad_to_uniform_degree(&mut axes_pieces);

    Ok((axes_pieces, total_t))
}
