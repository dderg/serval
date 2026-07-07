mod ladder;
mod profile;
mod sampled;
mod straight;

#[cfg(test)]
pub(crate) use ladder::FIT_TRUNC_ACC_MM_S2;
pub(crate) use ladder::{
    FIT_TRUNC_POS_FACTOR, LADDER_PROBES_U, ladder_fit, quintic_in_u, truncated_piece,
};

use geometry::{Move, MoveVelocity, SurfaceTransform};
use nurbs::ScalarNurbs;
use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs};
#[cfg(test)]
pub(crate) use straight::apply_pressure_advance;
#[cfg(test)]
use trajectory::ChainStage;
use trajectory::{CompiledChain, ShapedSegment};

use profile::build_profile;
use sampled::{Sampler, ZWarp, refine_span, regime_knot_times, z_warp_mode};
use straight::lower_straight_from_phases;

/// Duplicated from `runtime::piece_ring::MAX_PIECE_COEFFS` (this crate must
/// not depend on the MCU runtime); equality is enforced by the cross-crate
/// const test in motion-engine.
pub const MAX_PIECE_COEFFS: usize = 8;

const MIN_PIECE_DURATION_S: f64 = 1e-9;
/// Below this span the Bézier round trip corrupts a piece's acceleration
/// (error ~ `ulp(pos)/h²` ≈ 1.4 mm/s² at 100 mm and 100 ns) and jerk; phases
/// this short merge into a neighbor instead of lowering to their own piece.
const MIN_PHASE_PIECE_S: f64 = 1e-7;
const MAX_SUBDIVISION_DEPTH: u32 = 22;

const MIN_FIT_PIECE_S: f64 = 1e-4;

/// Fit acceptance budgets for one lowered piece, probed at the interior
/// Chebyshev nodes (endpoints are matched exactly by construction).
#[derive(Debug, Clone, Copy)]
pub struct FitTol {
    pub pos_mm: f64,
    /// Absolute acceleration-error budget for a fitted piece. The positional
    /// weighting alone (`accel_err·h²/8`) lets a short piece end with an
    /// acceleration hundreds of mm/s² off the profile — positionally
    /// invisible, but adjacent pieces then step by twice that error at their
    /// shared knot, and the dispatched trajectory carries a jerk spike the
    /// planner never asked for. Only accel-feedforward consumers (EtherCAT
    /// servos) can tell; steppers just follow positions.
    pub accel_mm_s2: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoweringError {
    SourceMismatch,
    EmptyProfile,
    DegeneratePhase,
    FollowerAxisOutOfRange { axis_index: usize },
}

impl std::fmt::Display for LoweringError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::SourceMismatch => write!(f, "geometry and velocity move sources differ"),
            Self::EmptyProfile => write!(f, "velocity move has fewer than two samples"),
            Self::DegeneratePhase => write!(f, "velocity profile phase is degenerate"),
            Self::FollowerAxisOutOfRange { axis_index } => {
                write!(
                    f,
                    "follower axis {axis_index} exceeds the registry start vector"
                )
            }
        }
    }
}

impl std::error::Error for LoweringError {}

/// Zero-pad every piece of every axis to the move's maximum degree — both
/// `bezier_pieces_to_nurbs` (uniform degree per curve) and `lane_curve`'s
/// cross-axis addition for CoreXY mixing require it. Enqueue's Chebyshev
/// truncation recovers each piece's true degree at the wire.
fn pad_to_uniform_degree(axes_pieces: &mut [Vec<BezierPiece>]) {
    let max_len = axes_pieces
        .iter()
        .flatten()
        .map(|p| p.coeffs.len())
        .max()
        .unwrap_or(1);
    assert!(
        max_len <= MAX_PIECE_COEFFS,
        "fitted piece degree {} exceeds the wire maximum",
        max_len - 1
    );
    for piece in axes_pieces.iter_mut().flatten() {
        piece.coeffs.resize(max_len, 0.0);
    }
}

pub fn lower_move(
    gm: &Move,
    vm: &MoveVelocity,
    t_start: f64,
    start_pos: &[f64],
    fit_tol: FitTol,
    axis_chains: &[CompiledChain],
    mesh: Option<&SurfaceTransform>,
) -> Result<ShapedSegment, LoweringError> {
    let (axes_pieces, total_t) =
        lower_move_pieces(gm, vm, t_start, start_pos, fit_tol, axis_chains, mesh)?;
    let axes: Vec<ScalarNurbs> = axes_pieces
        .iter()
        .map(|p| bezier_pieces_to_nurbs(p))
        .collect();
    Ok(ShapedSegment {
        axes,
        followers: gm.segment.followers.clone(),
        t_start,
        t_end: t_start + total_t,
        motor_mask: 0,
        source_line: 0,
    })
}

/// Lower a move to per-axis cubic Bézier pieces in monomial form
/// (`pos = c0 + c1·τ + c2·τ² + c3·τ³`, `τ = t − u_start`) — the trajectory the
/// firmware executes, before it is packed into NURBS. Returns the per-axis pieces
/// and the move duration.
pub fn lower_move_pieces(
    gm: &Move,
    vm: &MoveVelocity,
    t_start: f64,
    start_pos: &[f64],
    fit_tol: FitTol,
    axis_chains: &[CompiledChain],
    mesh: Option<&SurfaceTransform>,
) -> Result<(Vec<Vec<BezierPiece>>, f64), LoweringError> {
    if gm.source != vm.source {
        return Err(LoweringError::SourceMismatch);
    }
    let z_warp = z_warp_mode(mesh, gm, start_pos);
    // The closed-form phase path expresses each axis as one constant scale times
    // the arc-length profile; a ramped follower's ratio varies along the move
    // and a surface-warped Z varies with XY — so route those through the sampled
    // exact phase path, as does a warp flat enough to be one constant offset.
    let ramped = gm.segment.followers.iter().any(|f| f.is_ramped());
    if !vm.phases.is_empty() && !ramped && !matches!(z_warp, ZWarp::Surface(_)) {
        let z_offset = match z_warp {
            ZWarp::Constant(c) => c,
            _ => 0.0,
        };
        return lower_straight_from_phases(gm, vm, t_start, start_pos, axis_chains, z_offset);
    }
    let (profile, total_t) = build_profile(&vm.samples)?;
    // A jerk-regime change is a curvature kink in the arc-length profile: no
    // polynomial spans it within the acceleration budget, so blind bisection
    // cascades down to the floor around each one. Pinning a grid knot at every
    // regime boundary lets both sides fit their full smooth spans instead.
    let mut knots = regime_knot_times(&profile, fit_tol);
    knots.retain(|&t| t > MIN_PIECE_DURATION_S && total_t - t > MIN_PIECE_DURATION_S);
    let spatial = gm.segment.spatial.as_ref();

    let n_axes = start_pos.len().max(3);
    for f in &gm.segment.followers {
        if f.axis_index >= n_axes {
            return Err(LoweringError::FollowerAxisOutOfRange {
                axis_index: f.axis_index,
            });
        }
    }

    let sampler = Sampler {
        profile: &profile,
        spatial,
        start_pos,
        followers: &gm.segment.followers,
        s_len: gm.segment.s_len(),
        axis_chains,
        z_warp,
    };
    // Each axis refines its own grid: an axis only pays for the knots its own
    // signal needs (a follower is blind to path curvature, z to planar motion),
    // and the guarantee is unchanged — every piece of every axis is probed
    // against the same tolerances.
    let mut axes_pieces: Vec<Vec<BezierPiece>> = vec![Vec::new(); n_axes];
    let no_knots: Vec<f64> = Vec::new();
    for (axis, pieces) in axes_pieces.iter_mut().enumerate() {
        let driven = [axis];
        // A follower sees the profile's jerk edges scaled by its ratio, far
        // below the acceleration budget — seeding them would only cut its
        // near-linear track into spatial-sized pieces.
        let axis_knots = if axis < 3 { &knots } else { &no_knots };
        let mut bounds = vec![0.0];
        let mut prev = 0.0;
        for &k in axis_knots.iter().chain(std::iter::once(&total_t)) {
            if k - prev > MIN_PIECE_DURATION_S {
                refine_span(&sampler, &driven, fit_tol, prev, k, 0, &mut bounds);
                prev = k;
            }
        }
        for w in bounds.windows(2) {
            let (ta, tb) = (w[0], w[1]);
            if tb - ta <= MIN_PIECE_DURATION_S {
                continue;
            }
            let (ua, ub) = (t_start + ta, t_start + tb);
            pieces.push(sampler.fitted_piece(axis, ta, tb, ua, ub, fit_tol));
        }
    }
    pad_to_uniform_degree(&mut axes_pieces);

    Ok((axes_pieces, total_t))
}

#[cfg(test)]
mod tests;
