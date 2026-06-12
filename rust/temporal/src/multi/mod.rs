use crate::{Limits, TopProfile};
use nurbs::VectorNurbs;
use thiserror::Error;

#[non_exhaustive]
#[derive(Debug, Clone, Copy)]
pub enum GridStrategy {
    Fixed(usize),
    Adaptive {
        min_n: usize,
        max_n: usize,
        target_grid_spacing_mm: f64,
    },
}

#[derive(Debug, Clone, Copy)]
pub struct SegmentInput<'a> {
    pub curve: &'a VectorNurbs<f64, 3>,
    pub limits: Limits,
    pub followers: &'a [crate::FollowerDemand],
    /// `Some(length)` marks a follower-only move planned on a virtual path of
    /// this arclength; `curve` has zero displacement and the follower rows do
    /// all the limiting.
    pub virtual_path: Option<f64>,
}

#[derive(Debug, Clone, Copy)]
pub struct BatchInput<'a> {
    pub segments: &'a [SegmentInput<'a>],
    /// Input-shaper kernels + pre-batch follower history; `None` = no shaper
    /// folding anywhere in the batch.
    pub shaping: Option<&'a BatchShaping>,
    pub grid_strategy: GridStrategy,
    pub worker_threads: usize,
    pub initial_velocity: f64,
    /// Path accel at the batch start. Pinned in the SOCP only when `initial_velocity > 0`;
    /// at a rest start it MUST be 0.0 (asserted) and the rest envelope governs.
    pub initial_accel: f64,
    pub terminal_velocity: f64,
}

#[derive(Debug, Clone, Default)]
pub struct BatchShaping {
    pub axis_kernels: [Option<nurbs::algebra::PiecewisePolynomialKernel<f64>>; 3],
    pub follower_history: Option<crate::FollowerHistory>,
}

#[derive(Debug)]
pub struct BatchOutput {
    pub profiles: Vec<TopProfile>,
    pub junctions: Vec<JunctionInfo>,
    pub joining_sweeps: u32,
    pub joining_status: JoiningStatus,
}

#[non_exhaustive]
#[derive(Debug, Clone, Copy)]
pub enum JoiningStatus {
    Converged,
    StalledOnInfeasibleSegment { last_dirty_count: usize },
    CappedAtMaxSweeps { last_dirty_count: usize },
}

#[derive(Debug, Clone, Copy)]
pub struct JunctionInfo {
    pub between_segments: (usize, usize),
    pub v_junction: f64,
}

#[derive(Debug, Error)]
pub enum BatchError {
    #[error("empty segment buffer")]
    EmptySegments,
    #[error("worker_threads must be ≥ 1")]
    InvalidThreads,
    #[error("segment {0}: {1}")]
    Segment(usize, crate::topp::ScheduleError),
    #[error("invalid follower demand: {0}")]
    InvalidFollowerDemand(String),
    #[error(
        "follower tail exchange did not settle after {passes} passes \
         (chain {chain} total time still moving {rel_change:.4})"
    )]
    TailExchangeDiverged {
        passes: u32,
        chain: usize,
        rel_change: f64,
    },
}

fn virtual_grid_n(strategy: &GridStrategy, length: f64) -> usize {
    match *strategy {
        GridStrategy::Fixed(n) => n.max(2),
        GridStrategy::Adaptive {
            min_n,
            max_n,
            target_grid_spacing_mm,
        } => {
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            let n = (length / target_grid_spacing_mm).ceil() as usize;
            n.clamp(min_n, max_n).max(2)
        }
    }
}

pub fn plan_batch(input: BatchInput<'_>) -> Result<BatchOutput, BatchError> {
    use crate::multi::{chain, grid, joining, junction, parallel};
    use crate::topp::chain::ChainGrid;
    use crate::topp::path::sample_arclength_grid;
    use nurbs::VectorNurbs;

    if input.segments.is_empty() {
        return Err(BatchError::EmptySegments);
    }
    if input.worker_threads == 0 {
        return Err(BatchError::InvalidThreads);
    }
    assert!(
        input.initial_velocity > 0.0 || input.initial_accel == 0.0,
        "rest start requires initial_accel == 0"
    );

    let k = input.segments.len();

    let kinds: Vec<junction::JunctionKind> = (0..k - 1)
        .map(|i| {
            junction::classify_junction_curves(input.segments[i].curve, input.segments[i + 1].curve)
        })
        .collect();
    let chain_ranges = chain::partition_chains(k, &kinds);
    let n_chains = chain_ranges.len();

    let grid_max_n = match input.grid_strategy {
        GridStrategy::Adaptive { max_n, .. } => Some(max_n),
        GridStrategy::Fixed(_) => None,
    };

    let chain_grids: Vec<ChainGrid> = chain_ranges
        .iter()
        .map(|range| {
            if let Some(length) = input.segments[*range.start()].virtual_path {
                assert_eq!(
                    range.clone().count(),
                    1,
                    "virtual-path segment must be isolated in its own chain — \
                     zero tangents classify both junctions as corners"
                );
                let seg = &input.segments[*range.start()];
                let n = virtual_grid_n(&input.grid_strategy, length);
                return ChainGrid::virtual_path(length, n, seg.limits, seg.followers.to_vec(), 0.0)
                    .map_err(|e| BatchError::InvalidFollowerDemand(e.to_string()));
            }
            assert!(
                range
                    .clone()
                    .all(|i| input.segments[i].virtual_path.is_none()),
                "virtual-path segment fused into a multi-segment chain — \
                 junction classification must isolate zero-displacement curves"
            );
            let chain_curves: Vec<&VectorNurbs<f64, 3>> =
                range.clone().map(|i| input.segments[i].curve).collect();
            let mut ns: Vec<usize> = chain_curves
                .iter()
                .map(|c| grid::compute_n(&input.grid_strategy, c))
                .collect();
            let absorbed = grid::classify_absorbed(&ns, &chain_curves, grid_max_n);
            grid::reconcile_junction_n(&mut ns, &chain_curves, grid_max_n, &absorbed);
            for (n, &a) in ns.iter_mut().zip(&absorbed) {
                if a {
                    *n = 2;
                }
            }
            let seg_grids: Result<Vec<_>, _> = range
                .clone()
                .zip(ns)
                .map(|(seg_idx, n)| {
                    sample_arclength_grid(input.segments[seg_idx].curve, n).map_err(|e| {
                        BatchError::Segment(
                            seg_idx,
                            crate::topp::ScheduleError::PathParam(format!("{e}")),
                        )
                    })
                })
                .collect();
            let seg_limits: Vec<_> = range.clone().map(|i| input.segments[i].limits).collect();
            let seg_followers: Vec<_> = range
                .clone()
                .map(|i| input.segments[i].followers.to_vec())
                .collect();
            seg_grids.and_then(|grids| {
                ChainGrid::try_from_segment_grids_with_followers(
                    grids,
                    seg_limits,
                    seg_followers,
                    &absorbed,
                )
                .map_err(|e| BatchError::InvalidFollowerDemand(e.to_string()))
            })
        })
        .collect::<Result<Vec<_>, _>>()?;

    let mut chain_grids = chain_grids;
    if let Some(shaping) = input.shaping {
        for (c, cg) in chain_grids.iter_mut().enumerate() {
            cg.axis_kernels = shaping.axis_kernels.clone();
            if c == 0 {
                cg.follower_history = shaping.follower_history.clone();
            }
        }
    }

    let mut states: Vec<joining::ChainState> = chain_ranges
        .iter()
        .enumerate()
        .map(|(c, _range)| {
            let v_start = if c == 0 { input.initial_velocity } else { 0.0 };
            let v_end = if c + 1 == n_chains {
                input.terminal_velocity
            } else {
                0.0
            };
            let a_start = if c == 0 && input.initial_velocity > 0.0 {
                Some(input.initial_accel)
            } else {
                None
            };
            joining::ChainState {
                v_start,
                v_end,
                a_start,
                profile: None,
                dirty: true,
            }
        })
        .collect();

    parallel::fan_out_solves(&chain_grids, &mut states, input.worker_threads)?;

    let corner_caps: Vec<f64> = vec![0.0; n_chains.saturating_sub(1)];

    let (sweeps, joining_status) = joining::join_until_converged(
        &chain_grids,
        &mut states,
        &corner_caps,
        input.worker_threads,
    )?;

    joining::exchange_follower_tails(&mut chain_grids, &mut states, input.worker_threads)?;

    // Slice each chain profile into per-segment profiles and flatten.
    let profiles: Vec<TopProfile> = states
        .into_iter()
        .zip(chain_grids.iter())
        .flat_map(|(state, cg)| {
            let chain_profile = state.profile.expect("all chain profiles solved by stage 5");
            chain::slice_chain_profile(&chain_profile, &cg.segment_ranges)
        })
        .collect();

    let junction_infos: Vec<JunctionInfo> = (0..k - 1)
        .map(|j| JunctionInfo {
            between_segments: (j, j + 1),
            v_junction: profiles[j].samples.last().map_or(0.0, |s| s.v),
        })
        .collect();

    Ok(BatchOutput {
        profiles,
        junctions: junction_infos,
        joining_sweeps: sweeps,
        joining_status,
    })
}

#[cfg(test)]
mod tests;

mod chain;
mod grid;
mod joining;
mod junction;
mod parallel;
