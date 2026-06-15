use crate::{GridConfig, Limits, TopProfile};
use constraints::{BoundaryInfeasibility, BuildOutcome, build_chain};
use nurbs::VectorNurbs;
use scaling::SolverScale;

pub mod chain;
pub mod constraints;
pub(crate) mod follower;
pub(crate) mod output;
pub mod path;
pub mod scaling;
pub(crate) mod solver;
pub use solver::counters;
pub mod stencil;
pub(crate) mod verify;
pub mod window;

pub use constraints::EndpointConditions;
pub use solver::{AxisJerkGradient, axis_jerk_gradient_for_test};

#[non_exhaustive]
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub enum ToleranceMode {
    Tight,
    Fast,
    #[default]
    Auto,
}

#[derive(Debug, thiserror::Error)]
pub enum ScheduleError {
    #[error("invalid endpoint velocity: {0}")]
    InvalidEndpointVelocity(&'static str),
    #[error("invalid endpoint acceleration: {0}")]
    InvalidEndpointAccel(&'static str),
    #[error("path parameterization failed: {0}")]
    PathParam(String),
    #[error("solver setup failed: {0}")]
    SolverSetup(String),
    #[error(
        "follower shaper-folding SLP diverged after {refreezes} refreezes \
         (worst ratio {worst_ratio:.4})"
    )]
    FollowerSlpDiverged { refreezes: u32, worst_ratio: f64 },
}

pub fn schedule_chain_with_tolerance(
    chain: &chain::ChainGrid,
    endpoints: EndpointConditions,
    tolerance: ToleranceMode,
) -> Result<TopProfile, ScheduleError> {
    schedule_chain_with_refreeze_cap(chain, endpoints, tolerance, follower::FOLLOWER_REFREEZE_MAX)
}

pub(crate) fn schedule_chain_with_refreeze_cap(
    chain: &chain::ChainGrid,
    endpoints: EndpointConditions,
    tolerance: ToleranceMode,
    refreeze_max: u32,
) -> Result<TopProfile, ScheduleError> {
    solver::counters::inc_chain_schedule(chain.n_points());

    let v_start = endpoints.v_start;
    let v_end = endpoints.v_end;

    if !v_start.is_finite() || v_start < 0.0 {
        return Err(ScheduleError::InvalidEndpointVelocity(
            "v_start must be finite, ≥ 0",
        ));
    }
    if !v_end.is_finite() || v_end < 0.0 {
        return Err(ScheduleError::InvalidEndpointVelocity(
            "v_end must be finite, ≥ 0",
        ));
    }
    if let Some(a0) = endpoints.a_start {
        if !a0.is_finite() {
            return Err(ScheduleError::InvalidEndpointAccel(
                "a_start must be finite",
            ));
        }
        if v_start == 0.0 {
            return Err(ScheduleError::InvalidEndpointAccel(
                "a_start requires v_start > 0 (pinning accel at rest forces b_1 = 0)",
            ));
        }
    }

    let scale = SolverScale::for_chain(chain);
    let scaled = scale.scale_chain_grid(chain);
    let scaled_endpoints = EndpointConditions {
        v_start: scale.scale_velocity(v_start),
        v_end: scale.scale_velocity(v_end),
        a_start: endpoints.a_start.map(|a| scale.to_scaled_accel(a)),
    };

    let bundle = match build_chain(&scaled, scaled_endpoints, &scale) {
        BuildOutcome::Ok(b) => b,
        BuildOutcome::Boundary(BoundaryInfeasibility::StartAboveMvc { mvc_b }) => {
            return Ok(boundary_infeasible_profile(
                &chain.s,
                crate::BoundarySide::Start,
                scale.unscale_b(mvc_b),
                0,
            ));
        }
        BuildOutcome::Boundary(BoundaryInfeasibility::EndAboveMvc { mvc_b }) => {
            let last = chain.s.len() - 1;
            return Ok(boundary_infeasible_profile(
                &chain.s,
                crate::BoundarySide::End,
                scale.unscale_b(mvc_b),
                last,
            ));
        }
        BuildOutcome::Boundary(BoundaryInfeasibility::EndBelowMinReachable { min_b }) => {
            let last = chain.s.len() - 1;
            return Ok(min_reachable_infeasible_profile(
                &chain.s,
                crate::BoundarySide::End,
                scale.unscale_b(min_b),
                last,
            ));
        }
        BuildOutcome::Boundary(BoundaryInfeasibility::EndAboveMaxReachable { max_b }) => {
            let last = chain.s.len() - 1;
            return Ok(max_reachable_infeasible_profile(
                &chain.s,
                crate::BoundarySide::End,
                scale.unscale_b(max_b),
                last,
            ));
        }
    };

    let call_slp = |tol| {
        if scaled.has_active_windows() {
            solve_with_refreeze(&bundle, &scaled, tol, &scale, refreeze_max)
        } else {
            solver::slp_solve_with_axis_jerk_chain(&bundle, &scaled, None, tol, &scale)
                .map_err(|e| ScheduleError::SolverSetup(format!("{e}")))
        }
    };

    let (mut solver_result, slp_outcome) = match tolerance {
        ToleranceMode::Tight => call_slp(1e-8)?,
        ToleranceMode::Fast => call_slp(1e-5)?,
        ToleranceMode::Auto => {
            let (fast_result, fast_outcome) = call_slp(1e-5)?;
            if solver_outcome_is_success(&fast_result, &fast_outcome) || crate::deadline::expired()
            {
                (fast_result, fast_outcome)
            } else {
                solver::counters::mark_auto_second_pass();
                call_slp(1e-8)?
            }
        }
    };

    scale.unscale_result(&mut solver_result);
    let verify_report = verify::check_chain(chain, &solver_result);

    Ok(output::assemble(
        &chain.s,
        &solver_result,
        &verify_report,
        GridConfig {
            scheme: crate::GridScheme::UniformArclength,
            n: chain.n_points(),
        },
        slp_outcome,
    ))
}

const FOLLOWER_REFREEZE_KM_ALPHA: f64 = 0.7;

fn solve_with_refreeze(
    bundle: &constraints::ConstraintBundle,
    scaled: &chain::ChainGrid,
    tol: f64,
    scale: &SolverScale,
    refreeze_max: u32,
) -> Result<(solver::SolverResult, solver::SlpOutcome), ScheduleError> {
    let (result, _outcome) =
        solver::slp_solve_with_axis_jerk_chain(bundle, scaled, None, tol, scale)
            .map_err(|e| ScheduleError::SolverSetup(format!("{e}")))?;
    let mut b_freeze = result.b.clone();
    let mut last_worst = f64::INFINITY;
    for _refreeze in 0..refreeze_max {
        let windows = follower::build_follower_windows(scaled, &b_freeze);
        let (r2, o2) =
            solver::slp_solve_with_axis_jerk_chain(bundle, scaled, Some(&windows), tol, scale)
                .map_err(|e| ScheduleError::SolverSetup(format!("{e}")))?;
        let drift = follower::refreeze_drift(&windows, &r2.b, scaled);
        last_worst = follower::max_windowed_ratio(&windows, scaled, &r2.b, &r2.a);
        if drift < follower::REFREEZE_DRIFT_TOL && last_worst <= 1.0 + solver::SLP9_EPS_FEAS {
            return Ok((r2, o2));
        }
        if crate::deadline::expired() && last_worst <= 1.0 + solver::SLP9_EPS_FEAS {
            return Ok((r2, o2));
        }
        for (f, n) in b_freeze.iter_mut().zip(&r2.b) {
            *f = (1.0 - FOLLOWER_REFREEZE_KM_ALPHA) * *f + FOLLOWER_REFREEZE_KM_ALPHA * n;
        }
    }
    Err(ScheduleError::FollowerSlpDiverged {
        refreezes: refreeze_max,
        worst_ratio: last_worst,
    })
}

pub fn schedule_segment(
    curve: &VectorNurbs<f64, 3>,
    limits: &Limits,
    grid: &GridConfig,
    v_start: f64,
    v_end: f64,
) -> Result<TopProfile, ScheduleError> {
    schedule_segment_with_tolerance(curve, limits, grid, v_start, v_end, ToleranceMode::Tight)
}

pub fn schedule_segment_with_tolerance(
    curve: &VectorNurbs<f64, 3>,
    limits: &Limits,
    grid: &GridConfig,
    v_start: f64,
    v_end: f64,
    tolerance: ToleranceMode,
) -> Result<TopProfile, ScheduleError> {
    schedule_segment_with_followers(curve, limits, grid, v_start, v_end, tolerance, &[])
}

pub fn schedule_segment_with_followers(
    curve: &VectorNurbs<f64, 3>,
    limits: &Limits,
    grid: &GridConfig,
    v_start: f64,
    v_end: f64,
    tolerance: ToleranceMode,
    followers: &[crate::FollowerDemand],
) -> Result<TopProfile, ScheduleError> {
    if !v_start.is_finite() || v_start < 0.0 {
        return Err(ScheduleError::InvalidEndpointVelocity(
            "v_start must be finite, ≥ 0",
        ));
    }
    if !v_end.is_finite() || v_end < 0.0 {
        return Err(ScheduleError::InvalidEndpointVelocity(
            "v_end must be finite, ≥ 0",
        ));
    }
    if !matches!(grid.scheme, crate::GridScheme::UniformArclength) {
        return Err(ScheduleError::SolverSetup(
            "only GridScheme::UniformArclength is implemented in Step 4".into(),
        ));
    }

    let arc_grid = path::sample_arclength_grid(curve, grid.n)
        .map_err(|e| ScheduleError::PathParam(format!("{e}")))?;

    let chain = chain::ChainGrid::try_from_segment_grids_with_followers(
        vec![arc_grid],
        vec![*limits],
        vec![followers.to_vec()],
        &[false],
    )
    .map_err(|e| ScheduleError::SolverSetup(e.to_string()))?;
    schedule_chain_with_tolerance(
        &chain,
        EndpointConditions {
            v_start,
            v_end,
            a_start: None,
        },
        tolerance,
    )
}

fn solver_outcome_is_success(result: &solver::SolverResult, outcome: &solver::SlpOutcome) -> bool {
    let status_ok = matches!(
        result.status,
        solver::SolverStatus::Solved | solver::SolverStatus::SolvedInexact { .. }
    );
    let outcome_ok = matches!(outcome, solver::SlpOutcome::Converged { .. });
    status_ok && outcome_ok
}

fn boundary_infeasible_profile(
    s: &[f64],
    side: crate::BoundarySide,
    mvc_b: f64,
    at_grid: usize,
) -> TopProfile {
    use crate::{BindingConstraint, BindingSummary, GridSample, InfeasibleReason, SolveStatus};
    let samples = s
        .iter()
        .map(|&si| GridSample {
            s: si,
            v: 0.0,
            a: 0.0,
            b: 0.0,
            binding: BindingConstraint::Boundary,
        })
        .collect();
    TopProfile {
        samples,
        status: SolveStatus::Infeasible {
            at_grid,
            reason: InfeasibleReason::BoundaryAboveMVC { side, mvc_b },
        },
        grid_scheme: crate::GridScheme::UniformArclength,
        total_time: f64::INFINITY,
        binding: BindingSummary::default(),
    }
}

fn max_reachable_infeasible_profile(
    s: &[f64],
    side: crate::BoundarySide,
    max_b: f64,
    at_grid: usize,
) -> TopProfile {
    use crate::{BindingConstraint, BindingSummary, GridSample, InfeasibleReason, SolveStatus};
    let samples = s
        .iter()
        .map(|&si| GridSample {
            s: si,
            v: max_b.max(0.0).sqrt(),
            a: 0.0,
            b: max_b,
            binding: BindingConstraint::Boundary,
        })
        .collect();
    TopProfile {
        samples,
        status: SolveStatus::Infeasible {
            at_grid,
            reason: InfeasibleReason::BoundaryAboveMaxReachable { side, max_b },
        },
        grid_scheme: crate::GridScheme::UniformArclength,
        total_time: f64::INFINITY,
        binding: BindingSummary::default(),
    }
}

fn min_reachable_infeasible_profile(
    s: &[f64],
    side: crate::BoundarySide,
    min_b: f64,
    at_grid: usize,
) -> TopProfile {
    use crate::{BindingConstraint, BindingSummary, GridSample, InfeasibleReason, SolveStatus};
    let samples = s
        .iter()
        .map(|&si| GridSample {
            s: si,
            v: min_b.max(0.0).sqrt(),
            a: 0.0,
            b: min_b,
            binding: BindingConstraint::Boundary,
        })
        .collect();
    TopProfile {
        samples,
        status: SolveStatus::Infeasible {
            at_grid,
            reason: InfeasibleReason::BoundaryBelowMinReachable { side, min_b },
        },
        grid_scheme: crate::GridScheme::UniformArclength,
        total_time: f64::INFINITY,
        binding: BindingSummary::default(),
    }
}

#[cfg(test)]
mod tests;
