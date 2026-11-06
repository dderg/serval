use geometry::{ExtruderCaps, LeaderSpill, RestSettle, SPATIAL_AXIS_SETS};
use nurbs::algebra::PiecewisePolynomialKernel;
use trajectory::{AxisChainSet, ChainStage};

use crate::AxisRegistry;

const SPATIAL_AXES: usize = 3;

/// How far one acceleration switch steps the planned acceleration, per
/// unit of budget: from braking at the budget to accelerating at it.
const SWITCH_STEP_PER_BUDGET: f64 = 2.0;

/// The shaped path's jerk gains over the set of spatial axes `set` through
/// the kernels of the shaped leaders on them, per unit of planned
/// acceleration and per unit of budget a lone switch steps, which never
/// asks more than the variation; 0 where none is.
fn path_jerk_gains(shaped: &[(usize, &PiecewisePolynomialKernel)], set: usize) -> (f64, f64) {
    let kernels: Vec<&PiecewisePolynomialKernel> = shaped
        .iter()
        .filter(|(axis, _)| set & (1 << axis) != 0)
        .map(|(_, kernel)| *kernel)
        .collect();
    if kernels.is_empty() {
        return (0.0, 0.0);
    }
    let variation = PiecewisePolynomialKernel::directional_variation(&kernels);
    let switch = SWITCH_STEP_PER_BUDGET * PiecewisePolynomialKernel::directional_peak(&kernels);
    (variation, switch.min(variation))
}

/// The [`ExtruderCaps`] of every follower axis with leaders and a limit
/// (`[axis] max_velocity` / `max_accel`, set or inherited from the extruder's
/// extrude-only caps), from its chain as compiled: the advance's slope and
/// curvature bounds, the follower's own kernel, and its spatial leaders',
/// whose settle onto a rest point an extrude-only move waits for to within
/// its share of `max_path_deviation_mm`.
pub fn extruder_caps(
    registry: &AxisRegistry,
    chains: &AxisChainSet,
    max_path_deviation_mm: f64,
) -> Result<Vec<ExtruderCaps>, String> {
    let mut caps = Vec::new();
    for (axis, leaders) in &chains.followers {
        let decl = &registry.decls()[*axis];
        if leaders.is_empty() || (decl.max_velocity.is_none() && decl.max_accel.is_none()) {
            continue;
        }
        let chain = &chains.chains[*axis];
        let (advance_slope_s, advance_curvature_s2_per_mm) = match chain.transform() {
            None => (0.0, 0.0),
            Some(ChainStage::DerivativeGains { k1, k2 }) => {
                if *k2 != 0.0 {
                    return Err(format!(
                        "axis '{}' has limits and an acceleration gain (k2 = {k2}): the planner \
                         bounds an advance's velocity gain only; drop the axis's max_velocity / \
                         max_accel or the acceleration gain",
                        decl.name
                    ));
                }
                (k1.abs(), 0.0)
            }
            Some(ChainStage::NonlinearAdvance(advance)) => {
                (advance.max_slope(), advance.max_curvature())
            }
            Some(ChainStage::SmoothKernel(_)) => {
                unreachable!("a chain's transform is never its kernel")
            }
        };
        let mut shaped = Vec::new();
        let mut unshaped_leaders = [false; SPATIAL_AXES];
        for &leader in leaders.iter().filter(|&&leader| leader < SPATIAL_AXES) {
            match chains.chains[leader].kernel() {
                Some(kernel) => shaped.push((leader, kernel)),
                None => unshaped_leaders[leader] = true,
            }
        }
        let leader_kernels: Vec<&PiecewisePolynomialKernel> =
            shaped.iter().map(|(_, kernel)| *kernel).collect();
        let leader_spill = LeaderSpill::of_kernels(&leader_kernels);
        let (leaders_back, leaders_forward) = chains.leaders_support(*axis);
        let gains: [(f64, f64); SPATIAL_AXIS_SETS] =
            std::array::from_fn(|set| path_jerk_gains(&shaped, set));
        caps.push(ExtruderCaps {
            axis_index: *axis,
            max_velocity_mm_s: decl.max_velocity.unwrap_or(f64::INFINITY),
            max_accel_mm_s2: decl.max_accel.unwrap_or(f64::INFINITY),
            advance_slope_s,
            advance_curvature_s2_per_mm,
            path_jerk_per_accel_hz: gains.map(|(variation, _)| variation),
            path_jerk_per_accel_switch_hz: gains.map(|(_, switch)| switch),
            unshaped_leaders,
            own_jerk_per_accel_hz: chain.kernel_jerk_per_accel_hz(),
            instantaneous_corner_velocity_mm_s: chain.instantaneous_corner_velocity(),
            leader_spill,
            leader_kernel_reach_s: leaders_forward.max(-leaders_back),
            rest_settle: RestSettle::of_kernels(&leader_kernels, max_path_deviation_mm),
        });
    }
    Ok(caps)
}

#[cfg(test)]
mod tests;
