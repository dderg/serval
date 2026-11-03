use geometry::ExtruderCaps;
use trajectory::{AxisChainSet, ChainStage};

use crate::AxisRegistry;

const SPATIAL_AXES: usize = 3;

/// One fitted-path jerk gain per unit of planned acceleration through a
/// kernel `w`: a single step in the acceleration under the disk, at most
/// `2A`, leaves `w` as a jerk of at most `2A·max|w|`. Without a kernel the
/// step passes unsmoothed.
fn jerk_per_accel(kernel_peak: Option<f64>) -> f64 {
    kernel_peak.map_or(f64::INFINITY, |peak| 2.0 * peak)
}

/// The [`ExtruderCaps`] of every follower axis with leaders and a limit
/// (`[axis] max_velocity` / `max_accel`, set or inherited from the extruder's
/// extrude-only caps), from its chain as compiled: the advance's slope and
/// curvature bounds, the follower's own kernel, and its spatial leaders'.
pub fn extruder_caps(
    registry: &AxisRegistry,
    chains: &AxisChainSet,
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
        let mut path_jerk_per_accel_hz = [0.0; SPATIAL_AXES];
        for &leader in leaders.iter().filter(|&&leader| leader < SPATIAL_AXES) {
            path_jerk_per_accel_hz[leader] =
                jerk_per_accel(chains.chains[leader].kernel().map(|k| k.peak_abs()));
        }
        caps.push(ExtruderCaps {
            axis_index: *axis,
            max_velocity_mm_s: decl.max_velocity.unwrap_or(f64::INFINITY),
            max_accel_mm_s2: decl.max_accel.unwrap_or(f64::INFINITY),
            advance_slope_s,
            advance_curvature_s2_per_mm,
            path_jerk_per_accel_hz,
            own_jerk_per_accel_hz: jerk_per_accel(chain.kernel().map(|k| k.peak_abs())),
            instantaneous_corner_velocity_mm_s: chain.instantaneous_corner_velocity(),
        });
    }
    Ok(caps)
}

#[cfg(test)]
mod tests;
