use geometry::step_capacity::SPATIAL_LANES;
use geometry::{FollowerStepLoad, McuStepLoad, StepCapacity};

use crate::kinematics::KinematicsModule;
use crate::mcu_config::{McuAxisConfig, McuStepCapacity};

/// The planner's view of every mcu that measured its step dispatch: a motor
/// stepping `d` mm per step at `c` ticks a step takes `c / (d·C)` of an
/// mcu's `C` ticks per second for each mm/s it moves. `None` where no mcu
/// carries a measured capacity, so nothing is capped.
///
/// # Panics
///
/// When two capped mcus disagree on the kinematics, which one machine
/// cannot.
#[must_use]
pub fn step_capacity(mcus: &[McuAxisConfig]) -> Option<StepCapacity> {
    let mut kinematics = None;
    let loads: Vec<McuStepLoad> = mcus
        .iter()
        .filter_map(|cfg| {
            let capacity = cfg.hw.step_capacity.as_ref()?;
            let tag = *kinematics.get_or_insert(cfg.hw.kinematics);
            assert_eq!(
                tag, cfg.hw.kinematics,
                "mcu '{}' reports kinematics tag {} but another capped mcu reports {tag}",
                capacity.mcu_name, cfg.hw.kinematics,
            );
            Some(mcu_load(cfg, capacity))
        })
        .filter(|load| {
            load.lane_s_per_mm.iter().any(|&share| share > 0.0) || !load.followers.is_empty()
        })
        .collect();
    let module = KinematicsModule::from_tag(kinematics?)
        .expect("build_mcu_configs validated the kinematics tag");
    (!loads.is_empty()).then(|| StepCapacity {
        lane_weights: std::array::from_fn(|lane| module.lane_weights(lane)),
        mcus: loads,
    })
}

fn mcu_load(cfg: &McuAxisConfig, capacity: &McuStepCapacity) -> McuStepLoad {
    let mut lane_s_per_mm = [0.0; SPATIAL_LANES];
    let mut followers: Vec<FollowerStepLoad> = Vec::new();
    for (lane, &axis) in cfg.axes.iter().enumerate() {
        let share: f64 = cfg
            .motor_range(lane)
            .map(|motor| {
                f64::from(capacity.ticks_per_step[motor])
                    / (cfg.hw.microstep_distance[motor] * capacity.ticks_per_s)
            })
            .sum();
        if share == 0.0 {
            continue;
        }
        if axis < SPATIAL_LANES {
            lane_s_per_mm[axis] += share;
        } else {
            followers.push(FollowerStepLoad {
                axis_index: axis,
                s_per_mm: share,
            });
        }
    }
    McuStepLoad {
        mcu: capacity.mcu_name.clone(),
        lane_s_per_mm,
        followers,
    }
}

#[cfg(test)]
#[path = "step_capacity_tests.rs"]
mod step_capacity_tests;
