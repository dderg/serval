use crate::lock_ext::LockExt;
use std::sync::{Arc, Mutex};

use crate::kinematics::{KinematicsModule, SPATIAL_AXES};
use crate::mcu_config::McuAxisConfig;
use crate::types::AxisKey;
use host_rt::passthrough_queue::PassthroughRouter;

#[derive(Debug, thiserror::Error)]
pub enum ReconstructError {
    #[error(
        "clock unsynced: {description} (endstop_mcu={endstop_mcu}, \
         axis_mcu={axis_mcu}, trip_clock={trip_clock})"
    )]
    ClockUnsynced {
        description: String,
        endstop_mcu: u32,
        axis_mcu: u32,
        trip_clock: u64,
    },
}

pub fn reconstruct_axis_position(
    endstop_mcu: u32,
    trip_clock: u64,
    axis_key: AxisKey,
    router: &Arc<Mutex<PassthroughRouter>>,
    history: &Arc<Mutex<crate::motion_history::HistoryStore>>,
    window_start_host: f64,
) -> Result<f64, String> {
    let axis_mcu = axis_key.mcu_id;

    let trip_host = {
        let router_guard = router.lock_ok();
        crate::motion_history::clock_to_host(
            &router_guard,
            crate::types::mcu_handle_from_raw(endstop_mcu),
            trip_clock,
        )
        .map_err(|description| {
            ReconstructError::ClockUnsynced {
                description,
                endstop_mcu,
                axis_mcu,
                trip_clock,
            }
            .to_string()
        })?
    };

    if trip_host <= window_start_host {
        return Err(format!(
            "endstop trip host time {trip_host:.6}s predates this homing move \
             (window starts at {window_start_host:.6}s) — stale trip or \
             mis-synced clock (endstop_mcu={endstop_mcu} trip_clock={trip_clock} \
             axis_mcu={axis_mcu})"
        ));
    }

    let store = history.lock_ok();
    let st = store
        .state_at_host(axis_key, trip_host, None)
        .map_err(|e| e.to_string())?;
    Ok(st.position)
}

pub fn trajectory_final_position(
    axis_key: AxisKey,
    history: &Arc<Mutex<crate::motion_history::HistoryStore>>,
) -> Result<f64, String> {
    let store = history.lock_ok();
    store.final_position(axis_key).ok_or_else(|| {
        format!("trajectory_final_position: no recorded motion for axis {axis_key:?}")
    })
}

fn cartesian_from_motor_lanes(
    configs: &[McuAxisConfig],
    mut lane_position: impl FnMut(AxisKey) -> Result<f64, String>,
) -> Result<[f64; SPATIAL_AXES], String> {
    let mut motor_frame = [0.0_f64; SPATIAL_AXES];
    let mut lane_owner = [None::<u32>; SPATIAL_AXES];
    for cfg in configs {
        for &lane in &cfg.axes {
            if lane >= SPATIAL_AXES {
                continue;
            }
            if let Some(prior_mcu) = lane_owner[lane] {
                return Err(format!(
                    "cartesian_from_motor_lanes: spatial lane {lane} is configured \
                     on both mcu {prior_mcu} and mcu {}",
                    cfg.mcu_id
                ));
            }
            lane_owner[lane] = Some(cfg.mcu_id);
            motor_frame[lane] = lane_position(AxisKey {
                mcu_id: cfg.mcu_id,
                axis: lane as u8,
            })?;
        }
    }
    if let Some(missing) = lane_owner.iter().position(Option::is_none) {
        return Err(format!(
            "cartesian_from_motor_lanes: spatial lane {missing} is not configured \
             on any mcu — cannot assemble a cartesian position"
        ));
    }
    let kin_tag = configs
        .iter()
        .find(|c| c.axes.contains(&0usize))
        .map(|c| c.kinematics)
        .expect("lane 0 owner exists: checked above");
    Ok(KinematicsModule::from_tag(kin_tag)
        .map_err(|e| e.to_string())?
        .inverse(motor_frame))
}

pub fn reconstruct_cartesian_position(
    endstop_mcu: u32,
    trip_clock: u64,
    configs: &[McuAxisConfig],
    router: &Arc<Mutex<PassthroughRouter>>,
    history: &Arc<Mutex<crate::motion_history::HistoryStore>>,
    window_start_host: f64,
) -> Result<geometry::MachinePos, String> {
    cartesian_from_motor_lanes(configs, |key| {
        reconstruct_axis_position(
            endstop_mcu,
            trip_clock,
            key,
            router,
            history,
            window_start_host,
        )
    })
    .map(geometry::MachinePos)
}

pub fn final_cartesian_position(
    configs: &[McuAxisConfig],
    history: &Arc<Mutex<crate::motion_history::HistoryStore>>,
) -> Result<geometry::MachinePos, String> {
    cartesian_from_motor_lanes(configs, |key| trajectory_final_position(key, history))
        .map(geometry::MachinePos)
}

pub fn broadcast_stop<S, F>(
    mcu_ids: &std::collections::HashSet<u32, S>,
    axis_mcu: u32,
    call: F,
) -> Result<u64, String>
where
    S: std::hash::BuildHasher,
    F: Fn(u32) -> Result<mcu_protocol::messages::StopResponse, String>,
{
    let mut errors: Vec<String> = Vec::new();
    let mut axis_discard_clock: Option<u64> = None;
    for &mcu_id in mcu_ids {
        match call(mcu_id) {
            Ok(resp) if resp.result != 0 => {
                errors.push(format!(
                    "Stop rejected by mcu {mcu_id}: result={}",
                    resp.result
                ));
            }
            Ok(resp) => {
                if mcu_id == axis_mcu {
                    axis_discard_clock = Some(resp.discard_clock);
                }
            }
            Err(e) => errors.push(e),
        }
    }
    if !errors.is_empty() {
        return Err(format!(
            "EndstopTrip Stop broadcast failed: {}",
            errors.join("; ")
        ));
    }
    axis_discard_clock
        .ok_or_else(|| format!("EndstopTrip: axis MCU {axis_mcu} did not report a discard clock"))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveFaultRoute {
    HomingError,
    LatchForKlippy,
}

pub fn route_drive_fault(fault_mcu: u32, homing_axis_mcu: Option<u32>) -> DriveFaultRoute {
    if homing_axis_mcu == Some(fault_mcu) {
        DriveFaultRoute::HomingError
    } else {
        DriveFaultRoute::LatchForKlippy
    }
}

#[cfg(test)]
mod tests;
