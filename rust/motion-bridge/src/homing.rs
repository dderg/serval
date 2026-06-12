use std::sync::{Arc, Mutex};

use crate::pump::AxisKey;
use kalico_host_rt::passthrough_queue::PassthroughRouter;

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
    window_start_clock: u64,
) -> Result<f64, String> {
    let axis_mcu = axis_key.mcu_id;

    let (axis_clock, trip_host_secs, host_now_secs) = if endstop_mcu == axis_mcu {
        (trip_clock, 0.0, 0.0)
    } else {
        let router_guard = router.lock().unwrap_or_else(|p| p.into_inner());
        let host_secs = router_guard
            .clock_to_host_secs(crate::types::mcu_handle_from_raw(endstop_mcu), trip_clock)
            .ok_or_else(|| {
                ReconstructError::ClockUnsynced {
                    description: format!(
                        "clock_to_host_secs returned None for endstop mcu {endstop_mcu}"
                    ),
                    endstop_mcu,
                    axis_mcu,
                    trip_clock,
                }
                .to_string()
            })?;
        let axis_clock = router_guard
            .host_time_to_mcu_clock(crate::types::mcu_handle_from_raw(axis_mcu), host_secs)
            .map_err(|e| {
                ReconstructError::ClockUnsynced {
                    description: format!(
                        "host_time_to_mcu_clock failed for axis mcu {axis_mcu}: {e:?}"
                    ),
                    endstop_mcu,
                    axis_mcu,
                    trip_clock,
                }
                .to_string()
            })?;
        (axis_clock, host_secs, router_guard.host_now_secs())
    };

    if axis_clock <= window_start_clock {
        return Err(format!(
            "endstop trip clock {axis_clock} predates this homing move \
             (window starts at {window_start_clock}) — stale trip or \
             mis-synced clock (endstop_mcu={endstop_mcu} trip_clock={trip_clock} \
             trip_host_secs={trip_host_secs:.6} host_now={host_now_secs:.6} \
             axis_mcu={axis_mcu})"
        ));
    }

    let store = history.lock().unwrap_or_else(|p| p.into_inner());
    store
        .state_at_clock(axis_key, axis_clock, None)
        .map(|st| st.position)
        .map_err(|e| e.to_string())
}

pub fn trajectory_final_position(
    axis_key: AxisKey,
    history: &Arc<Mutex<crate::motion_history::HistoryStore>>,
) -> Result<f64, String> {
    let store = history.lock().unwrap_or_else(|p| p.into_inner());
    store.final_position(axis_key).ok_or_else(|| {
        format!("trajectory_final_position: no recorded motion for axis {axis_key:?}")
    })
}

pub fn broadcast_stop<S, F>(
    mcu_ids: &std::collections::HashSet<u32, S>,
    axis_mcu: u32,
    call: F,
) -> Result<u64, String>
where
    S: std::hash::BuildHasher,
    F: Fn(u32) -> Result<kalico_protocol::messages::StopResponse, String>,
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
