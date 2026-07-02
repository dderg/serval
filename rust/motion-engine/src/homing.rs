use std::sync::{Arc, Mutex};

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
        let router_guard = router.lock().unwrap_or_else(|p| p.into_inner());
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

    let shadow_clock = {
        let router_guard = router.lock().unwrap_or_else(|p| p.into_inner());
        router_guard
            .host_time_to_mcu_clock(crate::types::mcu_handle_from_raw(axis_mcu), trip_host)
            .ok()
    };
    let store = history.lock().unwrap_or_else(|p| p.into_inner());
    let st = store
        .state_at_host(axis_key, trip_host, None)
        .map_err(|e| e.to_string())?;
    if let Some(shadow_clock) = shadow_clock {
        crate::motion_history::check_shadow_divergence(
            axis_key,
            st.position,
            store.state_at_clock_legacy(axis_key, shadow_clock, u64::MAX),
        );
    }
    Ok(st.position)
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
