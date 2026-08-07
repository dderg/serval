use crate::lock_ext::LockExt;
use std::sync::{Arc, Mutex};

use crate::kinematics::{KinematicsModule, SPATIAL_AXES};
use crate::mcu_config::{McuAxisConfig, SteppingMode};
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

/// Firmware resets the trip latch at arm and pre-arm buffered trips are
/// dropped, so a trip older than the arm window by more than clock-sync
/// jitter has no legitimate source: the clock model is broken.
pub const STALE_TRIP_HARD_LIMIT_S: f64 = 1.0;

pub fn reconstruct_axis_position(
    endstop_mcu: u32,
    trip_clock: u64,
    axis_key: AxisKey,
    router: &Arc<Mutex<PassthroughRouter>>,
    history: &Arc<Mutex<crate::motion_history::HistoryStore>>,
    window_start_host: f64,
    lane_start: f64,
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

    if trip_host < window_start_host - STALE_TRIP_HARD_LIMIT_S {
        return Err(format!(
            "endstop trip host time {trip_host:.6}s predates this homing move \
             (window starts at {window_start_host:.6}s) by more than \
             {STALE_TRIP_HARD_LIMIT_S}s — mis-synced clock \
             (endstop_mcu={endstop_mcu} trip_clock={trip_clock} \
             axis_mcu={axis_mcu})"
        ));
    }
    if trip_host <= window_start_host {
        tracing::warn!(
            subsystem = "homing",
            event = "insta_trip_clamped",
            endstop_mcu,
            axis_mcu = axis_key.mcu_id,
            axis = axis_key.axis,
            trip_host,
            window_start_host,
            lane_start,
            "trip at or before the arm window — the axis had not moved; \
             clamping to the run's start position"
        );
        return Ok(lane_start);
    }

    if !history.lock_ok().is_tracked(axis_key) {
        tracing::warn!(
            subsystem = "homing",
            event = "pre_motion_trip_clamped",
            endstop_mcu,
            axis_mcu = axis_key.mcu_id,
            axis = axis_key.axis,
            trip_host,
            lane_start,
            "trip on an axis with no recorded motion since attach — \
             motion-caused trips cannot precede their pieces' recording, \
             so this trip predates the run; clamping to its start position"
        );
        return Ok(lane_start);
    }

    // The trip is answered in the axis MCU's clock domain — the domain
    // the recorded pieces are keyed in — never against their host-time
    // keys. Host keys are the schedule as projected at send time; the
    // clock↔host mapping drifts between send and trip (sync jitter,
    // and in the simulator the virtual clock legally slips against
    // real time), and that drift lands the lookup a velocity-scaled
    // distance away. A remote endstop converts through both CURRENT
    // models back-to-back, so their shared drift cancels.
    let axis_clock = if endstop_mcu == axis_mcu {
        trip_clock
    } else {
        router
            .lock_ok()
            .host_time_to_mcu_clock(crate::types::mcu_handle_from_raw(axis_mcu), trip_host)
            .map_err(|e| format!("host_time_to_mcu_clock failed for axis mcu {axis_mcu}: {e:?}"))?
    };

    let store = history.lock_ok();
    match store.state_at_clock(axis_key, axis_clock, trip_host, None) {
        Ok(st) => Ok(st.position),
        Err(e @ crate::motion_history::HistoryError::BeforeRetainedWindow { .. })
            if store.predates_all_recorded_motion(axis_key, axis_clock) =>
        {
            tracing::warn!(
                subsystem = "homing",
                event = "pre_motion_trip_clamped",
                endstop_mcu,
                axis_mcu = axis_key.mcu_id,
                axis = axis_key.axis,
                trip_host,
                lane_start,
                error = %e,
                "trip precedes every piece ever recorded for this axis — \
                 motion-caused trips cannot precede their pieces' recording, \
                 so this trip predates the run; clamping to its start position"
            );
            Ok(lane_start)
        }
        Err(e) => Err(e.to_string()),
    }
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

pub fn motor_frame_start(
    configs: &[McuAxisConfig],
    start: geometry::MachinePos,
) -> Result<[f64; SPATIAL_AXES], String> {
    let kin_tag = configs
        .iter()
        .find(|c| c.axes.contains(&0usize))
        .map(|c| c.kinematics)
        .ok_or_else(|| {
            "spatial lane 0 is not configured on any mcu — cannot assemble \
             a cartesian position"
                .to_string()
        })?;
    Ok(KinematicsModule::from_tag(kin_tag)
        .map_err(|e| e.to_string())?
        .forward(start.0))
}

pub fn reconstruct_cartesian_position(
    endstop_mcu: u32,
    trip_clock: u64,
    configs: &[McuAxisConfig],
    router: &Arc<Mutex<PassthroughRouter>>,
    history: &Arc<Mutex<crate::motion_history::HistoryStore>>,
    window_start_host: f64,
    start: geometry::MachinePos,
) -> Result<geometry::MachinePos, String> {
    let motor_start = motor_frame_start(configs, start)?;
    cartesian_from_motor_lanes(configs, |key| {
        reconstruct_axis_position(
            endstop_mcu,
            trip_clock,
            key,
            router,
            history,
            window_start_host,
            motor_start[usize::from(key.axis)],
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

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StepcompressLane {
    pub mcu_id: u32,
    pub axis: u8,
    pub motor: usize,
    pub oid: u32,
    pub microstep_distance: f64,
    pub invert_dir: bool,
}

impl StepcompressLane {
    #[must_use]
    pub fn steps_to_mm(&self, mcu_count: i64) -> f64 {
        self.trajectory_steps(mcu_count) as f64 * self.microstep_distance
    }

    #[must_use]
    pub fn mm_to_steps(&self, mm: f64) -> i64 {
        (mm / self.microstep_distance).round() as i64
    }

    #[must_use]
    pub fn trajectory_steps(&self, mcu_count: i64) -> i64 {
        if self.invert_dir {
            -mcu_count
        } else {
            mcu_count
        }
    }
}

pub fn stepcompress_lane(
    cfg: &McuAxisConfig,
    axis_key: AxisKey,
) -> Result<Option<StepcompressLane>, String> {
    if cfg.stepping_mode != SteppingMode::Stepcompress {
        return Ok(None);
    }
    let mcu_id = cfg.mcu_id;
    let axis = axis_key.axis;
    let motor = cfg
        .axes
        .iter()
        .position(|&a| a == usize::from(axis))
        .ok_or_else(|| {
            format!(
                "stepcompress_lane: mcu {mcu_id} does not serve axis {axis} \
                 (configured axes {:?})",
                cfg.axes
            )
        })?;
    let oid = *cfg.stepper_oids.get(motor).ok_or_else(|| {
        format!(
            "stepcompress mcu {mcu_id} axis {axis}: motor {motor} has no stepper oid \
             (stepper_oids has {} entries for {} axes)",
            cfg.stepper_oids.len(),
            cfg.axes.len()
        )
    })?;
    let microstep_distance = *cfg.microstep_distance.get(motor).ok_or_else(|| {
        format!(
            "stepcompress mcu {mcu_id} axis {axis}: motor {motor} has no microstep distance \
             (microstep_distance has {} entries for {} axes)",
            cfg.microstep_distance.len(),
            cfg.axes.len()
        )
    })?;
    if microstep_distance <= 0.0 || !microstep_distance.is_finite() {
        return Err(format!(
            "stepcompress mcu {mcu_id} axis {axis}: microstep distance {microstep_distance} \
             is not a positive length"
        ));
    }
    let invert_dir = *cfg.invert_dir.get(motor).ok_or_else(|| {
        format!(
            "stepcompress mcu {mcu_id} axis {axis}: motor {motor} has no direction polarity \
             (invert_dir has {} entries for {} axes)",
            cfg.invert_dir.len(),
            cfg.axes.len()
        )
    })?;
    Ok(Some(StepcompressLane {
        mcu_id,
        axis,
        motor,
        oid,
        microstep_distance,
        invert_dir,
    }))
}

/// One lane's reconcile verdict: the mcu's executed step count against the
/// piece history at the same clock.
pub struct LaneReconcile {
    pub lane: StepcompressLane,
    pub history_position: f64,
    pub executed_steps: i64,
}

impl LaneReconcile {
    #[must_use]
    pub fn executed_position(&self) -> f64 {
        self.lane.steps_to_mm(self.executed_steps)
    }

    #[must_use]
    pub fn divergence(&self) -> f64 {
        (self.executed_position() - self.history_position).abs()
    }

    #[must_use]
    pub fn diverged(&self) -> bool {
        self.divergence() > self.lane.microstep_distance
    }

    fn describe(&self) -> String {
        format!(
            "mcu={} axis={} expected={:.6}mm (piece history) actual={:.6}mm \
             (stepper_get_position oid={} count={}) divergence={:.6}mm exceeds \
             one microstep ({:.6}mm)",
            self.lane.mcu_id,
            self.lane.axis,
            self.history_position,
            self.executed_position(),
            self.lane.oid,
            self.executed_steps,
            self.divergence(),
            self.lane.microstep_distance
        )
    }
}

pub fn reconcile_stepcompress_axis(
    cfg: &McuAxisConfig,
    axis_key: AxisKey,
    history_position: f64,
    query_step_count: &dyn Fn(&StepcompressLane) -> Result<i64, String>,
    reseed_step_counter: &dyn Fn(&StepcompressLane, i64) -> Result<(), String>,
) -> Result<f64, String> {
    let Some(lane) = stepcompress_lane(cfg, axis_key)? else {
        return Ok(history_position);
    };
    let verdict = LaneReconcile {
        lane,
        history_position,
        executed_steps: query_step_count(&lane)?,
    };
    if verdict.diverged() {
        return Err(format!(
            "stepcompress trip reconcile diverged: {}",
            verdict.describe()
        ));
    }
    reseed_step_counter(&lane, lane.trajectory_steps(verdict.executed_steps))?;
    Ok(history_position)
}

/// Every lane is measured before any lane is reseeded: a trip that halted the
/// whole cohort mid-move only makes sense as a whole, and reseeding the lanes
/// that happened to be checked before a diverging one would leave the cohort
/// half-adopted. On CoreXY the sibling belt lane is also the only thing that
/// distinguishes a timing error from a counting error, so a divergence report
/// names every lane, not just the first.
pub fn reconcile_stepcompress_lanes(
    configs: &[McuAxisConfig],
    mut lane_position: impl FnMut(AxisKey) -> Result<f64, String>,
    query_step_count: &dyn Fn(&StepcompressLane) -> Result<i64, String>,
    reseed_step_counter: &dyn Fn(&StepcompressLane, i64) -> Result<(), String>,
) -> Result<(), String> {
    let mut verdicts = Vec::new();
    for cfg in configs {
        if cfg.stepping_mode != SteppingMode::Stepcompress {
            continue;
        }
        for &lane in &cfg.axes {
            let axis_key = AxisKey {
                mcu_id: cfg.mcu_id,
                axis: lane as u8,
            };
            let Some(lane) = stepcompress_lane(cfg, axis_key)? else {
                continue;
            };
            verdicts.push(LaneReconcile {
                lane,
                history_position: lane_position(axis_key)?,
                executed_steps: query_step_count(&lane)?,
            });
        }
    }
    let diverged: Vec<String> = verdicts
        .iter()
        .filter(|v| v.diverged())
        .map(LaneReconcile::describe)
        .collect();
    if !diverged.is_empty() {
        return Err(format!(
            "stepcompress trip reconcile diverged: {}",
            diverged.join("; ")
        ));
    }
    for v in &verdicts {
        reseed_step_counter(&v.lane, v.lane.trajectory_steps(v.executed_steps))?;
    }
    Ok(())
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
