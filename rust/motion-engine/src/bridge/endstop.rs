use crate::lock_ext::LockExt;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use super::{HomingRun, HomingState, McuAxisConfig, McuConnection, PassthroughRouter};

#[derive(Clone)]
pub(super) struct TripDeps {
    pub(super) homing: Arc<HomingState>,
    pub(super) pump_tx: Arc<Mutex<Option<crossbeam_channel::Sender<crate::pump::PumpMsg>>>>,
    pub(super) mcus: Arc<Mutex<HashMap<u32, McuConnection>>>,
    pub(super) router: Arc<Mutex<PassthroughRouter>>,
    pub(super) motion_history: Arc<Mutex<crate::motion_history::HistoryStore>>,
    pub(super) mcu_axis_configs: Arc<Mutex<Vec<McuAxisConfig>>>,
    pub(super) stepcompress_endpoints:
        Arc<Mutex<HashMap<u32, Arc<Mutex<crate::pump::StepcompressEndpoint>>>>>,
}

pub(super) fn dispatch_endstop_trip(
    deps: &TripDeps,
    event_mcu: u32,
    endstop_id: u8,
    trip_clock: u64,
) {
    let run_opt: Option<HomingRun> = {
        let mut guard = deps.homing.run.lock_ok();
        guard.take()
    };
    let run = match run_opt {
        None => {
            tracing::warn!(
                subsystem = "trip-relay",
                event = "early_trip_buffered",
                mcu = event_mcu,
                endstop_id,
                trip_clock,
                "terminal report arrived before the homing run was registered — buffered"
            );
            *deps.homing.pending_trip.lock_ok() = Some((event_mcu, endstop_id, trip_clock));
            return;
        }
        Some(r) => r,
    };
    if run.endstop_id != endstop_id || run.endstop_mcu != event_mcu {
        tracing::warn!(
            subsystem = "trip-relay",
            event = "trip_identity_mismatch",
            mcu = event_mcu,
            endstop_id,
            expected_mcu = run.endstop_mcu,
            expected_endstop_id = run.endstop_id,
            trip_clock,
            "terminal report does not match the active homing run — ignored"
        );
        let mut guard = deps.homing.run.lock_ok();
        *guard = Some(run);
        return;
    }

    {
        let mut cohort_guard = deps.homing.active_drip_cohort.lock_ok();
        *cohort_guard = None;
    }

    let pump_tx_opt = deps.pump_tx.lock_ok().clone();

    let transports: HashMap<u32, Arc<dyn host_rt::mcu_call::McuCall>> = {
        let mcus = deps.mcus.lock_ok();
        mcus.iter()
            .filter_map(|(&id, conn)| {
                if let Some(io) = conn.host_io.as_ref() {
                    Some((id, Arc::clone(io) as Arc<dyn host_rt::mcu_call::McuCall>))
                } else {
                    conn.endpoint_conn
                        .as_ref()
                        .map(|ec| (id, Arc::clone(ec) as Arc<dyn host_rt::mcu_call::McuCall>))
                }
            })
            .collect()
    };

    let router_arc = Arc::clone(&deps.router);
    let history_arc = Arc::clone(&deps.motion_history);
    let configs: Vec<McuAxisConfig> = deps.mcu_axis_configs.lock_ok().clone();
    let host_ios: HashMap<u32, Arc<host_rt::host_io::McuHostIo>> = {
        let mcus = deps.mcus.lock_ok();
        mcus.iter()
            .filter_map(|(&id, conn)| conn.host_io.as_ref().map(|io| (id, Arc::clone(io))))
            .collect()
    };
    let endpoints = deps.stepcompress_endpoints.lock_ok().clone();

    std::thread::Builder::new()
        .name("homing-trip-handler".into())
        .spawn(move || {
            let stop_timeout = Duration::from_secs(3);

            let stepper_mcu_ids: std::collections::HashSet<u32> =
                run.all_axis_keys.iter().map(|k| k.mcu_id).collect();

            if let Some(tx) = pump_tx_opt.as_ref() {
                let _ = tx.send(crate::pump::PumpMsg::DripDisarm(run.cohort));
                let (ack_tx, ack_rx) = std::sync::mpsc::sync_channel(1);
                if tx
                    .send(crate::pump::PumpMsg::Halt {
                        keys: run.all_axis_keys.clone(),
                        ack: ack_tx,
                    })
                    .is_err()
                    || ack_rx.recv_timeout(Duration::from_secs(1)).is_err()
                {
                    let _ = run.notify.send(Err(
                        "EndstopTrip: pump did not halt before endpoint Stop".into(),
                    ));
                    return;
                }
            }

            use mcu_protocol::codec::Decode as _;
            let stop_call = |mcu_id: u32| -> Result<mcu_protocol::messages::StopResponse, String> {
                let transport = transports
                    .get(&mcu_id)
                    .ok_or_else(|| format!("Stop: no transport for mcu {mcu_id}"))?;
                let (_kind, body) = transport
                    .mcu_call(mcu_protocol::MessageKind::Stop, Vec::new(), stop_timeout)
                    .map_err(|e| format!("Stop call failed for mcu {mcu_id}: {e:?}"))?;
                mcu_protocol::messages::StopResponse::decode(&body)
                    .map_err(|e| format!("Stop decode failed for mcu {mcu_id}: {e:?}"))
            };

            let discard_clock = match crate::homing::broadcast_stop(
                &stepper_mcu_ids,
                run.axis_key.mcu_id,
                stop_call,
            ) {
                Ok(c) => c,
                Err(e) => {
                    let _ = run.notify.send(Err(e));
                    return;
                }
            };

            let axis_key = run.axis_key;
            let run_start = run.start_pos;
            let reconstruct_cartesian =
                |source_mcu: u32, clock: u64| -> Result<geometry::MachinePos, String> {
                    crate::homing::reconstruct_cartesian_position(
                        source_mcu,
                        clock,
                        &configs,
                        &router_arc,
                        &history_arc,
                        run.window_start_host,
                        run_start,
                    )
                };
            let motor_start = match crate::homing::motor_frame_start(&configs, run_start) {
                Ok(v) => v,
                Err(e) => {
                    let _ = run.notify.send(Err(e));
                    return;
                }
            };

            let query_step_count = |lane: &crate::homing::StepcompressLane| -> Result<i64, String> {
                let io = host_ios.get(&lane.mcu_id).ok_or_else(|| {
                    format!(
                        "stepper_get_position: no host_io for stepcompress mcu {}",
                        lane.mcu_id
                    )
                })?;
                let params = io
                    .call_args(
                        "stepper_get_position",
                        &[(
                            "oid".to_string(),
                            host_rt::host_io::parser::ArgValue::Int(i64::from(lane.oid)),
                        )],
                        "stepper_position",
                        stop_timeout,
                    )
                    .map_err(|e| {
                        format!(
                            "stepper_get_position failed for mcu {} oid {}: {e:?}",
                            lane.mcu_id, lane.oid
                        )
                    })?;
                params.try_get_i32("pos").map(i64::from).ok_or_else(|| {
                    format!(
                        "stepper_position from mcu {} oid {} carries no `pos` field",
                        lane.mcu_id, lane.oid
                    )
                })
            };
            let reseed_step_counter =
                |lane: &crate::homing::StepcompressLane, count: i64| -> Result<(), String> {
                    let endpoint = endpoints.get(&lane.mcu_id).ok_or_else(|| {
                        format!(
                            "stepcompress reconcile: no shim endpoint registered for mcu {}",
                            lane.mcu_id
                        )
                    })?;
                    let mut guard = endpoint.lock_ok();
                    guard.abort_outbound();
                    guard.reset_motor_position(lane.motor, count)
                };

            let outcome = reconstruct_cartesian(run.endstop_mcu, trip_clock)
                .and_then(|trip| {
                    reconstruct_cartesian(axis_key.mcu_id, discard_clock)
                        .map(|final_pos| (trip, final_pos, trip_clock))
                })
                .and_then(|positions| {
                    crate::homing::reconcile_stepcompress_lanes(
                        &configs,
                        |key| {
                            crate::homing::reconstruct_axis_position(
                                axis_key.mcu_id,
                                discard_clock,
                                key,
                                &router_arc,
                                &history_arc,
                                run.window_start_host,
                                motor_start.get(usize::from(key.axis)).copied(),
                            )
                        },
                        &query_step_count,
                        &reseed_step_counter,
                    )
                    .map(|()| positions)
                });

            let outcome = outcome.and_then(|positions| {
                for &mcu_id in &stepper_mcu_ids {
                    let transport = transports
                        .get(&mcu_id)
                        .ok_or_else(|| format!("ResumeStream: no transport for mcu {mcu_id}"))?;
                    let (_kind, body) = transport
                        .mcu_call(
                            mcu_protocol::MessageKind::ResumeStream,
                            Vec::new(),
                            stop_timeout,
                        )
                        .map_err(|e| format!("ResumeStream call failed for mcu {mcu_id}: {e:?}"))?;
                    let resp = mcu_protocol::messages::ResumeStreamResponse::decode(&body)
                        .map_err(|e| {
                            format!("ResumeStream decode failed for mcu {mcu_id}: {e:?}")
                        })?;
                    if resp.result != 0 {
                        return Err(format!(
                            "ResumeStream rejected by mcu {mcu_id}: result={}",
                            resp.result
                        ));
                    }
                }
                if let Some(tx) = pump_tx_opt.as_ref() {
                    tx.send(crate::pump::PumpMsg::Resume(run.all_axis_keys.clone()))
                        .map_err(|_| "EndstopTrip: pump channel closed before resume")?;
                    let (ack_tx, ack_rx) = std::sync::mpsc::sync_channel(1);
                    tx.send(crate::pump::PumpMsg::Barrier(ack_tx))
                        .map_err(|_| "EndstopTrip: pump channel closed before resume barrier")?;
                    ack_rx.recv_timeout(Duration::from_secs(1)).map_err(|_| {
                        "EndstopTrip: pump did not acknowledge resume after endpoint ResumeStream"
                    })?;
                }
                Ok(positions)
            });
            if let Err(e) = outcome.as_ref() {
                tracing::error!(
                    subsystem = "trip-relay",
                    event = "trip_handler_failed",
                    mcu = event_mcu,
                    endstop_id,
                    trip_clock,
                    error = %e,
                    "endstop trip handling failed — the homing move is aborted"
                );
            }
            let _ = run.notify.send(outcome);
        })
        .expect("spawn homing-trip-handler");
}
