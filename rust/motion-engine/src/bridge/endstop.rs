use crate::lock_ext::LockExt;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use super::{
    HomingRun, HomingState, McuAxisConfig, McuConnection, PassthroughRouter, RemoteFreeze,
};

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

#[derive(Debug, PartialEq, Eq)]
pub(super) enum TripMatch {
    Unmatched,
    Partial(Option<RemoteFreeze>),
    Final(Option<RemoteFreeze>),
}

/// Pure verdict for an inbound trip report: which member of the run it is,
/// whether the run continues (partial), and which remote motor to freeze.
/// A partial match removes the member from `remaining_trips`.
pub(super) fn match_trip(run: &mut HomingRun, event_mcu: u32, endstop_id: u8) -> TripMatch {
    let Some(member_idx) = run
        .remaining_trips
        .iter()
        .position(|t| t.endstop_mcu == event_mcu && t.endstop_id == endstop_id)
    else {
        return TripMatch::Unmatched;
    };
    if run.remaining_trips.len() > 1 {
        let member = run.remaining_trips.swap_remove(member_idx);
        return TripMatch::Partial(member.remote_freeze);
    }
    TripMatch::Final(run.remaining_trips[member_idx].remote_freeze)
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
    let mut run = match run_opt {
        None => {
            tracing::warn!(
                subsystem = "trip-relay",
                event = "early_trip_buffered",
                mcu = event_mcu,
                endstop_id,
                trip_clock,
                "terminal report arrived before the homing run was registered — buffered"
            );
            deps.homing
                .pending_trips
                .lock_ok()
                .push((event_mcu, endstop_id, trip_clock));
            return;
        }
        Some(r) => r,
    };
    let final_freeze = match match_trip(&mut run, event_mcu, endstop_id) {
        TripMatch::Unmatched => {
            tracing::warn!(
                subsystem = "trip-relay",
                event = "trip_identity_mismatch",
                mcu = event_mcu,
                endstop_id,
                expected = ?run.remaining_trips,
                trip_clock,
                "terminal report does not match the active homing run — ignored"
            );
            let mut guard = deps.homing.run.lock_ok();
            *guard = Some(run);
            return;
        }
        TripMatch::Partial(freeze_opt) => {
            tracing::info!(
                subsystem = "trip-relay",
                event = "partial_trip",
                mcu = event_mcu,
                endstop_id,
                trip_clock,
                remaining = run.remaining_trips.len(),
                "endstop tripped ahead of its group — motor frozen, run continues"
            );
            let notify = run.notify.clone();
            let remote = freeze_opt.filter(|f| f.motor_mcu != event_mcu);
            let pending = Arc::clone(&run.pending_suppresses);
            if remote.is_some() {
                let (count, _) = &*pending;
                *count.lock_ok() += 1;
            }
            {
                let mut guard = deps.homing.run.lock_ok();
                *guard = Some(run);
            }
            if let Some(freeze) = remote {
                send_remote_freeze(deps, notify, pending, freeze, event_mcu, endstop_id);
            }
            return;
        }
        TripMatch::Final(freeze) => freeze,
    };

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

            let mut terminal_errors = Vec::new();
            if let Err(e) = wait_for_pending_suppresses(&run.pending_suppresses) {
                terminal_errors.push(e);
            }

            let mut suppression_clock = None;
            if let Some(freeze) = final_freeze {
                if freeze.motor_mcu == event_mcu {
                    suppression_clock = Some((event_mcu, trip_clock));
                } else {
                    let outcome = transports
                        .get(&freeze.motor_mcu)
                        .ok_or_else(|| {
                            format!("StepperSuppress: no transport for mcu {}", freeze.motor_mcu)
                        })
                        .and_then(|t| suppress_call(t.as_ref(), freeze));
                    match outcome {
                        Ok(clock32) => {
                            let reference = router_arc
                                .lock_ok()
                                .compute_ack_clock(crate::types::mcu_handle_from_raw(
                                    freeze.motor_mcu,
                                ))
                                .unwrap_or(0);
                            suppression_clock = Some((
                                freeze.motor_mcu,
                                crate::remote_trigger::relay_trip_clock(clock32, reference),
                            ));
                        }
                        Err(e) => {
                            tracing::error!(
                                subsystem = "trip-relay",
                                event = "cross_mcu_suppress_failed",
                                mcu = event_mcu,
                                endstop_id,
                                motor_mcu = freeze.motor_mcu,
                                error = %e,
                                "final-trip stepper suppress failed; stopping the homing cohort"
                            );
                            terminal_errors.push(e);
                        }
                    }
                }
            }
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
                    terminal_errors
                        .push("EndstopTrip: pump did not halt before endpoint Stop".to_string());
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
                Ok(c) => Some(c),
                Err(e) => {
                    terminal_errors.push(e);
                    None
                }
            };
            if !terminal_errors.is_empty() {
                let _ = run.notify.send(Err(terminal_errors.join("; ")));
                return;
            }
            let discard_clock = discard_clock.expect("successful Stop has a discard clock");

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

            let (final_source_mcu, final_clock) =
                suppression_clock.unwrap_or((axis_key.mcu_id, discard_clock));
            let lane_starts = crate::mcu_config::reanchor_axis_targets(&configs, run_start);
            let outcome = reconstruct_cartesian(event_mcu, trip_clock).and_then(|trip| {
                crate::homing::reconcile_stepcompress_lanes(
                    &configs,
                    |key| {
                        crate::homing::reconstruct_axis_position(
                            final_source_mcu,
                            final_clock,
                            key,
                            &router_arc,
                            &history_arc,
                            run.window_start_host,
                            lane_starts
                                .iter()
                                .find(|(lane_key, _)| *lane_key == key)
                                .map(|(_, position)| *position),
                        )
                    },
                    &query_step_count,
                    &reseed_step_counter,
                )
                .map(|final_pos| (trip, final_pos, trip_clock))
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

fn send_remote_freeze(
    deps: &TripDeps,
    notify: crossbeam_channel::Sender<
        Result<(geometry::MachinePos, geometry::MachinePos, u64), String>,
    >,
    pending_suppresses: Arc<(std::sync::Mutex<usize>, std::sync::Condvar)>,
    freeze: RemoteFreeze,
    event_mcu: u32,
    endstop_id: u8,
) {
    let transport: Option<Arc<dyn host_rt::mcu_call::McuCall>> = {
        let mcus = deps.mcus.lock_ok();
        mcus.get(&freeze.motor_mcu).and_then(|conn| {
            if let Some(io) = conn.host_io.as_ref() {
                Some(Arc::clone(io) as Arc<dyn host_rt::mcu_call::McuCall>)
            } else {
                conn.endpoint_conn
                    .as_ref()
                    .map(|ec| Arc::clone(ec) as Arc<dyn host_rt::mcu_call::McuCall>)
            }
        })
    };
    std::thread::Builder::new()
        .name("homing-suppress".into())
        .spawn(move || {
            let outcome = transport
                .ok_or_else(|| {
                    format!("StepperSuppress: no transport for mcu {}", freeze.motor_mcu)
                })
                .and_then(|t| suppress_call(t.as_ref(), freeze));
            if let Err(e) = outcome {
                tracing::error!(
                    subsystem = "trip-relay",
                    event = "cross_mcu_suppress_failed",
                    mcu = event_mcu,
                    endstop_id,
                    motor_mcu = freeze.motor_mcu,
                    motor = freeze.motor_idx,
                    stepper = freeze.stepper_idx,
                    error = %e,
                    "cross-MCU stepper suppress failed — the homing move is aborted"
                );
                let _ = notify.send(Err(e));
            }
            let (count, ready) = &*pending_suppresses;
            let mut count = count.lock_ok();
            *count -= 1;
            ready.notify_all();
        })
        .expect("spawn homing-suppress");
}
pub(super) fn wait_for_pending_suppresses(
    pending: &Arc<(std::sync::Mutex<usize>, std::sync::Condvar)>,
) -> Result<(), String> {
    let (count, ready) = &**pending;
    let count = count.lock_ok();
    let (count, timeout) = ready
        .wait_timeout_while(count, Duration::from_secs(4), |count| *count != 0)
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    if timeout.timed_out() && *count != 0 {
        return Err(format!(
            "StepperSuppress: {} partial call(s) did not finish before terminal Stop",
            *count
        ));
    }
    Ok(())
}
fn suppress_call(
    transport: &dyn host_rt::mcu_call::McuCall,
    freeze: RemoteFreeze,
) -> Result<u32, String> {
    use mcu_protocol::codec::{Decode as _, Encode as _};
    let mut body = Vec::with_capacity(3);
    mcu_protocol::messages::StepperSuppress {
        motor: freeze.motor_idx,
        stepper: freeze.stepper_idx,
        engage: 1,
    }
    .encode(&mut body);
    let (_kind, resp_body) = transport
        .mcu_call(
            mcu_protocol::MessageKind::StepperSuppress,
            body,
            Duration::from_secs(3),
        )
        .map_err(|e| {
            format!(
                "StepperSuppress call failed for mcu {}: {e:?}",
                freeze.motor_mcu
            )
        })?;
    let resp =
        mcu_protocol::messages::StepperSuppressResponse::decode(&resp_body).map_err(|e| {
            format!(
                "StepperSuppress decode failed for mcu {}: {e:?}",
                freeze.motor_mcu
            )
        })?;
    if resp.effective_clock == 0 {
        return Err(format!(
            "StepperSuppress rejected by mcu {}",
            freeze.motor_mcu
        ));
    }
    Ok(resp.effective_clock)
}
