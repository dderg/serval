use motion_core::lock_ext::LockExt;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use super::{
    HomingRun, HomingState, McuAxisConfig, McuConnection, McuHandle, PassthroughRouter,
    RemoteFreeze,
};

#[derive(Clone)]
pub(super) struct TripDeps {
    pub(super) homing: Arc<HomingState>,
    pub(super) pump_tx: Arc<Mutex<Option<crossbeam_channel::Sender<motion_core::pump::PumpMsg>>>>,
    pub(super) mcus: Arc<Mutex<HashMap<u32, McuConnection>>>,
    pub(super) router: Arc<Mutex<PassthroughRouter>>,
    pub(super) motion_history: Arc<Mutex<motion_core::motion_history::HistoryStore>>,
    pub(super) mcu_axis_configs: Arc<Mutex<Vec<McuAxisConfig>>>,
    pub(super) axis_transports: Arc<motion_core::axis_transport::AxisTransports>,
}

impl McuConnection {
    pub(super) fn homing_transport(&self) -> Option<Arc<dyn host_rt::mcu_call::McuCall>> {
        self.host_io
            .as_ref()
            .map(|io| Arc::clone(io) as Arc<dyn host_rt::mcu_call::McuCall>)
            .or_else(|| {
                self.endpoint_conn
                    .as_ref()
                    .map(|conn| Arc::clone(conn) as Arc<dyn host_rt::mcu_call::McuCall>)
            })
    }
}

impl TripDeps {
    fn endpoint_command(&self, command: motion_core::pump::EndpointCommand) -> Result<(), String> {
        let tx = self
            .pump_tx
            .lock_ok()
            .clone()
            .ok_or_else(|| "endpoint command: execution owner is not running".to_string())?;
        super::axis_transport_api::endpoint_command(&tx, command)
    }

    fn transport(&self, mcu_id: u32) -> Option<Arc<dyn host_rt::mcu_call::McuCall>> {
        self.mcus
            .lock_ok()
            .get(&mcu_id)
            .and_then(McuConnection::homing_transport)
    }

    fn step_count(&self, lane: &motion_core::homing::StepcompressLane) -> Result<i64, String> {
        let io = self
            .mcus
            .lock_ok()
            .get(&lane.mcu_id)
            .and_then(|conn| conn.host_io.clone())
            .ok_or_else(|| {
                format!(
                    "stepper_get_position: no host_io for stepcompress mcu {}",
                    lane.mcu_id
                )
            })?;
        let params = io
            .call_args(
                "stepper_get_position",
                &[(
                    "oid",
                    host_rt::host_io::parser::ArgValue::Int(i64::from(lane.oid)),
                )],
                "stepper_position",
                Duration::from_secs(3),
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
    }
}

#[derive(Debug, PartialEq, Eq)]
pub(super) enum TripMatch {
    Unmatched,
    Partial(Option<RemoteFreeze>),
    Final(Option<RemoteFreeze>),
}

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
    let (run, final_freeze) = {
        let mut state = deps.homing.lifecycle.lock_ok();
        let Some(run) = state.trip_run((event_mcu, endstop_id, trip_clock)) else {
            return;
        };
        match match_trip(run, event_mcu, endstop_id) {
            TripMatch::Unmatched => return,
            TripMatch::Partial(freeze) => {
                let Some(freeze) = freeze else { return };
                let cohort = run.cohort;
                state.pending_suppresses += 1;
                drop(state);
                if freeze.motor_mcu != event_mcu {
                    send_remote_freeze(deps, cohort, freeze, event_mcu, endstop_id);
                } else {
                    let outcome = cut_frozen_motor_stream(deps, freeze);
                    finish_partial_work(deps, cohort, outcome.err());
                }
                return;
            }
            TripMatch::Final(freeze) => {
                let run = state.take_terminal(|_| true).unwrap();
                (run, freeze)
            }
        }
    };

    let deps = deps.clone();
    std::thread::Builder::new()
        .name("homing-trip-handler".into())
        .spawn(move || {
            let pump_tx_opt = deps.pump_tx.lock_ok().clone();
            let configs = deps.mcu_axis_configs.lock_ok().clone();
            let homing = &deps.homing;
            let stop_timeout = Duration::from_secs(3);

            let stepper_mcu_ids: std::collections::HashSet<u32> =
                run.all_axis_keys.iter().map(|k| k.mcu_id).collect();

            let mut terminal_errors = Vec::new();
            if let Err(e) = homing.wait_for_pending_suppresses(run.cohort) {
                terminal_errors.push(e);
            }

            let mut suppression_clock = None;
            if let Some(freeze) = final_freeze {
                if freeze.motor_mcu == event_mcu {
                    suppression_clock = Some((event_mcu, trip_clock));
                } else {
                    let outcome = deps
                        .transport(freeze.motor_mcu)
                        .ok_or_else(|| {
                            format!("StepperSuppress: no transport for mcu {}", freeze.motor_mcu)
                        })
                        .and_then(|t| suppress_call(t.as_ref(), freeze));
                    match outcome {
                        Ok(clock32) => {
                            let reference = deps
                                .router
                                .lock_ok()
                                .compute_ack_clock(McuHandle::from_raw(freeze.motor_mcu))
                                .unwrap_or(0);
                            suppression_clock = Some((
                                freeze.motor_mcu,
                                motion_services::remote_trigger::relay_trip_clock(
                                    clock32, reference,
                                ),
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
                let _ = tx.send(motion_core::pump::PumpMsg::DripDisarm(run.cohort));
                let (ack_tx, ack_rx) = std::sync::mpsc::sync_channel(1);
                if tx
                    .send(motion_core::pump::PumpMsg::Halt {
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
                let transport = deps
                    .transport(mcu_id)
                    .ok_or_else(|| format!("Stop: no transport for mcu {mcu_id}"))?;
                let (_kind, body) = transport
                    .mcu_call(mcu_protocol::MessageKind::Stop, Vec::new(), stop_timeout)
                    .map_err(|e| format!("Stop call failed for mcu {mcu_id}: {e:?}"))?;
                mcu_protocol::messages::StopResponse::decode(&body)
                    .map_err(|e| format!("Stop decode failed for mcu {mcu_id}: {e:?}"))
            };

            let discard_clock = match motion_core::homing::broadcast_stop(
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
                homing.complete(run.cohort, Err(terminal_errors.join("; ")));
                return;
            }
            let discard_clock = discard_clock.expect("successful Stop has a discard clock");

            let axis_key = run.axis_key;
            let run_start = run.start_pos;
            let reconstruct_cartesian =
                |source_mcu: u32, clock: u64| -> Result<geometry::MachinePos, String> {
                    motion_core::homing::reconstruct_cartesian_position(
                        source_mcu,
                        clock,
                        &configs,
                        &deps.router,
                        &deps.motion_history,
                        run.window_start_host,
                        run_start,
                    )
                };

            let reseed_step_counter =
                |lane: &motion_core::homing::StepcompressLane, count: i64| -> Result<(), String> {
                    deps.endpoint_command(motion_core::pump::EndpointCommand::ReseedMotor {
                        mcu_id: lane.mcu_id,
                        motor: lane.motor,
                        count,
                    })
                };

            let (final_source_mcu, final_clock) =
                suppression_clock.unwrap_or((axis_key.mcu_id, discard_clock));
            let lane_starts = motion_core::mcu_config::reanchor_axis_targets(&configs, run_start);
            let outcome = reconstruct_cartesian(event_mcu, trip_clock).and_then(|trip| {
                motion_core::homing::reconcile_stepcompress_lanes(
                    &configs,
                    &deps.axis_transports,
                    |key| {
                        motion_core::homing::reconstruct_axis_position(
                            final_source_mcu,
                            final_clock,
                            key,
                            &deps.router,
                            &deps.motion_history,
                            run.window_start_host,
                            lane_starts
                                .iter()
                                .find(|(lane_key, _)| *lane_key == key)
                                .map(|(_, position)| *position),
                        )
                    },
                    &|lane| deps.step_count(lane),
                    &reseed_step_counter,
                )
                .map(|final_pos| (trip, final_pos, trip_clock))
            });

            let outcome = outcome.and_then(|positions| {
                if let Some(error) = homing.lifecycle.lock_ok().failure.clone() {
                    return Err(error);
                }
                for &mcu_id in &stepper_mcu_ids {
                    let transport = deps
                        .transport(mcu_id)
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
                    tx.send(motion_core::pump::PumpMsg::Resume(
                        run.all_axis_keys.clone(),
                    ))
                    .map_err(|_| "EndstopTrip: pump channel closed before resume")?;
                    let (ack_tx, ack_rx) = std::sync::mpsc::sync_channel(1);
                    tx.send(motion_core::pump::PumpMsg::Barrier(ack_tx))
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
            homing.complete(run.cohort, outcome);
        })
        .expect("spawn homing-trip-handler");
}

fn cut_frozen_motor_stream(deps: &TripDeps, freeze: RemoteFreeze) -> Result<(), String> {
    let lane = motion_core::homing::stepcompress_lane_of_oid(
        &deps.mcu_axis_configs.lock_ok(),
        freeze.motor_mcu,
        freeze.stepper_oid,
    )?;
    let executed = deps.step_count(&lane)?;
    deps.endpoint_command(motion_core::pump::EndpointCommand::FreezeMotor {
        mcu_id: lane.mcu_id,
        motor: lane.motor,
        count: lane.trajectory_steps(executed),
    })?;
    tracing::info!(
        subsystem = "trip-relay",
        event = "keyed_freeze_cut",
        mcu = lane.mcu_id,
        axis = lane.axis,
        motor = lane.motor,
        oid = lane.oid,
        executed_steps = executed,
        "keyed trip cut and reseeded only the frozen motor's stream"
    );
    Ok(())
}

fn send_remote_freeze(
    deps: &TripDeps,
    cohort: u64,
    freeze: RemoteFreeze,
    event_mcu: u32,
    endstop_id: u8,
) {
    let transport = deps.transport(freeze.motor_mcu);
    let deps = deps.clone();
    std::thread::Builder::new()
        .name("homing-suppress".into())
        .spawn(move || {
            let outcome = transport
                .ok_or_else(|| {
                    format!("StepperSuppress: no transport for mcu {}", freeze.motor_mcu)
                })
                .and_then(|t| suppress_call(t.as_ref(), freeze));
            if let Err(e) = &outcome {
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
            }
            let outcome = outcome.and_then(|_| cut_frozen_motor_stream(&deps, freeze));
            finish_partial_work(&deps, cohort, outcome.err());
        })
        .expect("spawn homing-suppress");
}

fn finish_partial_work(deps: &TripDeps, cohort: u64, error: Option<String>) {
    let terminal = deps.homing.retire_partial(cohort, error);
    if let Some(run) = terminal {
        if let Some(tx) = deps.pump_tx.lock_ok().clone() {
            let _ = tx.send(motion_core::pump::PumpMsg::Flush(run.all_axis_keys));
            let _ = tx.send(motion_core::pump::PumpMsg::DripDisarm(cohort));
        }
        deps.homing
            .complete(cohort, Err("partial homing freeze failed".into()));
    }
}
fn suppress_call(
    transport: &dyn host_rt::mcu_call::McuCall,
    freeze: RemoteFreeze,
) -> Result<u32, String> {
    use mcu_protocol::codec::{Decode as _, Encode as _};
    let stepper_oid = u8::try_from(freeze.stepper_oid).map_err(|_| {
        format!(
            "StepperSuppress: stepper oid {} on mcu {} exceeds u8",
            freeze.stepper_oid, freeze.motor_mcu
        )
    })?;
    let mut body = Vec::with_capacity(4);
    mcu_protocol::messages::StepperSuppress {
        motor: freeze.motor_idx,
        stepper: freeze.stepper_idx,
        engage: 1,
        stepper_oid,
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
            "StepperSuppress on mcu {} reported a zero effective clock — the freeze instant is \
             unknown and the trip clock cannot be relayed",
            freeze.motor_mcu
        ));
    }
    Ok(resp.effective_clock)
}
