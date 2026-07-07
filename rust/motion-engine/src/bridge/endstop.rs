use crate::lock_ext::LockExt;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use super::{HomingRun, McuAxisConfig, McuConnection, PassthroughRouter};

#[derive(Clone)]
pub(super) struct TripDeps {
    pub(super) homing_run: Arc<Mutex<Option<HomingRun>>>,
    pub(super) pending_trip: Arc<Mutex<Option<(u32, u8, u64)>>>,
    pub(super) active_drip_cohort: Arc<Mutex<Option<u64>>>,
    pub(super) pump_tx: Arc<Mutex<Option<crossbeam_channel::Sender<crate::pump::PumpMsg>>>>,
    pub(super) mcus: Arc<Mutex<HashMap<u32, McuConnection>>>,
    pub(super) router: Arc<Mutex<PassthroughRouter>>,
    pub(super) motion_history: Arc<Mutex<crate::motion_history::HistoryStore>>,
    pub(super) mcu_axis_configs: Arc<Mutex<Vec<McuAxisConfig>>>,
}

pub(super) fn dispatch_endstop_trip(
    deps: &TripDeps,
    event_mcu: u32,
    endstop_id: u8,
    trip_clock: u64,
) {
    let run_opt: Option<HomingRun> = {
        let mut guard = deps.homing_run.lock_ok();
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
            *deps.pending_trip.lock_ok() = Some((event_mcu, endstop_id, trip_clock));
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
        let mut guard = deps.homing_run.lock_ok();
        *guard = Some(run);
        return;
    }

    {
        let mut cohort_guard = deps.active_drip_cohort.lock_ok();
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

    std::thread::Builder::new()
        .name("homing-trip-handler".into())
        .spawn(move || {
            let stop_timeout = Duration::from_secs(3);

            let stepper_mcu_ids: std::collections::HashSet<u32> =
                run.all_axis_keys.iter().map(|k| k.mcu_id).collect();

            if let Some(tx) = pump_tx_opt.as_ref() {
                let _ = tx.send(crate::pump::PumpMsg::Flush(run.all_axis_keys.clone()));
                let _ = tx.send(crate::pump::PumpMsg::DripDisarm(run.cohort));
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
            let reconstruct_cartesian = |source_mcu: u32, clock: u64| -> Result<[f64; 3], String> {
                let cfg = configs
                    .iter()
                    .find(|c| c.mcu_id == axis_key.mcu_id)
                    .ok_or_else(|| {
                        format!("EndstopTrip: no axis config for mcu {}", axis_key.mcu_id)
                    })?;
                crate::homing::reconstruct_cartesian_position(
                    source_mcu,
                    clock,
                    cfg,
                    &router_arc,
                    &history_arc,
                    run.window_start_host,
                )
            };

            let outcome = reconstruct_cartesian(run.endstop_mcu, trip_clock).and_then(|trip| {
                reconstruct_cartesian(axis_key.mcu_id, discard_clock)
                    .map(|final_pos| (trip, final_pos, trip_clock))
            });

            let outcome = outcome.and_then(|positions| {
                if let Some(tx) = pump_tx_opt.as_ref() {
                    let (ack_tx, ack_rx) = std::sync::mpsc::sync_channel(1);
                    let _ = tx.send(crate::pump::PumpMsg::Barrier(ack_tx));
                    if ack_rx.recv_timeout(Duration::from_secs(1)).is_err() {
                        return Err("EndstopTrip: pump did not acknowledge the flush barrier \
                                 before stream resume"
                            .into());
                    }
                }
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
                Ok(positions)
            });
            let _ = run.notify.send(outcome);
        })
        .expect("spawn homing-trip-handler");
}
