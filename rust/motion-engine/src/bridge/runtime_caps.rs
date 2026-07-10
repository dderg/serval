use crate::lock_ext::LockExt;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use host_rt::host_io::McuHostIo;
use host_rt::mcu_serial_conn::McuSerialConn;

use super::state::McuConnection;

#[derive(Debug, thiserror::Error)]
pub(crate) enum RuntimeCapsError {
    #[error("mcu_call QueryRuntimeCaps: {0}")]
    Call(String),
    #[error("QueryRuntimeCaps: unexpected response kind {got:?}")]
    UnexpectedKind { got: mcu_protocol::MessageKind },
    #[error("decode RuntimeCapsResponse: {0}")]
    Decode(String),
}

pub(crate) fn require_positive(value: Option<f64>, name: &str) -> PyResult<()> {
    if let Some(v) = value {
        if !(v.is_finite() && v > 0.0) {
            return Err(PyValueError::new_err(format!(
                "{name} must be finite and positive"
            )));
        }
    }
    Ok(())
}

pub(crate) fn decode_runtime_caps_body(
    body: &[u8],
) -> Result<mcu_protocol::messages::RuntimeCapsResponse, RuntimeCapsError> {
    use mcu_protocol::codec::{Cursor, Decode};
    use mcu_protocol::messages::RuntimeCapsResponse;
    let mut c = Cursor::new(body);
    RuntimeCapsResponse::decode_from(&mut c).map_err(|e| RuntimeCapsError::Decode(format!("{e:?}")))
}

pub(crate) fn query_runtime_caps(
    io: &McuHostIo,
    timeout: std::time::Duration,
) -> Result<mcu_protocol::messages::RuntimeCapsResponse, RuntimeCapsError> {
    use mcu_protocol::MessageKind;
    let (kind, body) = io
        .mcu_call(MessageKind::QueryRuntimeCaps, Vec::new(), timeout)
        .map_err(|e| RuntimeCapsError::Call(format!("{e:?}")))?;
    if kind != MessageKind::RuntimeCapsResponse {
        return Err(RuntimeCapsError::UnexpectedKind { got: kind });
    }
    decode_runtime_caps_body(&body)
}

enum MotorQuery {
    Serial(Arc<McuHostIo>),
    EtherCat(Arc<McuSerialConn>, Vec<usize>),
}

impl MotorQuery {
    fn is_ethercat(&self) -> bool {
        matches!(self, MotorQuery::EtherCat(..))
    }
}

pub(crate) fn slots_for_axis(slot_axes: &[usize], axis: usize) -> Vec<u8> {
    slot_axes
        .iter()
        .enumerate()
        .filter(|&(_, &a)| a == axis)
        .filter_map(|(s, _)| u8::try_from(s).ok())
        .collect()
}

pub(crate) fn place_motor_response(
    resp: &mcu_protocol::messages::MotorStateResponse,
    slot_to_axis: &[usize],
    is_ethercat: bool,
    motors: &mut [Option<f64>],
    vmotors: &mut [Option<f64>],
) {
    fn put(
        motors: &mut [Option<f64>],
        vmotors: &mut [Option<f64>],
        slot: usize,
        m: &mcu_protocol::messages::MotorSample,
    ) {
        if slot < motors.len() {
            motors[slot] = Some(f64::from(m.pos_q16) / 65536.0);
            vmotors[slot] = Some(f64::from(m.vel_q16) / 65536.0);
        }
    }
    if is_ethercat {
        // With AWD, several slots claim the same axis; the lowest slot is the
        // axis's reporting drive so the answer is deterministic.
        let mut samples: Vec<&mcu_protocol::messages::MotorSample> = resp.motors.iter().collect();
        samples.sort_by_key(|m| m.slot);
        for m in samples {
            if let Some(&axis) = slot_to_axis.get(m.slot as usize) {
                if motors.get(axis).is_some_and(Option::is_some) {
                    continue;
                }
                put(motors, vmotors, axis, m);
            }
        }
    } else {
        for m in &resp.motors {
            put(motors, vmotors, m.slot as usize, m);
        }
    }
}

pub(crate) fn collect_motor_positions_inner(
    mcu_axis_configs: &Mutex<Vec<crate::mcu_config::McuAxisConfig>>,
    mcus: &Mutex<HashMap<u32, McuConnection>>,
    timeout: std::time::Duration,
) -> Result<HashMap<String, (f64, f64)>, String> {
    use host_rt::mcu_call::McuCall;
    use mcu_protocol::MessageKind;
    use mcu_protocol::codec::{Cursor, Decode};
    use mcu_protocol::messages::MotorStateResponse;
    use runtime::stepping_state::MAX_AXES;

    let configs = mcu_axis_configs.lock_ok().clone();
    if configs.is_empty() {
        return Err("query_motor_positions: no axes configured".into());
    }
    let kin_tag = configs
        .iter()
        .find(|c| c.axes.contains(&0usize))
        .map(|c| c.kinematics)
        .unwrap_or(runtime::segment::KinematicTag::Cartesian as u8);

    let mut motors: [Option<f64>; MAX_AXES] = [None; MAX_AXES];
    let mut vmotors: [Option<f64>; MAX_AXES] = [None; MAX_AXES];

    for cfg in &configs {
        let q = {
            let map = mcus.lock_ok();
            let Some(conn) = map.get(&cfg.mcu_id) else {
                continue;
            };
            if conn.ethercat_socket.is_some() {
                match conn.endpoint_conn.as_ref() {
                    Some(ep) => {
                        if conn.ethercat_slot_axes.is_empty() {
                            return Err(format!(
                                "query_motor_positions: EtherCAT mcu {} has an empty \
                                 slot->axis map — cannot attribute drive positions",
                                cfg.mcu_id
                            ));
                        }
                        MotorQuery::EtherCat(Arc::clone(ep), conn.ethercat_slot_axes.clone())
                    }
                    None => continue,
                }
            } else {
                match conn.host_io.as_ref() {
                    Some(io) => MotorQuery::Serial(Arc::clone(io)),
                    None => continue,
                }
            }
        };
        let (kind, body) = match &q {
            MotorQuery::Serial(io) => {
                io.mcu_call(MessageKind::QueryMotorState, Vec::new(), timeout)
            }
            MotorQuery::EtherCat(ep, _) => {
                ep.mcu_call(MessageKind::QueryMotorState, Vec::new(), timeout)
            }
        }
        .map_err(|e| format!("query mcu {}: {e:?}", cfg.mcu_id))?;
        if kind != MessageKind::MotorStateResponse {
            return Err(format!(
                "query mcu {}: unexpected kind {kind:?}",
                cfg.mcu_id
            ));
        }
        let mut c = Cursor::new(&body);
        let resp = MotorStateResponse::decode_from(&mut c)
            .map_err(|e| format!("query mcu {}: decode {e:?}", cfg.mcu_id))?;
        let slot_to_axis = match &q {
            MotorQuery::EtherCat(_, slot_axes) => slot_axes.as_slice(),
            MotorQuery::Serial(_) => &cfg.axes,
        };
        place_motor_response(
            &resp,
            slot_to_axis,
            q.is_ethercat(),
            &mut motors,
            &mut vmotors,
        );
    }
    crate::position_query::assemble_cartesian(&motors, &vmotors, kin_tag)
}

pub(crate) fn query_ethercat_runtime_caps(
    conn: &McuSerialConn,
    timeout: std::time::Duration,
) -> Result<mcu_protocol::messages::RuntimeCapsResponse, RuntimeCapsError> {
    use host_rt::mcu_call::McuCall;
    use mcu_protocol::MessageKind;
    let (kind, body) = conn
        .mcu_call(MessageKind::QueryRuntimeCaps, Vec::new(), timeout)
        .map_err(|e| RuntimeCapsError::Call(format!("{e:?}")))?;
    if kind != MessageKind::RuntimeCapsResponse {
        return Err(RuntimeCapsError::UnexpectedKind { got: kind });
    }
    decode_runtime_caps_body(&body)
}
