use std::time::Duration;

use host_rt::mcu_serial_conn::McuSerialConn;
use mcu_protocol::codec::Encode as _;
use mcu_protocol::messages::{
    CaptureDrive, MessageKind, StartCapture, StartCaptureResponse, StopCapture, StopCaptureResponse,
};

use crate::servo_call::mcu_typed_call;

const START_CAPTURE_TIMEOUT: Duration = Duration::from_secs(5);
const STOP_CAPTURE_TIMEOUT: Duration = Duration::from_secs(30);

pub fn send_start_capture(
    conn: &McuSerialConn,
    path: &str,
    started_utc: &str,
    drives: &[(u8, String)],
) -> Result<i32, String> {
    let body = StartCapture {
        path: path.to_owned(),
        started_utc: started_utc.to_owned(),
        drives: drives
            .iter()
            .map(|(slot, name)| CaptureDrive {
                slot: *slot,
                name: name.clone(),
            })
            .collect(),
    }
    .encoded_to_vec();
    let r: StartCaptureResponse = mcu_typed_call(
        conn,
        "StartCapture",
        MessageKind::StartCapture,
        MessageKind::StartCaptureResponse,
        body,
        START_CAPTURE_TIMEOUT,
    )?;
    Ok(r.result)
}

pub fn send_stop_capture(conn: &McuSerialConn) -> Result<StopCaptureResponse, String> {
    mcu_typed_call(
        conn,
        "StopCapture",
        MessageKind::StopCapture,
        MessageKind::StopCaptureResponse,
        StopCapture.encoded_to_vec(),
        STOP_CAPTURE_TIMEOUT,
    )
}

#[cfg(test)]
mod tests;
