use std::time::Duration;

use host_rt::mcu_call::McuCall as _;
use host_rt::mcu_serial_conn::McuSerialConn;
use mcu_protocol::codec::{Decode as _, Encode as _};
use mcu_protocol::messages::{
    CaptureDrive, MessageKind, StartCapture, StartCaptureResponse, StopCapture, StopCaptureResponse,
};

const CAPTURE_TIMEOUT: Duration = Duration::from_secs(5);

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
    let (kind, resp) = conn
        .mcu_call(MessageKind::StartCapture, body, CAPTURE_TIMEOUT)
        .map_err(|e| format!("StartCapture transport: {e:?}"))?;
    if kind != MessageKind::StartCaptureResponse {
        return Err(format!(
            "StartCapture: unexpected response kind 0x{:04x}",
            kind.as_u16()
        ));
    }
    let r = StartCaptureResponse::decode(&resp)
        .map_err(|e| format!("StartCaptureResponse decode: {e:?}"))?;
    Ok(r.result)
}

pub fn send_stop_capture(conn: &McuSerialConn) -> Result<StopCaptureResponse, String> {
    let (kind, resp) = conn
        .mcu_call(
            MessageKind::StopCapture,
            StopCapture.encoded_to_vec(),
            CAPTURE_TIMEOUT,
        )
        .map_err(|e| format!("StopCapture transport: {e:?}"))?;
    if kind != MessageKind::StopCaptureResponse {
        return Err(format!(
            "StopCapture: unexpected response kind 0x{:04x}",
            kind.as_u16()
        ));
    }
    StopCaptureResponse::decode(&resp).map_err(|e| format!("StopCaptureResponse decode: {e:?}"))
}

#[cfg(test)]
mod tests;
