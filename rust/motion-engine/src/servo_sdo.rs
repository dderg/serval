use std::time::Duration;

use host_rt::mcu_serial_conn::McuSerialConn;
use mcu_protocol::codec::Encode as _;
use mcu_protocol::messages::{
    ERR_SDO_TRANSPORT, ERR_SDO_UNSUPPORTED_SIZE, ERR_SDO_VALUE_RANGE, ERR_SDO_VERIFY_MISMATCH,
    MessageKind, SdoRead, SdoReadResponse, SdoWrite, SdoWriteResponse,
};

use crate::servo_call::mcu_typed_call;

const SDO_TIMEOUT: Duration = Duration::from_secs(5);

pub fn send_sdo_read(
    conn: &McuSerialConn,
    slot: u8,
    index: u16,
    subindex: u8,
) -> Result<SdoReadResponse, String> {
    let body = SdoRead {
        slot,
        index,
        subindex,
    }
    .encoded_to_vec();
    mcu_typed_call(
        conn,
        "SdoRead",
        MessageKind::SdoRead,
        MessageKind::SdoReadResponse,
        body,
        SDO_TIMEOUT,
    )
}

pub fn send_sdo_write(
    conn: &McuSerialConn,
    slot: u8,
    index: u16,
    subindex: u8,
    size: u8,
    value: i64,
) -> Result<SdoWriteResponse, String> {
    let body = SdoWrite {
        slot,
        index,
        subindex,
        size,
        value,
    }
    .encoded_to_vec();
    mcu_typed_call(
        conn,
        "SdoWrite",
        MessageKind::SdoWrite,
        MessageKind::SdoWriteResponse,
        body,
        SDO_TIMEOUT,
    )
}

pub fn failure_text(result: i32) -> String {
    match result {
        ERR_SDO_UNSUPPORTED_SIZE => "object size unsupported (must be 1..=4 bytes)".into(),
        ERR_SDO_VERIFY_MISMATCH => "readback mismatch".into(),
        ERR_SDO_TRANSPORT => "SDO transport failure (no CoE abort code)".into(),
        ERR_SDO_VALUE_RANGE => "value does not fit the object width".into(),
        code if code > 0 => format!("CoE abort 0x{:08x}", code as u32),
        code => format!("endpoint error {code}"),
    }
}

#[cfg(test)]
mod tests;
