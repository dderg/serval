use std::time::Duration;

use host_rt::mcu_call::McuCall as _;
use host_rt::mcu_serial_conn::McuSerialConn;
use mcu_protocol::MessageKind;
use mcu_protocol::codec::Decode;

pub fn mcu_typed_call<Resp: Decode>(
    conn: &McuSerialConn,
    op: &str,
    req_kind: MessageKind,
    resp_kind: MessageKind,
    body: Vec<u8>,
    timeout: Duration,
) -> Result<Resp, String> {
    let (kind, resp) = conn
        .mcu_call(req_kind, body, timeout)
        .map_err(|e| format!("{op} transport: {e:?}"))?;
    if kind != resp_kind {
        return Err(format!(
            "{op}: unexpected response kind 0x{:04x}",
            kind.as_u16()
        ));
    }
    Resp::decode(&resp).map_err(|e| format!("{op} decode: {e:?}"))
}
