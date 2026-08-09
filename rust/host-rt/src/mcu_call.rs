use std::time::Duration;

use mcu_protocol::MessageKind;

use crate::transport::TransportError;

pub trait McuCall: Send + Sync {
    fn mcu_call(
        &self,
        kind: MessageKind,
        body: Vec<u8>,
        timeout: Duration,
    ) -> Result<(MessageKind, Vec<u8>), TransportError>;
}

impl McuCall for crate::host_io::McuHostIo {
    fn mcu_call(
        &self,
        kind: MessageKind,
        body: Vec<u8>,
        timeout: Duration,
    ) -> Result<(MessageKind, Vec<u8>), TransportError> {
        crate::host_io::McuHostIo::mcu_call(self, kind, body, timeout)
    }
}
