use crate::host_io::reactor::{Reactor, ReactorState};
use crate::transport::TransportError;
use runtime::error::FaultCode;

impl Reactor {
    pub(crate) fn transition_closed_on_io_fault(
        &mut self,
        context: &'static str,
        error: &TransportError,
    ) {
        let (os_errno, io_kind) = match error {
            TransportError::Io(io) => (io.raw_os_error(), Some(io.kind())),
            _ => (None, None),
        };
        let drain_curve: Vec<String> = (0..10)
            .map(|_| {
                let depth = self
                    .io
                    .bytes_to_write()
                    .map(|b| b.to_string())
                    .unwrap_or_else(|e| format!("err:{e}"));
                std::thread::sleep(std::time::Duration::from_millis(20));
                depth
            })
            .collect();
        tracing::error!(
            subsystem = "mcu-comms",
            event = "transport_io_fault",
            context,
            os_errno = ?os_errno,
            io_kind = ?io_kind,
            error = %error,
            unacked_n = self.unacked_window.len(),
            pending_piece_frames = self.outbound.pending_piece_frames.len(),
            outq_drain_curve_20ms = %drain_curve.join(","),
            "transport IO fault; transitioning Closed"
        );
        if self.pending_host_fault.is_none() {
            self.pending_host_fault = Some(crate::host_io::runtime_events::FaultEvent {
                fault_code: FaultCode::HostDisconnect.as_u16(),
                fault_detail: 0,
                segment_id: 0,
                synthesized: false,
            });
        }
        self.state = ReactorState::Closed;
    }

    pub(crate) fn close_if_io_fault(
        &mut self,
        context: &'static str,
        error: &TransportError,
    ) -> bool {
        let is_io = matches!(error, TransportError::Io(_));
        if is_io {
            self.transition_closed_on_io_fault(context, error);
        }
        is_io
    }
}
