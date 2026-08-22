use crate::host_io::mcu_session::PendingMcuCall;
use crate::host_io::reactor::Reactor;
use crate::transport::TransportError;

impl Reactor {
    pub(super) fn fail_pending_on_mcu_shutdown(
        &mut self,
        response_name: &str,
        params: &crate::transport::MessageParams,
    ) {
        if self.awaiting_response.len() == 0 && self.outbound.pending_submissions.is_empty() {
            return;
        }
        let reason = params
            .fields
            .get("static_string_id")
            .and_then(|v| match v {
                crate::transport::MessageValue::U32(n) => Some(*n as i32),
                crate::transport::MessageValue::I32(n) => Some(*n),
                _ => None,
            })
            .and_then(|id| self.parser.static_strings.get(&id).cloned())
            .unwrap_or_else(|| format!("unresolved reason ({response_name})"));
        tracing::error!(
            subsystem = "mcu-comms",
            event = "mcu_shutdown_fail_fast",
            response = %response_name,
            %reason,
            awaiting = self.awaiting_response.len(),
            pending = self.outbound.pending_submissions.len(),
            "MCU reports shutdown; failing pending calls instead of timing out"
        );
        for entry in self.awaiting_response.drain_all() {
            let _ = entry
                .completion
                .send(Err(TransportError::McuShutdown(reason.clone())));
        }
        for p in self.outbound.pending_submissions.drain(..) {
            let _ = p
                .completion
                .send(Err(TransportError::McuShutdown(reason.clone())));
        }
    }

    pub(super) fn flush_all_completions(&mut self) {
        self.pending_clock_sent_raw = None;
        for entry in self.awaiting_response.drain_all() {
            let _ = entry.completion.send(Err(TransportError::Closed));
        }
        self.unacked_window.clear();
        for p in self.outbound.pending_submissions.drain(..) {
            let _ = p.completion.send(Err(TransportError::Closed));
        }
        self.outbound.pending_fire_and_forget.clear();
        self.outbound.pending_outbound_order.clear();

        let drained: Vec<PendingMcuCall> = self
            .transport_state
            .pending
            .drain()
            .map(|(_, v)| v)
            .collect();
        for p in drained {
            let _ = p.completion.send(Err(TransportError::Closed));
        }
        if let Some(c) = self.transport_state.identify_pending.take() {
            let _ = c.send(Err(TransportError::Closed));
        }
    }

    pub(crate) fn gc_transport_pending(&mut self) {
        let now = self.clock.now();
        let expired: Vec<u32> = self
            .transport_state
            .pending
            .iter()
            .filter_map(|(cid, p)| if p.deadline <= now { Some(*cid) } else { None })
            .collect();
        for cid in expired {
            if let Some(p) = self.transport_state.pending.remove(&cid) {
                let _ = p.completion.send(Err(TransportError::Timeout));
            }
        }
    }
}
