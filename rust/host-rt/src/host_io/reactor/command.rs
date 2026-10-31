use std::time::Instant;

use crate::host_io::mcu_session::{PendingMcuCall, build_kalico_identify_frame};
use crate::host_io::reactor::{Reactor, ReactorState};
use crate::transport::TransportError;

impl Reactor {
    pub(super) fn handle_command(&mut self, cmd: crate::host_io::ReactorCommand) {
        use crate::host_io::ReactorCommand;
        match cmd {
            ReactorCommand::Call {
                call_id,
                payload,
                expected_response_name,
                completion,
                deadline,
            } => self.handle_call(
                call_id,
                payload,
                expected_response_name,
                completion,
                deadline,
            ),
            ReactorCommand::Abandon(call_id) => {
                self.awaiting_response.mark_abandoned(call_id);
            }
            ReactorCommand::Shutdown => {
                self.state = ReactorState::Closed;
                self.closed_via_shutdown = true;
            }
            ReactorCommand::MarkExpectedDisconnect => self.handle_mark_expected_disconnect(),
            ReactorCommand::AttachHeartbeatCallback(cb) => {
                self.event_dispatcher.heartbeat_callback = Some(cb);
            }
            ReactorCommand::SetMcuLogHook(hook) => {
                self.event_dispatcher.set_mcu_log_hook(hook);
            }
            ReactorCommand::SubscribeFault { sender, reply } => {
                let result = self.event_dispatcher.fault_latch.subscribe(sender);
                let _ = reply.send(result);
            }
            ReactorCommand::SubscribeRuntimeEvents {
                priority,
                bulk,
                reply,
            } => self.handle_subscribe_runtime_events(priority, bulk, reply),
            ReactorCommand::FireAndForget { payload } => self.handle_fire_and_forget(payload),
            ReactorCommand::FireAndForgetBatch {
                payloads,
                reserved_blocks,
                enqueued_at,
            } => {
                let waited = enqueued_at.elapsed();
                if waited > std::time::Duration::from_millis(20)
                    && self.last_channel_wait_warn.elapsed().as_millis() >= 500
                {
                    self.last_channel_wait_warn = std::time::Instant::now();
                    tracing::warn!(
                        subsystem = "mcu-comms",
                        event = "channel_wait_high",
                        mcu = %self.mcu_label,
                        waited_ms = waited.as_millis() as u64,
                        blocks = payloads.len(),
                        "batch sat this long in the submission channel before the reactor took it"
                    );
                }
                self.handle_fire_and_forget_batch(&payloads, reserved_blocks)
            }
            ReactorCommand::McuIdentify {
                completion,
                deadline: _,
            } => self.handle_mcu_identify(completion),
            ReactorCommand::McuCall {
                kind,
                body,
                completion,
                deadline,
            } => self.handle_mcu_call(kind, body, completion, deadline),
            ReactorCommand::GetClockAndDeliver => self.handle_get_clock_and_deliver(),
            ReactorCommand::Noop => {}
            ReactorCommand::RegisterInterceptor {
                msg_name,
                oid,
                callback,
                reply,
            } => {
                let id = self.interceptors.register(msg_name, oid, callback);
                let _ = reply.send(id);
            }
            ReactorCommand::UnregisterInterceptor { id } => {
                self.interceptors.unregister(id);
            }
        }
    }

    fn handle_call(
        &mut self,
        call_id: u64,
        payload: Vec<u8>,
        expected_response_name: String,
        completion: std::sync::mpsc::SyncSender<
            Result<crate::transport::MessageParams, TransportError>,
        >,
        deadline: Instant,
    ) {
        tracing::debug!(
            subsystem = "mcu-comms",
            event = "call",
            call_id,
            resp = %expected_response_name,
            payload_len = payload.len(),
            unacked = self.unacked_window.len(),
            pending_sub = self.outbound.pending_submissions.len(),
            state = ?self.state,
            "Call"
        );
        if let Err(e) = self.dispatch_submission(
            call_id,
            payload,
            expected_response_name,
            completion.clone(),
            deadline,
        ) {
            self.close_if_io_fault("handle_command/call", &e);
            let _ = completion.send(Err(e));
        }
    }

    fn handle_mark_expected_disconnect(&mut self) {
        tracing::info!(
            subsystem = "mcu-comms",
            event = "expected_disconnect",
            transport_pending = self.transport_state.pending.len(),
            await_n = self.awaiting_response.len(),
            unacked_n = self.unacked_window.len(),
            "MarkExpectedDisconnect received"
        );
        self.closed_via_shutdown = true;
    }

    fn handle_subscribe_runtime_events(
        &mut self,
        priority: std::sync::mpsc::SyncSender<crate::host_io::runtime_events::RuntimeEvent>,
        bulk: std::sync::mpsc::SyncSender<crate::host_io::runtime_events::RuntimeEvent>,
        reply: std::sync::mpsc::SyncSender<Result<(), crate::transport::SubscribeError>>,
    ) {
        let result = self
            .event_dispatcher
            .runtime_event_dispatcher
            .subscribe(priority, bulk);
        let _ = reply.send(result);
    }

    fn handle_fire_and_forget(&mut self, payload: Vec<u8>) {
        if let Err(e) = self.dispatch_fire_and_forget(payload, false) {
            tracing::error!(
                subsystem = "mcu-comms",
                event = "fire_and_forget_send_error",
                error = %e,
                "FireAndForget: send error"
            );
            self.close_if_io_fault("handle_command/fire_and_forget", &e);
        }
    }

    fn handle_fire_and_forget_batch(&mut self, payloads: &[Vec<u8>], reserved_blocks: usize) {
        let blocks = match crate::host_io::wire::pack_blocks(payloads) {
            Ok(blocks) => blocks,
            Err(detail) => {
                self.outbound.fire_and_forget_depth.release(reserved_blocks);
                tracing::error!(
                    subsystem = "mcu-comms",
                    event = "fire_and_forget_batch_pack_error",
                    detail = %detail,
                    "FireAndForgetBatch: refusing to frame an unpackable burst"
                );
                return;
            }
        };
        for block in blocks {
            if let Err(e) = self.dispatch_fire_and_forget(block, false) {
                self.outbound.fire_and_forget_depth.release(reserved_blocks);
                tracing::error!(
                    subsystem = "mcu-comms",
                    event = "fire_and_forget_batch_send_error",
                    error = %e,
                    "FireAndForgetBatch: block write failed; abandoning the rest of the burst \
                     rather than putting later blocks on the wire ahead of it"
                );
                self.close_if_io_fault("handle_command/fire_and_forget_batch", &e);
                return;
            }
        }
        self.outbound.fire_and_forget_depth.release(reserved_blocks);
    }

    fn handle_mcu_identify(
        &mut self,
        completion: std::sync::mpsc::SyncSender<
            Result<crate::host_io::mcu_session::IdentifyOutcome, TransportError>,
        >,
    ) {
        let cid = self.transport_state.allocate_correlation_id();
        let frame = build_kalico_identify_frame(cid);
        if self.transport_state.identify_pending.is_some() {
            let _ = completion.send(Err(TransportError::Backpressure));
            return;
        }
        self.transport_state.identify_pending = Some(completion);
        if let Err(e) = self.write_frame(&frame) {
            self.close_if_io_fault("handle_command/mcu_identify", &e);
            if let Some(c) = self.transport_state.identify_pending.take() {
                let _ = c.send(Err(e));
            }
        }
    }

    fn handle_mcu_call(
        &mut self,
        kind: mcu_protocol::MessageKind,
        body: Vec<u8>,
        completion: std::sync::mpsc::SyncSender<
            Result<crate::host_io::mcu_session::McuCallOutcome, TransportError>,
        >,
        deadline: Instant,
    ) {
        if !self.transport_state.identified {
            let _ = completion.send(Err(TransportError::Parse(
                "kalico transport not yet identified".into(),
            )));
            return;
        }
        let cid = self.transport_state.allocate_correlation_id();
        let frame = mcu_transport::wire_helpers::control_frame(kind, cid, &body);
        self.transport_state.pending.insert(
            cid,
            PendingMcuCall {
                completion,
                deadline,
            },
        );
        if let Err(e) = self.write_frame(&frame) {
            self.close_if_io_fault("handle_command/mcu_call", &e);
            if let Some(p) = self.transport_state.pending.remove(&cid) {
                let _ = p.completion.send(Err(e));
            }
        }
    }

    fn handle_get_clock_and_deliver(&mut self) {
        match self.parser.encode("get_clock") {
            Ok(payload) => {
                // The RAW send stamp is captured inside
                // dispatch_fire_and_forget at the actual wire write —
                // never here, where the frame may still queue behind a
                // busy link for milliseconds.
                if let Err(e) = self.dispatch_fire_and_forget(payload, true) {
                    tracing::error!(
                        subsystem = "mcu-comms",
                        event = "get_clock_async_send_error",
                        error = %e,
                        "GetClockAndDeliver dispatch failed"
                    );
                    self.close_if_io_fault("handle_command/get_clock_and_deliver", &e);
                }
            }
            Err(e) => {
                tracing::error!(
                    subsystem = "mcu-comms",
                    event = "get_clock_async_encode_error",
                    error = ?e,
                    "GetClockAndDeliver: encode 'get_clock' failed"
                );
            }
        }
    }
}
