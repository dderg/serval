use crate::host_io::mcu_session::{McuDispatchResult, dispatch_mcu_frame};
use crate::host_io::reactor::{READ_TIMEOUT, Reactor, ZERO_BYTE_DEBOUNCE};
use crate::transport::TransportError;
use mcu_transport::demux::{Frame, KlipperFrame, PollOutcome};
use runtime::error::FaultCode;

impl Reactor {
    pub(crate) fn handle_inbound_frame(
        &mut self,
        frame: KlipperFrame,
    ) -> Result<(), TransportError> {
        let bytes = frame.bytes();
        if bytes.len() < crate::host_io::wire::MESSAGE_MIN {
            return Ok(());
        }
        let wire_seq_nibble = bytes[1] & 0x0F;
        if bytes.len() == crate::host_io::wire::MESSAGE_MIN {
            self.handle_ack_nak(wire_seq_nibble)?;
            return Ok(());
        }
        let rseq =
            crate::host_io::wire::decode_absolute(self.seq_window.receive_seq, wire_seq_nibble);
        let rseq_jump = rseq.saturating_sub(self.seq_window.receive_seq);
        if rseq_jump > 1 {
            tracing::warn!(
                subsystem = "mcu-comms",
                event = "rx_seq_jump",
                receive_seq_prev = self.seq_window.receive_seq,
                receive_seq_new = rseq,
                jump = rseq_jump,
                "receive_seq jumped >1: MCU dropped a response or we missed a frame"
            );
        }
        if rseq != self.seq_window.receive_seq {
            self.update_receive_seq(rseq)?;
        }
        let decoded = match self.parser.decode(bytes) {
            Ok(d) => d,
            Err(e) => {
                tracing::warn!(
                    subsystem = "mcu-comms",
                    event = "decode_error",
                    error = ?e,
                    bytes_len = bytes.len(),
                    first16 = ?&bytes[..bytes.len().min(16)],
                    "frame decode error"
                );
                return Ok(());
            }
        };

        match decoded {
            crate::host_io::parser::DecodedFrame::Response { name, params } => {
                if name == "shutdown" || name == "is_shutdown" {
                    self.fail_pending_on_mcu_shutdown(&name, &params);
                }
                let await_len_before = self.awaiting_response.len();
                if let Some(idx) = self.awaiting_response.find_match(&name) {
                    let entry = self.awaiting_response.remove(idx);
                    tracing::debug!(
                        subsystem = "mcu-comms",
                        event = "await_response",
                        tid = ?std::thread::current().id(),
                        %name,
                        idx,
                        await_len = await_len_before,
                        matched_call_id = entry.call_id,
                        matched_seq = entry.seq,
                        "solicited response matched"
                    );
                    let mut params = params;
                    params.sent_time_raw = entry.sent_time_raw;
                    params.recv_time_raw = crate::clock::monotonic_raw_secs();
                    let _ = entry.completion.send(Ok(params));
                } else {
                    let oid = oid_from_params(&params);
                    if name.contains("software_trip") || name.contains("trsync_state") {
                        tracing::debug!(
                            subsystem = "mcu-comms",
                            event = "unsolicited_frame",
                            %name,
                            ?oid,
                            interceptor_count = self.interceptors.entry_count(),
                            params = ?params,
                            "unsolicited frame (software_trip/trsync_state)"
                        );
                    } else {
                        tracing::debug!(
                            subsystem = "mcu-comms",
                            event = "unsolicited_frame",
                            %name,
                            ?oid,
                            interceptor_count = self.interceptors.entry_count(),
                            "unsolicited frame"
                        );
                    }
                    if name == "clock" {
                        if let Some(sent_raw) = self.pending_clock_sent_raw.take() {
                            let recv_raw = crate::clock::monotonic_raw_secs();
                            let mut stamped = params.clone();
                            stamped.sent_time_raw = sent_raw;
                            stamped.recv_time_raw = recv_raw;
                            let event =
                                crate::host_io::runtime_events::RuntimeEvent::PassthroughResponse {
                                    name,
                                    params: stamped,
                                };
                            self.dispatch_runtime_event(event);
                            return Ok(());
                        }
                    }

                    self.interceptors.dispatch(&name, oid, &params);

                    tracing::debug!(
                        subsystem = "mcu-comms",
                        event = "unsolicited_no_interceptor",
                        tid = ?std::thread::current().id(),
                        %name,
                        await_len = await_len_before,
                        "unsolicited frame with no interceptor match"
                    );
                    let event = crate::host_io::runtime_events::RuntimeEvent::PassthroughResponse {
                        name,
                        params,
                    };
                    self.dispatch_runtime_event(event);
                }
            }
            crate::host_io::parser::DecodedFrame::Output { name, params } => {
                let oid = oid_from_params(&params);
                let interceptor_count = self.interceptors.entry_count();
                if interceptor_count > 0 {
                    tracing::debug!(
                        subsystem = "trip-relay",
                        event = "output_frame_intercepted",
                        %name,
                        ?oid,
                        interceptor_count,
                        "output frame dispatched through interceptors"
                    );
                }
                self.interceptors.dispatch(&name, oid, &params);
                let event = crate::host_io::runtime_events::RuntimeEvent::lift(&name, params);
                self.dispatch_runtime_event(event);
            }
        }
        Ok(())
    }

    fn dispatch_runtime_event(&mut self, event: crate::host_io::runtime_events::RuntimeEvent) {
        self.event_dispatcher.dispatch(event);
    }
}

fn oid_from_params(params: &crate::transport::MessageParams) -> Option<u32> {
    params.fields.get("oid").and_then(|v| match v {
        crate::transport::MessageValue::U32(n) => Some(*n),
        crate::transport::MessageValue::I32(n) => Some(*n as u32),
        _ => None,
    })
}

impl Reactor {
    pub(crate) fn handle_kalico_frame(&mut self, channel: u8, payload: &[u8]) {
        match dispatch_mcu_frame(&mut self.transport_state, channel, payload) {
            McuDispatchResult::Handled | McuDispatchResult::Ignored => {}
            McuDispatchResult::Event(ev) => {
                self.dispatch_runtime_event(ev);
            }
        }
    }
}

impl Reactor {
    pub(super) fn poll_serial(&mut self) {
        let t0 = std::time::Instant::now();
        let deadline = self.clock.now() + READ_TIMEOUT;
        let outcome = self.io.poll_frames_until(deadline);
        let dt = t0.elapsed();
        if dt > std::time::Duration::from_millis(5) {
            let label: &'static str = match &outcome {
                Ok(PollOutcome::Frames { .. }) => "Frames",
                Ok(PollOutcome::Timeout) => "Timeout",
                Ok(PollOutcome::PhantomZero) => "PhantomZero",
                Err(_) => "Err",
            };
            tracing::debug!(
                subsystem = "mcu-comms",
                event = "slow_poll",
                dt_ms = dt.as_secs_f64() * 1000.0,
                outcome = label,
                "poll_serial exceeded 5ms"
            );
        }
        match outcome {
            Ok(PollOutcome::Frames { frames, errors }) => {
                self.zero_byte_first_seen = None;
                self.zero_byte_consec = 0;
                if !frames.is_empty() || !errors.is_empty() {
                    self.last_recv_time = self.clock.now();
                }
                for e in errors {
                    tracing::warn!(
                        subsystem = "mcu-comms",
                        event = "kalico_stream_error",
                        mcu = %self.mcu_label,
                        error = %e,
                        "kalico stream error"
                    );
                }
                for f in frames {
                    match f {
                        Frame::Klipper(kf) => {
                            if let Err(e) = self.handle_inbound_frame(kf) {
                                tracing::error!(
                                    subsystem = "mcu-comms",
                                    event = "inbound_frame_fatal",
                                    mcu = %self.mcu_label,
                                    error = ?e,
                                    "inbound frame handling failed (ack/retransmit write?) — \
                                     closing transport"
                                );
                                return;
                            }
                        }
                        Frame::Kalico { channel, payload } => {
                            self.handle_kalico_frame(channel, &payload);
                        }
                    }
                }
            }
            Ok(PollOutcome::Timeout) => {
                self.zero_byte_first_seen = None;
                self.zero_byte_consec = 0;
            }
            Ok(PollOutcome::PhantomZero) => {
                self.zero_byte_consec = self.zero_byte_consec.saturating_add(1);
                let now = self.clock.now();
                let first = *self.zero_byte_first_seen.get_or_insert(now);
                if now.duration_since(first) >= ZERO_BYTE_DEBOUNCE {
                    let silence_ms = now.duration_since(self.last_recv_time).as_millis();
                    let since_write_ms = now.duration_since(self.last_write_time).as_millis();
                    tracing::warn!(
                        subsystem = "mcu-comms",
                        event = "usb_drop_phantom_zero",
                        mcu = %self.mcu_label,
                        silence_ms = %silence_ms,
                        since_write_ms = %since_write_ms,
                        consec_zero = self.zero_byte_consec,
                        debounce = ?ZERO_BYTE_DEBOUNCE,
                        "[usb-drop] PhantomZero (Ok(0) past debounce window)"
                    );
                    self.close_with_host_fault(FaultCode::HostDisconnect, 0);
                }
            }
            Err(e) => {
                let now = self.clock.now();
                let silence_ms = now.duration_since(self.last_recv_time).as_millis();
                let since_write_ms = now.duration_since(self.last_write_time).as_millis();
                let (os_errno, io_kind) = match &e {
                    TransportError::Io(io) => (io.raw_os_error(), Some(io.kind())),
                    _ => (None, None),
                };
                tracing::warn!(
                    subsystem = "mcu-comms",
                    event = "usb_drop_poll_error",
                    mcu = %self.mcu_label,
                    silence_ms = %silence_ms,
                    since_write_ms = %since_write_ms,
                    consec_zero = self.zero_byte_consec,
                    os_errno = ?os_errno,
                    io_kind = ?io_kind,
                    error = ?e,
                    "[usb-drop] poll error"
                );
                self.close_with_host_fault(FaultCode::HostDisconnect, 0);
            }
        }
    }
}
