use std::sync::Arc;
use std::sync::mpsc::{SyncSender, TrySendError};

use arc_swap::ArcSwap;

use crate::fault::FaultLatch;
use crate::host_io::runtime_events::{CreditFreedEvent, McuLogEvent, RuntimeEvent, StatusEvent};

/// The bulk lane is bounded exactly like the priority lane: the reactor thread
/// also drives the wire, so a stalled subscriber must never be allowed to
/// block it or to grow the queue without bound. Excess bulk samples are
/// dropped and counted.
#[derive(Debug, Default)]
pub struct RuntimeEventDispatcher {
    priority: Option<SyncSender<RuntimeEvent>>,
    bulk: Option<SyncSender<RuntimeEvent>>,
    priority_overflow: bool,
    bulk_overflow: bool,
    bulk_dropped: u64,
}

impl RuntimeEventDispatcher {
    pub fn dispatch(&mut self, event: RuntimeEvent) {
        if event.is_bulk_data() {
            self.send_bulk(event);
        } else {
            self.send_priority(event);
        }
    }

    fn send_bulk(&mut self, event: RuntimeEvent) {
        let Some(tx) = self.bulk.as_ref() else {
            return;
        };
        match tx.try_send(event) {
            Ok(()) => {
                if self.bulk_overflow {
                    tracing::warn!(
                        subsystem = "mcu-comms",
                        event = "runtime_event_subscriber_recovered",
                        lane = "bulk",
                        dropped_total = self.bulk_dropped,
                        "bulk runtime-event subscriber is draining again"
                    );
                }
                self.bulk_overflow = false;
            }
            Err(TrySendError::Full(event)) => {
                self.bulk_dropped += 1;
                if !self.bulk_overflow {
                    tracing::error!(
                        subsystem = "mcu-comms",
                        event = "runtime_event_subscriber_overflow",
                        lane = "bulk",
                        dropped = runtime_event_name(&event),
                        dropped_total = self.bulk_dropped,
                        "bulk runtime-event subscriber stalled; dropping rather than stalling the \
                         reactor"
                    );
                }
                self.bulk_overflow = true;
            }
            Err(TrySendError::Disconnected(_)) => {
                self.bulk = None;
                self.bulk_overflow = false;
            }
        }
    }

    fn send_priority(&mut self, event: RuntimeEvent) {
        let Some(tx) = self.priority.as_ref() else {
            return;
        };
        match tx.try_send(event) {
            Ok(()) => self.priority_overflow = false,
            Err(TrySendError::Full(event)) => {
                if !self.priority_overflow {
                    tracing::error!(
                        subsystem = "mcu-comms",
                        event = "runtime_event_subscriber_overflow",
                        lane = "priority",
                        dropped = runtime_event_name(&event),
                        "runtime-event subscriber overflow; dropping"
                    );
                }
                self.priority_overflow = true;
            }
            Err(TrySendError::Disconnected(_)) => {
                self.priority = None;
                self.priority_overflow = false;
            }
        }
    }

    pub fn subscribe(
        &mut self,
        priority: SyncSender<RuntimeEvent>,
        bulk: SyncSender<RuntimeEvent>,
    ) -> Result<(), crate::transport::SubscribeError> {
        if self.priority.is_some() || self.bulk.is_some() {
            return Err(crate::transport::SubscribeError::AlreadySubscribed {
                channel: "runtime_event",
            });
        }
        self.priority = Some(priority);
        self.bulk = Some(bulk);
        self.priority_overflow = false;
        self.bulk_overflow = false;
        Ok(())
    }
}

fn runtime_event_name(event: &RuntimeEvent) -> &str {
    match event {
        RuntimeEvent::CreditFreed(_) => "credit_freed",
        RuntimeEvent::Fault(_) => "fault",
        RuntimeEvent::Status(_) => "status",
        RuntimeEvent::EndstopTrip(_) => "endstop_trip",
        RuntimeEvent::McuLog(_) => "mcu_log",
        RuntimeEvent::Heartbeat { .. } => "heartbeat",
        RuntimeEvent::UnknownOutput { .. } => "unknown_output",
        RuntimeEvent::PassthroughResponse { name, .. } => name,
    }
}

// Manual Debug — heartbeat_callback and mcu_log_hook are trait objects and cannot derive.
impl std::fmt::Debug for EventDispatcher {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("EventDispatcher")
            .field("fault_latch", &self.fault_latch)
            .field("status_snapshot", &"<ArcSwap<StatusEvent>>")
            .field("runtime_event_dispatcher", &self.runtime_event_dispatcher)
            .field("status_retired_watermark", &self.status_retired_watermark)
            .field(
                "heartbeat_callback",
                if self.heartbeat_callback.is_some() {
                    &"Some(<fn>)"
                } else {
                    &"None"
                },
            )
            .field(
                "mcu_log_hook",
                if self.mcu_log_hook.is_some() {
                    &"Some(<fn>)"
                } else {
                    &"None"
                },
            )
            .finish_non_exhaustive()
    }
}

pub struct EventDispatcher {
    pub fault_latch: FaultLatch,
    pub status_snapshot: Arc<ArcSwap<StatusEvent>>,
    pub runtime_event_dispatcher: RuntimeEventDispatcher,
    status_retired_watermark: u32,
    pub heartbeat_callback: Option<Arc<dyn Fn(&[u32], &[u64]) + Send + Sync>>,
    pub mcu_log_hook: Option<Box<dyn Fn(McuLogEvent) + Send + Sync>>,
}

impl EventDispatcher {
    pub fn new(status_snapshot: Arc<ArcSwap<StatusEvent>>) -> Self {
        Self {
            fault_latch: FaultLatch::default(),
            status_snapshot,
            runtime_event_dispatcher: RuntimeEventDispatcher::default(),
            status_retired_watermark: 0,
            heartbeat_callback: None,
            mcu_log_hook: None,
        }
    }

    pub fn set_mcu_log_hook<F>(&mut self, f: F)
    where
        F: Fn(McuLogEvent) + Send + Sync + 'static,
    {
        self.mcu_log_hook = Some(Box::new(f));
    }

    pub fn dispatch(&mut self, event: RuntimeEvent) {
        match event {
            RuntimeEvent::CreditFreed(e) => {
                self.runtime_event_dispatcher
                    .dispatch(RuntimeEvent::CreditFreed(e));
            }
            RuntimeEvent::Fault(e) => {
                let signed_code = e.fault_code as i16 as i32;
                tracing::warn!(
                    subsystem = "mcu-comms",
                    event = "fault_event_received",
                    fault_code = signed_code,
                    wire_u16 = e.fault_code,
                    fault_detail = e.fault_detail,
                    segment_id = e.segment_id,
                    synthesized = e.synthesized,
                    "[KALICO-FAULT] received FaultEvent (segment_id is the -311 \
                     stacked PC = addr2line target: the instruction the \
                     interrupted context was about to execute, i.e. the code \
                     holding the CPU/PRIMASK across the late tick; 0 for non-311 \
                     faults; see runtime_contract::error::FaultCode: -308=PieceStartInPast \
                     -309=RingFull -310=StepsPerSampleExceeded \
                     -311=TickIntervalExceeded -302=MathNonFinite \
                     -303=PieceAdvanceUnderflow -300=StepQueueOverflow)"
                );
                self.fault_latch.dispatch(e.clone());
                self.runtime_event_dispatcher
                    .dispatch(RuntimeEvent::Fault(e));
            }
            RuntimeEvent::Status(e) => {
                let synth_credit = self.handle_status_frame(&e);
                self.runtime_event_dispatcher
                    .dispatch(RuntimeEvent::Status(e));
                if let Some(c) = synth_credit {
                    self.dispatch(RuntimeEvent::CreditFreed(c));
                }
            }
            RuntimeEvent::Heartbeat {
                retired_counts,
                playback_clocks,
            } => {
                if let Some(cb) = &self.heartbeat_callback {
                    cb(&retired_counts, &playback_clocks);
                }
            }
            RuntimeEvent::EndstopTrip(_)
            | RuntimeEvent::UnknownOutput { .. }
            | RuntimeEvent::PassthroughResponse { .. } => {
                self.runtime_event_dispatcher.dispatch(event);
            }
            RuntimeEvent::McuLog(e) => {
                if let Some(hook) = &self.mcu_log_hook {
                    hook(e.clone());
                }
                self.runtime_event_dispatcher
                    .dispatch(RuntimeEvent::McuLog(e));
            }
        }
    }

    fn handle_status_frame(&mut self, frame: &StatusEvent) -> Option<CreditFreedEvent> {
        const ENGINE_STATUS_FAULT: u8 = 3;
        const Q_N_MINUS_1: u8 = 7;

        self.status_snapshot.store(Arc::new(frame.clone()));

        if frame.engine_status == ENGINE_STATUS_FAULT && self.fault_latch.cell.is_none() {
            let synthesized = crate::host_io::runtime_events::FaultEvent {
                fault_code: frame.last_fault,
                fault_detail: frame.fault_detail,
                segment_id: frame.current_segment_id,
                synthesized: true,
            };
            self.fault_latch.dispatch(synthesized);
        }

        let watermark = frame.retired_through_segment_id;
        #[allow(clippy::cast_possible_wrap)]
        let advanced = (watermark.wrapping_sub(self.status_retired_watermark) as i32) > 0;
        if advanced {
            self.status_retired_watermark = watermark;
            let free_slots = Q_N_MINUS_1.saturating_sub(frame.queue_depth);
            Some(CreditFreedEvent {
                retired_through_segment_id: watermark,
                free_slots,
            })
        } else {
            None
        }
    }
}

#[cfg(test)]
mod dispatch_tests;
