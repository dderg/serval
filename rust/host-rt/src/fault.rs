use std::sync::mpsc::SyncSender;

use crate::host_io::runtime_events::FaultEvent as RuntimeFaultEvent;
use crate::transport::SubscribeError;

#[derive(Debug, Default)]
pub struct FaultLatch {
    pub cell: Option<RuntimeFaultEvent>,
    pub subscriber: Option<SyncSender<RuntimeFaultEvent>>,
}

impl FaultLatch {
    pub fn dispatch(&mut self, event: RuntimeFaultEvent) {
        let upgrade = self
            .cell
            .as_ref()
            .map(|c| c.synthesized && !event.synthesized)
            .unwrap_or(false);
        if self.cell.is_none() || upgrade {
            self.cell = Some(event.clone());
            if let Some(tx) = &self.subscriber {
                let _ = tx.send(event);
            }
        }
    }

    pub fn subscribe(&mut self, tx: SyncSender<RuntimeFaultEvent>) -> Result<(), SubscribeError> {
        if self.subscriber.is_some() {
            return Err(SubscribeError::AlreadySubscribed { channel: "fault" });
        }
        if let Some(latched) = &self.cell {
            let _ = tx.send(latched.clone());
        }
        self.subscriber = Some(tx);
        Ok(())
    }
}

#[cfg(test)]
mod fault_latch_tests;
