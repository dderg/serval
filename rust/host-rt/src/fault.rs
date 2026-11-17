use crate::host_io::runtime_events::FaultEvent as RuntimeFaultEvent;

#[derive(Debug, Default)]
pub struct FaultLatch {
    pub cell: Option<RuntimeFaultEvent>,
}

impl FaultLatch {
    pub fn dispatch(&mut self, event: RuntimeFaultEvent) {
        let upgrade = self
            .cell
            .as_ref()
            .map(|c| c.synthesized && !event.synthesized)
            .unwrap_or(false);
        if self.cell.is_none() || upgrade {
            self.cell = Some(event);
        }
    }
}

#[cfg(test)]
mod fault_latch_tests;
