pub const BACKGROUND_PRIORITY_CLOCK: u64 = u64::MAX;

/// MCU timers compare 32-bit clocks: a waketime more than 2^31 ticks ahead of
/// the MCU's now reads as the past and trips "Timer too close". Timed commands
/// are therefore held until within 2^30 ticks, which stays deep inside the
/// half-range on every supported clock frequency.
pub const MCU_TIMER_HORIZON_TICKS: u64 = 1 << 30;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct NotifyId(u64);

impl NotifyId {
    pub const fn none() -> Self {
        Self(0)
    }

    pub const fn new(id: u64) -> Self {
        Self(id)
    }

    pub fn is_none(&self) -> bool {
        self.0 == 0
    }

    pub fn raw(&self) -> u64 {
        self.0
    }
}

#[derive(Debug, Clone)]
pub struct PassthroughEntry {
    bytes: Vec<u8>,
    min_clock: u64,
    req_clock: u64,
    notify_id: NotifyId,
}

impl PassthroughEntry {
    pub fn new(bytes: Vec<u8>, min_clock: u64, req_clock: u64, notify_id: NotifyId) -> Self {
        Self {
            bytes,
            min_clock,
            req_clock,
            notify_id,
        }
    }

    pub fn bytes(&self) -> &[u8] {
        &self.bytes
    }

    pub fn min_clock(&self) -> u64 {
        self.min_clock
    }

    pub fn req_clock(&self) -> u64 {
        self.req_clock
    }

    pub fn notify_id(&self) -> NotifyId {
        self.notify_id
    }

    pub fn is_background_priority(&self) -> bool {
        self.req_clock == BACKGROUND_PRIORITY_CLOCK
    }

    pub fn emit_clock(&self) -> u64 {
        if self.req_clock == 0 || self.is_background_priority() {
            return self.min_clock;
        }
        self.min_clock
            .max(self.req_clock.saturating_sub(MCU_TIMER_HORIZON_TICKS))
    }
}

#[cfg(test)]
mod tests;
