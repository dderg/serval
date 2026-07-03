use core::sync::atomic::{AtomicU32, Ordering};

use runtime::fault_sink::FaultSink;
use runtime::motion_core::{get_position_and_velocity, ArmedPiece};
use runtime::piece_ring::{PieceEntry, RingDescriptor};

pub const CLOCK_FREQ_HZ: f32 = 1_000_000_000.0;

pub const AXIS_RING_CAPACITY: usize = 1024;

pub const EC_DC_PERIOD_NS: u32 = 1_000_000;

pub const FAULT_REG_NONE: u32 = 0;

pub const ENGINE_STATE_FAULT: u8 = 3;

pub struct EtherCatFaultSink<'a> {
    reg: &'a AtomicU32,
}

impl core::fmt::Debug for EtherCatFaultSink<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("EtherCatFaultSink")
            .field("reg", &self.reg.load(Ordering::Relaxed))
            .finish()
    }
}

impl FaultSink for EtherCatFaultSink<'_> {
    fn piece_start_in_past(&self, _axis_idx: usize, deficit_us: u32) {
        // Wire layout: bits[31:16] = deficit_us saturated to u16::MAX,
        // bits[15:0] = RUNTIME_ERR_PIECE_START_IN_PAST (-308 as u16 = 0xFECC).
        // Must match host's StatusHeartbeat decoder.
        #[allow(clippy::cast_sign_loss)]
        let code_u16 = (-308_i32 as i16) as u16;
        let deficit_hi16 = (deficit_us.min(u32::from(u16::MAX))) as u16;
        let val = (u32::from(deficit_hi16) << 16) | u32::from(code_u16);
        self.reg.store(val, Ordering::Release);
    }
}

pub struct AxisRing {
    storage: [PieceEntry; AXIS_RING_CAPACITY],
    desc: RingDescriptor,
    armed: Option<ArmedPiece>,
    fault: AtomicU32,
    slot: usize,
}

impl AxisRing {
    pub fn new() -> Self {
        Self::with_slot(0)
    }

    pub fn with_slot(slot: usize) -> Self {
        Self {
            storage: [PieceEntry::zeroed(); AXIS_RING_CAPACITY],
            desc: RingDescriptor::new(0, AXIS_RING_CAPACITY),
            armed: None,
            fault: AtomicU32::new(FAULT_REG_NONE),
            slot,
        }
    }

    pub fn push_entry(&mut self, entry: PieceEntry) -> Result<(), ()> {
        self.desc.push(&mut self.storage, entry)
    }

    pub fn free(&self) -> usize {
        self.desc.ring_depth - self.desc.len()
    }

    pub fn push_from_bytes(&mut self, piece_count: u8, bytes: &[u8]) -> u8 {
        let mut pushed = 0u8;
        let mut rest = bytes;
        for _ in 0..piece_count {
            let (entry, wire_len) = match PieceEntry::parse_wire(rest) {
                Ok(v) => v,
                Err(e) => {
                    tracing::warn!(
                        subsystem = "ethercat",
                        event = "push_from_bytes_malformed_entry",
                        error = ?e,
                        pushed,
                        piece_count,
                        "AxisRing::push_from_bytes: malformed wire entry"
                    );
                    return pushed;
                }
            };
            if self.desc.push(&mut self.storage, entry).is_err() {
                tracing::warn!(
                    subsystem = "ethercat",
                    event = "push_from_bytes_ring_full",
                    pushed,
                    piece_count,
                    "AxisRing::push_from_bytes: ring full"
                );
                break;
            }
            pushed += 1;
            rest = &rest[wire_len..];
        }
        pushed
    }

    pub fn sample(&mut self, now_ns: u64) -> Option<(f32, f32, f32)> {
        let slot = self.slot;
        let AxisRing {
            ref mut armed,
            ref mut desc,
            ref storage,
            ref fault,
            ..
        } = *self;
        let sink = EtherCatFaultSink { reg: fault };
        let (pos, vel) = get_position_and_velocity(
            armed,
            desc,
            storage,
            now_ns,
            EC_DC_PERIOD_NS,
            CLOCK_FREQ_HZ,
            slot,
            &sink,
        )?;
        let p = armed
            .as_ref()
            .expect("sample yielded a value with no armed piece");
        let acc = p.eval_accel(now_ns);
        Some((pos, vel, acc))
    }

    pub fn take_fault(&self) -> Option<u32> {
        let prev = self.fault.swap(FAULT_REG_NONE, Ordering::Acquire);
        if prev != FAULT_REG_NONE {
            Some(prev)
        } else {
            None
        }
    }

    pub fn retired_count(&self) -> u32 {
        self.desc.retired_count()
    }

    pub fn is_empty(&self) -> bool {
        self.desc.is_empty()
    }

    pub fn reset(&mut self) {
        self.desc.drain();
        self.armed = None;
        self.fault.store(FAULT_REG_NONE, Ordering::Relaxed);
    }
}

impl Default for AxisRing {
    fn default() -> Self {
        Self::new()
    }
}

impl core::fmt::Debug for AxisRing {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("AxisRing")
            .field("len", &self.desc.len())
            .field("retired", &self.desc.retired_count())
            .finish()
    }
}

#[cfg(test)]
mod tests;
