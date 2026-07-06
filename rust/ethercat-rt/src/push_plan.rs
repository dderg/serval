use mcu_protocol::messages::AxisPieces;
use mcu_protocol::result_codes::RING_FULL;
use runtime::piece_ring::PieceEntry;

/// Route an axis block's `axis_idx` to a ring slot. A single-slave endpoint
/// sends every block to slot 0 — the chain has one drive, so the host need not
/// know its axis number. A multi-slave endpoint matches the per-slave axis the
/// claim configured.
pub fn resolve_slot(axis_idx: u8, slave_axes: &[u8]) -> Option<usize> {
    if slave_axes.len() == 1 {
        Some(0)
    } else {
        slave_axes.iter().position(|&a| a == axis_idx)
    }
}

/// Resolve every axis of a bundle to its ring slot and confirm the whole bundle
/// routes and fits before any ring is mutated, returning the per-axis target
/// slots in `axes` order.
///
/// The pump delivers all of one MCU's axes as one bundle and re-sends the whole
/// bundle byte-for-byte on any reject. The EtherCAT ring appends, so it cannot
/// honor the idempotent slot-addressed placement the pump assumes of an MCU
/// ring — a partial push would re-append the axes that already landed on the
/// next re-send. Validating up front keeps the bundle atomic: the caller pushes
/// only on `Ok`, so a rejected bundle touches nothing and is safe to retry.
pub fn plan_bundle(
    axes: &[AxisPieces],
    slave_axes: &[u8],
    free: impl Fn(usize) -> usize,
) -> Result<Vec<usize>, i32> {
    let mut staged = vec![0usize; slave_axes.len()];
    let mut slots = Vec::with_capacity(axes.len());
    for axis in axes {
        let slot = resolve_slot(axis.axis_idx, slave_axes).ok_or(RING_FULL)?;
        let count = usize::from(axis.piece_count);
        // Walk the variable-length entries so a malformed bundle is rejected
        // before any ring mutation.
        let mut rest = axis.pieces_bytes.as_slice();
        for _ in 0..count {
            let (_, wire_len) = PieceEntry::parse_wire(rest).map_err(|_| RING_FULL)?;
            rest = &rest[wire_len..];
        }
        if staged[slot] + count > free(slot) {
            return Err(RING_FULL);
        }
        staged[slot] += count;
        slots.push(slot);
    }
    Ok(slots)
}

#[cfg(test)]
mod tests;
