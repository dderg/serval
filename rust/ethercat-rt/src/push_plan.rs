use mcu_protocol::messages::AxisPieces;
use mcu_protocol::result_codes::RING_FULL;
use runtime::piece_ring::PieceEntry;

/// Route an axis block's `axis_idx` to its ring slots. A single-slave endpoint
/// sends every block to slot 0 — the chain has one drive, so the host need not
/// know its axis number. A multi-slave endpoint matches every per-slave axis
/// the claim configured: two drives sharing a belt both claim the axis, and
/// each drive's ring receives the full piece stream (AWD fan-out).
pub fn resolve_slots(axis_idx: u8, slave_axes: &[u8]) -> Vec<usize> {
    if slave_axes.len() == 1 {
        vec![0]
    } else {
        slave_axes
            .iter()
            .enumerate()
            .filter_map(|(slot, &a)| (a == axis_idx).then_some(slot))
            .collect()
    }
}

/// Resolve every axis of a bundle to its ring slots and confirm the whole
/// bundle routes and fits before any ring is mutated, returning the per-axis
/// target slots in `axes` order.
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
) -> Result<Vec<Vec<usize>>, i32> {
    let mut staged = vec![0usize; slave_axes.len()];
    let mut slots = Vec::with_capacity(axes.len());
    for axis in axes {
        let axis_slots = resolve_slots(axis.axis_idx, slave_axes);
        if axis_slots.is_empty() {
            return Err(RING_FULL);
        }
        let count = usize::from(axis.piece_count);
        // Walk the variable-length entries so a malformed bundle is rejected
        // before any ring mutation.
        let mut rest = axis.pieces_bytes.as_slice();
        for _ in 0..count {
            let (_, wire_len) = PieceEntry::parse_wire(rest).map_err(|_| RING_FULL)?;
            rest = &rest[wire_len..];
        }
        for &slot in &axis_slots {
            if staged[slot] + count > free(slot) {
                return Err(RING_FULL);
            }
            staged[slot] += count;
        }
        slots.push(axis_slots);
    }
    Ok(slots)
}

#[cfg(test)]
mod tests;
