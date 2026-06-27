use super::{plan_bundle, resolve_slot};
use mcu_protocol::messages::{AxisPieces, PIECE_ENTRY_LEN};
use mcu_protocol::result_codes::RING_FULL;

fn axis(axis_idx: u8, piece_count: u8) -> AxisPieces {
    AxisPieces {
        axis_idx,
        piece_count,
        start_slot: 0,
        new_head: 0,
        pieces_bytes: vec![0u8; usize::from(piece_count) * PIECE_ENTRY_LEN],
    }
}

#[test]
fn single_slave_routes_any_axis_to_slot_zero() {
    assert_eq!(resolve_slot(7, &[3]), Some(0));
    let plan = plan_bundle(&[axis(7, 2)], &[3], |_| 10).unwrap();
    assert_eq!(plan, vec![0]);
}

#[test]
fn multi_slave_routes_each_axis_to_its_configured_slot() {
    let slave_axes = [0u8, 1u8];
    assert_eq!(resolve_slot(0, &slave_axes), Some(0));
    assert_eq!(resolve_slot(1, &slave_axes), Some(1));
    let plan = plan_bundle(&[axis(1, 2), axis(0, 3)], &slave_axes, |_| 10).unwrap();
    assert_eq!(plan, vec![1, 0]);
}

#[test]
fn unmapped_axis_rejects_whole_bundle() {
    let slave_axes = [0u8, 1u8];
    assert_eq!(resolve_slot(2, &slave_axes), None);
    assert_eq!(
        plan_bundle(&[axis(0, 1), axis(2, 1)], &slave_axes, |_| 10),
        Err(RING_FULL)
    );
}

#[test]
fn bundle_exceeding_free_capacity_rejects() {
    let slave_axes = [0u8, 1u8];
    assert_eq!(
        plan_bundle(&[axis(0, 8), axis(1, 2)], &slave_axes, |slot| {
            if slot == 0 {
                4
            } else {
                10
            }
        }),
        Err(RING_FULL)
    );
}

#[test]
fn capacity_accumulates_across_axes_targeting_one_slot() {
    let slave_axes = [5u8, 5u8];
    assert_eq!(
        plan_bundle(&[axis(5, 3), axis(5, 3)], &slave_axes, |_| 5),
        Err(RING_FULL)
    );
    assert_eq!(
        plan_bundle(&[axis(5, 3), axis(5, 2)], &slave_axes, |_| 5).unwrap(),
        vec![0, 0]
    );
}

#[test]
fn short_payload_rejects_before_any_push() {
    let slave_axes = [0u8];
    let mut a = axis(0, 4);
    a.pieces_bytes.truncate(PIECE_ENTRY_LEN);
    assert_eq!(plan_bundle(&[a], &slave_axes, |_| 100), Err(RING_FULL));
}
