use runtime::piece_ring::{PIECE_ENTRY_BYTES, PieceEntry};

#[test]
fn piece_entry_is_48_bytes() {
    assert_eq!(core::mem::size_of::<PieceEntry>(), PIECE_ENTRY_BYTES);
}

#[test]
fn piece_entry_is_8_byte_aligned() {
    assert_eq!(core::mem::align_of::<PieceEntry>(), 8);
}

#[test]
fn piece_entry_constant_endpoints() {
    let entry = PieceEntry {
        start_time: 0,
        duration: 0.001,
        coeff_count: 1,
        coeffs: [5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ..PieceEntry::zeroed()
    };

    assert!(
        (entry.pos_start() - 5.0).abs() < 1e-5,
        "constant pos_start expected 5.0, got {}",
        entry.pos_start()
    );
    assert!(
        (entry.pos_end() - 5.0).abs() < 1e-5,
        "constant pos_end expected 5.0, got {}",
        entry.pos_end()
    );
    assert!(
        entry.vel_start().abs() < 1e-5,
        "constant vel_start expected 0.0, got {}",
        entry.vel_start()
    );
    assert!(
        entry.vel_end().abs() < 1e-5,
        "constant vel_end expected 0.0, got {}",
        entry.vel_end()
    );
}

#[test]
fn piece_entry_linear_endpoints() {
    let entry = PieceEntry {
        start_time: 0,
        duration: 0.01,
        coeff_count: 2,
        coeffs: [0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ..PieceEntry::zeroed()
    };

    assert!(
        entry.pos_start().abs() < 1e-5,
        "linear pos_start expected 0.0, got {}",
        entry.pos_start()
    );
    assert!(
        (entry.pos_end() - 1.0).abs() < 1e-5,
        "linear pos_end expected 1.0, got {}",
        entry.pos_end()
    );
    assert!(
        (entry.vel_start() - 100.0).abs() < 1e-3,
        "linear vel_start expected 100.0, got {}",
        entry.vel_start()
    );
    assert!(
        (entry.vel_end() - 100.0).abs() < 1e-3,
        "linear vel_end expected 100.0, got {}",
        entry.vel_end()
    );
}

#[test]
fn piece_entry_end_time() {
    let entry = PieceEntry {
        start_time: 1000,
        duration: 0.001,
        ..PieceEntry::zeroed()
    };
    let end = entry.end_time(550_000_000.0_f32);
    assert_eq!(end, 551_000, "end_time mismatch: got {end}");
}

use runtime::piece_ring::RingDescriptor;

fn make_rd_storage<const N: usize>() -> [PieceEntry; N] {
    [PieceEntry::zeroed(); N]
}

fn pe(start: u64) -> PieceEntry {
    PieceEntry {
        start_time: start,
        ..PieceEntry::zeroed()
    }
}

#[test]
fn write_slot_lands_at_absolute_index_without_advancing_head() {
    let mut storage = make_rd_storage::<8>();
    let ring = RingDescriptor::new(0, 8);
    ring.write_slot(&mut storage, 5, pe(1234));
    assert_eq!(storage[5].start_time, 1234);
    assert_eq!(ring.len(), 0);
    assert!(ring.is_empty());
}

#[test]
fn commit_head_makes_slots_visible_and_is_monotone() {
    let mut storage = make_rd_storage::<8>();
    let mut ring = RingDescriptor::new(0, 8);
    ring.write_slot(&mut storage, 0, pe(10));
    ring.write_slot(&mut storage, 1, pe(20));
    ring.commit_head(2);
    assert_eq!(ring.len(), 2);
    assert_eq!(ring.peek(&storage).unwrap().start_time, 10);
    ring.commit_head(1); // stale re-send ignored
    assert_eq!(ring.len(), 2);
}

#[test]
fn advance_counter_retires_front_and_increments_retired() {
    let mut storage = make_rd_storage::<4>();
    let mut ring = RingDescriptor::new(0, 4);
    ring.write_slot(&mut storage, 0, pe(10));
    ring.write_slot(&mut storage, 1, pe(20));
    ring.commit_head(2);
    ring.advance_counter();
    assert_eq!(ring.retired_count(), 1);
    assert_eq!(ring.peek(&storage).unwrap().start_time, 20);
    assert_eq!(ring.len(), 1);
}

#[test]
fn empty_full_distinct_via_monotonic_difference() {
    let mut storage = make_rd_storage::<2>();
    let mut ring = RingDescriptor::new(0, 2);
    assert!(ring.is_empty());
    ring.write_slot(&mut storage, 0, pe(1));
    ring.write_slot(&mut storage, 1, pe(2));
    ring.commit_head(2);
    assert!(ring.is_full());
    assert!(!ring.is_empty());
}

#[test]
fn rd_retired_cursor_wraps_after_depth_advances() {
    let mut storage = make_rd_storage::<2>();
    let mut ring = RingDescriptor::new(0, 2);

    ring.write_slot(&mut storage, 0, pe(10));
    ring.write_slot(&mut storage, 1, pe(20));
    ring.commit_head(2);
    assert_eq!(ring.len(), 2);

    ring.advance_counter();
    assert_eq!(ring.retired_count(), 1);
    assert_eq!(ring.tail, 1, "tail must be 1 after first advance");
    assert_eq!(ring.peek(&storage).unwrap().start_time, 20);

    ring.advance_counter();
    assert_eq!(ring.retired_count(), 2);
    assert_eq!(
        ring.tail, 0,
        "tail must wrap to 0 after ring_depth advances"
    );
    assert!(ring.is_empty());
    ring.write_slot(&mut storage, 0, pe(30));
    ring.write_slot(&mut storage, 1, pe(40));
    ring.commit_head(4);
    assert_eq!(ring.len(), 2);
    assert_eq!(ring.peek(&storage).unwrap().start_time, 30);
}

#[test]
fn rd_commit_head_rejects_over_capacity_and_stale_behind_retired() {
    let mut storage = make_rd_storage::<4>();
    let mut ring = RingDescriptor::new(0, 4);
    ring.write_slot(&mut storage, 0, pe(1));
    ring.write_slot(&mut storage, 1, pe(2));
    ring.write_slot(&mut storage, 2, pe(3));
    ring.write_slot(&mut storage, 3, pe(4));

    ring.commit_head(3);
    assert_eq!(ring.len(), 3);

    ring.advance_counter();
    assert_eq!(ring.retired_count(), 1);
    assert_eq!(ring.len(), 2);

    let head_before = ring.head;
    ring.commit_head(6);
    assert_eq!(
        ring.head, head_before,
        "over-capacity commit_head must be rejected"
    );

    ring.commit_head(0);
    assert_eq!(
        ring.head, head_before,
        "behind-retired commit_head must be rejected"
    );

    ring.write_slot(&mut storage, ring.head as usize % 4, pe(50));
    ring.write_slot(&mut storage, (ring.head as usize + 1) % 4, pe(60));
    ring.commit_head(5);
    assert_eq!(
        ring.len(),
        4,
        "commit to exactly ring_depth occupancy must be accepted"
    );
}
