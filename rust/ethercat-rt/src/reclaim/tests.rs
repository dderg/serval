use mcu_protocol::messages::{AxisPieces, PushPieces};

use super::*;

fn bundle(bytes: usize) -> PushPieces {
    PushPieces {
        axes: vec![AxisPieces {
            axis_idx: 0,
            piece_count: 0,
            start_slot: 0,
            new_head: 0,
            pieces_bytes: vec![0u8; bytes],
        }],
    }
}

#[test]
fn dispose_never_blocks_even_with_ring_overrun() {
    let mut reclaim = Reclaim::spawn();
    let started = std::time::Instant::now();
    for _ in 0..(RECLAIM_RING_CAPACITY * 4) {
        reclaim.dispose(bundle(1024));
    }
    assert!(
        started.elapsed() < Duration::from_secs(1),
        "dispose must never block the caller"
    );
}

#[test]
fn drop_joins_the_janitor_after_draining() {
    let mut reclaim = Reclaim::spawn();
    for _ in 0..16 {
        reclaim.dispose(bundle(64));
    }
    drop(reclaim);
}
