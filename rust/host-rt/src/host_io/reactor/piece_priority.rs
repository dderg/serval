//! Piece frames yield the wire to control traffic: a queued kernel tty
//! out-buffer defers piece-channel writes, while klipper-channel frames
//! write unconditionally.

use super::*;
use crate::host_io::test_harness::ReactorHarness;
use std::sync::mpsc::sync_channel;
use std::time::Duration;

fn identified_harness() -> ReactorHarness {
    let mut h = ReactorHarness::new();
    h.reactor.transport_state.identified = true;
    h
}

fn set_outq(h: &ReactorHarness, bytes: u32) {
    *h.port_handles.outq.lock().unwrap() = bytes;
}

fn submit_piece_call(h: &ReactorHarness) {
    let (completion, _rx) = sync_channel(1);
    h.submission_tx
        .send(ReactorCommand::McuCall {
            channel: mcu_protocol::MCU_CHANNEL_PIECES,
            kind: mcu_protocol::MessageKind::PushPieces,
            body: vec![0xAB; 32],
            completion,
            deadline: std::time::Instant::now() + Duration::from_secs(5),
        })
        .unwrap();
}

#[test]
fn piece_frame_writes_immediately_on_shallow_outq() {
    let mut h = identified_harness();
    submit_piece_call(&h);
    h.tick();
    assert!(
        !h.tx_log().is_empty(),
        "piece frame must reach the wire when the tty queue is shallow"
    );
    assert!(h.reactor.outbound.pending_piece_frames.is_empty());
}

#[test]
fn piece_frame_deferred_while_outq_deep_then_flushed() {
    let mut h = identified_harness();
    set_outq(&h, PIECE_OUTQ_BUDGET_BYTES + 1);
    submit_piece_call(&h);
    h.tick();
    assert!(
        h.tx_log().is_empty(),
        "piece frame must be held back while the tty queue exceeds the budget"
    );
    assert_eq!(h.reactor.outbound.pending_piece_frames.len(), 1);

    set_outq(&h, 0);
    h.tick();
    assert!(
        !h.tx_log().is_empty(),
        "held piece frame must flush once the tty queue drains"
    );
    assert!(h.reactor.outbound.pending_piece_frames.is_empty());
}

#[test]
fn control_frame_bypasses_deep_outq_that_defers_pieces() {
    let mut h = identified_harness();

    set_outq(&h, PIECE_OUTQ_BUDGET_BYTES + 1);
    submit_piece_call(&h);
    h.submission_tx
        .send(ReactorCommand::FireAndForgetTyped {
            payload: vec![0xAA, 0xBB],
        })
        .unwrap();
    h.tick();

    let tx = h.tx_log();
    assert!(
        !tx.is_empty(),
        "control frame must reach the wire despite the deep tty queue"
    );
    assert_eq!(
        h.reactor.outbound.pending_piece_frames.len(),
        1,
        "piece frame must still be held back"
    );
    assert_ne!(tx[0], 0x55, "the written frame must be klipper, not piece");
}
