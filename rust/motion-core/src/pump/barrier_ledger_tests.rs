use super::*;

const OID: u32 = 5;

#[test]
fn modular_ordering_survives_the_sequence_wrap() {
    assert!(barrier_seq_after(2, 1));
    assert!(!barrier_seq_after(1, 2));
    assert!(barrier_seq_after(0, u32::MAX));
    assert!(barrier_seq_before(u32::MAX, 0));
    assert!(
        !barrier_seq_after(7, 7),
        "equal is neither before nor after"
    );
    assert!(barrier_seq_covers(7, 7));
    assert!(barrier_seq_covers(9, 7), "an ack covers everything earlier");
    assert!(!barrier_seq_covers(7, 9));
}

#[test]
fn the_seed_is_odd_so_a_fresh_run_cannot_be_covered_by_zero() {
    assert_eq!(barrier_seq_seed() % 2, 1);
}

#[test]
fn issue_hands_out_consecutive_receipts_per_oid() {
    let mut ledger = BarrierLedger::with_seed(100);
    let first = ledger.issue(OID);
    let second = ledger.issue(OID);
    let other = ledger.issue(OID + 1);
    assert_eq!(first, BarrierId { oid: OID, seq: 100 });
    assert_eq!(second, BarrierId { oid: OID, seq: 101 });
    assert_eq!(
        other,
        BarrierId {
            oid: OID + 1,
            seq: 100
        }
    );
}

#[test]
fn an_ack_covers_every_earlier_receipt_on_that_oid() {
    let mut ledger = BarrierLedger::with_seed(100);
    let first = ledger.issue(OID);
    let second = ledger.issue(OID);
    assert!(!ledger.is_acked(first));
    ledger.record_ack(OID, second.seq).expect("issued");
    assert!(ledger.is_acked(first), "the mcu acks in queue order");
    assert!(ledger.is_acked(second));
}

#[test]
fn an_ack_for_an_unknown_oid_faults() {
    let mut ledger = BarrierLedger::with_seed(100);
    assert_eq!(ledger.record_ack(OID, 100), Err(AckFault::Unknown));
}

#[test]
fn an_ack_ahead_of_what_the_host_issued_faults() {
    let mut ledger = BarrierLedger::with_seed(100);
    ledger.issue(OID);
    assert_eq!(
        ledger.record_ack(OID, 101),
        Err(AckFault::Unissued { issued: 101 })
    );
}

#[test]
fn an_ack_walking_the_high_water_mark_backwards_faults() {
    let mut ledger = BarrierLedger::with_seed(100);
    ledger.issue(OID);
    ledger.issue(OID);
    ledger.issue(OID);
    ledger.record_ack(OID, 101).expect("issued");
    assert_eq!(
        ledger.record_ack(OID, 100),
        Err(AckFault::Regressed { high_water: 101 })
    );
}

#[test]
fn ordered_acks_reject_skips_without_covering_the_missing_receipt() {
    let mut ledger = BarrierLedger::with_seed(100);
    let first = ledger.issue(OID);
    let second = ledger.issue(OID);
    assert_eq!(
        ledger.record_ordered_ack(OID, second.seq),
        Err(AckFault::OutOfOrder {
            expected: first.seq
        })
    );
    assert!(!ledger.is_acked(first));
    assert_eq!(ledger.record_ordered_ack(OID, first.seq), Ok(true));
    assert_eq!(ledger.record_ordered_ack(OID, second.seq), Ok(true));
}

#[test]
fn only_ordered_acks_ignore_pre_seed_and_covered_replays() {
    let mut ledger = BarrierLedger::with_seed(100);
    let id = ledger.issue(OID);
    assert_eq!(ledger.record_ordered_ack(OID, 99), Ok(false));
    assert!(!ledger.is_acked(id));
    assert_eq!(ledger.record_ordered_ack(OID, id.seq), Ok(true));
    assert_eq!(ledger.record_ordered_ack(OID, id.seq), Ok(false));
    assert_eq!(
        ledger.record_ack(OID, id.seq),
        Err(AckFault::Regressed { high_water: id.seq })
    );

    let other = ledger.issue(OID + 1);
    ledger.record_ack(other.oid, 99).unwrap();
    assert!(!ledger.is_acked(other));
    assert_eq!(
        ledger.record_ack(other.oid, 98),
        Err(AckFault::Regressed { high_water: 99 })
    );
}

#[test]
fn cancellation_covers_late_acks_without_discarding_sent_receipts_or_reusing_sequences() {
    let mut ledger = BarrierLedger::with_seed(u32::MAX);
    let sent = ledger.issue(OID);
    let cancelled = ledger.issue(OID);
    let other = ledger.issue(OID + 1);
    ledger.note_sent(sent, 10);
    ledger.note_sent(other, 20);
    ledger.cancel(cancelled);
    assert!(ledger.is_acked(sent));
    assert_eq!(ledger.record_ordered_ack(OID, sent.seq), Ok(false));
    assert_eq!(ledger.sent_clock_of(sent), Some(10));
    ledger.forget_sent(OID);
    assert_eq!(ledger.sent_clock_of(sent), None);
    assert_eq!(ledger.sent_clock_of(other), Some(20));
    let next = ledger.issue(OID);
    assert_eq!(next.seq, 1);
    assert_eq!(ledger.record_ordered_ack(OID, next.seq), Ok(true));
    assert!(!ledger.is_acked(other));
    ledger.clear_sent();
    assert!(!ledger.has_sent());
    assert_eq!(ledger.record_ordered_ack(other.oid, other.seq), Ok(true));
}

#[test]
fn an_unsent_receipt_is_never_overdue() {
    let mut ledger = BarrierLedger::with_seed(100);
    ledger.issue(OID);
    assert!(ledger.overdue(1_000_000, 10).is_empty());
}

#[test]
fn a_sent_receipt_goes_overdue_and_an_ack_clears_it() {
    let mut ledger = BarrierLedger::with_seed(100);
    let id = ledger.issue(OID);
    ledger.note_sent(id, 1_000);
    assert!(
        ledger.overdue(1_050, 100).is_empty(),
        "still inside the deadline"
    );
    assert_eq!(ledger.overdue(2_000, 100), vec![(id, 1_000)]);
    ledger.record_ack(OID, id.seq).expect("issued");
    ledger.prune_acked();
    assert!(
        ledger.overdue(2_000, 100).is_empty(),
        "an acked receipt is no longer outstanding"
    );
}
