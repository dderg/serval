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

#[test]
fn the_ledger_line_names_every_acked_oid() {
    let mut ledger = BarrierLedger::with_seed(100);
    assert_eq!(ledger.ledger_line(), "no barrier acks recorded");
    ledger.issue(OID);
    ledger.record_ack(OID, 100).expect("issued");
    assert_eq!(ledger.ledger_line(), format!("oid {OID} acked 100"));
}
