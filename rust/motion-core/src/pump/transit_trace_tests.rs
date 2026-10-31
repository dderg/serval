use super::transit_trace::{TransitTraceRecord, record, snapshot_last};

#[test]
fn snapshot_returns_published_records_in_sequence_order() {
    let marker = u64::MAX - 17;
    record(TransitTraceRecord {
        sequence: 0,
        mcu_id: u32::MAX,
        axis: u8::MAX,
        piece_count: 7,
        room: 11,
        guard_recorded_ns: marker - 1,
        guard_mcu_clock: 29,
        send_started_ns: marker,
        send_elapsed_ns: 13,
        host_front_start_time: 17,
        result: -308,
    });

    let records = snapshot_last(64);
    let record = records
        .iter()
        .find(|record| record.send_started_ns == marker)
        .expect("published trace record must be visible");

    assert_eq!(record.mcu_id, u32::MAX);
    assert_eq!(record.axis, u8::MAX);
    assert_eq!(record.piece_count, 7);
    assert_eq!(record.room, 11);
    assert_eq!(record.guard_recorded_ns, marker - 1);
    assert_eq!(record.guard_mcu_clock, 29);
    assert_eq!(record.send_elapsed_ns, 13);
    assert_eq!(record.host_front_start_time, 17);
    assert_eq!(record.result, -308);
    assert!(
        records
            .windows(2)
            .all(|pair| pair[0].sequence < pair[1].sequence)
    );
}
