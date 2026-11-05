use super::*;
use motion_core::dispatch_record::{HEADER_LEN, RECORDED_AXES, RecordKind, SEGMENT_FRESH_EPOCH};

const CAPACITY_CFG: &str = include_str!("../../../../tools/bench/configs/capacity.cfg");
const SEGMENT_FLAGS_AT: usize = 1 + 3 * 8 + 4;
const SEGMENT_FIXED_LEN: usize = SEGMENT_FLAGS_AT + 1 + 4;
const DRAIN_LEN: usize = 1 + 1 + 8;

#[derive(Debug, PartialEq)]
enum Summary {
    Segment { fresh_epoch: bool },
    Drain { cause: u8, moves_sent: u64 },
}

fn summarize(bytes: &[u8]) -> Vec<Summary> {
    let mut at = HEADER_LEN;
    let mut records = Vec::new();
    while at < bytes.len() {
        let kind = bytes[at];
        if kind == RecordKind::Segment as u8 {
            let flags = bytes[at + SEGMENT_FLAGS_AT];
            let n = u32::from_le_bytes(
                bytes[at + SEGMENT_FLAGS_AT + 1..at + SEGMENT_FIXED_LEN]
                    .try_into()
                    .unwrap(),
            );
            records.push(Summary::Segment {
                fresh_epoch: flags & SEGMENT_FRESH_EPOCH != 0,
            });
            at += SEGMENT_FIXED_LEN + n as usize * (1 + RECORDED_AXES as usize * 3) * 8;
        } else if kind == RecordKind::Drain as u8 {
            records.push(Summary::Drain {
                cause: bytes[at + 1],
                moves_sent: u64::from_le_bytes(bytes[at + 2..at + DRAIN_LEN].try_into().unwrap()),
            });
            at += DRAIN_LEN;
        } else {
            panic!("unexpected record kind {kind} at byte {at}");
        }
    }
    assert_eq!(at, bytes.len(), "the last record ends the file");
    records
}

fn record_of(gcode: &str) -> StreamRecord {
    let planner = planner_config::from_doc::planner_config_from_text(CAPACITY_CFG).unwrap();
    let waypoints = snapshot_core::waypoints::parse_gcode(gcode, &planner.cartesian).unwrap();
    snapshot_core::stream_record(&waypoints, &planner, None).unwrap()
}

#[test]
fn the_plan_record_drains_at_every_dwell_and_reanchors_after_it() {
    let record = record_of(
        "G21\nG90\nM83\nG1 X10 Y10 F6000\nG1 X40 Y10\nG4 P100\nG1 X40 Y40\nG4 P50\nG1 X10 Y40\nG1 X10 Y10\n",
    );
    let encoded = plan_dispatch_record(&record).unwrap();
    let summary = summarize(&encoded);

    let drains: Vec<(u8, u64)> = summary
        .iter()
        .filter_map(|record| match record {
            Summary::Drain { cause, moves_sent } => Some((*cause, *moves_sent)),
            Summary::Segment { .. } => None,
        })
        .collect();
    assert_eq!(drains, vec![(DrainCause::Requested as u8, 1), (0, 2)]);
    assert_eq!(
        summary
            .iter()
            .filter(|record| matches!(record, Summary::Segment { .. }))
            .count(),
        record.segments.len()
    );
    let mut after_drain = true;
    for (index, entry) in summary.iter().enumerate() {
        match entry {
            Summary::Drain { .. } => after_drain = true,
            Summary::Segment { fresh_epoch } => {
                assert_eq!(
                    *fresh_epoch, after_drain,
                    "record {index}: only the first segment of an epoch re-anchors"
                );
                after_drain = false;
            }
        }
    }
}

#[test]
fn every_plan_segment_is_sampled_on_its_own_carrier_from_origin_zero() {
    let record = record_of("G21\nG90\nM83\nG1 X10 Y10 F6000\nG1 X60 Y30 E2\n");
    let encoded = plan_dispatch_record(&record).unwrap();
    let mut expected = Vec::new();
    encode_header(&mut expected);
    for (index, seg) in record.segments.iter().enumerate() {
        encode_segment(&mut expected, seg, 0.0, index == 0).unwrap();
    }
    assert_eq!(encoded, expected);
}

#[test]
fn a_rest_count_the_inputs_do_not_match_is_refused() {
    let mut record = record_of("G21\nG90\nM83\nG1 X10 Y10 F6000\nG4 P100\nG1 X40 Y10\n");
    record.drained_at_rests.push(record.segments.len());
    let error = plan_dispatch_record(&record).expect_err("an unmatched rest is refused");
    assert!(error.contains("drains"), "{error}");
}
