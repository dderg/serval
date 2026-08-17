#![allow(
    clippy::panic,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing
)]

use super::*;

fn header(start_clock: u64, interval_ticks: u32, count: u16) -> SampleRunHeader {
    SampleRunHeader::new(start_clock, interval_ticks, count)
}

#[test]
fn span_and_end_clock_place_the_next_run() {
    let h = header(1_000, 25, 4);
    assert_eq!(h.span_ticks(), 100);
    assert_eq!(h.end_clock(), 1_100);
    assert_eq!(h.last_sample_clock(), 1_075);
}

#[test]
fn single_sample_run_is_legal() {
    let h = header(500, 40, 1);
    assert_eq!(h.end_clock(), 540);
    assert_eq!(h.last_sample_clock(), 500);
    let view = SampleRunView::new(h, &[7]).expect("count=1 run is legal");
    assert_eq!(view.last_position(), Some(7));
}

#[test]
fn view_rejects_count_disagreeing_with_samples() {
    let err = SampleRunView::new(header(0, 10, 3), &[1, 2]).expect_err("mismatch must fault");
    assert_eq!(
        err,
        SampleRunError::CountMismatch {
            count: 3,
            samples: 2
        }
    );
}

#[test]
fn view_iteration_walks_the_clock_grid() {
    let view = SampleRunView::new(header(100, 10, 3), &[5, 6, 8]).expect("valid run");
    let walked: Vec<(u64, i32)> = view.iter().collect();
    assert_eq!(walked, vec![(100, 5), (110, 6), (120, 8)]);
}

#[test]
fn degenerate_headers_fault() {
    assert_eq!(
        SampleRunView::new(header(9, 10, 0), &[]).expect_err("zero count"),
        SampleRunError::ZeroCount { start_clock: 9 }
    );
    assert_eq!(
        SampleRunView::new(header(9, 0, 2), &[1, 2]).expect_err("zero interval"),
        SampleRunError::ZeroInterval { start_clock: 9 }
    );
    let err = SampleRunView::new(header(u64::MAX - 5, 10, 4), &[0, 0, 0, 0])
        .expect_err("span overflows the clock");
    assert_eq!(
        err,
        SampleRunError::SpanOverflow {
            start_clock: u64::MAX - 5,
            interval_ticks: 10,
            count: 4
        }
    );
}

#[test]
fn abutting_runs_are_accepted_in_sequence() {
    let mut cursor = LaneCursor::new();
    cursor.anchor(1_000, 40);
    let first = SampleRunView::new(header(1_000, 25, 4), &[41, 43, 46, 50]).expect("valid");
    cursor.admit(&first).expect("first run abuts the anchor");
    assert_eq!(cursor.next_clock(), Some(1_100));
    assert_eq!(cursor.position(), 50);

    let second = SampleRunView::new(header(1_100, 50, 2), &[55, 61]).expect("valid");
    cursor
        .admit(&second)
        .expect("interval may change between runs");
    assert_eq!(cursor.next_clock(), Some(1_200));
    assert_eq!(cursor.position(), 61);
}

#[test]
fn a_gap_faults_instead_of_padding() {
    let mut cursor = LaneCursor::new();
    cursor.anchor(1_000, 0);
    cursor.accept(&header(1_000, 25, 4)).expect("valid");
    let err = cursor
        .accept(&header(1_101, 25, 4))
        .expect_err("a one-tick hole is a fault");
    assert_eq!(
        err,
        SampleRunError::Discontinuity {
            expected_clock: 1_100,
            start_clock: 1_101
        }
    );
    assert_eq!(cursor.next_clock(), Some(1_100), "cursor must not advance");
}

#[test]
fn an_overlapping_run_faults_too() {
    let mut cursor = LaneCursor::new();
    cursor.anchor(0, 0);
    cursor.accept(&header(0, 10, 4)).expect("valid");
    assert_eq!(
        cursor.accept(&header(30, 10, 2)).expect_err("overlap"),
        SampleRunError::Discontinuity {
            expected_clock: 40,
            start_clock: 30
        }
    );
}

#[test]
fn an_unanchored_lane_refuses_runs() {
    let mut cursor = LaneCursor::new();
    assert!(!cursor.is_anchored());
    assert_eq!(
        cursor.accept(&header(77, 10, 1)).expect_err("no anchor"),
        SampleRunError::NotAnchored { start_clock: 77 }
    );
}

#[test]
fn re_anchoring_crosses_a_discontinuity() {
    let mut cursor = LaneCursor::new();
    cursor.anchor(0, 0);
    cursor.accept(&header(0, 10, 4)).expect("valid");
    cursor.anchor(9_000, -250);
    cursor
        .accept(&header(9_000, 10, 1))
        .expect("the explicit anchor sanctions the jump");
    assert_eq!(cursor.position(), -250);
}

#[test]
fn faults_carry_distinct_codes_and_strings() {
    let faults = [
        SampleRunError::NotAnchored { start_clock: 0 },
        SampleRunError::Discontinuity {
            expected_clock: 0,
            start_clock: 1,
        },
        SampleRunError::ZeroCount { start_clock: 0 },
        SampleRunError::ZeroInterval { start_clock: 0 },
        SampleRunError::SpanOverflow {
            start_clock: 0,
            interval_ticks: 1,
            count: 1,
        },
        SampleRunError::Capacity { capacity: 0 },
        SampleRunError::CountMismatch {
            count: 0,
            samples: 1,
        },
        SampleRunError::CountExceeded { count: 0, cap: 0 },
        SampleRunError::DeltaOverflow { index: 0, delta: 0 },
        SampleRunError::PositionOverflow {
            index: 0,
            position: 0,
        },
        SampleRunError::Truncated { index: 0 },
        SampleRunError::Trailing {
            consumed: 0,
            len: 1,
        },
    ];
    let mut codes: Vec<u16> = faults.iter().map(SampleRunError::fault_code).collect();
    let count = codes.len();
    codes.sort_unstable();
    codes.dedup();
    assert_eq!(codes.len(), count, "fault codes must be distinct");

    let mut strings: Vec<&str> = faults.iter().map(SampleRunError::as_str).collect();
    strings.sort_unstable();
    strings.dedup();
    assert_eq!(strings.len(), count, "fault strings must be distinct");
    assert!(codes.iter().all(|&code| code != 0), "0 is reserved");
}

#[test]
fn buf_accumulates_then_reports_capacity() {
    let mut buf: SampleRunBuf<3> = SampleRunBuf::new(400, 20);
    assert!(buf.is_empty());
    assert_eq!(buf.next_clock(), 400);
    buf.push(1).expect("room");
    assert_eq!(buf.next_clock(), 420);
    buf.push(2).expect("room");
    buf.push(3).expect("room");
    assert!(buf.is_full());
    assert_eq!(
        buf.push(4).expect_err("full"),
        SampleRunError::Capacity { capacity: 3 }
    );
    assert_eq!(buf.samples(), &[1, 2, 3]);
    assert_eq!(buf.last_position(), Some(3));
    assert_eq!(buf.view().expect("valid").header().end_clock(), 460);

    buf.reset(1_000, 10);
    assert!(buf.is_empty());
    assert_eq!(buf.next_clock(), 1_000);
}

#[test]
fn buf_view_of_an_empty_buf_faults_rather_than_lying() {
    let buf: SampleRunBuf<4> = SampleRunBuf::new(0, 10);
    assert_eq!(
        buf.view().expect_err("empty"),
        SampleRunError::ZeroCount { start_clock: 0 }
    );
}

#[test]
fn overlay_adds_on_a_matching_grid() {
    let mut base = [100, 110, 120];
    let base_header = header(50, 10, 3);
    let overlay = SampleRunView::new(base_header, &[1, -2, 3]).expect("valid");
    SampleRunView::overlay_onto(&mut base, base_header, &overlay).expect("grids match");
    assert_eq!(base, [101, 108, 123]);
}

#[test]
fn overlay_on_a_foreign_grid_faults() {
    let mut base = [0, 0];
    let base_header = header(50, 10, 2);
    let overlay = SampleRunView::new(header(60, 10, 2), &[1, 1]).expect("valid");
    assert_eq!(
        SampleRunView::overlay_onto(&mut base, base_header, &overlay).expect_err("grid mismatch"),
        SampleRunError::Discontinuity {
            expected_clock: 50,
            start_clock: 60
        }
    );
}

fn roundtrip(base: i32, samples: &[i32]) -> usize {
    let mut wire = [0u8; 512];
    let written = encode_deltas(base, samples, &mut wire).expect("encodes");
    let mut decoded = [0i32; SAMPLE_RUN_COUNT_MAX];
    decode_deltas(
        base,
        wire.get(..written).expect("in range"),
        samples.len(),
        &mut decoded,
    )
    .expect("decodes");
    assert_eq!(decoded.get(..samples.len()).expect("in range"), samples);
    written
}

#[test]
fn delta_codec_roundtrips_across_magnitudes() {
    roundtrip(0, &[0, 0, 0]);
    roundtrip(1_000, &[1_064, 1_128, 1_192]);
    roundtrip(0, &[-1, 1, -1, 1]);
    roundtrip(0, &[1_073_741_823]);
    roundtrip(1_073_741_823, &[-1_073_741_824]);
    roundtrip(-500, &[-500]);
}

#[test]
fn small_deltas_cost_one_byte_and_typical_print_deltas_cost_two() {
    assert_eq!(roundtrip(0, &[10, 20, 30, 40]), 4);
    assert_eq!(delta_bytes(0, 63).expect("in range"), 1);
    assert_eq!(delta_bytes(0, -64).expect("in range"), 1);
    assert_eq!(delta_bytes(0, 64).expect("in range"), 2);
    assert_eq!(delta_bytes(0, 8_191).expect("in range"), 2);
    assert_eq!(delta_bytes(0, 8_192).expect("in range"), 3);
    assert!(delta_bytes(0, 128).expect("in range") <= SAMPLE_DELTA_BYTES_MAX);
}

#[test]
fn delta_bytes_predicts_encode_exactly() {
    let samples = [3, -900, 900_000, 900_001, -2_000_000];
    let mut base = 0;
    let mut predicted = 0;
    for position in samples {
        predicted += delta_bytes(base, position).expect("in range");
        base = position;
    }
    assert_eq!(roundtrip(0, &samples), predicted);
}

#[test]
fn encode_reports_capacity_rather_than_silently_truncating() {
    let mut wire = [0u8; 3];
    assert_eq!(
        encode_deltas(0, &[1_000_000, 2_000_000], &mut wire).expect_err("too small"),
        SampleRunError::Capacity { capacity: 3 }
    );
}

#[test]
fn encode_refuses_more_samples_than_the_wire_cap() {
    let samples = [1i32; SAMPLE_RUN_COUNT_MAX + 1];
    let mut wire = [0u8; 512];
    assert_eq!(
        encode_deltas(0, &samples, &mut wire).expect_err("over cap"),
        SampleRunError::CountExceeded {
            count: SAMPLE_RUN_COUNT_MAX + 1,
            cap: SAMPLE_RUN_COUNT_MAX
        }
    );
}

#[test]
fn encode_refuses_a_delta_wider_than_i32() {
    let mut wire = [0u8; 512];
    assert_eq!(
        encode_deltas(i32::MIN, &[i32::MAX], &mut wire).expect_err("delta overflows i32"),
        SampleRunError::DeltaOverflow {
            index: 0,
            delta: i64::from(i32::MAX) - i64::from(i32::MIN)
        }
    );
}

#[test]
fn decode_faults_on_a_truncated_payload() {
    let mut wire = [0u8; 512];
    let written = encode_deltas(0, &[100_000], &mut wire).expect("encodes");
    let mut out = [0i32; 4];
    assert_eq!(
        decode_deltas(0, wire.get(..written - 1).expect("in range"), 1, &mut out)
            .expect_err("mid-delta end"),
        SampleRunError::Truncated { index: 0 }
    );
}

#[test]
fn decode_faults_when_the_output_lane_is_too_small() {
    let mut wire = [0u8; 512];
    let written = encode_deltas(0, &[1, 2, 3], &mut wire).expect("encodes");
    let mut out = [0i32; 2];
    assert_eq!(
        decode_deltas(0, wire.get(..written).expect("in range"), 3, &mut out)
            .expect_err("lane too small"),
        SampleRunError::Capacity { capacity: 2 }
    );
}

#[test]
fn decode_faults_when_the_stream_walks_off_the_lane() {
    let mut wire = [0u8; 512];
    let written = encode_deltas(0, &[i32::MAX], &mut wire).expect("encodes");
    let mut out = [0i32; 4];
    let err = decode_deltas(1, wire.get(..written).expect("in range"), 1, &mut out)
        .expect_err("position overflows i32");
    assert!(matches!(
        err,
        SampleRunError::PositionOverflow { index: 0, .. }
    ));
}

#[test]
fn a_full_run_of_worst_case_deltas_still_fits_the_wire_cap() {
    let mut base = 0i32;
    let mut samples = [0i32; SAMPLE_RUN_COUNT_MAX];
    for (index, slot) in samples.iter_mut().enumerate() {
        base += if index % 2 == 0 { 8_191 } else { -8_191 };
        *slot = base;
    }
    let mut wire = [0u8; 512];
    let written = encode_deltas(0, &samples, &mut wire).expect("encodes");
    assert!(
        written > SAMPLE_RUN_DATA_MAX,
        "two-byte deltas must overrun the cap, so producers close runs on bytes"
    );
    let mut budget = 0usize;
    let mut fitted = 0usize;
    let mut previous = 0i32;
    for position in samples {
        let cost = delta_bytes(previous, position).expect("in range");
        if budget + cost > SAMPLE_RUN_DATA_MAX {
            break;
        }
        budget += cost;
        fitted += 1;
        previous = position;
    }
    assert_eq!(fitted, 24);
    assert_eq!(budget, SAMPLE_RUN_DATA_MAX);
}

#[test]
fn decode_faults_on_bytes_past_the_declared_count() {
    let mut wire = [0u8; 512];
    let written = encode_deltas(0, &[1, 2, 3], &mut wire).expect("encodes");
    let mut out = [0i32; 4];
    assert_eq!(
        decode_deltas(0, wire.get(..written).expect("in range"), 2, &mut out)
            .expect_err("count disagrees with payload"),
        SampleRunError::Trailing {
            consumed: 2,
            len: 3
        }
    );
}
