use super::SendError;
use super::step_tap::{
    PAYLOAD_OFFSET, RECORD_LEN, RecordKind, StepTap, TAP_MAGIC, TapLane, TapRecord, encode_frame,
};
use super::stepcompress_sink::{ClockSource, FrameEgress};
use crossbeam_channel::{Receiver, bounded};
use host_rt::host_io::parser::ArgValue;
use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};

const MCU: u32 = 3;
const CLOCK_NOW: u64 = 5_000_000_123;
const OID_A: u32 = 7;
const OID_B: u32 = 8;

type Frame = (&'static str, Vec<(String, ArgValue)>);
pub(crate) type DecodedFrame = (String, BTreeMap<String, i64>);

fn clock() -> ClockSource {
    Arc::new(|_| Some((CLOCK_NOW, 84e6)))
}

fn frame(name: &'static str, args: &[(&str, i64)]) -> Frame {
    (
        name,
        args.iter()
            .map(|&(key, value)| (key.to_string(), ArgValue::Int(value)))
            .collect(),
    )
}

fn payload_u32(record: &TapRecord, at: usize) -> i64 {
    let at = PAYLOAD_OFFSET + at;
    i64::from(u32::from_le_bytes(record.0[at..at + 4].try_into().unwrap()))
}

fn payload_i32(record: &TapRecord, at: usize) -> i64 {
    let at = PAYLOAD_OFFSET + at;
    i64::from(i32::from_le_bytes(record.0[at..at + 4].try_into().unwrap()))
}

fn payload_u16(record: &TapRecord, at: usize) -> i64 {
    let at = PAYLOAD_OFFSET + at;
    i64::from(u16::from_le_bytes(record.0[at..at + 2].try_into().unwrap()))
}

fn payload_i16(record: &TapRecord, at: usize) -> i64 {
    let at = PAYLOAD_OFFSET + at;
    i64::from(i16::from_le_bytes(record.0[at..at + 2].try_into().unwrap()))
}

pub(crate) fn record_kind(record: &TapRecord) -> u8 {
    record.0[0]
}

pub(crate) fn record_oid(record: &TapRecord) -> u32 {
    u32::from_le_bytes(record.0[4..8].try_into().unwrap())
}

pub(crate) fn decode_frame(record: &TapRecord) -> Option<DecodedFrame> {
    let oid = i64::from(record_oid(record));
    let (name, args): (&str, Vec<(&str, i64)>) = match record_kind(record) {
        3 => ("reset_step_clock", vec![("clock", payload_u32(record, 0))]),
        4 => (
            "set_next_step_dir",
            vec![("dir", i64::from(record.0[PAYLOAD_OFFSET]))],
        ),
        5 => (
            "queue_step",
            vec![
                ("interval", payload_u32(record, 0)),
                ("count", payload_u16(record, 4)),
                ("add", payload_i16(record, 6)),
            ],
        ),
        6 => (
            "queue_step_hp",
            vec![
                ("interval", payload_u32(record, 0)),
                ("count", payload_u16(record, 4)),
                ("add", payload_i16(record, 6)),
                ("add2", payload_i16(record, 8)),
                ("shift", payload_i16(record, 10)),
            ],
        ),
        7 => (
            "stepcompress_barrier",
            vec![("seq", payload_u32(record, 0))],
        ),
        8 => (
            "stepcompress_set_position",
            vec![("pos", payload_i32(record, 0))],
        ),
        _ => return None,
    };
    let mut decoded: BTreeMap<String, i64> =
        args.into_iter().map(|(k, v)| (k.to_string(), v)).collect();
    decoded.insert("oid".to_string(), oid);
    Some((name.to_string(), decoded))
}

pub(crate) fn as_decoded(name: &str, args: &[(String, ArgValue)]) -> DecodedFrame {
    let args = args
        .iter()
        .map(|(key, value)| match value {
            ArgValue::Int(v) => (key.clone(), *v),
            other => panic!("{name} `{key}` is not an integer: {other:?}"),
        })
        .collect();
    (name.to_string(), args)
}

fn sent_names() -> (FrameEgress, Arc<Mutex<Vec<&'static str>>>) {
    let sent = Arc::new(Mutex::new(Vec::new()));
    let log = Arc::clone(&sent);
    let egress: FrameEgress = Arc::new(move |frames: &[Frame]| {
        log.lock()
            .unwrap()
            .extend(frames.iter().map(|(name, _)| *name));
        Ok(())
    });
    (egress, sent)
}

fn tap_with_capacity(capacity: usize) -> (Arc<StepTap>, Receiver<TapRecord>) {
    let (tx, rx) = bounded(capacity);
    (Arc::new(StepTap::from_sender(tx, MCU, clock())), rx)
}

#[test]
fn the_tap_records_exactly_the_frames_sent_in_order() {
    let (tap, rx) = tap_with_capacity(64);
    let (inner, sent) = sent_names();
    let egress = tap.wrap(inner);
    let first = [
        frame(
            "reset_step_clock",
            &[("oid", i64::from(OID_A)), ("clock", 1_000)],
        ),
        frame(
            "set_next_step_dir",
            &[("oid", i64::from(OID_A)), ("dir", 1)],
        ),
        frame(
            "queue_step",
            &[
                ("oid", i64::from(OID_A)),
                ("interval", 100),
                ("count", 3),
                ("add", -2),
            ],
        ),
    ];
    let second = [
        frame("kalico_wire_probe", &[("clock", 42)]),
        frame(
            "stepcompress_barrier",
            &[("oid", i64::from(OID_A)), ("seq", 9)],
        ),
        frame(
            "stepcompress_set_position",
            &[("oid", i64::from(OID_B)), ("pos", -12)],
        ),
        frame(
            "queue_step",
            &[
                ("oid", i64::from(OID_B)),
                ("interval", 250),
                ("count", 1),
                ("add", 0),
            ],
        ),
    ];
    egress(&first).unwrap();
    egress(&second).unwrap();

    assert_eq!(sent.lock().unwrap().len(), 7);
    let records: Vec<TapRecord> = rx.try_iter().collect();
    let decoded: Vec<DecodedFrame> = records.iter().filter_map(decode_frame).collect();
    let expected: Vec<DecodedFrame> = first
        .iter()
        .chain(&second[1..])
        .map(|(name, args)| as_decoded(name, args))
        .collect();
    assert_eq!(decoded.len(), records.len());
    assert_eq!(decoded, expected);
    assert!(
        records
            .iter()
            .all(|r| r.0[8..16] == CLOCK_NOW.to_le_bytes())
    );
}

#[test]
fn queue_step_hp_keeps_every_argument_at_its_own_offset() {
    let args = frame(
        "queue_step_hp",
        &[
            ("oid", i64::from(OID_A)),
            ("interval", 0x0102_0304),
            ("count", 0x0506),
            ("add", -0x0708),
            ("add2", 0x090A),
            ("shift", 11),
        ],
    );
    let record = encode_frame(args.0, &args.1, CLOCK_NOW).unwrap().unwrap();
    assert_eq!(record_kind(&record), RecordKind::QueueStepHp as u8);
    assert_eq!(decode_frame(&record), Some(as_decoded(args.0, &args.1)));
    assert!(record.0[PAYLOAD_OFFSET + 12..].iter().all(|&b| b == 0));
}

#[test]
fn an_argument_wider_than_its_wire_field_is_refused() {
    let args = frame(
        "queue_step_hp",
        &[
            ("oid", 1),
            ("interval", 1),
            ("count", 1),
            ("add", 0),
            ("add2", 40_000),
            ("shift", 0),
        ],
    );
    let error = encode_frame(args.0, &args.1, 0).unwrap_err();
    assert!(error.contains("add2"), "{error}");
}

#[test]
fn a_batch_the_transport_refused_is_not_recorded() {
    let (tap, rx) = tap_with_capacity(64);
    let refusing: FrameEgress =
        Arc::new(|_: &[Frame]| Err(SendError::Transient("link busy".to_string())));
    let egress = tap.wrap(refusing);
    let result = egress(&[frame(
        "reset_step_clock",
        &[("oid", i64::from(OID_A)), ("clock", 1_000)],
    )]);
    assert!(matches!(result, Err(SendError::Transient(_))));
    assert_eq!(rx.try_iter().count(), 0);
}

#[test]
fn a_full_writer_channel_is_fatal_and_stays_fatal() {
    let (tap, _rx) = tap_with_capacity(1);
    let (inner, _) = sent_names();
    let egress = tap.wrap(inner);
    let step = frame(
        "queue_step",
        &[
            ("oid", i64::from(OID_A)),
            ("interval", 100),
            ("count", 1),
            ("add", 0),
        ],
    );
    let result = egress(&[step.clone(), step.clone()]);
    assert!(
        matches!(&result, Err(SendError::Fatal(message)) if message.contains("fell behind")),
        "{result:?}"
    );
    assert!(matches!(egress(&[]), Err(SendError::Fatal(_))));
}

#[test]
fn an_unrecognised_frame_is_fatal() {
    let (tap, _rx) = tap_with_capacity(64);
    let (inner, _) = sent_names();
    let egress = tap.wrap(inner);
    let result = egress(&[frame("queue_step_v2", &[("oid", 1)])]);
    assert!(
        matches!(&result, Err(SendError::Fatal(message)) if message.contains("unrecognised")),
        "{result:?}"
    );
}

fn lanes() -> [TapLane; 2] {
    [
        TapLane {
            oid: OID_A,
            axis: 0,
            motor_in_axis: 0,
            microstep_distance: 0.0125,
            invert_dir: false,
        },
        TapLane {
            oid: OID_B,
            axis: 3,
            motor_in_axis: 0,
            microstep_distance: 0.0024,
            invert_dir: true,
        },
    ]
}

fn scratch_dir(name: &str) -> std::path::PathBuf {
    let dir = std::env::temp_dir().join(format!("step-tap-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn tap_files(dir: &std::path::Path) -> Vec<Vec<u8>> {
    let mut paths: Vec<_> = std::fs::read_dir(dir)
        .unwrap()
        .map(|entry| entry.unwrap().path())
        .collect();
    paths.sort();
    paths.iter().map(|p| std::fs::read(p).unwrap()).collect()
}

#[test]
fn closing_the_tap_flushes_every_record_to_its_file() {
    let dir = scratch_dir("close");
    let tap = StepTap::open(&dir, MCU, 84e6, &lanes(), clock()).unwrap();
    let (inner, _) = sent_names();
    let egress = tap.wrap(inner);
    egress(&[frame(
        "reset_step_clock",
        &[("oid", i64::from(OID_A)), ("clock", 77)],
    )])
    .unwrap();
    drop(egress);
    drop(tap);
    let files = tap_files(&dir);
    std::fs::remove_dir_all(&dir).unwrap();
    assert_eq!(files.len(), 1);
    let bytes = &files[0];
    assert_eq!(bytes.len(), TAP_MAGIC.len() + 4 * RECORD_LEN);
    assert_eq!(bytes[..8], TAP_MAGIC);
    let kinds: Vec<u8> = bytes[8..].chunks(RECORD_LEN).map(|r| r[0]).collect();
    assert_eq!(
        kinds,
        [
            RecordKind::Mcu as u8,
            RecordKind::Lane as u8,
            RecordKind::Lane as u8,
            RecordKind::ResetStepClock as u8,
        ]
    );
    assert_eq!(bytes[8 + RECORD_LEN + 1], 0);
    assert_eq!(bytes[8 + 2 * RECORD_LEN + 1], 1);
}

#[test]
fn a_second_session_in_one_process_never_truncates_the_first() {
    let dir = scratch_dir("sessions");
    let first = StepTap::open(&dir, MCU, 84e6, &lanes(), clock()).unwrap();
    let second = StepTap::open(&dir, MCU, 84e6, &lanes(), clock()).unwrap();
    drop(second);
    drop(first);
    let files = tap_files(&dir);
    std::fs::remove_dir_all(&dir).unwrap();
    assert_eq!(files.len(), 2);
    for bytes in files {
        assert_eq!(bytes.len(), TAP_MAGIC.len() + 3 * RECORD_LEN);
    }
}

#[test]
fn opening_without_a_clock_estimate_fails() {
    let dir = scratch_dir("noclock");
    let no_clock: ClockSource = Arc::new(|_| None);
    let error = StepTap::open(&dir, MCU, 84e6, &lanes(), no_clock)
        .err()
        .expect("a tap cannot stamp its header without a clock");
    std::fs::remove_dir_all(&dir).unwrap();
    assert!(error.contains("no clock estimate"), "{error}");
}
