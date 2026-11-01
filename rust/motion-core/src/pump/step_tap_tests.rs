use super::SendError;
use super::step_tap::{RECORD_LEN, RecordKind, StepTap, TAP_MAGIC, TapLane, TapRecord};
use super::stepcompress_sink::{ClockSource, FrameEgress};
use crossbeam_channel::{Receiver, bounded};
use host_rt::host_io::parser::ArgValue;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

const MCU: u32 = 3;
const CLOCK_NOW: u64 = 5_000_000_123;
const OID_A: u32 = 7;
const OID_B: u32 = 8;

type Frame = (&'static str, Vec<(String, ArgValue)>);

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

fn u32_at(record: &TapRecord, offset: usize) -> u32 {
    u32::from_le_bytes(record.payload()[offset..offset + 4].try_into().unwrap())
}

#[test]
fn the_tap_records_exactly_the_frames_sent_in_order() {
    let (tap, rx) = tap_with_capacity(64);
    let (inner, sent) = sent_names();
    let egress = tap.wrap(inner);
    egress(&[
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
    ])
    .unwrap();
    egress(&[
        frame("kalico_wire_probe", &[("clock", 42)]),
        frame(
            "stepcompress_barrier",
            &[("oid", i64::from(OID_A)), ("seq", 9)],
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
    ])
    .unwrap();

    assert_eq!(sent.lock().unwrap().len(), 6);
    let records: Vec<TapRecord> = rx.try_iter().collect();
    let kinds: Vec<u8> = records.iter().map(TapRecord::kind).collect();
    assert_eq!(
        kinds,
        [
            RecordKind::ResetStepClock as u8,
            RecordKind::SetNextStepDir as u8,
            RecordKind::QueueStep as u8,
            RecordKind::Barrier as u8,
            RecordKind::QueueStep as u8,
        ]
    );
    let oids: Vec<u32> = records.iter().map(TapRecord::oid).collect();
    assert_eq!(oids, [OID_A, OID_A, OID_A, OID_A, OID_B]);
    assert_eq!(u32_at(&records[0], 0), 1_000);
    assert_eq!(records[1].payload()[0], 1);
    assert_eq!(u32_at(&records[2], 0), 100);
    assert_eq!(&records[2].payload()[4..8], &[3, 0, 0xFE, 0xFF]);
    assert_eq!(u32_at(&records[3], 0), 9);
    assert!(
        records
            .iter()
            .all(|r| r.0[8..16] == CLOCK_NOW.to_le_bytes())
    );
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

#[test]
fn a_projection_is_recorded_once_per_epoch_and_oid() {
    let (tap, _rx) = tap_with_capacity(64);
    assert!(tap.projection_changed(OID_A, 84e6, 1_000.0));
    assert!(!tap.projection_changed(OID_A, 84e6, 1_000.4));
    assert!(tap.projection_changed(OID_A, 84e6, 1_001.0));
    assert!(tap.projection_changed(OID_A, 84.001e6, 1_001.0));
    assert!(tap.projection_changed(OID_B, 84.001e6, 1_001.0));
}

#[test]
fn the_writer_puts_the_header_and_frames_in_the_file() {
    let dir = std::env::temp_dir().join(format!("step-tap-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let lanes = [
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
    ];
    let tap = StepTap::open(&dir, MCU, 84e6, &lanes, clock()).unwrap();
    let (inner, _) = sent_names();
    let egress = tap.wrap(inner);
    egress(&[frame(
        "reset_step_clock",
        &[("oid", i64::from(OID_A)), ("clock", 77)],
    )])
    .unwrap();
    drop(egress);
    drop(tap);
    let path = dir.join(format!("step_tap_mcu{MCU}.bin"));
    let expected = TAP_MAGIC.len() + 4 * RECORD_LEN;
    let deadline = Instant::now() + Duration::from_secs(2);
    let bytes = loop {
        let bytes = std::fs::read(&path).unwrap();
        if bytes.len() >= expected || Instant::now() > deadline {
            break bytes;
        }
        std::thread::sleep(Duration::from_millis(10));
    };
    std::fs::remove_dir_all(&dir).unwrap();
    assert_eq!(bytes.len(), expected);
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
