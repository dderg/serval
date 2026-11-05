use super::*;
use std::panic::AssertUnwindSafe;
use std::sync::Arc;
use std::time::Instant;
use trajectory::{ContinuousAxis, NudgeProfile};

const AXES: usize = RECORDED_AXES as usize;
const SEGMENT_FIXED_LEN: usize = 1 + 3 * 8 + 4 + 1 + 4;

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct DecodedSegment {
    pub t0: f64,
    pub t_start: f64,
    pub t_end: f64,
    pub source_line: u32,
    pub flags: u8,
    pub times: Vec<f64>,
    pub samples: Vec<[[f64; 3]; AXES]>,
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) enum Decoded {
    Segment(DecodedSegment),
    Drain { cause: u8, moves_sent: u64 },
    Nudge(RecordedNudge),
}

struct Cursor<'a> {
    bytes: &'a [u8],
    at: usize,
}

impl Cursor<'_> {
    fn take<const N: usize>(&mut self) -> [u8; N] {
        let taken = self.bytes[self.at..self.at + N].try_into().unwrap();
        self.at += N;
        taken
    }
    fn u8(&mut self) -> u8 {
        self.take::<1>()[0]
    }
    fn u32(&mut self) -> u32 {
        u32::from_le_bytes(self.take())
    }
    fn u64(&mut self) -> u64 {
        u64::from_le_bytes(self.take())
    }
    fn f64(&mut self) -> f64 {
        f64::from_le_bytes(self.take())
    }
}

/// Every record after the header, as the analyzer reads them.
pub(crate) fn decode(bytes: &[u8]) -> Vec<Decoded> {
    let mut cursor = Cursor { bytes, at: 0 };
    assert_eq!(cursor.take::<8>(), DISPATCH_RECORD_MAGIC);
    assert_eq!(cursor.u32(), RECORDED_AXES);
    assert_eq!(cursor.f64(), SAMPLE_INTERVAL_S);
    let mut records = Vec::new();
    while cursor.at < bytes.len() {
        let kind = cursor.u8();
        records.push(match kind {
            1 => {
                let t0 = cursor.f64();
                let t_start = cursor.f64();
                let t_end = cursor.f64();
                let source_line = cursor.u32();
                let flags = cursor.u8();
                let n = cursor.u32();
                let (times, samples) = (0..n)
                    .map(|_| {
                        let t = cursor.f64();
                        let pva: [[f64; 3]; AXES] =
                            std::array::from_fn(|_| std::array::from_fn(|_| cursor.f64()));
                        (t, pva)
                    })
                    .unzip();
                Decoded::Segment(DecodedSegment {
                    t0,
                    t_start,
                    t_end,
                    source_line,
                    flags,
                    times,
                    samples,
                })
            }
            2 => Decoded::Drain {
                cause: cursor.u8(),
                moves_sent: cursor.u64(),
            },
            3 => Decoded::Nudge(RecordedNudge {
                t0: cursor.f64(),
                t_start: cursor.f64(),
                t_end: cursor.f64(),
                mcu_id: cursor.u32(),
                axis: cursor.u8(),
            }),
            other => panic!("unknown record kind {other} at byte {}", cursor.at - 1),
        });
    }
    records
}

pub(crate) fn scratch_dir(name: &str) -> std::path::PathBuf {
    let dir = std::env::temp_dir().join(format!("dispatch-record-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// The only record file in `dir`, read after its recorder closed.
pub(crate) fn only_record(dir: &Path) -> Vec<u8> {
    let paths: Vec<_> = std::fs::read_dir(dir)
        .unwrap()
        .map(|entry| entry.unwrap().path())
        .collect();
    assert_eq!(paths.len(), 1, "one record per recorder: {paths:?}");
    std::fs::read(&paths[0]).unwrap()
}

fn nudge_profile(t_start: f64) -> NudgeProfile {
    crate::nudge::plan_nudge_profile(0, 3.0, 50.0, 2000.0, t_start).unwrap()
}

fn segment_with_axes(axes: usize, t_start: f64) -> ContinuousSegment {
    let profile = nudge_profile(t_start);
    let hold = |position| ContinuousAxis::Hold {
        position,
        t_start: profile.t_start(),
        t_end: profile.t_end(),
    };
    let all = [
        ContinuousAxis::Nudge(profile.clone()),
        hold(12.5),
        hold(-0.2),
        ContinuousAxis::Nudge(profile.clone()),
    ];
    ContinuousSegment {
        axes: all[..axes].to_vec().into(),
        followers: Vec::new().into(),
        spatial_path: false,
        t_start: profile.t_start(),
        t_end: profile.t_end(),
        motor_mask: 0xF,
        source_line: 7,
        rest_at_end: true,
    }
}

fn segment(t_start: f64) -> ContinuousSegment {
    segment_with_axes(AXES, t_start)
}

fn pva(seg: &ContinuousSegment, axis: usize, t: f64) -> [f64; 3] {
    let p = seg.eval_axis(axis, t).unwrap();
    [p.position, p.velocity, p.acceleration]
}

#[test]
fn the_header_names_the_format_the_axes_and_the_sample_interval() {
    let mut out = Vec::new();
    encode_header(&mut out);
    assert_eq!(out.len(), HEADER_LEN);
    assert_eq!(decode(&out), Vec::new());
}

fn instants(seg: &ContinuousSegment) -> Vec<SampleInstant> {
    sample_instants(seg).unwrap()
}

#[test]
fn a_segment_is_sampled_from_end_to_end_no_wider_apart_than_the_interval() {
    let seg = segment(2.0);
    let samples = instants(&seg);
    assert_eq!(samples[0].t, seg.t_start);
    assert_eq!(samples[samples.len() - 1].t, seg.t_end);
    for pair in samples.windows(2) {
        let gap = pair[1].t - pair[0].t;
        assert!(
            (0.0..=SAMPLE_INTERVAL_S * (1.0 + 1e-12)).contains(&gap),
            "gap {gap} between {pair:?}"
        );
    }
    for sample in &samples {
        assert!(
            (seg.t_start..=seg.t_end).contains(&sample.eval_t),
            "{sample:?}"
        );
    }
}

#[test]
fn every_breakpoint_is_read_once_from_each_side() {
    let seg = segment(2.0);
    let samples = instants(&seg);
    let interior: Vec<f64> = seg
        .breakpoints()
        .into_iter()
        .filter(|&b| b > seg.t_start && b < seg.t_end)
        .collect();
    assert!(!interior.is_empty(), "the nudge fixture has corners");
    let mut corners_with_a_jump = 0;
    for corner in interior {
        let at: Vec<usize> = (0..samples.len())
            .filter(|&k| samples[k].t == corner)
            .collect();
        let [left, right] = at[..] else {
            panic!("corner {corner} is sampled at {at:?}");
        };
        assert_eq!(right, left + 1);
        assert_eq!(
            samples[left].eval_t,
            trajectory::continuous::interior_time_below(corner)
        );
        assert_eq!(
            samples[right].eval_t,
            trajectory::continuous::interior_time_above(corner)
        );
        if (0..AXES).any(|axis| {
            pva(&seg, axis, samples[left].eval_t)[2] != pva(&seg, axis, samples[right].eval_t)[2]
        }) {
            corners_with_a_jump += 1;
        }
    }
    assert!(
        corners_with_a_jump > 0,
        "the fixture's acceleration jumps at a corner"
    );
}

#[test]
fn breakpoints_closer_than_the_device_resolution_are_one_corner() {
    let mut seg = segment(2.0);
    let sliver = seg.t_end - MIN_BREAKPOINT_GAP_S / 10.0;
    seg.axes = [
        seg.axes[0].clone(),
        ContinuousAxis::Hold {
            position: 1.0,
            t_start: seg.t_start,
            t_end: sliver,
        },
        seg.axes[2].clone(),
        seg.axes[3].clone(),
    ]
    .to_vec()
    .into();
    let samples = instants(&seg);
    assert!(samples.iter().all(|sample| sample.t != sliver));
    assert_eq!(
        samples
            .iter()
            .filter(|sample| sample.t == seg.t_end)
            .count(),
        1
    );
}

#[test]
fn a_span_that_is_not_a_finite_forward_interval_is_refused() {
    for (t_start, t_end) in [(1.0, 0.5), (f64::NAN, 1.0), (0.0, f64::INFINITY)] {
        let mut seg = segment(0.0);
        seg.t_start = t_start;
        seg.t_end = t_end;
        assert!(
            sample_instants(&seg).is_err(),
            "[{t_start}, {t_end}] must be refused"
        );
    }
}

#[test]
fn a_segment_record_carries_its_anchoring_and_its_carrier_at_every_sample() {
    let seg = segment(2.0);
    let mut out = Vec::new();
    encode_segment(&mut out, &seg, 1234.5, true).unwrap();
    let expected = instants(&seg);
    let n = expected.len();
    assert_eq!(out.len(), SEGMENT_FIXED_LEN + n * (1 + AXES * 3) * 8);
    let mut framed = Vec::new();
    encode_header(&mut framed);
    framed.extend_from_slice(&out);
    let [Decoded::Segment(record)] = &decode(&framed)[..] else {
        panic!("one segment record");
    };
    assert_eq!(record.t0, 1234.5);
    assert_eq!((record.t_start, record.t_end), (seg.t_start, seg.t_end));
    assert_eq!(record.source_line, 7);
    assert_eq!(record.flags, SEGMENT_FRESH_EPOCH | SEGMENT_REST_AT_END);
    assert_eq!(record.samples.len(), n);
    for (k, (sample, instant)) in record.samples.iter().zip(&expected).enumerate() {
        assert_eq!(record.times[k], instant.t);
        for axis in 0..AXES {
            assert_eq!(
                sample[axis],
                pva(&seg, axis, instant.eval_t),
                "sample {k} axis {axis}"
            );
        }
    }
}

#[test]
fn a_continuation_segment_without_a_rest_sets_no_flags() {
    let mut seg = segment(0.0);
    seg.rest_at_end = false;
    let mut out = Vec::new();
    encode_header(&mut out);
    encode_segment(&mut out, &seg, 0.0, false).unwrap();
    let [Decoded::Segment(record)] = &decode(&out)[..] else {
        panic!("one segment record");
    };
    assert_eq!(record.flags, 0);
}

#[test]
fn a_segment_short_of_the_recorded_axes_is_refused_and_leaves_the_buffer_alone() {
    let mut out = vec![9, 9];
    let error = encode_segment(&mut out, &segment_with_axes(3, 0.0), 0.0, false)
        .expect_err("three axes cannot fill a four-axis record");
    assert!(error.contains("3 axes"), "{error}");
    assert_eq!(out, vec![9, 9]);
}

#[test]
fn drain_and_nudge_records_are_packed_without_padding() {
    let mut out = Vec::new();
    encode_drain(&mut out, DrainCause::PacerBrake, 0x0102_0304_0506_0708);
    assert_eq!(out, [2, 1, 8, 7, 6, 5, 4, 3, 2, 1]);
    let nudge = RecordedNudge {
        t0: 10.0,
        t_start: 1.0,
        t_end: 1.5,
        mcu_id: 3,
        axis: 2,
    };
    out.clear();
    encode_nudge(&mut out, &nudge);
    assert_eq!(out.len(), 1 + 3 * 8 + 4 + 1);
    let mut framed = Vec::new();
    encode_header(&mut framed);
    encode_drain(&mut framed, DrainCause::Requested, 5);
    framed.extend_from_slice(&out);
    assert_eq!(
        decode(&framed),
        vec![
            Decoded::Drain {
                cause: 0,
                moves_sent: 5
            },
            Decoded::Nudge(nudge)
        ]
    );
}

#[test]
fn the_recorder_writes_exactly_what_the_encoders_produce_in_dispatch_order() {
    let dir = scratch_dir("roundtrip");
    let first = segment(0.0);
    let second = segment(first.t_end);
    let nudge = RecordedNudge {
        t0: 7.0,
        t_start: 3.0,
        t_end: 3.2,
        mcu_id: 1,
        axis: 0,
    };
    let recorder = DispatchRecorder::open(&dir).unwrap();
    let name = recorder
        .path()
        .file_name()
        .unwrap()
        .to_str()
        .unwrap()
        .to_string();
    let session = name
        .strip_prefix(&format!("dispatch_record_p{}_s", std::process::id()))
        .and_then(|rest| rest.strip_suffix(".bin"));
    assert!(
        session.is_some_and(|session| session.parse::<u32>().is_ok()),
        "{name}"
    );
    recorder.record_segment(&first, 100.0, true);
    recorder.record_segment(&second, 100.0, false);
    recorder.record_drain(DrainCause::PacerBrake, 2);
    recorder.record_nudge(nudge);
    recorder.record_drain(DrainCause::Requested, 2);
    recorder.check().unwrap();
    drop(recorder);

    let mut expected = Vec::new();
    encode_header(&mut expected);
    encode_segment(&mut expected, &first, 100.0, true).unwrap();
    encode_segment(&mut expected, &second, 100.0, false).unwrap();
    encode_drain(&mut expected, DrainCause::PacerBrake, 2);
    encode_nudge(&mut expected, &nudge);
    encode_drain(&mut expected, DrainCause::Requested, 2);
    assert_eq!(only_record(&dir), expected);
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn every_recorder_opens_its_own_file() {
    let dir = scratch_dir("sessions");
    let first = DispatchRecorder::open(&dir).unwrap();
    let second = DispatchRecorder::open(&dir).unwrap();
    assert_ne!(first.path(), second.path());
    drop(first);
    drop(second);
    assert_eq!(std::fs::read_dir(&dir).unwrap().count(), 2);
    std::fs::remove_dir_all(&dir).unwrap();
}

fn assert_drop_reports_incomplete(recorder: DispatchRecorder) {
    let dropped = std::panic::catch_unwind(AssertUnwindSafe(move || drop(recorder)));
    assert!(
        dropped.is_err(),
        "closing a faulted record must fail loudly"
    );
}

#[test]
fn a_full_backlog_faults_the_record() {
    let (tx, _rx) = bounded(1);
    let recorder = DispatchRecorder::from_sender(tx, PathBuf::from("full.bin"));
    recorder.record_drain(DrainCause::Requested, 0);
    recorder.check().unwrap();
    recorder.record_drain(DrainCause::Requested, 0);
    let error = recorder.check().expect_err("a dropped entry is fatal");
    assert!(error.contains("backlog full"), "{error}");
    assert_drop_reports_incomplete(recorder);
}

#[test]
fn a_stopped_writer_faults_the_record() {
    let (tx, rx) = bounded(4);
    drop(rx);
    let recorder = DispatchRecorder::from_sender(tx, PathBuf::from("stopped.bin"));
    recorder.record_segment(&segment(0.0), 0.0, true);
    let error = recorder.check().expect_err("an unwritten entry is fatal");
    assert!(error.contains("writer stopped"), "{error}");
    assert_drop_reports_incomplete(recorder);
}

#[test]
fn a_segment_the_writer_cannot_encode_faults_the_record() {
    let dir = scratch_dir("encode-fault");
    let recorder = DispatchRecorder::open(&dir).unwrap();
    recorder.record_segment(&segment_with_axes(3, 0.0), 0.0, true);
    let deadline = Instant::now() + Duration::from_secs(5);
    let error = loop {
        if let Err(error) = recorder.check() {
            break error;
        }
        assert!(Instant::now() < deadline, "the writer never reported");
        std::thread::sleep(Duration::from_millis(1));
    };
    assert!(error.contains("3 axes"), "{error}");
    let recorder = Arc::into_inner(recorder).expect("the only holder");
    assert_drop_reports_incomplete(recorder);
    std::fs::remove_dir_all(&dir).unwrap();
}
