use super::*;
use runtime::sample_run::decode_deltas;
use std::sync::atomic::AtomicU64;

const MCU_ID: u32 = 3;
const OID: u32 = 7;
const CYCLES_PER_SECOND: f64 = 1_000_000.0;
const SAMPLE_RATE_HZ: u32 = 2_000;
const QUANTUM_MM: f32 = 0.01;

struct Sent {
    name: String,
    args: Vec<(String, ArgValue)>,
}

impl Sent {
    fn int(&self, name: &str) -> i64 {
        self.args
            .iter()
            .find(|(key, _)| key == name)
            .and_then(|(_, value)| match value {
                ArgValue::Int(v) => Some(*v),
                _ => None,
            })
            .unwrap_or_else(|| panic!("{} has no int arg {name}", self.name))
    }

    fn bytes(&self) -> Vec<u8> {
        self.args
            .iter()
            .find(|(key, _)| key == "data")
            .and_then(|(_, value)| match value {
                ArgValue::Bytes(v) => Some(v.clone()),
                _ => None,
            })
            .unwrap_or_else(|| panic!("{} has no data arg", self.name))
    }
}

struct Harness {
    endpoint: SampleEndpoint,
    now: Arc<AtomicU64>,
    sent: Arc<Mutex<Vec<Sent>>>,
    control: crossbeam_channel::Receiver<PumpMsg>,
    readback: Arc<Mutex<Option<(u64, i32)>>>,
}

impl Harness {
    fn advance(&self, ticks: u64) {
        self.now.fetch_add(ticks, Ordering::Relaxed);
    }

    fn taken(&self) -> Vec<Sent> {
        std::mem::take(&mut self.sent.lock_ok())
    }

    fn runs(&self) -> Vec<Sent> {
        self.taken()
            .into_iter()
            .filter(|s| s.name == SAMPLE_RUN_NAME)
            .collect()
    }

    fn drain_control(&self) {
        while self.control.try_recv().is_ok() {}
    }
}

fn lane_cfg(axis: u8, oid: u32) -> SampleLaneConfig {
    SampleLaneConfig {
        axis,
        oid,
        cycles_per_second: CYCLES_PER_SECOND,
        sample_rate_hz: SAMPLE_RATE_HZ,
        position_quantum_mm: QUANTUM_MM,
        max_units_per_sample: 4_096,
    }
}

fn harness(lanes: &[SampleLaneConfig]) -> Harness {
    let now = Arc::new(AtomicU64::new(0));
    let now_for_clock = Arc::clone(&now);
    let clock_of: ClockSource =
        Arc::new(move |_| Some((now_for_clock.load(Ordering::Relaxed), CYCLES_PER_SECOND)));
    let sent = Arc::new(Mutex::new(Vec::new()));
    let sent_for_egress = Arc::clone(&sent);
    let egress: FrameEgress = Arc::new(move |burst| {
        let mut log = sent_for_egress.lock_ok();
        for (name, args) in burst {
            log.push(Sent {
                name: (*name).to_string(),
                args: args.clone(),
            });
        }
        Ok(())
    });
    let (tx, control) = crossbeam_channel::unbounded();
    let mut endpoint = SampleEndpoint::new(MCU_ID, lanes, egress, clock_of, tx)
        .expect("lane config is representable");
    let readback = Arc::new(Mutex::new(None));
    let readback_for_query = Arc::clone(&readback);
    let query: SamplePositionQuery = Arc::new(move |_| {
        readback_for_query
            .lock_ok()
            .ok_or_else(|| "no readback armed".to_string())
    });
    endpoint.set_position_query(query);
    endpoint
        .reset_position(&vec![0; lanes.len()])
        .expect("seed matches the lane count");
    Harness {
        endpoint,
        now,
        sent,
        control,
        readback,
    }
}

/// One piece per call, chained so consecutive pieces abut.
fn piece(start_time: u64, from_mm: f32, to_mm: f32, duration: f32) -> PieceEntry {
    let mut entry = PieceEntry::zeroed();
    entry.start_time = start_time;
    entry.duration = duration;
    entry.coeff_count = 2;
    entry.coeffs[0] = 0.5 * (from_mm + to_mm);
    entry.coeffs[1] = 0.5 * (to_mm - from_mm);
    entry
}

fn overlay_piece(start_time: u64, to_mm: f32, duration: f32) -> PieceEntry {
    let mut entry = piece(start_time, 0.0, to_mm, duration);
    entry.motor_mask = 1;
    entry
}

fn frame(axis: u8, pieces: Vec<PieceEntry>) -> AxisFrame {
    AxisFrame {
        axis,
        pieces,
        start_slot: 0,
        new_head: 0,
        room: 64,
        guard_recorded_ns: 0,
        guard_mcu_clock: 0,
    }
}

/// Walk every `sample_run` on the wire, decoding it against the running lane
/// position, and return the reconstructed absolute samples.
fn reconstruct(runs: &[Sent], anchor: i32) -> Vec<i32> {
    let mut position = anchor;
    let mut out = Vec::new();
    for run in runs {
        let count = usize::try_from(run.int("count")).expect("count fits");
        let mut decoded = vec![0i32; count];
        decode_deltas(position, &run.bytes(), count, &mut decoded).expect("wire payload decodes");
        position = *decoded.last().expect("non-empty run");
        out.extend(decoded);
    }
    out
}

#[test]
fn a_lane_anchors_once_then_streams_abutting_runs() {
    let mut h = harness(&[lane_cfg(0, OID)]);
    h.endpoint
        .send_frames(MCU_ID, &[frame(0, vec![piece(0, 0.0, 10.0, 0.05)])])
        .expect("pieces accepted");
    h.advance((CYCLES_PER_SECOND * 0.1) as u64);
    h.endpoint.tick().expect("tick");

    let sent = h.taken();
    let anchors: Vec<&Sent> = sent
        .iter()
        .filter(|s| s.name == SAMPLE_ANCHOR_NAME)
        .collect();
    assert_eq!(anchors.len(), 1, "the lane anchors exactly once");
    let anchor = anchors[0];
    assert_eq!(anchor.int("oid"), i64::from(OID));

    let runs: Vec<&Sent> = sent.iter().filter(|s| s.name == SAMPLE_RUN_NAME).collect();
    assert!(!runs.is_empty(), "the lane streamed runs");
    let period = (CYCLES_PER_SECOND / f64::from(SAMPLE_RATE_HZ)) as u32;
    for run in &runs {
        assert_eq!(run.int("interval"), i64::from(period));
        assert!(run.int("count") > 0);
        assert!(
            run.bytes().len() <= SAMPLE_RUN_DATA_MAX,
            "payload must fit one wire block"
        );
        assert!(run.int("count") <= SAMPLE_RUN_COUNT_MAX as i64);
    }
    h.drain_control();
}

#[test]
fn runs_abut_exactly_on_the_wire() {
    let mut h = harness(&[lane_cfg(0, OID)]);
    h.endpoint
        .send_frames(
            MCU_ID,
            &[frame(
                0,
                vec![
                    piece(0, 0.0, 20.0, 0.05),
                    piece((CYCLES_PER_SECOND * 0.05) as u64, 20.0, 60.0, 0.05),
                ],
            )],
        )
        .expect("pieces accepted");
    h.advance((CYCLES_PER_SECOND * 0.2) as u64);
    h.endpoint.tick().expect("tick");

    let sent = h.taken();
    let anchor_clock = sent
        .iter()
        .find(|s| s.name == SAMPLE_ANCHOR_NAME)
        .expect("anchored")
        .int("clock") as u64;
    let mut expected = anchor_clock;
    let mut runs = 0;
    for run in sent.iter().filter(|s| s.name == SAMPLE_RUN_NAME) {
        let interval = run.int("interval") as u64;
        let count = run.int("count") as u64;
        expected += interval * count;
        runs += 1;
    }
    assert!(runs > 1, "the trajectory spans several runs");
    let lane_end = expected;
    assert_eq!(
        lane_end % (CYCLES_PER_SECOND as u64 / u64::from(SAMPLE_RATE_HZ)),
        anchor_clock % (CYCLES_PER_SECOND as u64 / u64::from(SAMPLE_RATE_HZ)),
        "abutting runs never leave the lane's sample grid"
    );
    h.drain_control();
}

#[test]
fn samples_land_on_the_lane_quantum_and_track_the_trajectory() {
    let mut h = harness(&[lane_cfg(0, OID)]);
    h.endpoint
        .send_frames(MCU_ID, &[frame(0, vec![piece(0, 0.0, 10.0, 0.05)])])
        .expect("pieces accepted");
    h.advance((CYCLES_PER_SECOND * 0.1) as u64);
    h.endpoint.tick().expect("tick");
    let sent = h.taken();
    let anchor = sent
        .iter()
        .find(|s| s.name == SAMPLE_ANCHOR_NAME)
        .expect("anchored")
        .int("position") as i32;
    let runs: Vec<Sent> = sent
        .into_iter()
        .filter(|s| s.name == SAMPLE_RUN_NAME)
        .collect();
    let samples = reconstruct(&runs, anchor);
    assert!(samples.len() >= 90, "0.05 s at 2 kHz is ~100 samples");
    assert!(
        samples.windows(2).all(|pair| pair[1] >= pair[0]),
        "a monotone move never walks the lane backwards"
    );
    let travelled =
        f32::from(i16::try_from(samples.last().copied().unwrap()).unwrap()) * QUANTUM_MM;
    assert!(
        (travelled - 10.0).abs() < 0.05,
        "the lane lands within a couple of quanta of 10 mm, got {travelled}"
    );
    h.drain_control();
}

#[test]
fn the_lane_window_bounds_runs_in_flight() {
    let mut h = harness(&[lane_cfg(0, OID)]);
    let mut pieces = Vec::new();
    let mut start = 0u64;
    let mut from = 0.0f32;
    for _ in 0..40 {
        pieces.push(piece(start, from, from + 5.0, 0.05));
        start += (CYCLES_PER_SECOND * 0.05) as u64;
        from += 5.0;
    }
    h.endpoint
        .send_frames(MCU_ID, &[frame(0, pieces)])
        .expect("pieces accepted");
    for _ in 0..8 {
        h.endpoint.tick().expect("tick");
    }
    let in_flight = h.endpoint.in_flight_runs();
    assert!(
        in_flight.iter().all(|&depth| depth <= SAMPLE_WINDOW_RUNS),
        "the in-flight window is bounded, saw {in_flight:?}"
    );
    h.drain_control();
}

#[test]
fn a_stalled_lane_never_outruns_the_send_lead() {
    let mut h = harness(&[lane_cfg(0, OID)]);
    let mut pieces = Vec::new();
    let mut start = 0u64;
    let mut from = 0.0f32;
    for _ in 0..200 {
        pieces.push(piece(start, from, from + 5.0, 0.05));
        start += (CYCLES_PER_SECOND * 0.05) as u64;
        from += 5.0;
    }
    h.endpoint
        .send_frames(MCU_ID, &[frame(0, pieces)])
        .expect("pieces accepted");
    let sent = h.taken();
    let lead_ticks = (CYCLES_PER_SECOND * SEND_LEAD_SECONDS) as u64;
    let furthest = sent
        .iter()
        .filter(|s| s.name == SAMPLE_RUN_NAME)
        .map(|run| run.int("interval") as u64 * run.int("count") as u64)
        .sum::<u64>();
    assert!(
        furthest <= lead_ticks + u64::from(SAMPLE_RATE_HZ),
        "the sink sent {furthest} ticks of samples against a {lead_ticks} tick lead"
    );
    h.drain_control();
}

#[test]
fn an_overlay_piece_rides_its_own_relativized_run() {
    let mut h = harness(&[lane_cfg(0, OID)]);
    h.endpoint
        .send_frames(MCU_ID, &[frame(0, vec![piece(0, 0.0, 10.0, 0.05)])])
        .expect("kinematic pieces accepted");
    let before = h.endpoint.lane_positions();
    h.taken();

    let overlay_start = (CYCLES_PER_SECOND * 0.05) as u64;
    h.endpoint
        .send_frames(
            MCU_ID,
            &[frame(0, vec![overlay_piece(overlay_start, 0.5, 0.02)])],
        )
        .expect("overlay accepted");
    h.endpoint.tick().expect("tick");

    let sent = h.taken();
    let overlays: Vec<&Sent> = sent
        .iter()
        .filter(|s| s.name == SAMPLE_OVERLAY_NAME)
        .collect();
    assert!(
        !overlays.is_empty(),
        "a motor_mask piece leaves as sample_overlay, not on the abutting stream"
    );
    let first = overlays[0];
    let count = usize::try_from(first.int("count")).expect("count fits");
    let mut decoded = vec![0i32; count];
    decode_deltas(0, &first.bytes(), count, &mut decoded).expect("overlay decodes against zero");
    assert!(
        decoded.first().copied().unwrap_or(i32::MAX).abs() < 8,
        "an overlay run starts relativized to its own frame, got {:?}",
        decoded.first()
    );
    assert_eq!(
        h.endpoint.lane_positions(),
        before,
        "an overlay leaves the lane's absolute frame where the kinematic stream left it"
    );
    h.drain_control();
}

#[test]
fn a_seam_gap_re_anchors_the_lane() {
    let mut h = harness(&[lane_cfg(0, OID)]);
    h.endpoint
        .send_frames(MCU_ID, &[frame(0, vec![piece(0, 0.0, 10.0, 0.05)])])
        .expect("pieces accepted");
    h.taken();

    let rejoin = (CYCLES_PER_SECOND * 0.2) as u64;
    h.endpoint.mark_seam_gap(0, rejoin).expect("axis exists");
    h.endpoint
        .send_frames(MCU_ID, &[frame(0, vec![piece(rejoin, 10.0, 20.0, 0.05)])])
        .expect("pieces past the gap accepted");
    h.endpoint.tick().expect("tick");

    let sent = h.taken();
    let anchors: Vec<&Sent> = sent
        .iter()
        .filter(|s| s.name == SAMPLE_ANCHOR_NAME)
        .collect();
    assert_eq!(
        anchors.len(),
        1,
        "the sanctioned hole is crossed by exactly one fresh anchor"
    );
    assert!(
        anchors[0].int("clock") as u64 >= rejoin,
        "the anchor lands at the rejoin, not before it"
    );
    h.drain_control();
}

#[test]
fn an_unsent_reanchor_cut_needs_no_barrier() {
    let mut h = harness(&[lane_cfg(0, OID)]);
    let cut_at = (CYCLES_PER_SECOND * 0.02) as u64;
    h.endpoint
        .mark_reanchor(0, cut_at, Some(CYCLES_PER_SECOND))
        .expect("axis exists");
    h.endpoint
        .send_frames(MCU_ID, &[frame(0, vec![piece(cut_at, 0.0, 10.0, 0.05)])])
        .expect("pieces accepted");
    let sent = h.taken();
    assert!(
        sent.iter().all(|s| s.name != SAMPLE_BARRIER_NAME),
        "nothing had reached the wire, so no receipt is owed"
    );
    h.drain_control();
}

#[test]
fn a_sent_reanchor_cut_parks_the_lane_until_the_barrier_reconciles() {
    let mut h = harness(&[lane_cfg(0, OID)]);
    h.endpoint
        .send_frames(MCU_ID, &[frame(0, vec![piece(0, 0.0, 10.0, 0.05)])])
        .expect("pieces accepted");
    let parked_at = h.endpoint.lane_positions()[0];
    h.taken();

    let cut_at = (CYCLES_PER_SECOND * 0.05) as u64;
    h.endpoint
        .mark_reanchor(0, cut_at, Some(CYCLES_PER_SECOND))
        .expect("axis exists");
    h.endpoint
        .send_frames(MCU_ID, &[frame(0, vec![piece(cut_at, 10.0, 30.0, 0.05)])])
        .expect("pieces held behind the cut");
    let sent = h.taken();
    let barrier = sent
        .iter()
        .find(|s| s.name == SAMPLE_BARRIER_NAME)
        .expect("a sent lane owes a barrier");
    let seq = u32::try_from(barrier.int("seq")).expect("seq fits");

    h.endpoint.tick().expect("a parked lane still ticks");
    assert!(
        h.taken().iter().all(|s| s.name != SAMPLE_RUN_NAME),
        "a parked lane emits nothing until the cut reconciles"
    );

    *h.readback.lock_ok() = Some((cut_at, i32::try_from(parked_at).expect("fits")));
    h.endpoint
        .on_barrier_ack(OID, seq)
        .expect("the readback matches the host's expectation");
    let sent = h.taken();
    assert!(
        sent.iter().any(|s| s.name == SAMPLE_ANCHOR_NAME),
        "reconciliation re-anchors the lane"
    );
    assert!(
        sent.iter().any(|s| s.name == SAMPLE_RUN_NAME),
        "the held pieces resume streaming"
    );
    h.drain_control();
}

#[test]
fn a_readback_disagreeing_with_the_host_is_fatal() {
    let mut h = harness(&[lane_cfg(0, OID)]);
    h.endpoint
        .send_frames(MCU_ID, &[frame(0, vec![piece(0, 0.0, 10.0, 0.05)])])
        .expect("pieces accepted");
    let parked_at = h.endpoint.lane_positions()[0];
    h.taken();

    let cut_at = (CYCLES_PER_SECOND * 0.05) as u64;
    h.endpoint
        .mark_reanchor(0, cut_at, Some(CYCLES_PER_SECOND))
        .expect("axis exists");
    h.endpoint
        .send_frames(MCU_ID, &[frame(0, vec![piece(cut_at, 10.0, 30.0, 0.05)])])
        .expect("pieces held");
    let seq = h
        .taken()
        .iter()
        .find(|s| s.name == SAMPLE_BARRIER_NAME)
        .map(|s| u32::try_from(s.int("seq")).expect("seq fits"))
        .expect("a barrier was issued");

    *h.readback.lock_ok() = Some((cut_at, i32::try_from(parked_at + 7).expect("fits")));
    let error = h
        .endpoint
        .on_barrier_ack(OID, seq)
        .expect_err("a position mismatch is never papered over");
    let SendError::Fatal(message) = error else {
        panic!("a reanchor mismatch must be fatal");
    };
    assert!(
        message.contains("reanchor position mismatch"),
        "unexpected message: {message}"
    );
    assert!(h.endpoint.is_fatal(), "the endpoint latches and stops");
    h.drain_control();
}

#[test]
fn an_unissued_barrier_ack_is_fatal() {
    let mut h = harness(&[lane_cfg(0, OID)]);
    let error = h
        .endpoint
        .on_barrier_ack(OID, 1)
        .expect_err("the mcu cannot ack a receipt the host never issued");
    let SendError::Fatal(message) = error else {
        panic!("a bogus ack must be fatal");
    };
    assert!(
        message.contains("is bogus"),
        "unexpected message: {message}"
    );
    h.drain_control();
}

#[test]
fn frames_for_a_foreign_mcu_are_fatal() {
    let mut h = harness(&[lane_cfg(0, OID)]);
    let error = h
        .endpoint
        .send_frames(MCU_ID + 1, &[frame(0, vec![piece(0, 0.0, 1.0, 0.01)])])
        .expect_err("a misaddressed bundle is never silently absorbed");
    assert!(matches!(error, SendError::Fatal(_)));
    h.drain_control();
}

#[test]
fn an_unconfigured_axis_is_fatal() {
    let mut h = harness(&[lane_cfg(0, OID)]);
    let error = h
        .endpoint
        .send_frames(MCU_ID, &[frame(3, vec![piece(0, 0.0, 1.0, 0.01)])])
        .expect_err("an unknown lane is never guessed at");
    let SendError::Fatal(message) = error else {
        panic!("an unknown axis must be fatal");
    };
    assert!(
        message.contains("no lane configured for axis 3"),
        "unexpected message: {message}"
    );
    h.drain_control();
}

#[test]
fn a_move_faster_than_the_lane_cap_is_fatal() {
    let mut cfg = lane_cfg(0, OID);
    cfg.max_units_per_sample = 4;
    let mut h = harness(&[cfg]);
    let error = h
        .endpoint
        .send_frames(MCU_ID, &[frame(0, vec![piece(0, 0.0, 100.0, 0.05)])])
        .expect_err("a lane never quietly drops motion it cannot represent");
    let SendError::Fatal(message) = error else {
        panic!("exceeding the lane cap must be fatal");
    };
    assert!(
        message.contains("exceeds the lane cap"),
        "unexpected message: {message}"
    );
    h.drain_control();
}

#[test]
fn duplicate_axes_are_rejected_at_construction() {
    let (tx, _rx) = crossbeam_channel::unbounded();
    let clock_of: ClockSource = Arc::new(|_| Some((0, CYCLES_PER_SECOND)));
    let egress: FrameEgress = Arc::new(|_| Ok(()));
    let Err(error) = SampleEndpoint::new(
        MCU_ID,
        &[lane_cfg(0, OID), lane_cfg(0, OID + 1)],
        egress,
        clock_of,
        tx,
    ) else {
        panic!("two lanes cannot own one axis");
    };
    let SendError::Fatal(message) = error else {
        panic!("a duplicate axis must be fatal");
    };
    assert!(
        message.contains("configured twice"),
        "unexpected message: {message}"
    );
}

#[test]
fn an_unrepresentable_sample_rate_is_rejected_at_construction() {
    let (tx, _rx) = crossbeam_channel::unbounded();
    let clock_of: ClockSource = Arc::new(|_| Some((0, CYCLES_PER_SECOND)));
    let egress: FrameEgress = Arc::new(|_| Ok(()));
    let mut cfg = lane_cfg(0, OID);
    cfg.sample_rate_hz = 4_000_000;
    let Err(error) = SampleEndpoint::new(MCU_ID, &[cfg], egress, clock_of, tx) else {
        panic!("a sample rate faster than the clock is not representable");
    };
    let SendError::Fatal(message) = error else {
        panic!("an unrepresentable rate must be fatal");
    };
    assert!(
        message.contains("not representable"),
        "unexpected message: {message}"
    );
}

#[test]
fn two_lanes_stream_independently() {
    let mut h = harness(&[lane_cfg(0, OID), lane_cfg(1, OID + 1)]);
    h.endpoint
        .send_frames(
            MCU_ID,
            &[
                frame(0, vec![piece(0, 0.0, 10.0, 0.05)]),
                frame(1, vec![piece(0, 0.0, -4.0, 0.05)]),
            ],
        )
        .expect("both lanes accepted");
    h.advance((CYCLES_PER_SECOND * 0.1) as u64);
    h.endpoint.tick().expect("tick");
    let sent = h.taken();
    for oid in [OID, OID + 1] {
        assert!(
            sent.iter()
                .any(|s| s.name == SAMPLE_ANCHOR_NAME && s.int("oid") == i64::from(oid)),
            "lane oid {oid} anchored"
        );
        assert!(
            sent.iter()
                .any(|s| s.name == SAMPLE_RUN_NAME && s.int("oid") == i64::from(oid)),
            "lane oid {oid} streamed"
        );
    }
    let positions = h.endpoint.lane_positions();
    assert!(positions[0] > 0 && positions[1] < 0, "got {positions:?}");
    h.drain_control();
}

#[test]
fn retirement_counts_the_pieces_the_lane_has_consumed() {
    let mut h = harness(&[lane_cfg(0, OID)]);
    assert_eq!(h.endpoint.retired_counts(), vec![0]);
    h.endpoint
        .send_frames(
            MCU_ID,
            &[frame(
                0,
                vec![
                    piece(0, 0.0, 5.0, 0.02),
                    piece((CYCLES_PER_SECOND * 0.02) as u64, 5.0, 10.0, 0.02),
                ],
            )],
        )
        .expect("pieces accepted");
    h.advance((CYCLES_PER_SECOND * 0.1) as u64);
    h.endpoint.tick().expect("tick");
    assert_eq!(
        h.endpoint.retired_counts(),
        vec![2],
        "both pieces were sampled through"
    );
    h.drain_control();
}

#[test]
fn every_payload_fits_one_wire_block() {
    let mut h = harness(&[lane_cfg(0, OID)]);
    let mut pieces = Vec::new();
    let mut start = 0u64;
    let mut from = 0.0f32;
    for index in 0..30 {
        let to = from + if index % 2 == 0 { 30.0 } else { -30.0 };
        pieces.push(piece(start, from, to, 0.02));
        start += (CYCLES_PER_SECOND * 0.02) as u64;
        from = to;
    }
    h.endpoint
        .send_frames(MCU_ID, &[frame(0, pieces)])
        .expect("pieces accepted");
    for _ in 0..6 {
        h.advance((CYCLES_PER_SECOND * 0.05) as u64);
        h.endpoint.tick().expect("tick");
        for run in h.runs() {
            assert!(
                run.bytes().len() <= SAMPLE_RUN_DATA_MAX,
                "payload of {} bytes overruns the wire cap",
                run.bytes().len()
            );
            assert!(run.int("count") <= SAMPLE_RUN_COUNT_MAX as i64);
        }
    }
    h.drain_control();
}

#[test]
fn a_stream_hole_without_a_seam_marker_is_fatal() {
    let mut h = harness(&[lane_cfg(0, OID)]);
    h.endpoint
        .send_frames(MCU_ID, &[frame(0, vec![piece(0, 0.0, 10.0, 0.05)])])
        .expect("pieces accepted");
    let hole = (CYCLES_PER_SECOND * 0.2) as u64;
    let error = h
        .endpoint
        .send_frames(MCU_ID, &[frame(0, vec![piece(hole, 10.0, 20.0, 0.05)])])
        .expect_err("an unsanctioned hole is never padded over");
    let SendError::Fatal(message) = error else {
        panic!("a stream hole must be fatal");
    };
    assert!(
        message.contains("hole after the sample at"),
        "unexpected message: {message}"
    );
    h.drain_control();
}
