use super::{EtherCatRing, RingFiller, WireSink};
use crate::lock_ext::LockExt;
use crate::pump::{AxisFrame, AxisKey, SendError, SpanSink};
use ethercat_rt::server::FrameServer;
use ethercat_rt::setpoint_fill::{CLOCK_FREQ_HZ, ChainFiller, LaneSpec};
use ethercat_rt::wire::{Command, push_sample_runs_response_frame};
use host_rt::mcu_serial_conn::McuSerialConn;
use mcu_protocol::messages::{LANE_RUN_FLAG_REANCHOR, LANE_RUN_FLAG_TAIL, LaneRun};
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use trajectory::{
    ClockedMotorSpan, ContinuousAxis, MotorGroup, MotorSpan, MotorTerm, NudgeProfile,
};

const MCU_ID: u32 = 4;
const AXIS: u8 = 0;
const INTERVAL_NS: u64 = 250_000;
const CPM: f64 = 3_276.8;
const GRID_INDEX: u64 = 1_000;
const GRID_CLOCK: u64 = 8_000_000_000;
const SPAN_SECS: f64 = 0.010;
const SPAN_NS: u64 = 10_000_000;
/// Two of these overrun one fill window (256 cycles = 64 ms), so the tail of
/// the pair stays staged after the first drain — a lane only ever holds the
/// active view plus one successor, so depth comes from length, not count.
const DEEP_SPAN_SECS: f64 = 0.040;
const DEEP_SPAN_NS: u64 = 40_000_000;

/// The fake endpoint: serves the kalico socket, answers every
/// `PushSampleRuns` with the grid pair it is told to report, and keeps every
/// lane run it accepted so the test can assert the stream the sink produced.
struct RingEndpoint {
    received: Arc<Mutex<Vec<LaneRun>>>,
    grid_index: Arc<AtomicU32>,
    free_cycles: Arc<AtomicU32>,
    reject: Arc<AtomicBool>,
    stop: Arc<AtomicBool>,
    socket_path: String,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl RingEndpoint {
    fn start(name: &str) -> Self {
        let socket_path = format!("/tmp/kalico-ring-sink-{}-{name}.sock", std::process::id());
        let _ = std::fs::remove_file(&socket_path);
        let received = Arc::new(Mutex::new(Vec::new()));
        let grid_index = Arc::new(AtomicU32::new(0));
        let free_cycles = Arc::new(AtomicU32::new(1024));
        let reject = Arc::new(AtomicBool::new(false));
        let stop = Arc::new(AtomicBool::new(false));
        let (ready_tx, ready_rx) = std::sync::mpsc::sync_channel(1);

        let thread = {
            let (path, received, grid_index, free_cycles, reject, stop) = (
                socket_path.clone(),
                Arc::clone(&received),
                Arc::clone(&grid_index),
                Arc::clone(&free_cycles),
                Arc::clone(&reject),
                Arc::clone(&stop),
            );
            std::thread::spawn(move || {
                let mut server = FrameServer::bind(&path).expect("endpoint: bind");
                ready_tx.send(()).expect("endpoint readiness receiver");
                while !stop.load(Ordering::Relaxed) {
                    for cmd in server.poll_commands() {
                        if let Command::PushSampleRuns {
                            correlation_id,
                            msg,
                        } = cmd
                        {
                            let lanes: Vec<(u8, u32)> = msg
                                .lanes
                                .iter()
                                .map(|l| (l.axis_idx, free_cycles.load(Ordering::Relaxed)))
                                .collect();
                            let result = if reject.load(Ordering::Relaxed) {
                                -318
                            } else {
                                received.lock_ok().extend(msg.lanes);
                                0
                            };
                            let advance = u64::from(grid_index.load(Ordering::Relaxed));
                            server.respond(&push_sample_runs_response_frame(
                                correlation_id,
                                result,
                                GRID_CLOCK,
                                (GRID_INDEX + advance, GRID_CLOCK + advance * INTERVAL_NS),
                                &lanes,
                            ));
                        }
                    }
                    std::thread::sleep(Duration::from_millis(1));
                }
            })
        };

        ready_rx
            .recv_timeout(Duration::from_secs(2))
            .expect("endpoint socket never appeared");
        Self {
            received,
            grid_index,
            free_cycles,
            reject,
            stop,
            socket_path,
            thread: Some(thread),
        }
    }

    fn runs(&self) -> Vec<LaneRun> {
        self.received.lock_ok().clone()
    }
}

impl Drop for RingEndpoint {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
        let _ = std::fs::remove_file(&self.socket_path);
    }
}

struct Harness {
    endpoint: RingEndpoint,
    sink: WireSink,
    filler: RingFiller,
    _conn: Arc<McuSerialConn>,
}

fn harness(name: &str) -> Harness {
    let endpoint = RingEndpoint::start(name);
    let conn = Arc::new(
        McuSerialConn::connect(&endpoint.socket_path).expect("client connects to the endpoint"),
    );
    let mut chain = ChainFiller::new(
        &[LaneSpec {
            axis: AXIS,
            cmd_counts_per_mm: CPM,
            ff_lead_ns: 0,
        }],
        None,
        INTERVAL_NS,
        400,
    );
    chain
        .observe_grid(GRID_INDEX, GRID_CLOCK)
        .expect("the claim-time grid is the first observation");
    let filler: RingFiller = Arc::new(Mutex::new(chain));
    let sink = WireSink {
        stepcompress: HashMap::new(),
        samples: HashMap::new(),
        ethercat: HashMap::from([(
            MCU_ID,
            EtherCatRing {
                conn: Arc::downgrade(&conn),
                ring: Arc::clone(&filler),
            },
        )]),
        timeout: Duration::from_secs(5),
        transports: Arc::new(crate::axis_transport::AxisTransports::from_configs(&[])),
    };
    Harness {
        endpoint,
        sink,
        filler,
        _conn: conn,
    }
}

fn span(start_ns: u64, duration_s: f64, from_mm: f64, to_mm: f64) -> ClockedMotorSpan {
    let delta = to_mm - from_mm;
    let profile =
        NudgeProfile::try_new(delta, delta.abs() / duration_s, 0.0, 0.0).expect("cruise profile");
    let duration = profile.duration();
    let groups: Arc<[MotorGroup]> = Arc::from([
        MotorGroup::Independent(MotorTerm {
            source_axis: 0,
            axis: ContinuousAxis::Hold {
                position: from_mm,
                t_start: 0.0,
                t_end: duration,
            },
            scale: 1.0,
        }),
        MotorGroup::Independent(MotorTerm {
            source_axis: 0,
            axis: ContinuousAxis::Nudge(profile),
            scale: 1.0,
        }),
    ]);
    let signal =
        Arc::new(MotorSpan::try_new(groups, 0.0, duration, 0, 0, false).expect("motor span"));
    #[allow(clippy::cast_precision_loss)]
    let start_clock_exact = start_ns as f64;
    let start_host = start_clock_exact / CLOCK_FREQ_HZ;
    ClockedMotorSpan::try_new(
        Arc::clone(&signal),
        signal.t_start,
        signal.t_end,
        start_host,
        start_host + duration,
        start_clock_exact,
        CLOCK_FREQ_HZ,
    )
    .expect("a positive-duration view on the nanosecond DC clock")
}

fn linear_span(start_ns: u64, from_mm: f64, to_mm: f64) -> ClockedMotorSpan {
    span(start_ns, SPAN_SECS, from_mm, to_mm)
}

/// The deep stage: one lane's two slots filled with long views, so the fill
/// cannot ship all of it in a single window.
fn deep_spans(start_ns: u64) -> Vec<ClockedMotorSpan> {
    vec![
        span(start_ns, DEEP_SPAN_SECS, 0.0, 4.0),
        span(start_ns + DEEP_SPAN_NS, DEEP_SPAN_SECS, 4.0, 8.0),
    ]
}

fn frame(spans: Vec<ClockedMotorSpan>) -> AxisFrame {
    AxisFrame {
        axis: AXIS,
        spans,
        new_head: 0,
        room: 1024,
        guard_recorded_ns: 0,
        guard_mcu_clock: 0,
    }
}

fn key() -> AxisKey {
    AxisKey {
        mcu_id: MCU_ID,
        axis: AXIS,
    }
}

#[test]
fn a_ring_endpoint_receives_abutting_sample_runs_for_a_two_span_trajectory() {
    let h = harness("abut");
    let start = GRID_CLOCK + INTERVAL_NS * 8;
    h.sink
        .send_mcu_frames(
            MCU_ID,
            &[frame(vec![
                linear_span(start, 0.0, 1.0),
                linear_span(start + SPAN_NS, 1.0, 3.0),
            ])],
        )
        .expect("the ring endpoint accepts the fill");

    let runs = h.endpoint.runs();
    assert!(!runs.is_empty(), "the sink must have shipped lane runs");
    assert_eq!(
        runs[0].start_index,
        GRID_INDEX + 8,
        "the first run starts on the grid index covering the first view"
    );
    assert_eq!(
        runs[0].flags & LANE_RUN_FLAG_REANCHOR,
        LANE_RUN_FLAG_REANCHOR,
        "the first run of an epoch anchors the lane"
    );
    let mut next_index = runs[0].start_index;
    for run in &runs {
        assert_eq!(
            run.axis_idx, AXIS,
            "every run belongs to the endpoint's only lane"
        );
        assert_eq!(run.interval_ticks, INTERVAL_NS as u32);
        assert_eq!(
            run.start_index, next_index,
            "successive runs must abut on the grid without a gap or an overlap"
        );
        next_index = run.start_index + run.samples.len() as u64;
    }
    let covered: usize = runs.iter().map(|r| r.samples.len()).sum();
    assert_eq!(
        covered,
        (2 * SPAN_NS / INTERVAL_NS) as usize,
        "the two views are sampled once per DC cycle end to end"
    );
    let last = runs.last().expect("at least one run");
    assert_eq!(
        last.flags & LANE_RUN_FLAG_TAIL,
        LANE_RUN_FLAG_TAIL,
        "the run that reaches the end of the trajectory declares the hold"
    );
    let positions: Vec<i32> = runs
        .iter()
        .flat_map(|r| r.samples.iter().map(|s| s.pos_counts))
        .collect();
    assert_eq!(positions[0], 0, "the anchored epoch starts at its origin");
    assert!(
        positions.windows(2).all(|w| w[1] >= w[0]),
        "a monotonically rising trajectory yields monotonically rising counts"
    );
    let last_clock = start + INTERVAL_NS * (positions.len() as u64 - 1);
    let tail = linear_span(start + SPAN_NS, 1.0, 3.0);
    let expected_mm = tail
        .eval_at_clock(last_clock)
        .expect("the last grid clock lies inside the tail view")
        .position;
    let span_mm = f64::from(*positions.last().unwrap()) / CPM;
    assert!(
        (span_mm - expected_mm).abs() < 1.0 / CPM,
        "the last sample must be the analytic trajectory at the last grid clock: \
         got {span_mm} mm, expected {expected_mm} mm"
    );
}

/// The EtherCAT span transport carries a fresh-epoch discontinuity on the
/// wire and lets the endpoint re-create its count map, so `mark_reanchor` and
/// `mark_seam_gap` are no-ops for it. The ring must reach the same place from
/// the stream alone: the run before the hole declares the hold, the run after
/// it re-anchors.
#[test]
fn a_stream_time_hole_closes_one_run_and_re_anchors_the_next() {
    let h = harness("gap");
    let start = GRID_CLOCK + INTERVAL_NS * 8;
    h.sink
        .send_mcu_frames(MCU_ID, &[frame(vec![linear_span(start, 0.0, 1.0)])])
        .expect("the pre-hole stream is accepted");
    let before = h.endpoint.runs();
    assert_eq!(
        before.last().expect("a run").flags & LANE_RUN_FLAG_TAIL,
        LANE_RUN_FLAG_TAIL,
        "the run reaching the hole declares the hold"
    );

    let rejoin = start + SPAN_NS + INTERVAL_NS * 40;
    h.sink
        .send_mcu_frames(MCU_ID, &[frame(vec![linear_span(rejoin, 1.0, 2.0)])])
        .expect("the post-hole stream is accepted");
    let resumed = &h.endpoint.runs()[before.len()];
    assert_eq!(
        resumed.flags & LANE_RUN_FLAG_REANCHOR,
        LANE_RUN_FLAG_REANCHOR,
        "the run after a stream-time hole re-anchors rather than abutting"
    );
    assert_eq!(
        resumed.start_index,
        GRID_INDEX + 8 + SPAN_NS / INTERVAL_NS + 40,
        "it starts on the grid index covering the rejoin clock"
    );
}

#[test]
fn grid_feedback_advances_the_filler_so_a_later_fill_lands_on_the_reported_grid() {
    let h = harness("grid");
    let start = GRID_CLOCK + INTERVAL_NS * 8;
    h.sink
        .send_mcu_frames(MCU_ID, &[frame(vec![linear_span(start, 0.0, 1.0)])])
        .expect("first fill accepted");
    let first_len: u64 = h
        .endpoint
        .runs()
        .iter()
        .map(|r| r.samples.len() as u64)
        .sum();
    assert!(
        first_len > 0,
        "the first fill must have covered the trajectory it was given"
    );

    let advance: u32 = 200;
    h.endpoint.grid_index.store(advance, Ordering::Relaxed);
    h.filler.lock_ok().cut_axis(AXIS);
    let far = GRID_CLOCK + u64::from(advance) * INTERVAL_NS + INTERVAL_NS * 16;
    h.sink
        .send_mcu_frames(MCU_ID, &[frame(vec![linear_span(far, 5.0, 6.0)])])
        .expect("a fill against the advanced grid");
    let last = h.endpoint.runs().last().cloned().expect("runs exist");
    assert_eq!(
        last.start_index,
        GRID_INDEX + u64::from(advance) + 16,
        "indices after the feedback are measured from the reported pair, not the claim pair"
    );
}

#[test]
fn a_grid_that_regresses_is_fatal_rather_than_a_silent_reindex() {
    let h = harness("regress");
    h.endpoint.grid_index.store(500, Ordering::Relaxed);
    let start = GRID_CLOCK + 500 * INTERVAL_NS + INTERVAL_NS * 8;
    h.sink
        .send_mcu_frames(MCU_ID, &[frame(vec![linear_span(start, 0.0, 1.0)])])
        .expect("the advanced grid is accepted");

    h.endpoint.grid_index.store(0, Ordering::Relaxed);
    h.filler.lock_ok().cut_axis(AXIS);
    let error = h
        .sink
        .send_mcu_frames(MCU_ID, &[frame(vec![linear_span(start, 0.0, 1.0)])])
        .expect_err("a grid index below the last observed one must not be adopted");
    let SendError::Fatal(message) = error else {
        panic!("a grid regression must be fatal, got {error:?}");
    };
    assert!(
        message.contains("sample_fill_grid_regression"),
        "the error must name the invariant, got: {message}"
    );
}

#[test]
fn a_cut_drops_the_staged_runs_and_the_lane_re_anchors_loudly() {
    let h = harness("cut");
    let start = GRID_CLOCK + INTERVAL_NS * 8;
    h.endpoint.free_cycles.store(0, Ordering::Relaxed);
    h.sink
        .send_mcu_frames(MCU_ID, &[frame(deep_spans(start))])
        .expect("the first window is accepted");
    assert!(
        h.filler.lock_ok().wants_drain(),
        "the trajectory beyond the first window must still be staged"
    );
    let before = h.endpoint.runs().len();

    h.sink.flush_keys(&[key()]).expect("flush is accepted");
    assert!(
        !h.filler.lock_ok().wants_drain(),
        "a cut must drop every staged view — nothing may still reach the ring"
    );

    h.endpoint.free_cycles.store(1024, Ordering::Relaxed);
    let resumed = GRID_CLOCK + INTERVAL_NS * 4_000;
    h.sink
        .send_mcu_frames(MCU_ID, &[frame(vec![linear_span(resumed, 42.0, 43.0)])])
        .expect("post-cut motion is accepted");
    let runs = h.endpoint.runs();
    assert!(
        runs.len() > before,
        "the resumed stream must have reached the endpoint"
    );
    let first_after_cut = &runs[before];
    assert_eq!(
        first_after_cut.flags & LANE_RUN_FLAG_REANCHOR,
        LANE_RUN_FLAG_REANCHOR,
        "the run resuming a cut lane must re-anchor instead of claiming to continue"
    );
    assert_eq!(
        first_after_cut.start_index,
        GRID_INDEX + 4_000,
        "the re-anchored run starts where the resumed trajectory does"
    );
    assert_eq!(
        first_after_cut.samples[0].pos_counts, 0,
        "a re-anchored epoch restarts its count frame at its own origin"
    );
}

#[test]
fn a_halt_cuts_the_staged_lane_through_the_pump_sink_hook() {
    let h = harness("halt");
    let start = GRID_CLOCK + INTERVAL_NS * 8;
    h.endpoint.free_cycles.store(0, Ordering::Relaxed);
    h.sink
        .send_mcu_frames(MCU_ID, &[frame(deep_spans(start))])
        .expect("the first window is accepted");
    assert!(h.filler.lock_ok().wants_drain());

    h.sink.cut_staged(&[key()]).expect("halt cut");
    assert!(
        !h.filler.lock_ok().wants_drain(),
        "the halt hook must drop the stage exactly as flush does"
    );
}

#[test]
fn a_frame_for_an_axis_the_filler_does_not_drive_is_fatal() {
    let h = harness("unknown-axis");
    let mut stray = frame(vec![linear_span(GRID_CLOCK + INTERVAL_NS * 8, 0.0, 1.0)]);
    stray.axis = AXIS + 7;
    let error = h
        .sink
        .send_mcu_frames(MCU_ID, &[stray])
        .expect_err("an axis with no setpoint lane must not be silently dropped");
    let SendError::Fatal(message) = error else {
        panic!("an unknown lane must be fatal, got {error:?}");
    };
    assert!(
        message.contains("has no setpoint lane"),
        "the error must name the missing lane, got: {message}"
    );
}

#[test]
fn the_drain_tick_only_covers_ring_endpoints_and_only_while_they_owe_samples() {
    let h = harness("tick");
    assert_eq!(h.sink.drain_tick_mcus(), vec![MCU_ID]);
    assert!(
        !h.sink.wants_drain_tick(MCU_ID),
        "an endpoint with nothing staged owes no tick"
    );
    h.sink.drain_tick(MCU_ID).expect("an idle tick is a no-op");
    assert!(
        h.endpoint.runs().is_empty(),
        "an idle tick must not put anything on the wire"
    );

    let start = GRID_CLOCK + INTERVAL_NS * 8;
    h.endpoint.free_cycles.store(0, Ordering::Relaxed);
    h.sink
        .send_mcu_frames(MCU_ID, &[frame(deep_spans(start))])
        .expect("the first window is accepted");
    let after_send = h.endpoint.runs().len();
    assert!(h.sink.wants_drain_tick(MCU_ID));

    h.endpoint.free_cycles.store(1024, Ordering::Relaxed);
    h.sink.drain_tick(MCU_ID).expect("the tick ships the rest");
    assert!(
        h.endpoint.runs().len() > after_send,
        "the tick must ship the trajectory left over past the first window"
    );
    assert!(
        !h.sink.wants_drain_tick(MCU_ID),
        "once the stage is empty the endpoint owes no further tick"
    );
}

/// The pump re-sends a failed bundle byte-identically. A staged sample stream
/// is not idempotent the way a slot-addressed ring write is, so a rejected
/// fill must leave the lane with nothing staged — the retry restages from
/// scratch and its run re-anchors.
#[test]
fn a_rejected_fill_drops_the_stage_so_the_retry_re_anchors() {
    let h = harness("reject");
    let start = GRID_CLOCK + INTERVAL_NS * 8;
    let spans = deep_spans(start);
    h.endpoint.reject.store(true, Ordering::Relaxed);
    let error = h
        .sink
        .send_mcu_frames(MCU_ID, &[frame(spans.clone())])
        .expect_err("an endpoint reject must surface");
    assert!(
        matches!(error, SendError::Transient(_) | SendError::Fatal(_)),
        "a reject is an error, got {error:?}"
    );
    assert!(
        !h.filler.lock_ok().wants_drain(),
        "the failed bundle must not leave views staged for a non-idempotent re-send"
    );

    h.endpoint.reject.store(false, Ordering::Relaxed);
    let before = h.endpoint.runs().len();
    h.sink
        .send_mcu_frames(MCU_ID, &[frame(spans)])
        .expect("the retry is accepted");
    let runs = h.endpoint.runs();
    assert!(runs.len() > before, "the retry must reach the endpoint");
    assert_eq!(
        runs[before].flags & LANE_RUN_FLAG_REANCHOR,
        LANE_RUN_FLAG_REANCHOR,
        "the retry's first run re-anchors, discarding what the failed attempt left in the ring"
    );
    assert_eq!(
        runs[before].start_index,
        GRID_INDEX + 8,
        "the retry restages the whole bundle from its first view"
    );
}
