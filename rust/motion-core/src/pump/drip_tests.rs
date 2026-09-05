use super::*;
use crate::lock_ext::LockExt;
use crossbeam_channel::unbounded;
use std::sync::atomic::AtomicU64;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use trajectory::{ClockedMotorSpan, ContinuousAxis, MotorGroup, MotorSpan, MotorTerm};

/// The drip lanes run on the 1 kHz clock the pump callbacks report, so a
/// span's start clock is its start time in milliseconds.
const FREQ: f64 = 1000.0;
const SOURCE_LINE: u32 = 5;

#[allow(clippy::cast_precision_loss)]
fn span_dur(start_clock: u64, secs: f64) -> ClockedMotorSpan {
    let t_start = start_clock as f64 / FREQ;
    let t_end = t_start + secs;
    let groups: Arc<[MotorGroup]> = Arc::from(vec![MotorGroup::Independent(MotorTerm {
        source_axis: 0,
        axis: ContinuousAxis::Hold {
            position: 0.0,
            t_start,
            t_end,
        },
        scale: 1.0,
    })]);
    let signal = MotorSpan::try_new(groups, t_start, t_end, 0, SOURCE_LINE, true)
        .expect("a hold motor span is dispatchable");
    ClockedMotorSpan::try_new(
        Arc::new(signal),
        t_start,
        t_end,
        t_start,
        t_end,
        start_clock as f64,
        FREQ,
    )
    .expect("the projected view spans at least one clock")
}

fn span(start_clock: u64) -> ClockedMotorSpan {
    span_dur(start_clock, 0.001)
}

struct NullSink;

impl SpanSink for NullSink {
    fn send_frame(
        &self,
        _key: AxisKey,
        _spans: &[ClockedMotorSpan],
        _new_head: u32,
        _room: u32,
    ) -> Result<i32, SendError> {
        Ok(mcu_protocol::result_codes::OK)
    }
}

#[derive(Clone)]
struct CountingSink {
    sent: Arc<Mutex<Vec<(AxisKey, u64)>>>,
}

impl CountingSink {
    fn new() -> Self {
        Self {
            sent: Arc::new(Mutex::new(Vec::new())),
        }
    }
    fn sent(&self) -> Vec<(AxisKey, u64)> {
        self.sent.lock_ok().clone()
    }
}

impl SpanSink for CountingSink {
    fn send_frame(
        &self,
        key: AxisKey,
        spans: &[ClockedMotorSpan],
        _new_head: u32,
        _room: u32,
    ) -> Result<i32, SendError> {
        let mut sent = self.sent.lock_ok();
        for s in spans {
            sent.push((key, s.start_clock));
        }
        Ok(mcu_protocol::result_codes::OK)
    }
}

#[test]
fn stall_detection_fires_when_floor_stuck() {
    let ka = AxisKey { mcu_id: 0, axis: 0 };
    let kb = AxisKey { mcu_id: 0, axis: 1 };
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let stall_msgs: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let stall_msgs_clone = Arc::clone(&stall_msgs);

    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            NullSink,
            PumpCallbacks {
                on_drip_stall: Box::new(move |msg: String| {
                    stall_msgs_clone.lock_ok().push(msg);
                }),
                ..PumpCallbacks::noop(64)
            },
            None,
            std::sync::Arc::new(crate::drain::DrainLedger::new()),
            Arc::new(AtomicU64::new(0)),
        );
    });

    ctl.send(PumpMsg::DripArm(DripArm {
        cohort: 55,
        participants: vec![ka, kb],
        timeout: Duration::from_millis(30),
    }))
    .unwrap();

    for key in [ka, kb] {
        data.send(EnqueueMsg {
            epoch_freq: None,
            key,
            spans: (0..20).map(|i| span_dur(i as u64, 0.003)).collect(),
            epoch: crate::anchor::StreamEpoch::Continuation,
            lead_secs: DRIP_WINDOW_SECS,
            source_line: SOURCE_LINE,
            batch_end: true,
        })
        .unwrap();
    }

    std::thread::sleep(Duration::from_millis(200));

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();

    let msgs = stall_msgs.lock_ok();
    assert_eq!(msgs.len(), 1, "expected one stall, got: {msgs:?}");
    assert!(
        msgs[0].contains("execution stalled"),
        "stall must identify missing execution progress; got: {}",
        msgs[0]
    );
}

#[test]
fn advancing_lane_does_not_hide_a_stalled_lane() {
    let advancing = AxisKey { mcu_id: 0, axis: 0 };
    let stalled = AxisKey { mcu_id: 0, axis: 1 };
    let sink = CountingSink::new();
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let stall_msgs: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let stall_msgs_clone = Arc::clone(&stall_msgs);

    let sink_clone = sink.clone();
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            sink_clone,
            PumpCallbacks {
                mcu_clock_of: Box::new(|_| Some((0u64, FREQ))),
                on_drip_stall: Box::new(move |msg: String| {
                    stall_msgs_clone.lock_ok().push(msg);
                }),
                ..PumpCallbacks::noop(64)
            },
            None,
            std::sync::Arc::new(crate::drain::DrainLedger::new()),
            Arc::new(AtomicU64::new(0)),
        );
    });

    for (key, batch_end) in [(advancing, false), (stalled, true)] {
        data.send(EnqueueMsg {
            epoch_freq: None,
            key,
            spans: (0..20).map(|i| span_dur(i as u64, 0.003)).collect(),
            epoch: crate::anchor::StreamEpoch::Continuation,
            lead_secs: DRIP_WINDOW_SECS,
            source_line: SOURCE_LINE,
            batch_end,
        })
        .unwrap();
    }

    let send_deadline = Instant::now() + Duration::from_secs(2);
    while sink.sent().len() < 40 {
        assert!(
            Instant::now() < send_deadline,
            "pump did not send both lanes"
        );
        std::thread::yield_now();
    }

    ctl.send(PumpMsg::DripArm(DripArm {
        cohort: 56,
        participants: vec![advancing, stalled],
        timeout: Duration::from_millis(60),
    }))
    .unwrap();

    for retired in 1..=5 {
        ctl.send(PumpMsg::Heartbeat(HeartbeatMsg {
            mcu_id: 0,
            axes: vec![0, 1],
            consumed_counts: None,
            retired_counts: vec![retired, 0],
            retired_by: RetiredBy::Pulse,
        }))
        .unwrap();
        std::thread::sleep(Duration::from_millis(20));
    }

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();

    let msgs = stall_msgs.lock_ok();
    assert_eq!(
        msgs.len(),
        1,
        "expected one stalled-lane fault, got: {msgs:?}"
    );
    assert!(
        msgs[0].contains("mcu0 axis1: executed 0"),
        "fault must identify the lane that made no progress: {}",
        msgs[0]
    );
}

#[test]
fn fully_executed_cohort_awaiting_trip_is_not_a_stall() {
    let ka = AxisKey { mcu_id: 0, axis: 0 };
    let kb = AxisKey { mcu_id: 0, axis: 1 };
    let sink = CountingSink::new();
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let stall_msgs: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let stall_msgs_clone = Arc::clone(&stall_msgs);

    let sink_clone = sink.clone();
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            sink_clone,
            PumpCallbacks {
                mcu_clock_of: Box::new(|_| Some((0u64, FREQ))),
                on_drip_stall: Box::new(move |msg: String| {
                    stall_msgs_clone.lock_ok().push(msg);
                }),
                ..PumpCallbacks::noop(64)
            },
            None,
            std::sync::Arc::new(crate::drain::DrainLedger::new()),
            Arc::new(AtomicU64::new(0)),
        );
    });

    ctl.send(PumpMsg::DripArm(DripArm {
        cohort: 77,
        participants: vec![ka, kb],
        timeout: Duration::from_millis(30),
    }))
    .unwrap();
    for key in [ka, kb] {
        data.send(EnqueueMsg {
            epoch_freq: None,
            key,
            spans: (0..5).map(|i| span(i as u64)).collect(),
            epoch: crate::anchor::StreamEpoch::Continuation,
            lead_secs: MAX_LEAD_SECS,
            source_line: SOURCE_LINE,
            batch_end: true,
        })
        .unwrap();
    }

    let deadline = Instant::now() + Duration::from_secs(2);
    while sink.sent().len() < 10 {
        assert!(
            Instant::now() < deadline,
            "pump never sent the drip spans; sent: {:?}",
            sink.sent()
        );
        std::thread::yield_now();
    }
    ctl.send(PumpMsg::Heartbeat(HeartbeatMsg {
        mcu_id: 0,
        axes: vec![0, 1, 2, 3],
        consumed_counts: None,
        retired_counts: vec![5, 5, 0, 0],
        retired_by: RetiredBy::Pulse,
    }))
    .unwrap();

    std::thread::sleep(Duration::from_millis(200));
    assert!(
        stall_msgs.lock_ok().is_empty(),
        "an executed-and-drained cohort awaiting its trip must not abort: {:?}",
        stall_msgs.lock_ok()
    );

    ctl.send(PumpMsg::DripDisarm(77)).unwrap();
    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
    assert!(stall_msgs.lock_ok().is_empty());
}

/// A cohort releases exactly one view per lane per pass, so the lanes take
/// turns instead of one lane spending the whole window: a fair drip.
#[test]
fn a_cohort_releases_its_lanes_in_lockstep() {
    let ka = AxisKey { mcu_id: 0, axis: 0 };
    let kb = AxisKey { mcu_id: 0, axis: 1 };
    let sink = CountingSink::new();
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();

    let sink_clone = sink.clone();
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            sink_clone,
            PumpCallbacks {
                mcu_clock_of: Box::new(|_| Some((0u64, FREQ))),
                ..PumpCallbacks::noop(64)
            },
            None,
            std::sync::Arc::new(crate::drain::DrainLedger::new()),
            Arc::new(AtomicU64::new(0)),
        );
    });

    ctl.send(PumpMsg::DripArm(DripArm {
        cohort: 66,
        participants: vec![ka, kb],
        timeout: Duration::from_secs(60),
    }))
    .unwrap();
    for key in [ka, kb] {
        data.send(EnqueueMsg {
            epoch_freq: None,
            key,
            spans: (0..4).map(|i| span_dur(i as u64 * 10, 0.01)).collect(),
            epoch: crate::anchor::StreamEpoch::Continuation,
            lead_secs: DRIP_WINDOW_SECS,
            source_line: SOURCE_LINE,
            batch_end: key == kb,
        })
        .unwrap();
    }

    let deadline = Instant::now() + Duration::from_secs(2);
    while sink.sent().len() < 8 {
        assert!(
            Instant::now() < deadline,
            "cohort never released every view; sent: {:?}",
            sink.sent()
        );
        std::thread::yield_now();
    }
    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();

    let sent = sink.sent();
    for (index, chunk) in sent.chunks(2).enumerate() {
        let mut keys: Vec<AxisKey> = chunk.iter().map(|(k, _)| *k).collect();
        keys.sort();
        assert_eq!(
            keys,
            vec![ka, kb],
            "release round {index} must carry one view of each lane: {sent:?}"
        );
    }
}

/// A parked ethercat lane gets no spans during another axis's homing drip
/// (pure-hold lanes are skipped at enqueue), yet it is a cohort participant.
/// With nothing queued and nothing in flight it cannot execute anything, so
/// it must not pin the cohort floor at zero — progress on the lanes that do
/// have work must keep resetting the stall deadline.
#[test]
fn idle_participant_does_not_pin_the_cohort_floor() {
    let active = AxisKey { mcu_id: 0, axis: 0 };
    let parked = AxisKey { mcu_id: 2, axis: 0 };
    let sink = CountingSink::new();
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let stall_msgs: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let stall_msgs_clone = Arc::clone(&stall_msgs);

    let sink_clone = sink.clone();
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            sink_clone,
            PumpCallbacks {
                mcu_clock_of: Box::new(|_| Some((0u64, FREQ))),
                on_drip_stall: Box::new(move |msg: String| {
                    stall_msgs_clone.lock_ok().push(msg);
                }),
                ..PumpCallbacks::noop(64)
            },
            None,
            std::sync::Arc::new(crate::drain::DrainLedger::new()),
            Arc::new(AtomicU64::new(0)),
        );
    });

    ctl.send(PumpMsg::DripArm(DripArm {
        cohort: 88,
        participants: vec![active, parked],
        timeout: Duration::from_millis(60),
    }))
    .unwrap();

    for step in 0u32..4 {
        data.send(EnqueueMsg {
            epoch_freq: None,
            key: active,
            spans: vec![span(u64::from(step))],
            epoch: crate::anchor::StreamEpoch::Continuation,
            lead_secs: MAX_LEAD_SECS,
            source_line: SOURCE_LINE,
            batch_end: true,
        })
        .unwrap();
        let deadline = Instant::now() + Duration::from_secs(2);
        while sink.sent().len() < (step + 1) as usize {
            assert!(
                Instant::now() < deadline,
                "pump never sent drip span {step}; sent: {:?}",
                sink.sent()
            );
            std::thread::yield_now();
        }
        ctl.send(PumpMsg::Heartbeat(HeartbeatMsg {
            mcu_id: 0,
            axes: vec![0, 1, 2, 3],
            consumed_counts: None,
            retired_counts: vec![step + 1, 0, 0, 0],
            retired_by: RetiredBy::Pulse,
        }))
        .unwrap();
        std::thread::sleep(Duration::from_millis(40));
    }

    assert!(
        stall_msgs.lock_ok().is_empty(),
        "an idle participant must not stall a progressing cohort: {:?}",
        stall_msgs.lock_ok()
    );

    ctl.send(PumpMsg::DripDisarm(88)).unwrap();
    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

#[test]
fn non_participant_enqueue_aborts_cohort_and_drops_spans() {
    let participant = AxisKey { mcu_id: 0, axis: 0 };
    let outsider = AxisKey { mcu_id: 0, axis: 3 };
    let sink = CountingSink::new();
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let stall_msgs: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let stall_msgs_clone = Arc::clone(&stall_msgs);

    let sink_clone = sink.clone();
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            sink_clone,
            PumpCallbacks {
                mcu_clock_of: Box::new(|_| Some((0u64, FREQ))),
                on_drip_stall: Box::new(move |msg: String| {
                    stall_msgs_clone.lock_ok().push(msg);
                }),
                ..PumpCallbacks::noop(64)
            },
            None,
            std::sync::Arc::new(crate::drain::DrainLedger::new()),
            Arc::new(AtomicU64::new(0)),
        );
    });

    ctl.send(PumpMsg::DripArm(DripArm {
        cohort: 9,
        participants: vec![participant],
        timeout: Duration::from_secs(60),
    }))
    .unwrap();
    data.send(EnqueueMsg {
        epoch_freq: None,
        key: outsider,
        spans: (0..3).map(|i| span(i as u64)).collect(),
        epoch: crate::anchor::StreamEpoch::Continuation,
        lead_secs: MAX_LEAD_SECS,
        source_line: SOURCE_LINE,
        batch_end: true,
    })
    .unwrap();

    let deadline = Instant::now() + Duration::from_secs(2);
    while stall_msgs.lock_ok().is_empty() {
        assert!(
            Instant::now() < deadline,
            "non-participant enqueue never aborted the cohort"
        );
        std::thread::yield_now();
    }

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();

    let msgs = stall_msgs.lock_ok();
    assert_eq!(msgs.len(), 1, "expected one abort, got: {msgs:?}");
    assert!(
        msgs[0].contains("non-participant"),
        "abort must name the violation; got: {}",
        msgs[0]
    );
    assert!(
        sink.sent().is_empty(),
        "outsider spans must be dropped, got {:?}",
        sink.sent()
    );
}

#[test]
fn participant_release_tracks_mcu_clock_horizon() {
    let ka = AxisKey { mcu_id: 0, axis: 0 };
    let sink = CountingSink::new();
    let clock: Arc<Mutex<u64>> = Arc::new(Mutex::new(0));
    let clock_for_pump = Arc::clone(&clock);
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();

    let sink_clone = sink.clone();
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            sink_clone,
            PumpCallbacks {
                mcu_clock_of: Box::new(move |_| Some((*clock_for_pump.lock_ok(), FREQ))),
                ..PumpCallbacks::noop(64)
            },
            None,
            std::sync::Arc::new(crate::drain::DrainLedger::new()),
            Arc::new(AtomicU64::new(0)),
        );
    });

    ctl.send(PumpMsg::DripArm(DripArm {
        cohort: 12,
        participants: vec![ka],
        timeout: Duration::from_secs(60),
    }))
    .unwrap();
    data.send(EnqueueMsg {
        epoch_freq: None,
        key: ka,
        spans: vec![span(50), span(500)],
        epoch: crate::anchor::StreamEpoch::Continuation,
        lead_secs: DRIP_WINDOW_SECS,
        source_line: SOURCE_LINE,
        batch_end: true,
    })
    .unwrap();

    let deadline = Instant::now() + Duration::from_secs(2);
    while sink.sent().is_empty() {
        assert!(Instant::now() < deadline, "first span not sent");
        std::thread::yield_now();
    }
    std::thread::sleep(Duration::from_millis(50));
    assert_eq!(
        sink.sent(),
        vec![(ka, 50)],
        "the span at 500 is beyond horizon 100 and must be held"
    );

    *clock.lock_ok() = 450;
    let deadline = Instant::now() + Duration::from_secs(2);
    while sink.sent().len() < 2 {
        assert!(
            Instant::now() < deadline,
            "held span not released after clock advance"
        );
        std::thread::sleep(Duration::from_millis(5));
    }
    assert_eq!(sink.sent(), vec![(ka, 50), (ka, 500)]);

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

#[test]
fn unsynced_clock_releases_nothing_for_participants() {
    let ka = AxisKey { mcu_id: 0, axis: 0 };
    let sink = CountingSink::new();
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();

    ctl.send(PumpMsg::DripArm(DripArm {
        cohort: 13,
        participants: vec![ka],
        timeout: Duration::from_secs(60),
    }))
    .unwrap();
    data.send(EnqueueMsg {
        epoch_freq: None,
        key: ka,
        spans: (10..14).map(|i| span(i as u64)).collect(),
        epoch: crate::anchor::StreamEpoch::Continuation,
        lead_secs: DRIP_WINDOW_SECS,
        source_line: SOURCE_LINE,
        batch_end: true,
    })
    .unwrap();

    let sink_clone = sink.clone();
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            sink_clone,
            PumpCallbacks::noop(64),
            None,
            std::sync::Arc::new(crate::drain::DrainLedger::new()),
            Arc::new(AtomicU64::new(0)),
        );
    });
    std::thread::sleep(Duration::from_millis(100));
    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();

    assert!(
        sink.sent().is_empty(),
        "nothing may release without a clock, got {:?}",
        sink.sent()
    );
}

#[test]
fn retired_regression_triggers_on_drip_stall() {
    let ka = AxisKey { mcu_id: 3, axis: 2 };
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (_data, data_rx) = unbounded::<EnqueueMsg>();
    let stall_msgs: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let stall_msgs_clone = Arc::clone(&stall_msgs);

    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            NullSink,
            PumpCallbacks {
                on_drip_stall: Box::new(move |msg: String| {
                    stall_msgs_clone.lock_ok().push(msg);
                }),
                ..PumpCallbacks::noop(64)
            },
            None,
            std::sync::Arc::new(crate::drain::DrainLedger::new()),
            Arc::new(AtomicU64::new(0)),
        );
    });

    ctl.send(PumpMsg::DripArm(DripArm {
        cohort: 7,
        participants: vec![ka],
        timeout: Duration::from_secs(60),
    }))
    .unwrap();

    for retired in [5u32, 3] {
        ctl.send(PumpMsg::Heartbeat(HeartbeatMsg {
            mcu_id: 3,
            axes: vec![0, 1, 2],
            consumed_counts: None,
            retired_counts: vec![0, 0, retired],
            retired_by: RetiredBy::Pulse,
        }))
        .unwrap();
        std::thread::sleep(Duration::from_millis(50));
    }

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();

    let msgs = stall_msgs.lock_ok();
    assert_eq!(msgs.len(), 1, "expected one stall error, got: {msgs:?}");
    assert!(
        msgs[0].contains("regression") && msgs[0].contains("mcu3") && msgs[0].contains("axis2"),
        "error must describe the regression; got: {}",
        msgs[0]
    );
}

#[test]
fn mcu_reboot_retired_to_zero_triggers_regression() {
    let ka = AxisKey { mcu_id: 1, axis: 0 };
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let stall_msgs: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let stall_msgs_clone = Arc::clone(&stall_msgs);

    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            NullSink,
            PumpCallbacks {
                on_drip_stall: Box::new(move |msg: String| {
                    stall_msgs_clone.lock_ok().push(msg);
                }),
                ..PumpCallbacks::noop(64)
            },
            None,
            std::sync::Arc::new(crate::drain::DrainLedger::new()),
            Arc::new(AtomicU64::new(0)),
        );
    });

    data.send(EnqueueMsg {
        epoch_freq: None,
        key: ka,
        spans: vec![span(10)],
        epoch: crate::anchor::StreamEpoch::Continuation,
        lead_secs: DRIP_WINDOW_SECS,
        source_line: SOURCE_LINE,
        batch_end: true,
    })
    .unwrap();
    std::thread::sleep(Duration::from_millis(30));
    ctl.send(PumpMsg::Heartbeat(HeartbeatMsg {
        mcu_id: 1,
        axes: vec![0],
        consumed_counts: None,
        retired_counts: vec![40],
        retired_by: RetiredBy::Pulse,
    }))
    .unwrap();
    std::thread::sleep(Duration::from_millis(30));

    ctl.send(PumpMsg::DripArm(DripArm {
        cohort: 21,
        participants: vec![ka],
        timeout: Duration::from_secs(60),
    }))
    .unwrap();
    std::thread::sleep(Duration::from_millis(30));

    ctl.send(PumpMsg::Heartbeat(HeartbeatMsg {
        mcu_id: 1,
        axes: vec![0],
        consumed_counts: None,
        retired_counts: vec![0],
        retired_by: RetiredBy::Pulse,
    }))
    .unwrap();
    std::thread::sleep(Duration::from_millis(50));

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();

    let msgs = stall_msgs.lock_ok();
    assert_eq!(msgs.len(), 1, "expected one regression, got: {msgs:?}");
    assert!(msgs[0].contains("regression"), "got: {}", msgs[0]);
}

#[test]
fn drip_disarm_clears_cohort() {
    let ka = AxisKey { mcu_id: 0, axis: 0 };
    let outsider = AxisKey { mcu_id: 0, axis: 3 };
    let sink = CountingSink::new();
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let stall_msgs: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let stall_msgs_clone = Arc::clone(&stall_msgs);

    let sink_clone = sink.clone();
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            sink_clone,
            PumpCallbacks {
                mcu_clock_of: Box::new(|_| Some((0u64, FREQ))),
                on_drip_stall: Box::new(move |msg: String| {
                    stall_msgs_clone.lock_ok().push(msg);
                }),
                ..PumpCallbacks::noop(64)
            },
            None,
            std::sync::Arc::new(crate::drain::DrainLedger::new()),
            Arc::new(AtomicU64::new(0)),
        );
    });

    ctl.send(PumpMsg::DripArm(DripArm {
        cohort: 31,
        participants: vec![ka],
        timeout: Duration::from_secs(60),
    }))
    .unwrap();
    ctl.send(PumpMsg::DripDisarm(31)).unwrap();
    data.send(EnqueueMsg {
        epoch_freq: None,
        key: outsider,
        spans: vec![span(1)],
        epoch: crate::anchor::StreamEpoch::Continuation,
        lead_secs: MAX_LEAD_SECS,
        source_line: SOURCE_LINE,
        batch_end: true,
    })
    .unwrap();

    let deadline = Instant::now() + Duration::from_secs(2);
    while sink.sent().is_empty() {
        assert!(
            Instant::now() < deadline,
            "outsider span not sent after disarm"
        );
        std::thread::sleep(Duration::from_millis(5));
    }
    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();

    assert!(stall_msgs.lock_ok().is_empty());
    assert_eq!(sink.sent(), vec![(outsider, 1)]);
}

#[test]
fn drip_disarm_wrong_cohort_id_is_noop() {
    let ka = AxisKey { mcu_id: 0, axis: 0 };
    let outsider = AxisKey { mcu_id: 0, axis: 3 };
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let stall_msgs: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let stall_msgs_clone = Arc::clone(&stall_msgs);

    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            NullSink,
            PumpCallbacks {
                on_drip_stall: Box::new(move |msg: String| {
                    stall_msgs_clone.lock_ok().push(msg);
                }),
                ..PumpCallbacks::noop(64)
            },
            None,
            std::sync::Arc::new(crate::drain::DrainLedger::new()),
            Arc::new(AtomicU64::new(0)),
        );
    });

    ctl.send(PumpMsg::DripArm(DripArm {
        cohort: 31,
        participants: vec![ka],
        timeout: Duration::from_secs(60),
    }))
    .unwrap();
    ctl.send(PumpMsg::DripDisarm(999)).unwrap();
    data.send(EnqueueMsg {
        epoch_freq: None,
        key: outsider,
        spans: vec![span(1)],
        epoch: crate::anchor::StreamEpoch::Continuation,
        lead_secs: MAX_LEAD_SECS,
        source_line: SOURCE_LINE,
        batch_end: true,
    })
    .unwrap();

    let deadline = Instant::now() + Duration::from_secs(2);
    while stall_msgs.lock_ok().is_empty() {
        assert!(
            Instant::now() < deadline,
            "wrong-id disarm must leave the cohort armed; \
             outsider enqueue should still abort it"
        );
        std::thread::yield_now();
    }

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();

    let msgs = stall_msgs.lock_ok();
    assert_eq!(
        msgs.len(),
        1,
        "wrong-id disarm must not clear the cohort: {msgs:?}"
    );
}
