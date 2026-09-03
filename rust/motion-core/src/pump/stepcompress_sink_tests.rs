use super::*;
use crate::mcu_config::{LaneKind, McuAxisConfig};
use std::collections::HashMap;
use std::sync::atomic::{AtomicI64, AtomicU64};
use trajectory::{ClockedMotorSpan, ContinuousAxis, MotorGroup, MotorSpan, MotorTerm};

const MCU_ID: u32 = 3;
const OID: u32 = 7;
const CYCLES_PER_SECOND: f64 = 1_000_000.0;
const BUDGET: u32 = 4;
const MICROSTEP: f64 = 0.01;

/// How far past a resuming move's start clock a mid-stream re-anchor may
/// land: the first step of a 100 mm/s run at [`MICROSTEP`] resolution.
const RESUME_ANCHOR_SLACK_CYCLES: u64 = 200;

/// The harness teleports its mcu clock by whole seconds to force a drain, so
/// every barrier looks arbitrarily old. Tests that mean to exercise the ack
/// deadline restore [`BARRIER_ACK_DEADLINE_SECONDS`] themselves.
const TELEPORTING_CLOCK_ACK_DEADLINE_SECONDS: f64 = 3_600.0;

struct Harness {
    endpoint: StepcompressEndpoint,
    now: Arc<AtomicU64>,
    sent: Arc<Mutex<Vec<StepFrame>>>,
    barriers: Arc<Mutex<Vec<(u32, u32)>>>,
    seeds: Arc<Mutex<Vec<(u32, i64)>>>,
    heartbeats: crossbeam_channel::Receiver<PumpMsg>,
    bursts: Arc<Mutex<Vec<usize>>>,
    attempts: Arc<Mutex<Vec<Vec<String>>>>,
    fail_sends: Arc<AtomicBool>,
    query_count: Arc<AtomicI64>,
    auto_query: Arc<AtomicBool>,
    query_calls: Arc<AtomicU64>,
    clock_calls: Arc<AtomicU64>,
}

fn motor_cfg_for(oid: u32) -> MotorConfig {
    MotorConfig {
        oid,
        microstep_distance: MICROSTEP,
        invert_dir: false,
        cycles_per_second: CYCLES_PER_SECOND,
        min_rearm_cycles: 0,
        encoder: StepEncoder::Classic {
            max_error_ticks: step_shim::compress::DEFAULT_MAX_ERROR_TICKS,
        },
    }
}

fn buzz_profile(
    freq_start_millihz: u32,
    freq_end_millihz: u32,
    amplitude_nm: u32,
    duration_ms: u32,
    ramp_ms: u32,
) -> Arc<BuzzProfile> {
    Arc::new(
        crate::pump::BuzzWave {
            freq_start_millihz,
            freq_end_millihz,
            amplitude_nm,
            duration_ms,
            ramp_ms,
        }
        .profile()
        .expect("the wave describes a buzz"),
    )
}

fn buzz_start(h: &Harness) -> u64 {
    h.now.load(Ordering::Relaxed) + (CYCLES_PER_SECOND * SEND_LEAD_SECONDS) as u64
}

fn harness(budget: u32) -> Harness {
    harness_on_axis(budget, 0)
}

fn harness_on_axis(budget: u32, axis: usize) -> Harness {
    harness_axes(budget, vec![axis], vec![OID])
}

fn harness_axes(budget: u32, axes: Vec<usize>, oids: Vec<u32>) -> Harness {
    assert_eq!(axes.len(), oids.len());
    let now = Arc::new(AtomicU64::new(0));
    let now_for_clock = Arc::clone(&now);
    let clock_calls = Arc::new(AtomicU64::new(0));
    let clock_calls_for_clock = Arc::clone(&clock_calls);
    let clock_of: ClockSource = Arc::new(move |_| {
        clock_calls_for_clock.fetch_add(1, Ordering::Relaxed);
        Some((now_for_clock.load(Ordering::Relaxed), CYCLES_PER_SECOND))
    });
    let sent = Arc::new(Mutex::new(Vec::new()));
    let fail_sends = Arc::new(AtomicBool::new(false));
    let query_count = Arc::new(AtomicI64::new(0));
    let auto_query = Arc::new(AtomicBool::new(true));
    let query_calls = Arc::new(AtomicU64::new(0));
    let sent_for_egress = Arc::clone(&sent);
    let barriers = Arc::new(Mutex::new(Vec::new()));
    let barriers_for_egress = Arc::clone(&barriers);
    let seeds = Arc::new(Mutex::new(Vec::new()));
    let seeds_for_egress = Arc::clone(&seeds);
    let fail_for_egress = Arc::clone(&fail_sends);
    let bursts = Arc::new(Mutex::new(Vec::new()));
    let bursts_for_egress = Arc::clone(&bursts);
    let attempts = Arc::new(Mutex::new(Vec::new()));
    let attempts_for_egress = Arc::clone(&attempts);
    let egress: FrameEgress =
        Arc::new(move |frames: &[(&'static str, Vec<(String, ArgValue)>)]| {
            attempts_for_egress.lock_ok().push(
                frames
                    .iter()
                    .map(|(name, args)| format!("{name}{args:?}"))
                    .collect(),
            );
            if fail_for_egress.load(Ordering::Relaxed) {
                return Err(SendError::Transient("egress down".into()));
            }
            bursts_for_egress.lock_ok().push(frames.len());
            for (name, args) in frames {
                let arg = |key: &str| -> i64 {
                    match args.iter().find(|(k, _)| k == key).map(|(_, v)| v) {
                        Some(ArgValue::Int(v)) => *v,
                        other => panic!("missing int arg {key}: {other:?}"),
                    }
                };
                if *name == "stepcompress_barrier" {
                    barriers_for_egress
                        .lock_ok()
                        .push((arg("oid") as u32, arg("seq") as u32));
                    continue;
                }
                if *name == "kalico_wire_probe" {
                    continue;
                }
                if *name == "stepcompress_set_position" {
                    seeds_for_egress
                        .lock_ok()
                        .push((arg("oid") as u32, arg("pos")));
                    continue;
                }
                let frame = match *name {
                    "queue_step" => StepFrame::QueueStep {
                        oid: arg("oid") as u32,
                        interval: arg("interval") as u32,
                        count: arg("count") as u16,
                        add: arg("add") as i16,
                    },
                    "set_next_step_dir" => StepFrame::SetNextStepDir {
                        oid: arg("oid") as u32,
                        dir: arg("dir") as u8,
                    },
                    "reset_step_clock" => StepFrame::ResetStepClock {
                        oid: arg("oid") as u32,
                        clock: arg("clock") as u32,
                    },
                    other => panic!("unexpected command {other}"),
                };
                sent_for_egress.lock_ok().push(frame);
            }
            Ok(())
        });
    let (tx, rx) = crossbeam_channel::unbounded();
    let query_for_endpoint = Arc::clone(&query_count);
    let calls_for_query = Arc::clone(&query_calls);
    let motors = oids.iter().copied().map(motor_cfg_for).collect();
    let lanes: Vec<StepLaneConfig> = axes
        .iter()
        .zip(&oids)
        .map(|(&axis, &oid)| StepLaneConfig { axis, oid })
        .collect();
    let endpoint = StepcompressEndpoint::new(
        MCU_ID,
        StepShim::new(motors, SHIM_RING_DEPTH),
        &lanes,
        egress,
        tx,
        clock_of,
        budget,
        Arc::new(move |_| {
            calls_for_query.fetch_add(1, Ordering::Relaxed);
            Ok(query_for_endpoint.load(Ordering::Relaxed))
        }),
        None,
        TELEPORTING_CLOCK_ACK_DEADLINE_SECONDS,
    )
    .expect("one motor per axis builds a stepcompress endpoint");
    Harness {
        endpoint,
        now,
        sent,
        barriers,
        seeds,
        heartbeats: rx,
        bursts,
        attempts,
        fail_sends,
        query_calls,
        query_count,
        auto_query,
        clock_calls,
    }
}

impl Harness {
    fn sent_moves(&self) -> usize {
        self.sent
            .lock_ok()
            .iter()
            .filter(|f| matches!(f, StepFrame::QueueStep { .. }))
            .count()
    }

    fn latest_heartbeat(&self) -> Option<HeartbeatMsg> {
        let mut last = None;
        while let Ok(PumpMsg::Heartbeat(heartbeat)) = self.heartbeats.try_recv() {
            last = Some(heartbeat);
        }
        last
    }

    fn latest_retired(&self) -> Option<Vec<u32>> {
        self.latest_heartbeat()
            .map(|heartbeat| heartbeat.retired_counts)
    }

    fn ack_sent_barriers_result(&mut self) -> Result<(), SendError> {
        let issued: Vec<(u32, u32)> = std::mem::take(&mut self.barriers.lock_ok());
        for (oid, seq) in issued {
            if self.auto_query.load(Ordering::Relaxed) {
                self.query_count
                    .store(self.endpoint.shim.commanded_steps(0), Ordering::Relaxed);
            }
            self.endpoint.on_barrier_ack(oid, seq)?;
        }
        Ok(())
    }

    fn ack_sent_barriers(&mut self) {
        self.ack_sent_barriers_result().unwrap();
    }
}

/// One clocked view per call, chained so the shim's seam check passes. A view
/// whose endpoints match is an explicit hold; anything else is a linear ramp
/// the shim samples onto the microstep grid.
fn span_on(
    start_clock: u64,
    from_mm: f64,
    to_mm: f64,
    secs: f64,
    freq: f64,
    motor_mask: u8,
) -> ClockedMotorSpan {
    let t_start = start_clock as f64 / freq;
    let t_end = t_start + secs;
    let is_hold = from_mm.to_bits() == to_mm.to_bits();
    let axis = if is_hold {
        ContinuousAxis::Hold {
            position: from_mm,
            t_start,
            t_end,
        }
    } else {
        ContinuousAxis::Spline(Arc::new(
            nurbs::ScalarNurbs::try_new(
                1,
                vec![t_start, t_start, t_end, t_end],
                vec![from_mm, to_mm],
            )
            .expect("a linear lane curve is valid"),
        ))
    };
    let signal = MotorSpan::try_new(
        Arc::from(vec![MotorGroup::Independent(MotorTerm {
            source_axis: 0,
            axis,
            scale: 1.0,
        })]),
        t_start,
        t_end,
        motor_mask,
        u32::MAX,
        is_hold,
    )
    .expect("a single-term motor span is dispatchable");
    ClockedMotorSpan::try_new(
        Arc::new(signal),
        t_start,
        t_end,
        t_start,
        t_end,
        start_clock as f64,
        freq,
    )
    .expect("the projected view spans at least one clock")
}

fn reversing_spline_span(start_clock: u64) -> ClockedMotorSpan {
    let t_start = start_clock as f64 / CYCLES_PER_SECOND;
    let t_end = t_start + 0.01;
    let curve = nurbs::ScalarNurbs::try_new(
        3,
        vec![
            t_start, t_start, t_start, t_start, t_end, t_end, t_end, t_end,
        ],
        vec![0.0, 0.04, 0.04, 0.0],
    )
    .unwrap();
    let signal = MotorSpan::try_new(
        Arc::from([MotorGroup::Independent(MotorTerm {
            source_axis: 0,
            axis: ContinuousAxis::Spline(Arc::new(curve)),
            scale: 1.0,
        })]),
        t_start,
        t_end,
        0,
        u32::MAX,
        false,
    )
    .unwrap();
    ClockedMotorSpan::try_new(
        Arc::new(signal),
        t_start,
        t_end,
        t_start,
        t_end,
        start_clock as f64,
        CYCLES_PER_SECOND,
    )
    .unwrap()
}

fn span(start_clock: u64, from_mm: f64, to_mm: f64, secs: f64) -> ClockedMotorSpan {
    span_on(start_clock, from_mm, to_mm, secs, CYCLES_PER_SECOND, 0)
}

fn hold_span(start_clock: u64, at_mm: f64, secs: f64, freq: f64) -> ClockedMotorSpan {
    span_on(start_clock, at_mm, at_mm, secs, freq, 0)
}

/// The same views re-signed with a motor mask: the mask selects which motors
/// of a grouped axis a view actually drives.
fn masked(spans: Vec<ClockedMotorSpan>, motor_mask: u8) -> Vec<ClockedMotorSpan> {
    spans
        .into_iter()
        .map(|view| {
            let signal = MotorSpan::try_new(
                Arc::clone(&view.signal.groups),
                view.signal.t_start,
                view.signal.t_end,
                motor_mask,
                view.signal.source_line,
                view.signal.is_explicit_hold,
            )
            .expect("re-signing a valid span keeps it dispatchable");
            ClockedMotorSpan::try_new(
                Arc::new(signal),
                view.stream_t_start,
                view.stream_t_end,
                view.start_host,
                view.end_host,
                view.start_clock_exact,
                view.clock_freq_hz,
            )
            .expect("re-signing a valid view keeps its clock range")
        })
        .collect()
}

fn span_end_mm(view: &ClockedMotorSpan) -> f64 {
    view.signal
        .position(view.stream_t_end)
        .expect("a staged view evaluates at its own end")
}

fn axis_frame(spans: Vec<ClockedMotorSpan>) -> AxisFrame {
    frame_for_axis(0, spans)
}

fn frame_for_axis(axis: u8, spans: Vec<ClockedMotorSpan>) -> AxisFrame {
    AxisFrame {
        axis,
        spans,
        new_head: 0,
        room: SHIM_RING_DEPTH,
        guard_recorded_ns: 0,
        guard_mcu_clock: 0,
    }
}

/// The positions a ramp of `count` views walks through, endpoints included.
fn ramp_positions(count: usize, start_mm: f64, direction: f64) -> Vec<f64> {
    let mut at = start_mm;
    let mut out = Vec::with_capacity(count + 1);
    out.push(at);
    for i in 0..count {
        at += direction * 0.05 * (1 + (i % 4)) as f64;
        out.push(at);
    }
    out
}

/// Views whose speed changes every view, so `compress` cannot merge them into
/// one long move and the endpoint really has a queue of moves to pace.
fn ramp(start_clock: u64, count: usize) -> Vec<ClockedMotorSpan> {
    ramp_from(start_clock, count, 0.0)
}

fn ramp_from(start_clock: u64, count: usize, start_mm: f64) -> Vec<ClockedMotorSpan> {
    epoch_ramp_from(start_clock, count, CYCLES_PER_SECOND, start_mm, 1.0)
}

/// [`ramp_from`] on an arbitrary epoch slope: a lane that resumed on its own
/// re-anchored epoch carries views clocked with that epoch's freq, not the
/// endpoint's live clock rate.
fn epoch_ramp(start_clock: u64, count: usize, freq: f64) -> Vec<ClockedMotorSpan> {
    epoch_ramp_from(start_clock, count, freq, 0.0, 1.0)
}

fn epoch_ramp_from(
    start_clock: u64,
    count: usize,
    freq: f64,
    start_mm: f64,
    direction: f64,
) -> Vec<ClockedMotorSpan> {
    let secs = 0.002;
    let stride = (secs * freq) as u64;
    let positions = ramp_positions(count, start_mm, direction);
    (0..count)
        .map(|i| {
            span_on(
                start_clock + stride * i as u64,
                positions[i],
                positions[i + 1],
                secs,
                freq,
                0,
            )
        })
        .collect()
}

/// A straight constant-speed run from `from_mm` to `to_mm` split into `count`
/// contiguous views. The shim samples positions onto the microstep grid, so a
/// run over an exact step multiple yields an exact step count.
fn linear_run(start_clock: u64, from_mm: f64, to_mm: f64, count: usize) -> Vec<ClockedMotorSpan> {
    let secs = 0.01;
    let stride = (secs * CYCLES_PER_SECOND) as u64;
    let step_mm = (to_mm - from_mm) / count as f64;
    (0..count)
        .map(|i| {
            let from = from_mm + step_mm * i as f64;
            span(start_clock + stride * i as u64, from, from + step_mm, secs)
        })
        .collect()
}

/// A ramp whose views are long enough that a few in-flight moves buffer more
/// mcu time than the budget tests advance the clock per tick. `ramp`'s 2 ms
/// views cannot: four slots hold 8 ms of motion, so a 10 ms tick leaves the
/// backlog head behind the mcu clock — a pipe no host could deliver through.
fn paceable_ramp(start_clock: u64, count: usize) -> Vec<ClockedMotorSpan> {
    let secs = 0.02;
    let stride = (secs * CYCLES_PER_SECOND) as u64;
    let positions = ramp_positions(count, 0.0, 1.0);
    (0..count)
        .map(|i| {
            span(
                start_clock + stride * i as u64,
                positions[i],
                positions[i + 1],
                secs,
            )
        })
        .collect()
}

#[test]
fn grouped_axis_fans_out_to_every_motor_and_publishes_one_axis_credit() {
    let mut h = harness_axes(16, vec![0, 0], vec![7, 8]);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(linear_run(2_000, 0.0, 1.0, 2))])
        .unwrap();

    let mut direction_oids: Vec<u32> = h
        .sent
        .lock_ok()
        .iter()
        .filter_map(|frame| match frame {
            StepFrame::SetNextStepDir { oid, .. } => Some(*oid),
            _ => None,
        })
        .collect();
    direction_oids.sort_unstable();
    direction_oids.dedup();
    assert_eq!(direction_oids, vec![7, 8]);
    let steps_by_oid = h
        .sent
        .lock_ok()
        .iter()
        .filter_map(|frame| match frame {
            StepFrame::QueueStep { oid, count, .. } => Some((*oid, u32::from(*count))),
            _ => None,
        })
        .fold(
            std::collections::HashMap::new(),
            |mut totals, (oid, count)| {
                *totals.entry(oid).or_default() += count;
                totals
            },
        );
    assert_eq!(
        steps_by_oid,
        std::collections::HashMap::from([(7, 100), (8, 100)])
    );

    let heartbeat = h
        .latest_heartbeat()
        .expect("frame send publishes a heartbeat");
    assert_eq!(heartbeat.axes, vec![0]);
    assert_eq!(heartbeat.consumed_counts, Some(vec![2]));
    assert_eq!(heartbeat.retired_counts, vec![0]);
}

#[test]
fn one_view_preserves_crossings_after_an_internal_direction_reversal() {
    let mut h = harness(BUDGET);
    h.now.store(1_000, Ordering::Relaxed);

    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(vec![reversing_spline_span(2_000)])])
        .unwrap();

    let steps: u32 = h
        .sent
        .lock_ok()
        .iter()
        .map(|frame| match frame {
            StepFrame::QueueStep { count, .. } => u32::from(*count),
            _ => 0,
        })
        .sum();
    assert_eq!(steps, 6);
    assert_eq!(h.endpoint.shim.commanded_steps(0), 0);
}

#[test]
fn a_reseeded_grouped_axis_still_steps_every_motor() {
    let mut h = harness_axes(16, vec![0, 0], vec![7, 8]);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint.reset_axis_position(0, 0).unwrap();
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(linear_run(2_000, 0.0, 1.0, 2))])
        .unwrap();
    let steps_by_oid = h
        .sent
        .lock_ok()
        .iter()
        .filter_map(|frame| match frame {
            StepFrame::QueueStep { oid, count, .. } => Some((*oid, u32::from(*count))),
            _ => None,
        })
        .fold(
            std::collections::HashMap::new(),
            |mut totals, (oid, count)| {
                *totals.entry(oid).or_default() += count;
                totals
            },
        );
    assert_eq!(
        steps_by_oid,
        std::collections::HashMap::from([(7, 100), (8, 100)])
    );
}

#[test]
fn a_transport_reseed_reaches_every_motor_of_a_grouped_axis() {
    let mut h = harness_axes(16, vec![0, 0], vec![7, 8]);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint.reset_axis_position(0, 40).unwrap();

    let mut seeds = h.seeds.lock_ok().clone();
    seeds.sort_unstable();
    assert_eq!(
        seeds,
        vec![(7, 40), (8, 40)],
        "an AWD twin left unseeded generates no steps after a phase-mode exit"
    );
}
#[test]
fn selected_motor_frame_advances_grouped_axis_credit() {
    let mut h = harness_axes(16, vec![0, 0], vec![7, 8]);
    h.now.store(1_000, Ordering::Relaxed);
    let spans = masked(linear_run(2_000, 0.0, 0.2, 2), 0b0000_0001);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(spans)])
        .unwrap();

    let heartbeat = h
        .latest_heartbeat()
        .expect("frame send publishes a heartbeat");
    assert_eq!(heartbeat.axes, vec![0]);
    assert_eq!(heartbeat.consumed_counts, Some(vec![2]));
}

#[test]
fn sending_stops_once_the_in_flight_budget_is_reached() {
    let mut h = harness(BUDGET);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 40))])
        .unwrap();

    assert_eq!(h.sent_moves(), BUDGET as usize);
    assert!(
        !h.endpoint.backlog.is_empty(),
        "the rest of the drained frames must still be waiting"
    );
}

#[test]
fn in_flight_drains_as_the_clock_advances_and_sending_resumes() {
    let mut h = harness(BUDGET);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(paceable_ramp(2_000, 12))])
        .unwrap();
    let first = h.sent_moves();
    assert_eq!(first, BUDGET as usize);

    let mut previous = first;
    for step in 1..=20u64 {
        h.now.store(1_000 + step * 10_000, Ordering::Relaxed);
        h.endpoint.tick().unwrap();
        let now_sent = h.sent_moves();
        assert!(now_sent >= previous, "sent count must never go backwards");
        previous = now_sent;
    }
    assert!(
        previous > first,
        "advancing the mcu clock must free budget and release more moves ({previous} vs {first})"
    );
    assert!(h.endpoint.in_flight.len() as u32 <= BUDGET);
}

#[test]
fn move_slots_reclaim_when_the_mcu_loads_the_run() {
    let mut h = harness(1);
    h.endpoint.queue_outbound(
        0,
        Outbound::Step(StepFrame::QueueStep {
            oid: 6,
            interval: 10,
            count: 1,
            add: 0,
        }),
        1_000,
        1_010,
        0,
    );
    h.endpoint.queue_outbound(
        0,
        Outbound::Step(StepFrame::QueueStep {
            oid: 7,
            interval: 10,
            count: 1,
            add: 0,
        }),
        20_000,
        20_010,
        0,
    );

    h.endpoint
        .flush(McuClock {
            now: 0,
            freq: CYCLES_PER_SECOND,
        })
        .unwrap();
    assert_eq!(h.sent_moves(), 1);

    let margin = (CYCLES_PER_SECOND * CONSUMED_MARGIN_SECONDS) as u64;
    h.now.store(1_000 + margin, Ordering::Relaxed);
    h.endpoint
        .flush(McuClock {
            now: 1_000 + margin,
            freq: CYCLES_PER_SECOND,
        })
        .unwrap();
    assert_eq!(h.sent_moves(), 2);
}

#[test]
fn barriers_hold_move_slots_until_the_mcu_loads_them() {
    let mut h = harness(1);
    h.endpoint.queue_outbound(
        0,
        Outbound::Barrier(BarrierId { oid: OID, seq: 1 }),
        1_000,
        1_000,
        0,
    );
    h.endpoint.queue_outbound(
        0,
        Outbound::Step(StepFrame::QueueStep {
            oid: OID,
            interval: 10,
            count: 1,
            add: 0,
        }),
        20_000,
        20_010,
        0,
    );

    h.endpoint
        .flush(McuClock {
            now: 0,
            freq: CYCLES_PER_SECOND,
        })
        .unwrap();
    assert_eq!(h.barriers.lock_ok().as_slice(), &[(OID, 1)]);
    assert_eq!(h.sent_moves(), 0);

    let margin = (CYCLES_PER_SECOND * CONSUMED_MARGIN_SECONDS) as u64;
    h.now.store(1_000 + margin, Ordering::Relaxed);
    h.endpoint
        .flush(McuClock {
            now: 1_000 + margin,
            freq: CYCLES_PER_SECOND,
        })
        .unwrap();
    assert_eq!(h.sent_moves(), 1);
}

#[test]
fn retirement_only_counts_fully_sent_views_and_never_regresses() {
    let mut h = harness(BUDGET);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(paceable_ramp(2_000, 12))])
        .unwrap();

    let shim_consumed = h.endpoint.shim.consumed_counts();
    let published = h.latest_retired().expect("a heartbeat is always posted");
    assert!(
        published[0] < shim_consumed[0],
        "retirement must lag the shim while frames are still unsent ({published:?} vs \
         {shim_consumed:?})"
    );

    let mut last = published[0];
    for step in 1..=240u64 {
        h.now.store(1_000 + step * 10_000, Ordering::Relaxed);
        h.endpoint.tick().unwrap();
        h.ack_sent_barriers();
        if let Some(counts) = h.latest_retired() {
            assert!(
                counts[0] >= last,
                "retirement regressed {last} -> {}",
                counts[0]
            );
            last = counts[0];
        }
    }
    assert!(h.endpoint.backlog.is_empty(), "backlog must drain");
    assert_eq!(
        last,
        h.endpoint.shim.consumed_counts()[0],
        "once everything is sent, retirement must catch up to the shim"
    );
}

#[test]
fn backlog_ceiling_breach_is_fatal() {
    let mut h = harness(1);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .backlog
        .extend((0..BACKLOG_CEILING_FRAMES).map(|_| OutboundFrame {
            lane: 0,
            frame: Outbound::Step(StepFrame::QueueStep {
                oid: OID,
                interval: 10,
                count: 1,
                add: 0,
            }),
            start_clock: u64::MAX,
            end_clock: u64::MAX,
            enqueue_order: 0,
            queued_clock: 0,
        }));
    let err = h
        .endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 8))])
        .unwrap_err();
    match err {
        SendError::Fatal(msg) => {
            assert!(msg.contains("outbound step frames"), "{msg}");
            assert!(msg.contains(&BACKLOG_CEILING_FRAMES.to_string()), "{msg}");
        }
        other => panic!("expected Fatal, got {other:?}"),
    }
}

fn stale_queue_step(start_clock: u64) -> OutboundFrame {
    OutboundFrame {
        lane: 0,
        frame: Outbound::Step(StepFrame::QueueStep {
            oid: OID,
            interval: 10,
            count: 1,
            add: 0,
        }),
        start_clock,
        end_clock: start_clock + 10,
        enqueue_order: 0,
        queued_clock: start_clock,
    }
}

#[test]
fn a_queue_step_behind_the_mcu_clock_is_fatal_before_the_mcu_loads_it() {
    let mut h = harness(BUDGET);
    h.endpoint.backlog.push_back(stale_queue_step(1_000));
    h.now.store(1_000_000, Ordering::Relaxed);
    let err = h.endpoint.tick().unwrap_err();
    match err {
        SendError::Fatal(msg) => {
            assert!(msg.contains("Stepper too far in past"), "{msg}");
            assert!(msg.contains("999000 us behind"), "{msg}");
        }
        other => panic!("expected Fatal, got {other:?}"),
    }
    assert_eq!(
        h.sent_moves(),
        0,
        "a frame the mcu would fault on must never reach the wire"
    );
}

#[test]
fn a_queue_step_within_the_projection_guard_still_goes_out() {
    let mut h = harness(BUDGET);
    let guard_cycles = (CYCLES_PER_SECOND * 500e-6) as u64;
    h.now.store(1_000_000, Ordering::Relaxed);
    h.endpoint
        .backlog
        .push_back(stale_queue_step(1_000_000 - guard_cycles / 2));
    h.endpoint.tick().unwrap();
    assert_eq!(h.sent_moves(), 1);
}

fn stale_reset_step_clock(start_clock: u64) -> OutboundFrame {
    OutboundFrame {
        lane: 0,
        frame: Outbound::Step(StepFrame::ResetStepClock {
            oid: OID,
            clock: start_clock as u32,
        }),
        start_clock,
        end_clock: start_clock,
        enqueue_order: 0,
        queued_clock: start_clock,
    }
}

fn stale_set_next_step_dir(start_clock: u64) -> OutboundFrame {
    OutboundFrame {
        lane: 0,
        frame: Outbound::Step(StepFrame::SetNextStepDir { oid: OID, dir: 1 }),
        start_clock,
        end_clock: start_clock,
        enqueue_order: 0,
        queued_clock: start_clock,
    }
}

#[test]
fn a_reset_step_clock_behind_the_mcu_clock_is_fatal_before_egress() {
    let mut h = harness(BUDGET);
    h.endpoint.backlog.push_back(stale_reset_step_clock(1_000));
    h.now.store(1_000_000, Ordering::Relaxed);
    let err = h.endpoint.tick().unwrap_err();
    match err {
        SendError::Fatal(msg) => {
            assert!(msg.contains("reset_step_clock"), "{msg}");
            assert!(msg.contains("999000 us behind"), "{msg}");
            assert!(msg.contains("deficit of 999000 us"), "{msg}");
            assert!(msg.contains("projected mcu clock 1000000"), "{msg}");
        }
        other => panic!("expected Fatal, got {other:?}"),
    }
    assert!(
        h.sent.lock_ok().is_empty(),
        "a reset_step_clock the mcu would load in the past must never reach the wire — \
         it starts the step catch-up that starves the scheduler into \
         \"Rescheduled timer in the past\""
    );
}

#[test]
fn a_set_next_step_dir_behind_the_mcu_clock_is_fatal_before_egress() {
    let mut h = harness(BUDGET);
    h.endpoint.backlog.push_back(stale_set_next_step_dir(1_000));
    h.now.store(1_000_000, Ordering::Relaxed);
    let err = h.endpoint.tick().unwrap_err();
    match err {
        SendError::Fatal(msg) => {
            assert!(msg.contains("set_next_step_dir"), "{msg}");
            assert!(msg.contains("999000 us behind"), "{msg}");
        }
        other => panic!("expected Fatal, got {other:?}"),
    }
    assert!(h.sent.lock_ok().is_empty());
}

#[test]
fn a_reset_step_clock_within_the_projection_guard_still_goes_out() {
    let mut h = harness(BUDGET);
    let guard_cycles = (CYCLES_PER_SECOND * 500e-6) as u64;
    h.now.store(1_000_000, Ordering::Relaxed);
    h.endpoint
        .backlog
        .push_back(stale_reset_step_clock(1_000_000 - guard_cycles / 2));
    h.endpoint.tick().unwrap();
    assert!(
        h.sent
            .lock_ok()
            .iter()
            .any(|f| matches!(f, StepFrame::ResetStepClock { .. })),
        "a reset clock inside the projection floor margin is a normal re-anchor"
    );
}

#[test]
fn multi_lane_commands_leave_in_step_deadline_order() {
    let mut h = harness(8);
    for (oid, start_clock) in [(6, 100), (6, 400), (7, 200), (8, 300)] {
        h.endpoint.queue_outbound(
            0,
            Outbound::Step(StepFrame::QueueStep {
                oid,
                interval: 10,
                count: 1,
                add: 0,
            }),
            start_clock,
            start_clock + 10,
            0,
        );
    }

    h.endpoint
        .flush(McuClock {
            now: 0,
            freq: CYCLES_PER_SECOND,
        })
        .unwrap();

    let sent_oids: Vec<u32> = h
        .sent
        .lock_ok()
        .iter()
        .filter_map(|frame| match frame {
            StepFrame::QueueStep { oid, .. } => Some(*oid),
            _ => None,
        })
        .collect();
    assert_eq!(sent_oids, [6, 7, 8, 6]);
}

#[test]
fn the_send_lead_outlasts_the_link_retransmit_floor() {
    let min_rto = host_rt::host_io::rtt::MIN_RTO.as_secs_f64();
    assert!(
        SEND_LEAD_SECONDS >= 2.0 * min_rto,
        "a move queue {SEND_LEAD_SECONDS} s deep empties during the {min_rto} s the host stays \
         silent after a dropped ack, and the mcu shuts down with \"Timer too close\""
    );
}

#[test]
fn the_host_guard_rejects_a_stale_queue_step_before_the_send_lead_is_gone() {
    let host_guard = crate::pump::pump_loop::pump_past_guard_secs();
    assert!(
        host_guard < SEND_LEAD_SECONDS,
        "the pump lets a queue_step through until it is {host_guard} s behind the mcu clock — \
         the mcu shuts down on any late idle-stepper re-arm, so the host guard exists only to \
         name a send-time stall with its backlog and slot occupancy, and it must trip well \
         inside the {SEND_LEAD_SECONDS} s delivery lead"
    );
}

#[test]
fn a_dead_egress_retains_frames_without_failing_the_bundle() {
    let mut h = harness(BUDGET);
    h.now.store(1_000, Ordering::Relaxed);
    h.fail_sends.store(true, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 4))])
        .expect("the spans were consumed into the shim - failing the bundle would replay them");
    assert_eq!(h.sent_moves(), 0);
    assert!(!h.endpoint.backlog.is_empty());
}

#[test]
fn a_backpressured_bundle_is_not_replayed_into_a_span_gap() {
    let mut h = harness(BUDGET);
    h.now.store(1_000, Ordering::Relaxed);
    h.fail_sends.store(true, Ordering::Relaxed);
    let first = ramp(2_000, 4);
    let resume_clock = first.last().expect("ramp emits views").end_clock;
    let resume_mm = *ramp_positions(4, 0.0, 1.0).last().expect("positions");
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(first)])
        .expect("backpressure after consumption is absorbed");

    h.fail_sends.store(false, Ordering::Relaxed);
    let second = ramp_from(resume_clock, 4, resume_mm);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(second)])
        .expect("the contiguous follow-up must not trip a span gap");
    assert!(h.sent_moves() > 0);
}

#[test]
fn a_refused_burst_is_retried_verbatim_with_nothing_duplicated() {
    let mut h = harness(BUDGET);
    h.now.store(1_000, Ordering::Relaxed);
    h.fail_sends.store(true, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 8))])
        .expect("a refused burst is retained, not failed");
    let refused = h.attempts.lock_ok().clone();
    assert_eq!(refused.len(), 1, "{refused:?}");
    assert!(refused[0].len() > 1, "{refused:?}");
    assert_eq!(h.sent_moves(), 0);

    h.fail_sends.store(false, Ordering::Relaxed);
    h.endpoint.tick().expect("the retry goes through");

    let attempts = h.attempts.lock_ok().clone();
    assert_eq!(attempts.len(), 2, "{attempts:?}");
    assert_eq!(
        attempts[1][..refused[0].len()],
        refused[0][..],
        "the retry must re-offer the refused frames in the same order"
    );

    let mut delivered = attempts[1].clone();
    delivered.sort();
    let unique = delivered.len();
    delivered.dedup();
    assert_eq!(
        delivered.len(),
        unique,
        "a refused burst must not be dispatched twice"
    );
}

#[test]
fn abort_outbound_discards_unsent_frames() {
    let mut h = harness(BUDGET);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 40))])
        .unwrap();
    assert!(!h.endpoint.backlog.is_empty());
    h.endpoint.abort_outbound();
    assert!(h.endpoint.backlog.is_empty());
    assert!(h.endpoint.in_flight.is_empty());
}

#[test]
fn an_unmarked_overlap_is_a_loud_span_gap() {
    let mut h = harness(1024);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 40))])
        .unwrap();

    let gap = h
        .endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp_from(81_834, 8, 5.0))])
        .expect_err("an unmarked overlap is a loud SpanGap");
    assert!(format!("{gap:?}").contains("SpanGap"), "{gap:?}");
}

#[test]
fn a_marked_fresh_epoch_may_start_before_the_queued_stream_ends() {
    let mut h = harness(1024);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 40))])
        .unwrap();

    h.endpoint.mark_reanchor(0, 81_834, Some(CYCLES_PER_SECOND));
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp_from(81_834, 8, 5.0))])
        .expect("a marked fresh epoch may start at any clock");
    h.ack_sent_barriers();
    assert!(
        h.sent
            .lock_ok()
            .iter()
            .any(|f| matches!(f, StepFrame::ResetStepClock { .. })),
        "the new epoch must re-anchor the mcu step clock"
    );
}

#[test]
fn a_sent_cut_reseeds_from_the_mcu_executed_count() {
    let mut h = harness(1024);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 40))])
        .unwrap();
    h.endpoint.mark_reanchor(0, 81_834, Some(CYCLES_PER_SECOND));
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp_from(81_834, 8, 5.0))])
        .unwrap();

    let expected = h.endpoint.lanes[0]
        .pending_cut
        .as_ref()
        .expect("the cut is awaiting reconciliation")
        .expected_count;
    h.auto_query.store(false, Ordering::Relaxed);
    h.query_count.store(expected, Ordering::Relaxed);
    h.ack_sent_barriers_result()
        .expect("the injected MCU count matches the host expectation");

    assert_eq!(h.query_calls.load(Ordering::Relaxed), 1);
    assert!(h.endpoint.lanes[0].pending_cut.is_none());
    assert!(
        h.sent
            .lock_ok()
            .iter()
            .any(|frame| { matches!(frame, StepFrame::ResetStepClock { .. }) })
    );
}

#[test]
fn a_sent_cut_count_mismatch_is_fatal() {
    let mut h = harness(1024);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 40))])
        .unwrap();
    h.endpoint.mark_reanchor(0, 81_834, Some(CYCLES_PER_SECOND));
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp_from(81_834, 8, 5.0))])
        .unwrap();

    let expected = h.endpoint.lanes[0]
        .pending_cut
        .as_ref()
        .expect("the cut is awaiting reconciliation")
        .expected_count;
    h.auto_query.store(false, Ordering::Relaxed);
    h.query_count.store(expected + 1, Ordering::Relaxed);
    let err = h
        .ack_sent_barriers_result()
        .expect_err("a lost step must abort the cut");
    let message = format!("{err:?}");
    assert!(message.contains("host expected"), "{message}");
    assert!(message.contains("MCU reported"), "{message}");
    assert!(message.contains("delta"), "{message}");
}

/// A second idle-resume may be marked while a sent-frame cut still awaits its
/// barrier ack (the QGL probe cadence on a real transport): its frames are
/// swallowed into the pending cut's `held` run together with the first
/// resume's views, with an idle hole between them. Completing the cut must
/// replay `held` through the pending-seam ladder — validating it as one
/// contiguous fresh stream is the bench `SpanGap` fatal.
#[test]
fn a_resume_marked_while_a_cut_awaits_its_ack_replays_through_its_own_seam() {
    let mut h = harness(1024);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 40))])
        .unwrap();

    h.endpoint.mark_reanchor(0, 81_834, Some(CYCLES_PER_SECOND));
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp_from(81_834, 8, 5.0))])
        .unwrap();
    assert!(h.endpoint.lanes[0].pending_cut.is_some());

    h.endpoint
        .mark_reanchor(0, 500_000, Some(CYCLES_PER_SECOND));
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp_from(500_000, 8, 6.0))])
        .unwrap();
    assert_eq!(
        h.endpoint.lanes[0]
            .pending_cut
            .as_ref()
            .expect("the cut is awaiting reconciliation")
            .held
            .len(),
        16,
        "both resumes' views ride the pending cut"
    );

    let expected = h.endpoint.lanes[0]
        .pending_cut
        .as_ref()
        .expect("the cut is awaiting reconciliation")
        .expected_count;
    h.auto_query.store(false, Ordering::Relaxed);
    h.query_count.store(expected, Ordering::Relaxed);
    h.ack_sent_barriers_result()
        .expect("held frames replay through their own pending seam, not as one stream");

    assert!(
        h.endpoint.lanes.iter().all(|l| l.pending_cut.is_none()),
        "the second resume's cut was unsent and resolves host-exact"
    );
    assert!(
        h.endpoint.lanes.iter().all(|l| l.seams.is_empty()),
        "the second resume's seam mark was consumed by the held replay"
    );
}

/// Trip halt → external count reseed → retract → sent-frame cut →
/// re-approach. The homing reconcile adopts the mcu's executed count as a
/// fresh absolute origin; a later cut must re-anchor from that origin (not
/// the pre-trip stream total), or the re-approach lands the axis offset by
/// the reseed delta. Reproduces the trip/retract/re-approach sequence the
/// sim homing e2e tests run.
#[test]
fn trip_halt_reseed_retract_then_cut_re_approach_preserve_net_position() {
    let mut h = harness(1024);
    h.now.store(1_000, Ordering::Relaxed);

    // Approach 0 → 0.5 mm (50 steps); fully sent.
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 8))])
        .unwrap();

    // Trip: the endstop reconcile reads 20 executed steps and reseeds the
    // host stream to that count; the mcu counter adopts the same origin.
    h.endpoint.reset_motor_position(0, 20).unwrap();
    assert_eq!(
        h.endpoint.shim.commanded_steps(0),
        20,
        "the reseed must move the stream origin to the mcu readback"
    );

    // Retract 0.2 → 0.5 mm (+30 steps); sent before the cut is marked.
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(linear_run(50_000, 0.2, 0.5, 5))])
        .unwrap();

    // The re-approach starts a fresh epoch on the exact clock the sent
    // stream ended: the seam falls inside frames the mcu already received,
    // so the cut must reconcile through the mcu executed count.
    let seam = h.endpoint.lanes[0]
        .step_clock
        .expect("the sent volley anchored the lane's step clock");
    h.endpoint.mark_reanchor(0, seam, Some(CYCLES_PER_SECOND));
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(linear_run(100_000, 0.5, 0.0, 5))])
        .unwrap();

    let expected = h.endpoint.lanes[0]
        .pending_cut
        .as_ref()
        .expect("the cut is awaiting reconciliation")
        .expected_count;
    assert_eq!(
        expected, 50,
        "the host expectation is the reseeded origin plus the retract motion"
    );
    h.auto_query.store(false, Ordering::Relaxed);
    h.query_count.store(expected, Ordering::Relaxed);
    h.ack_sent_barriers_result()
        .expect("the mcu executed count matches the host bookkeeping");

    assert!(
        h.endpoint.lanes.iter().all(|l| l.pending_cut.is_none()),
        "the cut must complete once the barrier acks"
    );
    assert_eq!(
        h.endpoint.shim.halt_at(0, u64::MAX).unwrap().0,
        0,
        "trip + retract + re-approach must land on the reseeded origin, not \
         the pre-trip stream total"
    );
}

#[test]
fn an_unsent_only_cut_does_not_query_the_mcu() {
    let mut h = harness(4);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 8))])
        .unwrap();
    h.endpoint
        .mark_reanchor(0, 500_000, Some(CYCLES_PER_SECOND));
    h.auto_query.store(false, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp_from(500_000, 4, 1.0))])
        .expect("an unsent-only cut remains host-exact");

    assert!(h.endpoint.lanes.iter().all(|l| l.pending_cut.is_none()));
    assert_eq!(h.query_calls.load(Ordering::Relaxed), 0);
}

#[test]
fn a_bundle_spanning_the_epoch_boundary_is_cut_at_the_marked_view() {
    let mut h = harness(1024);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 8))])
        .unwrap();

    let mut spanning = ramp_from(18_000, 4, 1.0);
    spanning.extend(ramp_from(500_000, 4, 1.5));
    h.endpoint
        .mark_reanchor(0, 500_000, Some(CYCLES_PER_SECOND));
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(spanning)])
        .expect("the cut must land between the old tail and the new head");
}

#[test]
fn two_marked_gaps_in_one_buffered_stretch_both_apply_in_order() {
    // Two dwells inside one buffered stream (G4 G4, or stepper-enable
    // stalls) mark two seam gaps before any of their views reach the
    // endpoint. Both must survive queued — a single-slot mark would drop
    // the first seam and die on its SpanGap.
    let mut h = harness(1024);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 8))])
        .unwrap();

    h.endpoint.mark_seam_gap(0, 100_000);
    h.endpoint.mark_seam_gap(0, 700_000);

    let mut spanning = ramp_from(100_000, 4, 1.0);
    spanning.extend(ramp_from(700_000, 4, 1.5));
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(spanning)])
        .expect("both marked seam gaps must be sanctioned, in order");
    assert!(
        h.endpoint.lanes[0].seams.is_empty(),
        "both gaps must be consumed"
    );
}

#[test]
fn a_seam_gap_emits_no_mcu_frames() {
    // A rejoin hole is stationary: sanctioning it must not halt the shim,
    // reset the mcu step clock, or emit any frame — a mid-stream cut wedges
    // a live classic mcu.
    let mut h = harness(1024);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 8))])
        .unwrap();
    let resets_before = h
        .sent
        .lock_ok()
        .iter()
        .filter(|f| matches!(f, StepFrame::ResetStepClock { .. }))
        .count();

    h.endpoint.mark_seam_gap(0, 300_000);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp_from(300_000, 4, 1.0))])
        .expect("a marked forward gap is sanctioned");
    let resets_after = h
        .sent
        .lock_ok()
        .iter()
        .filter(|f| matches!(f, StepFrame::ResetStepClock { .. }))
        .count();
    assert_eq!(
        resets_before, resets_after,
        "a seam gap must not re-anchor the mcu step clock"
    );
}

#[test]
fn a_seam_gap_cannot_sanction_a_backward_overlap() {
    let mut h = harness(1024);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 40))])
        .unwrap();

    // The stream above runs well past 40_000; a "gap" pointing back into it
    // is an overlap and must stay loud.
    h.endpoint.mark_seam_gap(0, 40_000);
    let err = h
        .endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp_from(40_000, 4, 5.0))])
        .expect_err("a backward jump is an overlap, not a gap");
    assert!(format!("{err:?}").contains("SpanGap"), "{err:?}");
}

#[test]
fn a_cut_keeps_frames_that_were_already_emitted() {
    let mut h = harness(BUDGET);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 40))])
        .unwrap();
    let backlogged = h.endpoint.backlog.len();
    assert!(backlogged > 0);

    h.endpoint
        .mark_reanchor(0, 500_000, Some(CYCLES_PER_SECOND));
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp_from(500_000, 4, 5.0))])
        .unwrap();
    assert!(
        h.endpoint.backlog.len() >= backlogged,
        "already-emitted frames describe real steps and must still be delivered"
    );
}

#[test]
fn a_mark_that_never_matches_leaves_the_stream_alone() {
    let mut h = harness(BUDGET);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .mark_reanchor(0, 999_999_999, Some(CYCLES_PER_SECOND));
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 8))])
        .unwrap();
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp_from(18_000, 8, 1.0))])
        .expect("contiguous views still flow with an unmatched mark outstanding");
}
#[test]
fn reset_position_drops_the_stale_stream_and_re_emits_a_step_clock_reset() {
    let mut h = harness(BUDGET);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 40))])
        .unwrap();
    assert!(!h.endpoint.backlog.is_empty());

    h.endpoint.reset_position(&[5]).unwrap();
    assert!(h.endpoint.backlog.is_empty());
    assert!(h.endpoint.in_flight.is_empty());
    assert_eq!(
        h.seeds.lock_ok().as_slice(),
        &[(OID, 5)],
        "the mcu step counter must adopt the same origin as the shim"
    );
    assert_eq!(
        h.latest_retired(),
        Some(vec![40]),
        "aborted views must retire immediately so a position reseed can drain"
    );

    h.sent.lock_ok().clear();
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(82_000, 8))])
        .unwrap();
    assert!(
        h.sent
            .lock_ok()
            .iter()
            .any(|f| matches!(f, StepFrame::ResetStepClock { .. })),
        "a reseeded motor must re-anchor its step clock before the next move"
    );
}
#[test]
fn abort_axes_retires_flushed_views_immediately() {
    let mut h = harness(BUDGET);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 40))])
        .unwrap();

    h.endpoint.abort_axes(&[0]).unwrap();

    assert_eq!(h.latest_retired(), Some(vec![40]));
    h.sent.lock_ok().clear();
    h.now.store(3_000_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp_from(3_000_000, 8, 5.0))])
        .unwrap();
    assert!(
        h.sent
            .lock_ok()
            .iter()
            .any(|frame| matches!(frame, StepFrame::ResetStepClock { .. }))
    );
}

#[test]
fn a_position_seed_of_the_wrong_width_is_fatal() {
    let mut h = harness(BUDGET);
    let err = h.endpoint.reset_position(&[1, 2]).unwrap_err();
    match err {
        SendError::Fatal(msg) => assert!(msg.contains("configured axes"), "{msg}"),
        other => panic!("expected Fatal, got {other:?}"),
    }
}

fn stepcompress_cfg(move_queue_slots: u32) -> McuAxisConfig {
    McuAxisConfig {
        mcu_id: MCU_ID,
        axes: vec![0],
        kinematics: 0,
        max_motor_velocity: vec![100.0],
        ethercat: false,
        lane_kinds: vec![LaneKind::Pulse],
        motor_counts: vec![1],
        microstep_distance: vec![MICROSTEP],
        invert_dir: vec![false],
        stepper_oids: vec![OID],
        move_queue_slots,
        step_pulse_seconds: vec![2e-6],
        stepcompress_encoders: vec![StepcompressEncoder::Classic],
        phase_sample_rate: 0.0,
        phase_ring_depth: 0,
        stepcompress_max_error_secs: 25e-6,
    }
}

#[test]
fn a_move_queue_too_small_for_the_reserve_is_a_build_error() {
    let (tx, _rx) = crossbeam_channel::unbounded();
    let clock_of: ClockSource = Arc::new(|_| Some((0, CYCLES_PER_SECOND)));
    let err = match build_endpoint(
        &stepcompress_cfg(MOVE_SLOT_RESERVE),
        Weak::new(),
        tx,
        CYCLES_PER_SECOND,
        clock_of,
    ) {
        Err(e) => e,
        Ok(_) => panic!("a move queue equal to the reserve must not build an endpoint"),
    };
    assert!(err.contains("move-queue slots"), "{err}");
}

#[test]
fn classic_encoder_resolves_max_error_ticks_from_the_measured_clock() {
    let (tx, _rx) = crossbeam_channel::unbounded();
    let clock_of: ClockSource = Arc::new(|_| Some((0, CYCLES_PER_SECOND)));
    let mut cfg = stepcompress_cfg(128);
    cfg.stepcompress_encoders = vec![StepcompressEncoder::Classic];
    cfg.stepcompress_max_error_secs = 10e-6;
    build_endpoint(&cfg, Weak::new(), tx, CYCLES_PER_SECOND, clock_of)
        .expect("10us max_error at 1 MHz resolves to 10 ticks and must build");
}

#[test]
fn classic_encoder_with_a_sub_tick_max_error_is_a_build_error() {
    let (tx, _rx) = crossbeam_channel::unbounded();
    let clock_of: ClockSource = Arc::new(|_| Some((0, CYCLES_PER_SECOND)));
    let mut cfg = stepcompress_cfg(128);
    cfg.stepcompress_encoders = vec![StepcompressEncoder::Classic];
    cfg.stepcompress_max_error_secs = 1e-7;
    let err = match build_endpoint(&cfg, Weak::new(), tx, CYCLES_PER_SECOND, clock_of) {
        Err(e) => e,
        Ok(_) => panic!("a max_error below one tick must not build an endpoint"),
    };
    assert!(err.contains("tick budget"), "{err}");
}

#[test]
fn classic_encoder_with_an_overflowing_tick_budget_is_a_build_error() {
    let (tx, _rx) = crossbeam_channel::unbounded();
    let clock_of: ClockSource = Arc::new(|_| Some((0, CYCLES_PER_SECOND)));
    let mut cfg = stepcompress_cfg(128);
    cfg.stepcompress_encoders = vec![StepcompressEncoder::Classic];
    cfg.stepcompress_max_error_secs = 1e6;
    let err = match build_endpoint(&cfg, Weak::new(), tx, CYCLES_PER_SECOND, clock_of) {
        Err(e) => e,
        Ok(_) => panic!("a max_error past the u32 tick budget must not build an endpoint"),
    };
    assert!(err.contains("tick budget"), "{err}");
}

#[test]
fn hp_encoder_builds_an_endpoint_without_a_max_error_budget() {
    let (tx, _rx) = crossbeam_channel::unbounded();
    let clock_of: ClockSource = Arc::new(|_| Some((0, CYCLES_PER_SECOND)));
    let mut cfg = stepcompress_cfg(128);
    cfg.stepcompress_encoders = vec![StepcompressEncoder::HighPrecision];
    cfg.stepcompress_max_error_secs = 0.0;
    build_endpoint(&cfg, Weak::new(), tx, CYCLES_PER_SECOND, clock_of)
        .expect("hp ignores the max_error budget and must build");
}

/// One logical axis owns one contiguous run of motors. Two runs would route
/// frames to the union while crediting retirement per run, so the mismatch is
/// refused where it is configured rather than diagnosed from a stalled cohort.
#[test]
fn an_axis_split_across_two_motor_runs_is_a_build_error() {
    let (tx, _rx) = crossbeam_channel::unbounded();
    let clock_of: ClockSource = Arc::new(|_| Some((0, CYCLES_PER_SECOND)));
    let mut cfg = stepcompress_cfg(128);
    cfg.axes = vec![0, 1, 0];
    cfg.max_motor_velocity = vec![100.0; 3];
    cfg.lane_kinds = vec![LaneKind::Pulse; 3];
    cfg.motor_counts = vec![1; 3];
    cfg.microstep_distance = vec![MICROSTEP; 3];
    cfg.invert_dir = vec![false; 3];
    cfg.stepper_oids = vec![OID, OID + 1, OID + 2];
    cfg.step_pulse_seconds = vec![2e-6; 3];
    cfg.stepcompress_encoders = vec![StepcompressEncoder::Classic; 3];
    let err = match build_endpoint(&cfg, Weak::new(), tx, CYCLES_PER_SECOND, clock_of) {
        Err(e) => e,
        Ok(_) => panic!("an axis in two motor runs must not build an endpoint"),
    };
    assert!(err.contains("two separate motor runs"), "{err}");
}

#[test]
fn one_endpoint_can_mix_classic_and_high_precision_motors() {
    let (tx, _rx) = crossbeam_channel::unbounded();
    let clock_of: ClockSource = Arc::new(|_| Some((0, CYCLES_PER_SECOND)));
    let mut cfg = stepcompress_cfg(128);
    cfg.motor_counts = vec![2];
    cfg.microstep_distance = vec![0.01; 2];
    cfg.invert_dir = vec![false; 2];
    cfg.stepper_oids = vec![OID, OID + 1];
    cfg.step_pulse_seconds = vec![2e-6; 2];
    cfg.stepcompress_encoders = vec![
        StepcompressEncoder::Classic,
        StepcompressEncoder::HighPrecision,
    ];
    let endpoint = build_endpoint(&cfg, Weak::new(), tx, CYCLES_PER_SECOND, clock_of)
        .expect("one mcu may opt individual motors into high precision");
    assert!(matches!(
        endpoint.shim.motor_encoder(0),
        StepEncoder::Classic { .. }
    ));
    assert_eq!(endpoint.shim.motor_encoder(1), StepEncoder::HighPrecision);
}

#[test]
fn expand_clock32_picks_the_value_nearest_the_reference() {
    assert_eq!(expand_clock32(0x1_0000_0000, 0x0000_0010), 0x1_0000_0010);
    assert_eq!(expand_clock32(0x1_0000_0010, 0xffff_fff0), 0x0_ffff_fff0);
    assert_eq!(expand_clock32(0x0_ffff_fff0, 0x0000_0010), 0x1_0000_0010);
}

#[test]
fn queue_step_span_matches_the_mcu_stepper_loop() {
    let (interval, count, add) = (100u32, 5u16, 3i16);
    let mut clock = 0i64;
    let mut iv = i64::from(interval);
    for _ in 0..count {
        clock += iv;
        iv += i64::from(add);
    }
    assert_eq!(queue_step_span(interval, count, add), clock);
}

#[test]
fn ticks_alone_carry_a_finished_stream_to_full_retirement() {
    let mut h = harness(1024);
    h.now.store(1_000, Ordering::Relaxed);
    let spans = ramp(2_000, 10);
    let last_end = 2_000 + 2_000 * 10;
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(spans)])
        .unwrap();

    let mut now = 1_000_u64;
    while now < last_end + 1_200_000 {
        now += 10_000;
        h.now.store(now, Ordering::Relaxed);
        h.endpoint.tick().unwrap();
        h.ack_sent_barriers();
    }

    assert_eq!(
        h.latest_retired().expect("ticks post heartbeats"),
        vec![10],
        "every pushed view must retire once the clock passes the stream end"
    );
    assert_eq!(h.endpoint.shim.queued_spans(), 0);
}

#[test]
fn a_fresh_epoch_without_a_clock_slope_fails_loud() {
    let mut h = harness(1024);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 40))])
        .unwrap();

    h.endpoint.mark_reanchor(0, 81_834, None);
    let err = h
        .endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp_from(81_834, 8, 5.0))])
        .expect_err("a fresh epoch that carries no slope must not be cut silently");
    assert!(format!("{err:?}").contains("no clock slope"), "{err:?}");
}

#[test]
fn retirement_waits_for_execution_not_transmission() {
    let mut h = harness(1024);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 64))])
        .unwrap();

    let sent_while_unexecuted = h.sent.lock_ok().len();
    assert!(
        sent_while_unexecuted > 0,
        "frames must reach the wire before the mcu clock advances"
    );
    assert_eq!(
        h.endpoint.published_counts(),
        vec![0],
        "retirement must not advance while the moves are still in flight"
    );

    h.now.store(10_000_000, Ordering::Relaxed);
    h.endpoint.tick().unwrap();
    h.ack_sent_barriers();
    assert!(
        h.endpoint.published_counts()[0] > 0,
        "retirement must advance once the mcu has acked the cohort's barrier"
    );
}

#[test]
fn a_lone_follower_lane_reports_retirement_against_its_own_axis() {
    const EXTRUDER_AXIS: u8 = 3;
    let mut h = harness_on_axis(1024, EXTRUDER_AXIS as usize);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[frame_for_axis(EXTRUDER_AXIS, ramp(2_000, 64))])
        .unwrap();
    h.now.store(10_000_000, Ordering::Relaxed);
    h.endpoint.tick().unwrap();
    h.ack_sent_barriers();

    let heartbeat = h.latest_heartbeat().expect("a heartbeat is always posted");
    assert_eq!(
        heartbeat.axes,
        vec![EXTRUDER_AXIS],
        "the heartbeat speaks only for the axes this endpoint drives"
    );
    assert!(
        heartbeat.retired_counts[0] > 0,
        "the pump keys its rings by axis; motor 0's retirements must land on \
         axis {EXTRUDER_AXIS} or that lane's ring never drains: {:?}",
        heartbeat.retired_counts
    );
}

#[test]
fn a_virgin_follower_lane_emits_frames_and_a_barrier_on_first_motion() {
    const EXTRUDER_AXIS: u8 = 3;
    let mut h = harness_on_axis(1024, EXTRUDER_AXIS as usize);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .mark_reanchor(EXTRUDER_AXIS, 2_000, Some(CYCLES_PER_SECOND));
    h.endpoint
        .send_frames(MCU_ID, &[frame_for_axis(EXTRUDER_AXIS, ramp(2_000, 64))])
        .expect("first motion on a never-homed lane must be accepted");

    assert!(
        h.sent_moves() > 0,
        "the fresh-epoch cut must not swallow the lane's first steps"
    );
    h.now.store(10_000_000, Ordering::Relaxed);
    h.endpoint.tick().unwrap();
    assert!(
        !h.barriers.lock_ok().is_empty(),
        "without a barrier the mcu can never ack this lane's retirement"
    );
}

#[test]
fn four_motor_fresh_anchor_emits_and_releases_each_retirement_barrier() {
    let oids = vec![6, 7, 8, 9];
    let mut h = harness_axes(1024, vec![0, 1, 2, 3], oids.clone());
    h.now.store(1_000, Ordering::Relaxed);
    for axis in 0..4 {
        h.endpoint
            .mark_reanchor(axis, 2_000, Some(CYCLES_PER_SECOND));
    }
    h.endpoint
        .send_frames(
            MCU_ID,
            &(0..4)
                .map(|axis| frame_for_axis(axis, ramp(2_000, 64)))
                .collect::<Vec<_>>(),
        )
        .unwrap();

    h.now.store(10_000_000, Ordering::Relaxed);
    h.endpoint.tick().unwrap();
    let issued = std::mem::take(&mut *h.barriers.lock_ok());
    assert_eq!(issued.len(), oids.len(), "issued={issued:?}");
    assert_eq!(issued.iter().map(|&(oid, _)| oid).collect::<Vec<_>>(), oids);

    for &(oid, seq) in &issued[..3] {
        h.endpoint.on_barrier_ack(oid, seq).unwrap();
    }
    assert_eq!(h.endpoint.published_counts(), vec![0; 4]);

    let (oid, seq) = issued[3];
    h.endpoint.on_barrier_ack(oid, seq).unwrap();
    assert_eq!(h.endpoint.published_counts(), vec![64; 4]);
}

#[test]
fn a_cohort_is_consumed_before_its_barrier_retires_it() {
    let mut h = harness(1024);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 64))])
        .unwrap();

    h.now.store(10_000_000, Ordering::Relaxed);
    h.endpoint.tick().unwrap();
    assert!(
        h.endpoint.backlog.is_empty(),
        "every frame including the barrier must be on the wire"
    );
    let heartbeat = h.latest_heartbeat().expect("ticks post heartbeats");
    assert_eq!(heartbeat.consumed_counts, Some(vec![64]));
    assert_eq!(
        heartbeat.retired_counts,
        vec![0],
        "an unacked execution barrier must not report the cohort retired"
    );

    let issued = h.barriers.lock_ok().clone();
    let first_seq = issued.first().map(|&(oid, seq)| {
        assert_eq!(oid, OID);
        seq
    });
    assert!(first_seq.is_some());

    h.ack_sent_barriers();
    assert!(
        h.endpoint.published_counts()[0] > 0,
        "the ack must release the cohort"
    );
}

#[test]
fn retirement_barriers_coalesce_while_an_ack_is_outstanding() {
    let mut h = harness(1024);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 64))])
        .unwrap();
    assert_eq!(h.barriers.lock_ok().len(), 1);
    let first_seq = h.barriers.lock_ok()[0].1;

    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp_from(130_000, 64, 8.0))])
        .unwrap();
    for now in (140_000..=400_000).step_by(10_000) {
        h.now.store(now, Ordering::Relaxed);
        h.endpoint.tick().unwrap();
    }
    assert_eq!(
        h.barriers.lock_ok().len(),
        1,
        "an unacknowledged cohort must bound barrier traffic"
    );

    h.ack_sent_barriers();
    h.endpoint.tick().unwrap();
    let barriers = h.barriers.lock_ok();
    assert_eq!(
        barriers.as_slice(),
        &[(OID, first_seq.wrapping_add(1))],
        "coalesced retirements need one new execution watermark"
    );
}

#[test]
fn a_barrier_ack_below_the_high_water_mark_is_ignored() {
    let mut h = harness(1024);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 64))])
        .unwrap();
    h.now.store(10_000_000, Ordering::Relaxed);
    h.endpoint.tick().unwrap();
    assert!(!h.barriers.lock_ok().is_empty(), "the run issues barriers");

    let seq = h.barriers.lock_ok()[0].1;
    h.endpoint.on_barrier_ack(OID, seq).unwrap();
    h.endpoint
        .on_barrier_ack(OID, seq)
        .expect("a replayed ack is already covered, not a protocol break");
}

#[test]
fn barrier_acknowledgements_cross_rollover_and_ignore_pre_wrap_replay() {
    let mut h = harness(1024);
    h.endpoint.barrier_seq_seed = u32::MAX - 1;

    for (index, seq) in [u32::MAX - 1, u32::MAX, 0].into_iter().enumerate() {
        let retired = (index + 1) as u32;
        h.endpoint.publish_retirement(&[retired], 0);
        h.endpoint
            .flush(McuClock {
                now: 0,
                freq: CYCLES_PER_SECOND,
            })
            .unwrap();
        let issued: Vec<(u32, u32)> = std::mem::take(&mut h.barriers.lock_ok());
        assert_eq!(issued, vec![(OID, seq)]);
        h.endpoint.on_barrier_ack(OID, seq).unwrap();
        assert_eq!(h.endpoint.published_counts(), vec![retired]);
    }

    h.endpoint
        .on_barrier_ack(OID, u32::MAX)
        .expect("a pre-wrap replay is already covered by the post-wrap high-water mark");
    assert_eq!(h.endpoint.published_counts(), vec![3]);

    let err = h
        .endpoint
        .on_barrier_ack(OID, 1)
        .expect_err("the next post-wrap sequence has not been issued");
    assert!(format!("{err:?}").contains("ahead of"), "{err:?}");
}

#[test]
fn a_barrier_ack_ahead_of_what_was_issued_is_fatal() {
    let mut h = harness(1024);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 64))])
        .unwrap();
    let issued = h.barriers.lock_ok()[0].1;
    let err = h
        .endpoint
        .on_barrier_ack(OID, issued.wrapping_add(5))
        .expect_err("an ack for a barrier never issued must be fatal");
    assert!(format!("{err:?}").contains("ahead of"), "{err:?}");
}

#[test]
fn a_barrier_ack_for_an_unknown_oid_is_fatal() {
    let mut h = harness(1024);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 64))])
        .unwrap();
    let err = h
        .endpoint
        .on_barrier_ack(OID + 100, 0)
        .expect_err("an ack for an unknown oid must be fatal");
    assert!(
        format!("{err:?}").contains("no barrier was ever issued"),
        "{err:?}"
    );
}

#[test]
fn views_staged_after_a_mark_must_carry_the_incoming_epoch_slope() {
    const EPOCH_FREQ: f64 = CYCLES_PER_SECOND * 1.000_002;
    let mut h = harness(1024);
    assert_eq!(
        h.endpoint.shim.motor_cycles_per_second(0),
        CYCLES_PER_SECOND
    );

    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint.mark_reanchor(0, 2_000, Some(EPOCH_FREQ));
    let err = h
        .endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 4))])
        .expect_err("views clocked on the outgoing slope belong to the previous epoch");
    assert!(
        format!("{err:?}").contains("SpanFrequencyMismatch"),
        "{err:?}"
    );
}

#[test]
fn a_cut_moves_the_motor_onto_the_adopted_epoch_slope() {
    const EPOCH_FREQ: f64 = CYCLES_PER_SECOND * 1.000_002;
    let mut h = harness(1024);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 8))])
        .unwrap();
    h.endpoint.mark_reanchor(0, 50_000, Some(EPOCH_FREQ));
    h.endpoint
        .send_frames(
            MCU_ID,
            &[axis_frame(epoch_ramp_from(50_000, 4, EPOCH_FREQ, 1.0, 1.0))],
        )
        .unwrap();
    assert_eq!(
        h.endpoint.shim.motor_cycles_per_second(0),
        EPOCH_FREQ,
        "the shim adopted the epoch slope at the cut"
    );
}

/// A lane that holds through a long print (Z parked while XY runs) is
/// unreachable from the clock the mcu stepper is anchored on when it finally
/// moves. The endpoint must re-anchor it mid-stream and keep pacing,
/// retiring and barrier-acking exactly as before the re-anchor.
#[test]
fn a_lane_parked_past_the_encoder_window_re_anchors_mid_stream() {
    let hold_secs = (step_shim::compress::CLOCK_DIFF_MAX as f64 / CYCLES_PER_SECOND).ceil() + 60.0;
    let mut h = harness(1024);
    h.now.store(1_000, Ordering::Relaxed);

    let lift = span(2_000, 0.0, 1.0, 0.010);
    let hold = span(lift.end_clock, 1.0, 1.0, hold_secs);
    let resume = span(hold.end_clock, 1.0, 2.0, 0.010);
    let end = resume.end_clock;
    h.endpoint
        .send_frames(
            MCU_ID,
            &[axis_frame(vec![lift.clone(), hold.clone(), resume.clone()])],
        )
        .unwrap();

    for now in [lift.end_clock, hold.end_clock, end + 1_000_000] {
        h.now.store(now, Ordering::Relaxed);
        h.endpoint.tick().unwrap();
        h.ack_sent_barriers();
    }
    for _ in 0..RETIREMENT_IDLE_TICKS {
        h.endpoint.tick().unwrap();
        h.ack_sent_barriers();
    }

    let sent = h.sent.lock_ok().clone();
    let resets: Vec<u64> = sent
        .iter()
        .filter_map(|f| match *f {
            StepFrame::ResetStepClock { clock, .. } => Some(u64::from(clock)),
            _ => None,
        })
        .collect();
    assert_eq!(
        resets.len(),
        2,
        "the parked lane must be re-anchored exactly once: {resets:?}"
    );
    assert!(
        resets[1] >= resume.start_clock
            && resets[1] < resume.start_clock + RESUME_ANCHOR_SLACK_CYCLES,
        "the re-anchor must land at the first step of the resuming move, not at \
         some stale cursor: anchor {}, move starts {}",
        resets[1],
        resume.start_clock
    );
    let steps: u32 = sent
        .iter()
        .map(|f| match *f {
            StepFrame::QueueStep { count, .. } => u32::from(count),
            _ => 0,
        })
        .sum();
    assert_eq!(steps, 200, "no step may be lost across the re-anchor");
    assert_eq!(
        h.endpoint.published_counts(),
        vec![3],
        "retirement must still complete across a mid-stream re-anchor"
    );
}

#[test]
fn a_flush_hands_the_whole_burst_to_the_transport_in_one_call() {
    let mut h = harness(1024);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 40))])
        .unwrap();
    let bursts = h.bursts.lock_ok().clone();
    let frames: usize = bursts.iter().sum();
    assert!(frames > 8, "expected a multi-frame burst, got {frames}");
    assert_eq!(
        bursts.iter().filter(|&&n| n > 0).count(),
        1,
        "a flush must reach the transport as one batch so it can be packed \
         into full message blocks, got bursts {bursts:?}"
    );
}

#[test]
fn a_budget_capped_flush_still_batches_what_it_may_send() {
    let mut h = harness(4);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 40))])
        .unwrap();
    let bursts = h.bursts.lock_ok().clone();
    assert_eq!(bursts.len(), 1, "{bursts:?}");
    assert_eq!(h.sent_moves(), 4, "the move-slot budget still bounds sends");
    assert!(!h.endpoint.backlog.is_empty());
}

/// Mirrors `STEP_CLOCK_HORIZON_TICKS` in `src/stepper_classic.c`, where a step
/// clock this far from the MCU's own clock shuts the board down with
/// "Step clock beyond sync horizon".
const MCU_STEP_CLOCK_HORIZON_TICKS: u64 = 3 << 28;

/// The MCU guard's bound must admit every frame the endpoint can legitimately
/// emit: the drain window reaches `SEND_LEAD_SECONDS` past the projected MCU
/// clock, which is a fraction of the horizon at every bench frequency.
#[test]
fn the_mcu_horizon_admits_the_endpoints_full_send_lead() {
    for freq in [1_000_000.0_f64, 168_000_000.0, 400_000_000.0, 480_000_000.0] {
        let lead_ticks = (SEND_LEAD_SECONDS * freq) as u64;
        assert!(
            lead_ticks < MCU_STEP_CLOCK_HORIZON_TICKS,
            "at {freq} Hz the endpoint reaches {lead_ticks} ticks ahead, above the \
             {MCU_STEP_CLOCK_HORIZON_TICKS}-tick mcu horizon — the guard would reject \
             healthy frames"
        );
    }
}

/// Guard interplay: with a healthy clock record every step clock the endpoint
/// puts on the wire lands inside `SEND_LEAD_SECONDS` of the MCU clock at send
/// time, and therefore far inside the MCU's horizon. The MCU guard can only
/// fire on a wrong host record, never on healthy pacing.
#[test]
fn no_emitted_step_clock_leaves_the_mcu_sync_horizon() {
    let slow_ramp = paceable_ramp(2_000, 48);
    let mut h = harness(64);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(slow_ramp)])
        .unwrap();

    let lead_ticks = (SEND_LEAD_SECONDS * CYCLES_PER_SECOND) as u64;
    let mut cursor: Option<u64> = None;
    let mut checked = 0usize;
    let mut now = 1_000u64;
    let mut rounds_with_frames = 0usize;
    let mut checked_this_round = false;
    for _ in 0..260 {
        for frame in std::mem::take(&mut *h.sent.lock_ok()) {
            let first_step = match frame {
                StepFrame::ResetStepClock { clock, .. } => {
                    cursor = Some(u64::from(clock));
                    u64::from(clock)
                }
                StepFrame::QueueStep {
                    interval,
                    count,
                    add,
                    ..
                } => {
                    let at = cursor.expect("a queue_step must follow a reset_step_clock");
                    let first = at + u64::from(interval);
                    cursor = Some(at.saturating_add_signed(queue_step_span(interval, count, add)));
                    first
                }
                StepFrame::SetNextStepDir { .. } | StepFrame::QueueStepHp { .. } => continue,
            };
            checked += 1;
            checked_this_round = true;
            let distance = first_step as i64 - now as i64;
            assert!(
                distance <= lead_ticks as i64,
                "step clock {first_step} is {distance} ticks ahead of the mcu clock \
                 {now} — past the {lead_ticks}-tick send lead"
            );
            assert!(
                distance.unsigned_abs() < MCU_STEP_CLOCK_HORIZON_TICKS,
                "step clock {first_step} is {distance} ticks from the mcu clock {now} — \
                 the mcu would shut down on the sync horizon"
            );
        }
        if checked_this_round {
            rounds_with_frames += 1;
            checked_this_round = false;
        }
        now += 5_000;
        h.now.store(now, Ordering::Relaxed);
        h.endpoint.tick().unwrap();
        h.ack_sent_barriers();
    }
    assert!(
        checked >= 10,
        "the run must exercise a real volley of step frames, saw {checked}"
    );
    assert!(
        rounds_with_frames > 1,
        "the endpoint must have released frames across several ticks for the send \
         lead to be under test, saw {rounds_with_frames} rounds with frames"
    );
}

fn four_motor_harness() -> (Harness, Vec<u32>) {
    let oids = vec![6, 7, 8, 9];
    let mut h = harness_axes(1024, vec![0, 1, 2, 3], oids.clone());
    h.now.store(1_000, Ordering::Relaxed);
    for axis in 0..4 {
        h.endpoint
            .mark_reanchor(axis, 2_000, Some(CYCLES_PER_SECOND));
    }
    (h, oids)
}

/// Drive one retirement cohort across all four lanes and return the barriers
/// the endpoint put on the wire for it.
fn run_cohort(h: &mut Harness, start: u64, from_mm: f64) -> Vec<(u32, u32)> {
    h.now.store(start.saturating_sub(1_000), Ordering::Relaxed);
    h.endpoint
        .send_frames(
            MCU_ID,
            &(0..4)
                .map(|axis| frame_for_axis(axis, ramp_from(start, 64, from_mm)))
                .collect::<Vec<_>>(),
        )
        .unwrap();
    h.now.store(start + 10_000_000, Ordering::Relaxed);
    h.endpoint.tick().unwrap();
    std::mem::take(&mut *h.barriers.lock_ok())
}

/// The bench saw one lane collect four barriers per volley while its siblings
/// collected one — the signature of a motor index leaking into the oid the
/// barrier is addressed to. Every cohort must address each oid exactly once.
#[test]
fn every_cohort_addresses_each_oid_with_exactly_one_barrier() {
    let (mut h, oids) = four_motor_harness();
    let mut all = Vec::new();
    let mut start = 2_000u64;
    let mut from_mm = 0.0_f64;
    for cohort in 0..3 {
        let issued = run_cohort(&mut h, start, from_mm);
        assert_eq!(
            issued.iter().map(|&(oid, _)| oid).collect::<Vec<_>>(),
            oids,
            "cohort {cohort} must carry one barrier per configured oid: {issued:?}"
        );
        for &(oid, seq) in &issued {
            h.endpoint.on_barrier_ack(oid, seq).unwrap();
        }
        assert_eq!(
            h.endpoint.published_counts(),
            vec![64 * (cohort + 1); 4],
            "cohort {cohort} must retire once every lane's barrier is acked"
        );
        all.extend(issued);
        start += 2_000 * 64;
        from_mm += 8.0;
    }
    let mut unique = all.clone();
    unique.sort_unstable();
    unique.dedup();
    assert_eq!(
        unique.len(),
        all.len(),
        "no (oid, seq) may be issued twice: {all:?}"
    );
}

#[test]
fn a_barrier_ack_replayed_by_the_mcu_retires_its_cohort_once() {
    let (mut h, oids) = four_motor_harness();
    let issued = run_cohort(&mut h, 2_000, 0.0);
    assert_eq!(issued.len(), oids.len(), "{issued:?}");

    for &(oid, seq) in &issued {
        for _ in 0..4 {
            h.endpoint
                .on_barrier_ack(oid, seq)
                .expect("a replayed ack is a duplicate receipt, not a protocol break");
        }
    }
    assert_eq!(h.endpoint.published_counts(), vec![64; 4]);

    let next = run_cohort(&mut h, 2_000 + 2_000 * 64, 8.0);
    assert_eq!(
        next.iter().map(|&(oid, _)| oid).collect::<Vec<_>>(),
        oids,
        "replayed acks must not consume the next cohort's barriers: {next:?}"
    );
}

#[test]
fn a_lost_barrier_ack_trips_the_deadline_instead_of_waiting_forever() {
    let deadline_ticks = (BARRIER_ACK_DEADLINE_SECONDS * CYCLES_PER_SECOND) as u64;
    let (mut h, _) = four_motor_harness();
    let issued = run_cohort(&mut h, 2_000, 0.0);
    h.endpoint.barrier_ack_deadline_secs = BARRIER_ACK_DEADLINE_SECONDS;
    let (lost_oid, lost_seq) = *issued.last().expect("the cohort issued barriers");
    for &(oid, seq) in &issued[..issued.len() - 1] {
        h.endpoint.on_barrier_ack(oid, seq).unwrap();
    }
    assert_eq!(
        h.endpoint.published_counts(),
        vec![0; 4],
        "the cohort must still be waiting on the lost ack"
    );
    let sent_clock = h
        .endpoint
        .sent_barriers
        .iter()
        .find(|sent| sent.id.oid == lost_oid && sent.id.seq == lost_seq)
        .map(|sent| sent.sent_clock)
        .expect("the lost barrier reached the wire");

    h.now
        .store(sent_clock + deadline_ticks - 1, Ordering::Relaxed);
    h.endpoint
        .tick()
        .expect("inside the deadline a barrier is merely in flight");

    h.now.store(sent_clock + deadline_ticks, Ordering::Relaxed);
    let err = h
        .endpoint
        .tick()
        .expect_err("a barrier the mcu never acks must not park the cohort forever");
    let SendError::Fatal(message) = err else {
        panic!("a lost barrier ack is unrecoverable: {err:?}");
    };
    assert!(
        message.contains(&format!("oid={lost_oid} seq={lost_seq}")),
        "the fatal must name the outstanding barrier: {message}"
    );
    for &(oid, seq) in &issued[..issued.len() - 1] {
        assert!(
            message.contains(&format!("oid={oid} acked_through_seq={seq}")),
            "the fatal must carry the received-ack ledger: {message}"
        );
    }
    let escalated = h.heartbeats.try_iter().any(|msg| {
        matches!(
            msg,
            PumpMsg::StepcompressFatal { mcu_id, .. } if mcu_id == MCU_ID
        )
    });
    assert!(
        escalated,
        "the endpoint must escalate to the pump, or the drain still hangs"
    );
}

/// The pacer used to only log a fatal tick, so a frame the egress guard
/// refused was retried every 10 ms forever: the backlog froze, the retirement
/// cohort never released and klippy's `wait_moves` hung with the hotend parked
/// hot (bench, 2026-08-17 10:31). A fatal is unrecoverable by construction —
/// it must escalate once and take the endpoint out of the rotation.
#[test]
fn a_guard_tripped_head_frame_escalates_once_and_latches() {
    let mut h = harness(1024);
    h.now.store(5_000_000, Ordering::Relaxed);
    let err = h
        .endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 8))])
        .expect_err("a volley whose head is seconds in the past must not reach the wire");
    let SendError::Fatal(message) = err else {
        panic!("a late volley head is unrecoverable: {err:?}");
    };
    assert!(message.contains("reset_step_clock"), "{message}");
    assert!(h.endpoint.is_fatal());

    let attempts_at_latch = h.attempts.lock_ok().len();
    for _ in 0..4 {
        let repeat = h
            .endpoint
            .tick()
            .expect_err("a latched endpoint must refuse every further tick");
        let SendError::Fatal(repeat) = repeat else {
            panic!("the latched error must stay fatal: {repeat:?}");
        };
        assert_eq!(repeat, message);
    }
    assert_eq!(
        h.attempts.lock_ok().len(),
        attempts_at_latch,
        "a latched endpoint must not touch the wire again"
    );

    let escalations = h
        .heartbeats
        .try_iter()
        .filter(|msg| matches!(msg, PumpMsg::StepcompressFatal { mcu_id, .. } if *mcu_id == MCU_ID))
        .count();
    assert_eq!(
        escalations, 1,
        "the pump must be told exactly once, so it exits and klippy's wait_moves raises"
    );
}

#[test]
fn the_pacer_stops_ticking_an_endpoint_that_went_fatal() {
    let mut h = harness(BUDGET);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 40))])
        .expect("the first volley is punctual; the budget holds the rest back");
    h.now.store(5_000_000, Ordering::Relaxed);
    let Harness {
        endpoint,
        clock_calls,
        heartbeats,
        ..
    } = h;

    let endpoint = Arc::new(Mutex::new(endpoint));
    let pacer = StepcompressPacer::spawn(vec![Arc::clone(&endpoint)]);
    std::thread::sleep(PACER_TICK * 10);
    let after_latch = clock_calls.load(Ordering::Relaxed);
    std::thread::sleep(PACER_TICK * 10);
    assert_eq!(
        clock_calls.load(Ordering::Relaxed),
        after_latch,
        "the pacer must drop a fatal endpoint instead of retrying it every tick"
    );
    drop(pacer);

    assert!(endpoint.lock_ok().is_fatal());
    let escalations = heartbeats
        .try_iter()
        .filter(|msg| matches!(msg, PumpMsg::StepcompressFatal { mcu_id, .. } if *mcu_id == MCU_ID))
        .count();
    assert_eq!(escalations, 1);
}

/// A lane holds — Z or the extruder between layer changes — for minutes
/// before it steps again, and the shim carries no stepped clock for the hold:
/// the stream's committed origin is the seam the hold began on. Basing the
/// resumed volley's reset_step_clock on that origin hands the mcu a frame
/// minutes in the past; the egress guard then refuses the head of the volley
/// and the lane never resumes (bench, 2026-08-17 10:31:29 — reset at the
/// 10:31:28.806 anchor clock, 224 ms late). The reset must track the volley's
/// own first step, which the drain releases inside the send lead.
#[test]
fn a_lane_that_holds_before_it_steps_resumes_on_a_punctual_reset() {
    const EPOCH_FREQ: f64 = CYCLES_PER_SECOND * 1.000_002;
    const HOLD_SECS: f64 = 5.0;
    let anchor = 10_000_000_u64;
    let anchor_lead = (0.5 * CYCLES_PER_SECOND) as u64;
    let send_lead = (SEND_LEAD_SECONDS * CYCLES_PER_SECOND) as u64;
    let tick_ticks = CYCLES_PER_SECOND as u64 / 10;

    let mut h = harness(1024);
    h.now.store(anchor - anchor_lead, Ordering::Relaxed);
    h.endpoint.mark_reanchor(0, anchor, Some(EPOCH_FREQ));

    let hold = hold_span(anchor, 0.0, HOLD_SECS, EPOCH_FREQ);
    let motion_start = hold.end_clock;
    let mut spans = vec![hold];
    spans.extend(epoch_ramp(motion_start, 8, EPOCH_FREQ));
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(spans)])
        .expect("a marked fresh epoch may start at any clock");

    let mut emitted = None;
    for _ in 0..(HOLD_SECS as u64 * 10 + 20) {
        let now = h.now.load(Ordering::Relaxed) + tick_ticks;
        h.now.store(now, Ordering::Relaxed);
        h.endpoint
            .tick()
            .expect("a lane resuming from a hold must never hand the mcu a late frame");
        let reset = h.sent.lock_ok().iter().find_map(|f| match f {
            StepFrame::ResetStepClock { clock, .. } => Some(u64::from(*clock)),
            _ => None,
        });
        if let Some(reset) = reset {
            emitted = Some((reset, now));
            break;
        }
    }

    let (reset, now_at_send) = emitted.expect("the resumed lane must re-anchor the mcu step clock");
    assert!(
        reset >= now_at_send,
        "reset {reset} reached the wire {} ticks behind the mcu clock {now_at_send}",
        now_at_send - reset
    );
    assert!(
        reset <= now_at_send + send_lead,
        "reset {reset} is further than the {send_lead}-tick send lead ahead of {now_at_send}"
    );
}

/// The same hold, resumed the other way round. `set_next_step_dir` carries no
/// clock on the wire, so its guard clock is pure host bookkeeping — and it
/// used to be the lane's cursor from *before* the hold, seconds behind the
/// projected mcu clock, while the volley it heads is punctual (bench,
/// 2026-08-17 10:43 — dir frame 586 ms late, print killed). A clock-less
/// frame must be timed by the step it applies to.
#[test]
fn a_lane_that_reverses_after_a_hold_times_its_dir_frame_by_the_step_it_heads() {
    const EPOCH_FREQ: f64 = CYCLES_PER_SECOND * 1.000_002;
    const HOLD_SECS: f64 = 5.0;
    const RISE: usize = 6;
    let anchor = 10_000_000_u64;
    let anchor_lead = (0.5 * CYCLES_PER_SECOND) as u64;
    let tick_ticks = CYCLES_PER_SECOND as u64 / 10;

    let mut h = harness(1024);
    h.now.store(anchor - anchor_lead, Ordering::Relaxed);
    h.endpoint.mark_reanchor(0, anchor, Some(EPOCH_FREQ));

    let mut spans = epoch_ramp(anchor, RISE, EPOCH_FREQ);
    let top_mm = span_end_mm(spans.last().expect("the rise carries views"));
    let hold_start = spans.last().expect("the rise carries views").end_clock;
    let hold = hold_span(hold_start, top_mm, HOLD_SECS, EPOCH_FREQ);
    let resume_start = hold.end_clock;
    spans.push(hold);
    spans.extend(epoch_ramp_from(
        resume_start,
        RISE,
        EPOCH_FREQ,
        top_mm,
        -1.0,
    ));
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(spans)])
        .expect("a marked fresh epoch may start at any clock");

    let mut reversed_at = None;
    for _ in 0..(HOLD_SECS as u64 * 10 + 40) {
        let now = h.now.load(Ordering::Relaxed) + tick_ticks;
        h.now.store(now, Ordering::Relaxed);
        h.endpoint
            .tick()
            .expect("a lane reversing out of a hold must never hand the mcu a late frame");
        let sent = h.sent.lock_ok();
        if let Some(index) = sent
            .iter()
            .position(|f| matches!(f, StepFrame::SetNextStepDir { dir: 0, .. }))
        {
            reversed_at = Some((index, sent.len()));
            break;
        }
    }

    let (dir_index, sent_len) = reversed_at.expect("the resumed lane must reverse the mcu latch");
    assert!(
        dir_index + 1 < sent_len,
        "the reversing dir frame must be followed by the run it applies to"
    );
    assert!(
        matches!(h.sent.lock_ok()[dir_index + 1], StepFrame::QueueStep { .. }),
        "set_next_step_dir must precede the queue_step it applies to: {:?}",
        &h.sent.lock_ok()[dir_index..],
    );
}

/// A sent-frame cut parks the lane until the mcu acks the barrier and
/// `complete_cut` re-anchors it with a fresh `reset_step_clock`. The mcu holds
/// every stepper in that window under `SF_NEED_RESET` and silently discards any
/// `queue_step` that arrives before the reset (`stepper_classic.c`
/// `enqueue_move`), so nothing may leave the endpoint for a cut-pending oid
/// until its reset heads the resumed volley.
#[test]
fn a_cut_pending_lane_sends_nothing_before_its_reset() {
    let mut h = harness(1024);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 40))])
        .unwrap();
    h.endpoint.mark_reanchor(0, 81_834, Some(CYCLES_PER_SECOND));
    h.auto_query.store(false, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp_from(81_834, 8, 5.0))])
        .unwrap();
    assert!(h.endpoint.lanes[0].pending_cut.is_some());

    let parked = h.sent.lock_ok().len();
    for _ in 0..20 {
        let now = h.now.load(Ordering::Relaxed) + 2_000;
        h.now.store(now, Ordering::Relaxed);
        h.endpoint.tick().expect("a parked cut is not a fault");
    }
    assert_eq!(
        h.sent.lock_ok().len(),
        parked,
        "frames escaped a cut-pending lane before its reset: {:?}",
        &h.sent.lock_ok()[parked..],
    );

    let expected = h.endpoint.lanes[0]
        .pending_cut
        .as_ref()
        .expect("the cut is awaiting reconciliation")
        .expected_count;
    h.query_count.store(expected, Ordering::Relaxed);
    h.ack_sent_barriers_result()
        .expect("the injected mcu count matches the host expectation");
    let sent = h.sent.lock_ok();
    let reset = sent[parked..]
        .iter()
        .position(|f| matches!(f, StepFrame::ResetStepClock { .. }))
        .expect("the completed cut re-anchors the lane");
    assert!(
        sent[parked..][..reset]
            .iter()
            .all(|f| !matches!(f, StepFrame::QueueStep { .. })),
        "the resumed volley must open with its reset: {:?}",
        &sent[parked..],
    );
}

/// A cut barrier that never reaches the wire is the endpoint's one unbounded
/// silent wedge: the lane's pieces sit in `PendingCut::held`, `complete_cut`
/// only ever runs on the ack, and the deadline used to skip any barrier absent
/// from `sent_barriers` — so nothing executed, nothing retired, no error, and
/// the sole symptom was the pump's drip cohort stalling at floor 0 (bench
/// session k-1786973103: every axis "executed 0 queued 94 in_flight 194").
/// Here a saturated move-slot budget holds the barrier in the backlog.
#[test]
fn a_cut_barrier_that_never_reaches_the_wire_trips_the_deadline() {
    const DEADLINE_SECS: f64 = 0.002;
    let deadline_ticks = (DEADLINE_SECS * CYCLES_PER_SECOND) as u64;
    let mut h = harness(1);
    h.endpoint.barrier_ack_deadline_secs = DEADLINE_SECS;
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 40))])
        .unwrap();

    let boundary = h.endpoint.lanes[0]
        .last_sent_boundary
        .expect("the run reached the wire");
    h.endpoint
        .mark_reanchor(0, boundary, Some(CYCLES_PER_SECOND));
    h.auto_query.store(false, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp_from(boundary, 8, 5.0))])
        .unwrap();
    assert!(
        h.endpoint.lanes[0].pending_cut.is_some(),
        "the seam falls inside sent frames, so the cut must reconcile"
    );
    let eligible_clock = h
        .endpoint
        .backlog
        .iter()
        .find_map(|out| match out.frame {
            Outbound::Barrier(_) => Some(out.queued_clock.max(out.start_clock)),
            Outbound::Step(_) => None,
        })
        .expect("the saturated budget holds the cut barrier in the backlog");
    assert!(
        h.barriers.lock_ok().is_empty(),
        "the barrier must not have reached the wire"
    );

    let before_deadline = eligible_clock + deadline_ticks - 1;
    h.endpoint
        .check_barrier_deadline(McuClock {
            now: before_deadline,
            freq: CYCLES_PER_SECOND,
        })
        .expect("inside the deadline a backlogged barrier is merely waiting on a slot");

    let err = h
        .endpoint
        .check_barrier_deadline(McuClock {
            now: eligible_clock + deadline_ticks,
            freq: CYCLES_PER_SECOND,
        })
        .expect_err("a barrier that never reaches the wire must not park the lane forever");
    let SendError::Fatal(message) = err else {
        panic!("a wedged cut is unrecoverable: {err:?}");
    };
    assert!(message.contains("backlogged, never sent"), "{message}");
    assert!(message.contains(&format!("oid={OID}")), "{message}");
}

#[test]
fn host_buzz_returns_to_base_without_advancing_retirement() {
    let mut h = harness(1024);
    let profile = buzz_profile(50_000, 50_000, 100_000, 20, 2);
    h.endpoint
        .arm_buzz(0b001, 0, &profile, buzz_start(&h))
        .expect("idle pulse lane accepts a buzz");
    for tick in 1..=20 {
        h.now
            .store(tick * (CYCLES_PER_SECOND as u64 / 100), Ordering::Relaxed);
        h.endpoint.tick().expect("buzz tick");
    }
    assert!(h.endpoint.buzz.is_none());
    assert_eq!(h.endpoint.shim.commanded_steps(0), 0);
    assert!(h.sent_moves() > 0);
    let (_, retired) = h
        .endpoint
        .counts_by_axis(&h.endpoint.shim.consumed_counts());
    assert_eq!(retired, vec![0]);
}

#[test]
fn host_buzz_anchors_to_the_sampled_position_not_quantized_steps() {
    let mut h = harness(1024);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(vec![span(100_000, 0.0, 0.004, 0.01)])])
        .expect("stage a sub-step move");
    h.now.store(2 * CYCLES_PER_SECOND as u64, Ordering::Relaxed);
    h.endpoint.tick().expect("drain the sub-step move");
    h.ack_sent_barriers();
    assert_eq!(h.endpoint.shim.commanded_steps(0), 0);
    assert_eq!(h.endpoint.shim.queued_spans(), 0);
    let profile = buzz_profile(70_000, 230_000, 23_500, 6, 1);
    h.endpoint
        .arm_buzz(0b001, 0, &profile, buzz_start(&h))
        .expect("idle lane accepts the chirp");
    let signal = h.endpoint.buzz.as_ref().expect("armed buzz").signals[0]
        .as_ref()
        .expect("the driven motor carries a buzz signal");
    let base = signal
        .position(signal.t_start)
        .expect("the buzz signal evaluates at its own start");
    assert!((base - 0.004).abs() < 1e-12);
    let end = signal
        .position(signal.t_end)
        .expect("the buzz signal evaluates at its own end");
    assert!((end - 0.004).abs() < 1e-12);
}

#[test]
fn host_buzz_rejects_a_lane_with_queued_trajectory() {
    let mut h = harness(1024);
    h.endpoint
        .shim
        .push_spans(0, &[span(100_000, 0.0, 1.0, 0.01)])
        .expect("stage trajectory");
    let profile = buzz_profile(50_000, 50_000, 100_000, 20, 2);
    let error = h
        .endpoint
        .arm_buzz(0b001, 0, &profile, buzz_start(&h))
        .expect_err("queued trajectory must reject a buzz");
    assert!(error.to_string().contains("trajectory remains queued"));
}

const H7_FREQ: f64 = 520_000_000.0;

struct McuStepper {
    base: u32,
    need_reset: bool,
}

impl McuStepper {
    fn new() -> Self {
        Self {
            base: 0,
            need_reset: true,
        }
    }

    fn reset_clock(&mut self, clock: u32) {
        self.base = clock;
        self.need_reset = false;
    }

    fn queue_step(&mut self, interval: u32, count: u16, add: i16) -> Option<u32> {
        if self.need_reset {
            return None;
        }
        let first = self.base.wrapping_add(interval);
        let span = queue_step_span(interval, count, add);
        self.base = self.base.wrapping_add(span as u32);
        Some(first)
    }
}

fn h7_harness(oids: Vec<u32>) -> Harness {
    let axis = 2usize;
    let axes: Vec<usize> = vec![axis; oids.len()];
    let now = Arc::new(AtomicU64::new(0));
    let now_for_clock = Arc::clone(&now);
    let clock_calls = Arc::new(AtomicU64::new(0));
    let clock_calls_for_clock = Arc::clone(&clock_calls);
    let clock_of: ClockSource = Arc::new(move |_| {
        clock_calls_for_clock.fetch_add(1, Ordering::Relaxed);
        Some((now_for_clock.load(Ordering::Relaxed), H7_FREQ))
    });
    let sent = Arc::new(Mutex::new(Vec::new()));
    let fail_sends = Arc::new(AtomicBool::new(false));
    let query_count = Arc::new(AtomicI64::new(0));
    let auto_query = Arc::new(AtomicBool::new(true));
    let query_calls = Arc::new(AtomicU64::new(0));
    let sent_for_egress = Arc::clone(&sent);
    let barriers = Arc::new(Mutex::new(Vec::new()));
    let barriers_for_egress = Arc::clone(&barriers);
    let seeds = Arc::new(Mutex::new(Vec::new()));
    let seeds_for_egress = Arc::clone(&seeds);
    let fail_for_egress = Arc::clone(&fail_sends);
    let bursts = Arc::new(Mutex::new(Vec::new()));
    let bursts_for_egress = Arc::clone(&bursts);
    let attempts = Arc::new(Mutex::new(Vec::new()));
    let attempts_for_egress = Arc::clone(&attempts);
    let egress: FrameEgress =
        Arc::new(move |frames: &[(&'static str, Vec<(String, ArgValue)>)]| {
            attempts_for_egress.lock_ok().push(
                frames
                    .iter()
                    .map(|(name, args)| format!("{name}{args:?}"))
                    .collect(),
            );
            if fail_for_egress.load(Ordering::Relaxed) {
                return Err(SendError::Transient("egress down".into()));
            }
            bursts_for_egress.lock_ok().push(frames.len());
            for (name, args) in frames {
                let arg = |key: &str| -> i64 {
                    match args.iter().find(|(k, _)| k == key).map(|(_, v)| v) {
                        Some(ArgValue::Int(v)) => *v,
                        other => panic!("missing int arg {key}: {other:?}"),
                    }
                };
                if *name == "stepcompress_barrier" {
                    barriers_for_egress
                        .lock_ok()
                        .push((arg("oid") as u32, arg("seq") as u32));
                    continue;
                }
                if *name == "kalico_wire_probe" {
                    continue;
                }
                if *name == "stepcompress_set_position" {
                    seeds_for_egress
                        .lock_ok()
                        .push((arg("oid") as u32, arg("pos")));
                    continue;
                }
                let frame = match *name {
                    "queue_step" => StepFrame::QueueStep {
                        oid: arg("oid") as u32,
                        interval: arg("interval") as u32,
                        count: arg("count") as u16,
                        add: arg("add") as i16,
                    },
                    "set_next_step_dir" => StepFrame::SetNextStepDir {
                        oid: arg("oid") as u32,
                        dir: arg("dir") as u8,
                    },
                    "reset_step_clock" => StepFrame::ResetStepClock {
                        oid: arg("oid") as u32,
                        clock: arg("clock") as u32,
                    },
                    other => panic!("unexpected command {other}"),
                };
                sent_for_egress.lock_ok().push(frame);
            }
            Ok(())
        });
    let (tx, rx) = crossbeam_channel::unbounded();
    let query_for_endpoint = Arc::clone(&query_count);
    let calls_for_query = Arc::clone(&query_calls);
    let motors: Vec<MotorConfig> = oids
        .iter()
        .map(|&oid| MotorConfig {
            oid,
            microstep_distance: MICROSTEP,
            invert_dir: false,
            cycles_per_second: H7_FREQ,
            min_rearm_cycles: 0,
            encoder: StepEncoder::Classic {
                max_error_ticks: step_shim::compress::DEFAULT_MAX_ERROR_TICKS,
            },
        })
        .collect();
    let lanes: Vec<StepLaneConfig> = axes
        .iter()
        .zip(&oids)
        .map(|(&axis, &oid)| StepLaneConfig { axis, oid })
        .collect();
    let endpoint = StepcompressEndpoint::new(
        MCU_ID,
        StepShim::new(motors, SHIM_RING_DEPTH),
        &lanes,
        egress,
        tx,
        clock_of,
        1024,
        Arc::new(move |_| {
            calls_for_query.fetch_add(1, Ordering::Relaxed);
            Ok(query_for_endpoint.load(Ordering::Relaxed))
        }),
        None,
        TELEPORTING_CLOCK_ACK_DEADLINE_SECONDS,
    )
    .expect("three motors on one axis build a stepcompress endpoint");
    Harness {
        endpoint,
        now,
        sent,
        barriers,
        seeds,
        heartbeats: rx,
        bursts,
        attempts,
        fail_sends,
        query_calls,
        query_count,
        auto_query,
        clock_calls,
    }
}

fn h7_ramp(start_clock: u64, count: usize, start_mm: f64, direction: f64) -> Vec<ClockedMotorSpan> {
    epoch_ramp_from(start_clock, count, H7_FREQ, start_mm, direction)
}

fn verify_mcu_agreement(
    frames: &[StepFrame],
    lanes: &[Lane],
    mcu: &mut HashMap<u32, McuStepper>,
    _mcu_now_u32: u32,
) -> Result<(), String> {
    for (idx, frame) in frames.iter().enumerate() {
        match *frame {
            StepFrame::ResetStepClock { oid, clock } => {
                let stepper = mcu.entry(oid).or_insert_with(McuStepper::new);
                stepper.reset_clock(clock);
            }
            StepFrame::SetNextStepDir { .. } => {}
            StepFrame::QueueStep {
                oid,
                interval,
                count,
                add,
            } => {
                let stepper = mcu.entry(oid).or_insert_with(McuStepper::new);
                if stepper.need_reset {
                    return Err(format!(
                        "oid {oid} frame {idx}: queue_step arrived while mcu stepper \
                         still needs reset — reset_step_clock was either not sent or \
                         sorted behind this frame due to a stale start_clock"
                    ));
                }
                stepper.queue_step(interval, count, add);
            }
            StepFrame::QueueStepHp { .. } => {}
        }
    }
    for (&oid, stepper) in mcu.iter() {
        if stepper.need_reset {
            continue;
        }
        let host_cursor = lanes
            .iter()
            .find(|lane| lane.oid == oid)
            .and_then(|lane| lane.step_clock)
            .unwrap_or(0);
        if host_cursor as u32 != stepper.base {
            return Err(format!(
                "oid {oid}: host/mcu base divergence: host step_clock low32={} \
                 mcu base={}, host_64={host_cursor}, delta={}",
                host_cursor as u32,
                stepper.base,
                (host_cursor as u32 as i64) - (stepper.base as i64)
            ));
        }
    }
    Ok(())
}
/// Drives a Z_TILT_ADJUST–shaped sequence through the endpoint at H7 clock
/// frequency (520 MHz, 32-bit half-wrap 4.13 s).  The crux: an outstanding
/// `begin_cut` holds `resume_clock = lanes[motor].step_clock` captured seconds
/// before `complete_cut` runs.  When the gap exceeds the 32-bit half-wrap,
/// `complete_cut → queue_step_volley(resume_clock, tail)` hands `frame_clocks`
/// a reference so stale that `expand_clock32` picks the wrong 2³²-multiple,
/// mis-sorting the new epoch's `ResetStepClock` behind its own `QueueStep`
/// frames in the deadline-ordered backlog.
#[test]
fn repeated_probe_trips_with_h7_half_wrap_idle_gaps() {
    let oids = vec![5u32, 6, 7];
    let mut h = h7_harness(oids.clone());
    let axis: u8 = 2;

    let mut mcu: HashMap<u32, McuStepper> = HashMap::new();
    for &oid in &oids {
        mcu.insert(oid, McuStepper::new());
    }

    let idle_gap = (H7_FREQ * 6.3) as u64;
    let view_stride = (0.002 * H7_FREQ) as u64;
    let tick_step = (H7_FREQ * 0.010) as u64;

    let mut position_mm = 0.0f64;
    let mut now: u64 = (H7_FREQ * 2.0) as u64;
    h.now.store(now, Ordering::Relaxed);

    let gradual_ticks = |h: &mut Harness, now: &mut u64, count: usize| {
        for _ in 0..count {
            *now += tick_step;
            h.now.store(*now, Ordering::Relaxed);
            h.endpoint.tick().unwrap();
            h.ack_sent_barriers_result().unwrap();
        }
    };

    let verify = |h: &mut Harness, mcu: &mut HashMap<u32, McuStepper>, now: u64, label: &str| {
        let frames: Vec<StepFrame> = std::mem::take(&mut h.sent.lock_ok());
        verify_mcu_agreement(&frames, &h.endpoint.lanes, mcu, now as u32)
            .unwrap_or_else(|e| panic!("{label}: {e}"));
    };

    for probe in 0..3usize {
        // ---- Motion volley, paced far enough that the reanchor cut
        //      lands inside the already-sent region → begin_cut.
        let volley_start = now + (H7_FREQ * 0.01) as u64;
        h.endpoint.mark_reanchor(axis, volley_start, Some(H7_FREQ));
        let volley_views = 50;
        let volley = h7_ramp(volley_start, volley_views, position_mm, 1.0);
        h.endpoint
            .send_frames(MCU_ID, &[frame_for_axis(axis, volley)])
            .unwrap_or_else(|e| panic!("probe {probe} volley: {e}"));

        gradual_ticks(&mut h, &mut now, 15);

        // ---- Reanchor inside the sent region → begin_cut fires.
        let cut_at = volley_start + view_stride * 16;
        assert!(
            h.endpoint.lanes[0]
                .last_sent_boundary
                .is_some_and(|b| cut_at <= b),
            "probe {probe}: cut_at must be inside the sent region"
        );

        h.endpoint.mark_reanchor(axis, cut_at, Some(H7_FREQ));

        // The barrier sits at the end of the sent region and the mcu reports
        // having executed all of it, so the motor physically rests where the
        // shim has walked it: the resume signal starts there.
        let pos_at_cut = h.endpoint.shim.commanded_position(0);
        let resume = h7_ramp(cut_at, 12, pos_at_cut, -1.0);
        h.endpoint
            .send_frames(MCU_ID, &[frame_for_axis(axis, resume)])
            .unwrap_or_else(|e| panic!("probe {probe} resume send: {e}"));

        assert!(
            h.endpoint.lanes.iter().any(|l| l.pending_cut.is_some()),
            "probe {probe}: begin_cut must have fired"
        );

        // ---- KEY: let >half-wrap of time elapse while the cut is
        //      pending, making resume_clock stale.
        verify(&mut h, &mut mcu, now, &format!("probe {probe} pre-idle"));

        now += idle_gap;
        h.now.store(now, Ordering::Relaxed);

        // Complete the cut. complete_cut uses cut.resume_clock as the
        // `now` for queue_step_volley, but the clock has wrapped.
        let cuts: Vec<usize> = (0..h.endpoint.lanes.len())
            .filter(|&motor| h.endpoint.lanes[motor].pending_cut.is_some())
            .collect();
        for &motor in &cuts {
            let expected = h.endpoint.lanes[motor]
                .pending_cut
                .as_ref()
                .expect("the cut was just observed")
                .expected_count;
            h.auto_query.store(false, Ordering::Relaxed);
            h.query_count.store(expected, Ordering::Relaxed);
        }
        h.ack_sent_barriers_result()
            .unwrap_or_else(|e| panic!("probe {probe} cut ack: {e}"));

        gradual_ticks(&mut h, &mut now, 30);
        verify(&mut h, &mut mcu, now, &format!("probe {probe} post-cut"));

        // The resume lay inside the region the mcu had already executed, so
        // the shim dropped it; the next volley starts where the motor is.
        position_mm = h.endpoint.shim.commanded_position(0);
    }
}
