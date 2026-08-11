use super::*;
use crate::mcu_config::{McuAxisConfig, SteppingMode};
use runtime::piece_ring::PieceEntry;
use std::sync::atomic::AtomicU64;

const MCU_ID: u32 = 3;
const OID: u32 = 7;
const CYCLES_PER_SECOND: f64 = 1_000_000.0;
const BUDGET: u32 = 4;

struct Harness {
    endpoint: StepcompressEndpoint,
    now: Arc<AtomicU64>,
    sent: Arc<Mutex<Vec<StepFrame>>>,
    barriers: Arc<Mutex<Vec<(u32, u32)>>>,
    seeds: Arc<Mutex<Vec<(u32, i64)>>>,
    heartbeats: crossbeam_channel::Receiver<PumpMsg>,
    bursts: Arc<Mutex<Vec<usize>>>,
    fail_sends: Arc<AtomicBool>,
}

fn motor_cfg() -> MotorConfig {
    MotorConfig {
        oid: OID,
        microstep_distance: 0.01,
        invert_dir: false,
        max_steps_per_sample: 16,
        sample_rate_hz: 10_000.0,
        cycles_per_second: CYCLES_PER_SECOND,
        min_rearm_cycles: 0,
    }
}

fn harness(budget: u32) -> Harness {
    harness_on_axis(budget, 0)
}

fn harness_on_axis(budget: u32, axis: usize) -> Harness {
    let now = Arc::new(AtomicU64::new(0));
    let now_for_clock = Arc::clone(&now);
    let clock_of: ClockSource =
        Arc::new(move |_| Some((now_for_clock.load(Ordering::Relaxed), CYCLES_PER_SECOND)));
    let sent = Arc::new(Mutex::new(Vec::new()));
    let fail_sends = Arc::new(AtomicBool::new(false));
    let sent_for_egress = Arc::clone(&sent);
    let barriers = Arc::new(Mutex::new(Vec::new()));
    let barriers_for_egress = Arc::clone(&barriers);
    let seeds = Arc::new(Mutex::new(Vec::new()));
    let seeds_for_egress = Arc::clone(&seeds);
    let fail_for_egress = Arc::clone(&fail_sends);
    let bursts = Arc::new(Mutex::new(Vec::new()));
    let bursts_for_egress = Arc::clone(&bursts);
    let egress: FrameEgress =
        Arc::new(move |frames: &[(&'static str, Vec<(String, ArgValue)>)]| {
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
    let endpoint = StepcompressEndpoint::new(
        MCU_ID,
        StepShim::new(vec![motor_cfg()], SHIM_RING_DEPTH),
        vec![axis],
        vec![OID],
        egress,
        tx,
        clock_of,
        budget,
    );
    Harness {
        endpoint,
        now,
        sent,
        barriers,
        seeds,
        heartbeats: rx,
        bursts,
        fail_sends,
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

    fn latest_retired(&self) -> Option<Vec<u32>> {
        let mut last = None;
        while let Ok(PumpMsg::Heartbeat(hb)) = self.heartbeats.try_recv() {
            last = Some(hb.retired_counts);
        }
        last
    }

    fn ack_sent_barriers(&mut self) {
        let issued: Vec<(u32, u32)> = std::mem::take(&mut self.barriers.lock_ok());
        for (oid, seq) in issued {
            self.endpoint.on_barrier_ack(oid, seq).unwrap();
        }
    }
}

/// One piece per call, chained so the shim's contiguity check passes.
fn piece(start_time: u64, from_mm: f32, to_mm: f32, duration: f32) -> PieceEntry {
    let mut entry = PieceEntry::zeroed();
    entry.start_time = start_time;
    entry.duration = duration;
    entry.coeff_count = 2;
    entry.coeffs[0] = 0.5 * (from_mm + to_mm);
    entry.coeffs[1] = 0.5 * (to_mm - from_mm);
    entry
}

fn axis_frame(pieces: Vec<PieceEntry>) -> AxisFrame {
    frame_for_axis(0, pieces)
}

fn frame_for_axis(axis: u8, pieces: Vec<PieceEntry>) -> AxisFrame {
    AxisFrame {
        axis,
        pieces,
        start_slot: 0,
        new_head: 0,
        room: SHIM_RING_DEPTH,
        guard_recorded_ns: 0,
        guard_mcu_clock: 0,
    }
}

/// Pieces whose speed changes every piece, so `compress` cannot merge them
/// into one long move and the endpoint really has a queue of moves to pace.
fn ramp(start_time: u64, count: usize) -> Vec<PieceEntry> {
    ramp_from(start_time, count, 0.0)
}

fn ramp_from(start_time: u64, count: usize, start_mm: f32) -> Vec<PieceEntry> {
    let dur = 0.002_f32;
    let span = (f64::from(dur) * CYCLES_PER_SECOND) as u64;
    let mut at = start_mm;
    (0..count)
        .map(|i| {
            let from = at;
            at += 0.05 * (1 + (i % 4)) as f32;
            piece(start_time + span * i as u64, from, at, dur)
        })
        .collect()
}

/// A ramp whose pieces are long enough that a few in-flight moves buffer more
/// mcu time than the budget tests advance the clock per tick. `ramp`'s 2 ms
/// pieces cannot: four slots hold 8 ms of motion, so a 10 ms tick leaves the
/// backlog head behind the mcu clock — a pipe no host could deliver through.
fn paceable_ramp(start_time: u64, count: usize) -> Vec<PieceEntry> {
    let dur = 0.02_f32;
    let span = (f64::from(dur) * CYCLES_PER_SECOND) as u64;
    let mut at = 0.0_f32;
    (0..count)
        .map(|i| {
            let from = at;
            at += 0.05 * (1 + (i % 4)) as f32;
            piece(start_time + span * i as u64, from, at, dur)
        })
        .collect()
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
fn retirement_only_counts_fully_sent_pieces_and_never_regresses() {
    let mut h = harness(BUDGET);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(paceable_ramp(2_000, 12))])
        .unwrap();

    let shim_retired = h.endpoint.shim.retired_counts();
    let published = h.latest_retired().expect("a heartbeat is always posted");
    assert!(
        published[0] < shim_retired[0],
        "retirement must lag the shim while frames are still unsent ({published:?} vs \
         {shim_retired:?})"
    );

    let mut last = published[0];
    for step in 1..=40u64 {
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
        h.endpoint.shim.retired_counts()[0],
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
            frame: Outbound::Step(StepFrame::QueueStep {
                oid: OID,
                interval: 10,
                count: 1,
                add: 0,
            }),
            start_clock: u64::MAX,
            end_clock: u64::MAX,
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
        frame: Outbound::Step(StepFrame::QueueStep {
            oid: OID,
            interval: 10,
            count: 1,
            add: 0,
        }),
        start_clock,
        end_clock: start_clock + 10,
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
fn a_dead_egress_surfaces_instead_of_dropping_frames() {
    let mut h = harness(BUDGET);
    h.now.store(1_000, Ordering::Relaxed);
    h.fail_sends.store(true, Ordering::Relaxed);
    let err = h
        .endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 4))])
        .unwrap_err();
    assert!(matches!(err, SendError::Transient(_)), "{err:?}");
    assert_eq!(h.sent_moves(), 0);
    assert!(!h.endpoint.backlog.is_empty());
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
fn a_marked_fresh_epoch_may_start_before_the_queued_stream_ends() {
    let mut h = harness(1024);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 40))])
        .unwrap();

    let gap = h
        .endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp_from(81_834, 8, 5.0))])
        .expect_err("an unmarked overlap is still a loud PieceGap");
    assert!(format!("{gap:?}").contains("PieceGap"), "{gap:?}");

    h.endpoint.mark_reanchor(0, 81_834, Some(CYCLES_PER_SECOND));
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp_from(81_834, 8, 5.0))])
        .expect("a marked fresh epoch may start at any clock");
    assert!(
        h.sent
            .lock_ok()
            .iter()
            .any(|f| matches!(f, StepFrame::ResetStepClock { .. })),
        "the new epoch must re-anchor the mcu step clock"
    );
}

#[test]
fn a_bundle_spanning_the_epoch_boundary_is_cut_at_the_marked_piece() {
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
    // stalls) mark two seam gaps before any of their pieces reach the
    // endpoint. Both must survive queued — a single-slot mark would drop
    // the first seam and die on its PieceGap.
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
        !h.endpoint.pending_seams.contains_key(&0),
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
    assert!(format!("{err:?}").contains("PieceGap"), "{err:?}");
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
        .expect("contiguous pieces still flow with an unmatched mark outstanding");
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
fn a_position_seed_of_the_wrong_width_is_fatal() {
    let mut h = harness(BUDGET);
    let err = h.endpoint.reset_position(&[1, 2]).unwrap_err();
    match err {
        SendError::Fatal(msg) => assert!(msg.contains("configured axes"), "{msg}"),
        other => panic!("expected Fatal, got {other:?}"),
    }
}

fn stepcompress_cfg(mode: SteppingMode, move_queue_slots: u32) -> McuAxisConfig {
    McuAxisConfig {
        mcu_id: MCU_ID,
        axes: vec![0],
        kinematics: 0,
        caps: Default::default(),
        max_motor_velocity: vec![100.0],
        ethercat: false,
        stepping_mode: mode,
        microstep_distance: vec![0.01],
        invert_dir: vec![false],
        stepper_oids: vec![OID],
        stepcompress_sample_rate: 10_000.0,
        move_queue_slots,
        step_pulse_seconds: vec![2e-6],
    }
}

#[test]
fn a_move_queue_too_small_for_the_reserve_is_a_build_error() {
    let (tx, _rx) = crossbeam_channel::unbounded();
    let clock_of: ClockSource = Arc::new(|_| Some((0, CYCLES_PER_SECOND)));
    let err = match build_endpoint(
        &stepcompress_cfg(SteppingMode::Stepcompress, MOVE_SLOT_RESERVE),
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
fn piece_mode_mcus_never_reach_endpoint_construction() {
    let cfgs = [
        stepcompress_cfg(SteppingMode::Piece, 0),
        stepcompress_cfg(SteppingMode::Stepcompress, 128),
    ];
    let built: Vec<u32> = cfgs
        .iter()
        .filter(|c| c.stepping_mode == SteppingMode::Stepcompress)
        .map(|c| c.move_queue_slots)
        .collect();
    assert_eq!(built, vec![128]);

    let (tx, _rx) = crossbeam_channel::unbounded();
    let clock_of: ClockSource = Arc::new(|_| Some((0, CYCLES_PER_SECOND)));
    let endpoint = build_endpoint(&cfgs[1], Weak::new(), tx, CYCLES_PER_SECOND, clock_of).unwrap();
    assert_eq!(endpoint.budget, 128 - MOVE_SLOT_RESERVE);
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
    let pieces = ramp(2_000, 10);
    let last_end = 2_000 + 2_000 * 10;
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(pieces)])
        .unwrap();

    let mut now = 1_000_u64;
    while now < last_end + 100_000 {
        now += 10_000;
        h.now.store(now, Ordering::Relaxed);
        h.endpoint.tick().unwrap();
        h.ack_sent_barriers();
    }

    assert_eq!(
        h.latest_retired().expect("ticks post heartbeats"),
        vec![10],
        "every pushed piece must retire once the clock passes the stream end"
    );
    assert_eq!(h.endpoint.shim.queued_pieces(), 0);
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
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 8))])
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
        .send_frames(MCU_ID, &[frame_for_axis(EXTRUDER_AXIS, ramp(2_000, 8))])
        .unwrap();
    h.now.store(10_000_000, Ordering::Relaxed);
    h.endpoint.tick().unwrap();
    h.ack_sent_barriers();

    let heartbeat = h.latest_retired().expect("a heartbeat is always posted");
    assert_eq!(
        heartbeat.len(),
        usize::from(EXTRUDER_AXIS) + 1,
        "the heartbeat must be indexed by axis, so it must reach axis {EXTRUDER_AXIS}"
    );
    assert!(
        heartbeat[usize::from(EXTRUDER_AXIS)] > 0,
        "the pump keys its rings by axis; motor 0's retirements must land on \
         axis {EXTRUDER_AXIS} or that lane's ring never drains: {heartbeat:?}"
    );
    assert_eq!(
        &heartbeat[..usize::from(EXTRUDER_AXIS)],
        &[0, 0, 0],
        "lanes this mcu does not own must not be credited"
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
        .send_frames(MCU_ID, &[frame_for_axis(EXTRUDER_AXIS, ramp(2_000, 8))])
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
fn a_cohort_is_not_retired_until_its_barrier_is_acked() {
    let mut h = harness(1024);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 8))])
        .unwrap();

    h.now.store(10_000_000, Ordering::Relaxed);
    h.endpoint.tick().unwrap();
    assert!(
        h.endpoint.backlog.is_empty(),
        "every frame including the barrier must be on the wire"
    );
    assert_eq!(
        h.endpoint.published_counts(),
        vec![0],
        "a clock watermark far past the stream must not retire an unacked cohort"
    );

    let issued = h.barriers.lock_ok().clone();
    assert_eq!(issued.first().map(|&(oid, seq)| (oid, seq)), Some((OID, 0)));

    h.ack_sent_barriers();
    assert!(
        h.endpoint.published_counts()[0] > 0,
        "the ack must release the cohort"
    );
}

#[test]
fn a_barrier_ack_that_skips_ahead_is_fatal() {
    let mut h = harness(1024);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 8))])
        .unwrap();
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp_from(18_000, 8, 1.0))])
        .unwrap();
    h.now.store(10_000_000, Ordering::Relaxed);
    h.endpoint.tick().unwrap();
    assert!(h.barriers.lock_ok().len() >= 2, "the run issues barriers");

    let err = h
        .endpoint
        .on_barrier_ack(OID, 1)
        .expect_err("a skipped barrier leaves a cohort unaccounted for");
    assert!(format!("{err:?}").contains("out of order"), "{err:?}");
}

#[test]
fn a_barrier_ack_below_the_high_water_mark_is_ignored() {
    let mut h = harness(1024);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 8))])
        .unwrap();
    h.now.store(10_000_000, Ordering::Relaxed);
    h.endpoint.tick().unwrap();
    assert!(!h.barriers.lock_ok().is_empty(), "the run issues barriers");

    h.endpoint.on_barrier_ack(OID, 0).unwrap();
    h.endpoint
        .on_barrier_ack(OID, 0)
        .expect("a replayed ack is already covered, not a protocol break");
}

#[test]
fn a_barrier_ack_ahead_of_what_was_issued_is_fatal() {
    let mut h = harness(1024);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 8))])
        .unwrap();
    let issued = h.barriers.lock_ok().len() as u32;
    let err = h
        .endpoint
        .on_barrier_ack(OID, issued + 5)
        .expect_err("an ack for a barrier never issued must be fatal");
    assert!(format!("{err:?}").contains("ahead of"), "{err:?}");

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
fn the_seam_basis_is_the_slope_the_shim_will_hold_when_the_pieces_land() {
    let mut h = harness(1024);
    let basis = h.endpoint.seam_basis(0).expect("axis 0 is configured");
    assert_eq!(basis.freq, CYCLES_PER_SECOND);
    assert_eq!(
        basis.skew_budget_cycles,
        step_shim::MAX_SEAM_SKEW_CYCLES / 2,
        "a duration rewrite may spend at most half the seam tolerance, so the shim's check \
         still has room to catch a real break"
    );

    const EPOCH_FREQ: f64 = CYCLES_PER_SECOND * 1.000_002;
    h.endpoint.mark_reanchor(0, 9_000, Some(EPOCH_FREQ));
    assert_eq!(
        h.endpoint.seam_basis(0).unwrap().freq,
        EPOCH_FREQ,
        "pieces staged after a mark belong to the incoming epoch and must be merged on its slope"
    );

    assert!(
        h.endpoint.seam_basis(1).is_none(),
        "an axis this endpoint does not carry has no seam here"
    );
}

#[test]
fn a_cut_moves_the_seam_basis_onto_the_adopted_epoch_slope() {
    const EPOCH_FREQ: f64 = CYCLES_PER_SECOND * 1.000_002;
    let mut h = harness(1024);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 8))])
        .unwrap();
    h.endpoint.mark_reanchor(0, 50_000, Some(EPOCH_FREQ));
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp_from(50_000, 4, 1.0))])
        .unwrap();
    assert_eq!(
        h.endpoint.seam_basis(0).unwrap().freq,
        EPOCH_FREQ,
        "the shim adopted the epoch slope at the cut; the basis reports it without the mark"
    );
}

/// A lane that holds through a long print (Z parked while XY runs) is
/// unreachable from the clock the mcu stepper is anchored on when it finally
/// moves. The endpoint must re-anchor it mid-stream and keep pacing,
/// retiring and barrier-acking exactly as before the re-anchor.
#[test]
fn a_lane_parked_past_the_encoder_window_re_anchors_mid_stream() {
    #[allow(clippy::cast_possible_truncation)]
    let cps = CYCLES_PER_SECOND as f32;
    let hold_secs = (step_shim::compress::CLOCK_DIFF_MAX as f32 / cps).ceil() + 60.0;
    let mut h = harness(1024);
    h.now.store(1_000, Ordering::Relaxed);

    let lift = piece(2_000, 0.0, 1.0, 0.010);
    let hold = piece(lift.end_time(cps), 1.0, 1.0, hold_secs);
    let resume = piece(hold.end_time(cps), 1.0, 2.0, 0.010);
    let end = resume.end_time(cps);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(vec![lift, hold, resume])])
        .unwrap();

    for now in [lift.end_time(cps), hold.end_time(cps), end + 1_000_000] {
        h.now.store(now, Ordering::Relaxed);
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
    let sample_period = (CYCLES_PER_SECOND / 10_000.0) as u64;
    assert!(
        resets[1] >= resume.start_time && resets[1] < resume.start_time + 2 * sample_period,
        "the re-anchor must land at the first step of the resuming move, not at \
         some stale cursor: anchor {}, move starts {}",
        resets[1],
        resume.start_time
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
