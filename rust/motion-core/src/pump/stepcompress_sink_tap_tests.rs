use super::*;
use crate::pump::step_tap::{RecordKind, StepTap, TapRecord};
use crate::pump::step_tap_tests::{
    DecodedFrame, as_decoded, decode_frame, record_kind, record_oid,
};

const OID_B: u32 = OID + 1;

struct TapProbe {
    records: crossbeam_channel::Receiver<TapRecord>,
    accepted: Arc<Mutex<Vec<DecodedFrame>>>,
    seen: Vec<TapRecord>,
}

impl TapProbe {
    fn attach(h: &mut Harness) -> Self {
        let accepted = Arc::new(Mutex::new(Vec::new()));
        let log = Arc::clone(&accepted);
        let inner = Arc::clone(&h.endpoint.egress);
        h.endpoint.egress =
            Arc::new(move |frames: &[(&'static str, Vec<(String, ArgValue)>)]| {
                inner(frames)?;
                log.lock_ok().extend(
                    frames
                        .iter()
                        .filter(|(name, _)| *name != "kalico_wire_probe")
                        .map(|(name, args)| as_decoded(name, args)),
                );
                Ok(())
            });
        let (tx, rx) = crossbeam_channel::unbounded();
        let tap = StepTap::from_sender(tx, MCU_ID, Arc::clone(&h.endpoint.clock_of));
        h.endpoint.attach_tap(Arc::new(tap));
        Self {
            records: rx,
            accepted,
            seen: Vec::new(),
        }
    }

    fn records(&mut self) -> &[TapRecord] {
        self.seen.extend(self.records.try_iter());
        &self.seen
    }

    fn accepted_count(&self) -> usize {
        self.accepted.lock_ok().len()
    }

    fn halts(&mut self) -> Vec<u32> {
        self.records()
            .iter()
            .filter(|r| record_kind(r) == RecordKind::Halt as u8)
            .map(record_oid)
            .collect()
    }

    fn assert_replays_the_wire(&mut self) {
        let records = self.records().to_vec();
        let frames: Vec<DecodedFrame> = records
            .iter()
            .filter(|r| record_kind(r) != RecordKind::Halt as u8)
            .map(|r| decode_frame(r).expect("every non-halt record is a frame"))
            .collect();
        let accepted = self.accepted.lock_ok().clone();
        assert!(!accepted.is_empty(), "the scenario sent nothing");
        assert_eq!(frames, accepted);
    }

    fn sent(&self, name: &str) -> usize {
        self.accepted
            .lock_ok()
            .iter()
            .filter(|(sent, _)| sent == name)
            .count()
    }

    fn kinds_from(&mut self, start: usize) -> Vec<(u8, u32)> {
        self.records()[start..]
            .iter()
            .map(|r| (record_kind(r), record_oid(r)))
            .collect()
    }
}

#[test]
fn the_tap_replays_the_wire_through_sent_and_unsent_cuts() {
    let mut h = harness(1024);
    let mut tap = TapProbe::attach(&mut h);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 40))])
        .and_then(|()| h.endpoint.tick())
        .unwrap();
    h.endpoint.mark_reanchor(0, 81_834, Some(CYCLES_PER_SECOND));
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp_from(81_834, 8, 5.0))])
        .and_then(|()| h.endpoint.tick())
        .unwrap();
    h.endpoint
        .mark_reanchor(0, 500_000, Some(CYCLES_PER_SECOND));
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp_from(500_000, 8, 6.0))])
        .and_then(|()| h.endpoint.tick())
        .unwrap();
    let expected = h.endpoint.lanes[0]
        .pending_cut
        .as_ref()
        .expect("the sent cut awaits reconciliation")
        .expected_count;
    h.auto_query.store(false, Ordering::Relaxed);
    h.query_count.store(expected, Ordering::Relaxed);
    h.ack_sent_barriers_result().unwrap();
    for tick in 1..=100 {
        h.now.store(1_000 + tick * 10_000, Ordering::Relaxed);
        h.endpoint.tick().unwrap();
    }

    assert_eq!(
        h.query_calls.load(Ordering::Relaxed),
        1,
        "the sent cut reconciled"
    );
    assert!(h.endpoint.lanes.iter().all(|l| l.pending_cut.is_none()));
    assert!(tap.sent("reset_step_clock") >= 3, "one reset per epoch");
    assert!(tap.sent("queue_step") > 0);
    assert!(tap.sent("stepcompress_barrier") > 0);
    tap.assert_replays_the_wire();
    assert!(tap.halts().is_empty());
}

#[test]
fn reset_motor_position_records_a_halt_where_it_happened() {
    let mut h = harness(1024);
    let mut tap = TapProbe::attach(&mut h);
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(ramp(2_000, 8))])
        .and_then(|()| h.endpoint.tick())
        .unwrap();
    let before = tap.accepted_count();
    h.endpoint.reset_motor_position(0, 20).unwrap();
    assert_eq!(
        tap.kinds_from(before),
        [(RecordKind::Halt as u8, OID)],
        "the halt follows exactly the frames sent before it"
    );
    h.endpoint
        .send_frames(MCU_ID, &[axis_frame(linear_run(50_000, 0.2, 0.5, 5))])
        .and_then(|()| h.endpoint.tick())
        .unwrap();
    tap.assert_replays_the_wire();
    assert_eq!(tap.halts(), [OID]);
}

fn two_lane_harness() -> Harness {
    harness_axes(1024, vec![0, 1], vec![OID, OID_B])
}

fn send_on_both(h: &mut Harness) {
    h.now.store(1_000, Ordering::Relaxed);
    h.endpoint
        .send_frames(
            MCU_ID,
            &[
                frame_for_axis(0, ramp(2_000, 40)),
                frame_for_axis(1, ramp(2_000, 40)),
            ],
        )
        .and_then(|()| h.endpoint.tick())
        .unwrap();
}

#[test]
fn abort_outbound_records_one_halt_per_oid() {
    let mut h = two_lane_harness();
    let mut tap = TapProbe::attach(&mut h);
    send_on_both(&mut h);
    let before = tap.accepted_count();
    h.endpoint.abort_outbound();
    assert_eq!(
        tap.kinds_from(before),
        [
            (RecordKind::Halt as u8, OID),
            (RecordKind::Halt as u8, OID_B)
        ]
    );
    tap.assert_replays_the_wire();
}

#[test]
fn a_position_reset_records_its_halts_and_seeds() {
    let mut h = two_lane_harness();
    let mut tap = TapProbe::attach(&mut h);
    send_on_both(&mut h);
    let before = tap.accepted_count();
    h.endpoint.reset_position(&[0.0, 0.0]).unwrap();
    let halt = RecordKind::Halt as u8;
    let seed = RecordKind::SetPosition as u8;
    assert_eq!(
        tap.kinds_from(before),
        [
            (halt, OID),
            (halt, OID_B),
            (halt, OID),
            (seed, OID),
            (halt, OID_B),
            (seed, OID_B),
        ]
    );
    tap.assert_replays_the_wire();
}

#[test]
fn the_tap_replays_a_host_buzz() {
    let mut h = harness(1024);
    let mut tap = TapProbe::attach(&mut h);
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
    assert!(tap.sent("queue_step") > 0);
    tap.assert_replays_the_wire();
}
