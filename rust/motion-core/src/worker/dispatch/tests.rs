//! Every way a stream at rest sets off (#983), and every way its hold ends
//! without one.

use super::*;
use trajectory::ContinuousAxis;

const RESERVE_S: f64 = 0.29;
const REST_HOLD_S: f64 = 0.0084;

/// Sets off what reaches it at rest, as the anchor does: a dispatch puts the
/// stream in motion, a park brings it to rest.
#[derive(Default)]
struct Recorder {
    dispatched: Vec<(f64, f64)>,
    parks: usize,
    in_motion: bool,
    starts_in: Duration,
}

impl SegmentSink for Recorder {
    fn dispatch(&mut self, seg: &ContinuousSegment) -> Result<(), DispatchError> {
        self.dispatched.push((seg.t_start, seg.t_end));
        self.in_motion = true;
        Ok(())
    }
    fn dispatch_nudge(
        &mut self,
        mcu_id: u32,
        axis: u8,
        _motor_mask: u8,
        _profile: &NudgeProfile,
    ) -> Result<(), DispatchError> {
        panic!("no nudge is sent here: mcu {mcu_id} axis {axis}")
    }
    fn mark_parked(&mut self) {
        self.parks += 1;
        self.in_motion = false;
    }
    fn follow_stream_to(&mut self, _stream_t: f64) {}
    fn set_off_in(&self, _seg: &ContinuousSegment) -> Option<Duration> {
        (!self.in_motion).then_some(self.starts_in)
    }
}

fn segment(t_start: f64, t_end: f64) -> ContinuousSegment {
    let hold = |position| ContinuousAxis::Hold {
        position,
        t_start,
        t_end,
    };
    ContinuousSegment {
        axes: Arc::from([hold(0.0), hold(0.0), hold(0.0)]),
        followers: Arc::from([]),
        spatial_path: false,
        t_start,
        t_end,
        motor_mask: 0,
        source_line: 1,
        rest_at_end: false,
    }
}

fn dispatcher() -> (Dispatcher, Arc<WorkerLinks>) {
    let links: Arc<WorkerLinks> = Arc::default();
    links.publish_reserve(RESERVE_S);
    (Dispatcher::new(Arc::clone(&links), Arc::default()), links)
}

/// The lowerer's rest hold, then the first move behind it.
fn feed_rest_hold_and_move(
    dispatcher: &mut Dispatcher,
    sink: &mut Recorder,
    move_s: f64,
) -> (f64, f64) {
    dispatcher.feed(TrajectoryItem::Seg(segment(0.0, REST_HOLD_S)), sink);
    let first_move = (REST_HOLD_S, REST_HOLD_S + move_s);
    dispatcher.feed(
        TrajectoryItem::Seg(segment(first_move.0, first_move.1)),
        sink,
    );
    first_move
}

#[test]
fn a_stream_at_rest_sets_off_once_its_reserve_is_committed() {
    let (mut dispatcher, links) = dispatcher();
    let mut sink = Recorder::default();
    feed_rest_hold_and_move(&mut dispatcher, &mut sink, 0.1);
    assert!(
        sink.dispatched.is_empty(),
        "set off on {:?}",
        sink.dispatched
    );
    let hold = links.set_off_hold().expect("the set-off is held");
    assert!((hold.held_s - (REST_HOLD_S + 0.1)).abs() < 1e-12);

    dispatcher.feed(
        TrajectoryItem::Seg(segment(REST_HOLD_S + 0.1, 0.5)),
        &mut sink,
    );

    assert_eq!(
        sink.dispatched,
        vec![
            (0.0, REST_HOLD_S),
            (REST_HOLD_S, REST_HOLD_S + 0.1),
            (REST_HOLD_S + 0.1, 0.5)
        ]
    );
    assert_eq!(links.set_off_hold(), None);
}

#[test]
fn a_stream_in_motion_continues_as_its_segments_come() {
    let (mut dispatcher, _links) = dispatcher();
    let mut sink = Recorder {
        in_motion: true,
        ..Recorder::default()
    };
    dispatcher.feed(TrajectoryItem::Seg(segment(0.0, REST_HOLD_S)), &mut sink);
    assert_eq!(sink.dispatched, vec![(0.0, REST_HOLD_S)]);
}

/// A jog: everything it is ends at rest before the reserve, so it sets off
/// the moment the drain parks it, without waiting on the reserve.
#[test]
fn a_stream_shorter_than_its_reserve_sets_off_as_it_parks() {
    let (mut dispatcher, links) = dispatcher();
    let mut sink = Recorder::default();
    let first_move = feed_rest_hold_and_move(&mut dispatcher, &mut sink, 0.05);

    dispatcher.feed(TrajectoryItem::Parked, &mut sink);

    assert_eq!(sink.dispatched, vec![(0.0, REST_HOLD_S), first_move]);
    assert_eq!(sink.parks, 1);
    assert_eq!(links.set_off_hold(), None);
}

#[test]
fn a_control_token_sets_off_what_it_follows_before_it_acts() {
    let (mut dispatcher, _links) = dispatcher();
    let mut sink = Recorder::default();
    let first_move = feed_rest_hold_and_move(&mut dispatcher, &mut sink, 0.05);
    let (ack_tx, ack_rx) = crossbeam_channel::bounded(1);

    dispatcher.feed(
        TrajectoryItem::Control(Control::Dispatch(DispatchCommand::Barrier(ack_tx))),
        &mut sink,
    );

    assert_eq!(sink.dispatched, vec![(0.0, REST_HOLD_S), first_move]);
    let ack = ack_rx.try_recv().expect("the barrier acknowledged");
    assert_eq!(ack.dispatched_through, Some(first_move.1));
}

#[test]
fn a_dwell_sets_off_what_it_follows_before_it_moves_the_stream_clock() {
    let (mut dispatcher, _links) = dispatcher();
    let mut sink = Recorder::default();
    let first_move = feed_rest_hold_and_move(&mut dispatcher, &mut sink, 0.05);

    dispatcher.feed(
        TrajectoryItem::Control(Control::Dwell { secs: 1.0 }),
        &mut sink,
    );

    assert_eq!(sink.dispatched, vec![(0.0, REST_HOLD_S), first_move]);
}

/// CANCEL_PRINT, an emergency stop and FIRMWARE_RESTART discard what the
/// pipeline holds: a stream held at rest never moves and stays parked.
#[test]
fn a_discard_drops_a_held_set_off_without_moving() {
    let (mut dispatcher, links) = dispatcher();
    let mut sink = Recorder::default();
    feed_rest_hold_and_move(&mut dispatcher, &mut sink, 0.05);
    links.discard.store(true, Ordering::Release);

    dispatcher.feed(
        TrajectoryItem::Control(Control::Reset { pos: vec![0.0; 3] }),
        &mut sink,
    );

    assert!(sink.dispatched.is_empty(), "moved: {:?}", sink.dispatched);
    assert!(!sink.in_motion);
    assert_eq!(links.set_off_hold(), None);
}

#[test]
fn a_halt_drops_a_held_set_off_without_moving() {
    let (mut dispatcher, links) = dispatcher();
    let mut sink = Recorder::default();
    feed_rest_hold_and_move(&mut dispatcher, &mut sink, 0.05);

    dispatcher.halt("endpoint fault");
    dispatcher.feed(TrajectoryItem::Parked, &mut sink);

    assert!(sink.dispatched.is_empty(), "moved: {:?}", sink.dispatched);
    assert_eq!(links.set_off_hold(), None);
}

#[test]
fn a_homing_move_sets_off_as_it_comes() {
    let (mut dispatcher, links) = dispatcher();
    let mut sink = Recorder::default();
    links.finite_homing_admission.store(true, Ordering::Release);

    dispatcher.feed(TrajectoryItem::Seg(segment(0.0, REST_HOLD_S)), &mut sink);

    assert_eq!(sink.dispatched, vec![(0.0, REST_HOLD_S)]);
    assert_eq!(links.set_off_hold(), None);
}

/// After a rest inside a print (M400, G4, a PAUSE and RESUME) the stream
/// sets off again under the same rule.
#[test]
fn a_stream_parked_mid_print_sets_off_again_only_on_its_reserve() {
    let (mut dispatcher, _links) = dispatcher();
    let mut sink = Recorder::default();
    dispatcher.feed(TrajectoryItem::Seg(segment(0.0, 1.0)), &mut sink);
    dispatcher.feed(TrajectoryItem::Parked, &mut sink);
    sink.dispatched.clear();

    dispatcher.feed(
        TrajectoryItem::Seg(segment(1.0, 1.0 + REST_HOLD_S)),
        &mut sink,
    );
    assert!(
        sink.dispatched.is_empty(),
        "set off on {:?}",
        sink.dispatched
    );
    dispatcher.feed(
        TrajectoryItem::Seg(segment(1.0 + REST_HOLD_S, 2.0)),
        &mut sink,
    );

    assert_eq!(sink.dispatched.len(), 2);
}

/// The intake bounds what stands ahead of the playhead. A held set-off
/// stands ahead of one not yet running: its wait to start and its motion
/// count, as they did once its first segment was anchored at once.
#[test]
fn a_held_set_off_counts_against_the_intake_cap() {
    const PRIMED_START: Duration = Duration::from_millis(540);
    let (mut dispatcher, links) = dispatcher();
    let intake = super::super::intake_gate::IntakeGate::new(Arc::default(), Arc::clone(&links));
    let inbox_ns = (intake.cap_secs() - 0.5) * 1e9;
    intake.enter(inbox_ns as u64);
    assert!(intake.admits(), "nothing ahead but the inbox");

    let mut sink = Recorder {
        starts_in: PRIMED_START,
        ..Recorder::default()
    };
    dispatcher.feed(TrajectoryItem::Seg(segment(0.0, REST_HOLD_S)), &mut sink);

    assert!(
        !intake.admits(),
        "a set-off {PRIMED_START:?} from starting held {REST_HOLD_S}s the intake did not count"
    );
}

/// A stream parked on a stop it has not reached sets off again from where
/// its committed runway ends: that wait is the runway itself, counted once.
#[test]
fn a_set_off_behind_a_playing_runway_counts_that_runway_once() {
    const PARKED_RUNWAY: Duration = Duration::from_millis(1000);
    let (mut dispatcher, links) = dispatcher();
    let frontier = Arc::new(super::super::CommittedFrontier::default());
    frontier.advance_to(Instant::now() + PARKED_RUNWAY);
    let intake = super::super::intake_gate::IntakeGate::new(frontier, Arc::clone(&links));
    let room_s = intake.cap_secs() - PARKED_RUNWAY.as_secs_f64() - REST_HOLD_S;
    intake.enter(((room_s - 0.1) * 1e9) as u64);

    let mut sink = Recorder {
        starts_in: PARKED_RUNWAY,
        ..Recorder::default()
    };
    dispatcher.feed(TrajectoryItem::Seg(segment(0.0, REST_HOLD_S)), &mut sink);

    assert!(
        intake.admits(),
        "the {PARKED_RUNWAY:?} runway a held set-off waits behind was counted twice"
    );
}
