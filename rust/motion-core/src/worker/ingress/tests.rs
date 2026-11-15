use super::*;

fn ingress(output_tx: crossbeam_channel::Sender<motion_pipeline::TrajectoryItem>) -> Ingress {
    let (pump, _control_rx) = crossbeam_channel::unbounded();
    let frontier: Arc<crate::worker::CommittedFrontier> = Arc::default();
    let links: Arc<WorkerLinks> = Arc::default();
    let pacer = Pacer::default();
    let intake = Arc::new(IntakeGate::new(
        Arc::clone(&frontier),
        Arc::clone(&links),
        pacer.reserve_secs(),
    ));
    Ingress {
        config: crate::worker::tests::cfg(),
        odometer: vec![0.0; 4],
        t_next: 0.0,
        pipeline: motion_pipeline::Pipeline::new(
            crate::worker::tests::cfg(),
            trajectory::AxisChainSet::default(),
            vec![0.0; 4],
            0.0,
        )
        .into_staged(move |item| output_tx.send(item).is_ok()),
        links,
        timeline: crate::worker::tests::idle_timeline(frontier),
        pacer,
        throttle: None,
        last_line: 0,
        moves_sent: 0,
        dispatch_record: None,
        pump,
        intake,
    }
}

#[test]
fn a_submitter_the_intake_refused_is_woken_once_the_playhead_makes_room() {
    let (output_tx, _output) = crossbeam_channel::unbounded();
    let ingress = ingress(output_tx);
    let excess = Duration::from_millis(100);
    ingress
        .timeline
        .frontier
        .advance_to(Instant::now() + Duration::from_secs_f64(ingress.intake.cap_secs()) + excess);
    assert!(!ingress.intake.admits());
    ingress.links.wakeup.arm();
    let (_submitter, rx) = crossbeam_channel::bounded::<StreamMsg>(1);

    let received = ingress.recv_reopening_intake(&rx, Some(3 * excess));

    assert!(matches!(
        received,
        Err(crossbeam_channel::RecvTimeoutError::Timeout)
    ));
    assert!(
        format!("{:?}", ingress.links.wakeup).contains("armed: false"),
        "the playhead reopened the intake {excess:?} into a {:?} wait without waking \
         the parked submitter",
        3 * excess
    );
}

/// The pump takes trajectory only while its span backlog has room, and
/// planning goes on into the stage inboxes meanwhile. In the simulator the
/// planning side outran the pump at print start: 16 s of long moves waited
/// there, uncounted, and PAUSE took 17 s to stop the axes.
#[test]
fn motion_planned_ahead_of_the_pump_counts_against_the_cap() {
    const STROKE_MM: f64 = 40.0;
    const MAX_STROKES: u32 = 64;
    let (output_tx, _output) = crossbeam_channel::unbounded();
    let mut ingress = ingress(output_tx);
    let mut at = [0.0; 3];
    for line_no in 1..=MAX_STROKES {
        if ingress.pipeline.planned_through_s() >= ingress.intake.cap_secs() {
            break;
        }
        let end = [if at[0] > 0.0 { 0.0 } else { STROKE_MM }, 0.0, 0.0];
        ingress.handle_move(crate::worker::tests::line(line_no, at, end));
        at = end;
    }

    let planned_s = ingress.pipeline.planned_through_s();
    assert!(planned_s >= ingress.intake.cap_secs());
    assert!(
        !ingress.intake.admits(),
        "the intake admits with {planned_s:.2}s planned ahead of a pump that took none of it"
    );
}

fn answer_barrier(
    output: &crossbeam_channel::Receiver<motion_pipeline::TrajectoryItem>,
    before_answer: impl FnOnce(),
) {
    while let Ok(item) = output.recv() {
        if let motion_pipeline::TrajectoryItem::Control(Control::Dispatch(
            DispatchCommand::Barrier(reply),
        )) = item
        {
            before_answer();
            reply
                .send(BarrierAck {
                    dispatched_through: None,
                    result: Ok(()),
                })
                .unwrap();
            return;
        }
    }
}

#[test]
fn a_slow_drain_reserves_runway_for_repeating_it_even_after_a_faster_drain() {
    let (output_tx, output) = crossbeam_channel::unbounded();
    let mut ingress = ingress(output_tx);
    let slow = Duration::from_millis(300);
    let downstream = std::thread::spawn(move || {
        for delay in [slow, Duration::ZERO, Duration::ZERO] {
            answer_barrier(&output, || std::thread::sleep(delay));
        }
    });

    ingress
        .timeline
        .frontier
        .advance_to(Instant::now() + Duration::from_secs(1));
    ingress.pacer.note_move(Instant::now());
    ingress.drain_and_fence();
    let widened = ingress.pacer.reserve_secs();
    assert!(widened >= slow.as_secs_f64() * pacer::DRAIN_RESERVE_SAFETY);
    ingress.drain_and_fence();
    assert!(ingress.pacer.reserve_secs() >= widened);

    ingress.timeline.frontier.clear();
    ingress
        .timeline
        .frontier
        .advance_to(Instant::now() + Duration::from_millis(250));
    ingress
        .pacer
        .note_move(Instant::now() - Duration::from_secs(1));
    assert!(ingress.drain_or_runway(InboxFoundEmpty(())).is_none());
    downstream.join().unwrap();
}

/// A pump full of committed motion admits the drain's trajectory only as
/// fast as the playhead frees room, so the drain takes as long as the
/// runway it waits behind, while the committed end keeps the same lead the
/// whole time. On the bench that wait widened the reserve to twice the
/// runway, and every brake it then ordered resumed a reserve ahead: 3 to
/// 94 s rests in one print.
#[test]
fn a_drain_the_playhead_paces_costs_no_runway() {
    let (output_tx, output) = crossbeam_channel::unbounded();
    let mut ingress = ingress(output_tx);
    let frontier = Arc::clone(&ingress.timeline.frontier);
    let lead = Duration::from_secs(1);
    frontier.advance_to(Instant::now() + lead);
    let unmeasured_reserve_s = ingress.pacer.reserve_secs();
    let downstream = std::thread::spawn(move || {
        answer_barrier(&output, || {
            let behind_the_playhead = Instant::now() + Duration::from_millis(300);
            while Instant::now() < behind_the_playhead {
                frontier.advance_to(Instant::now() + lead);
                std::thread::sleep(Duration::from_millis(5));
            }
        });
    });

    ingress.pacer.note_move(Instant::now());
    ingress.drain_and_fence();
    downstream.join().unwrap();

    assert_eq!(
        ingress.pacer.reserve_secs(),
        unmeasured_reserve_s,
        "a drain that waited on the playhead, its committed end a steady {lead:?} \
         ahead, widened the reserve"
    );
}

/// An emergency stop halts the dispatcher, which throws the committed
/// timeline away: on the bench the pacer counted the whole 16.7 s runway an
/// M112 discarded as lost. A discarded timeline was never played, so it
/// costs the reserve nothing and must not trip the reserve cap.
#[test]
fn a_halt_during_a_drain_costs_no_runway() {
    let (output_tx, output) = crossbeam_channel::unbounded();
    let mut ingress = ingress(output_tx);
    let frontier = Arc::clone(&ingress.timeline.frontier);
    frontier.advance_to(Instant::now() + Duration::from_secs(10));
    let unmeasured_reserve_s = ingress.pacer.reserve_secs();
    let downstream = std::thread::spawn(move || {
        answer_barrier(&output, || frontier.clear());
    });

    ingress.pacer.note_move(Instant::now());
    ingress.drain_and_fence();
    downstream.join().unwrap();

    assert_eq!(ingress.pacer.reserve_secs(), unmeasured_reserve_s);
}
