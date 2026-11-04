use super::*;

#[test]
fn a_slow_drain_reserves_runway_for_repeating_it_even_after_a_faster_drain() {
    let (output_tx, output) = crossbeam_channel::unbounded();
    let (pump, _control_rx) = crossbeam_channel::unbounded();
    let mut ingress = Ingress {
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
        links: Arc::default(),
        timeline: crate::worker::tests::idle_timeline(Arc::default()),
        pacer: Pacer::default(),
        throttle: None,
        last_line: 0,
        pump,
    };
    let slow = Duration::from_millis(300);
    let downstream = std::thread::spawn(move || {
        for delay in [slow, Duration::ZERO, Duration::ZERO] {
            while let Ok(item) = output.recv() {
                if let motion_pipeline::TrajectoryItem::Control(Control::Dispatch(
                    DispatchCommand::Barrier(reply),
                )) = item
                {
                    std::thread::sleep(delay);
                    reply
                        .send(BarrierAck {
                            dispatched_through: None,
                            result: Ok(()),
                        })
                        .unwrap();
                    break;
                }
            }
        }
    });

    ingress.pacer.note_move(Instant::now());
    ingress.drain_and_fence();
    let widened = ingress.pacer.reserve_secs();
    assert!(widened >= slow.as_secs_f64() * pacer::DRAIN_RESERVE_SAFETY);
    ingress.drain_and_fence();
    assert!(ingress.pacer.reserve_secs() >= widened);

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
