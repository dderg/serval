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
        ),
        output: output_tx,
        links: Arc::default(),
        timeline: crate::worker::tests::idle_timeline(Arc::default()),
        undrained_since: Some(Instant::now()),
        worst_drain_s: 0.0,
        busy_runway: BusyRunwayWatch::default(),
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

    ingress.drain_and_fence();
    let widened = ingress.reserve_secs();
    assert!(widened >= slow.as_secs_f64() * DRAIN_RESERVE_SAFETY);
    ingress.drain_and_fence();
    assert!(ingress.reserve_secs() >= widened);

    ingress
        .timeline
        .frontier
        .advance_to(Instant::now() + Duration::from_millis(250));
    ingress.undrained_since = Some(Instant::now() - Duration::from_secs(1));
    assert!(ingress.drain_or_runway().is_none());
    downstream.join().unwrap();
}

fn drains_while_observing(runways_s: &[f64], reserve_s: f64) -> Vec<usize> {
    let mut watch = BusyRunwayWatch::default();
    runways_s
        .iter()
        .enumerate()
        .filter(|&(_, &runway_s)| watch.observe(runway_s, reserve_s))
        .map(|(index, _)| index)
        .collect()
}

#[test]
fn a_busy_stream_gaining_runway_under_a_widened_reserve_never_brakes() {
    let reserve_s = 2.4;
    let sawtooth_growth: Vec<f64> = (0..200)
        .map(|step| 0.01 * f64::from(step) - if step % 4 == 0 { 0.0 } else { 0.05 })
        .map(|runway_s: f64| runway_s.max(0.0))
        .collect();
    assert_eq!(
        drains_while_observing(&sawtooth_growth, reserve_s),
        Vec::<usize>::new()
    );
}

#[test]
fn a_busy_stream_that_never_built_runway_has_nothing_to_brake() {
    assert_eq!(drains_while_observing(&[0.0; 50], 0.5), Vec::<usize>::new());
}

#[test]
fn a_busy_stream_brakes_once_it_loses_half_the_reserve_below_its_best_runway() {
    let reserve_s = 1.0;
    let falling = [3.0, 0.9, 0.8, 0.7, 0.5, 0.41, 0.39, 0.3];
    assert_eq!(drains_while_observing(&falling, reserve_s), vec![6, 7]);
}

#[test]
fn regaining_the_reserve_forgets_the_runway_seen_under_it() {
    let reserve_s = 1.0;
    let recovering = [0.9, 1.2, 0.6, 0.45, 0.39];
    assert_eq!(
        drains_while_observing(&recovering, reserve_s),
        Vec::<usize>::new()
    );
}
