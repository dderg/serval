use super::WorkerPool;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::time::{Duration, Instant};

fn panic_message(payload: &(dyn std::any::Any + Send)) -> &str {
    payload
        .downcast_ref::<&str>()
        .copied()
        .or_else(|| payload.downcast_ref::<String>().map(String::as_str))
        .expect("panic payload is a message")
}

#[test]
fn results_come_back_in_job_order_with_every_job_run_once() {
    let pool = WorkerPool::with_helpers(3);
    let runs = AtomicUsize::new(0);
    let squares = pool
        .map_indexed(
            4,
            1000,
            || (),
            |(), index| {
                runs.fetch_add(1, Ordering::Relaxed);
                index * index
            },
        )
        .expect("no job panics");
    assert_eq!(squares, (0..1000).map(|i| i * i).collect::<Vec<_>>());
    assert_eq!(runs.load(Ordering::Relaxed), 1000);
}

#[test]
fn each_participant_builds_its_state_once_and_only_when_it_takes_a_job() {
    let pool = WorkerPool::with_helpers(7);
    let states = AtomicUsize::new(0);
    let results = pool
        .map_indexed(
            8,
            2,
            || states.fetch_add(1, Ordering::Relaxed),
            |_, index| index,
        )
        .expect("no job panics");
    assert_eq!(results, vec![0, 1]);
    assert!((1..=2).contains(&states.load(Ordering::Relaxed)));
}

#[test]
fn a_job_error_is_returned_at_its_index() {
    let pool = WorkerPool::with_helpers(3);
    let results = pool
        .map_indexed(
            4,
            64,
            || (),
            |(), index| {
                if index % 9 == 5 {
                    Err(index)
                } else {
                    Ok(index)
                }
            },
        )
        .expect("no job panics");
    assert_eq!(results.into_iter().collect::<Result<Vec<_>, _>>(), Err(5));
}

#[test]
fn a_panicking_job_surfaces_as_a_panicked_fan_out_and_the_pool_stays_usable() {
    let pool = WorkerPool::with_helpers(3);
    let payload = pool
        .map_indexed(
            4,
            64,
            || (),
            |(), index| assert!(index != 17, "job 17 failed"),
        )
        .expect_err("job 17 panics");
    assert_eq!(panic_message(&*payload), "job 17 failed");
    let after = pool
        .map_indexed(4, 8, || (), |(), index| index)
        .expect("no job panics");
    assert_eq!(after, (0..8).collect::<Vec<_>>());
}

#[test]
fn a_panic_on_a_helper_thread_reaches_the_caller() {
    let pool = WorkerPool::with_helpers(1);
    let caller = std::thread::current().id();
    let helper_ran = AtomicBool::new(false);
    let payload = pool
        .map_indexed(
            2,
            2,
            || (),
            |(), _| {
                if std::thread::current().id() == caller {
                    let deadline = Instant::now() + Duration::from_secs(10);
                    while !helper_ran.load(Ordering::Acquire) {
                        assert!(Instant::now() < deadline, "no helper joined the fan-out");
                        std::thread::yield_now();
                    }
                } else {
                    helper_ran.store(true, Ordering::Release);
                    panic!("helper job failed");
                }
            },
        )
        .expect_err("the helper panics");
    assert_eq!(panic_message(&*payload), "helper job failed");
}

#[test]
#[should_panic(expected = "inline job failed")]
fn a_single_participant_panics_inline() {
    let pool = WorkerPool::with_helpers(3);
    let _ = pool.map_indexed(1, 4, || (), |(), _| panic!("inline job failed"));
}

#[test]
fn nested_fan_outs_finish_when_they_outnumber_the_helpers() {
    let pool = WorkerPool::with_helpers(2);
    let totals = pool
        .map_indexed(
            3,
            6,
            || (),
            |(), outer| {
                pool.map_indexed(4, 50, || (), |(), inner| outer * 100 + inner)
                    .expect("no inner job panics")
                    .into_iter()
                    .sum::<usize>()
            },
        )
        .expect("no outer job panics");
    let expected = (0..6)
        .map(|outer| (0..50).map(|inner| outer * 100 + inner).sum::<usize>())
        .collect::<Vec<_>>();
    assert_eq!(totals, expected);
}

#[test]
fn a_pool_that_never_fans_out_spawns_no_threads() {
    let pool = WorkerPool::with_helpers(3);
    let results = pool
        .map_indexed(1, 5, || (), |(), index| index)
        .expect("no job panics");
    assert_eq!(results, (0..5).collect::<Vec<_>>());
    assert!(pool.helpers.get().is_none());
}
