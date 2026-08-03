use super::*;
use std::time::Duration;

fn at(base: Instant, secs: f64) -> Instant {
    base + Duration::from_secs_f64(secs)
}

#[test]
fn executing_committed_motion_is_never_lagging() {
    let base = Instant::now();
    let mut diag = DrainWaitDiag::new(base, 20.0);
    let mut outstanding: f64 = 20.0;
    let mut t = 0.0;
    while outstanding > 0.0 {
        t += 0.01;
        outstanding -= 0.01;
        assert!(
            diag.poll(at(base, t), outstanding.max(0.0)).is_none(),
            "a 20 s buffer takes 20 s to drain; that is the SLA, not a lag (t={t})"
        );
    }
    assert!(t > 19.0, "the wait really did span the buffered motion");
}

#[test]
fn retirement_stuck_past_the_horizon_reports_once_then_on_cadence() {
    let base = Instant::now();
    let mut diag = DrainWaitDiag::new(base, 0.5);
    assert!(diag.poll(at(base, 1.0), 0.5).is_none());
    assert!(
        diag.poll(at(base, 1.5), 0.0).is_none(),
        "the grace window starts when the horizon is spent"
    );
    let first = diag
        .poll(at(base, 1.5 + OVERDUE_GRACE_SECS), 0.0)
        .expect("unretired motion a full lead past the horizon is lagging");
    assert!((first.waited_s - (1.5 + OVERDUE_GRACE_SECS)).abs() < 1e-6);
    assert!((first.overdue_s - OVERDUE_GRACE_SECS).abs() < 1e-6);
    assert!(
        (first.horizon_s - 0.5).abs() < 1e-6,
        "the report carries the horizon the wait was owed"
    );

    assert!(
        diag.poll(at(base, 4.0), 0.0).is_none(),
        "repeats are rate limited"
    );
    let second = diag
        .poll(at(base, 1.5 + OVERDUE_GRACE_SECS + REPORT_PERIOD_SECS), 0.0)
        .expect("a still-stuck drain reports again on cadence");
    assert!(second.overdue_s > first.overdue_s);
}

#[test]
fn motion_committed_mid_wait_rewinds_the_horizon() {
    let base = Instant::now();
    let mut diag = DrainWaitDiag::new(base, 0.0);
    assert!(diag.poll(at(base, 0.1), 0.0).is_none());
    assert!(
        diag.poll(at(base, 0.2), 1.0).is_none(),
        "a dwell committed mid-wait puts motion back ahead of the drain"
    );
    assert!(
        diag.poll(at(base, 0.2 + OVERDUE_GRACE_SECS), 0.0).is_none(),
        "the grace window restarts from the moment the new horizon is spent"
    );
    assert!(
        diag.poll(at(base, 0.3 + 2.0 * OVERDUE_GRACE_SECS), 0.0)
            .is_some(),
        "a lane still unretired a lead past the new horizon is lagging"
    );
}

#[test]
fn a_drain_with_nothing_committed_reports_within_a_lead() {
    let base = Instant::now();
    let mut diag = DrainWaitDiag::new(base, 0.0);
    assert!(diag.poll(base, 0.0).is_none());
    assert!(
        diag.poll(at(base, OVERDUE_GRACE_SECS), 0.0).is_some(),
        "no committed motion at all and still unretired: nothing to wait for"
    );
}
