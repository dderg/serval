use super::*;

fn axis(pending: u32, pushed: u32, retired: u32) -> AxisDrainState {
    AxisDrainState {
        pending,
        pushed,
        retired,
        staged_motion: pending,
        hold_tail: 0,
    }
}

fn snapshot(entries: &[((u32, u8), AxisDrainState)]) -> BTreeMap<(u32, u8), AxisDrainState> {
    entries.iter().copied().collect()
}

#[test]
fn empty_ledger_is_trivially_drained() {
    let d = DrainLedger::new();
    assert!(d.drained());
    assert!(d.wait_drained(Duration::from_millis(20)).is_ok());
}

#[test]
fn drained_only_when_pushed_equals_retired_and_nothing_pending() {
    let d = DrainLedger::new();
    d.publish(snapshot(&[((1, 0), axis(0, 3, 2))]));
    assert!(!d.drained());
    d.publish(snapshot(&[((1, 0), axis(5, 3, 3))]));
    assert!(!d.drained(), "staged pieces keep the axis undrained");
    d.publish(snapshot(&[((1, 0), axis(0, 3, 3))]));
    assert!(d.drained());
}

#[test]
fn multi_axis_all_must_drain() {
    let d = DrainLedger::new();
    d.publish(snapshot(&[
        ((0, 0), axis(0, 17474, 17474)),
        ((1, 2), axis(0, 86, 40)),
    ]));
    assert!(d.wait_drained(Duration::from_millis(20)).is_err());
    d.publish(snapshot(&[
        ((0, 0), axis(0, 17474, 17474)),
        ((1, 2), axis(0, 86, 86)),
    ]));
    assert!(d.wait_drained(Duration::from_millis(20)).is_ok());
}

#[test]
fn wait_drained_wakes_on_publish() {
    let d = std::sync::Arc::new(DrainLedger::new());
    d.publish(snapshot(&[((0, 3), axis(0, 10, 4))]));
    let d2 = std::sync::Arc::clone(&d);
    let waiter = std::thread::spawn(move || d2.wait_drained(Duration::from_secs(5)));
    std::thread::sleep(Duration::from_millis(30));
    d.publish(snapshot(&[((0, 3), axis(0, 10, 10))]));
    assert!(waiter.join().unwrap().is_ok());
}

#[test]
fn trailing_hold_coverage_does_not_gate_drain() {
    let d = DrainLedger::new();
    let hold_tail = AxisDrainState {
        pending: 0,
        pushed: 10,
        retired: 9,
        staged_motion: 0,
        hold_tail: 1,
    };
    d.publish(snapshot(&[((0, 0), hold_tail)]));
    assert!(
        d.drained(),
        "an unretired wire tail that is all hold coverage is not motion"
    );
    let motion_behind_hold = AxisDrainState {
        retired: 8,
        ..hold_tail
    };
    d.publish(snapshot(&[((0, 0), motion_behind_hold)]));
    assert!(
        !d.drained(),
        "an unretired motion piece behind the hold tail still gates"
    );
}

#[test]
fn staged_holds_gate_reseed_until_the_pump_hands_them_to_the_endpoint() {
    let d = DrainLedger::new();
    let staged_holds = AxisDrainState {
        pending: 3,
        pushed: 5,
        retired: 5,
        staged_motion: 0,
        hold_tail: 0,
    };
    d.publish(snapshot(&[((0, 0), staged_holds)]));
    assert!(
        !d.drained(),
        "a reseed must not overtake staged holds and orphan their seam mark"
    );
    let handed_to_endpoint = AxisDrainState {
        pending: 0,
        pushed: 8,
        hold_tail: 3,
        ..staged_holds
    };
    d.publish(snapshot(&[((0, 0), handed_to_endpoint)]));
    assert!(
        d.drained(),
        "wire-side trailing holds need not execute before reseed"
    );
}

#[test]
fn timeout_reports_lagging_axes() {
    let d = DrainLedger::new();
    d.publish(snapshot(&[((0, 3), axis(2, 100, 90))]));
    let err = d.wait_drained(Duration::from_millis(20)).unwrap_err();
    assert!(err.contains("mcu0 axis3"), "{err}");
    assert!(err.contains("pending 2 pushed 100 retired 90"), "{err}");
    assert_eq!(d.lagging_axes(), vec![(0, 3, axis(2, 100, 90))]);
}
