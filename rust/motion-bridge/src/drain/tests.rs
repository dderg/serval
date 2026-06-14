use super::*;

#[test]
fn drained_when_retired_equals_sent() {
    let d = DrainSync::new();
    d.add_sent(1, 0, 3);
    d.add_sent(1, 1, 2);
    assert!(d.wait_drained(Duration::from_millis(20)).is_err());
    d.set_retired(1, 0, 3);
    d.set_retired(1, 1, 2);
    assert!(d.wait_drained(Duration::from_millis(20)).is_ok());
}

#[test]
fn no_sent_is_trivially_drained() {
    let d = DrainSync::new();
    assert!(d.wait_drained(Duration::from_millis(20)).is_ok());
}

#[test]
fn reset_clears_sent_cumulative_retired_does_not_break_drain() {
    let d = DrainSync::new();
    d.add_sent(1, 0, 5);
    d.set_retired(1, 0, 5);
    assert!(d.wait_drained(Duration::from_millis(20)).is_ok());
    d.reset();
    d.set_retired(1, 0, 5);
    assert!(d.wait_drained(Duration::from_millis(20)).is_ok());
}

#[test]
fn regression_cumulative_retired_across_two_streams() {
    let d = DrainSync::new();

    d.add_sent(0, 0, 8737);
    d.set_retired(0, 0, 8737);
    assert!(
        d.wait_drained(Duration::from_millis(20)).is_ok(),
        "stream 1 should be drained"
    );

    d.reset();

    d.set_retired(0, 0, 8737);
    assert!(
        d.wait_drained(Duration::from_millis(20)).is_ok(),
        "after reset with no new sent, trivially drained"
    );

    d.add_sent(0, 0, 8737);
    assert!(
        d.wait_drained(Duration::from_millis(20)).is_err(),
        "stream 2 not yet drained before MCU catches up"
    );
    d.set_retired(0, 0, 17474);
    assert!(
        d.wait_drained(Duration::from_millis(20)).is_ok(),
        "stream 2 should be drained at cumulative 17474"
    );
}

#[test]
fn post_reset_partial_retired_not_drained() {
    let d = DrainSync::new();

    d.add_sent(0, 1, 86);
    d.set_retired(0, 1, 86);
    assert!(d.wait_drained(Duration::from_millis(20)).is_ok());

    d.reset();

    d.add_sent(0, 1, 100);
    d.set_retired(0, 1, 86);
    assert!(
        d.wait_drained(Duration::from_millis(20)).is_err(),
        "delta 0 against sent 100 must not be drained"
    );
    d.set_retired(0, 1, 136);
    assert!(
        d.wait_drained(Duration::from_millis(20)).is_err(),
        "delta 50 against sent 100 must not be drained"
    );
    d.set_retired(0, 1, 186);
    assert!(
        d.wait_drained(Duration::from_millis(20)).is_ok(),
        "delta 100 against sent 100 must be drained"
    );
}

#[test]
fn trip_unsend_reconciles_with_discard() {
    let d = DrainSync::new();

    d.set_retired(0, 0, 1000);
    d.reset();

    d.add_sent(0, 0, 200);
    d.set_retired(0, 0, 1080);
    assert!(
        d.wait_drained(Duration::from_millis(20)).is_err(),
        "mid-trip: 200 sent vs delta 80 — not drained"
    );

    d.unsend(0, 0, 80);
    d.set_retired(0, 0, 1120);
    assert!(
        d.wait_drained(Duration::from_millis(20)).is_ok(),
        "after unsend(80) + discard to head: delta 120 == sent 120 — drained"
    );
}

#[test]
fn multi_axis_all_must_drain() {
    let d = DrainSync::new();
    d.add_sent(0, 0, 17474);
    d.add_sent(1, 2, 86);

    d.set_retired(0, 0, 17474);
    assert!(
        d.wait_drained(Duration::from_millis(20)).is_err(),
        "axis (1,2) still pending"
    );
    d.set_retired(1, 2, 86);
    assert!(
        d.wait_drained(Duration::from_millis(20)).is_ok(),
        "all axes drained"
    );
}

#[test]
fn wait_room_unblocks_when_retired_advances() {
    let d = std::sync::Arc::new(DrainSync::new());
    d.add_sent(0, 0, 16); // ring of 16, fully occupied
    assert_eq!(d.room(0, 0, 16), 0);
    let d2 = d.clone();
    let h = std::thread::spawn(move || {
        d2.wait_room(0, 0, 16, 4, Duration::from_secs(2)).unwrap();
    });
    std::thread::sleep(std::time::Duration::from_millis(20));
    d.set_retired(0, 0, 5); // 5 drained -> room 5 >= 4
    h.join().unwrap();
}

#[test]
fn wait_room_returns_err_on_timeout() {
    let d = DrainSync::new();
    d.add_sent(0, 0, 16);
    let r = d.wait_room(0, 0, 16, 4, Duration::from_millis(30));
    assert!(r.is_err(), "should time out when nothing drains");
}

#[test]
fn room_accounts_for_baseline_after_reset() {
    let d = DrainSync::new();
    // A prior stream drained fully: MCU cumulative retired reached 100.
    d.add_sent(0, 0, 100);
    d.set_retired(0, 0, 100);
    // New stream: reset clears sent, snapshots baseline = 100.
    d.reset();
    // Nothing in flight this stream -> full ring, despite retired==100.
    assert_eq!(d.room(0, 0, 16), 16);
    // Send 4 this stream; MCU still at cumulative 100.
    d.add_sent(0, 0, 4);
    assert_eq!(d.room(0, 0, 16), 12);
    // MCU retires 2 this stream -> cumulative 102.
    d.set_retired(0, 0, 102);
    assert_eq!(d.room(0, 0, 16), 14);
}
