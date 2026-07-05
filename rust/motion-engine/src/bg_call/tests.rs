use std::sync::mpsc::channel;

use super::BgCalls;

#[test]
fn pending_until_the_call_returns_then_consumed() {
    let calls = BgCalls::default();
    let (release_tx, release_rx) = channel();
    let id = calls.start("test", move || {
        release_rx.recv().map_err(|e| e.to_string())?;
        Ok(())
    });
    assert_eq!(calls.done(id), Ok(false));
    release_tx.send(()).unwrap();
    while calls.done(id) == Ok(false) {
        std::thread::yield_now();
    }
    assert!(calls.done(id).is_err(), "a resolved id must be consumed");
}

#[test]
fn call_error_surfaces_from_done() {
    let calls = BgCalls::default();
    let id = calls.start("test", || Err("endpoint result -312".to_string()));
    let outcome = loop {
        match calls.done(id) {
            Ok(false) => std::thread::yield_now(),
            other => break other,
        }
    };
    assert_eq!(outcome, Err("endpoint result -312".to_string()));
}

#[test]
fn unknown_id_is_a_loud_error() {
    let calls = BgCalls::default();
    assert!(calls.done(42).is_err());
}

#[test]
fn concurrent_calls_resolve_independently() {
    let calls = BgCalls::default();
    let (slow_tx, slow_rx) = channel();
    let slow = calls.start("slow", move || {
        slow_rx.recv().map_err(|e| e.to_string())?;
        Ok(())
    });
    let fast = calls.start("fast", || Ok(()));
    while calls.done(fast) == Ok(false) {
        std::thread::yield_now();
    }
    assert_eq!(calls.done(slow), Ok(false));
    slow_tx.send(()).unwrap();
    while calls.done(slow) == Ok(false) {
        std::thread::yield_now();
    }
}
