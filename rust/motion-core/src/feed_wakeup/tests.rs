use std::io::Read;

use super::FeedWakeup;

fn drain(w: &FeedWakeup) -> usize {
    let mut buf = [0u8; 64];
    match (&w.rx).read(&mut buf) {
        Ok(n) => n,
        Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => 0,
        Err(e) => panic!("wakeup read: {e}"),
    }
}

#[test]
fn space_freed_is_silent_unless_armed() {
    let w = FeedWakeup::default();
    w.notify_space_freed();
    assert_eq!(drain(&w), 0, "unarmed space wakeup must write nothing");
    w.arm();
    w.notify_space_freed();
    assert_eq!(drain(&w), 1, "armed space wakeup must write one byte");
    w.notify_space_freed();
    assert_eq!(drain(&w), 0, "arming is consumed by the first wakeup");
}

#[test]
fn fence_resolution_always_pings() {
    let w = FeedWakeup::default();
    w.notify_fence_resolved();
    w.notify_fence_resolved();
    assert!(drain(&w) >= 1, "fence resolution must wake the host");
}

#[test]
fn ping_survives_a_full_buffer() {
    let w = FeedWakeup::default();
    for _ in 0..100_000 {
        w.notify_fence_resolved();
    }
    while drain(&w) > 0 {}
    w.arm();
    w.notify_space_freed();
    assert_eq!(drain(&w), 1, "wakeup must keep working after overflow");
}
