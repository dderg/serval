use std::time::Duration;

use crate::clock::{Clock, monotonic_raw_secs};
use crate::host_io::CommandTiming;
use crate::host_io::reactor::outbound::{ClockEstimate, ScheduledPayload};
use crate::host_io::test_harness::ReactorHarness;

fn seed_clock(harness: &mut ReactorHarness, last_clock: u64) {
    harness.reactor.clock_estimate = Some(
        ClockEstimate::from_raw(
            1_000_000.0,
            monotonic_raw_secs(),
            last_clock,
            harness.clock.now(),
        )
        .unwrap(),
    );
}

fn payload_written(harness: &ReactorHarness, payload: u8) -> bool {
    harness.port_handles.tx.lock().unwrap().contains(&payload)
}

#[test]
fn future_minclock_does_not_write_early() {
    let mut harness = ReactorHarness::new();
    seed_clock(&mut harness, 1_000_000);
    harness
        .reactor
        .enqueue_scheduled(
            CommandTiming::Timed {
                min_clock: 2_000_000,
                req_clock: 0,
            },
            ScheduledPayload::FireAndForget(vec![0xA1]),
        )
        .unwrap();

    harness.reactor.drain_scheduled_commands();
    assert!(!payload_written(&harness, 0xA1));
    harness.clock.advance(Duration::from_millis(1_100));
    harness.reactor.drain_scheduled_commands();
    assert!(payload_written(&harness, 0xA1));
}

#[test]
fn reqclock_uses_one_hundred_millisecond_send_ahead() {
    let mut harness = ReactorHarness::new();
    seed_clock(&mut harness, 1_000_000);
    harness
        .reactor
        .enqueue_scheduled(
            CommandTiming::Timed {
                min_clock: 0,
                req_clock: 2_000_000,
            },
            ScheduledPayload::FireAndForget(vec![0xA2]),
        )
        .unwrap();

    harness.clock.advance(Duration::from_millis(800));
    harness.reactor.drain_scheduled_commands();
    assert!(!payload_written(&harness, 0xA2));
    harness.clock.advance(Duration::from_millis(150));
    harness.reactor.drain_scheduled_commands();
    assert!(payload_written(&harness, 0xA2));
}

#[test]
fn immediate_command_bypasses_future_timed_command() {
    let mut harness = ReactorHarness::new();
    seed_clock(&mut harness, 1_000_000);
    harness
        .reactor
        .enqueue_scheduled(
            CommandTiming::Timed {
                min_clock: 3_000_000,
                req_clock: 3_000_000,
            },
            ScheduledPayload::FireAndForget(vec![0xA3]),
        )
        .unwrap();
    harness
        .reactor
        .enqueue_scheduled(
            CommandTiming::Immediate,
            ScheduledPayload::FireAndForget(vec![0xA4]),
        )
        .unwrap();

    assert!(payload_written(&harness, 0xA4));
    assert!(!payload_written(&harness, 0xA3));
}

#[test]
fn timed_commands_keep_fifo_order() {
    let mut harness = ReactorHarness::new();
    seed_clock(&mut harness, 1_000_000);
    for (min_clock, payload) in [(3_000_000, 0xA5), (0, 0xA6)] {
        harness
            .reactor
            .enqueue_scheduled(
                CommandTiming::Timed {
                    min_clock,
                    req_clock: 0,
                },
                ScheduledPayload::FireAndForget(vec![payload]),
            )
            .unwrap();
    }

    harness.reactor.drain_scheduled_commands();
    assert!(!payload_written(&harness, 0xA5));
    assert!(!payload_written(&harness, 0xA6));
    harness.clock.advance(Duration::from_millis(2_100));
    harness.reactor.drain_scheduled_commands();
    let bytes = harness.port_handles.tx.lock().unwrap();
    let first = bytes.iter().position(|byte| *byte == 0xA5).unwrap();
    let second = bytes.iter().position(|byte| *byte == 0xA6).unwrap();
    assert!(first < second);
}

#[test]
fn background_minclock_is_preserved() {
    let mut harness = ReactorHarness::new();
    seed_clock(&mut harness, 1_000_000);
    harness
        .reactor
        .enqueue_scheduled(
            CommandTiming::Background {
                min_clock: 2_000_000,
            },
            ScheduledPayload::FireAndForget(vec![0xA7]),
        )
        .unwrap();

    harness.reactor.drain_scheduled_commands();
    assert!(!payload_written(&harness, 0xA7));
    harness.clock.advance(Duration::from_millis(1_100));
    harness.reactor.drain_scheduled_commands();
    assert!(payload_written(&harness, 0xA7));
}

#[test]
fn background_admits_only_one_command_per_tick() {
    let mut harness = ReactorHarness::new();
    for payload in [0xA8, 0xA9] {
        harness
            .reactor
            .enqueue_scheduled(
                CommandTiming::Background { min_clock: 0 },
                ScheduledPayload::FireAndForget(vec![payload]),
            )
            .unwrap();
    }

    harness.reactor.drain_scheduled_commands();
    assert!(payload_written(&harness, 0xA8));
    assert!(!payload_written(&harness, 0xA9));
    harness.reactor.drain_scheduled_commands();
    assert!(payload_written(&harness, 0xA9));
}

#[test]
fn ordinary_control_queue_drains_before_background() {
    let mut harness = ReactorHarness::new();
    harness
        .reactor
        .outbound
        .enqueue_fire_and_forget(vec![0xAA], false);
    harness
        .reactor
        .enqueue_scheduled(
            CommandTiming::Background { min_clock: 0 },
            ScheduledPayload::FireAndForget(vec![0xAB]),
        )
        .unwrap();

    harness.reactor.drain_pending_submissions();
    harness.reactor.drain_scheduled_commands();
    let bytes = harness.port_handles.tx.lock().unwrap();
    let control = bytes.iter().position(|byte| *byte == 0xAA).unwrap();
    let background = bytes.iter().position(|byte| *byte == 0xAB).unwrap();
    assert!(control < background);
}

#[test]
fn timed_command_without_clock_estimate_fails_loudly() {
    let mut harness = ReactorHarness::new();
    let error = harness
        .reactor
        .enqueue_scheduled(
            CommandTiming::Timed {
                min_clock: 1,
                req_clock: 1,
            },
            ScheduledPayload::FireAndForget(vec![0xAA]),
        )
        .unwrap_err();
    assert!(error.to_string().contains("no MCU clock estimate"));
}
