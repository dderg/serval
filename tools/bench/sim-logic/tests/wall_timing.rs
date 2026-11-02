use bench_sim_logic::config::SimConfig;
use bench_sim_logic::tracker::{Step, Tracker};
use bench_sim_logic::walls::{Wall, Walls};

const TICKS_PER_AUX: u32 = 10;
const PROBE: u8 = 1 << 3;
const X_MIN: u8 = 1 << 0;

fn config(walls: &[Wall], origin: [i32; 4]) -> SimConfig {
    let mut set = Walls::default();
    for &wall in walls {
        set.push(wall).unwrap();
    }
    SimConfig {
        id: 1,
        origin,
        walls: set,
        ..SimConfig::default()
    }
}

fn probe_at_zero() -> Wall {
    Wall {
        pin: 3,
        active_high: true,
        at_or_below: true,
        coeffs: [0, 0, 1, 0],
        threshold: 0,
    }
}

fn corexy_x_at_or_above(threshold: i32) -> Wall {
    Wall {
        pin: 0,
        active_high: true,
        at_or_below: false,
        coeffs: [1, 1, 0, 0],
        threshold,
    }
}

fn steps(list: &[(u64, i32)]) -> Vec<Step> {
    list.iter()
        .map(|&(time, sign)| Step { time, sign })
        .collect()
}

fn event_tick(tracker: &mut Tracker, per_channel: [&[Step]; 4]) -> u8 {
    tracker.tick_steps(per_channel, 0).unwrap().levels
}

#[test]
fn the_crossing_step_counts_in_the_snapshot_that_holds_it() {
    let mut tracker = Tracker::start(&config(&[probe_at_zero()], [0, 0, 2, 0]), TICKS_PER_AUX, 0);
    let first = steps(&[(10, -1), (8_399, -1)]);
    assert_eq!(event_tick(&mut tracker, [&[], &[], &first, &[]]), PROBE);
    assert_eq!(tracker.positions()[2], 0);
}

#[test]
fn a_crossing_step_just_after_the_boundary_waits_for_the_next_snapshot() {
    let mut tracker = Tracker::start(&config(&[probe_at_zero()], [0, 0, 2, 0]), TICKS_PER_AUX, 0);
    let first = steps(&[(8_399, -1)]);
    let second = steps(&[(8_400, -1)]);
    assert_eq!(event_tick(&mut tracker, [&[], &[], &first, &[]]), 0);
    assert_eq!(event_tick(&mut tracker, [&[], &[], &second, &[]]), PROBE);
}

#[test]
fn a_touch_and_return_inside_one_snapshot_asserts_for_that_snapshot() {
    let mut tracker = Tracker::start(&config(&[probe_at_zero()], [0, 0, 1, 0]), TICKS_PER_AUX, 0);
    let touch = steps(&[(100, -1), (900, 1)]);
    assert_eq!(event_tick(&mut tracker, [&[], &[], &touch, &[]]), PROBE);
    assert_eq!(tracker.positions()[2], 1);
    assert_eq!(event_tick(&mut tracker, [&[], &[], &[], &[]]), 0);
}

#[test]
fn corexy_walls_follow_the_time_order_across_channels() {
    let wall = corexy_x_at_or_above(1);
    let a_first = (steps(&[(5, 1)]), steps(&[(10, -1)]));
    let b_first = (steps(&[(10, 1)]), steps(&[(5, -1)]));
    let mut tracker = Tracker::start(&config(&[wall], [0; 4]), TICKS_PER_AUX, 0);
    assert_eq!(
        event_tick(&mut tracker, [&a_first.0, &a_first.1, &[], &[]]),
        X_MIN
    );
    let mut tracker = Tracker::start(&config(&[wall], [0; 4]), TICKS_PER_AUX, 0);
    assert_eq!(
        event_tick(&mut tracker, [&b_first.0, &b_first.1, &[], &[]]),
        0
    );
}

#[test]
fn walls_on_other_channels_ignore_a_channel_s_steps() {
    let mut tracker = Tracker::start(
        &config(&[probe_at_zero(), corexy_x_at_or_above(100)], [0, 0, 5, 0]),
        TICKS_PER_AUX,
        0,
    );
    let extruder = steps(&[(1, -1), (2, -1), (3, -1)]);
    assert_eq!(event_tick(&mut tracker, [&[], &[], &[], &extruder]), 0);
    assert_eq!(tracker.positions(), [0, 0, 5, -3]);
}

#[test]
fn count_mode_sees_a_single_channel_touch_through_its_excursion() {
    let mut tracker = Tracker::start(&config(&[probe_at_zero()], [0, 0, 2, 0]), TICKS_PER_AUX, 0);
    let output = tracker.tick(&[0, 0, 0, 0], &[0, 0, -2, 0], 0).unwrap();
    assert_eq!(output.levels, PROBE);
    assert_eq!(tracker.tick(&[0; 4], &[0; 4], 0).unwrap().levels, 0);
}

#[test]
fn count_mode_excursions_short_of_the_wall_do_not_trigger() {
    let mut tracker = Tracker::start(&config(&[probe_at_zero()], [0, 0, 2, 0]), TICKS_PER_AUX, 0);
    assert_eq!(
        tracker
            .tick(&[0, 0, 0, 0], &[0, 0, -1, 0], 0)
            .unwrap()
            .levels,
        0
    );
}

#[test]
fn count_mode_judges_corexy_walls_at_the_tick_end() {
    let mut tracker = Tracker::start(
        &config(&[corexy_x_at_or_above(1)], [0; 4]),
        TICKS_PER_AUX,
        0,
    );
    let output = tracker.tick(&[0, 0, 0, 0], &[1, 0, 0, 0], 0).unwrap();
    assert_eq!(output.levels, 0);
    assert_eq!(
        tracker
            .tick(&[1, 0, 0, 0], &[1, 0, 0, 0], 0)
            .unwrap()
            .levels,
        X_MIN
    );
}

#[test]
fn count_and_event_ticks_agree_on_monotone_motion() {
    let walls = [probe_at_zero(), corexy_x_at_or_above(4)];
    let mut counts = Tracker::start(&config(&walls, [0, 0, 3, 0]), TICKS_PER_AUX, 0);
    let mut events = Tracker::start(&config(&walls, [0, 0, 3, 0]), TICKS_PER_AUX, 0);
    for tick in 0..6u64 {
        let base = tick * 1_000;
        let a = steps(&[(base + 1, 1)]);
        let z = steps(&[(base + 2, -1)]);
        let counted = counts
            .tick(&[1, 0, -1, 0], &[1, 0, -1, 0], 0)
            .unwrap()
            .levels;
        assert_eq!(
            event_tick(&mut events, [&a, &[], &z, &[]]),
            counted,
            "tick {tick}"
        );
    }
    assert_eq!(counts.positions(), events.positions());
}
