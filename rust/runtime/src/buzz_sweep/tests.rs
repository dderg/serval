#![allow(
    clippy::unwrap_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::indexing_slicing,
    clippy::panic
)]

use super::*;
use std::vec::Vec;

const MICROSTEP_MM: f32 = 40.0 / (200.0 * 32.0);

fn sweep(f0: f32, f1: f32, amplitude_mm: f32, total: f32, ramp: f32) -> ToneParams {
    ToneParams {
        omega: TWO_PI * f0,
        mu: TWO_PI * (f1 - f0) / total,
        amplitude_mm,
        sign: 1.0,
        base_mm: 0.0,
        microstep_distance: MICROSTEP_MM,
        anchor_cycle: 0,
        cycles_per_second: 550_000_000.0,
        total_seconds: total,
        ramp_seconds: ramp,
    }
}

fn collect(p: &ToneParams) -> Vec<ToneCrossing> {
    let mut out = Vec::new();
    let mut cursor = SweepCursor::start(p);
    loop {
        match next_crossing_sweep(p, cursor) {
            Ok((c, next)) => {
                assert!(out.len() < 5_000_000, "runaway");
                out.push(c);
                cursor = next;
            }
            Err(ToneError::Done) => break,
            Err(e) => panic!("unexpected fault: {e:?}"),
        }
    }
    out
}

#[test]
fn completes_with_monotonic_times_in_window() {
    let p = sweep(5.0, 50.0, 0.3, 0.2, 0.02);
    let xs = collect(&p);
    assert!(xs.len() > 50, "expected a dense stream, got {}", xs.len());
    let mut prev = 0.0_f32;
    for c in &xs {
        assert!(
            c.t > prev,
            "time not strictly increasing: {} <= {}",
            c.t,
            prev
        );
        assert!(
            c.t <= p.total_seconds + 1e-6,
            "crossing past the window: {} > {}",
            c.t,
            p.total_seconds
        );
        assert!(c.dir == 1 || c.dir == -1, "bad dir {}", c.dir);
        prev = c.t;
    }
}

#[test]
fn parks_on_base_net_zero() {
    let p = sweep(5.0, 60.0, 0.06, 0.25, 0.02);
    let xs = collect(&p);
    let net: i32 = xs.iter().map(|c| i32::from(c.dir)).sum();
    assert_eq!(net, 0, "sweep did not return to base (net steps {net})");
    assert_eq!(xs.last().unwrap().level, 0, "final edge is not on base");
}

#[test]
fn steep_band_does_not_fault() {
    let p = sweep(5.0, 135.0, 0.4, 0.15, 0.015);
    let xs = collect(&p);
    assert!(xs.len() > 100, "expected a dense stream, got {}", xs.len());
}

#[test]
fn finer_microstepping_stays_clean() {
    let mut p = sweep(8.0, 48.0, 0.05, 0.2, 0.02);
    p.microstep_distance = 40.0 / (200.0 * 256.0);
    let xs = collect(&p);
    assert!(
        xs.len() > 200,
        "expected more crossings at 256, got {}",
        xs.len()
    );
    let net: i32 = xs.iter().map(|c| i32::from(c.dir)).sum();
    assert_eq!(net, 0, "net steps {net}");
}

#[test]
fn lobe_count_and_durations_track_the_frequency_staircase() {
    let p = sweep(10.0, 40.0, 0.06, 0.4, 0.02);
    let xs = collect(&p);
    let lobe_ends: Vec<f32> = xs.iter().filter(|c| c.level == 0).map(|c| c.t).collect();
    assert!(lobe_ends.len() >= 6, "too few lobes: {}", lobe_ends.len());
    let first = lobe_ends[1] - lobe_ends[0];
    let last = lobe_ends[lobe_ends.len() - 1] - lobe_ends[lobe_ends.len() - 2];
    assert!(
        last < first,
        "lobes did not shorten across the sweep: first {first}, last {last}"
    );
    assert!(first <= 0.5 / 10.0 + 5e-3, "first lobe too long: {first}");
    assert!(last >= 0.5 / 40.0 - 5e-3, "last lobe too short: {last}");
}

#[test]
fn step_freq_climbs_and_clamps_to_band_end() {
    let p = sweep(5.0, 50.0, 0.05, 1.0, 0.02);
    let mut f = start_hz(&p);
    assert!((f - 5.0).abs() < 1e-3);
    let mut steps = 0;
    while f < end_hz(&p) - 1e-4 {
        let next = step_freq(&p, f);
        assert!(next > f, "frequency did not climb at {f}");
        f = next;
        steps += 1;
        assert!(steps < 10_000_000, "runaway staircase");
    }
    assert!(
        (f - end_hz(&p)).abs() < 1.0,
        "did not converge to band end: {f}"
    );
    assert!((step_freq(&p, end_hz(&p)) - end_hz(&p)).abs() < 1e-3);
}

#[test]
fn downward_sweep_clamps_to_lower_end() {
    let p = sweep(50.0, 10.0, 0.05, 0.2, 0.02);
    assert!(hz_per_sec(&p) < 0.0);
    let next = step_freq(&p, 50.0);
    assert!(
        next < 50.0 && next >= end_hz(&p),
        "bad downward step {next}"
    );
    assert!((step_freq(&p, end_hz(&p)) - end_hz(&p)).abs() < 1e-3);
    let xs = collect(&p);
    let net: i32 = xs.iter().map(|c| i32::from(c.dir)).sum();
    assert_eq!(net, 0, "downward sweep net steps {net}");
}
