#![allow(
    clippy::unwrap_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::indexing_slicing,
    clippy::panic
)]

use super::*;
use crate::buzz_gen::{ToneParams, position_rel};
use std::vec::Vec;

const LUT_STEP_MM: f32 = 40.0 / (200.0 * 256.0);

fn cfg() -> XdirectConfig {
    XdirectConfig::new(LUT_STEP_MM, 10_000.0)
}

fn tone(freq_hz: f32, amplitude_mm: f32, total: f32, ramp: f32) -> ToneParams {
    ToneParams {
        omega: 2.0 * core::f32::consts::PI * freq_hz,
        mu: 0.0,
        amplitude_mm,
        sign: 1.0,
        base_mm: 0.0,
        microstep_distance: LUT_STEP_MM,
        anchor_cycle: 0,
        cycles_per_second: 550_000_000.0,
        total_seconds: total,
        ramp_seconds: ramp,
    }
}

fn chirp(f0: f32, f1: f32, amplitude_mm: f32, total: f32, ramp: f32) -> ToneParams {
    ToneParams {
        omega: 2.0 * core::f32::consts::PI * f0,
        mu: 2.0 * core::f32::consts::PI * (f1 - f0) / total,
        amplitude_mm,
        sign: 1.0,
        base_mm: 0.0,
        microstep_distance: LUT_STEP_MM,
        anchor_cycle: 0,
        cycles_per_second: 550_000_000.0,
        total_seconds: total,
        ramp_seconds: ramp,
    }
}

fn collect(p: &ToneParams, cfg: &XdirectConfig) -> Vec<XdirectUpdate> {
    let mut out = Vec::new();
    let mut cursor = XdirectCursor::start(p, cfg);
    loop {
        match next_update(p, cfg, cursor) {
            Ok((u, next)) => {
                assert!(out.len() < 2_000_000, "runaway");
                out.push(u);
                cursor = next;
            }
            Err(ToneError::Done) => break,
            Err(e) => panic!("unexpected fault: {e:?}"),
        }
    }
    out
}

#[test]
fn tone_pins_peak_amplitude_exactly() {
    let amp = 0.0461_f32;
    let p = tone(55.0, amp, 0.2, 0.02);
    let ups = collect(&p, &cfg());
    assert!(
        ups.len() > 100,
        "expected a dense stream, got {}",
        ups.len()
    );

    let peak_steps = (amp / LUT_STEP_MM).round() as i32;
    let max_seen = ups.iter().map(|u| u.offset_steps.abs()).max().unwrap();
    assert_eq!(
        max_seen, peak_steps,
        "peak offset {max_seen} must equal the analytic peak {peak_steps}"
    );
    let min_seen = ups.iter().map(|u| u.offset_steps).min().unwrap();
    assert_eq!(
        min_seen, -peak_steps,
        "the negative extremum must be pinned too"
    );
}

#[test]
fn tone_update_rate_is_exact_integer_multiple_of_frequency() {
    let f = 55.0_f32;
    let p = tone(f, 0.0461, 0.3, 0.02);
    let ups = collect(&p, &cfg());

    let gaps: Vec<f32> = ups
        .windows(2)
        .map(|w| w[1].t - w[0].t)
        .filter(|g| *g > 0.0)
        .collect();
    let interior = &gaps[2..gaps.len() - 2];
    let gmin = interior.iter().copied().fold(f32::INFINITY, f32::min);
    let gmax = interior.iter().copied().fold(0.0_f32, f32::max);
    assert!(
        gmax / gmin < 1.001,
        "constant-frequency spacing must be uniform: min={gmin:.3e} max={gmax:.3e}"
    );

    let n = (1.0 / (gmin * f)).round() as i32;
    assert_eq!(n % 4, 0, "N must be a multiple of 4, got {n}");
    let rate = n as f32 * f;
    assert!(
        rate <= 10_000.0 * 1.02,
        "realized rate {rate:.0} must sit at/under the budget"
    );
    assert!((gmin - 1.0 / (n as f32 * f)).abs() < gmin * 1e-3);
}

#[test]
fn tone_zero_crossings_are_grid_points() {
    let p = tone(55.0, 0.0461, 0.2, 0.02);
    let ups = collect(&p, &cfg());
    let at_zero = ups.iter().filter(|u| u.offset_steps == 0).count();
    assert!(
        at_zero >= 10,
        "expected a sample at each zero crossing, saw {at_zero}"
    );
}

#[test]
fn time_is_strictly_monotonic_and_in_window() {
    let p = tone(55.0, 0.0461, 0.2, 0.02);
    let ups = collect(&p, &cfg());
    for w in ups.windows(2) {
        assert!(w[1].t > w[0].t, "time must strictly advance");
    }
    for u in &ups {
        assert!(u.t >= 0.0 && u.t <= p.total_seconds);
    }
}

#[test]
fn emitted_offsets_reconstruct_the_analytic_sine() {
    let p = tone(55.0, 0.0461, 0.2, 0.02);
    for u in collect(&p, &cfg()) {
        let exact = position_rel(&p, u.t);
        let cmd = u.offset_steps as f32 * LUT_STEP_MM;
        assert!(
            (cmd - exact).abs() <= LUT_STEP_MM * 1.01,
            "commanded {cmd} vs exact {exact} differ by more than one LUT step at t={}",
            u.t
        );
    }
}

#[test]
fn offset_jump_per_update_is_bounded() {
    let p = tone(55.0, 0.0461, 0.2, 0.02);
    for w in collect(&p, &cfg()).windows(2) {
        let d = (w[1].offset_steps - w[0].offset_steps).abs();
        assert!(
            d <= 3,
            "offset jumped {d} steps in one update — grid too coarse"
        );
    }
}

#[test]
fn tone_parks_on_base_at_the_end() {
    let p = tone(55.0, 0.0461, 0.2, 0.02);
    let ups = collect(&p, &cfg());
    let last = ups.last().unwrap();
    assert_eq!(last.offset_steps, 0, "axis must park on base");
    assert!(
        (last.t - p.total_seconds).abs() < 1e-6,
        "the close must land at the window end, got t={}",
        last.t
    );
}

#[test]
fn sweep_holds_a_flat_update_rate_not_a_rising_one() {
    let p = chirp(5.0, 135.0, 0.507, 1.0, 0.05);
    let ups = collect(&p, &cfg());

    let rate_in = |lo: f32, hi: f32| -> f32 {
        let n = ups.iter().filter(|u| u.t >= lo && u.t < hi).count();
        n as f32 / (hi - lo)
    };
    let early = rate_in(0.1, 0.3);
    let late = rate_in(0.7, 0.9);
    assert!(
        early > 0.0 && late > 0.0,
        "expected updates in both windows"
    );
    let ratio = late / early;
    assert!(
        (0.5..=2.0).contains(&ratio),
        "update rate should stay flat across the sweep; early={early:.0}/s late={late:.0}/s ratio={ratio:.2}"
    );
}

#[test]
fn sweep_peak_offsets_taper_with_amplitude() {
    let p = chirp(5.0, 135.0, 0.507, 1.0, 0.05);
    let ups = collect(&p, &cfg());
    let peak_in = |lo: f32, hi: f32| -> i32 {
        ups.iter()
            .filter(|u| u.t >= lo && u.t < hi)
            .map(|u| u.offset_steps.abs())
            .max()
            .unwrap_or(0)
    };
    let early_peak = peak_in(0.1, 0.3);
    let late_peak = peak_in(0.7, 0.9);
    assert!(
        early_peak > late_peak * 3,
        "amplitude should taper over the sweep: early peak {early_peak} vs late {late_peak}"
    );
}

#[test]
fn sweep_changes_n_only_at_turning_points() {
    let p = chirp(5.0, 135.0, 0.507, 1.0, 0.05);
    let ups = collect(&p, &cfg());

    let phi_of = |t: f32| (p.omega + 0.5 * p.mu * t) * t;
    let phis: Vec<f32> = ups[..ups.len() - 1].iter().map(|u| phi_of(u.t)).collect();
    let dphi: Vec<f32> = phis.windows(2).map(|w| w[1] - w[0]).collect();

    let half_pi = core::f32::consts::FRAC_PI_2;
    let pi = core::f32::consts::PI;
    let mut seams = 0;
    for i in 1..dphi.len() {
        if (dphi[i] - dphi[i - 1]).abs() > dphi[i - 1] * 0.02 {
            let phi = phis[i];
            let nearest = (((phi - half_pi) / pi).round()) * pi + half_pi;
            assert!(
                (phi - nearest).abs() < 0.05,
                "Δφ changed at φ={phi:.4}, not a turning point (nearest {nearest:.4})"
            );
            seams += 1;
        }
    }
    assert!(seams > 0, "the sweep should cross several N seams");
}
