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

const LUT_STEP_MM: f32 = 40.0 / (200.0 * 256.0); // 0.78125 um — XDIRECT native res

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
    let mut cursor = XdirectCursor::start(p);
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
    let cfg = XdirectConfig {
        lut_step_mm: LUT_STEP_MM,
        grid_steps: 4,
    };
    let ups = collect(&p, &cfg);
    assert!(
        ups.len() > 100,
        "expected a dense stream, got {}",
        ups.len()
    );

    let peak_steps = (amp / LUT_STEP_MM).round() as i32;
    let max_seen = ups.iter().map(|u| u.offset_steps.abs()).max().unwrap();
    // The forced-extrema emission must reach the true peak, not clip below it.
    assert_eq!(
        max_seen, peak_steps,
        "peak offset {max_seen} must equal the analytic peak {peak_steps}"
    );
}

#[test]
fn tone_respects_grid_spacing_and_monotonic_time() {
    let p = tone(55.0, 0.0461, 0.2, 0.02);
    let cfg = XdirectConfig {
        lut_step_mm: LUT_STEP_MM,
        grid_steps: 4,
    };
    let ups = collect(&p, &cfg);
    for w in ups.windows(2) {
        assert!(w[1].t > w[0].t, "time must strictly advance");
        let d = (w[1].offset_steps - w[0].offset_steps).abs();
        assert!(
            d <= cfg.grid_steps + 1,
            "offset jumped {d} > grid_steps {} — a grid line was skipped",
            cfg.grid_steps
        );
    }
    for u in &ups {
        assert!(u.t >= 0.0 && u.t <= p.total_seconds);
    }
}

#[test]
fn emitted_offsets_reconstruct_the_analytic_sine() {
    let p = tone(55.0, 0.0461, 0.2, 0.02);
    let cfg = XdirectConfig {
        lut_step_mm: LUT_STEP_MM,
        grid_steps: 4,
    };
    for u in collect(&p, &cfg) {
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
fn sweep_holds_a_flat_update_rate_not_a_rising_one() {
    // 5 -> 135 Hz: amplitude tapers ~27x, but constant peak velocity means a
    // constant-displacement grid yields a ~flat update rate. A fixed-N scheme
    // would instead rise ~27x with frequency.
    let p = chirp(5.0, 135.0, 0.507, 1.0, 0.05);
    let cfg = XdirectConfig {
        lut_step_mm: LUT_STEP_MM,
        grid_steps: 8,
    };
    let ups = collect(&p, &cfg);

    let rate_in = |lo: f32, hi: f32| -> f32 {
        let n = ups.iter().filter(|u| u.t >= lo && u.t < hi).count();
        n as f32 / (hi - lo)
    };
    let early = rate_in(0.1, 0.3); // ~12-30 Hz
    let late = rate_in(0.7, 0.9); // ~95-120 Hz
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
    let cfg = XdirectConfig {
        lut_step_mm: LUT_STEP_MM,
        grid_steps: 8,
    };
    let ups = collect(&p, &cfg);
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
