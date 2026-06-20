use super::*;

const SAMPLE_RATE: f32 = 10_000.0;

fn armed(
    buzz: &mut Buzz,
    axis_mask: u8,
    sign_mask: u8,
    freq_mhz: u32,
    amp_nm: u32,
    dur_ms: u32,
    ramp_ms: u32,
) {
    assert_eq!(
        buzz.arm(4, axis_mask, sign_mask, freq_mhz, amp_nm, dur_ms, ramp_ms),
        0
    );
    buzz.poll(SAMPLE_RATE);
}

#[test]
fn envelope_is_zero_at_both_ends() {
    let total = 1000;
    let ramp = 100;
    assert_eq!(envelope(0, total, ramp), 0.0);
    assert_eq!(envelope(total - 1, total, ramp), 0.0);
    assert_eq!(envelope(total, total, ramp), 0.0);
    // Flat top reaches unity.
    assert_eq!(envelope(500, total, ramp), 1.0);
}

#[test]
fn envelope_never_exceeds_unity_or_drops_below_zero() {
    let total = 200;
    let ramp = 50;
    for t in 0..total {
        let e = envelope(t, total, ramp);
        assert!((0.0..=1.0).contains(&e), "env({t}) = {e} out of range");
    }
}

#[test]
fn arm_rejects_out_of_range_arguments() {
    let buzz = Buzz::new();
    // zero frequency
    assert_eq!(buzz.arm(4, 0b1, 0, 0, 10_000, 1000, 100), -1);
    // frequency above ceiling
    assert_eq!(buzz.arm(4, 0b1, 0, 9_000_000, 10_000, 1000, 100), -1);
    // amplitude above ceiling
    assert_eq!(buzz.arm(4, 0b1, 0, 100_000, 9_000_000, 1000, 100), -1);
    // axis bit beyond num_axes (axis 4 set, only 0..3 valid)
    assert_eq!(buzz.arm(4, 0b1_0000, 0, 100_000, 10_000, 1000, 100), -1);
}

#[test]
fn arm_disarm_form_is_accepted_and_deactivates() {
    let mut buzz = Buzz::new();
    armed(&mut buzz, 0b11, 0, 100_000, 10_000, 100, 10);
    assert!(buzz.is_active());
    // amplitude 0 == disarm; must be accepted (returns 0) and clear activity.
    assert_eq!(buzz.arm(4, 0b11, 0, 100_000, 0, 100, 10), 0);
    buzz.poll(SAMPLE_RATE);
    assert!(!buzz.is_active());
}

#[test]
fn corexy_pair_is_phase_coherent_and_anti_phase_on_y() {
    // axis 0 (+) and axis 1 (-): cartesian-Y excitation on CoreXY.
    let mut buzz = Buzz::new();
    armed(&mut buzz, 0b11, 0b10, 100_000, 50_000, 200, 20);
    // Advance into the flat top to a quarter-period (625 ticks = 6.25 periods
    // at 100 Hz / 10 kHz) where the envelope is unity and sin is near its peak.
    for _ in 0..625 {
        let _ = (buzz.sample(0), buzz.sample(1));
        buzz.advance();
    }
    let a = buzz.sample(0);
    let b = buzz.sample(1);
    assert!(
        a.offset.abs() > 1e-6,
        "expected non-trivial offset, got {}",
        a.offset
    );
    // Same phase + envelope, opposite sign => exact negatives.
    assert!(
        (a.offset + b.offset).abs() < 1e-6,
        "A {} not anti-phase with B {}",
        a.offset,
        b.offset
    );
    assert!((a.velocity + b.velocity).abs() < 1e-6);
}

#[test]
fn unaffected_axes_get_zero_sample() {
    let mut buzz = Buzz::new();
    armed(&mut buzz, 0b01, 0, 100_000, 50_000, 200, 20);
    for _ in 0..500 {
        buzz.advance();
    }
    assert_eq!(buzz.sample(1), BuzzSample::ZERO);
    assert!(buzz.affects_axis(0));
    assert!(!buzz.affects_axis(1));
}

#[test]
fn full_run_ends_at_zero_offset_and_deactivates() {
    let mut buzz = Buzz::new();
    // 95 ms @ 10 kHz = 950 ticks ~= 11.875 periods at 125 Hz (non-integer).
    let expected_ticks = 950;
    armed(&mut buzz, 0b01, 0, 125_000, 40_000, 95, 8);
    let mut last_offset = f32::NAN;
    for _ in 0..expected_ticks {
        assert!(buzz.is_active());
        last_offset = buzz.sample(0).offset;
        buzz.advance();
    }
    // Envelope forces the final emitted offset to exactly zero regardless of
    // where the sine phase lands -> position returns to base (net-zero).
    assert_eq!(last_offset, 0.0);
    assert!(!buzz.is_active());
    assert_eq!(buzz.sample(0), BuzzSample::ZERO);
}

#[test]
fn sample_start_offset_telescopes_from_previous_tick() {
    let mut buzz = Buzz::new();
    armed(&mut buzz, 0b01, 0, 100_000, 50_000, 200, 20);
    let mut prev_emitted = 0.0f32;
    for _ in 0..400 {
        let s = buzz.sample(0);
        // This tick's sample-start must equal the previous tick's offset, so
        // per-tick dispatch deltas form a continuous chain.
        assert!((s.sample_start_offset - prev_emitted).abs() < 1e-7);
        prev_emitted = s.offset;
        buzz.advance();
    }
}

#[test]
fn re_arming_restarts_phase_and_tick() {
    let mut buzz = Buzz::new();
    armed(&mut buzz, 0b01, 0, 100_000, 50_000, 200, 20);
    for _ in 0..500 {
        buzz.advance();
    }
    // New command mid-flight: fresh seq must restart cleanly from tick 0.
    armed(&mut buzz, 0b01, 0, 80_000, 30_000, 150, 15);
    assert!(buzz.is_active());
    // tick 0 of a fresh run has zero envelope -> zero offset.
    assert_eq!(buzz.sample(0).offset, 0.0);
}
