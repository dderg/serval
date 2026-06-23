use super::TickChain;

const FREQ: f64 = 84_000_000.0;

#[test]
fn seam_is_continuous_across_segments() {
    let mut c = TickChain::new(0.0);
    c.anchor(0, 1_000_000);
    // segment A: 5 ms, two pieces (offsets 0 and 2.5 ms).
    assert_eq!(c.piece_tick(0, 0.0, FREQ), Some(1_000_000));
    let a_dur = 0.005;
    c.advance(0, a_dur, FREQ);
    // segment B's first piece (offset 0) starts exactly where A ended.
    let a_end = 1_000_000 + (a_dur * FREQ) as u64;
    assert_eq!(c.piece_tick(0, 0.0, FREQ), Some(a_end));
}

#[test]
fn anchor_does_not_depend_on_absolute_jitter() {
    // Re-projecting would shift the seam by the estimate jitter; chaining ignores
    // the live absolute entirely between anchors — the seam tick is fixed by the
    // previous segment's end regardless of what the absolute projection now says.
    let mut c = TickChain::new(0.0);
    c.anchor(0, 1_000_000);
    c.advance(0, 0.005, FREQ);
    let seam = c.piece_tick(0, 0.0, FREQ).unwrap();
    // A wildly different "live absolute" arrives — with zero slew the seam holds.
    c.slew(0, 9_999_999_999, FREQ);
    assert_eq!(c.piece_tick(0, 0.0, FREQ), Some(seam));
}

#[test]
fn slew_is_bounded_per_seam() {
    let mut c = TickChain::new(1e-6); // 1 µs budget => 84 ticks at 84 MHz
    c.anchor(0, 1_000_000);
    c.slew(0, 1_000_000 + 10_000, FREQ); // wants +10000, capped to +84
    assert_eq!(c.piece_tick(0, 0.0, FREQ), Some(1_000_084));
    c.slew(0, 0, FREQ); // wants large negative, capped to -84
    assert_eq!(c.piece_tick(0, 0.0, FREQ), Some(1_000_000));
}

#[test]
fn each_mcu_chains_independently() {
    let mut c = TickChain::new(0.0);
    c.anchor(0, 1_000_000);
    c.anchor(1, 5_000_000);
    let f0 = 84_000_000.0;
    let f1 = 72_000_000.0; // a different crystal — chains at its own rate
    c.advance(0, 0.005, f0);
    c.advance(1, 0.005, f1);
    assert_eq!(
        c.piece_tick(0, 0.0, f0),
        Some(1_000_000 + (0.005 * f0) as u64)
    );
    assert_eq!(
        c.piece_tick(1, 0.0, f1),
        Some(5_000_000 + (0.005 * f1) as u64)
    );
}

#[test]
fn unanchored_mcu_yields_none_and_slew_is_noop() {
    let mut c = TickChain::new(1e-6);
    assert_eq!(c.piece_tick(7, 0.0, FREQ), None);
    c.slew(7, 1_000, FREQ); // no anchor yet — must not create one
    assert!(!c.is_anchored(7));
}

#[test]
fn fresh_reanchor_replaces_drifted_chain() {
    let mut c = TickChain::new(0.0);
    c.anchor(0, 1_000_000);
    c.advance(0, 1.0, FREQ); // drift far ahead
    c.anchor(0, 2_000_000); // underrun/fresh: hard re-anchor
    assert_eq!(c.piece_tick(0, 0.0, FREQ), Some(2_000_000));
}
