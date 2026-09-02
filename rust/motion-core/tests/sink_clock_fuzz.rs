//! The stepcompress endpoint's two clock maps: `McuClock`'s seconds<->ticks
//! conversion, which paces every send window, and `StepBuzz::clock_at`, the
//! single anchored map every resonance-buzz chunk is projected from.

use motion_core::pump::clock_probe::{buzz_clock_at, mcu_secs, mcu_ticks};
use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;
use step_shim::ring::SEAM_ROUNDING_CYCLES;
use trajectory::MAX_SPAN_SECS;

/// Every mcu step clock the benches run, from the slowest simulated part to
/// the H723's 520 MHz.
fn arb_freq() -> impl Strategy<Value = f64> {
    prop_oneof![
        Just(1.0e6),
        Just(64.0e6),
        Just(72.0e6),
        Just(168.0e6),
        Just(400.0e6),
        Just(480.0e6),
        Just(520.0e6),
        1.0e6..520.0e6,
    ]
}

/// A pacing window, from a sub-microsecond margin to a day of uptime.
fn arb_secs() -> impl Strategy<Value = f64> {
    prop_oneof![
        Just(0.0),
        1e-9..1e-6,
        1e-6..1e-3,
        1e-3..1.0,
        1.0..1e2,
        1e2..1e5,
    ]
}

/// Anchors from a freshly booted mcu up to days of 520 MHz uptime, where one
/// tick is already coarser than an ulp of the anchor.
fn arb_anchor() -> impl Strategy<Value = f64> {
    prop_oneof![1.0..1e6, 1e6..1e9, 1e9..1e12, 1e12..1e14]
}

fn rounded_product(freq: f64, secs: f64) -> f64 {
    libm::round(freq * secs)
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 512,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/sink_clock_fuzz.txt",
        ))),
        ..ProptestConfig::default()
    })]

    /// A longer window can never buy fewer ticks — the drain cutoff, the
    /// barrier deadline and the send lead are all derived by widening a
    /// window, and a non-monotone conversion would shrink one of them.
    #[test]
    fn ticks_never_shrink_as_the_window_widens(
        freq in arb_freq(),
        a in arb_secs(),
        b in arb_secs(),
    ) {
        let (short, long) = (a.min(b), a.max(b));
        prop_assert!(
            mcu_ticks(freq, short) <= mcu_ticks(freq, long),
            "{short} s -> {} ticks but {long} s -> {} ticks at {freq} Hz",
            mcu_ticks(freq, short),
            mcu_ticks(freq, long)
        );
    }

    /// A zero window is zero ticks: `lead_horizon` and the probe interval both
    /// add the conversion to a live clock, so any offset at zero would shift
    /// the mcu's whole pacing frame.
    #[test]
    fn a_zero_window_is_zero_ticks(freq in arb_freq()) {
        prop_assert_eq!(mcu_ticks(freq, 0.0), 0);
    }

    /// The conversion truncates, so it must land on the tick below the exact
    /// product and never further than one tick from it.
    #[test]
    fn ticks_land_within_one_tick_of_the_exact_product(
        freq in arb_freq(),
        secs in arb_secs(),
    ) {
        let ticks = mcu_ticks(freq, secs) as f64;
        let exact = rounded_product(freq, secs);
        prop_assert!(
            (ticks - exact).abs() <= 1.0,
            "{secs} s at {freq} Hz gave {ticks} ticks, {exact} rounded"
        );
        prop_assert!(
            ticks <= freq * secs,
            "{secs} s at {freq} Hz overshot the exact product"
        );
    }

    /// `flush` measures elapsed work by converting a tick delta to seconds and
    /// back through the same clock; the round trip may lose the truncated
    /// remainder and nothing more.
    #[test]
    fn a_tick_count_survives_the_round_trip_through_seconds(
        freq in arb_freq(),
        ticks in 0u64..100_000_000_000_000,
    ) {
        let recovered = mcu_ticks(freq, mcu_secs(freq, ticks));
        prop_assert!(
            recovered == ticks || recovered + 1 == ticks,
            "{ticks} ticks at {freq} Hz came back as {recovered}"
        );
    }

    /// Every buzz chunk is projected from one anchor, so the map must be
    /// exact at that anchor and never run backwards.
    #[test]
    fn the_buzz_map_pins_its_anchor_and_never_runs_backwards(
        anchor in arb_anchor(),
        cycles_per_second in arb_freq(),
        origin in 0.0f64..1e3,
        a in 0.0f64..10.0,
        b in 0.0f64..10.0,
    ) {
        prop_assert_eq!(
            buzz_clock_at(anchor, cycles_per_second, origin, origin),
            anchor,
            "the map must pin its own anchor"
        );
        let (early, late) = (origin + a.min(b), origin + a.max(b));
        prop_assert!(
            buzz_clock_at(anchor, cycles_per_second, origin, early)
                <= buzz_clock_at(anchor, cycles_per_second, origin, late),
            "stream {early} s projected past stream {late} s"
        );
    }

    /// `generate_buzz` walks the stream in `MAX_SPAN_SECS` chunks and clocks
    /// each one from the anchor alone. Every chunk must therefore span at
    /// least one tick — a chunk that does not is rejected by
    /// `ClockedMotorSpan::try_new` and fails the whole buzz.
    #[test]
    fn every_buzz_chunk_spans_at_least_one_tick(
        anchor in arb_anchor(),
        cycles_per_second in arb_freq(),
        origin in 0.0f64..1e3,
        chunk in 0usize..400,
    ) {
        let start = origin + chunk as f64 * MAX_SPAN_SECS;
        let end = start + MAX_SPAN_SECS;
        let start_clock = libm::round(buzz_clock_at(anchor, cycles_per_second, origin, start));
        let end_clock = libm::round(buzz_clock_at(anchor, cycles_per_second, origin, end));
        prop_assert!(
            end_clock > start_clock,
            "chunk {chunk} spans no tick: {start_clock} -> {end_clock} at {cycles_per_second} Hz \
             from anchor {anchor}"
        );
    }

    /// Chunk k's end clock and chunk k+1's start clock are two roundings of
    /// one shared stream instant, which is exactly what the shim's seam
    /// admission budgets. Anchored on the one map they stay inside that
    /// budget at every mcu uptime; a chain that re-derived its origin from
    /// each rounded chunk would walk out of it.
    #[test]
    fn consecutive_buzz_chunks_share_their_seam_tick(
        anchor in arb_anchor(),
        cycles_per_second in arb_freq(),
        origin in 0.0f64..1e3,
        chunk in 0usize..400,
    ) {
        let start = origin + chunk as f64 * MAX_SPAN_SECS;
        let next = origin + (chunk + 1) as f64 * MAX_SPAN_SECS;
        let start_exact = buzz_clock_at(anchor, cycles_per_second, origin, start);
        let end_clock = libm::round(start_exact + (next - start) * cycles_per_second);
        let next_clock = libm::round(buzz_clock_at(anchor, cycles_per_second, origin, next));
        prop_assert!(
            (end_clock - next_clock).abs() <= SEAM_ROUNDING_CYCLES as f64,
            "chunk {} ends at {} but chunk {} starts at {}, past the {} tick seam budget",
            chunk,
            end_clock,
            chunk + 1,
            next_clock,
            SEAM_ROUNDING_CYCLES
        );
    }
}
