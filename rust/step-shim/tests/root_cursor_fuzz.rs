use std::sync::Arc;

use nurbs::ScalarNurbs;
use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;
use step_shim::ring::SpanQueue;
use step_shim::root_cursor::{EVAL_COUNT, StepRoot, StepRootCursor};
use step_shim::{MotorConfig, StepEncoder};
use trajectory::{ClockedMotorSpan, ContinuousAxis, MotorGroup, MotorSpan, MotorTerm};

const MIN_CLOCKS: u64 = 1_000;
const MAX_CLOCKS: u64 = 60_000;
const MAX_DEGREE: usize = 3;
const MAX_SEGMENTS: usize = 3;
const MIN_LOG10_AMPLITUDE_MM: f64 = -14.0;
const SOURCE_AXIS: usize = 2;

/// How the cursor learns where the rotor sits before the first root.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Anchor {
    /// The first evaluated position becomes the lattice origin.
    Fresh,
    /// The counter is reseeded so the absolute lattice holds the position.
    Reseeded,
    /// A motor-local signal opens an overlay lattice at its own origin.
    Overlay,
}

/// A position placed against a lattice threshold, to the last bit the
/// coordinate can express. The lattice is hysteresis-free: a step fires the
/// moment a position *reaches* a threshold, so `Exact` owes that step,
/// `UlpShort` owes nothing, and `UlpPast` owes the same one step. At a base
/// coordinate of hundreds of millimetres an ulp is ten orders of magnitude
/// under a microstep, which is where a threshold comparison and a
/// difference-against-nominal comparison stop agreeing.
#[derive(Debug, Clone, Copy)]
enum Offset {
    Exact(i64),
    UlpShort(i64),
    UlpPast(i64),
}

impl Offset {
    fn level(self) -> i64 {
        match self {
            Self::Exact(level) | Self::UlpShort(level) | Self::UlpPast(level) => level,
        }
    }
}

#[derive(Debug, Clone)]
struct Case {
    clock_freq_hz: f64,
    clocks: u64,
    microstep_mm: f64,
    base_mm: f64,
    degree: usize,
    segments: usize,
    amplitude_mm: f64,
    offsets: Vec<f64>,
    anchor: Anchor,
    /// Where the run's own last position lands on the lattice it started on.
    landing: Option<Offset>,
    /// Where a hold following the run parks on the lattice the run leaves
    /// behind. Only ever within one microstep of nominal - two is drift, and
    /// drift is a separate, deliberately fatal contract.
    rest: Option<Offset>,
    cuts: Vec<u64>,
}

impl Case {
    fn duration(&self) -> f64 {
        self.clocks as f64 / self.clock_freq_hz
    }

    fn motor_mask(&self) -> u8 {
        match self.anchor {
            Anchor::Overlay => 1,
            Anchor::Fresh | Anchor::Reseeded => 0,
        }
    }

    fn run_signal(&self) -> Arc<MotorSpan> {
        let duration = self.duration();
        let order = self.degree + 1;
        let mut knots = vec![0.0; order];
        knots.extend((1..self.segments).map(|i| duration * i as f64 / self.segments as f64));
        knots.extend(std::iter::repeat_n(duration, order));
        let mut control_points = self
            .offsets
            .iter()
            .map(|u| self.base_mm + self.amplitude_mm * u)
            .collect::<Vec<f64>>();
        assert_eq!(control_points.len(), knots.len() - order);
        if let Some(landing) = self.landing {
            let start = Lattice::for_anchor(self.anchor, self.microstep_mm, control_points[0]);
            *control_points.last_mut().expect("a clamped spline") =
                start.offset_position(self.microstep_mm, landing);
        }
        let curve = ScalarNurbs::try_new(self.degree as u8, knots, control_points)
            .expect("a clamped spline");
        self.span(
            MotorGroup::Spline {
                curve: Arc::new(curve),
                summed_scale: 1.0,
            },
            0.0,
            duration,
        )
    }

    fn span(&self, group: MotorGroup, t_start: f64, t_end: f64) -> Arc<MotorSpan> {
        Arc::new(
            MotorSpan::try_new(
                Arc::from([group]),
                t_start,
                t_end,
                self.motor_mask(),
                0,
                false,
            )
            .expect("a dispatchable motor span"),
        )
    }

    fn clocked(&self, signal: Arc<MotorSpan>, t_start: f64, t_end: f64) -> ClockedMotorSpan {
        ClockedMotorSpan::try_new(
            signal,
            t_start,
            t_end,
            t_start,
            t_end,
            t_start * self.clock_freq_hz,
            self.clock_freq_hz,
        )
        .expect("a clocked motor span")
    }

    /// The run, optionally followed by a hold parked against the lattice the
    /// run ends on. The hold's rest position is read off the reference walk,
    /// so the case is built to sit exactly where the cursor's own bookkeeping
    /// will be when the hold opens.
    fn views(&self) -> Vec<ClockedMotorSpan> {
        let duration = self.duration();
        let run = self.clocked(self.run_signal(), 0.0, duration);
        let Some(rest) = self.rest else {
            return vec![run];
        };
        let after_run = walk_lattice(self, std::slice::from_ref(&run)).0;
        let position = after_run.offset_position(self.microstep_mm, rest);
        let hold = self.span(
            MotorGroup::Independent(MotorTerm {
                source_axis: SOURCE_AXIS,
                axis: ContinuousAxis::Hold {
                    position,
                    t_start: duration,
                    t_end: 2.0 * duration,
                },
                scale: 1.0,
            }),
            duration,
            2.0 * duration,
        );
        vec![run, self.clocked(hold, duration, 2.0 * duration)]
    }

    fn config(&self) -> MotorConfig {
        MotorConfig {
            oid: 0,
            microstep_distance: self.microstep_mm,
            invert_dir: false,
            cycles_per_second: self.clock_freq_hz,
            encoder: StepEncoder::Classic { max_error_ticks: 0 },
            min_rearm_cycles: 0,
        }
    }
}

/// The fastest the spline can move is `degree * max|dP| / min knot span`;
/// keep that under one microstep per clock so no two roots share a clock.
fn max_amplitude_mm(clocks: u64, microstep_mm: f64, degree: usize, segments: usize) -> f64 {
    clocks as f64 * microstep_mm / (4.0 * degree as f64 * segments as f64)
}

fn arb_offset(levels: std::ops::RangeInclusive<i64>) -> impl Strategy<Value = Offset> {
    (levels, 0..3usize, any::<bool>()).prop_map(|(level, kind, negate)| {
        let level = if negate { -level } else { level };
        match kind {
            0 => Offset::Exact(level),
            1 => Offset::UlpShort(level),
            _ => Offset::UlpPast(level),
        }
    })
}

fn arb_case() -> impl Strategy<Value = Case> {
    let freq = prop_oneof![
        Just(1_000_000.0),
        Just(16_000_000.0),
        Just(72_000_000.0),
        Just(180_000_000.0),
        Just(520_000_000.0),
    ];
    let microstep = prop_oneof![
        Just(0.0025),
        Just(0.005),
        Just(0.008),
        Just(0.01),
        Just(0.0125),
        0.001..0.02,
    ];
    let anchor = prop_oneof![
        Just(Anchor::Fresh),
        Just(Anchor::Reseeded),
        Just(Anchor::Overlay)
    ];
    let landing = prop_oneof![
        2 => Just(None),
        3 => arb_offset(1..=3).prop_map(Some),
    ];
    (
        freq,
        MIN_CLOCKS..=MAX_CLOCKS,
        microstep,
        -350.0..350.0,
        1..=MAX_DEGREE,
        1..=MAX_SEGMENTS,
        anchor,
        landing,
    )
        .prop_flat_map(
            |(clock_freq_hz, clocks, microstep_mm, base_mm, degree, segments, anchor, landing)| {
                let control_points = degree + segments;
                let amplitude_cap = max_amplitude_mm(clocks, microstep_mm, degree, segments);
                let log_amplitude = MIN_LOG10_AMPLITUDE_MM..libm::log10(amplitude_cap);
                (
                    log_amplitude,
                    prop::collection::vec(-1.0..=1.0, control_points),
                    prop::collection::vec((1..clocks, any::<bool>()), 1..4),
                    any::<bool>(),
                    arb_rest(anchor),
                )
                    .prop_map(
                        move |(log_amplitude, offsets, cut_pairs, cut_before_last, rest)| {
                            let mut cuts = cut_pairs
                                .into_iter()
                                .flat_map(|(clock, adjacent)| {
                                    [Some(clock), adjacent.then_some(clock + 1)]
                                })
                                .flatten()
                                .chain(cut_before_last.then_some(clocks - 1))
                                .filter(|&clock| clock < clocks)
                                .collect::<Vec<u64>>();
                            cuts.sort_unstable();
                            cuts.dedup();
                            Case {
                                clock_freq_hz,
                                clocks,
                                microstep_mm,
                                base_mm,
                                degree,
                                segments,
                                amplitude_mm: libm::pow(10.0, log_amplitude),
                                offsets,
                                anchor,
                                landing,
                                rest,
                                cuts,
                            }
                        },
                    )
            },
        )
}

/// A hold is a second signal, and a second signal opens its own overlay
/// lattice at its own origin - there is no carried lattice left for it to owe
/// a step to, so the trailing hold only means something on the lane lattice.
fn arb_rest(anchor: Anchor) -> impl Strategy<Value = Option<Offset>> {
    if anchor == Anchor::Overlay {
        return Just(None).boxed();
    }
    prop_oneof![
        1 => Just(None),
        3 => arb_offset(0..=1).prop_map(Some),
    ]
    .boxed()
}

struct Lattice {
    origin_mm: f64,
    step_count: i64,
}

impl Lattice {
    fn for_anchor(anchor: Anchor, microstep_mm: f64, start_position: f64) -> Self {
        match anchor {
            Anchor::Fresh | Anchor::Overlay => Self {
                origin_mm: start_position,
                step_count: 0,
            },
            Anchor::Reseeded => Self {
                origin_mm: 0.0,
                step_count: (start_position / microstep_mm).round() as i64,
            },
        }
    }

    fn threshold(&self, microstep_mm: f64, advance: i64) -> f64 {
        self.origin_mm + (self.step_count + advance) as f64 * microstep_mm
    }

    fn offset_position(&self, microstep_mm: f64, offset: Offset) -> f64 {
        let level = self.threshold(microstep_mm, offset.level());
        let away = level + offset.level() as f64;
        match offset {
            Offset::Exact(_) => level,
            Offset::UlpShort(_) => libm::nextafter(level, self.threshold(microstep_mm, 0)),
            Offset::UlpPast(_) => libm::nextafter(level, away),
        }
    }
}

/// Every clock of every view, in order, against the hysteresis-free lattice
/// the cursor walks: a step fires the first clock the position reaches the
/// next threshold in either direction.
fn walk_lattice(case: &Case, views: &[ClockedMotorSpan]) -> (Lattice, Vec<StepRoot>) {
    let microstep_mm = case.microstep_mm;
    let first = views.first().expect("at least the run");
    let mut lattice = Lattice::for_anchor(
        case.anchor,
        microstep_mm,
        first
            .position_at_clock(first.start_clock)
            .expect("the view start"),
    );
    let mut roots = Vec::new();
    for view in views {
        let position = |clock: u64| view.position_at_clock(clock).expect("a clock in the view");
        for clock in view.start_clock + 1..=view.end_clock {
            let p = position(clock);
            let advance = if p >= lattice.threshold(microstep_mm, 1) {
                1
            } else if p <= lattice.threshold(microstep_mm, -1) {
                -1
            } else {
                continue;
            };
            lattice.step_count += advance;
            assert!(
                p < lattice.threshold(microstep_mm, 1) && p > lattice.threshold(microstep_mm, -1),
                "the generator let the signal cross two lattice levels in one clock"
            );
            roots.push(StepRoot {
                clock,
                dir: u8::from(advance > 0),
                advance: advance as i8,
            });
        }
    }
    (lattice, roots)
}

fn reference_roots(case: &Case, views: &[ClockedMotorSpan]) -> Vec<StepRoot> {
    walk_lattice(case, views).1
}

fn seeded_cursor(case: &Case, views: &[ClockedMotorSpan]) -> StepRootCursor {
    let config = case.config();
    let mut cursor = StepRootCursor::new(&config);
    if let Anchor::Reseeded = case.anchor {
        let first = views.first().expect("at least the run");
        let start_position = first
            .position_at_clock(first.start_clock)
            .expect("the view start");
        cursor.reset_to((start_position / case.microstep_mm).round() as i64, 0);
    }
    cursor
}

fn drain_in_chunks(case: &Case, views: &[ClockedMotorSpan], cuts: &[u64]) -> Vec<StepRoot> {
    let config = case.config();
    let mut queue = SpanQueue::new(views.len() as u32);
    for view in views {
        queue.push(0, view.clone()).expect("an admissible view");
    }
    let mut cursor = seeded_cursor(case, views);
    let mut roots = Vec::new();
    for &up_to_clock in cuts.iter().chain(std::iter::once(&u64::MAX)) {
        cursor
            .advance(0, &config, &mut queue, up_to_clock, &mut roots, None)
            .expect("a drainable signal");
    }
    assert!(queue.is_empty(), "every view must be consumed");
    roots
}

fn evaluations_during(f: impl FnOnce()) -> u64 {
    let before = EVAL_COUNT.with(std::cell::Cell::get);
    f();
    EVAL_COUNT.with(std::cell::Cell::get) - before
}

/// A monotonic run costs a bounded regula-falsi bracket per root; a genuine
/// reversal costs one halving chain down to a single clock, which is at most
/// `log2(clocks)` windows deep with a handful of evaluations each.
fn evaluation_budget(case: &Case, roots: usize) -> u64 {
    let halvings = u64::from(case.clocks.ilog2()) + 2;
    let reversals = (case.degree * case.segments) as u64;
    8 * roots as u64 + 24 * halvings * reversals + 64
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 256,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/root_cursor_fuzz.txt",
        ))),
        ..ProptestConfig::default()
    })]

    #[test]
    fn every_root_is_the_first_clock_reaching_its_lattice_level(case in arb_case()) {
        let views = case.views();
        let expected = reference_roots(&case, &views);

        let mut roots = Vec::new();
        let evaluations = evaluations_during(|| {
            roots = drain_in_chunks(&case, &views, &[]);
        });

        prop_assert_eq!(&roots, &expected);
        let budget = evaluation_budget(&case, expected.len());
        prop_assert!(
            evaluations <= budget,
            "{evaluations} evaluations for {} roots exceed the {budget} budget",
            expected.len()
        );
    }

    #[test]
    fn a_drain_split_at_arbitrary_clocks_emits_the_same_roots(case in arb_case()) {
        let views = case.views();

        let whole = drain_in_chunks(&case, &views, &[]);
        let chunked = drain_in_chunks(&case, &views, &case.cuts);

        prop_assert_eq!(chunked, whole);
    }
}
