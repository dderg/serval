//! The settled lookahead (#860) only moves decisions earlier: on any
//! stream the fit stage and the planner emit exactly what the counted
//! lookahead emits, move for move and bit for bit.

use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;

use super::hold_time_tests::{BenchChain, Step, line};
use crate::fit_stage::Lookahead;
use crate::types::PlannedItem;

const BED_LO_MM: f64 = 20.0;
const BED_HI_MM: f64 = 280.0;
const LAYER_Z_MM: f64 = 0.3;
const STREAM_STEPS: usize = 48;
const FIRST_LINE: usize = 3;
const EXTRUDE_PER_MM: f64 = 0.035;

/// One stretch of a generated stream, what slicers emit.
#[derive(Debug, Clone)]
enum Stretch {
    Dense {
        count: usize,
        length: f64,
        turn: f64,
    },
    Sparse {
        length: f64,
        turn: f64,
    },
    Travel {
        length: f64,
        turn: f64,
    },
    Reversal {
        length: f64,
        offset: f64,
    },
    Arc {
        facets: usize,
        radius: f64,
        sweep: f64,
    },
    Retract {
        de: f64,
    },
    Accel {
        cap: Option<f64>,
    },
    Feed {
        mm_s: f64,
    },
    Drain,
}

fn stretch() -> impl Strategy<Value = Stretch> {
    prop_oneof![
        (2_usize..12, 0.1_f64..2.0, -0.6_f64..0.6).prop_map(|(count, length, turn)| {
            Stretch::Dense {
                count,
                length,
                turn,
            }
        }),
        (15.0_f64..200.0, -3.1_f64..3.1)
            .prop_map(|(length, turn)| Stretch::Sparse { length, turn }),
        (5.0_f64..200.0, -3.1_f64..3.1).prop_map(|(length, turn)| Stretch::Travel { length, turn }),
        (10.0_f64..150.0, -0.6_f64..0.6)
            .prop_map(|(length, offset)| Stretch::Reversal { length, offset }),
        (3_usize..14, 2.0_f64..400.0, 0.02_f64..0.4).prop_map(|(facets, radius, sweep)| {
            Stretch::Arc {
                facets,
                radius,
                sweep,
            }
        }),
        (0.2_f64..3.0).prop_map(|de| Stretch::Retract { de }),
        prop::option::of(1000.0_f64..10_000.0).prop_map(|cap| Stretch::Accel { cap }),
        (5.0_f64..400.0).prop_map(|mm_s| Stretch::Feed { mm_s }),
        Just(Stretch::Drain),
    ]
}

/// Lays stretches out as G-code lines on the bed, turning back from its
/// edges, numbered as a file.
struct Layout {
    at: [f64; 3],
    heading: f64,
    feed: f64,
    accel: Option<f64>,
    steps: Vec<Step>,
}

impl Layout {
    fn new() -> Self {
        Self {
            at: [BED_LO_MM, BED_LO_MM, LAYER_Z_MM],
            heading: 0.0,
            feed: 100.0,
            accel: None,
            steps: Vec::new(),
        }
    }

    fn go(&mut self, length: f64, de: f64) {
        let mut end = self.at;
        for _ in 0..4 {
            end = [
                self.at[0] + length * self.heading.cos(),
                self.at[1] + length * self.heading.sin(),
                self.at[2],
            ];
            let inside = (BED_LO_MM..=BED_HI_MM).contains(&end[0])
                && (BED_LO_MM..=BED_HI_MM).contains(&end[1]);
            if inside {
                break;
            }
            self.heading += std::f64::consts::FRAC_PI_2;
        }
        let end = [
            end[0].clamp(BED_LO_MM, BED_HI_MM),
            end[1].clamp(BED_LO_MM, BED_HI_MM),
            end[2],
        ];
        self.push(end, de);
    }

    fn push(&mut self, end: [f64; 3], de: f64) {
        let seq = FIRST_LINE + self.steps.len();
        let gcode = line(seq, self.at, end, de, self.feed);
        self.steps.push(Step::Line(gcode, self.accel));
        self.at = end;
    }

    fn lay(&mut self, stretch: &Stretch) {
        match *stretch {
            Stretch::Dense {
                count,
                length,
                turn,
            } => {
                for _ in 0..count {
                    self.heading += turn;
                    self.go(length, EXTRUDE_PER_MM * length);
                }
            }
            Stretch::Sparse { length, turn } => {
                self.heading += turn;
                self.go(length, EXTRUDE_PER_MM * length);
            }
            Stretch::Travel { length, turn } => {
                self.heading += turn;
                self.go(length, 0.0);
            }
            Stretch::Reversal { length, offset } => {
                self.go(length, EXTRUDE_PER_MM * length);
                self.heading += std::f64::consts::PI + 1e-3 * offset;
                self.go(length, EXTRUDE_PER_MM * length);
            }
            Stretch::Arc {
                facets,
                radius,
                sweep,
            } => {
                let chord = 2.0 * radius * (0.5 * sweep / facets as f64).sin();
                for _ in 0..facets {
                    self.heading += sweep / facets as f64;
                    self.go(chord, EXTRUDE_PER_MM * chord);
                }
            }
            Stretch::Retract { de } => {
                self.push(self.at, -de);
                self.push(self.at, de);
            }
            Stretch::Accel { cap } => self.accel = cap,
            Stretch::Feed { mm_s } => self.feed = mm_s,
            Stretch::Drain => self.steps.push(Step::Drain),
        }
    }
}

fn assert_same(settled: &[PlannedItem], counted: &[PlannedItem]) -> Result<(), TestCaseError> {
    prop_assert_eq!(
        settled.len(),
        counted.len(),
        "a different number of planned items"
    );
    for (index, (a, b)) in settled.iter().zip(counted).enumerate() {
        match (a, b) {
            (PlannedItem::Move(a), PlannedItem::Move(b)) => {
                prop_assert_eq!(
                    &a.geometry,
                    &b.geometry,
                    "move {} differs in geometry",
                    index
                );
                prop_assert_eq!(
                    &a.velocity,
                    &b.velocity,
                    "move {} differs in velocity",
                    index
                );
                prop_assert_eq!(&a.budget, &b.budget, "move {} differs in budget", index);
            }
            (PlannedItem::Drain, PlannedItem::Drain) => {}
            _ => prop_assert!(false, "item {} differs in kind", index),
        }
    }
    Ok(())
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 64,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/lookahead_tests.txt",
        ))),
        ..ProptestConfig::default()
    })]

    /// Dense facets, long lines, travels, reversals, facetted arcs,
    /// retractions, acceleration and feedrate changes and drains, on the
    /// bench printer's chain (shaped corners, blends, extrusion caps).
    #[test]
    fn the_settled_lookahead_plans_what_the_counted_one_plans(
        stretches in prop::collection::vec(stretch(), 1..STREAM_STEPS),
    ) {
        let mut layout = Layout::new();
        for stretch in &stretches {
            layout.lay(stretch);
        }
        let chain = BenchChain::new();
        let settled = chain.plan_with(&layout.steps, Lookahead::Settled);
        let counted = chain.plan_with(&layout.steps, Lookahead::Counted);
        assert_same(&settled, &counted)?;
    }
}
