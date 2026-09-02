use std::sync::{Arc, LazyLock};

use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;
use step_shim::ShimError;
use step_shim::ring::{SEAM_ROUNDING_CYCLES, SpanQueue};
use trajectory::{
    ClockedMotorSpan, ContinuousAxis, MotorGroup, MotorSpan, MotorTerm, NudgeProfile,
};

const MOTOR: usize = 3;
const CYCLES_PER_SECOND: f64 = 1_000_000.0;
const ANCHOR_CLOCK: u64 = 1_000_000;

/// The queue only ever reads a view's clock range, so every generated view is
/// the same trivial one-microstep nudge with its range overwritten.
static TEMPLATE: LazyLock<ClockedMotorSpan> = LazyLock::new(|| {
    let travel_mm = 0.01;
    let duration = travel_mm / 1.0;
    let profile =
        NudgeProfile::try_new(travel_mm, 1.0, 0.0, 0.0).expect("a constant-velocity nudge");
    let signal = Arc::new(
        MotorSpan::try_new(
            Arc::from([MotorGroup::Independent(MotorTerm {
                source_axis: 0,
                axis: ContinuousAxis::Nudge(profile),
                scale: 1.0,
            })]),
            0.0,
            duration,
            0,
            0,
            false,
        )
        .expect("a dispatchable motor span"),
    );
    ClockedMotorSpan::try_new(signal, 0.0, duration, 0.0, duration, 0.0, CYCLES_PER_SECOND)
        .expect("a clocked motor span")
});

fn clocked(start_clock: u64, end_clock: u64) -> ClockedMotorSpan {
    let mut view = TEMPLATE.clone();
    view.start_clock = start_clock;
    view.end_clock = end_clock;
    view
}

#[derive(Debug, Clone, PartialEq, Eq)]
enum Refusal {
    QueueFull {
        motor: usize,
    },
    Degenerate {
        motor: usize,
        start_clock: u64,
        end_clock: u64,
    },
    Gap {
        motor: usize,
        expected: u64,
        got: u64,
        tolerance: u64,
    },
    Unexpected(String),
}

fn refusal(error: &ShimError) -> Refusal {
    match *error {
        ShimError::QueueFull { motor } => Refusal::QueueFull { motor },
        ShimError::SpanClockDegenerate {
            motor,
            start_clock,
            end_clock,
        } => Refusal::Degenerate {
            motor,
            start_clock,
            end_clock,
        },
        ShimError::SpanGap {
            motor,
            expected,
            got,
            tolerance,
        } => Refusal::Gap {
            motor,
            expected,
            got,
            tolerance,
        },
        ref other => Refusal::Unexpected(format!("{other}")),
    }
}

fn outcome(result: Result<(), ShimError>) -> Result<(), Refusal> {
    result.map_err(|error| refusal(&error))
}

/// A `Vec`-backed queue driven by the same op sequence as [`SpanQueue`].
#[derive(Debug)]
struct Model {
    capacity: u32,
    views: Vec<(u64, u64)>,
    converted: u32,
    abandoned: u32,
    accepted: u32,
    seam: Option<u64>,
    anchor: u64,
}

impl Model {
    fn new(capacity: u32) -> Self {
        Self {
            capacity,
            views: Vec::new(),
            converted: 0,
            abandoned: 0,
            accepted: 0,
            seam: None,
            anchor: ANCHOR_CLOCK,
        }
    }

    fn admissible(seam: Option<u64>, start_clock: u64, end_clock: u64) -> Result<(), Refusal> {
        if end_clock <= start_clock {
            return Err(Refusal::Degenerate {
                motor: MOTOR,
                start_clock,
                end_clock,
            });
        }
        if let Some(expected) = seam {
            if start_clock.abs_diff(expected) > SEAM_ROUNDING_CYCLES {
                return Err(Refusal::Gap {
                    motor: MOTOR,
                    expected,
                    got: start_clock,
                    tolerance: SEAM_ROUNDING_CYCLES,
                });
            }
        }
        Ok(())
    }

    fn push(&mut self, start_clock: u64, end_clock: u64) -> Result<(), Refusal> {
        if self.views.len() as u32 >= self.capacity {
            return Err(Refusal::QueueFull { motor: MOTOR });
        }
        Self::admissible(self.seam, start_clock, end_clock)?;
        self.seam = Some(end_clock);
        self.anchor = end_clock;
        self.views.push((start_clock, end_clock));
        self.accepted += 1;
        Ok(())
    }

    fn validate(&self, batch: &[(u64, u64)]) -> Result<(), Refusal> {
        let mut seam = self.seam;
        for &(start_clock, end_clock) in batch {
            Self::admissible(seam, start_clock, end_clock)?;
            seam = Some(end_clock);
        }
        if self.views.len() + batch.len() > self.capacity as usize {
            return Err(Refusal::QueueFull { motor: MOTOR });
        }
        Ok(())
    }

    fn release_active(&mut self) {
        if !self.views.is_empty() {
            self.views.remove(0);
            self.converted += 1;
        }
    }

    fn abandon_all(&mut self) {
        self.abandoned += self.views.len() as u32;
        self.views.clear();
        self.seam = None;
    }

    fn accept_forward_gap(&mut self, at_start_clock: u64) -> Result<(), Refusal> {
        if let Some(expected) = self.seam {
            if at_start_clock.saturating_add(SEAM_ROUNDING_CYCLES) < expected {
                return Err(Refusal::Gap {
                    motor: MOTOR,
                    expected,
                    got: at_start_clock,
                    tolerance: SEAM_ROUNDING_CYCLES,
                });
            }
        }
        self.seam = None;
        self.anchor = at_start_clock;
        Ok(())
    }

    fn detach_seam(&mut self) -> Result<(), Refusal> {
        if !self.views.is_empty() {
            return Err(Refusal::QueueFull { motor: MOTOR });
        }
        self.seam = None;
        Ok(())
    }

    fn released(&self) -> u32 {
        self.converted + self.abandoned
    }

    fn clock_for(&self, offset: i64) -> u64 {
        self.seam
            .unwrap_or(self.anchor)
            .saturating_add_signed(offset)
    }
}

#[derive(Debug, Clone)]
enum Op {
    Push { start_offset: i64, length: i64 },
    ValidateThenPushBatch { batch: Vec<(i64, i64)> },
    Release,
    AbandonAll,
    AcceptForwardGap { start_offset: i64 },
    DetachSeam,
}

/// Offsets crowd the seam so the rounding tolerance is the common case, with
/// occasional overlaps and dwells far outside it.
fn arb_offset() -> impl Strategy<Value = i64> {
    prop_oneof![
        10 => -4i64..=4,
        3 => -600i64..600,
        1 => prop_oneof![Just(-1_000_000i64), Just(1_000_000i64)],
    ]
}

fn arb_length() -> impl Strategy<Value = i64> {
    prop_oneof![3 => -2i64..=2, 10 => 1i64..4_000]
}

fn arb_op() -> impl Strategy<Value = Op> {
    prop_oneof![
        10 => (arb_offset(), arb_length())
            .prop_map(|(start_offset, length)| Op::Push { start_offset, length }),
        4 => prop::collection::vec((arb_offset(), arb_length()), 1..4)
            .prop_map(|batch| Op::ValidateThenPushBatch { batch }),
        8 => Just(Op::Release),
        2 => Just(Op::AbandonAll),
        3 => arb_offset().prop_map(|start_offset| Op::AcceptForwardGap { start_offset }),
        2 => Just(Op::DetachSeam),
    ]
}

fn check_agreement(queue: &SpanQueue, model: &Model) -> Result<(), TestCaseError> {
    prop_assert_eq!(queue.len(), model.views.len(), "queued view count");
    prop_assert_eq!(queue.is_empty(), model.views.is_empty(), "emptiness");
    prop_assert_eq!(
        queue.released(),
        model.released(),
        "released credit: {} converted + {} abandoned",
        model.converted,
        model.abandoned
    );
    prop_assert_eq!(
        queue.len() as u32 + queue.released(),
        model.accepted,
        "every accepted view is either queued or released"
    );
    prop_assert!(
        queue.len() as u32 <= model.capacity,
        "{} views exceed the {} the ring holds",
        queue.len(),
        model.capacity
    );
    match (queue.active(), model.views.first()) {
        (Some(active), Some(&(start_clock, end_clock))) => {
            prop_assert_eq!(
                (active.start_clock, active.end_clock),
                (start_clock, end_clock),
                "the oldest unreleased view"
            );
        }
        (None, None) => {}
        (active, expected) => prop_assert!(
            false,
            "active view {:?} does not match the model's {:?}",
            active.map(|v| (v.start_clock, v.end_clock)),
            expected
        ),
    }
    Ok(())
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 512,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/span_queue_fuzz.txt",
        ))),
        ..ProptestConfig::default()
    })]

    #[test]
    fn a_queue_answers_every_op_exactly_as_the_model_does(
        capacity in 1u32..5,
        ops in prop::collection::vec(arb_op(), 1..40),
    ) {
        let mut queue = SpanQueue::new(capacity);
        let mut model = Model::new(capacity);
        check_agreement(&queue, &model)?;

        for (index, op) in ops.iter().enumerate() {
            match op {
                Op::Push { start_offset, length } => {
                    let start_clock = model.clock_for(*start_offset);
                    let end_clock = start_clock.saturating_add_signed(*length);
                    let got = outcome(queue.push(MOTOR, clocked(start_clock, end_clock)));
                    let want = model.push(start_clock, end_clock);
                    prop_assert_eq!(
                        got,
                        want,
                        "op {}: push {}..{}",
                        index,
                        start_clock,
                        end_clock
                    );
                }
                Op::ValidateThenPushBatch { batch } => {
                    let mut seam = model.seam.unwrap_or(model.anchor);
                    let mut ranges = Vec::with_capacity(batch.len());
                    for (start_offset, length) in batch {
                        let start_clock = seam.saturating_add_signed(*start_offset);
                        let end_clock = start_clock.saturating_add_signed(*length);
                        seam = end_clock;
                        ranges.push((start_clock, end_clock));
                    }
                    let views: Vec<ClockedMotorSpan> = ranges
                        .iter()
                        .map(|&(start_clock, end_clock)| clocked(start_clock, end_clock))
                        .collect();

                    let got = outcome(queue.validate(MOTOR, &views));
                    let want = model.validate(&ranges);
                    prop_assert_eq!(&got, &want, "op {}: validate {:?}", index, ranges);

                    if got.is_ok() {
                        for (&(start_clock, end_clock), view) in ranges.iter().zip(&views) {
                            let pushed = outcome(queue.push(MOTOR, view.clone()));
                            let modelled = model.push(start_clock, end_clock);
                            prop_assert_eq!(
                                &pushed,
                                &modelled,
                                "op {}: a validated batch must push",
                                index
                            );
                            prop_assert!(
                                pushed.is_ok(),
                                "op {}: validate accepted {}..{} but the push refused it: {:?}",
                                index,
                                start_clock,
                                end_clock,
                                pushed
                            );
                        }
                    }
                }
                Op::Release => {
                    queue.release_active();
                    model.release_active();
                }
                Op::AbandonAll => {
                    queue.abandon_all();
                    model.abandon_all();
                }
                Op::AcceptForwardGap { start_offset } => {
                    let at = model.clock_for(*start_offset);
                    let got = outcome(queue.accept_forward_gap(MOTOR, at));
                    let want = model.accept_forward_gap(at);
                    prop_assert_eq!(got, want, "op {}: accept forward gap at {}", index, at);
                }
                Op::DetachSeam => {
                    let got = outcome(queue.detach_seam(MOTOR));
                    let want = model.detach_seam();
                    prop_assert_eq!(got, want, "op {}: detach seam", index);
                }
            }
            check_agreement(&queue, &model)?;
        }
    }

    /// A batch is admitted whole or not at all: `validate` must accept exactly
    /// the batches whose views all push, and must report a malformed view even
    /// when the batch also overflows the ring.
    #[test]
    fn validate_accepts_exactly_the_batches_that_push_one_by_one(
        capacity in 1u32..5,
        preloaded in 0usize..5,
        batch in prop::collection::vec((arb_offset(), arb_length()), 1..6),
    ) {
        let mut queue = SpanQueue::new(capacity);
        let mut model = Model::new(capacity);
        let mut clock = ANCHOR_CLOCK;
        for _ in 0..preloaded {
            let end_clock = clock + 500;
            if queue.push(MOTOR, clocked(clock, end_clock)).is_ok() {
                model.push(clock, end_clock).expect("the model agrees");
            }
            clock = end_clock;
        }

        let mut seam = model.seam.unwrap_or(model.anchor);
        let mut ranges = Vec::with_capacity(batch.len());
        for (start_offset, length) in &batch {
            let start_clock = seam.saturating_add_signed(*start_offset);
            let end_clock = start_clock.saturating_add_signed(*length);
            seam = end_clock;
            ranges.push((start_clock, end_clock));
        }
        let views: Vec<ClockedMotorSpan> = ranges
            .iter()
            .map(|&(start_clock, end_clock)| clocked(start_clock, end_clock))
            .collect();

        let validated = outcome(queue.validate(MOTOR, &views));
        let queued_before = queue.len();

        let mut refused = None;
        let mut accepted = 0usize;
        for view in &views {
            match outcome(queue.push(MOTOR, view.clone())) {
                Ok(()) => accepted += 1,
                Err(error) => {
                    refused = Some(error);
                    break;
                }
            }
        }

        prop_assert_eq!(
            validated.is_ok(),
            refused.is_none(),
            "validate said {:?} but pushing one by one said {:?}",
            validated,
            refused
        );
        if validated.is_ok() {
            prop_assert_eq!(accepted, views.len());
            prop_assert_eq!(queue.len(), queued_before + views.len());
        } else {
            let malformed = ranges
                .iter()
                .any(|&(start_clock, end_clock)| end_clock <= start_clock);
            let reported_full = validated == Err(Refusal::QueueFull { motor: MOTOR });
            prop_assert!(
                !(malformed && reported_full),
                "a malformed view must outrank a full ring: {:?}",
                ranges
            );
        }
    }
}
