use std::sync::Arc;

use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;
use step_shim::compress::{DEFAULT_MAX_ERROR_TICKS, StepMove};
use step_shim::{MotorConfig, StepEncoder, StepFrame, StepShim};
use trajectory::{
    ClockedMotorSpan, ContinuousAxis, MotorGroup, MotorSpan, MotorTerm, NudgeProfile,
};

const FREQ: f64 = 1_000_000.0;
const ANCHOR_CLOCK: u64 = 1_000;
const QUEUE_DEPTH: u32 = 16;
const MAX_VIEWS: usize = 6;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Root {
    clock: u64,
    dir: u8,
    advance: i8,
}

/// One motor's stream: a single monotonic nudge, cut into contiguous clocked
/// views. `spacing_cycles` is the clocks per microstep travelled, which keeps
/// the signal slower than one lattice level per clock.
#[derive(Debug, Clone)]
struct Lane {
    oid: u32,
    microstep_mm: f64,
    invert_dir: bool,
    encoder: StepEncoder,
    microsteps: u64,
    lattice_phase: f64,
    spacing_cycles: u64,
    accel_cycles: u64,
    backwards: bool,
    cuts: Vec<u64>,
}

struct Stream {
    cfg: MotorConfig,
    views: Vec<ClockedMotorSpan>,
    roots: Vec<Root>,
    end_position: f64,
}

impl Lane {
    fn travel_mm(&self) -> f64 {
        let magnitude = (self.microsteps as f64 + self.lattice_phase) * self.microstep_mm;
        if self.backwards {
            -magnitude
        } else {
            magnitude
        }
    }

    fn profile(&self) -> NudgeProfile {
        let travel_mm = self.travel_mm();
        let cruise_seconds = (self.microsteps + 1) as f64 * self.spacing_cycles as f64 / FREQ;
        let speed_mm_s = travel_mm.abs() / cruise_seconds;
        let accel_mm_s2 = if self.accel_cycles == 0 {
            0.0
        } else {
            speed_mm_s * FREQ / self.accel_cycles as f64
        };
        NudgeProfile::try_new(travel_mm, speed_mm_s, accel_mm_s2, 0.0)
            .expect("a monotonic nudge over a positive duration")
    }

    fn build(&self) -> Stream {
        let profile = self.profile();
        let duration = profile.duration();
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
        let total_cycles = (duration * FREQ).floor() as u64;
        let mut boundaries = vec![0];
        boundaries.extend(self.cuts.iter().copied().filter(|c| *c < total_cycles));
        boundaries.push(total_cycles);
        boundaries.sort_unstable();
        boundaries.dedup();

        let views = boundaries
            .windows(2)
            .map(|edge| {
                ClockedMotorSpan::try_new(
                    Arc::clone(&signal),
                    edge[0] as f64 / FREQ,
                    edge[1] as f64 / FREQ,
                    edge[0] as f64 / FREQ,
                    edge[1] as f64 / FREQ,
                    (ANCHOR_CLOCK + edge[0]) as f64,
                    FREQ,
                )
                .expect("a representable clocked view")
            })
            .collect::<Vec<ClockedMotorSpan>>();

        let roots = reference_roots(&views, self.microstep_mm, self.invert_dir);
        let last = views.last().expect("at least one view");
        let end_position = last
            .position_at_clock(last.end_clock)
            .expect("the view's last clock");
        Stream {
            cfg: MotorConfig {
                oid: self.oid,
                microstep_distance: self.microstep_mm,
                invert_dir: self.invert_dir,
                cycles_per_second: FREQ,
                encoder: self.encoder,
                min_rearm_cycles: 0,
            },
            views,
            roots,
            end_position,
        }
    }
}

/// Every clock of the stream, in order, against the lane lattice the cursor
/// walks: a step fires the first clock the position reaches the next threshold.
/// A view owns its last clock, so its successor is walked from one clock later.
fn reference_roots(views: &[ClockedMotorSpan], microstep_mm: f64, invert_dir: bool) -> Vec<Root> {
    let mut roots = Vec::new();
    let mut step_count = 0_i64;
    let mut owner = 0usize;
    let first_clock = views[0].start_clock;
    let last_clock = views[views.len() - 1].end_clock;
    let threshold = |count: i64| count as f64 * microstep_mm;
    for clock in first_clock..=last_clock {
        while views[owner].end_clock < clock {
            owner += 1;
        }
        let position = views[owner]
            .position_at_clock(clock)
            .expect("a clock inside its view");
        let advance = if position >= threshold(step_count + 1) {
            1_i8
        } else if position <= threshold(step_count - 1) {
            -1_i8
        } else {
            continue;
        };
        step_count += i64::from(advance);
        assert!(
            position < threshold(step_count + 1) && position > threshold(step_count - 1),
            "the generator let the signal cross two lattice levels in one clock"
        );
        roots.push(Root {
            clock,
            dir: u8::from((advance > 0) != invert_dir),
            advance,
        });
    }
    roots
}

/// The lattice level a monotone travel from the origin ends on, which is the
/// microstep count `travel / microstep_distance` truncated the way the walk
/// truncates it.
fn microsteps_reached(position: f64, microstep_mm: f64) -> i64 {
    let mut count = 0_i64;
    if position >= 0.0 {
        while (count + 1) as f64 * microstep_mm <= position {
            count += 1;
        }
    } else {
        while (count - 1) as f64 * microstep_mm >= position {
            count -= 1;
        }
    }
    count
}

fn frame_oid(frame: &StepFrame) -> u32 {
    match *frame {
        StepFrame::ResetStepClock { oid, .. }
        | StepFrame::SetNextStepDir { oid, .. }
        | StepFrame::QueueStep { oid, .. }
        | StepFrame::QueueStepHp { oid, .. } => oid,
    }
}

#[derive(Debug, Default)]
struct Wire {
    steps: usize,
    resets: usize,
    dirs: Vec<u8>,
    /// Step index and clock of every step the wire pins down: all of them for
    /// the classic packer, the endpoints of each move for the hp packer, whose
    /// interior walk `compress_hp_fuzz` owns.
    landmarks: Vec<(usize, u64)>,
    exact_clocks: Option<Vec<u64>>,
}

fn decode_lane(frames: &[StepFrame], cfg: &MotorConfig) -> Wire {
    let exact = matches!(cfg.encoder, StepEncoder::Classic { .. });
    let mut wire = Wire {
        exact_clocks: exact.then(Vec::new),
        ..Wire::default()
    };
    let mut cursor = 0_u64;
    for frame in frames {
        match *frame {
            StepFrame::ResetStepClock { oid, clock } if oid == cfg.oid => {
                wire.resets += 1;
                cursor = u64::from(clock);
            }
            StepFrame::SetNextStepDir { oid, dir } if oid == cfg.oid => wire.dirs.push(dir),
            StepFrame::QueueStep {
                oid,
                interval,
                count,
                add,
            } if oid == cfg.oid => {
                let step_move = StepMove {
                    interval,
                    count,
                    add,
                };
                for nth in 1..=count {
                    let clock = step_move.step_clock(cursor, nth);
                    wire.landmarks
                        .push((wire.steps + usize::from(nth) - 1, clock));
                    if let Some(clocks) = wire.exact_clocks.as_mut() {
                        clocks.push(clock);
                    }
                }
                cursor = step_move.last_clock(cursor);
                wire.steps += usize::from(count);
            }
            StepFrame::QueueStepHp {
                oid,
                count,
                first_step,
                last_step,
                ..
            } if oid == cfg.oid => {
                wire.landmarks.push((wire.steps, cursor + first_step));
                if count > 1 {
                    wire.landmarks
                        .push((wire.steps + usize::from(count) - 1, cursor + last_step));
                }
                cursor += last_step;
                wire.steps += usize::from(count);
            }
            _ => {}
        }
    }
    wire
}

fn check_wire(
    wire: &Wire,
    stream: &Stream,
    up_to_clock: u64,
    commanded_steps: i64,
) -> Result<(), TestCaseError> {
    let solved: Vec<Root> = stream
        .roots
        .iter()
        .copied()
        .filter(|root| root.clock <= up_to_clock)
        .collect();
    prop_assert_eq!(
        wire.steps,
        solved.len(),
        "oid {}: the wire carries {} steps for {} solved roots",
        stream.cfg.oid,
        wire.steps,
        solved.len()
    );
    prop_assert_eq!(
        commanded_steps,
        solved
            .iter()
            .map(|root| i64::from(root.advance))
            .sum::<i64>(),
        "oid {}: commanded step counter",
        stream.cfg.oid
    );
    prop_assert_eq!(
        wire.resets,
        usize::from(!solved.is_empty()),
        "oid {}: a stream re-anchors the mcu's step clock exactly once",
        stream.cfg.oid
    );
    prop_assert_eq!(
        wire.dirs.len(),
        usize::from(!solved.is_empty()),
        "oid {}: a single-direction stream latches dir once",
        stream.cfg.oid
    );
    if let Some(&dir) = wire.dirs.first() {
        prop_assert_eq!(dir, solved[0].dir, "oid {}: dir bit", stream.cfg.oid);
    }

    let allowance = match stream.cfg.encoder {
        StepEncoder::Classic { max_error_ticks } => u64::from(max_error_ticks),
        StepEncoder::HighPrecision => u64::from(DEFAULT_MAX_ERROR_TICKS),
    };
    let late_allowance = match stream.cfg.encoder {
        StepEncoder::Classic { .. } => 0,
        StepEncoder::HighPrecision => u64::from(DEFAULT_MAX_ERROR_TICKS),
    };
    let mut previous = 0_u64;
    for &(index, clock) in &wire.landmarks {
        prop_assert!(
            clock > previous,
            "oid {}: step {index} at {clock} does not advance past {previous}",
            stream.cfg.oid
        );
        prop_assert!(
            clock <= up_to_clock,
            "oid {}: step {index} at {clock} is past the {up_to_clock} drain window",
            stream.cfg.oid
        );
        let target = solved[index].clock;
        prop_assert!(
            clock <= target + late_allowance && target <= clock + allowance,
            "oid {}: step {index} at {clock} is outside the {allowance}/{late_allowance} \
             tick allowance of its root {target}",
            stream.cfg.oid
        );
        previous = clock;
    }
    Ok(())
}

fn arb_lane(oid: u32) -> impl Strategy<Value = Lane> {
    let microstep = prop_oneof![
        Just(0.0025),
        Just(0.005),
        Just(0.01),
        Just(0.0125),
        0.002..0.02
    ];
    let encoder = prop_oneof![
        Just(StepEncoder::Classic { max_error_ticks: 0 }),
        Just(StepEncoder::Classic {
            max_error_ticks: 25
        }),
        (1u32..2_000).prop_map(|max_error_ticks| StepEncoder::Classic { max_error_ticks }),
        Just(StepEncoder::HighPrecision),
    ];
    (
        microstep,
        any::<bool>(),
        encoder,
        1u64..60,
        0.0..1.0f64,
        4u64..48,
        prop_oneof![Just(0u64), 1u64..400],
        any::<bool>(),
    )
        .prop_flat_map(
            move |(
                microstep_mm,
                invert_dir,
                encoder,
                microsteps,
                lattice_phase,
                spacing_cycles,
                accel_cycles,
                backwards,
            )| {
                let span_cycles = (microsteps + 1) * spacing_cycles + accel_cycles;
                prop::collection::vec(1..span_cycles.max(2), 0..MAX_VIEWS).prop_map(move |cuts| {
                    Lane {
                        oid,
                        microstep_mm,
                        invert_dir,
                        encoder,
                        microsteps,
                        lattice_phase,
                        spacing_cycles,
                        accel_cycles,
                        backwards,
                        cuts,
                    }
                })
            },
        )
}

fn arb_lanes() -> impl Strategy<Value = Vec<Lane>> {
    prop_oneof![
        arb_lane(7).prop_map(|lane| vec![lane]),
        (arb_lane(7), arb_lane(11)).prop_map(|(first, second)| vec![first, second]),
    ]
}

fn seeded_shim(streams: &[Stream]) -> StepShim {
    let mut shim = StepShim::new(
        streams.iter().map(|stream| stream.cfg).collect(),
        QUEUE_DEPTH,
    );
    for (motor, stream) in streams.iter().enumerate() {
        shim.reset_position(motor, 0);
        shim.push_spans(motor, &stream.views)
            .expect("a contiguous run of views");
    }
    shim
}

fn drain_at(streams: &[Stream], stops: &[u64]) -> Result<Vec<StepFrame>, TestCaseError> {
    let mut shim = seeded_shim(streams);
    let mut frames = Vec::new();
    for &up_to_clock in stops {
        let drained = shim
            .drain_budgeted(up_to_clock, None)
            .map_err(|error| TestCaseError::fail(format!("drain to {up_to_clock}: {error}")))?;
        frames.extend(drained);
    }
    for (motor, stream) in streams.iter().enumerate() {
        prop_assert_eq!(
            shim.commanded_steps(motor),
            stream
                .roots
                .iter()
                .filter(|root| root.clock <= *stops.last().expect("a drain window"))
                .map(|root| i64::from(root.advance))
                .sum::<i64>(),
            "oid {}: commanded steps after draining to {:?}",
            stream.cfg.oid,
            stops.last()
        );
    }
    Ok(frames)
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 256,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/shim_drain_fuzz.txt",
        ))),
        ..ProptestConfig::default()
    })]

    #[test]
    fn a_drain_emits_every_root_it_solved_in_wire_order(
        lanes in arb_lanes(),
        window in 0u64..=100,
    ) {
        let streams: Vec<Stream> = lanes.iter().map(Lane::build).collect();
        let end_clock = streams
            .iter()
            .map(|stream| stream.views[stream.views.len() - 1].end_clock)
            .max()
            .expect("at least one lane");
        let up_to_clock = if window == 100 {
            u64::MAX
        } else {
            ANCHOR_CLOCK + (end_clock - ANCHOR_CLOCK) * window / 99
        };

        let mut shim = seeded_shim(&streams);
        let frames = shim
            .drain_budgeted(up_to_clock, None)
            .map_err(|error| TestCaseError::fail(format!("{error}")))?;

        for (motor, stream) in streams.iter().enumerate() {
            let wire = decode_lane(&frames, &stream.cfg);
            check_wire(&wire, stream, up_to_clock, shim.commanded_steps(motor))?;
            prop_assert_eq!(
                microsteps_reached(stream.end_position, stream.cfg.microstep_distance),
                stream.roots.iter().map(|root| i64::from(root.advance)).sum::<i64>(),
                "oid {}: the walked roots must reach travel/microstep_distance",
                stream.cfg.oid
            );
        }
        let attributed: usize = streams
            .iter()
            .map(|stream| {
                frames
                    .iter()
                    .filter(|frame| frame_oid(frame) == stream.cfg.oid)
                    .count()
            })
            .sum();
        prop_assert_eq!(
            attributed,
            frames.len(),
            "every frame belongs to exactly one configured oid"
        );
    }

    #[test]
    fn a_chunked_drain_puts_the_same_stream_on_the_wire(
        lanes in arb_lanes(),
        stops in prop::collection::vec(0u64..=99, 1..5),
    ) {
        let streams: Vec<Stream> = lanes.iter().map(Lane::build).collect();
        let end_clock = streams
            .iter()
            .map(|stream| stream.views[stream.views.len() - 1].end_clock)
            .max()
            .expect("at least one lane");
        let mut windows: Vec<u64> = stops
            .iter()
            .map(|stop| ANCHOR_CLOCK + (end_clock - ANCHOR_CLOCK) * stop / 99)
            .collect();
        windows.sort_unstable();
        windows.push(u64::MAX);

        let whole = drain_at(&streams, &[u64::MAX])?;
        let chunked = drain_at(&streams, &windows)?;

        for stream in &streams {
            let whole_wire = decode_lane(&whole, &stream.cfg);
            let chunked_wire = decode_lane(&chunked, &stream.cfg);
            prop_assert_eq!(
                chunked_wire.steps,
                whole_wire.steps,
                "oid {}: step count over {:?}",
                stream.cfg.oid,
                windows
            );
            prop_assert_eq!(
                chunked_wire.resets,
                whole_wire.resets,
                "oid {}: a chunked drain must not re-anchor the mcu",
                stream.cfg.oid
            );
            prop_assert_eq!(
                chunked_wire.dirs,
                whole_wire.dirs,
                "oid {}: dir latching",
                stream.cfg.oid
            );
            if stream.cfg.encoder == (StepEncoder::Classic { max_error_ticks: 0 }) {
                prop_assert_eq!(
                    chunked_wire.exact_clocks,
                    whole_wire.exact_clocks,
                    "oid {}: an exact encoder emits the solved clocks whatever the pacing",
                    stream.cfg.oid
                );
            }
        }
    }
}
