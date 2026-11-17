//! The stream time the turning guard, and the planning front around it,
//! hold back on the Pi bench printer's chain (`tools/bench/printer.cfg`: a
//! kernel-free linear advance behind smooth shapers, corners blended).
//! Whatever they hold, a pause plays out before it stops (#860).

use std::sync::Arc;

use geometry::segment::SourceRange;
use geometry::{MoveVelocity, VALLEY_LEAD_BUDGET_S};
use planner_config::RuntimeLimits;
use planner_config::from_doc::planner_config_from_text;
use trajectory::AxisChainSet;

use super::{SCALE_FLOOR, TurningGuard};
use crate::fit_stage::{FitStage, Lookahead};
use crate::live_config::{GcodeLine, GcodeMoveBase, extruder_caps, stream_config};
use crate::planner::Planner;
use crate::types::{PlannedItem, StreamConfig, StreamInput};

const BENCH_PRINTER: &str = "tools/bench/printer.cfg";
const FIRST_LAYERS: &str = "tools/sim/gcode/voron_first_layers.gcode";
const FIRST_LAYERS_LINES: usize = 1500;
const SPEED_LADDER: &str = "tools/bench/gcode/speed_ladder.gcode";
const WHOLE_FILE: usize = usize::MAX;
const MM_PER_MIN_PER_MM_S: f64 = 60.0;
/// Where every stream here starts, as the live path homes the toolhead.
const HOME: [f64; 3] = [0.0; 3];

/// `tools/bench/session_scenarios.zigzag_moves`: the `pause_sparse` row.
const ZIGZAG_STROKES: usize = 120;
const ZIGZAG_FEED_MM_S: f64 = 100.0;
const TRAVEL_FEED_MM_S: f64 = 200.0;
const Z_FEED_MM_S: f64 = 10.0;

/// `tools/bench/gcode/extrude_ladder.gcode`: 20 mm in and out, at rising
/// feedrates, with the leaders at rest.
const EXTRUDE_MM: f64 = 20.0;
const EXTRUDE_FEEDS_MM_S: [f64; 7] = [5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0];
const EXTRUDES_PER_FEED: usize = 4;

/// The guard's windows a move waits on, past its own end: its check reads
/// [`VALLEY_LEAD_BUDGET_S`] and the kernels' forward reach beyond it; a
/// back-off a pending check leads to touches as far as the kernels' and
/// that budget's reach behind the first move unchecked; and a move leaves
/// only once its kernels and the budget read nothing past the seam it
/// freezes. Between those two seams lies the brake a back-off past the
/// frozen one may ask: to rest from the seam's planned speed at
/// [`SCALE_FLOOR`] of the budget.
const WINDOWS: usize = 3;
/// The moves the guard holds whole where moves outlast its windows: the
/// newest, which its check reads past; the one the back-off window ends
/// in; the one the frozen seam starts, at a move's start; and the one the
/// last window ends in.
const WHOLE_MOVES: usize = 4;

/// What the planning front may hold besides the fitted move the next line
/// may still merge into, and the half of the move before it a run that
/// move may start can still ease over.
const FRONT_REST_S: f64 = 1.5;

/// One input of a stream: a G-code line under a runtime acceleration cap,
/// or a drain (a dwell, a control token).
#[derive(Debug, Clone, Copy)]
pub(super) enum Step {
    Line(GcodeLine, Option<f64>),
    Drain,
}

pub(super) struct BenchChain {
    base: GcodeMoveBase,
    pub config: StreamConfig,
    pub chains: AxisChainSet,
}

impl BenchChain {
    pub fn new() -> Self {
        let text = std::fs::read_to_string(source_tree::repo_file(BENCH_PRINTER))
            .expect("read the bench printer.cfg");
        let cfg = planner_config_from_text(&text).expect("the bench printer.cfg parses");
        let chains = cfg.compile_active_chains().expect("its chains compile");
        let config = stream_config(&cfg, None)
            .expect("its stream config builds")
            .with_shaped_corners(&chains);
        let caps = extruder_caps(&cfg).expect("its extruder caps build");
        Self {
            base: GcodeMoveBase::of(&cfg, &caps).expect("its move base builds"),
            config,
            chains,
        }
    }

    /// The planner's output for `lines`, fitted and planned as the live
    /// path does, the stream closed at the end.
    pub fn plan(&self, lines: &[GcodeLine]) -> Vec<PlannedItem> {
        let runtime = RuntimeLimits::default();
        let mut fit = FitStage::new(self.config.corner).into_driver();
        let mut planner = Planner::new(self.config.clone(), Arc::default());
        let mut planned = Vec::new();
        let mut collect = |item| {
            planned.push(item);
            true
        };
        for line in lines {
            let Some(m) = self
                .base
                .line_move(&runtime, *line)
                .expect("the line moves")
            else {
                continue;
            };
            assert!(fit.feed(StreamInput::Move(m), &mut |item| {
                planner.feed(item, &mut collect)
            }));
        }
        assert!(fit.finish(&mut |item| planner.feed(item, &mut collect)));
        assert!(planner.finish(&mut collect));
        planned
    }

    /// The planner's output for `steps`, each line under its runtime
    /// acceleration cap, a drain where a step asks for one, fitted and
    /// planned with `lookahead`, the stream closed at the end.
    pub(super) fn plan_with(&self, steps: &[Step], lookahead: Lookahead) -> Vec<PlannedItem> {
        let mut runtime = RuntimeLimits::default();
        let mut fit = FitStage::with_lookahead(self.config.corner, lookahead).into_driver();
        let mut planner = Planner::with_lookahead(self.config.clone(), Arc::default(), lookahead);
        let mut planned = Vec::new();
        let mut collect = |item| {
            planned.push(item);
            true
        };
        for step in steps {
            let item = match step {
                Step::Line(line, accel) => {
                    runtime.set_accel_cap(*accel).expect("a positive cap");
                    let Some(m) = self
                        .base
                        .line_move(&runtime, *line)
                        .expect("the line moves")
                    else {
                        continue;
                    };
                    StreamInput::Move(m)
                }
                Step::Drain => StreamInput::Drain,
            };
            assert!(fit.feed(item, &mut |item| planner.feed(item, &mut collect)));
        }
        assert!(fit.finish(&mut |item| planner.feed(item, &mut collect)));
        assert!(planner.finish(&mut collect));
        planned
    }

    /// The most stream time the guard held back, at the plan it holds, as
    /// the planned moves arrived one at a time, and the bound its windows
    /// allow.
    fn held(&self, planned: Vec<PlannedItem>) -> Held {
        let mut guard = TurningGuard::new(self.config.clone(), &self.chains, &HOME);
        let bound = Bound::of(&guard, &planned);
        let mut arrived_s = 0.0;
        let mut most = Held {
            bound_s: bound.s,
            ..Held::default()
        };
        for item in planned {
            if let PlannedItem::Move(planned) = &item {
                arrived_s += duration_s(&planned.velocity);
            }
            assert!(guard.feed(item, &mut |_| true));
            let held_s: f64 = guard.entries.velocities.iter().map(duration_s).sum();
            if held_s > most.held_s {
                most = Held {
                    held_s,
                    at_s: arrived_s,
                    ..most
                };
            }
        }
        assert!(
            guard.entries.moves.is_empty(),
            "the closed stream left {} moves held",
            guard.entries.moves.len()
        );
        most
    }

    /// The most stream time the planning front (fit stage, planner and
    /// guard) held back past the moves it must hold, as the lines arrived
    /// one at a time, timed on the plan it emitted.
    fn front_held(&self, lines: &[GcodeLine]) -> FrontHeld {
        let runtime = RuntimeLimits::default();
        let mut fit = FitStage::new(self.config.corner).into_driver();
        let mut planner = Planner::new(self.config.clone(), Arc::default());
        let mut guard = TurningGuard::new(self.config.clone(), &self.chains, &HOME);
        let mut stream = FrontStream::default();
        for line in lines {
            let Some(m) = self
                .base
                .line_move(&runtime, *line)
                .expect("the line moves")
            else {
                continue;
            };
            assert!(fit.feed(StreamInput::Move(m), &mut |item| {
                stream.fit(item, &mut planner, &mut guard)
            }));
            stream.arrived(line.seq);
        }
        assert!(fit.finish(&mut |item| stream.fit(item, &mut planner, &mut guard)));
        assert!(planner.finish(&mut |item| guard.feed(item, &mut |item| stream.emit(&item))));
        stream.held()
    }
}

#[derive(Debug, Default)]
struct Held {
    held_s: f64,
    at_s: f64,
    bound_s: f64,
}

impl Held {
    fn assert_within_bound(&self, what: &str) {
        assert!(
            self.held_s <= self.bound_s,
            "the guard held {:.3} s of {what} back at t={:.3} s, past the {:.3} s its windows \
             and the floor brake need",
            self.held_s,
            self.at_s,
            self.bound_s
        );
    }
}

/// The front's input and output on one timeline: the fitted moves' source
/// lines, the end of every move the guard emitted, and how much it had
/// emitted when each line arrived, with the path the fit stage and the
/// planner had handed on by then.
#[derive(Default)]
struct FrontStream {
    fitted: Vec<SourceRange>,
    fitted_mm: f64,
    planned_mm: f64,
    emitted: Vec<(u32, f64)>,
    emitted_path: Vec<(f64, f64)>,
    emitted_mm: f64,
    emitted_s: f64,
    arrivals: Vec<Arrival>,
}

struct Arrival {
    line: u32,
    emitted_s: f64,
    fitted_mm: f64,
    planned_mm: f64,
}

impl FrontStream {
    fn fit(&mut self, item: StreamInput, planner: &mut Planner, guard: &mut TurningGuard) -> bool {
        if let StreamInput::Move(m) = &item {
            self.fitted.push(m.source);
            self.fitted_mm += m.segment.s_len();
        }
        planner.feed(item, &mut |item| {
            if let PlannedItem::Move(planned) = &item {
                self.planned_mm += planned.geometry.segment.s_len();
            }
            guard.feed(item, &mut |item| self.emit(&item))
        })
    }

    fn emit(&mut self, item: &PlannedItem) -> bool {
        if let PlannedItem::Move(planned) = item {
            self.emitted_s += duration_s(&planned.velocity);
            self.emitted_mm += planned.geometry.segment.s_len();
            self.emitted
                .push((planned.geometry.source.start_line, self.emitted_s));
            self.emitted_path.push((self.emitted_mm, self.emitted_s));
        }
        true
    }

    fn arrived(&mut self, line: u32) {
        self.arrivals.push(Arrival {
            line,
            emitted_s: self.emitted_s,
            fitted_mm: self.fitted_mm,
            planned_mm: self.planned_mm,
        });
    }

    /// When the plan has covered `path_mm`, at the end of the move that
    /// covers it.
    fn time_at(&self, path_mm: f64) -> f64 {
        let index = self
            .emitted_path
            .partition_point(|&(end_mm, _)| end_mm < path_mm);
        self.emitted_path
            .get(index)
            .map_or(self.emitted_s, |&(_, end_s)| end_s)
    }

    /// The first line of the fitted move `line` ended up in.
    fn chain_start(&self, line: u32) -> u32 {
        self.fitted
            .iter()
            .filter(|source| (source.start_line..=source.end_line).contains(&line))
            .map(|source| source.start_line)
            .min()
            .unwrap_or(line)
    }

    /// Where the plan stands when every line before `line` has played.
    fn played_before(&self, line: u32) -> f64 {
        self.emitted
            .iter()
            .filter(|(start_line, _)| *start_line < line)
            .map(|(_, end_s)| *end_s)
            .fold(0.0, f64::max)
    }

    fn held(&self) -> FrontHeld {
        let mut most = FrontHeld::default();
        let mut previous_line = None;
        for arrival in &self.arrivals {
            let line = arrival.line;
            let newest = self.chain_start(line);
            let newest_from_s = self.played_before(newest);
            let previous_s = previous_line
                .map(|previous| self.chain_start(previous))
                .filter(|&previous| previous < newest)
                .map_or(0.0, |previous| newest_from_s - self.played_before(previous));
            let past_newest_s = (newest_from_s - arrival.emitted_s).max(0.0);
            let excess_s = past_newest_s - 0.5 * previous_s;
            if excess_s > most.excess_s {
                let fitted_s = self.time_at(arrival.fitted_mm);
                let planned_s = self.time_at(arrival.planned_mm);
                most = FrontHeld {
                    excess_s,
                    past_newest_s,
                    previous_s,
                    line,
                    stages_s: [
                        (newest_from_s - fitted_s).max(0.0),
                        fitted_s - planned_s,
                        planned_s - arrival.emitted_s,
                    ],
                };
            }
            previous_line = Some(line);
        }
        most
    }
}

/// What the front held at its worst, past the moves it must hold, and how
/// the fit stage, the planner and the guard shared it.
#[derive(Debug, Default)]
struct FrontHeld {
    excess_s: f64,
    past_newest_s: f64,
    previous_s: f64,
    line: u32,
    stages_s: [f64; 3],
}

impl FrontHeld {
    fn assert_within_bound(&self, what: &str) {
        let [fit_s, planner_s, guard_s] = self.stages_s;
        assert!(
            self.excess_s <= FRONT_REST_S,
            "the front held {:.3} s of {what} back past the newest fitted move at line {}, \
             {:.3} s past half the {:.3} s move before it and {FRONT_REST_S} s (fit stage \
             {fit_s:.3} s, planner {planner_s:.3} s, guard {guard_s:.3} s)",
            self.past_newest_s,
            self.line,
            self.excess_s - FRONT_REST_S,
            self.previous_s,
        );
    }
}

/// [`WINDOWS`] spans of the guard's own reach, [`WHOLE_MOVES`] of the
/// longest move, and the longest floor brake from a planned seam speed.
struct Bound {
    s: f64,
}

impl Bound {
    fn of(guard: &TurningGuard, planned: &[PlannedItem]) -> Self {
        let h = guard.h();
        let (back, ahead) = guard.span;
        let window_s = VALLEY_LEAD_BUDGET_S + 2.0 * (back + ahead + 2) as f64 * h;
        let moves = planned.iter().filter_map(|item| match item {
            PlannedItem::Move(planned) => Some(planned),
            _ => None,
        });
        let (longest_s, brake_s) = moves.fold((0.0_f64, 0.0_f64), |(longest, brake), m| {
            let floor_accel = m.velocity.accel * SCALE_FLOOR / std::f64::consts::SQRT_2;
            (
                longest.max(duration_s(&m.velocity)),
                brake.max(m.velocity.entry_v / floor_accel),
            )
        });
        Self {
            s: WINDOWS as f64 * window_s + WHOLE_MOVES as f64 * longest_s + brake_s,
        }
    }
}

fn duration_s(velocity: &MoveVelocity) -> f64 {
    velocity.phases.iter().map(|phase| phase.dt).sum()
}

pub(super) fn line(seq: usize, start: [f64; 3], end: [f64; 3], de: f64, feed: f64) -> GcodeLine {
    GcodeLine {
        start,
        delta: std::array::from_fn(|axis| end[axis] - start[axis]),
        de,
        feedrate_mm_s: feed,
        seq: u32::try_from(seq).expect("a line number fits u32"),
    }
}

/// A Z move, a travel, then strokes of 80 mm at 100 mm/s, back and forth,
/// stepping 0.4 mm over after every second one.
fn zigzag() -> Vec<GcodeLine> {
    let up = [0.0, 0.0, 0.3];
    let mut at = [60.0, 20.0, 0.3];
    let mut lines = vec![
        line(3, HOME, up, 0.0, Z_FEED_MM_S),
        line(4, up, at, 0.0, TRAVEL_FEED_MM_S),
    ];
    for stroke in 0..ZIGZAG_STROKES {
        let x = if stroke % 2 == 0 { 140.0 } else { 60.0 };
        let end = [x, 20.0 + (stroke / 2) as f64 * 0.4, 0.3];
        lines.push(line(5 + stroke, at, end, 0.3, ZIGZAG_FEED_MM_S));
        at = end;
    }
    lines
}

fn extrude_ladder() -> Vec<GcodeLine> {
    let at = HOME;
    EXTRUDE_FEEDS_MM_S
        .iter()
        .flat_map(|&feed| std::iter::repeat_n(feed, EXTRUDES_PER_FEED))
        .enumerate()
        .map(|(index, feed)| {
            let de = if index % 2 == 0 {
                EXTRUDE_MM
            } else {
                -EXTRUDE_MM
            };
            line(3 + index, at, at, de, feed)
        })
        .collect()
}

/// The first lines of a slicer file that moves by `G1` alone in absolute
/// coordinates with relative extrusion (`G90`, `M83`).
fn gcode_file(path: &str, limit: usize) -> Vec<GcodeLine> {
    let text = std::fs::read_to_string(source_tree::repo_file(path))
        .unwrap_or_else(|e| panic!("read {path}: {e}"));
    let mut at = HOME;
    let mut feed = 0.0;
    let mut lines = Vec::new();
    for (index, raw) in text.lines().take(limit).enumerate() {
        let code = raw.split(';').next().unwrap_or_default();
        let mut words = code.split_whitespace();
        if words.next() != Some("G1") {
            continue;
        }
        let mut end = at;
        let mut de = 0.0;
        for word in words {
            let (letter, value) = word.split_at(1);
            let value: f64 = value.parse().expect("a G1 word carries a number");
            match letter {
                "X" => end[0] = value,
                "Y" => end[1] = value,
                "Z" => end[2] = value,
                "E" => de = value,
                "F" => feed = value / MM_PER_MIN_PER_MM_S,
                _ => panic!("line {}: unexpected G1 word {word}", index + 1),
            }
        }
        lines.push(line(index + 1, at, end, de, feed));
        at = end;
    }
    lines
}

fn first_layers() -> Vec<GcodeLine> {
    gcode_file(FIRST_LAYERS, FIRST_LAYERS_LINES)
}

#[test]
fn the_guard_holds_a_zigzag_of_reversal_blends_within_its_windows() {
    let chain = BenchChain::new();
    chain
        .held(chain.plan(&zigzag()))
        .assert_within_bound("the zigzag");
}

#[test]
fn the_guard_holds_the_first_layers_within_its_windows() {
    let chain = BenchChain::new();
    chain
        .held(chain.plan(&first_layers()))
        .assert_within_bound("the first layers");
}

#[test]
fn the_guard_holds_extrusion_with_the_leaders_at_rest_within_its_windows() {
    let chain = BenchChain::new();
    chain
        .held(chain.plan(&extrude_ladder()))
        .assert_within_bound("the extrude ladder");
}

#[test]
fn the_front_holds_a_zigzag_within_its_windows() {
    let chain = BenchChain::new();
    chain
        .front_held(&zigzag())
        .assert_within_bound("the zigzag");
}

#[test]
fn the_front_holds_the_speed_ladder_within_its_windows() {
    let chain = BenchChain::new();
    chain
        .front_held(&gcode_file(SPEED_LADDER, WHOLE_FILE))
        .assert_within_bound("the speed ladder");
}

#[test]
fn the_front_holds_the_first_layers_within_its_windows() {
    let chain = BenchChain::new();
    chain
        .front_held(&first_layers())
        .assert_within_bound("the first layers");
}

#[test]
fn the_front_holds_extrusion_with_the_leaders_at_rest_within_its_windows() {
    let chain = BenchChain::new();
    chain
        .front_held(&extrude_ladder())
        .assert_within_bound("the extrude ladder");
}
