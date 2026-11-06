use std::sync::Arc;

use geometry::ExtruderCaps;
use planner_config::{PlannerConfig, RuntimeLimits};
use trajectory::AxisChainSet;

use super::TurningGuard;
use crate::fit_stage::FitStage;
use crate::live_config::{GcodeLine, GcodeMoveBase, extruder_caps, stream_config};
use crate::planner::Planner;
use crate::types::{PlannedItem, PlannedMove, StreamConfig, StreamInput};

const HIGH_ACCEL_RAMP: &str = "[printer]\n\
    max_velocity: 1000\n\
    max_accel: 70000\n\
    max_jerk: 0\n\
    square_corner_velocity: 5\n\
    max_z_velocity: 25\n\
    max_z_accel: 100\n\
    [kinematics]\n\
    type: corexy\n\
    axis_x: x\n\
    axis_y: y\n\
    axis_z: z\n\
    a_motors: a\n\
    b_motors: b\n\
    z_motors: z\n\
    [axis x]\n\
    post_processors: shaper_x\n\
    [axis y]\n\
    post_processors: shaper_y\n\
    [axis z]\n\
    post_processors: shaper_z\n\
    [axis e]\n\
    follows: x, y, z\n\
    motors: extruder\n\
    post_processors: pa\n\
    max_accel: 20000\n\
    [post_processor pa]\n\
    type: linear_pressure_advance\n\
    k: 0.02\n\
    [post_processor shaper_x]\n\
    type: smooth_mzv\n\
    frequency_hz: 191.0\n\
    [post_processor shaper_y]\n\
    type: smooth_mzv\n\
    frequency_hz: 129.4\n\
    [post_processor shaper_z]\n\
    type: smooth_mzv\n\
    frequency_hz: 100.0\n\
    [motor a]\n\
    drive: stepper\n\
    [motor b]\n\
    drive: stepper\n\
    [motor z]\n\
    drive: stepper\n\
    [motor extruder]\n\
    drive: stepper\n\
    [extruder]\n\
    axis: e\n\
    max_extrude_only_velocity: 50\n\
    max_extrude_only_accel: 5000\n";

const SHAPED_Z: &str = "[axis z]\npost_processors: shaper_z\n";

fn without_a_z_shaper() -> String {
    assert!(HIGH_ACCEL_RAMP.contains(SHAPED_Z));
    HIGH_ACCEL_RAMP.replace(SHAPED_Z, "[axis z]\n")
}

/// A printer whose planner reads the advance's jerk at `jerk_gain_share`
/// of the switch gain it plans at, so the guard behind it, which reads the
/// whole variation, meets the demand of a planner that under-estimates it.
struct UnsoundPlanner {
    cfg: PlannerConfig,
    config: StreamConfig,
    chains: AxisChainSet,
    real: ExtruderCaps,
    planned_under: ExtruderCaps,
}

impl UnsoundPlanner {
    fn new(profile: &str, jerk_gain_share: f64) -> Self {
        let cfg = planner_config::from_doc::planner_config_from_text(profile).unwrap();
        let config = stream_config(&cfg, None).unwrap();
        let chains = cfg.compile_active_chains().unwrap();
        let [real] = extruder_caps(&cfg).unwrap()[..] else {
            panic!("the profile has one extruder");
        };
        Self {
            cfg,
            config,
            chains,
            real,
            planned_under: ExtruderCaps {
                leader_peak_hz: real.leader_peak_hz.map(|gain| jerk_gain_share * gain),
                ..real
            },
        }
    }

    fn plan(&self, lines: &[GcodeLine]) -> Vec<PlannedItem> {
        let base = GcodeMoveBase::of(&self.cfg, &[self.planned_under]).unwrap();
        let runtime = RuntimeLimits::default();
        let mut fit = FitStage::new(self.config.corner).into_driver();
        let mut planner = Planner::new(self.config.clone(), Arc::default());
        let mut planned = Vec::new();
        let mut collect = |item| {
            planned.push(item);
            true
        };
        for line in lines {
            let m = base.line_move(&runtime, *line).unwrap().unwrap();
            assert!(fit.feed(StreamInput::Move(m), &mut |item| {
                planner.feed(item, &mut collect)
            }));
        }
        assert!(fit.finish(&mut |item| planner.feed(item, &mut collect)));
        assert!(planner.finish(&mut collect));
        planned
    }

    fn guard(&self, planned: impl IntoIterator<Item = PlannedItem>) -> Vec<PlannedMove> {
        let mut guard = TurningGuard::new(self.config.clone(), &self.chains);
        let mut emitted = Vec::new();
        for item in planned {
            let item = match item {
                PlannedItem::Move(mut planned) => {
                    planned.geometry.limits.extruder = Some(self.real);
                    PlannedItem::Move(planned)
                }
                other => other,
            };
            assert!(guard.feed(item, &mut |item| {
                if let PlannedItem::Move(planned) = item {
                    emitted.push(planned);
                }
                true
            }));
        }
        emitted
    }

    /// Guards the plan of `lines`: every move leaves, proven, so a second
    /// pass over what left changes nothing.
    fn assert_guards(&self, lines: &[GcodeLine]) {
        let planned = self.plan(lines);
        let moves = planned
            .iter()
            .filter(|item| matches!(item, PlannedItem::Move(_)))
            .count();
        let emitted = self.guard(planned);
        assert_eq!(emitted.len(), moves);
        let twice = self.guard(again(&emitted));
        for (once, twice) in emitted.iter().zip(&twice) {
            assert_eq!(
                once.velocity, twice.velocity,
                "line {}",
                once.geometry.source.start_line
            );
        }
    }
}

fn again(emitted: &[PlannedMove]) -> impl Iterator<Item = PlannedItem> + '_ {
    emitted
        .iter()
        .map(|planned| {
            PlannedItem::Move(PlannedMove {
                geometry: planned.geometry.clone(),
                velocity: planned.velocity.clone(),
                budget: planned.budget,
            })
        })
        .chain(std::iter::once(PlannedItem::Drain))
}

fn line(seq: u32, start: [f64; 2], end: [f64; 2], de: f64, feedrate_mm_s: f64) -> GcodeLine {
    GcodeLine {
        start: [start[0], start[1], 0.0],
        delta: [end[0] - start[0], end[1] - start[1], 0.0],
        de,
        feedrate_mm_s,
        seq,
    }
}

/// A 135° corner taken at 300 mm/s between two 60 mm extrusions, after
/// one at 150 mm/s and a travel back to its start (`corner_flow_cases`
/// lines 56–68).
fn corner_after_a_travel() -> Vec<GcodeLine> {
    let (start, corner, end) = ([0.0, 0.0], [60.0, 0.0], [102.42641, 42.42641]);
    vec![
        line(56, corner, end, 1.995608, 150.0),
        line(61, end, start, 0.0, 300.0),
        line(62, start, corner, 1.995608, 300.0),
        line(63, corner, end, 1.995608, 300.0),
        line(68, end, start, 0.0, 50.0),
    ]
}

/// Half a hole's inner perimeter, a short travel and the start of its outer
/// one, short chords at 330–430 mm/s (`voron_dense_top_layers` lines
/// 1332–1351).
fn hole_perimeters() -> Vec<GcodeLine> {
    let inner = [
        [118.761, 146.406],
        [119.068, 146.417],
        [119.427, 146.502],
        [119.663, 146.607],
        [119.969, 146.815],
        [120.226, 147.086],
        [120.417, 147.403],
        [120.508, 147.645],
        [120.574, 148.005],
        [120.574, 148.263],
        [120.508, 148.627],
        [120.362, 148.972],
        [120.171, 149.246],
    ];
    let inner_e = [
        0.01033, 0.01241, 0.0087, 0.01242, 0.01258, 0.01245, 0.0087, 0.01231, 0.00866, 0.01246,
        0.01259, 0.01126,
    ];
    let outer = [
        [120.549, 149.526],
        [120.768, 149.229],
        [120.955, 148.801],
        [121.045, 148.359],
        [121.052, 147.987],
        [120.976, 147.541],
        [120.834, 147.174],
    ];
    let outer_e = [0.01621, 0.0205, 0.01979, 0.01633, 0.01984, 0.01724];
    let (inner_feed, travel_feed, outer_feed) = (432.5609333333333, 1000.0, 331.60196666666667);
    let mut lines: Vec<GcodeLine> = inner
        .windows(2)
        .zip(inner_e)
        .zip(1332..)
        .map(|((ends, de), seq)| line(seq, ends[0], ends[1], de, inner_feed))
        .collect();
    lines.push(line(1344, inner[12], outer[0], 0.0, travel_feed));
    lines.extend(
        outer
            .windows(2)
            .zip(outer_e)
            .zip(1346..)
            .map(|((ends, de), seq)| line(seq, ends[0], ends[1], de, outer_feed)),
    );
    lines
}

#[test]
fn back_offs_under_an_under_estimated_jerk_gain_never_change_emitted_motion() {
    UnsoundPlanner::new(HIGH_ACCEL_RAMP, 0.45).assert_guards(&corner_after_a_travel());
}

#[test]
fn back_offs_cascading_under_a_planner_blind_to_the_advance_jerk_never_reach_emitted_motion() {
    UnsoundPlanner::new(&without_a_z_shaper(), 0.0).assert_guards(&hole_perimeters());
}
