//! What the shaper emits does not depend on how many lowered segments it
//! takes in before each emit.

use std::sync::Arc;

use geometry::{
    CornerFitConfig, Move, MoveContext, RATIO_RAMP_MIN_TIME_S, SourceRange, VelocityLimits,
    line_move,
};
use trajectory::{AxisChainSet, CompiledChain, ContinuousSegment};

use super::Shaper;
use crate::Pipeline;
use crate::types::{BaseItem, StreamConfig, TrajectoryItem};

const ACCEL_MM_S2: f64 = 5000.0;
const FEED_MM_S: f64 = 80.0;
const EXTRUSION_RATIO: f64 = 0.05;
/// The ratio ramp samples the leaders for its width nodes every half of
/// the shortest ramp, on a grid of absolute stream time.
const WIDTH_NODE_SPACING_S: f64 = 0.5 * RATIO_RAMP_MIN_TIME_S;
/// How close short of a rest a width node stacks with it.
const STACKING_REACH_MM: f64 = 1e-6;
const EMIT_BATCH_SEGMENTS: usize = 8;

fn config() -> StreamConfig {
    StreamConfig {
        corner: CornerFitConfig::default(),
        integration_tol: 1e-7,
        max_extrude_only_velocity_mm_s: 100.0,
        max_extrude_only_accel_mm_s2: 1000.0,
        fit_tol_mm: 1e-3,
        fit_tol_accel_mm_s2: 50.0,
        max_buffer_moves: 64,
        limits: limits(),
    }
}

fn limits() -> VelocityLimits {
    VelocityLimits::try_new(
        300.0,
        ACCEL_MM_S2,
        geometry::corner_deviation_from_scv(5.0, ACCEL_MM_S2),
        f64::INFINITY,
    )
    .unwrap()
}

fn line(line_no: u32, start: [f64; 3], end: [f64; 3], e: f64) -> Move {
    let ctx = MoveContext {
        extruder_axis: 3,
        feedrate_mm_s: FEED_MM_S,
        limits: limits(),
        source: SourceRange {
            start_line: line_no,
            end_line: line_no,
        },
    };
    line_move(start, end, e, ctx).unwrap()
}

/// A plain chain: the extruder follows the toolhead with no advance and no
/// kernel anywhere, so its ratio ramp is the only smoothing.
fn plain_chains() -> AxisChainSet {
    AxisChainSet {
        chains: vec![CompiledChain::default(); 4],
        followers: vec![(3, vec![0, 1, 2])],
    }
}

/// An extrusion of `length` mm that stops for a retract, a travel, and an
/// extrusion after it.
fn print_with_a_stop(length: f64) -> Vec<Move> {
    vec![
        line(1, [0.0, 0.0, 0.0], [length, 0.0, 0.0], EXTRUSION_RATIO * length),
        line(2, [length, 0.0, 0.0], [length, 0.0, 0.0], -0.5),
        line(3, [length, 0.0, 0.0], [length + 10.0, 0.0, 0.0], 0.0),
        line(4, [length + 10.0, 0.0, 0.0], [length + 20.0, 0.0, 0.0], 0.5),
    ]
}

fn lowered(moves: &[Move]) -> Vec<BaseItem> {
    let mut pipeline = Pipeline::new(config(), plain_chains(), vec![0.0; 4], 0.0);
    let mut items = Vec::new();
    let mut collect = |item| {
        items.push(item);
        true
    };
    for m in moves {
        assert!(pipeline.front.feed(m.clone().into(), &mut collect));
    }
    assert!(pipeline.front.finish(&mut collect));
    items
}

/// How far short of the stop at the extrusion's end the last width node
/// before it is sampled, if the stop ends a lowered segment.
fn width_node_short_of_the_stop(items: &[BaseItem]) -> Option<f64> {
    let extrusion = items.iter().find_map(|item| match item {
        BaseItem::Seg(seg) if seg.spatial_path && seg.rest_at_end => Some(seg),
        _ => None,
    })?;
    let node_t = (extrusion.t_end / WIDTH_NODE_SPACING_S).floor() * WIDTH_NODE_SPACING_S;
    let x = |t: f64| extrusion.eval_axis(0, t).expect("the extrusion evaluates").position;
    (node_t > extrusion.t_start).then(|| x(extrusion.t_end) - x(node_t))
}

/// The first extrusion length whose stop lands a width node within
/// stacking reach short of it.
fn length_stacking_a_node_on_the_stop() -> f64 {
    (0..2000)
        .map(|step| 20.0 + 5e-4 * f64::from(step))
        .find(|&length| {
            width_node_short_of_the_stop(&lowered(&print_with_a_stop(length))).is_some_and(
                |short| short > 0.01 * STACKING_REACH_MM && short < 0.9 * STACKING_REACH_MM,
            )
        })
        .expect("some extrusion length stops just past a width node")
}

fn shaper() -> Shaper {
    Shaper::new(plain_chains(), config().fit_tol(), Arc::default())
}

fn segments_of(out: Vec<TrajectoryItem>) -> Vec<ContinuousSegment> {
    out.into_iter()
        .filter_map(|item| match item {
            TrajectoryItem::Seg(seg) => Some(seg),
            TrajectoryItem::Parked | TrajectoryItem::Control(_) => None,
        })
        .collect()
}

fn emitted_per_segment(items: Vec<BaseItem>) -> Vec<ContinuousSegment> {
    let mut shaper = shaper();
    let mut out = Vec::new();
    let mut collect = |item| {
        out.push(item);
        true
    };
    for item in items {
        assert!(shaper.feed(item, &mut collect));
    }
    assert!(shaper.finish(&mut collect));
    segments_of(out)
}

fn emitted_in_batches(items: Vec<BaseItem>, batch: usize) -> Vec<ContinuousSegment> {
    let mut shaper = shaper();
    let mut out = Vec::new();
    let mut collect = |item| {
        out.push(item);
        true
    };
    let mut taken = 0;
    for item in items {
        match item {
            BaseItem::Seg(seg) => {
                shaper.pending.push(seg);
                taken += 1;
                if taken % batch == 0 {
                    let count = shaper.supported_count();
                    assert!(shaper.emit(count, false, &mut collect));
                }
            }
            other => assert!(shaper.feed(other, &mut collect)),
        }
    }
    assert!(shaper.finish(&mut collect));
    segments_of(out)
}

fn extruder_at_segment_ends(segments: &[ContinuousSegment]) -> Vec<(f64, f64)> {
    segments
        .iter()
        .map(|seg| {
            let e = seg.eval_axis(3, seg.t_end).expect("the extruder evaluates");
            (seg.t_end, e.position)
        })
        .collect()
}

/// A width node sampled a rounding short of a stop stacks with the stop.
/// The window closes on the stop itself, so the material laid up to it is
/// exact whether or not the travel after it is already laid when the
/// extrusion is projected: with one emit per segment it is not, with a
/// batch of segments it is.
#[test]
fn the_extruder_lays_the_same_material_whatever_the_emit_batching() {
    let length = length_stacking_a_node_on_the_stop();
    let per_segment = extruder_at_segment_ends(&emitted_per_segment(lowered(
        &print_with_a_stop(length),
    )));
    let batched = extruder_at_segment_ends(&emitted_in_batches(
        lowered(&print_with_a_stop(length)),
        EMIT_BATCH_SEGMENTS,
    ));
    assert_eq!(per_segment.len(), batched.len());
    for (&(t, e_alone), &(t_batched, e_batched)) in per_segment.iter().zip(&batched) {
        assert_eq!(t, t_batched);
        assert!(
            (e_alone - e_batched).abs() <= 1e-12,
            "extrusion {length} mm long: the extruder ends the segment at t={t} at {e_alone} mm \
             emitted per segment and at {e_batched} mm emitted {EMIT_BATCH_SEGMENTS} segments at \
             a time"
        );
    }
}
