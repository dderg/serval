use std::thread;

use crossbeam_channel::bounded;
use trajectory::AxisChainSet;

pub mod fit_stage;
pub mod lowerer;
pub mod lowering;
pub mod planner;
pub mod shaper;
pub mod timing;
mod types;

pub use fit_stage::FitStage;
pub use lowerer::{advance_odometer, dist3, run_lowerer};
pub use lowering::{FitTol, LoweringError, lower_move, lower_move_pieces};
pub use planner::Planner;
pub use shaper::Shaper;
pub use types::{
    BarrierAck, CONTIGUITY_EPS_MM, Control, LoweredItem, LoweredSegment, PipelineHandle,
    PlannedItem, PlannedMove, PostProcessError, ShapedItem, StreamConfig, StreamError, StreamInput,
    jerk_limited_brake_time,
};

/// Wires the pure stream stages (fit stage → planner → lowerer → shaper) into
/// OS threads. Production goes through `motion_engine::worker::setup_pipeline`,
/// which wraps these stages with the dispatcher and pump; this stage-only
/// wiring is also used standalone by offline consumers (seam harness,
/// trajectory dump) that have no hardware behind them.
pub fn setup_stages(
    config: StreamConfig,
    axis_chains: AxisChainSet,
    home_pos: Vec<f64>,
    t_start: f64,
) -> PipelineHandle {
    let (raw_tx, raw_rx) = bounded::<StreamInput>(64);
    let (fitted_tx, fitted_rx) = bounded::<StreamInput>(64);
    let (planned_tx, planned_rx) = bounded::<PlannedItem>(64);
    let (lowered_tx, lowered_rx) = bounded::<LoweredItem>(64);
    let (shaped_tx, shaped_rx) = bounded::<ShapedItem>(64);

    let mut chain = config.chain;
    chain.corner.ramp_accel_budget_mm_s2 = config.max_extrude_only_accel_mm_s2;
    let fit_stage = FitStage::new(chain);
    spawn_stage("kalico-fit", move || fit_stage.run(raw_rx, fitted_tx));

    let planner = Planner::new(config);
    spawn_stage("kalico-plan", move || planner.run(fitted_rx, planned_tx));

    let fit_tol = FitTol {
        pos_mm: config.fit_tol_mm,
        accel_mm_s2: config.fit_tol_accel_mm_s2,
    };
    let lower_chains = axis_chains.clone();
    spawn_stage("kalico-lower", move || {
        run_lowerer(
            planned_rx,
            lowered_tx,
            fit_tol,
            lower_chains,
            home_pos,
            t_start,
        );
    });

    let shaper = Shaper::new(axis_chains);
    spawn_stage("kalico-shape", move || shaper.run(lowered_rx, shaped_tx));

    PipelineHandle {
        input: raw_tx,
        output: shaped_rx,
    }
}

fn spawn_stage(name: &str, f: impl FnOnce() + Send + 'static) {
    thread::Builder::new()
        .name(name.to_string())
        .spawn(f)
        .unwrap_or_else(|e| panic!("spawn {name}: {e}"));
}

#[cfg(test)]
mod tests;
