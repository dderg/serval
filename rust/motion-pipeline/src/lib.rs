#![allow(clippy::result_large_err)]

use crossbeam_channel::Sender;
use trajectory::{AxisChainSet, ContinuousSegment};

pub mod fit_stage;
mod follower_projection;
mod lower_stage;
mod lowering;
pub mod planner;
mod shaper;
pub mod timing;
pub mod types;

use fit_stage::{FitDriver, FitStage};
use planner::Planner;
use shaper::Shaper;

pub use lower_stage::Lowerer;
pub use types::{
    BarrierAck, BaseItem, CONTIGUITY_EPS_MM, Control, DispatchCommand, PlannedItem, StreamConfig,
    StreamError, StreamInput, TrajectoryItem, advance_odometer,
};

pub struct Pipeline {
    fit: FitDriver,
    planner: Planner,
    lowerer: Lowerer,
    shaper: Shaper,
    fitted_tap: Option<Sender<geometry::Move>>,
}

impl Pipeline {
    pub fn new(
        config: StreamConfig,
        chains: AxisChainSet,
        home_pos: Vec<f64>,
        t_start: f64,
    ) -> Self {
        Self {
            fit: FitStage::new(config.corner).into_driver(),
            planner: Planner::new(config),
            lowerer: Lowerer::new(chains.rest_support(), home_pos, t_start),
            shaper: Shaper::new(chains, config.fit_tol()),
            fitted_tap: None,
        }
    }

    #[must_use]
    pub fn with_fitted_tap(mut self, tap: Sender<geometry::Move>) -> Self {
        self.fitted_tap = Some(tap);
        self
    }

    #[must_use]
    pub fn with_toolhead_tap(mut self, tap: Sender<ContinuousSegment>) -> Self {
        self.shaper = self.shaper.with_toolhead_tap(tap);
        self
    }

    pub fn feed(
        &mut self,
        item: StreamInput,
        output: &mut impl FnMut(TrajectoryItem) -> bool,
    ) -> bool {
        let Self {
            fit,
            planner,
            lowerer,
            shaper,
            fitted_tap,
        } = self;
        fit.feed(item, &mut |item| {
            if let (Some(tap), StreamInput::Move(m)) = (fitted_tap.as_ref(), &item) {
                tap.send(m.clone())
                    .expect("pipeline fitted observer disconnected");
            }
            planner.feed(item, &mut |item| {
                lowerer.feed(item, &mut |item| shaper.feed(item, output))
            })
        })
    }

    pub fn idle(&mut self, output: &mut impl FnMut(TrajectoryItem) -> bool) -> bool {
        let Self {
            planner,
            lowerer,
            shaper,
            ..
        } = self;
        planner.idle(&mut |item| lowerer.feed(item, &mut |item| shaper.feed(item, output)))
    }

    pub fn finish(&mut self, output: &mut impl FnMut(TrajectoryItem) -> bool) -> bool {
        let Self {
            fit,
            planner,
            lowerer,
            shaper,
            fitted_tap,
        } = self;
        fit.finish(&mut |item| {
            if let (Some(tap), StreamInput::Move(m)) = (fitted_tap.as_ref(), &item) {
                tap.send(m.clone())
                    .expect("pipeline fitted observer disconnected");
            }
            planner.feed(item, &mut |item| {
                lowerer.feed(item, &mut |item| shaper.feed(item, output))
            })
        }) && planner.finish(&mut |item| lowerer.feed(item, &mut |item| shaper.feed(item, output)))
            && shaper.finish(output)
    }
}

#[cfg(test)]
mod nonlinear_ripple_fuzz;
#[cfg(test)]
mod phase_ripple_fuzz;
#[cfg(test)]
mod tests;
