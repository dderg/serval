#![allow(clippy::result_large_err)]

use crossbeam_channel::Sender;
use trajectory::{AxisChainSet, ContinuousSegment};

#[cfg(feature = "fit-audit")]
pub mod fit_audit;
pub mod fit_stage;
mod follower_projection;
mod lower_stage;
mod lowering;
pub mod planner;
mod shaper;
mod staged;
pub mod timing;
pub mod types;

use fit_stage::{FitDriver, FitStage};
use planner::Planner;
use shaper::Shaper;

pub use lower_stage::Lowerer;
pub use staged::{SHAPER_INBOX_CAP, StagedPipeline};
pub use types::{
    BarrierAck, BaseItem, CONTIGUITY_EPS_MM, Control, DispatchCommand, PlannedItem, StreamConfig,
    StreamError, StreamInput, TrajectoryItem, advance_odometer,
};

pub struct Pipeline {
    front: PlanningFront,
    shaper: Shaper,
}

impl Pipeline {
    pub fn new(
        config: StreamConfig,
        chains: AxisChainSet,
        home_pos: Vec<f64>,
        t_start: f64,
    ) -> Self {
        Self {
            front: PlanningFront {
                fit: FitStage::new(config.corner).into_driver(),
                planner: Planner::new(config),
                lowerer: Lowerer::new(chains.rest_support(), home_pos, t_start),
                fitted_tap: None,
            },
            shaper: Shaper::new(chains, config.fit_tol()),
        }
    }

    #[must_use]
    pub fn with_fitted_tap(mut self, tap: Sender<geometry::Move>) -> Self {
        self.front.fitted_tap = Some(tap);
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
        let Self { front, shaper } = self;
        front.feed(item, &mut |item| shaper.feed(item, output))
    }

    pub fn idle(&mut self, output: &mut impl FnMut(TrajectoryItem) -> bool) -> bool {
        let Self { front, shaper } = self;
        front.idle(&mut |item| shaper.feed(item, output))
    }

    pub fn finish(&mut self, output: &mut impl FnMut(TrajectoryItem) -> bool) -> bool {
        let Self { front, shaper } = self;
        front.finish(&mut |item| shaper.feed(item, output)) && shaper.finish(output)
    }
}

/// Every stage ahead of the shaper: fitting, velocity planning, lowering.
struct PlanningFront {
    fit: FitDriver,
    planner: Planner,
    lowerer: Lowerer,
    fitted_tap: Option<Sender<geometry::Move>>,
}

impl PlanningFront {
    fn feed(&mut self, item: StreamInput, shape: &mut impl FnMut(BaseItem) -> bool) -> bool {
        let Self {
            fit,
            planner,
            lowerer,
            fitted_tap,
        } = self;
        fit.feed(item, &mut |item| {
            Self::tap_fitted(fitted_tap.as_ref(), &item);
            planner.feed(item, &mut |item| lowerer.feed(item, shape))
        })
    }

    fn idle(&mut self, shape: &mut impl FnMut(BaseItem) -> bool) -> bool {
        let Self {
            planner, lowerer, ..
        } = self;
        planner.idle(&mut |item| lowerer.feed(item, shape))
    }

    fn finish(&mut self, shape: &mut impl FnMut(BaseItem) -> bool) -> bool {
        let Self {
            fit,
            planner,
            lowerer,
            fitted_tap,
        } = self;
        fit.finish(&mut |item| {
            Self::tap_fitted(fitted_tap.as_ref(), &item);
            planner.feed(item, &mut |item| lowerer.feed(item, shape))
        }) && planner.finish(&mut |item| lowerer.feed(item, shape))
    }

    fn tap_fitted(tap: Option<&Sender<geometry::Move>>, item: &StreamInput) {
        if let (Some(tap), StreamInput::Move(m)) = (tap, item) {
            tap.send(m.clone())
                .expect("pipeline fitted observer disconnected");
        }
    }
}

#[cfg(test)]
mod nonlinear_ripple_fuzz;
#[cfg(test)]
mod phase_ripple_fuzz;
#[cfg(test)]
mod tests;
