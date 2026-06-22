use std::collections::{HashSet, VecDeque};

use geometry::path::lowering::PositionProfile;
use geometry::{
    ChainFitConfig, FitError, Move, VelocityConfig, VelocityError, VelocityLimits, fit_chain,
    plan_velocity_warm_start,
};
use trajectory::{AxisChainSet, ShapedSegment};

use crate::lowering::{LoweringError, lower_move};

#[derive(Debug, Clone, Copy)]
pub struct StreamConfig {
    pub chain: ChainFitConfig,
    pub velocity: VelocityConfig,
    pub fit_tol_mm: f64,
    pub keep_secs: f64,
    pub limits: VelocityLimits,
}

#[derive(Debug)]
pub enum StreamError {
    Fit(FitError),
    Velocity(VelocityError),
    Lowering(LoweringError),
}

impl std::fmt::Display for StreamError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Fit(e) => write!(f, "chain fit: {e:?}"),
            Self::Velocity(e) => write!(f, "velocity plan: {e:?}"),
            Self::Lowering(e) => write!(f, "lowering: {e}"),
        }
    }
}

impl std::error::Error for StreamError {}

impl From<FitError> for StreamError {
    fn from(e: FitError) -> Self {
        Self::Fit(e)
    }
}
impl From<VelocityError> for StreamError {
    fn from(e: VelocityError) -> Self {
        Self::Velocity(e)
    }
}
impl From<LoweringError> for StreamError {
    fn from(e: LoweringError) -> Self {
        Self::Lowering(e)
    }
}

pub struct StreamState {
    buffer: VecDeque<Move>,
    entry_v: f64,
    odometer: Vec<f64>,
    t_committed: f64,
    config: StreamConfig,
    axis_chains: AxisChainSet,
}

impl StreamState {
    #[must_use]
    pub fn new(
        config: StreamConfig,
        axis_chains: AxisChainSet,
        home_pos: &[f64],
        t_start: f64,
    ) -> Self {
        Self {
            buffer: VecDeque::new(),
            entry_v: 0.0,
            odometer: home_pos.to_vec(),
            t_committed: t_start,
            config,
            axis_chains,
        }
    }

    pub fn set_axis_chains(&mut self, axis_chains: AxisChainSet) {
        self.axis_chains = axis_chains;
    }

    pub fn reset(&mut self, home_pos: &[f64], t_start: f64) {
        self.buffer.clear();
        self.entry_v = 0.0;
        self.odometer = home_pos.to_vec();
        self.t_committed = t_start;
    }

    pub fn push(&mut self, m: Move) {
        self.buffer.push_back(m);
    }

    pub fn advance_time(&mut self, dt: f64) {
        debug_assert!(
            self.buffer.is_empty(),
            "advance_time requires a drained buffer"
        );
        debug_assert_eq!(self.entry_v, 0.0, "advance_time requires rest at the seam");
        if dt > 0.0 {
            self.t_committed += dt;
        }
    }

    pub fn advance_idle(&mut self, target_t: f64) {
        debug_assert!(
            self.buffer.is_empty(),
            "advance_idle requires a drained buffer"
        );
        debug_assert_eq!(self.entry_v, 0.0, "advance_idle requires rest at the seam");
        if target_t > self.t_committed {
            self.t_committed = target_t;
        }
    }

    pub fn restart_idle_timeline(&mut self) {
        debug_assert!(
            self.buffer.is_empty(),
            "restart_idle_timeline requires a drained buffer"
        );
        debug_assert_eq!(
            self.entry_v, 0.0,
            "restart_idle_timeline requires rest at the seam"
        );
        self.t_committed = 0.0;
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.buffer.is_empty()
    }

    #[must_use]
    pub fn buffered(&self) -> usize {
        self.buffer.len()
    }

    #[must_use]
    pub fn t_committed(&self) -> f64 {
        self.t_committed
    }

    #[must_use]
    pub fn entry_velocity(&self) -> f64 {
        self.entry_v
    }

    #[must_use]
    pub fn limits(&self) -> VelocityLimits {
        self.config.limits
    }

    pub fn commit(&mut self, force: bool) -> Result<Vec<ShapedSegment>, StreamError> {
        if self.buffer.is_empty() {
            return Ok(Vec::new());
        }

        let moves: Vec<Move> = self.buffer.iter().cloned().collect();
        let outcome = fit_chain(&moves, self.config.chain)?;
        let profile = plan_velocity_warm_start(&outcome, self.config.velocity, self.entry_v)?;

        let n = outcome.moves.len();
        let mut pos = self.odometer.clone();
        let mut t = self.t_committed;
        let mut segs: Vec<ShapedSegment> = Vec::with_capacity(n);
        let mut start_times: Vec<f64> = Vec::with_capacity(n);
        for (gm, vm) in outcome.moves.iter().zip(&profile.moves) {
            start_times.push(t);
            let seg = lower_move(
                gm,
                vm,
                t,
                &pos,
                self.config.fit_tol_mm,
                &self.axis_chains.chains,
            )?;
            t = seg.t_end;
            advance_odometer(&mut pos, gm);
            segs.push(seg);
        }
        let total_t = t - self.t_committed;

        let commit_count = if force {
            n
        } else {
            let unblended: HashSet<u32> =
                outcome.report.unblended.iter().map(|u| u.line_no).collect();
            let limit_t = self.t_committed + (total_t - self.config.keep_secs);
            let mut chosen = 0usize;
            for i in 1..n {
                if unblended.contains(&outcome.moves[i].source.start_line) {
                    if start_times[i] <= limit_t {
                        chosen = i;
                    } else {
                        break;
                    }
                }
            }
            chosen
        };

        if commit_count == 0 {
            return Ok(Vec::new());
        }

        let committed: Vec<ShapedSegment> = segs.into_iter().take(commit_count).collect();

        let mut seam_pos = self.odometer.clone();
        for gm in outcome.moves.iter().take(commit_count) {
            advance_odometer(&mut seam_pos, gm);
        }
        self.odometer = seam_pos;
        self.t_committed = committed.last().expect("commit_count > 0").t_end;
        self.entry_v = if commit_count == n {
            0.0
        } else {
            profile.moves[commit_count - 1].exit_v
        };

        if commit_count == n {
            self.buffer.clear();
        } else {
            let keep_line = outcome.moves[commit_count].source.start_line;
            while self
                .buffer
                .front()
                .is_some_and(|m| m.source.start_line < keep_line)
            {
                self.buffer.pop_front();
            }
        }

        Ok(committed)
    }
}

fn advance_odometer(pos: &mut [f64], gm: &Move) {
    let s_len = gm.segment.s_len();
    if let Some(seg) = &gm.segment.spatial {
        let end = seg.point_at(s_len);
        for axis in 0..3.min(pos.len()) {
            pos[axis] = end[axis];
        }
    }
    for f in &gm.segment.followers {
        if let Some(slot) = pos.get_mut(f.axis_index) {
            *slot += f.ratio * s_len;
        }
    }
}

#[cfg(test)]
mod tests;
