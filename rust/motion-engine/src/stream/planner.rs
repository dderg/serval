use crossbeam_channel::{Receiver, Sender};
use geometry::Move;

use super::PlannedMove;

pub(crate) struct Planner {
    entry_v: f64,
}

impl Planner {
    pub(crate) fn new() -> Self {
        Self { entry_v: 0.0 }
    }

    pub(crate) fn run(mut self, input: Receiver<Move>, output: Sender<PlannedMove>) {
        // TODO: accumulate fitted moves, plan a velocity profile, zip
        // geometry+velocity, and emit PlannedMoves.
        while let Ok(m) = input.recv() {
            let _ = (&mut self.entry_v, &output, m);
            todo!();
        }
    }
}
