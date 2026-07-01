use crossbeam_channel::{Receiver, Sender};
use geometry::{ChainFitConfig, Move};

pub(crate) struct Fitter {
    config: ChainFitConfig,
}

impl Fitter {
    pub(crate) fn new(config: ChainFitConfig) -> Self {
        Self { config }
    }

    pub(crate) fn run(self, input: Receiver<Move>, output: Sender<Move>) {
        // TODO: buffer raw moves, fit, decide when to commit, emit fitted moves
        while let Ok(m) = input.recv() {
            let _ = (&self.config, &output, m);
            todo!();
        }
    }
}
