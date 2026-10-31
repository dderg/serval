use std::time::Duration;

const ALPHA: f64 = 0.125;
const BETA: f64 = 0.25;
const K: f64 = 4.0;
// 500 ms floor: prevents retransmit storms flooding firmware's 192-byte receive_buf
// during long-running command_task stalls (e.g. Renode). Driven by srtt+4×rttvar after first sample.
// Any mcu-side queue the host keeps fed must outlast this floor — a queue
// shallower than one retransmit empties while the link is silent.
pub const MIN_RTO_MS: u64 = 125;
pub const MIN_RTO: Duration = Duration::from_millis(MIN_RTO_MS);
pub const MAX_RTO: Duration = Duration::from_secs(5);
const G: Duration = Duration::from_millis(1);

#[derive(Debug)]
pub struct RttEstimator {
    srtt: Option<Duration>,
    rttvar: Option<Duration>,
    rto: Duration,
}

impl Default for RttEstimator {
    fn default() -> Self {
        Self {
            srtt: None,
            rttvar: None,
            rto: MIN_RTO,
        }
    }
}

impl RttEstimator {
    pub fn current_rto(&self) -> Duration {
        self.rto
    }
}

impl RttEstimator {
    pub fn update(&mut self, r: Duration) {
        match self.srtt {
            None => {
                self.srtt = Some(r);
                self.rttvar = Some(r / 2);
            }
            Some(srtt) => {
                let diff = srtt.abs_diff(r);
                let rttvar_new = self.rttvar.unwrap().mul_f64(1.0 - BETA) + diff.mul_f64(BETA);
                self.rttvar = Some(rttvar_new);
                self.srtt = Some(srtt.mul_f64(1.0 - ALPHA) + r.mul_f64(ALPHA));
            }
        }
        let k_rttvar = self.rttvar.unwrap().mul_f64(K);
        let rto_raw = self.srtt.unwrap() + std::cmp::max(G, k_rttvar);
        self.rto = rto_raw.clamp(MIN_RTO, MAX_RTO);
    }

    pub fn backoff(&mut self) {
        self.rto = (self.rto * 2).clamp(MIN_RTO, MAX_RTO);
    }
}

#[cfg(test)]
mod tests;
