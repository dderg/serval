use crate::frontend::Move;
use crate::path::CurvatureProfile;
use crate::path::lowering::PositionProfile;

use super::super::CornerFitConfig;
use super::super::kernels::{EPMM_MIN, cocircular, epmm, grow_turning_band, run_vertices};
use super::super::line_of;
use super::super::vec3::{cross, normalize, turn_normal};
use super::{Heart, turning_signal};

pub(super) struct KappaSignal;

impl Heart for KappaSignal {
    fn arc_spans(
        &self,
        chain: &[Move],
        tol: f64,
        min_run: usize,
        corner: CornerFitConfig,
    ) -> Vec<(usize, usize)> {
        let n = chain.len();
        if n < min_run || chain.iter().any(|m| line_of(m).is_none()) {
            return Vec::new();
        }
        let Some(plane) = chain_plane(chain) else {
            return Vec::new();
        };
        let verts = run_vertices(chain);
        let (s, theta) = turning_signal(&verts, plane);

        let gate_epmm = chain.iter().any(|m| epmm(m) > EPMM_MIN);
        let mut spans = Vec::new();
        let mut i = 0;
        while i + 1 < n {
            if gate_epmm && epmm(&chain[i]) < EPMM_MIN {
                i += 1;
                continue;
            }
            let band_end = grow_turning_band(chain, i, corner, gate_epmm);
            let mut span_start = i;
            while span_start + min_run <= band_end + 1 {
                let span_end = grow_slope_span(&s, &theta, span_start, band_end, min_run);
                match in_band_prefix(chain, span_start, span_end, min_run, tol) {
                    Some(end) => {
                        spans.push((span_start, end));
                        span_start = end + 1;
                    }
                    None => span_start += 1,
                }
            }
            i = band_end + 1;
        }
        spans
    }
}

fn chain_plane(chain: &[Move]) -> Option<[f64; 3]> {
    for w in chain.windows(2) {
        let (a, b) = (line_of(&w[0])?, line_of(&w[1])?);
        let t_in = a.heading_at(a.s_len());
        let t_out = b.heading_at(0.0);
        if let Some(v) = turn_normal(t_in, t_out) {
            return Some(normalize(cross(t_in, v)));
        }
    }
    None
}

fn in_band_prefix(
    chain: &[Move],
    start: usize,
    span_end: usize,
    min_run: usize,
    tol: f64,
) -> Option<usize> {
    let mut end = span_end;
    while end + 1 - start >= min_run {
        if cocircular(&chain[start..=end], tol) {
            return Some(end);
        }
        end -= 1;
    }
    None
}

#[derive(Default, Clone, Copy)]
struct SlopeFit {
    n: f64,
    sx: f64,
    sy: f64,
    sxx: f64,
    sxy: f64,
}

impl SlopeFit {
    fn push(&mut self, x: f64, y: f64) {
        self.n += 1.0;
        self.sx += x;
        self.sy += y;
        self.sxx += x * x;
        self.sxy += x * y;
    }

    fn slope(&self) -> f64 {
        let denom = self.n * self.sxx - self.sx * self.sx;
        debug_assert!(
            denom > 0.0,
            "slope fit needs >=2 distinct arc-length samples"
        );
        (self.n * self.sxy - self.sx * self.sy) / denom
    }
}

fn grow_slope_span(
    s: &[f64],
    theta: &[f64],
    start: usize,
    band_end: usize,
    min_run: usize,
) -> usize {
    let lo = start + 1;
    let mut end = (start + min_run - 1).min(band_end);
    let mut fit = SlopeFit::default();
    for v in lo..=end + 1 {
        fit.push(s[v], theta[v]);
    }
    let mut slope = fit.slope();
    while end < band_end {
        let cand = end + 2;
        let mut trial = fit;
        trial.push(s[cand], theta[cand]);
        let next = trial.slope();
        if slope == 0.0 || next.signum() != slope.signum() {
            break;
        }
        fit = trial;
        slope = next;
        end += 1;
    }
    end
}

#[cfg(test)]
mod tests;
