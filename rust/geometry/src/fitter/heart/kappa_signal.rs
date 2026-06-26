use crate::frontend::Move;
use crate::path::CurvatureProfile;
use crate::path::lowering::PositionProfile;

use super::super::CornerFitConfig;
use super::super::kernels::{EPMM_MIN, cocircular, epmm, grow_turning_band, run_vertices};
use super::super::line_of;
use super::super::vec3::{cross, normalize, turn_normal};
use super::{Heart, turning_signal};

const KAPPA_REL_TOL: f64 = 0.25;

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
                let span_end = grow_kappa_span(&s, &theta, span_start, band_end);
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

fn leg_kappa(s: &[f64], theta: &[f64], vertex: usize) -> f64 {
    let turn = theta[vertex + 1] - theta[vertex];
    let span = 0.5 * (s[vertex + 1] - s[vertex - 1]);
    debug_assert!(span > 0.0, "leg span must be positive (legs are nonzero)");
    turn / span
}

fn grow_kappa_span(s: &[f64], theta: &[f64], start: usize, band_end: usize) -> usize {
    let mut end = start + 1;
    let mut mean = leg_kappa(s, theta, start + 1);
    let mut count = 1.0_f64;
    while end < band_end {
        let next = leg_kappa(s, theta, end + 1);
        if mean == 0.0
            || next.signum() != mean.signum()
            || (next - mean).abs() > KAPPA_REL_TOL * mean.abs()
        {
            break;
        }
        mean = (mean * count + next) / (count + 1.0);
        count += 1.0;
        end += 1;
    }
    end
}

#[cfg(test)]
mod tests;
