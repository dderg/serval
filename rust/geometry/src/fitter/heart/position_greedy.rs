use crate::frontend::Move;

use super::super::CornerFitConfig;
use super::super::kernels::{epmm, grow_cocircular_span, grow_turning_band};
use super::super::line_of;
use super::Heart;

pub(super) struct PositionGreedy;

impl Heart for PositionGreedy {
    fn arc_spans(
        &self,
        chain: &[Move],
        tol: f64,
        min_run: usize,
        corner: CornerFitConfig,
    ) -> Vec<(usize, usize)> {
        let gate_epmm = chain
            .iter()
            .any(|m| epmm(m) > super::super::kernels::EPMM_MIN);
        let mut spans = Vec::new();
        let n = chain.len();
        let mut i = 0;
        while i + 1 < n {
            if line_of(&chain[i]).is_none()
                || (gate_epmm && epmm(&chain[i]) < super::super::kernels::EPMM_MIN)
            {
                i += 1;
                continue;
            }
            let band_end = grow_turning_band(chain, i, corner, gate_epmm);
            let mut span_start = i;
            while span_start + min_run <= band_end + 1 {
                let span_end = grow_cocircular_span(chain, span_start, band_end, tol);
                if span_end + 1 - span_start >= min_run {
                    spans.push((span_start, span_end));
                    span_start = span_end + 1;
                } else {
                    span_start += 1;
                }
            }
            i = band_end + 1;
        }
        spans
    }
}

#[cfg(test)]
mod tests;
