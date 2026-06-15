use crate::multi::junction::JunctionKind;
use crate::{BindingSummary, GridSample, TopProfile};
use std::ops::RangeInclusive;

#[allow(clippy::range_minus_one)]
pub(crate) fn partition_chains(
    n_segments: usize,
    kinds: &[JunctionKind],
) -> Vec<RangeInclusive<usize>> {
    debug_assert_eq!(kinds.len() + 1, n_segments);
    let mut chains = Vec::new();
    let mut start = 0;
    for (k, kind) in kinds.iter().enumerate() {
        if *kind == JunctionKind::Corner {
            chains.push(start..=k);
            start = k + 1;
        }
    }
    chains.push(start..=n_segments - 1);
    chains
}

pub(crate) fn slice_chain_profile(
    chain: &TopProfile,
    segment_ranges: &[(usize, usize)],
) -> Vec<TopProfile> {
    segment_ranges
        .iter()
        .enumerate()
        .map(|(i, &(lo, hi))| {
            let s0 = chain.samples[lo].s;
            let mut samples: Vec<GridSample> = chain.samples[lo..=hi]
                .iter()
                .map(|smp| GridSample {
                    s: smp.s - s0,
                    ..*smp
                })
                .collect();
            if samples.len() == 1 {
                samples.push(samples[0]);
            }
            let mut total_time = 0.0;
            for w in samples.windows(2) {
                let ds = w[1].s - w[0].s;
                let v_sum = w[0].v + w[1].v;
                total_time += if v_sum > 1e-12 {
                    ds * 2.0 / v_sum
                } else {
                    ds / 1e-9_f64.max(w[0].v.max(w[1].v))
                };
            }
            if matches!(chain.status, crate::SolveStatus::Infeasible { .. }) {
                total_time = f64::INFINITY;
            }
            let binding = if i == 0 {
                chain.binding.clone()
            } else {
                BindingSummary::default()
            };
            TopProfile {
                samples,
                status: chain.status,
                grid_scheme: chain.grid_scheme,
                total_time,
                binding,
                deadline_truncated: chain.deadline_truncated,
            }
        })
        .collect()
}

#[cfg(test)]
mod tests;
