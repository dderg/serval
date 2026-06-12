use crate::topp::chain::ChainGrid;

pub(crate) fn emit_base_follower_rows(
    chain: &ChainGrid,
    off_b: usize,
    off_a: usize,
    mut push_row: impl FnMut(&[(usize, f64)], f64),
) -> usize {
    let mut count = 0;
    for i in 0..chain.s.len() {
        let lim = chain.limits_at(i);
        for f in chain.followers_at(i) {
            if f.pa_k != 0.0 {
                continue;
            }
            let r = f.ratio.abs();
            for (_, set) in lim.follower_sets() {
                if !set.axes.contains(f.axis) {
                    continue;
                }
                if set.v_max.is_finite() {
                    let cap = (set.v_max / r).powi(2);
                    push_row(&[(off_b + i, -1.0)], cap);
                    count += 1;
                }
                if set.a_max.is_finite() {
                    push_row(&[(off_a + i, -r)], set.a_max);
                    push_row(&[(off_a + i, r)], set.a_max);
                    count += 2;
                }
            }
        }
    }
    count
}

#[cfg(test)]
mod tests;
