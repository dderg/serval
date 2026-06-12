pub fn b_d_weights(hl: f64, hr: f64) -> [f64; 3] {
    debug_assert!(hl > 0.0 && hr > 0.0);
    let d = hl * hr * (hl + hr);
    [-hr * hr / d, (hr * hr - hl * hl) / d, hl * hl / d]
}

pub fn b_dd_weights(hl: f64, hr: f64) -> [f64; 3] {
    debug_assert!(hl > 0.0 && hr > 0.0);
    let d = hl * hr * (hl + hr);
    [2.0 * hr / d, -2.0 * (hl + hr) / d, 2.0 * hl / d]
}

pub fn stencil_at(i: usize, n: usize, h_intervals: &[f64]) -> ([usize; 3], f64, f64) {
    debug_assert!(n >= 3 && i < n && h_intervals.len() == n - 1);
    if i == 0 {
        ([0, 1, 2], h_intervals[0], h_intervals[1])
    } else if i == n - 1 {
        (
            [n - 3, n - 2, n - 1],
            h_intervals[n - 3],
            h_intervals[n - 2],
        )
    } else {
        ([i - 1, i, i + 1], h_intervals[i - 1], h_intervals[i])
    }
}

pub fn s_dddot_at_weights(b: &[f64], i: usize, h_intervals: &[f64]) -> f64 {
    let n = b.len();
    let (idx, hl, hr) = stencil_at(i, n, h_intervals);
    let w = b_dd_weights(hl, hr);
    let b_dd = w[0] * b[idx[0]] + w[1] * b[idx[1]] + w[2] * b[idx[2]];
    b[i].max(0.0).sqrt() * b_dd / 2.0
}

pub fn fornberg_weights(x0: f64, xs: &[f64], order: usize) -> Vec<Vec<f64>> {
    let n = xs.len();
    assert!(n > order, "need more nodes than derivative order");
    let mut w = vec![vec![0.0; n]; order + 1];
    let mut c1 = 1.0;
    let mut c4 = xs[0] - x0;
    w[0][0] = 1.0;
    for i in 1..n {
        let mn = i.min(order);
        let mut c2 = 1.0;
        let c5 = c4;
        c4 = xs[i] - x0;
        for j in 0..i {
            let c3 = xs[i] - xs[j];
            c2 *= c3;
            if j == i - 1 {
                for k in (1..=mn).rev() {
                    w[k][i] = c1 * (k as f64 * w[k - 1][i - 1] - c5 * w[k][i - 1]) / c2;
                }
                w[0][i] = -c1 * c5 * w[0][i - 1] / c2;
            }
            for k in (1..=mn).rev() {
                w[k][j] = (c4 * w[k][j] - k as f64 * w[k - 1][j]) / c3;
            }
            w[0][j] = c4 * w[0][j] / c3;
        }
        c1 = c2;
    }
    w
}

pub fn b_ddd_weights_at(i: usize, s: &[f64]) -> ([usize; 4], [f64; 4]) {
    let n = s.len();
    assert!(n >= 4, "third difference needs at least 4 points");
    let start = i.saturating_sub(1).min(n - 4);
    let idx = [start, start + 1, start + 2, start + 3];
    let xs = [s[idx[0]], s[idx[1]], s[idx[2]], s[idx[3]]];
    let w = fornberg_weights(s[i], &xs, 3);
    (idx, [w[3][0], w[3][1], w[3][2], w[3][3]])
}

pub fn s_ddddot_at_weights(b: &[f64], a_i: f64, i: usize, s: &[f64], h_intervals: &[f64]) -> f64 {
    let n = b.len();
    let (idx2, hl, hr) = stencil_at(i, n, h_intervals);
    let w2 = b_dd_weights(hl, hr);
    let b_dd = w2[0] * b[idx2[0]] + w2[1] * b[idx2[1]] + w2[2] * b[idx2[2]];
    let (idx3, w3) = b_ddd_weights_at(i, s);
    let b_ddd: f64 = (0..4).map(|k| w3[k] * b[idx3[k]]).sum();
    a_i * b_dd / 2.0 + b[i].max(0.0) * b_ddd / 2.0
}

#[cfg(test)]
mod tests;
