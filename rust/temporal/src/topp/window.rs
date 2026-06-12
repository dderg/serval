use nurbs::algebra::PiecewisePolynomialKernel;

pub fn frozen_time_map(b: &[f64], h_intervals: &[f64]) -> Vec<f64> {
    assert_eq!(b.len(), h_intervals.len() + 1);
    let mut t = Vec::with_capacity(b.len());
    let mut acc = 0.0;
    t.push(0.0);
    for (i, h) in h_intervals.iter().enumerate() {
        let v_sum = b[i].max(0.0).sqrt() + b[i + 1].max(0.0).sqrt();
        assert!(
            v_sum > 0.0,
            "frozen time map: zero speed across interval {i}"
        );
        acc += 2.0 * h / v_sum;
        t.push(acc);
    }
    t
}

pub fn eval_kernel(kernel: &PiecewisePolynomialKernel<f64>, z: f64) -> f64 {
    let (k_lo, k_hi) = kernel.support();
    if z < k_lo || z > k_hi {
        return 0.0;
    }
    for p in &kernel.pieces {
        if z >= p.u_start - 1e-15 && z <= p.u_end + 1e-15 {
            return p.evaluate(z);
        }
    }
    0.0
}

#[derive(Debug, Clone)]
pub struct WindowRow {
    pub weights: Vec<(usize, f64)>,
    pub history: f64,
}

#[derive(Debug, Clone)]
pub struct WindowOperator {
    rows: Vec<WindowRow>,
}

#[derive(Debug, Clone, Default)]
pub struct WindowHistory {
    pub dt: f64,
    /// Pre-chain signal values; sample `m` sits at `t = -(m + 0.5) * dt`.
    pub samples: Vec<f64>,
}

impl WindowHistory {
    #[must_use]
    pub fn empty() -> Self {
        Self::default()
    }

    #[must_use]
    pub fn constant_signal(value: f64, duration: f64, n: usize) -> Self {
        assert!(n > 0 && duration > 0.0);
        Self {
            dt: duration / n as f64,
            samples: vec![value; n],
        }
    }
}

impl WindowOperator {
    #[must_use]
    pub fn identity(n: usize) -> Self {
        Self {
            rows: (0..n)
                .map(|i| WindowRow {
                    weights: vec![(i, 1.0)],
                    history: 0.0,
                })
                .collect(),
        }
    }

    #[must_use]
    pub fn from_kernel(
        kernel: &PiecewisePolynomialKernel<f64>,
        t_map: &[f64],
        history: &WindowHistory,
    ) -> Self {
        Self::from_kernel_with_terminal(kernel, t_map, history, &WindowHistory::empty())
    }

    /// `terminal` holds post-chain signal samples; sample `m` sits at
    /// `t_end + (m + 0.5) * dt`. Kernel mass beyond the supplied samples
    /// falls back to holding the final chain sample.
    pub fn from_kernel_with_terminal(
        kernel: &PiecewisePolynomialKernel<f64>,
        t_map: &[f64],
        history: &WindowHistory,
        terminal: &WindowHistory,
    ) -> Self {
        let n = t_map.len();
        assert!(n >= 2, "window operator needs at least two samples");
        let (k_lo, k_hi) = kernel.support();
        let rows = (0..n)
            .map(|i| {
                let ti = t_map[i];
                let mut weights = Vec::new();
                let mut mass = 0.0;
                for (j, &tj) in t_map.iter().enumerate() {
                    let z = ti - tj;
                    if z < k_lo || z > k_hi {
                        continue;
                    }
                    let q = trapezoid_weight(t_map, j);
                    let w = eval_kernel(kernel, z) * q;
                    if w != 0.0 {
                        weights.push((j, w));
                        mass += w;
                    }
                }
                let mut history_value = 0.0;
                if history.dt > 0.0 {
                    for (m, &sample) in history.samples.iter().enumerate() {
                        let tau = -(m as f64 + 0.5) * history.dt;
                        let k = eval_kernel(kernel, ti - tau);
                        if k != 0.0 {
                            history_value += k * history.dt * sample;
                            mass += k * history.dt;
                        }
                    }
                }
                if terminal.dt > 0.0 {
                    let t_end = t_map[n - 1];
                    for (m, &sample) in terminal.samples.iter().enumerate() {
                        let tau = t_end + (m as f64 + 0.5) * terminal.dt;
                        let k = eval_kernel(kernel, ti - tau);
                        if k != 0.0 {
                            history_value += k * terminal.dt * sample;
                            mass += k * terminal.dt;
                        }
                    }
                }
                let leftover = 1.0 - mass;
                if leftover > 0.0 && ti + k_hi > t_map[n - 1] {
                    let last = weights
                        .iter_mut()
                        .rev()
                        .find(|(j, _)| *j == n - 1)
                        .map(|(_, w)| w);
                    match last {
                        Some(w) => *w += leftover,
                        None => weights.push((n - 1, leftover)),
                    }
                    mass = 1.0;
                }
                assert!(mass > 0.0, "window row {i}: zero kernel mass");
                for (_, w) in &mut weights {
                    *w /= mass;
                }
                WindowRow {
                    weights,
                    history: history_value / mass,
                }
            })
            .collect();
        Self { rows }
    }

    #[must_use]
    pub fn row(&self, i: usize) -> &WindowRow {
        &self.rows[i]
    }

    #[must_use]
    pub fn n_rows(&self) -> usize {
        self.rows.len()
    }
}

fn trapezoid_weight(t: &[f64], j: usize) -> f64 {
    let n = t.len();
    if j == 0 {
        (t[1] - t[0]) / 2.0
    } else if j == n - 1 {
        (t[n - 1] - t[n - 2]) / 2.0
    } else {
        (t[j + 1] - t[j - 1]) / 2.0
    }
}

#[cfg(test)]
mod tests;
