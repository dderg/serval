use nurbs::bezier::extract_bezier_pieces;
use nurbs::ScalarNurbs;
use temporal::{restricted_norm, Limits};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UtilFamily {
    Velocity,
    Accel,
    Jerk,
}

#[derive(Debug, Clone, Copy)]
pub struct PeakUtilization {
    pub ratio: f64,
    pub family: UtilFamily,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct UtilizationPeaks {
    pub vel_ratio: f64,
    pub accel_ratio: f64,
    pub jerk_ratio: f64,
    pub vel_mag: f64,
    pub accel_mag: f64,
    pub jerk_mag: f64,
}

impl UtilizationPeaks {
    fn merge_max(&mut self, o: &UtilizationPeaks) {
        self.vel_ratio = self.vel_ratio.max(o.vel_ratio);
        self.accel_ratio = self.accel_ratio.max(o.accel_ratio);
        self.jerk_ratio = self.jerk_ratio.max(o.jerk_ratio);
        self.vel_mag = self.vel_mag.max(o.vel_mag);
        self.accel_mag = self.accel_mag.max(o.accel_mag);
        self.jerk_mag = self.jerk_mag.max(o.jerk_mag);
    }

    #[must_use]
    pub fn worst(&self) -> Option<PeakUtilization> {
        let mut best = (self.vel_ratio, UtilFamily::Velocity);
        if self.accel_ratio > best.0 {
            best = (self.accel_ratio, UtilFamily::Accel);
        }
        if self.jerk_ratio > best.0 {
            best = (self.jerk_ratio, UtilFamily::Jerk);
        }
        (best.0 > 0.0).then_some(PeakUtilization {
            ratio: best.0,
            family: best.1,
        })
    }
}

const MCU_DT: f64 = 25e-6;

fn norm3(x: &[f64; 3]) -> f64 {
    (x[0] * x[0] + x[1] * x[1] + x[2] * x[2]).sqrt()
}

#[must_use]
pub fn segment_peak_utilization(
    axes: &[ScalarNurbs<f64>],
    limits: &Limits,
) -> Option<UtilizationPeaks> {
    if axes.len() < 3 {
        return None;
    }
    let pieces = extract_bezier_pieces(&axes[0]);
    let t0 = pieces.first()?.u_start;
    let t1 = pieces.last()?.u_end;
    let dur = t1 - t0;
    if dur <= 4.0 * MCU_DT {
        return None;
    }

    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    let n = (dur / MCU_DT).ceil() as usize;
    let dt = dur / n as f64;

    let sample = |t: f64| -> [f64; 3] {
        [
            nurbs::eval::eval(&axes[0], t),
            nurbs::eval::eval(&axes[1], t),
            nurbs::eval::eval(&axes[2], t),
        ]
    };
    let x: Vec<[f64; 3]> = (0..=n).map(|i| sample(t0 + dt * i as f64)).collect();

    let mut p = UtilizationPeaks::default();
    for i in 2..=n - 2 {
        let mut v = [0.0_f64; 3];
        let mut a = [0.0_f64; 3];
        let mut j = [0.0_f64; 3];
        for k in 0..3 {
            v[k] = (x[i + 1][k] - x[i - 1][k]) / (2.0 * dt);
            a[k] = (x[i + 1][k] - 2.0 * x[i][k] + x[i - 1][k]) / (dt * dt);
            j[k] = (x[i + 2][k] - 2.0 * x[i + 1][k] + 2.0 * x[i - 1][k] - x[i - 2][k])
                / (2.0 * dt * dt * dt);
        }
        p.vel_mag = p.vel_mag.max(norm3(&v));
        p.accel_mag = p.accel_mag.max(norm3(&a));
        p.jerk_mag = p.jerk_mag.max(norm3(&j));
        for (_, set) in limits.spatial_sets() {
            if set.v_max.is_finite() {
                p.vel_ratio = p.vel_ratio.max(restricted_norm(&v, set.axes) / set.v_max);
            }
            if set.a_max.is_finite() {
                p.accel_ratio = p.accel_ratio.max(restricted_norm(&a, set.axes) / set.a_max);
            }
            if set.j_max.is_finite() {
                p.jerk_ratio = p.jerk_ratio.max(restricted_norm(&j, set.axes) / set.j_max);
            }
        }
    }

    Some(p)
}

#[must_use]
pub fn window_peak_utilization<'a>(
    segments: impl IntoIterator<Item = (&'a [ScalarNurbs<f64>], &'a Limits)>,
) -> Option<UtilizationPeaks> {
    let mut acc: Option<UtilizationPeaks> = None;
    for (axes, limits) in segments {
        if let Some(p) = segment_peak_utilization(axes, limits) {
            match &mut acc {
                Some(a) => a.merge_max(&p),
                None => acc = Some(p),
            }
        }
    }
    acc
}

#[cfg(test)]
mod tests;
