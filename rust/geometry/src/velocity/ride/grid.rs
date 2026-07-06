use super::super::disk::disk_rail_accel;
use super::{POS_EPS_MM, Track};

const RANGE_MIN_BLOCK: usize = 64;
const KAPPA_EPS: f64 = 1e-12;

pub(super) struct Grid<'a> {
    pub(super) t: &'a Track<'a>,
    cap_range_min: Vec<f64>,
}

impl<'a> Grid<'a> {
    pub(super) fn new(t: &'a Track<'a>) -> Self {
        let cap_range_min = t
            .cap_v
            .chunks(RANGE_MIN_BLOCK)
            .map(|c| c.iter().fold(f64::INFINITY, |m, &x| m.min(x)))
            .collect();
        Self { t, cap_range_min }
    }

    pub(super) fn n(&self) -> usize {
        self.t.s.len()
    }

    /// Cell whose span contains `s` (clamped).
    pub(super) fn cell(&self, s: f64) -> usize {
        let n = self.n();
        let i = self.t.s.partition_point(|&x| x <= s);
        i.clamp(1, n - 1) - 1
    }

    pub(super) fn lerp_node(&self, arr: &[f64], s: f64) -> f64 {
        let c = self.cell(s);
        let span = self.t.s[c + 1] - self.t.s[c];
        let f = if span > POS_EPS_MM {
            ((s - self.t.s[c]) / span).clamp(0.0, 1.0)
        } else {
            0.0
        };
        arr[c] + f * (arr[c + 1] - arr[c])
    }

    /// Cap speed at `s`, linear in `v²` between nodes: a constant-decel brake
    /// segment of the envelope is linear in `v²`, so this represents it
    /// exactly where a linear-in-`v` chord would sag below it.
    pub(super) fn cap_at(&self, s: f64) -> f64 {
        let c = self.cell(s);
        let span = self.t.s[c + 1] - self.t.s[c];
        let f = if span > POS_EPS_MM {
            ((s - self.t.s[c]) / span).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let (w0, w1) = (
            self.t.cap_v[c] * self.t.cap_v[c],
            self.t.cap_v[c + 1] * self.t.cap_v[c + 1],
        );
        (w0 + f * (w1 - w0)).max(0.0).sqrt()
    }

    pub(super) fn slope_at(&self, s: f64) -> f64 {
        self.t.cap_a[self.cell(s)]
    }

    pub(super) fn rail_at(&self, s: f64, v: f64) -> f64 {
        disk_rail_accel(
            self.lerp_node(self.t.accel, s),
            self.lerp_node(self.t.kappa, s),
            v,
        )
    }

    pub(super) fn kappa_at(&self, s: f64) -> f64 {
        self.lerp_node(self.t.kappa, s)
    }

    pub(super) fn curved_near(&self, s: f64) -> bool {
        let c = self.cell(s);
        self.t.kappa[c].abs() > KAPPA_EPS || self.t.kappa[c + 1].abs() > KAPPA_EPS
    }

    /// Lower bound of the cap over `[s, s + dist]`.
    pub(super) fn cap_min_over(&self, s: f64, dist: f64) -> f64 {
        let lo = self.cell(s);
        let hi = self.cell(s + dist) + 1;
        let (b_lo, b_hi) = (lo / RANGE_MIN_BLOCK, hi / RANGE_MIN_BLOCK);
        let mut m = f64::INFINITY;
        if b_lo == b_hi {
            for &x in &self.t.cap_v[lo..=hi] {
                m = m.min(x);
            }
            return m;
        }
        for &x in &self.t.cap_v[lo..(b_lo + 1) * RANGE_MIN_BLOCK] {
            m = m.min(x);
        }
        for &x in &self.cap_range_min[(b_lo + 1)..b_hi] {
            m = m.min(x);
        }
        for &x in &self.t.cap_v[b_hi * RANGE_MIN_BLOCK..=hi] {
            m = m.min(x);
        }
        m
    }

    pub(super) fn end_s(&self) -> f64 {
        self.t.s[self.n() - 1]
    }
}
