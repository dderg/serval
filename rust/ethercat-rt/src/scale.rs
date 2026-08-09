#[derive(Debug, Clone, Copy)]
pub struct CountMap {
    pub counts_per_mm: f64,
    pub origin_counts: i32,
    pub origin_mm: f64,
}

impl CountMap {
    pub fn new(counts_per_mm: f64, actual_counts: i32, pos_mm: f64) -> Self {
        Self {
            counts_per_mm,
            origin_counts: actual_counts,
            origin_mm: pos_mm,
        }
    }

    pub fn target_counts(&self, pos_mm: f64) -> i32 {
        let delta = (pos_mm - self.origin_mm) * self.counts_per_mm;
        self.origin_counts + delta.round() as i32
    }
}

pub fn mm_to_counts(pos_mm: f64, counts_per_mm: f64) -> i32 {
    (pos_mm * counts_per_mm).round() as i32
}

/// 606Ch (velocity actual) on these drives reports encoder counts per second,
/// not rpm — the same convention servo_fit_compare.py uses. Dividing by the
/// SIGNED counts-per-mm maps into host-frame mm/s, so inverted slots come out
/// with the host sign.
pub fn velocity_mm_s(counts_per_s: i32, cmd_counts_per_mm: f64) -> f64 {
    f64::from(counts_per_s) / cmd_counts_per_mm
}

#[cfg(test)]
mod tests;
