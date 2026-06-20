#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JerkSample {
    pub j_t: f64,
    pub j_n: f64,
    pub j_n_geom: f64,
    pub j_n_couple: f64,
}

#[must_use]
pub fn jerk_at(kappa: f64, dkappa_ds: f64, v: f64, a_t: f64, seg_jerk: f64) -> JerkSample {
    let j_n_geom = dkappa_ds * v * v * v;
    let j_n_couple = 2.0 * kappa * v * a_t;
    JerkSample {
        j_t: seg_jerk,
        j_n: j_n_geom + j_n_couple,
        j_n_geom,
        j_n_couple,
    }
}

#[cfg(test)]
mod tests;
