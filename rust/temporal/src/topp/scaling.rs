use crate::Limits;
use crate::topp::path::ArclengthGrid;
use crate::topp::solver::SolverResult;

const V_TARGET_UNITS_PER_S: f64 = 10.0;

pub struct SolverScale {
    pub(crate) mm_per_unit: f64,
}

impl SolverScale {
    pub fn for_limits(limits: &Limits) -> Self {
        let sigma = limits.v_ceiling();
        if sigma <= 0.0 || !sigma.is_finite() {
            return Self::identity();
        }
        Self {
            mm_per_unit: sigma / V_TARGET_UNITS_PER_S,
        }
    }

    pub fn identity() -> Self {
        Self { mm_per_unit: 1.0 }
    }

    pub(crate) fn sigma(&self) -> f64 {
        self.mm_per_unit
    }

    pub(crate) fn scale_limits(&self, limits: &Limits) -> Limits {
        let s = self.sigma();
        limits.with_sets_mapped(|l| crate::LimitSet {
            axes: l.axes,
            v_max: l.v_max / s,
            a_max: l.a_max / s,
            j_max: l.j_max / s,
        })
    }

    pub(crate) fn scale_grid(&self, grid: &ArclengthGrid) -> ArclengthGrid {
        let s = self.sigma();
        ArclengthGrid {
            s: grid.s.iter().map(|v| v / s).collect(),
            u: grid.u.clone(),
            c: grid.c.iter().map(|p| p.map(|v| v / s)).collect(),
            c_prime: grid.c_prime.clone(),
            c_double_prime: grid
                .c_double_prime
                .iter()
                .map(|p| p.map(|v| v * s))
                .collect(),
            c_triple_prime: grid
                .c_triple_prime
                .iter()
                .map(|p| p.map(|v| v * s * s))
                .collect(),
            total_length: grid.total_length / s,
            inter_geom: grid
                .inter_geom
                .iter()
                .map(|iv| iv.iter().map(|smp| scale_inter_sample(smp, s)).collect())
                .collect(),
        }
    }

    pub(crate) fn scale_velocity(&self, v: f64) -> f64 {
        v / self.sigma()
    }

    pub(crate) fn unscale_result(&self, result: &mut SolverResult) {
        let s2 = self.sigma() * self.sigma();
        let s = self.sigma();
        for b in &mut result.b {
            *b *= s2;
        }
        for a in &mut result.a {
            *a *= s;
        }
    }

    pub(crate) fn unscale_b(&self, b: f64) -> f64 {
        b * self.sigma() * self.sigma()
    }

    pub(crate) fn to_scaled_b(&self, b: f64) -> f64 {
        let s2 = self.sigma() * self.sigma();
        b / s2
    }

    pub(crate) fn to_scaled_accel(&self, a: f64) -> f64 {
        a / self.sigma()
    }

    pub(crate) fn to_scaled_kappa(&self, kappa: f64) -> f64 {
        kappa * self.sigma()
    }

    pub fn for_chain(chain: &crate::topp::chain::ChainGrid) -> Self {
        let v_ceiling = chain
            .limits
            .iter()
            .map(Limits::v_ceiling)
            .fold(f64::NEG_INFINITY, f64::max);
        if v_ceiling <= 0.0 || !v_ceiling.is_finite() {
            return Self::identity();
        }

        let mut peak_mvc_b = 0.0_f64;
        let mut a_tan_max = 0.0_f64;
        for (i, g) in chain.geom.iter().enumerate() {
            scan_reachable_b(chain.limits_at(i), g, &mut peak_mvc_b, &mut a_tan_max);
        }
        for j in &chain.junctions {
            scan_reachable_b(
                &chain.limits[j.limits_idx],
                &j.geom,
                &mut peak_mvc_b,
                &mut a_tan_max,
            );
        }

        let length =
            chain.s.last().copied().unwrap_or(0.0) - chain.s.first().copied().unwrap_or(0.0);
        let b_reach = a_tan_max * length;
        let peak_b = match (peak_mvc_b > 0.0, b_reach > 0.0) {
            (true, true) => peak_mvc_b.min(b_reach),
            (true, false) => peak_mvc_b,
            (false, true) => b_reach,
            (false, false) => v_ceiling * v_ceiling,
        };

        let v_char = peak_b.sqrt().min(v_ceiling);
        let v_char = if v_char > 0.0 && v_char.is_finite() {
            v_char
        } else {
            v_ceiling
        };
        Self {
            mm_per_unit: v_char / V_TARGET_UNITS_PER_S,
        }
    }

    pub(crate) fn scale_chain_grid(
        &self,
        chain: &crate::topp::chain::ChainGrid,
    ) -> crate::topp::chain::ChainGrid {
        let s = self.sigma();
        let scale_geom = |g: &crate::topp::chain::PointGeom| crate::topp::chain::PointGeom {
            c_prime: g.c_prime,
            c_double_prime: g.c_double_prime.map(|v| v * s),
            c_triple_prime: g.c_triple_prime.map(|v| v * s * s),
        };
        crate::topp::chain::ChainGrid {
            s: chain.s.iter().map(|v| v / s).collect(),
            geom: chain.geom.iter().map(scale_geom).collect(),
            h_intervals: chain.h_intervals.iter().map(|h| h / s).collect(),
            limits_idx: chain.limits_idx.clone(),
            limits: chain.limits.iter().map(|l| self.scale_limits(l)).collect(),
            junctions: chain
                .junctions
                .iter()
                .map(|j| crate::topp::chain::JunctionDual {
                    idx: j.idx,
                    geom: scale_geom(&j.geom),
                    limits_idx: j.limits_idx,
                })
                .collect(),
            segment_ranges: chain.segment_ranges.clone(),
            followers: chain.followers.clone(),
            axis_kernels: chain.axis_kernels.clone(),
            follower_history: chain.follower_history.as_ref().map(|h| {
                let mut scaled = h.clone();
                for axis in &mut scaled.axis_velocity {
                    for v in axis.iter_mut() {
                        *v = self.scale_velocity(*v);
                    }
                }
                scaled
            }),
            follower_terminal: chain.follower_terminal.as_ref().map(|h| {
                let mut scaled = h.clone();
                for axis in &mut scaled.axis_velocity {
                    for v in axis.iter_mut() {
                        *v = self.scale_velocity(*v);
                    }
                }
                scaled
            }),
            inter_geom: chain
                .inter_geom
                .iter()
                .map(|iv| iv.iter().map(|smp| scale_inter_sample(smp, s)).collect())
                .collect(),
        }
    }
}

fn scan_reachable_b(
    limits: &Limits,
    geom: &crate::topp::chain::PointGeom,
    peak_mvc_b: &mut f64,
    a_tan_max: &mut f64,
) {
    use crate::topp::constraints::{COMP_FLOOR, KAPPA_FLOOR};
    let b = limits
        .mvc_b(&geom.c_prime, COMP_FLOOR)
        .min(limits.b_cent_cap(&geom.c_prime, &geom.c_double_prime, KAPPA_FLOOR));
    if b.is_finite() {
        *peak_mvc_b = peak_mvc_b.max(b);
    }
    let a_tan = limits.a_tan_cap(&geom.c_prime, COMP_FLOOR);
    if a_tan.is_finite() {
        *a_tan_max = a_tan_max.max(a_tan);
    }
}

fn scale_inter_sample(
    sample: &crate::topp::path::InterSample,
    s: f64,
) -> crate::topp::path::InterSample {
    crate::topp::path::InterSample {
        theta: sample.theta,
        c_prime: sample.c_prime,
        c_double_prime: sample.c_double_prime.map(|v| v * s),
    }
}

#[cfg(test)]
mod tests;
