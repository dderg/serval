use core::f64::consts::{FRAC_PI_2, PI};

/// Validated domain of the power-series evaluator; out-of-range arguments fail
/// loudly rather than returning a silently-wrong position.
pub(super) const FRESNEL_X_MAX: f64 = 3.0;

const SERIES_REL_TOL: f64 = 1e-17;
const SERIES_MAX_TERMS: usize = 100;

fn fresnel_cs(x: f64) -> (f64, f64) {
    assert!(
        x.abs() <= FRESNEL_X_MAX,
        "Fresnel argument {x} exceeds validated power-series domain {FRESNEL_X_MAX}"
    );
    let ax = x.abs();
    let x4 = ax * ax * ax * ax;
    let half_pi_sq = FRAC_PI_2 * FRAC_PI_2;

    let mut c_term = ax;
    let mut s_term = FRAC_PI_2 * ax * ax * ax / 3.0;
    let mut c = c_term;
    let mut s = s_term;
    for n in 0..SERIES_MAX_TERMS {
        let nf = n as f64;
        c_term *= -half_pi_sq * x4 / ((2.0 * nf + 1.0) * (2.0 * nf + 2.0))
            * ((4.0 * nf + 1.0) / (4.0 * nf + 5.0));
        s_term *= -half_pi_sq * x4 / ((2.0 * nf + 2.0) * (2.0 * nf + 3.0))
            * ((4.0 * nf + 3.0) / (4.0 * nf + 7.0));
        c += c_term;
        s += s_term;
        if c_term.abs() <= SERIES_REL_TOL * (1.0 + c.abs())
            && s_term.abs() <= SERIES_REL_TOL * (1.0 + s.abs())
        {
            let sign = x.signum();
            return (sign * c, sign * s);
        }
    }
    panic!("Fresnel power series did not converge for x={x}");
}

pub(super) fn clothoid_offset(kappa_0: f64, sigma: f64, s: f64) -> (f64, f64) {
    if sigma == 0.0 {
        if kappa_0 == 0.0 {
            return (s, 0.0);
        }
        return (
            (kappa_0 * s).sin() / kappa_0,
            (1.0 - (kappa_0 * s).cos()) / kappa_0,
        );
    }

    let abs_sigma = sigma.abs();
    let sign = sigma.signum();
    let a = kappa_0 * kappa_0 / (2.0 * sigma);
    let scale = (abs_sigma / PI).sqrt();
    let w0 = kappa_0 / sigma;
    let w1 = s + kappa_0 / sigma;

    let (c0, s0) = fresnel_cs(w0 * scale);
    let (c1, s1) = fresnel_cs(w1 * scale);
    let d_c = c1 - c0;
    let d_s = s1 - s0;

    let k = (PI / abs_sigma).sqrt();
    let (cos_a, sin_a) = (a.cos(), a.sin());
    let cx = k * (cos_a * d_c + sign * sin_a * d_s);
    let cy = k * (sign * cos_a * d_s - sin_a * d_c);
    (cx, cy)
}
