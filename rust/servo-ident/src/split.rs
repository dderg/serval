//! Pair load-share differential fit: two motors on one belt share reaction
//! forces by span stiffness, position-dependently. For each frame pair we
//! regress the measured torque differential `D = s_i·τ_i − s_j·τ_j` on the
//! fitted mode model's per-component belt forces scaled by `{1, p_belt}`,
//! yielding six odd coefficients per component the endpoint applies
//! antisymmetrically. Even (`|F|`) terms and per-capture intercepts are fit as
//! nuisances and reported, never fed forward: a large even contribution is a
//! role-dependent split (tension/pulley-drag asymmetry), not the symmetric
//! span-stiffness effect the profile carries.

use crate::capture::Capture;
use crate::linalg::solve_spd;
use crate::model::{coulomb_sign, PhysicalParams, Structure};
use crate::prep::{filter_segments, median_dt, segments, sinc_kernel};
use crate::profile_out::PairSplit;

/// One prepped capture's contribution to the split: its raw channels (for the
/// belt forces and belt coordinate), the delay-aligned + band-filtered torque
/// the mode fit used, and the exact keep mask the mode fit kept.
pub struct SplitCapture<'a> {
    pub cap: &'a Capture,
    pub torque_filt: &'a [Vec<f64>],
    pub keep: &'a [bool],
}

#[derive(Debug)]
pub struct PairReport {
    pub split: PairSplit,
    pub lambda: f64,
    pub w_stderr: [f64; 6],
    pub w_tvalue: [f64; 6],
    /// Diagnostic `|F_belt^I|`, `|F_belt^V|` coefficients — not written.
    pub even_coeff: [f64; 2],
    pub even_contribution: [f64; 2],
    pub max_odd_contribution: f64,
    pub intercepts: Vec<f64>,
    pub rms_before: f64,
    pub rms_after: f64,
    pub role_dependent: bool,
    pub samples: usize,
    /// Peak |w0 + w1·p| over the captured position range, per component
    /// [inertial, viscous, coulomb] — computed BEFORE the cap zeroed anything.
    pub peak_fraction: [f64; 3],
    /// Components zeroed in `split` because `peak_fraction` exceeded
    /// `SPLIT_MAX_FRACTION` (identifiability failure of the stroke plan).
    pub rejected: [bool; 3],
}

const N_FORCE_COLS: usize = 8;
const DEADBAND_MM_S: f64 = crate::model::COULOMB_DEADBAND_MM_S;
/// A pair's split fraction |w0 + w1·p| cannot physically approach 1 — that
/// would mean one motor carries more than the whole belt force. A fitted
/// component exceeding this cap anywhere in the captured position range is
/// an identifiability failure (degenerate stroke plan), not a measurement.
pub const SPLIT_MAX_FRACTION: f64 = 0.6;

struct Ols {
    theta: Vec<f64>,
    stderr: Vec<f64>,
}

fn ols(cols: &[Vec<f64>], y: &[f64]) -> Option<Ols> {
    let p = cols.len();
    let n = y.len();
    if n <= p {
        return None;
    }
    let scale: Vec<f64> = cols
        .iter()
        .map(|c| {
            let s2: f64 = c.iter().map(|x| x * x).sum();
            if s2 > 0.0 {
                s2.sqrt()
            } else {
                0.0
            }
        })
        .collect();
    if scale.iter().any(|&s| s == 0.0) {
        return None;
    }
    let mut ata = vec![0.0_f64; p * p];
    let mut aty = vec![0.0_f64; p];
    for k in 0..n {
        for i in 0..p {
            let ri = cols[i][k] / scale[i];
            aty[i] += ri * y[k];
            for j in 0..p {
                ata[i * p + j] += ri * cols[j][k] / scale[j];
            }
        }
    }
    let theta_s = solve_spd(&ata, &aty, p)?;
    let theta: Vec<f64> = (0..p).map(|i| theta_s[i] / scale[i]).collect();
    let mut rss = 0.0;
    for k in 0..n {
        let pred: f64 = (0..p).map(|i| theta[i] * cols[i][k]).sum();
        let e = y[k] - pred;
        rss += e * e;
    }
    let sigma2 = rss / (n - p) as f64;
    let stderr: Vec<f64> = (0..p)
        .map(|i| {
            let mut e = vec![0.0; p];
            e[i] = 1.0;
            match solve_spd(&ata, &e, p) {
                Some(c) if c[i] >= 0.0 => (sigma2 * c[i]).sqrt() / scale[i],
                _ => f64::NAN,
            }
        })
        .collect();
    Some(Ols { theta, stderr })
}

fn rms(v: &[f64]) -> f64 {
    if v.is_empty() {
        return 0.0;
    }
    (v.iter().map(|x| x * x).sum::<f64>() / v.len() as f64).sqrt()
}

/// The `N_FORCE_COLS` band-filtered force regressor columns and the filtered
/// differential `D`, all sample-aligned with the capture.
fn pair_columns(
    structure: &Structure,
    params: &PhysicalParams,
    signs: &[f64],
    cutoff_hz: f64,
    sc: &SplitCapture,
    first: usize,
    second: usize,
    lambda: f64,
) -> ([Vec<f64>; N_FORCE_COLS], Vec<f64>) {
    let cap = sc.cap;
    let n = cap.t.len();
    let n_modes = structure.mode_count();
    let n_slots = structure.axis_count();
    let s_i = signs[first];
    let s_j = signs[second];
    let belt_sign = s_i + lambda * s_j;

    let mut raw: [Vec<f64>; N_FORCE_COLS] = std::array::from_fn(|_| vec![0.0; n]);
    for k in 0..n {
        let mut g_i = [0.0_f64; 3];
        for md in 0..n_modes {
            let mut a = 0.0;
            let mut v = 0.0;
            for s in 0..n_slots {
                let f = structure.frame[md][s];
                a += f * cap.acc[s][k];
                v += f * cap.vel[s][k];
            }
            let f_inertial = params.mass[md] * a;
            let f_viscous = params.viscous[md] * v;
            let f_coulomb = params.coulomb[md] * coulomb_sign(v);
            let w = structure.frame[md][first];
            g_i[0] += w * f_inertial;
            g_i[1] += w * f_viscous;
            g_i[2] += w * f_coulomb;
        }
        let fb_i = belt_sign * g_i[0];
        let fb_v = belt_sign * g_i[1];
        let fb_c = belt_sign * g_i[2];
        let p = s_i * cap.pos[first][k];
        raw[0][k] = fb_i;
        raw[1][k] = fb_i * p;
        raw[2][k] = fb_v;
        raw[3][k] = fb_v * p;
        raw[4][k] = fb_c;
        raw[5][k] = fb_c * p;
        raw[6][k] = fb_i.abs();
        raw[7][k] = fb_v.abs();
    }

    let dt = median_dt(&cap.t);
    let segs = segments(&cap.t, dt);
    let kernel = if cutoff_hz > 0.0 {
        Some(sinc_kernel(cutoff_hz, dt))
    } else {
        None
    };
    let cols: [Vec<f64>; N_FORCE_COLS] =
        std::array::from_fn(|i| filter_segments(&raw[i], &segs, kernel.as_deref()));

    let d: Vec<f64> = (0..n)
        .map(|k| s_i * sc.torque_filt[first][k] - s_j * sc.torque_filt[second][k])
        .collect();
    (cols, d)
}

fn assert_shared_belt(captures: &[SplitCapture], first: usize, second: usize) {
    let (mut sx, mut sy, mut sxx, mut syy, mut sxy) = (0.0, 0.0, 0.0, 0.0, 0.0);
    let mut n = 0.0_f64;
    for sc in captures {
        let vi = &sc.cap.vel[first];
        let vj = &sc.cap.vel[second];
        for k in 0..vi.len() {
            if vi[k].abs() > DEADBAND_MM_S || vj[k].abs() > DEADBAND_MM_S {
                sx += vi[k];
                sy += vj[k];
                sxx += vi[k] * vi[k];
                syy += vj[k] * vj[k];
                sxy += vi[k] * vj[k];
                n += 1.0;
            }
        }
    }
    if n < 10.0 {
        return;
    }
    let cov = sxy - sx * sy / n;
    let varx = sxx - sx * sx / n;
    let vary = syy - sy * sy / n;
    if varx <= 0.0 || vary <= 0.0 {
        return;
    }
    let corr = cov / (varx * vary).sqrt();
    assert!(
        corr.abs() > 0.999,
        "slots {first},{second} are a frame pair but their commanded velocities \
         correlate at {corr:.4} (|corr| must exceed 0.999) — they do not share a \
         belt; the frame's parallel columns disagree with the capture"
    );
}

/// Fit every frame pair's load-share differential. Panics loudly on a
/// malformed pair (non-`±1` λ, mismatched signs length, or a capture missing
/// the commanded positions the split requires) so a bad setup is caught, not
/// silently skipped.
pub fn fit_pair_splits(
    structure: &Structure,
    params: &PhysicalParams,
    signs: &[f64],
    cutoff_hz: f64,
    captures: &[SplitCapture],
) -> Vec<PairReport> {
    let n_slots = structure.axis_count();
    assert_eq!(
        signs.len(),
        n_slots,
        "signs must carry one ±1 per slot ({n_slots})"
    );
    assert!(
        signs.iter().all(|&s| s == 1.0 || s == -1.0),
        "signs entries must be exactly ±1"
    );
    assert!(!captures.is_empty(), "split needs at least one capture");
    for sc in captures {
        assert!(
            sc.cap.has_positions(),
            "the pair split needs commanded positions, but a capture carries none"
        );
        assert_eq!(sc.keep.len(), sc.cap.t.len(), "keep mask length");
        assert_eq!(sc.torque_filt.len(), n_slots, "filtered torque slot count");
    }

    let n_cap = captures.len();
    let mut reports = Vec::new();
    for (first, second, lambda) in structure.pairs() {
        assert_shared_belt(captures, first, second);

        let mut cols: Vec<Vec<f64>> = vec![Vec::new(); N_FORCE_COLS + n_cap];
        let mut y: Vec<f64> = Vec::new();
        for (ci, sc) in captures.iter().enumerate() {
            let (fcols, d) = pair_columns(
                structure, params, signs, cutoff_hz, sc, first, second, lambda,
            );
            for k in 0..sc.keep.len() {
                if !sc.keep[k] {
                    continue;
                }
                for c in 0..N_FORCE_COLS {
                    cols[c].push(fcols[c][k]);
                }
                for ic in 0..n_cap {
                    cols[N_FORCE_COLS + ic].push(if ic == ci { 1.0 } else { 0.0 });
                }
                y.push(d[k]);
            }
        }
        let total = y.len();
        let fit = ols(&cols, &y).unwrap_or_else(|| {
            panic!(
                "pair {first},{second}: differential regression is rank-deficient \
                 ({total} kept rows) — the pair was never excited independently"
            )
        });

        let mut p_min = f64::INFINITY;
        let mut p_max = f64::NEG_INFINITY;
        for sc in captures {
            for k in 0..sc.keep.len() {
                if sc.keep[k] {
                    let p = signs[first] * sc.cap.pos[first][k];
                    p_min = p_min.min(p);
                    p_max = p_max.max(p);
                }
            }
        }

        let mut w: [f64; 6] = std::array::from_fn(|i| fit.theta[i]);
        let peak_fraction: [f64; 3] = std::array::from_fn(|c| {
            let (w0, w1) = (w[2 * c], w[2 * c + 1]);
            (w0 + w1 * p_min).abs().max((w0 + w1 * p_max).abs())
        });
        let rejected: [bool; 3] = std::array::from_fn(|c| peak_fraction[c] > SPLIT_MAX_FRACTION);
        for c in 0..3 {
            if rejected[c] {
                w[2 * c] = 0.0;
                w[2 * c + 1] = 0.0;
            }
        }
        let w_stderr: [f64; 6] = std::array::from_fn(|i| fit.stderr[i]);
        let w_tvalue: [f64; 6] = std::array::from_fn(|i| fit.theta[i] / fit.stderr[i]);
        let even_coeff = [fit.theta[6], fit.theta[7]];
        let intercepts: Vec<f64> = (0..n_cap).map(|c| fit.theta[N_FORCE_COLS + c]).collect();

        let col_rms = |i: usize| rms(&cols[i]);
        let max_odd_contribution = (0..6)
            .map(|i| w[i].abs() * col_rms(i))
            .fold(0.0_f64, f64::max);
        let even_contribution = [
            even_coeff[0].abs() * col_rms(6),
            even_coeff[1].abs() * col_rms(7),
        ];
        let role_dependent = even_contribution
            .iter()
            .any(|&c| c > 0.5 * max_odd_contribution);

        let rms_before = rms(&y);
        let mut rss_after = 0.0;
        for row in 0..total {
            let mut pred = 0.0;
            for i in 0..6 {
                pred += w[i] * cols[i][row];
            }
            for (ic, intercept) in intercepts.iter().enumerate() {
                pred += intercept * cols[N_FORCE_COLS + ic][row];
            }
            let e = y[row] - pred;
            rss_after += e * e;
        }
        let rms_after = (rss_after / total.max(1) as f64).sqrt();

        reports.push(PairReport {
            split: PairSplit { first, second, w },
            lambda,
            w_stderr,
            w_tvalue,
            even_coeff,
            even_contribution,
            max_odd_contribution,
            intercepts,
            rms_before,
            rms_after,
            role_dependent,
            samples: total,
            peak_fraction,
            rejected,
        });
    }
    reports
}
