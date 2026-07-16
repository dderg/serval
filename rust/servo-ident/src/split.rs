//! Pair load-share differential fit: two motors on one belt share reaction
//! forces by span stiffness, position-dependently. Elastic span sharing is
//! force-agnostic — the spans transmit whatever force the carriage needs, so
//! pure geometry predicts ONE shared `w(p) = w0 + w1·p_belt` for all force
//! components. For each frame pair we regress the measured torque
//! differential `D = s_i·τ_i − s_j·τ_j` on the fitted mode model's TOTAL
//! belt force scaled by `{1, p_belt}`; those two rank-1 coefficients are what
//! the profile carries and the endpoint applies antisymmetrically. Even
//! (`|F|`) terms and per-capture intercepts are fit as nuisances and
//! reported, never fed forward: a large even contribution is a
//! role-dependent split (tension/pulley-drag asymmetry), not the symmetric
//! span-stiffness effect the profile carries.
//!
//! Per-component structure beyond the shared `w(p)` has no mechanism and is
//! suspect (V/C column collinearity, role leakage, residual strain), so it is
//! never fed forward — but each pair still gets the free six-coefficient
//! per-component fit as a diagnostic and, with ≥2 capture windows, a
//! leave-one-window-out comparison of held-out prediction: if the free fit
//! predicts unseen windows clearly better than shared `w(p)`, the rank-1
//! constraint is discarding real structure and the report says so.

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

/// Leave-one-window-out comparison of held-out differential prediction.
/// Every rms pools the same folds' residuals (each held-out window's
/// intercept is refit as its residual mean, since the training fit cannot
/// know it), so the three numbers are directly comparable.
#[derive(Debug)]
pub struct SplitCrossval {
    pub folds: usize,
    /// Held-out rms with no split model at all (centered D).
    pub rms_none: f64,
    /// Held-out rms of the free six-coefficient per-component fit.
    pub rms_free: f64,
    /// Held-out rms of the rank-1 shared `w(p)·F_total` fit.
    pub rms_rank1: f64,
}

#[derive(Debug)]
pub struct PairReport {
    /// The rank-1 shared `w(p)` fit — the only split the profile carries.
    pub split: PairSplit,
    pub lambda: f64,
    pub w_stderr: [f64; 2],
    pub w_tvalue: [f64; 2],
    /// Diagnostic `|F_belt^I|`, `|F_belt^V|` coefficients — not written.
    pub even_coeff: [f64; 2],
    pub even_contribution: [f64; 2],
    pub max_odd_contribution: f64,
    pub intercepts: Vec<f64>,
    pub rms_before: f64,
    /// In-sample rms(D) after the rank-1 odd model + intercepts (even
    /// nuisances excluded).
    pub rms_after: f64,
    pub role_dependent: bool,
    pub samples: usize,
    /// Peak |w0 + w1·p| over the captured position range — computed BEFORE
    /// the cap zeroed anything.
    pub peak_fraction: f64,
    /// `split` zeroed because `peak_fraction` exceeded `SPLIT_MAX_FRACTION`
    /// (identifiability failure of the stroke plan).
    pub rejected: bool,
    /// Diagnostic free per-component fit `[I0, I1, V0, V1, C0, C1]` — never
    /// fed forward; per-component structure has no physical mechanism.
    pub free_w: [f64; 6],
    pub free_tvalue: [f64; 6],
    /// In-sample rms(D) after the free odd model + intercepts.
    pub free_rms_after: f64,
    /// Present with ≥2 capture windows (and ≥2 usable folds).
    pub crossval: Option<SplitCrossval>,
}

const N_FORCE_COLS: usize = 8;
const DEADBAND_MM_S: f64 = crate::model::COULOMB_DEADBAND_MM_S;
/// A pair's split fraction |w0 + w1·p| cannot physically approach 1 — that
/// would mean one motor carries more than the whole belt force. A fitted
/// component exceeding this cap anywhere in the captured position range is
/// an identifiability failure (degenerate stroke plan), not a measurement.
pub const SPLIT_MAX_FRACTION: f64 = 0.6;

/// Print the per-pair fit report to stderr and return the splits worth
/// writing to the profile (a rejected pair is omitted).
pub fn report_splits(reports: &[PairReport], axes: &[&str]) -> Vec<PairSplit> {
    const LABELS: [&str; 6] = ["I0", "I1", "V0", "V1", "C0", "C1"];
    let mut out = Vec::with_capacity(reports.len());
    for r in reports {
        let a = axes[r.split.first];
        let b = axes[r.split.second];
        eprintln!(
            "pair {a}/{b} (λ={:+.0}): rms(D) {:.2} -> {:.2} (shared w(p) model), {} samples",
            r.lambda, r.rms_before, r.rms_after, r.samples
        );
        for i in 0..2 {
            eprintln!(
                "  w{i} = {:+.6e}  (stderr {:.2e}, t = {:+.2})",
                r.split.w[i], r.w_stderr[i], r.w_tvalue[i]
            );
        }
        eprintln!(
            "  diag free per-component fit: in-sample rms(D) {:.2} — not fed forward",
            r.free_rms_after
        );
        for i in 0..6 {
            eprintln!(
                "    w_{} = {:+.6e}  (t = {:+.2})",
                LABELS[i], r.free_w[i], r.free_tvalue[i]
            );
        }
        match &r.crossval {
            Some(cv) => {
                eprintln!(
                    "  held-out rms(D) over {} window folds: no split {:.2}, shared \
                     w(p) {:.2}, per-component {:.2}",
                    cv.folds, cv.rms_none, cv.rms_rank1, cv.rms_free
                );
                if cv.rms_free < 0.95 * cv.rms_rank1 {
                    eprintln!(
                        "  WARNING pair {a}/{b}: the free per-component fit predicts \
                         held-out windows >=5% better than shared w(p) — the rank-1 \
                         constraint may be discarding real structure"
                    );
                }
            }
            None => eprintln!("  held-out rank-1 comparison skipped: needs >=2 capture windows"),
        }
        eprintln!(
            "  diag |F_I| coeff {:+.4e} (contrib {:.3}), |F_V| coeff {:+.4e} (contrib {:.3}); \
             largest odd contrib {:.3}",
            r.even_coeff[0],
            r.even_contribution[0],
            r.even_coeff[1],
            r.even_contribution[1],
            r.max_odd_contribution
        );
        for (c, off) in r.intercepts.iter().enumerate() {
            eprintln!("  diag intercept[capture {c}] {off:+.3}");
        }
        if r.role_dependent {
            eprintln!(
                "  WARNING pair {a}/{b}: role-dependent split detected — check belt \
                 tension/pulley drag; not fed forward"
            );
        }
        if r.rejected {
            eprintln!(
                "  WARNING pair {a}/{b}: split rejected — |w(p)| reaches \
                 {:.2} over the captured range (cap {SPLIT_MAX_FRACTION}); this \
                 stroke plan cannot identify it (position windows too narrow or \
                 accel confounded with position); pair omitted from the profile. \
                 Capture several SERVO_MEASURE_INERTIA windows at different \
                 positions to identify it.",
                r.peak_fraction,
            );
        } else {
            out.push(r.split.clone());
        }
    }
    out
}

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

/// One capture's kept rows of the pair regression: the eight force columns
/// and the differential, mask already applied.
struct CapRows {
    cols: [Vec<f64>; N_FORCE_COLS],
    d: Vec<f64>,
}

const FREE_ODD: usize = 6;
const RANK1_ODD: usize = 2;

fn n_odd(rank1: bool) -> usize {
    if rank1 {
        RANK1_ODD
    } else {
        FREE_ODD
    }
}

/// Odd regressor `i` of the chosen basis at one row: the free basis is the
/// six per-component columns; the rank-1 basis sums the components into
/// `F_total` (i = 0) and `F_total·p` (i = 1).
fn odd_col(cr: &CapRows, rank1: bool, i: usize, row: usize) -> f64 {
    if rank1 {
        cr.cols[i][row] + cr.cols[i + 2][row] + cr.cols[i + 4][row]
    } else {
        cr.cols[i][row]
    }
}

/// Pooled design for the chosen odd basis over every capture except `skip`:
/// odd columns, the two even nuisances, then one intercept indicator per
/// training capture.
fn design(cap_rows: &[CapRows], skip: Option<usize>, rank1: bool) -> (Vec<Vec<f64>>, Vec<f64>) {
    let odd = n_odd(rank1);
    let training: Vec<usize> = (0..cap_rows.len()).filter(|&c| Some(c) != skip).collect();
    let mut cols: Vec<Vec<f64>> = vec![Vec::new(); odd + 2 + training.len()];
    let mut y: Vec<f64> = Vec::new();
    for (ti, &ci) in training.iter().enumerate() {
        let cr = &cap_rows[ci];
        for row in 0..cr.d.len() {
            for i in 0..odd {
                cols[i].push(odd_col(cr, rank1, i, row));
            }
            cols[odd].push(cr.cols[6][row]);
            cols[odd + 1].push(cr.cols[7][row]);
            for tj in 0..training.len() {
                cols[odd + 2 + tj].push(if tj == ti { 1.0 } else { 0.0 });
            }
            y.push(cr.d[row]);
        }
    }
    (cols, y)
}

/// Centered residual sum of squares of one held-out capture under a fitted
/// odd + even model (the held-out intercept is unknowable from training, so
/// centering refits it).
fn heldout_rss(cr: &CapRows, rank1: bool, theta: &[f64]) -> (f64, usize) {
    let odd = n_odd(rank1);
    let resid: Vec<f64> = (0..cr.d.len())
        .map(|row| {
            let mut pred = 0.0;
            for i in 0..odd {
                pred += theta[i] * odd_col(cr, rank1, i, row);
            }
            pred += theta[odd] * cr.cols[6][row];
            pred += theta[odd + 1] * cr.cols[7][row];
            cr.d[row] - pred
        })
        .collect();
    (centered_rss(&resid), resid.len())
}

fn centered_rss(v: &[f64]) -> f64 {
    if v.is_empty() {
        return 0.0;
    }
    let mean = v.iter().sum::<f64>() / v.len() as f64;
    v.iter().map(|x| (x - mean) * (x - mean)).sum()
}

fn crossval(cap_rows: &[CapRows]) -> Option<SplitCrossval> {
    if cap_rows.len() < 2 {
        return None;
    }
    let mut folds = 0;
    let mut rows = 0usize;
    let (mut rss_none, mut rss_free, mut rss_rank1) = (0.0, 0.0, 0.0);
    for h in 0..cap_rows.len() {
        if cap_rows[h].d.is_empty() {
            continue;
        }
        let (cols_f, y_f) = design(cap_rows, Some(h), false);
        let (cols_r, y_r) = design(cap_rows, Some(h), true);
        let (Some(fit_f), Some(fit_r)) = (ols(&cols_f, &y_f), ols(&cols_r, &y_r)) else {
            continue;
        };
        let (fold_free, n_h) = heldout_rss(&cap_rows[h], false, &fit_f.theta);
        let (fold_rank1, _) = heldout_rss(&cap_rows[h], true, &fit_r.theta);
        rss_none += centered_rss(&cap_rows[h].d);
        rss_free += fold_free;
        rss_rank1 += fold_rank1;
        rows += n_h;
        folds += 1;
    }
    if folds < 2 || rows == 0 {
        return None;
    }
    let pooled = |rss: f64| (rss / rows as f64).sqrt();
    Some(SplitCrossval {
        folds,
        rms_none: pooled(rss_none),
        rms_free: pooled(rss_free),
        rms_rank1: pooled(rss_rank1),
    })
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

        let cap_rows: Vec<CapRows> = captures
            .iter()
            .map(|sc| {
                let (fcols, d) = pair_columns(
                    structure, params, signs, cutoff_hz, sc, first, second, lambda,
                );
                let mut cr = CapRows {
                    cols: std::array::from_fn(|_| Vec::new()),
                    d: Vec::new(),
                };
                for k in 0..sc.keep.len() {
                    if !sc.keep[k] {
                        continue;
                    }
                    for c in 0..N_FORCE_COLS {
                        cr.cols[c].push(fcols[c][k]);
                    }
                    cr.d.push(d[k]);
                }
                cr
            })
            .collect();
        let (cols_free, y_free) = design(&cap_rows, None, false);
        let total = y_free.len();
        let free_fit = ols(&cols_free, &y_free).unwrap_or_else(|| {
            panic!(
                "pair {first},{second}: differential regression is rank-deficient \
                 ({total} kept rows) — the pair was never excited independently"
            )
        });
        let free_w: [f64; 6] = std::array::from_fn(|i| free_fit.theta[i]);
        let free_tvalue: [f64; 6] = std::array::from_fn(|i| free_fit.theta[i] / free_fit.stderr[i]);
        let mut free_rss = 0.0;
        for row in 0..total {
            let mut pred = 0.0;
            for i in 0..FREE_ODD {
                pred += free_w[i] * cols_free[i][row];
            }
            for ic in 0..n_cap {
                pred += free_fit.theta[FREE_ODD + 2 + ic] * cols_free[FREE_ODD + 2 + ic][row];
            }
            let e = y_free[row] - pred;
            free_rss += e * e;
        }
        let free_rms_after = (free_rss / total.max(1) as f64).sqrt();

        let (cols, y) = design(&cap_rows, None, true);
        let fit = ols(&cols, &y).unwrap_or_else(|| {
            panic!(
                "pair {first},{second}: rank-1 shared-w(p) regression is \
                 rank-deficient even though the free fit succeeded — the \
                 summed force columns collapsed"
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

        let mut w = [fit.theta[0], fit.theta[1]];
        let peak_fraction = (w[0] + w[1] * p_min).abs().max((w[0] + w[1] * p_max).abs());
        let rejected = peak_fraction > SPLIT_MAX_FRACTION;
        if rejected {
            w = [0.0, 0.0];
        }
        let w_stderr = [fit.stderr[0], fit.stderr[1]];
        let w_tvalue = [fit.theta[0] / fit.stderr[0], fit.theta[1] / fit.stderr[1]];
        let even_coeff = [fit.theta[RANK1_ODD], fit.theta[RANK1_ODD + 1]];
        let intercepts: Vec<f64> = (0..n_cap).map(|c| fit.theta[RANK1_ODD + 2 + c]).collect();

        let col_rms = |i: usize| rms(&cols[i]);
        let max_odd_contribution = (0..RANK1_ODD)
            .map(|i| w[i].abs() * col_rms(i))
            .fold(0.0_f64, f64::max);
        let even_contribution = [
            even_coeff[0].abs() * col_rms(RANK1_ODD),
            even_coeff[1].abs() * col_rms(RANK1_ODD + 1),
        ];
        let role_dependent = even_contribution
            .iter()
            .any(|&c| c > 0.5 * max_odd_contribution);

        let rms_before = rms(&y);
        let mut rss_after = 0.0;
        for row in 0..total {
            let mut pred = w[0] * cols[0][row] + w[1] * cols[1][row];
            for (ic, intercept) in intercepts.iter().enumerate() {
                pred += intercept * cols[RANK1_ODD + 2 + ic][row];
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
            free_w,
            free_tvalue,
            free_rms_after,
            crossval: crossval(&cap_rows),
        });
    }
    reports
}
