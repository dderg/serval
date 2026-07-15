use crate::linalg::{solve_spd, sym_eig_extremes};
use crate::model::{PhysicalParams, Structure};

#[derive(Clone)]
pub struct FitInput {
    pub structure: Structure,
    /// Mode-space commanded acceleration, equal lengths: acc_mode[mode][sample]
    /// (mm/s²), already `frame`-projected and band-filtered.
    pub acc_mode: Vec<Vec<f64>>,
    pub vel_mode: Vec<Vec<f64>>,
    /// Per-mode coulomb sign regressor column, filtered identically to
    /// acc_mode/vel_mode/torque.
    pub cs_mode: Vec<Vec<f64>>,
    /// Measured torque per motor/slot (0.1% rated units).
    pub torque: Vec<Vec<f64>>,
    /// Extra per-motor regressor columns (extra[motor][column][sample]) —
    /// nuisance channels like the pulley-eccentricity sin/cos pair. Each
    /// motor must carry the same column count. Their coefficients absorb
    /// structured in-band disturbance that would otherwise bias the physical
    /// parameters; they are reported, not written to the profile.
    pub extra: Vec<Vec<Vec<f64>>>,
}

pub struct FitOptions {
    /// Refusal threshold on `FitResult::condition` (column-scaled Gram).
    pub max_condition: f64,
    pub saturation_abs: f64,
    pub max_saturated_fraction: f64,
    pub max_rms_residual: f64,
    /// Huber IRLS reweighting passes after the initial least squares. 0
    /// reproduces plain least squares.
    pub huber_iterations: usize,
    /// Huber threshold as a multiple of the robust residual scale.
    pub huber_k: f64,
}

impl Default for FitOptions {
    fn default() -> Self {
        Self {
            max_condition: 1.0e8,
            saturation_abs: 3900.0,
            max_saturated_fraction: 0.001,
            max_rms_residual: 100.0,
            huber_iterations: 3,
            huber_k: 1.345,
        }
    }
}

#[derive(Debug)]
pub enum FitError {
    ShapeMismatch(&'static str),
    SaturatedTorque { fraction: f64 },
    InsufficientExcitation { condition: f64 },
    UnexcitedMode { mode: usize },
    ResidualTooLarge { rms: f64 },
}

#[derive(Debug)]
pub struct FitResult {
    pub params: PhysicalParams,
    /// Fitted coefficients of the extra nuisance columns, per motor.
    pub extra_params: Vec<Vec<f64>>,
    /// In-sample RMS (0.1% rated units); optimism bias is negligible for
    /// sample counts far above the parameter count.
    pub rms_residual: f64,
    /// Standard error per raw theta parameter (same packing as
    /// `Structure::unpack`), from the weighted Gram matrix and residual
    /// variance. Two successive fits agree within noise when their
    /// parameters differ by less than ~2 combined standard errors.
    pub param_stderr: Vec<f64>,
    /// λmax/λmin of the column-scaled Gram matrix — an excitation-quality
    /// score, not cond(AᵀA).
    pub condition: f64,
    /// Time samples per motor; regression rows = samples × motor count.
    pub samples: usize,
}

struct Accum {
    ata: Vec<f64>,
    aty: Vec<f64>,
    col_norm2: Vec<f64>,
}

fn full_row(input: &FitInput, motor: usize, k: usize, p: usize) -> Vec<f64> {
    let s = &input.structure;
    let n_modes = s.mode_count();
    let acc_k: Vec<f64> = (0..n_modes).map(|m| input.acc_mode[m][k]).collect();
    let vel_k: Vec<f64> = (0..n_modes).map(|m| input.vel_mode[m][k]).collect();
    let cs_k: Vec<f64> = (0..n_modes).map(|m| input.cs_mode[m][k]).collect();
    let mut row = s.row(motor, &acc_k, &vel_k, &cs_k);
    let e = input.extra.first().map_or(0, Vec::len);
    row.resize(p, 0.0);
    let base = s.param_count();
    for (j, col) in input.extra.get(motor).into_iter().flatten().enumerate() {
        row[base + motor * e + j] = col[k];
    }
    row
}

fn accumulate(input: &FitInput, p: usize, weights: Option<&[f64]>) -> Accum {
    let s = &input.structure;
    let n_motors = s.axis_count();
    let n_samples = input.acc_mode[0].len();
    let mut ata = vec![0.0_f64; p * p];
    let mut aty = vec![0.0_f64; p];
    let mut col_norm2 = vec![0.0_f64; p];
    for k in 0..n_samples {
        for motor in 0..n_motors {
            let w = weights.map_or(1.0, |ws| ws[k * n_motors + motor]);
            let row = full_row(input, motor, k, p);
            let y = input.torque[motor][k];
            for i in 0..p {
                aty[i] += w * row[i] * y;
                col_norm2[i] += w * row[i] * row[i];
                for j in 0..p {
                    ata[i * p + j] += w * row[i] * row[j];
                }
            }
        }
    }
    Accum {
        ata,
        aty,
        col_norm2,
    }
}

fn solve_scaled(acc: &Accum, p: usize) -> Result<(Vec<f64>, f64, Vec<f64>), FitError> {
    let scale: Vec<f64> = acc
        .col_norm2
        .iter()
        .map(|&c| if c > 0.0 { c.sqrt() } else { 0.0 })
        .collect();
    if scale.iter().any(|&sc| sc == 0.0) {
        return Err(FitError::InsufficientExcitation {
            condition: f64::INFINITY,
        });
    }
    let mut ata_s = vec![0.0_f64; p * p];
    for i in 0..p {
        for j in 0..p {
            ata_s[i * p + j] = acc.ata[i * p + j] / (scale[i] * scale[j]);
        }
    }
    let (lo, hi) = sym_eig_extremes(&ata_s, p);
    let condition = if lo > 0.0 { hi / lo } else { f64::INFINITY };
    let aty_s: Vec<f64> = (0..p).map(|i| acc.aty[i] / scale[i]).collect();
    let theta_s =
        solve_spd(&ata_s, &aty_s, p).ok_or(FitError::InsufficientExcitation { condition })?;
    let theta: Vec<f64> = (0..p).map(|i| theta_s[i] / scale[i]).collect();
    Ok((theta, condition, scale))
}

/// Per-motor residual series for the given parameters (including the
/// nuisance-column coefficients), aligned with the input samples. Used for
/// the band-limited residual report.
pub fn residual_by_motor(
    input: &FitInput,
    params: &PhysicalParams,
    extra_params: &[Vec<f64>],
) -> Vec<Vec<f64>> {
    let s = &input.structure;
    let n_motors = s.axis_count();
    let e = input.extra.first().map_or(0, Vec::len);
    let p = s.param_count() + n_motors * e;
    let mut theta = s.pack(params);
    for coeffs in extra_params {
        assert_eq!(coeffs.len(), e, "extra coefficient count");
        theta.extend_from_slice(coeffs);
    }
    assert_eq!(theta.len(), p);
    let n_samples = input.acc_mode[0].len();
    let mut out = vec![Vec::with_capacity(n_samples); n_motors];
    for k in 0..n_samples {
        for (motor, series) in out.iter_mut().enumerate() {
            let row = full_row(input, motor, k, p);
            let pred: f64 = row.iter().zip(&theta).map(|(r, t)| r * t).sum();
            series.push(input.torque[motor][k] - pred);
        }
    }
    out
}

fn residuals(input: &FitInput, theta: &[f64], p: usize) -> Vec<f64> {
    let s = &input.structure;
    let n_motors = s.axis_count();
    let n_samples = input.acc_mode[0].len();
    let mut out = Vec::with_capacity(n_samples * n_motors);
    for k in 0..n_samples {
        for motor in 0..n_motors {
            let row = full_row(input, motor, k, p);
            let pred: f64 = row.iter().zip(theta).map(|(r, t)| r * t).sum();
            out.push(input.torque[motor][k] - pred);
        }
    }
    out
}

fn robust_sigma(res: &[f64]) -> f64 {
    let mut abs: Vec<f64> = res.iter().map(|e| e.abs()).collect();
    abs.sort_by(|a, b| a.partial_cmp(b).expect("non-finite residual"));
    let mad = abs[abs.len() / 2];
    1.4826 * mad
}

fn stderr_from(acc: &Accum, scale: &[f64], sigma2: f64, p: usize) -> Vec<f64> {
    let mut ata_s = vec![0.0_f64; p * p];
    for i in 0..p {
        for j in 0..p {
            ata_s[i * p + j] = acc.ata[i * p + j] / (scale[i] * scale[j]);
        }
    }
    (0..p)
        .map(|i| {
            let mut e = vec![0.0; p];
            e[i] = 1.0;
            let col = solve_spd(&ata_s, &e, p);
            match col {
                Some(c) if c[i] >= 0.0 => (sigma2 * c[i]).sqrt() / scale[i],
                _ => f64::NAN,
            }
        })
        .collect()
}

pub fn fit(input: &FitInput, opts: &FitOptions) -> Result<FitResult, FitError> {
    let s = &input.structure;
    let n_motors = s.axis_count();
    let n_modes = s.mode_count();
    if input.acc_mode.len() != n_modes
        || input.vel_mode.len() != n_modes
        || input.cs_mode.len() != n_modes
    {
        return Err(FitError::ShapeMismatch("mode channel count"));
    }
    if input.torque.len() != n_motors {
        return Err(FitError::ShapeMismatch("motor count"));
    }
    let n_samples = input.acc_mode[0].len();
    if n_samples == 0 {
        return Err(FitError::ShapeMismatch("no samples"));
    }
    for m in 0..n_modes {
        if input.acc_mode[m].len() != n_samples
            || input.vel_mode[m].len() != n_samples
            || input.cs_mode[m].len() != n_samples
        {
            return Err(FitError::ShapeMismatch("mode sample count"));
        }
    }
    for m in 0..n_motors {
        if input.torque[m].len() != n_samples {
            return Err(FitError::ShapeMismatch("torque sample count"));
        }
    }

    let saturated = input
        .torque
        .iter()
        .flatten()
        .filter(|t| t.abs() >= opts.saturation_abs)
        .count();
    let fraction = saturated as f64 / (n_motors * n_samples) as f64;
    if fraction > opts.max_saturated_fraction {
        return Err(FitError::SaturatedTorque { fraction });
    }

    let e = input.extra.first().map_or(0, Vec::len);
    if !input.extra.is_empty() {
        assert_eq!(input.extra.len(), n_motors, "extra columns per motor");
        for cols in &input.extra {
            assert_eq!(cols.len(), e, "same extra column count per motor");
            for c in cols {
                assert_eq!(c.len(), n_samples, "extra column sample count");
            }
        }
    }
    let p = s.param_count() + n_motors * e;
    let mut acc = accumulate(input, p, None);
    for k in 0..s.param_count() {
        if acc.col_norm2[k] == 0.0 {
            return Err(FitError::UnexcitedMode { mode: k / 3 });
        }
    }
    let (mut theta, condition, mut scale) = solve_scaled(&acc, p)?;
    if condition > opts.max_condition {
        return Err(FitError::InsufficientExcitation { condition });
    }

    let mut weights: Vec<f64> = Vec::new();
    for _ in 0..opts.huber_iterations {
        let res = residuals(input, &theta, p);
        let sigma = robust_sigma(&res);
        if sigma <= 0.0 {
            break;
        }
        let cut = opts.huber_k * sigma;
        weights = res
            .iter()
            .map(|e| if e.abs() <= cut { 1.0 } else { cut / e.abs() })
            .collect();
        acc = accumulate(input, p, Some(&weights));
        let (t, _c, sc) = solve_scaled(&acc, p)?;
        theta = t;
        scale = sc;
    }

    let res = residuals(input, &theta, p);
    let (mut wsq, mut wsum) = (0.0_f64, 0.0_f64);
    for (i, e) in res.iter().enumerate() {
        let w = if weights.is_empty() { 1.0 } else { weights[i] };
        wsq += w * e * e;
        wsum += w;
    }
    let dof = (wsum - p as f64).max(1.0);
    let sigma2 = wsq / dof;
    let rms = (res.iter().map(|e| e * e).sum::<f64>() / res.len() as f64).sqrt();
    if rms > opts.max_rms_residual {
        return Err(FitError::ResidualTooLarge { rms });
    }
    let param_stderr = stderr_from(&acc, &scale, sigma2, p);

    let base = s.param_count();
    let extra_params: Vec<Vec<f64>> = (0..n_motors)
        .map(|m| theta[base + m * e..base + (m + 1) * e].to_vec())
        .collect();
    Ok(FitResult {
        params: s.unpack(&theta[..base]),
        extra_params,
        rms_residual: rms,
        param_stderr,
        condition,
        samples: n_samples,
    })
}
