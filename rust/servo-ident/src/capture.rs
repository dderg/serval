#[derive(Debug)]
pub struct Capture {
    pub t: Vec<f64>,
    pub acc: Vec<Vec<f64>>,
    pub vel: Vec<Vec<f64>>,
    pub torque: Vec<Vec<f64>>,
}

#[derive(Debug)]
pub enum CaptureError {
    MissingColumn(String),
    Malformed { line: usize, what: String },
    TooShort,
}

pub fn parse_capture_csv(text: &str, axes: &[&str]) -> Result<Capture, CaptureError> {
    let mut lines = text.lines().enumerate();
    let (_, header) = lines.next().ok_or(CaptureError::TooShort)?;
    let cols: Vec<&str> = header.split(',').map(str::trim).collect();

    let col = |name: &str| {
        cols.iter()
            .position(|c| *c == name)
            .ok_or_else(|| CaptureError::MissingColumn(name.to_string()))
    };

    let t_col = col("t")?;
    let accel_cols: Vec<usize> = axes
        .iter()
        .map(|a| col(&format!("accel_{a}")))
        .collect::<Result<_, _>>()?;
    let vel_cols: Vec<usize> = axes
        .iter()
        .map(|a| col(&format!("vel_{a}")))
        .collect::<Result<_, _>>()?;
    let torque_cols: Vec<usize> = axes
        .iter()
        .map(|a| col(&format!("torque_{a}")))
        .collect::<Result<_, _>>()?;

    let mut t: Vec<f64> = Vec::new();
    let mut acc: Vec<Vec<f64>> = vec![Vec::new(); axes.len()];
    let mut vel: Vec<Vec<f64>> = vec![Vec::new(); axes.len()];
    let mut torque: Vec<Vec<f64>> = vec![Vec::new(); axes.len()];

    for (lineno, line) in lines {
        if line.trim().is_empty() {
            continue;
        }
        let fields: Vec<&str> = line.split(',').map(str::trim).collect();
        let num = |idx: usize| -> Result<f64, CaptureError> {
            fields
                .get(idx)
                .and_then(|f| f.parse().ok())
                .ok_or_else(|| CaptureError::Malformed {
                    line: lineno + 1,
                    what: format!("column {idx}"),
                })
        };
        t.push(num(t_col)?);
        for (a, ((&ac, &vc), &qc)) in accel_cols
            .iter()
            .zip(&vel_cols)
            .zip(&torque_cols)
            .enumerate()
        {
            acc[a].push(num(ac)?);
            vel[a].push(num(vc)?);
            torque[a].push(num(qc)?);
        }
    }

    if t.len() < 5 {
        return Err(CaptureError::TooShort);
    }

    Ok(Capture {
        t,
        acc,
        vel,
        torque,
    })
}

#[derive(Debug, Clone)]
pub struct PlateauOptions {
    /// How long the commanded acceleration must hold steady before a cycle
    /// counts — at once a constant-accel check and a settle window for the
    /// closed loop to catch up to the command.
    pub settle_s: f64,
    /// Steadiness tolerance as a fraction of the capture's peak |accel|.
    pub tol_frac: f64,
    /// Absolute floor on the steadiness tolerance (mm/s²).
    pub tol_floor: f64,
}

impl Default for PlateauOptions {
    fn default() -> Self {
        Self {
            settle_s: 0.012,
            tol_frac: 0.03,
            tol_floor: 1.0,
        }
    }
}

fn median(values: &[f64]) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    let mut v = values.to_vec();
    v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    v[v.len() / 2]
}

/// Keep only cycles on steady constant-acceleration plateaus: every motor's
/// commanded acceleration has held within tolerance over a contiguous settle
/// window. There the actual motion has caught up to the command, so regressing
/// measured torque against the (exact) commanded acceleration is unbiased.
/// The jerk transitions — where the closed loop lags and the soft-loop
/// "negative inertia" artifact lives — are dropped.
pub fn restrict_to_steady_accel(cap: &Capture, opts: &PlateauOptions) -> Capture {
    let n = cap.t.len();
    let n_motors = cap.acc.len();
    let peak = cap
        .acc
        .iter()
        .flatten()
        .fold(0.0_f64, |m, &a| m.max(a.abs()));
    let tol = opts.tol_floor.max(opts.tol_frac * peak);
    let dts: Vec<f64> = (1..n).map(|k| cap.t[k] - cap.t[k - 1]).collect();
    let dt_med = median(&dts);
    let window = if dt_med > 0.0 {
        ((opts.settle_s / dt_med).round() as usize).max(1)
    } else {
        1
    };

    let mut keep: Vec<usize> = Vec::new();
    for k in window..n {
        let contiguous = (k - window..k).all(|j| cap.t[j + 1] - cap.t[j] <= 1.5 * dt_med);
        if !contiguous {
            continue;
        }
        let steady = (k - window..=k)
            .all(|j| (0..n_motors).all(|m| (cap.acc[m][j] - cap.acc[m][k]).abs() <= tol));
        if steady {
            keep.push(k);
        }
    }

    Capture {
        t: keep.iter().map(|&k| cap.t[k]).collect(),
        acc: cap
            .acc
            .iter()
            .map(|c| keep.iter().map(|&k| c[k]).collect())
            .collect(),
        vel: cap
            .vel
            .iter()
            .map(|c| keep.iter().map(|&k| c[k]).collect())
            .collect(),
        torque: cap
            .torque
            .iter()
            .map(|c| keep.iter().map(|&k| c[k]).collect())
            .collect(),
    }
}
