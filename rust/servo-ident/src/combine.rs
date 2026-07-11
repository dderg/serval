//! CoreXY belt combine, ported from `scripts/servo_capture.py`
//! (`_parse_combine_spec`, `_belt_mm`, `combine_corexy`,
//! `compute_corexy_combine`). Each belt is `name[:sign][+name[:sign]...]`;
//! the belt signal is the mean of `sign * following_error / counts_per_mm`
//! across its motors. On-axis is `(A+B)/2`, cross-axis `(A-B)/2`.

use crate::scap::{Scap, FLAG_MOTION_ACTIVE};

pub struct Combine {
    pub axis: String,
    pub on_ferr: Vec<f64>,
    pub cross_ferr: Vec<f64>,
    pub moving: Vec<bool>,
}

type Belt = Vec<(String, i64)>;

fn parse_spec(spec: &str) -> Result<Vec<Belt>, String> {
    let mut belts = Vec::new();
    for belt_tok in spec.split(',') {
        let belt_tok = belt_tok.trim();
        if belt_tok.is_empty() {
            continue;
        }
        let mut terms: Belt = Vec::new();
        for tok in belt_tok.split('+') {
            let tok = tok.trim();
            let (name, sign) = match tok.split_once(':') {
                Some((n, s)) => {
                    let s = s
                        .trim()
                        .parse::<i64>()
                        .map_err(|_| format!("bad belt sign in {tok:?}"))?;
                    (n.trim().to_string(), s)
                }
                None => (tok.to_string(), 1),
            };
            terms.push((name, sign));
        }
        belts.push(terms);
    }
    Ok(belts)
}

fn belt_mm(cap: &Scap, terms: &Belt) -> Result<Vec<f64>, String> {
    let mut motors = Vec::new();
    for (name, sign) in terms {
        let idx = cap
            .drive_index(name)
            .ok_or_else(|| format!("combine motor {name:?} not in capture"))?;
        let cpm = cap.header.drives[idx].counts_per_mm;
        let ferr = cap.read_i64(idx, "following_error")?;
        motors.push((*sign, cpm, ferr));
    }
    let n = motors[0].2.len();
    let count = motors.len() as f64;
    let mut out = vec![0.0f64; n];
    for (sign, cpm, ferr) in &motors {
        let s = *sign as f64;
        for k in 0..n {
            out[k] += s * ferr[k] as f64 / cpm;
        }
    }
    for v in &mut out {
        *v /= count;
    }
    Ok(out)
}

pub fn compute_corexy_combine(
    cap: &Scap,
    spec: &str,
    axis: Option<&str>,
) -> Result<Combine, String> {
    let belts = parse_spec(spec)?;
    if belts.len() != 2 {
        return Err(format!("combine needs exactly two belts (got {spec:?})"));
    }
    let a = belt_mm(cap, &belts[0])?;
    let b = belt_mm(cap, &belts[1])?;
    let n = a.len();
    let x: Vec<f64> = (0..n).map(|k| 0.5 * (a[k] + b[k])).collect();
    let y: Vec<f64> = (0..n).map(|k| 0.5 * (a[k] - b[k])).collect();
    let axis = axis.unwrap_or("X").to_uppercase();
    let (on_ferr, cross_ferr) = if axis == "Y" { (y, x) } else { (x, y) };
    let first_idx = cap
        .drive_index(&belts[0][0].0)
        .ok_or_else(|| format!("combine motor {:?} not in capture", belts[0][0].0))?;
    let flags = cap.read_i64(first_idx, "flags")?;
    let moving = flags.iter().map(|&f| f & FLAG_MOTION_ACTIVE != 0).collect();
    Ok(Combine {
        axis,
        on_ferr,
        cross_ferr,
        moving,
    })
}

pub fn peak_abs(v: &[f64]) -> f64 {
    v.iter().fold(0.0_f64, |m, &x| m.max(x.abs()))
}

pub fn rms(v: &[f64]) -> f64 {
    if v.is_empty() {
        return 0.0;
    }
    (v.iter().map(|&x| x * x).sum::<f64>() / v.len() as f64).sqrt()
}
