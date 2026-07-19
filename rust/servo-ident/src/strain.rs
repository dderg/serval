//! `strain_map` run analysis: per raster line, bin the differential belt
//! torque along the sweep coordinate and split it into an elastic profile
//! (direction-symmetric part) and a friction profile (direction-antisymmetric
//! part). Wire shape and math mirror the reference
//! `analyze_strain.py` script bound to `GET /api/runs/<name>/strain`.

use std::path::Path;

use schemars::JsonSchema;
use serde::Serialize;
use serde_json::Value;
use ts_rs::TS;

use crate::results::Manifest;
use crate::scap::Scap;

pub const BIN_MM: f64 = 2.0;
const MOVING_MM_PER_CYCLE: f64 = 1e-4;

#[derive(Debug, Serialize, JsonSchema, TS)]
pub struct StrainBelt {
    pub pair: String,
    pub elastic: Vec<Option<f64>>,
    pub friction: Vec<Option<f64>>,
}

#[derive(Debug, Serialize, JsonSchema, TS)]
pub struct StrainLine {
    pub name: String,
    pub swept: Value,
    pub bin_centers: Vec<f64>,
    pub belts: Vec<StrainBelt>,
}

#[derive(Debug, Serialize, JsonSchema, TS)]
pub struct StrainMap {
    pub lines: Vec<StrainLine>,
}

pub fn belt_pairs(belts: &str) -> Result<Vec<[String; 2]>, String> {
    let mut pairs = Vec::new();
    for belt in belts.split(',') {
        let motors: Vec<String> = belt
            .split('+')
            .map(|m| m.split(':').next().unwrap_or("").to_string())
            .collect();
        let [m0, m1] = motors.as_slice() else {
            return Err(format!(
                "belt {belt:?} names {} motors, need exactly 2",
                motors.len()
            ));
        };
        pairs.push([m0.clone(), m1.clone()]);
    }
    if pairs.len() != 2 {
        return Err(format!(
            "belts {belts:?} describe {} belts, need exactly 2",
            pairs.len()
        ));
    }
    Ok(pairs)
}

fn mech_sign(invert: bool) -> f64 {
    if invert {
        -1.0
    } else {
        1.0
    }
}

fn ptp(v: &[f64]) -> f64 {
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for &x in v {
        lo = lo.min(x);
        hi = hi.max(x);
    }
    hi - lo
}

fn gradient(v: &[f64]) -> Result<Vec<f64>, String> {
    let n = v.len();
    if n < 2 {
        return Err(format!("capture has {n} record(s), need at least 2"));
    }
    let mut out = Vec::with_capacity(n);
    out.push(v[1] - v[0]);
    for i in 1..n - 1 {
        out.push((v[i + 1] - v[i - 1]) / 2.0);
    }
    out.push(v[n - 1] - v[n - 2]);
    Ok(out)
}

fn round_to(v: f64, decimals: i32) -> f64 {
    let scale = 10f64.powi(decimals);
    (v * scale).round() / scale
}

struct BinnedMeans {
    forward: Vec<Option<f64>>,
    backward: Vec<Option<f64>>,
}

fn bin_directional_means(
    sweep: &[f64],
    vel: &[f64],
    values: &[f64],
    span: f64,
    nbins: usize,
) -> BinnedMeans {
    let mut sums = vec![[0.0f64; 2]; nbins];
    let mut counts = vec![[0usize; 2]; nbins];
    for i in 0..sweep.len() {
        if vel[i].abs() <= MOVING_MM_PER_CYCLE {
            continue;
        }
        let dir = if vel[i] > 0.0 { 0 } else { 1 };
        let bin = ((sweep[i] / span * nbins as f64) as usize).min(nbins - 1);
        sums[bin][dir] += values[i];
        counts[bin][dir] += 1;
    }
    let mean = |dir: usize| {
        (0..nbins)
            .map(|b| (counts[b][dir] > 0).then(|| sums[b][dir] / counts[b][dir] as f64))
            .collect()
    };
    BinnedMeans {
        forward: mean(0),
        backward: mean(1),
    }
}

struct LineProfiles {
    bin_centers: Vec<f64>,
    belts: Vec<StrainBelt>,
}

fn analyze_line(scap: &Scap, pairs: &[[String; 2]], name: &str) -> Result<LineProfiles, String> {
    let mut mech_target = Vec::with_capacity(scap.header.drives.len());
    let mut mech_torque = Vec::with_capacity(scap.header.drives.len());
    for (di, drive) in scap.header.drives.iter().enumerate() {
        let sign = mech_sign(drive.invert);
        let counts = scap.read_f64(di, "target_counts")?;
        mech_target.push(
            counts
                .iter()
                .map(|c| sign * (c - counts[0]) / drive.counts_per_mm)
                .collect::<Vec<f64>>(),
        );
        let torque = scap.read_f64(di, "torque_actual")?;
        mech_torque.push(torque.iter().map(|t| sign * t / 10.0).collect::<Vec<f64>>());
    }
    let drive_idx = |motor: &str| {
        scap.drive_index(motor)
            .ok_or_else(|| format!("{name}: capture has no drive {motor:?}"))
    };
    let pa = &mech_target[drive_idx(&pairs[0][0])?];
    let pb = &mech_target[drive_idx(&pairs[1][0])?];
    let x: Vec<f64> = pa.iter().zip(pb).map(|(a, b)| (a + b) / 2.0).collect();
    let y: Vec<f64> = pa.iter().zip(pb).map(|(a, b)| (a - b) / 2.0).collect();
    let mut sweep = if ptp(&x) > ptp(&y) { x } else { y };
    let lo = sweep.iter().copied().fold(f64::INFINITY, f64::min);
    for s in &mut sweep {
        *s -= lo;
    }
    let span = ptp(&sweep);
    if span <= 0.0 {
        return Err(format!("{name}: no motion in capture (sweep span is 0)"));
    }
    let nbins = ((span / BIN_MM).round() as usize).max(2);
    let vel = gradient(&sweep).map_err(|e| format!("{name}: {e}"))?;
    let bin_centers = (0..nbins)
        .map(|i| round_to((i as f64 + 0.5) * span / nbins as f64, 2))
        .collect();

    let mut belts = Vec::with_capacity(pairs.len());
    for [m0, m1] in pairs {
        let t0 = &mech_torque[drive_idx(m0)?];
        let t1 = &mech_torque[drive_idx(m1)?];
        let diff: Vec<f64> = t0.iter().zip(t1).map(|(a, b)| (a - b) / 2.0).collect();
        let binned = bin_directional_means(&sweep, &vel, &diff, span, nbins);
        let combine = |scale: f64| {
            binned
                .forward
                .iter()
                .zip(&binned.backward)
                .map(|(f, b)| match (f, b) {
                    (Some(f), Some(b)) => Some(round_to((f + scale * b) / 2.0, 3)),
                    _ => None,
                })
                .collect()
        };
        belts.push(StrainBelt {
            pair: format!("{m0}/{m1}"),
            elastic: combine(1.0),
            friction: combine(-1.0),
        });
    }
    Ok(LineProfiles { bin_centers, belts })
}

pub fn is_strain_map(manifest: &Manifest) -> bool {
    manifest.experiment == "strain_map"
}

pub fn analyze_run(run_dir: &Path) -> Result<StrainMap, String> {
    let manifest_path = run_dir.join("manifest.json");
    let text = std::fs::read_to_string(&manifest_path)
        .map_err(|e| format!("read {}: {e}", manifest_path.display()))?;
    let manifest: Manifest = serde_json::from_str(&text)
        .map_err(|e| format!("{}: manifest parse: {e}", manifest_path.display()))?;
    if !is_strain_map(&manifest) {
        return Err(format!(
            "{}: experiment is {:?}, not \"strain_map\"",
            run_dir.display(),
            manifest.experiment
        ));
    }
    let belts = manifest.belts.as_deref().ok_or_else(|| {
        format!(
            "{}: strain_map manifest has no belts field",
            manifest_path.display()
        )
    })?;
    let pairs = belt_pairs(belts).map_err(|e| format!("{}: {e}", manifest_path.display()))?;
    let mut lines = Vec::with_capacity(manifest.steps.len());
    for step in &manifest.steps {
        let path = run_dir.join(format!("step_{}.scap", step.name));
        let scap = Scap::load(&path.display().to_string())?;
        let profiles = analyze_line(&scap, &pairs, &step.name)?;
        lines.push(StrainLine {
            name: step.name.clone(),
            swept: step.swept.clone(),
            bin_centers: profiles.bin_centers,
            belts: profiles.belts,
        });
    }
    Ok(StrainMap { lines })
}
