//! Golden parity gate: recompute the metrics/PSD/combine of the real bench
//! `.scap` fixtures with the Rust port and assert they match the Python
//! goldens frozen by `test/test_servo_capture_goldens.py`. Every golden field
//! is covered — the comparison walks the golden tree and requires matching
//! keys, so a missing field fails.

use std::io::Read;
use std::path::PathBuf;

use flate2::read::GzDecoder;
use serde_json::{json, Value};

use servo_ident::combine::{compute_corexy_combine, peak_abs, rms};
use servo_ident::metrics::{compute_metrics, drive_series, motion_segments};
use servo_ident::psd::{moving_psd, top_peaks};
use servo_ident::scap::Scap;

const SETTLE_BAND_COUNTS: i64 = 50;
const TORQUE_LIMIT_PER_MILLE: i64 = 1400;
const COMBINE_SPEC: &str = "motor_a:1+motor_a1:-1,motor_b:-1+motor_b1:-1";
const COMBINE_AXIS: &str = "X";
const SERIES_STRIDE: usize = 1000;
const REL_TOL: f64 = 1e-6;
const ABS_TOL: f64 = 1e-9;

const CAPTURES: [&str; 2] = [
    "cal_p880_s550_i2273_20260710_151516.scap",
    "cal_p1120_s700_i1786_20260710_151521.scap",
];

fn fixture_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../test/fixtures/servo_captures")
}

fn load_gz(name: &str) -> Vec<u8> {
    let path = fixture_dir().join(format!("{name}.gz"));
    let file =
        std::fs::File::open(&path).unwrap_or_else(|e| panic!("open {}: {e}", path.display()));
    let mut d = GzDecoder::new(file);
    let mut out = Vec::new();
    d.read_to_end(&mut out).expect("gunzip fixture");
    out
}

fn analyze(name: &str) -> Value {
    let bytes = load_gz(name);
    let cap = Scap::from_bytes(&bytes).expect("parse scap");
    let fs = cap.fs();
    let mut drives = serde_json::Map::new();
    for (idx, dname) in cap.drive_names().into_iter().enumerate() {
        let series = drive_series(&cap, idx).unwrap();
        let metrics =
            compute_metrics(&series, SETTLE_BAND_COUNTS, TORQUE_LIMIT_PER_MILLE, fs).unwrap();
        let segs = motion_segments(&series.flags);
        let (freqs, psd) = moving_psd(&series, &segs, fs).unwrap();
        let peaks: Vec<Value> = top_peaks(&freqs, &psd, 5)
            .into_iter()
            .map(|(f, p)| json!([f, p]))
            .collect();
        drives.insert(
            dname,
            json!({
                "metrics": serde_json::to_value(&metrics).unwrap(),
                "psd_peaks": peaks,
            }),
        );
    }
    let c = compute_corexy_combine(&cap, COMBINE_SPEC, Some(COMBINE_AXIS)).unwrap();
    let on: Vec<f64> = (0..c.on_ferr.len())
        .filter(|&k| c.moving[k])
        .map(|k| c.on_ferr[k])
        .collect();
    let cross: Vec<f64> = (0..c.cross_ferr.len())
        .filter(|&k| c.moving[k])
        .map(|k| c.cross_ferr[k])
        .collect();
    let sample = |v: &[f64]| -> Vec<f64> { v.iter().step_by(SERIES_STRIDE).copied().collect() };
    json!({
        "fs": fs,
        "settle_band_counts": SETTLE_BAND_COUNTS,
        "torque_limit_per_mille": TORQUE_LIMIT_PER_MILLE,
        "drives": Value::Object(drives),
        "combine": {
            "spec": COMBINE_SPEC,
            "axis": COMBINE_AXIS,
            "on_ferr_peak_mm": peak_abs(&on),
            "on_ferr_rms_mm": rms(&on),
            "cross_ferr_peak_mm": peak_abs(&cross),
            "on_ferr_sampled_mm": sample(&c.on_ferr),
            "cross_ferr_sampled_mm": sample(&c.cross_ferr),
        },
    })
}

fn assert_matches(actual: &Value, golden: &Value, where_: &str) {
    match golden {
        Value::Object(g) => {
            let a = actual
                .as_object()
                .unwrap_or_else(|| panic!("{where_}: expected object"));
            let mut gk: Vec<&String> = g.keys().collect();
            let mut ak: Vec<&String> = a.keys().collect();
            gk.sort();
            ak.sort();
            assert_eq!(gk, ak, "{where_}: keys differ");
            for (k, gv) in g {
                assert_matches(&a[k], gv, &format!("{where_}.{k}"));
            }
        }
        Value::Array(g) => {
            let a = actual
                .as_array()
                .unwrap_or_else(|| panic!("{where_}: expected array"));
            assert_eq!(a.len(), g.len(), "{where_}: length differs");
            for (i, (av, gv)) in a.iter().zip(g).enumerate() {
                assert_matches(av, gv, &format!("{where_}[{i}]"));
            }
        }
        Value::Number(gn) => {
            let an = actual
                .as_number()
                .unwrap_or_else(|| panic!("{where_}: expected number, got {actual}"));
            let g_int = gn.is_i64() || gn.is_u64();
            let a_int = an.is_i64() || an.is_u64();
            if g_int && a_int {
                assert_eq!(
                    an.as_i64(),
                    gn.as_i64(),
                    "{where_}: integer {an} vs golden {gn}"
                );
            } else {
                let (av, gv) = (an.as_f64().unwrap(), gn.as_f64().unwrap());
                let tol = ABS_TOL + REL_TOL * gv.abs();
                assert!(
                    (av - gv).abs() <= tol,
                    "{where_}: {av} vs golden {gv} (tol {tol})"
                );
            }
        }
        _ => assert_eq!(actual, golden, "{where_}"),
    }
}

fn goldens() -> Value {
    let path = fixture_dir().join("goldens.json");
    let text = std::fs::read_to_string(&path).expect("read goldens.json");
    serde_json::from_str(&text).expect("parse goldens.json")
}

#[test]
fn scap_metrics_match_python_goldens() {
    let g = goldens();
    for name in CAPTURES {
        assert_matches(&analyze(name), &g[name], name);
    }
}
