//! Strain-map analysis and `GET /api/runs/<name>/strain` endpoint tests:
//! synthesize a tiny strain_map run (triangle sweep, known elastic ramp plus
//! constant Coulomb friction) and assert the binned elastic/friction
//! profiles recover the ground truth, then drive the real router over it.

use std::io::{Read, Write};
use std::net::TcpStream;
use std::path::{Path, PathBuf};
use std::time::Duration;

use serde_json::Value;

use servo_ident::strain::{analyze_run, belt_pairs};
use servo_ident::{http, serve};

const CPM: f64 = 100.0;
const COUNTS_BASE: f64 = 5000.0;
const MOTORS: [(&str, bool); 4] = [
    ("motor_a", false),
    ("motor_a1", true),
    ("motor_b", true),
    ("motor_b1", true),
];
const BELTS: &str = "motor_a:1+motor_a1:-1,motor_b:-1+motor_b1:-1";
const ELASTIC_SLOPE_OFFSET_MM: f64 = 20.0;
const FRICTION_PCT: f64 = 3.0;
const BELT_B_ELASTIC_PCT: f64 = 5.0;

fn mech_sign(invert: bool) -> f64 {
    if invert {
        -1.0
    } else {
        1.0
    }
}

fn scap_bytes(mech_pos: &[Vec<f64>], mech_torque: &[Vec<f64>]) -> Vec<u8> {
    let drives = MOTORS
        .iter()
        .map(|(name, invert)| {
            format!("{{\"name\":\"{name}\",\"counts_per_mm\":{CPM},\"invert\":{invert}}}")
        })
        .collect::<Vec<_>>()
        .join(",");
    let header = format!(
        "{{\"version\":2,\"cycle_ns\":250000,\"record_size\":33,\
         \"drives\":[{drives}],\
         \"channels\":[{{\"name\":\"cycle_index\",\"dtype\":\"u64\",\"offset\":0}},\
         {{\"name\":\"flags\",\"dtype\":\"u8\",\"offset\":8}},\
         {{\"name\":\"target_counts\",\"dtype\":\"i32\",\"offset\":9}},\
         {{\"name\":\"torque_actual\",\"dtype\":\"i16\",\"offset\":13}}]}}"
    );
    let mut b = header.into_bytes();
    b.push(b'\n');
    let n = mech_pos[0].len();
    for k in 0..n {
        b.extend_from_slice(&(k as u64).to_le_bytes());
        b.push(3u8);
        for (di, (_, invert)) in MOTORS.iter().enumerate() {
            let sign = mech_sign(*invert);
            let counts = (COUNTS_BASE + mech_pos[di][k] * CPM * sign).round() as i32;
            let torque = (mech_torque[di][k] * 10.0 * sign).round() as i16;
            b.extend_from_slice(&counts.to_le_bytes());
            b.extend_from_slice(&torque.to_le_bytes());
        }
    }
    b
}

fn triangle_positions() -> Vec<f64> {
    let up = (0..=400).map(|k| k as f64 * 0.1);
    let down = (401..=800).map(|k| 40.0 - (k - 400) as f64 * 0.1);
    up.chain(down).collect()
}

/// Belt A carries a linear elastic ramp plus Coulomb friction flipping with
/// direction; belt B carries a pure DC elastic offset. All four torques are
/// exact multiples of 0.1% so the i16 channel stores them losslessly.
fn line_channels(sweep: &[f64]) -> (Vec<Vec<f64>>, Vec<Vec<f64>>) {
    let n = sweep.len();
    let apex = sweep
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.total_cmp(b.1))
        .map(|(i, _)| i)
        .unwrap();
    let torque_a: Vec<f64> = (0..n)
        .map(|k| {
            let dir = if k <= apex { 1.0 } else { -1.0 };
            (sweep[k] - ELASTIC_SLOPE_OFFSET_MM) + FRICTION_PCT * dir
        })
        .collect();
    let torque_a1: Vec<f64> = torque_a.iter().map(|t| -t).collect();
    let torque_b = vec![BELT_B_ELASTIC_PCT; n];
    let torque_b1 = vec![-BELT_B_ELASTIC_PCT; n];
    (
        vec![sweep.to_vec(); 4],
        vec![torque_a, torque_a1, torque_b, torque_b1],
    )
}

fn write_strain_run(run_dir: &Path, steps: &[(&str, Value, Vec<u8>)]) {
    std::fs::create_dir_all(run_dir).unwrap();
    let manifest = serde_json::json!({
        "version": 1,
        "experiment": "strain_map",
        "tag": "strain",
        "axis": "XY",
        "belts": BELTS,
        "stroke_plan": {"x_start": 0.0, "x_end": 40.0, "speed": 50.0, "line_spacing": 20.0},
        "steps": steps.iter().map(|(name, swept, _)| serde_json::json!({
            "name": name, "swept": swept, "capture": format!("step_{name}.scap"),
        })).collect::<Vec<_>>(),
    });
    std::fs::write(
        run_dir.join("manifest.json"),
        serde_json::to_string(&manifest).unwrap(),
    )
    .unwrap();
    for (name, _, bytes) in steps {
        std::fs::write(run_dir.join(format!("step_{name}.scap")), bytes).unwrap();
    }
}

fn temp_dir(label: &str) -> PathBuf {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let dir =
        std::env::temp_dir().join(format!("strain_map_{label}_{}_{nanos}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn xline_step() -> (&'static str, Value, Vec<u8>) {
    let sweep = triangle_positions();
    let (pos, tq) = line_channels(&sweep);
    (
        "xline_y030",
        serde_json::json!({"y": 30.0}),
        scap_bytes(&pos, &tq),
    )
}

/// Pure Y motion on corexy: belt A's lead moves +y, belt B's lead moves -y,
/// so (pa+pb)/2 stays flat and (pa-pb)/2 carries the sweep.
fn yline_step() -> (&'static str, Value, Vec<u8>) {
    let sweep = triangle_positions();
    let (_, tq) = line_channels(&sweep);
    let neg: Vec<f64> = sweep.iter().map(|s| -s).collect();
    let pos = vec![sweep.clone(), sweep.clone(), neg.clone(), neg];
    (
        "yline_x030",
        serde_json::json!({"x": 30.0}),
        scap_bytes(&pos, &tq),
    )
}

#[test]
fn triangle_sweep_recovers_elastic_ramp_and_friction() {
    let root = temp_dir("triangle");
    write_strain_run(&root, &[xline_step()]);

    let map = analyze_run(&root).unwrap();
    assert_eq!(map.lines.len(), 1);
    let line = &map.lines[0];
    assert_eq!(line.name, "xline_y030");
    assert_eq!(line.swept, serde_json::json!({"y": 30.0}));
    assert_eq!(line.bin_centers.len(), 20);
    assert_eq!(line.belts.len(), 2);
    assert_eq!(line.belts[0].pair, "motor_a/motor_a1");
    assert_eq!(line.belts[1].pair, "motor_b/motor_b1");

    for b in 0..20 {
        assert!((line.bin_centers[b] - (2.0 * b as f64 + 1.0)).abs() < 1e-9);
        let mean_pos_in_bin = 2.0 * b as f64 + 0.95;
        let expected_elastic = mean_pos_in_bin - ELASTIC_SLOPE_OFFSET_MM;
        let elastic_a = line.belts[0].elastic[b].expect("both directions sampled");
        let friction_a = line.belts[0].friction[b].expect("both directions sampled");
        assert!(
            (elastic_a - expected_elastic).abs() < 1e-3,
            "bin {b}: elastic {elastic_a} want {expected_elastic}"
        );
        assert!(
            (friction_a - FRICTION_PCT).abs() < 1e-3,
            "bin {b}: friction {friction_a} want {FRICTION_PCT}"
        );
        let elastic_b = line.belts[1].elastic[b].unwrap();
        let friction_b = line.belts[1].friction[b].unwrap();
        assert!(
            (elastic_b - BELT_B_ELASTIC_PCT).abs() < 1e-3,
            "bin {b}: {elastic_b}"
        );
        assert!(friction_b.abs() < 1e-3, "bin {b}: {friction_b}");
    }

    std::fs::remove_dir_all(&root).ok();
}

#[test]
fn y_sweep_line_uses_the_differential_coordinate() {
    let root = temp_dir("yline");
    write_strain_run(&root, &[yline_step()]);

    let map = analyze_run(&root).unwrap();
    let line = &map.lines[0];
    assert_eq!(line.bin_centers.len(), 20);
    let elastic_a = line.belts[0].elastic[5].unwrap();
    let expected = (2.0 * 5.0 + 0.95) - ELASTIC_SLOPE_OFFSET_MM;
    assert!(
        (elastic_a - expected).abs() < 1e-3,
        "{elastic_a} want {expected}"
    );

    std::fs::remove_dir_all(&root).ok();
}

#[test]
fn one_directional_sweep_yields_null_profiles() {
    let root = temp_dir("oneway");
    let sweep: Vec<f64> = (0..=400).map(|k| k as f64 * 0.1).collect();
    let (pos, tq) = line_channels(&sweep);
    write_strain_run(
        &root,
        &[(
            "xline_y030",
            serde_json::json!({"y": 30.0}),
            scap_bytes(&pos, &tq),
        )],
    );

    let map = analyze_run(&root).unwrap();
    let belt = &map.lines[0].belts[0];
    assert_eq!(belt.elastic.len(), 20);
    assert!(belt.elastic.iter().all(Option::is_none));
    assert!(belt.friction.iter().all(Option::is_none));

    std::fs::remove_dir_all(&root).ok();
}

#[test]
fn belt_pairs_parse_lead_and_follower_motors() {
    let pairs = belt_pairs(BELTS).unwrap();
    assert_eq!(pairs[0], ["motor_a".to_string(), "motor_a1".to_string()]);
    assert_eq!(pairs[1], ["motor_b".to_string(), "motor_b1".to_string()]);
    assert!(belt_pairs("motor_a:1,motor_b:1").is_err());
    assert!(belt_pairs("motor_a:1+motor_a1:-1").is_err());
}

// --- endpoint ---------------------------------------------------------------

struct HttpResult {
    status: u16,
    body: String,
}

fn request(port: u16, method: &str, path: &str) -> HttpResult {
    let mut stream = TcpStream::connect(("127.0.0.1", port)).expect("connect");
    stream
        .set_read_timeout(Some(Duration::from_secs(5)))
        .unwrap();
    let req = format!(
        "{method} {path} HTTP/1.1\r\nHost: 127.0.0.1\r\nContent-Length: 0\r\nConnection: close\r\n\r\n"
    );
    stream.write_all(req.as_bytes()).unwrap();
    let mut raw = Vec::new();
    stream.read_to_end(&mut raw).unwrap();
    let text = String::from_utf8_lossy(&raw);
    let mut parts = text.splitn(2, "\r\n\r\n");
    let head = parts.next().unwrap_or("");
    let body = parts.next().unwrap_or("").to_string();
    let status: u16 = head
        .lines()
        .next()
        .and_then(|l| l.split_whitespace().nth(1))
        .and_then(|s| s.parse().ok())
        .expect("status line");
    HttpResult { status, body }
}

fn spawn_server(captures_root: PathBuf) -> u16 {
    let listener = http::bind("127.0.0.1", 0).expect("bind ephemeral port");
    let port = listener.local_addr().unwrap().port();
    std::thread::spawn(move || {
        http::run(listener, move |req| serve::handle(&captures_root, req));
    });
    port
}

fn mtime(path: &Path) -> std::time::SystemTime {
    std::fs::metadata(path).unwrap().modified().unwrap()
}

#[test]
fn strain_endpoint_serves_analysis_and_caches_it() {
    let root = temp_dir("endpoint");
    let run_dir = root.join("strainrun");
    write_strain_run(&run_dir, &[xline_step(), yline_step()]);
    let port = spawn_server(root.clone());

    let resp = request(port, "GET", "/api/runs/strainrun/strain");
    assert_eq!(resp.status, 200, "{}", resp.body);
    let parsed: Value = serde_json::from_str(&resp.body).unwrap();
    let lines = parsed["lines"].as_array().unwrap();
    assert_eq!(lines.len(), 2);
    assert_eq!(lines[0]["name"], Value::from("xline_y030"));
    assert_eq!(lines[0]["swept"]["y"], Value::from(30.0));
    assert_eq!(
        lines[0]["belts"][0]["pair"],
        Value::from("motor_a/motor_a1")
    );
    assert_eq!(
        lines[0]["belts"][1]["pair"],
        Value::from("motor_b/motor_b1")
    );
    assert_eq!(lines[0]["bin_centers"].as_array().unwrap().len(), 20);
    assert_eq!(
        lines[0]["belts"][0]["elastic"].as_array().unwrap().len(),
        20
    );
    let cache_path = run_dir.join("strain.json");
    assert!(cache_path.is_file());

    let first_mtime = mtime(&cache_path);
    std::thread::sleep(Duration::from_millis(20));
    let resp2 = request(port, "GET", "/api/runs/strainrun/strain");
    assert_eq!(resp2.status, 200);
    assert_eq!(
        mtime(&cache_path),
        first_mtime,
        "fresh strain.json must not be recomputed"
    );

    std::thread::sleep(Duration::from_millis(20));
    let scap_path = run_dir.join("step_xline_y030.scap");
    let bytes = std::fs::read(&scap_path).unwrap();
    std::fs::write(&scap_path, bytes).unwrap();
    let resp3 = request(port, "GET", "/api/runs/strainrun/strain");
    assert_eq!(resp3.status, 200);
    assert!(
        mtime(&cache_path) > first_mtime,
        "a newer capture must trigger re-analysis"
    );

    std::fs::remove_dir_all(&root).ok();
}

#[test]
fn strain_endpoint_404s_for_missing_and_non_strain_runs() {
    let root = temp_dir("endpoint_404");
    let other = root.join("gains_run");
    std::fs::create_dir_all(&other).unwrap();
    std::fs::write(
        other.join("manifest.json"),
        serde_json::json!({
            "version": 1, "experiment": "gain_sweep", "tag": "g",
            "steps": [{"name": "s1", "capture": "step_s1.scap"}],
        })
        .to_string(),
    )
    .unwrap();
    let port = spawn_server(root.clone());

    let missing = request(port, "GET", "/api/runs/nope/strain");
    assert_eq!(missing.status, 404);

    let wrong = request(port, "GET", "/api/runs/gains_run/strain");
    assert_eq!(wrong.status, 404);
    let parsed: Value = serde_json::from_str(&wrong.body).unwrap();
    assert!(parsed["error"].as_str().unwrap().contains("gain_sweep"));

    std::fs::remove_dir_all(&root).ok();
}

#[test]
fn strain_endpoint_500s_on_a_malformed_scap() {
    let root = temp_dir("endpoint_500");
    let run_dir = root.join("strainrun");
    write_strain_run(&run_dir, &[xline_step()]);
    std::fs::write(run_dir.join("step_xline_y030.scap"), b"not a capture").unwrap();
    let port = spawn_server(root.clone());

    let resp = request(port, "GET", "/api/runs/strainrun/strain");
    assert_eq!(resp.status, 500, "{}", resp.body);
    assert!(!run_dir.join("strain.json").exists(), "no partial output");

    std::fs::remove_dir_all(&root).ok();
}
