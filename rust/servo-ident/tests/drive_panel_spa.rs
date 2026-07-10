//! The drive tuning panel's pure logic (display/raw unit conversion,
//! autofill derivation, changed-param diffing) lives in `web/app.js` as
//! plain functions rather than behind a Node toolchain this crate doesn't
//! otherwise need. This file is the substitute test rig: it asserts the
//! functions the panel is built from are actually present in the served
//! asset (a rename or an accidental delete during a refactor would slip
//! past `cargo build`, which never looks inside a `include_str!` blob), and
//! that `servo-cal demo`'s `drive_state.json` — the panel's only real
//! fixture — has the shape those functions assume: every param's `group`
//! known to the panel's section order, every motor agreeing (the "single
//! input" rendering path), and the two autofill targets pointing at
//! `speed_gain`.

use std::collections::BTreeSet;

use serde_json::Value;

use servo_ident::assets::APP_JS;
use servo_ident::demo::build_demo;

fn fixture_dir() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../test/fixtures/servo_captures")
}

fn temp_dir(label: &str) -> std::path::PathBuf {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let dir = std::env::temp_dir().join(format!(
        "servo_cal_spa_{label}_{}_{nanos}",
        std::process::id()
    ));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// One assertion per pure function the panel's logic is built from —
/// `docs/rewrite/servo-cal-dashboard.md` documents the contract that these
/// stay grep-able function declarations, not inlined anonymous closures a
/// future refactor could quietly drop.
#[test]
fn app_js_defines_the_pure_drive_panel_functions() {
    let required = [
        "function rawToDisplay(",
        "function displayToRaw(",
        "function deriveGainPositionFromSpeed(",
        "function deriveGainIntegralFromSpeed(",
        "function paramGroupSection(",
        "function groupParams(",
        "function motorRawValues(",
        "function valuesAgree(",
        "function pinnedEntries(",
        "function diffChangedParams(",
        "function buildServoTuneCommands(",
    ];
    for needle in required {
        assert!(
            APP_JS.contains(needle),
            "app.js must define {needle} — the drive tuning panel's pure logic"
        );
    }
}

/// `GET /api/drive_state` and the SPA are both driven by the same
/// `drive_state.json`; this asserts the demo's copy satisfies what the
/// panel's grouping/autofill/mixed-value logic assumes, so the demo
/// actually exercises every rendering path (grouped sections, uniform
/// single-input rows, autofill wiring) rather than just happening to parse.
#[test]
fn demo_drive_state_matches_panel_rendering_assumptions() {
    let out_dir = temp_dir("render");
    build_demo(&out_dir, &fixture_dir()).unwrap();
    let text = std::fs::read_to_string(out_dir.join("drive_state.json")).unwrap();
    let drive_state: Value = serde_json::from_str(&text).unwrap();

    let known_groups: BTreeSet<&str> = ["gains", "filters", "notch", "load", "experimental"].into();
    let params = drive_state["params"].as_array().unwrap();
    for p in params {
        let group = p["group"].as_str().unwrap();
        assert!(
            known_groups.contains(group),
            "demo param {} has group {group:?} outside the panel's known sections — \
             it would silently land in \"other\", defeating this fixture's purpose",
            p["name"]
        );
    }

    let motors = drive_state["motors"].as_object().unwrap();
    for p in params {
        let c_code = p["c_code"].as_str().unwrap();
        let mut values = motors.values().map(|m| {
            m[c_code]
                .as_i64()
                .unwrap_or_else(|| panic!("{c_code} missing on a motor"))
        });
        let first = values.next().unwrap();
        assert!(
            values.all(|v| v == first),
            "demo motors must agree on {c_code} — the panel's uniform \"one input\" \
             row is the path the demo is meant to exercise, not the mixed-badge path"
        );
    }

    let autofill_sources: Vec<&str> = params
        .iter()
        .filter_map(|p| p["autofill"].as_str())
        .collect();
    assert!(autofill_sources.contains(&"gain_position_from_speed"));
    assert!(autofill_sources.contains(&"gain_integral_from_speed"));
    let speed_gain = params
        .iter()
        .find(|p| p["name"] == "speed_gain")
        .expect("speed_gain must be present as the autofill source");
    assert!(
        speed_gain["autofill"].is_null(),
        "speed_gain is the autofill source, not an autofill target"
    );

    std::fs::remove_dir_all(&out_dir).ok();
}
