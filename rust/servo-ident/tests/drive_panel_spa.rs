//! The drive tuning panel's pure logic (changed-param diffing,
//! SERVO_TUNE line building) lives in `web/src/*.ts` as plain functions.
//! This file is the substitute test rig: it asserts the functions the
//! panel is built from are actually present in the sources bun bundles
//! into the served asset (a rename or an accidental delete during a
//! refactor would slip past `cargo build`, which never greps them), and
//! that `servo-cal demo`'s `drive_state.json` — the grid's only real
//! fixture — has the shape those functions assume: every param's `group`
//! known to the pages' section order, and motors agreeing everywhere
//! except the one deliberate drift fixture.

use std::collections::BTreeSet;

use serde_json::Value;

use servo_ident::demo::build_demo;

fn collect_ts(dir: &std::path::Path, sources: &mut Vec<String>) {
    for entry in std::fs::read_dir(dir).expect("read web/src dir") {
        let path = entry.expect("read web/src entry").path();
        if path.is_dir() {
            collect_ts(&path, sources);
            continue;
        }
        assert_eq!(
            path.extension().and_then(|e| e.to_str()),
            Some("ts"),
            "unexpected file in web/src: {}",
            path.display()
        );
        sources.push(std::fs::read_to_string(&path).expect("read web/src module"));
    }
}

fn all_js() -> String {
    let src_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("web/src");
    let mut sources = Vec::new();
    collect_ts(&src_dir, &mut sources);
    assert!(!sources.is_empty(), "no modules found in web/src");
    sources.join("\n")
}

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
            all_js().contains(needle),
            "the js modules must define {needle} — the drive tuning panel's pure logic"
        );
    }
}

/// Same contract for the differential FRF section: the series validation,
/// trace building, and mode-marker formatting stay grep-able function
/// declarations in the served asset.
#[test]
fn app_js_defines_the_differential_frf_functions() {
    let required = [
        "function differentialSeries(",
        "function frfTraces(",
        "function frfModeMarkers(",
        "function frfModeTableHtml(",
        "function renderFrfCharts(",
        "case \"differential\"",
    ];
    for needle in required {
        assert!(
            all_js().contains(needle),
            "the js modules must define {needle} — the differential FRF rendering"
        );
    }
}

/// Same contract for the tracking-metrics table (the gains/dynamics pages'
/// view on results.json's per-drive overshoot/settle/torque metrics) and
/// the console template that launches the tracking run those metrics come
/// from.
#[test]
fn app_js_defines_the_tracking_metrics_functions() {
    let required = [
        "function driveMoveSummary(",
        "function settleCellHtml(",
        "function torqueCellHtml(",
        "function metricsTableRows(",
        "function renderMetricsTable(",
        "function heatCellStyle(",
        "SERVO_MEASURE_TRACKING",
    ];
    for needle in required {
        assert!(
            all_js().contains(needle),
            "the js modules must define {needle} — the tracking metrics table"
        );
    }
}

/// Same contract for the metrics-vs-gain chart (the gains page's revival of
/// the old gain-report PNG's "metrics vs gain" panel: per-step overshoot /
/// ferr against the swept value, flagged steps as red rungs).
#[test]
fn app_js_defines_the_sweep_metrics_chart_functions() {
    let required = [
        "function sweptAxisKey(",
        "function sweepMetricsSeries(",
        "function renderSweepMetricsChart(",
        "function motorViewPerMotor(",
        "sweep-metrics-chart",
    ];
    for needle in required {
        assert!(
            all_js().contains(needle),
            "the js modules must define {needle} — the metrics-vs-gain chart"
        );
    }
}

/// Same contract for the console's response echo: klippy respond_info text
/// only travels Moonraker's websocket, so the console harvests it from
/// /server/gcode_store after each blocking script call and renders it under
/// the sent line — without this, command output is only visible in mainsail.
#[test]
fn app_js_defines_the_console_response_functions() {
    let required = [
        "function latestGcodeStoreTime(",
        "function fetchGcodeResponses(",
        "/server/gcode_store",
        "resp-line",
    ];
    for needle in required {
        assert!(
            all_js().contains(needle),
            "the js modules must define {needle} — the console response echo"
        );
    }
}

/// Same contract for the strain-map tab: the grouping, colormap, heatmap,
/// and profile/DC rendering stay grep-able function declarations in the
/// served asset, and the tab's data source stays the strain endpoint.
#[test]
fn app_js_defines_the_strain_map_functions() {
    let required = [
        "function strainGroups(",
        "function strainColor(",
        "function drawStrainHeatmap(",
        "function HeatmapCanvas(",
        "function ProfileChart(",
        "function DcBarsCanvas(",
        "\"strain_map\"",
        "function getRunStrain(",
    ];
    for needle in required {
        assert!(
            all_js().contains(needle),
            "the js modules must define {needle} — the strain map rendering"
        );
    }
}

/// `GET /api/drive_state` and the SPA are both driven by the same
/// `drive_state.json`; this asserts the demo's copy satisfies what the
/// panel's grouping/drift logic assumes, so the demo actually
/// exercises every rendering path (grouped sections, per-motor grid rows,
/// the drift highlight, enum selects) rather than just happening to
/// parse.
#[test]
fn demo_drive_state_matches_panel_rendering_assumptions() {
    let out_dir = temp_dir("render");
    build_demo(&out_dir, &fixture_dir()).unwrap();
    let text = std::fs::read_to_string(out_dir.join("drive_state.json")).unwrap();
    let drive_state: Value = serde_json::from_str(&text).unwrap();

    let known_groups: BTreeSet<&str> = [
        "gains",
        "filters",
        "notch",
        "speed_observer",
        "disturbance_observer",
    ]
    .into();
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

    let drift_c_code = "C01.40";
    let motors = drive_state["motors"].as_object().unwrap();
    for p in params {
        let c_code = p["c_code"].as_str().unwrap();
        let values: Vec<i64> = motors
            .values()
            .map(|m| {
                m[c_code]
                    .as_i64()
                    .unwrap_or_else(|| panic!("{c_code} missing on a motor"))
            })
            .collect();
        let agree = values.iter().all(|v| *v == values[0]);
        if c_code == drift_c_code {
            assert!(
                !agree,
                "demo motors must DISAGREE on {c_code} — it is the fixture for the \
                 grid's per-motor drift highlight"
            );
        } else {
            assert!(
                agree,
                "demo motors must agree on {c_code} — only {drift_c_code} carries the \
                 deliberate drift"
            );
        }
    }

    let notch_group_count = params.iter().filter(|p| p["group"] == "notch").count();
    assert_eq!(
        notch_group_count, 15,
        "demo must carry the full notch bank (5 × freq/width/depth)"
    );
    let speed_feedback_filter = params
        .iter()
        .find(|p| p["name"] == "speed_feedback_filter")
        .expect("speed_feedback_filter present");
    assert!(
        speed_feedback_filter["options"].is_object(),
        "speed_feedback_filter must ship enum options for the panel's labeled select"
    );

    let slots = drive_state["slots"].as_object().unwrap();
    assert_eq!(
        slots.keys().collect::<Vec<_>>(),
        motors.keys().collect::<Vec<_>>(),
        "slots must key exactly the dumped motors — the panel maps live-telemetry \
         slot numbers back to motor names through it"
    );
    let slot_indices: BTreeSet<u64> = slots.values().map(|v| v.as_u64().unwrap()).collect();
    assert_eq!(
        slot_indices.len(),
        slots.len(),
        "slot indices must be distinct"
    );

    std::fs::remove_dir_all(&out_dir).ok();
}
