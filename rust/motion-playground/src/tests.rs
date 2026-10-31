use crate::plan_core::{PARTIAL_BATCH_SEGMENTS, plan_json, plan_json_streaming};

const CONFIG: &str = r#"{
    "max_velocity": 300,
    "max_accel": 3000,
    "square_corner_velocity": 5,
    "max_jerk": 0
}"#;

fn zigzag_gcode() -> String {
    let mut g = String::from("G90\n");
    for i in 0..200 {
        let x = if i % 2 == 0 { 0.0 } else { 10.0 };
        g.push_str(&format!("G1 X{x} Y{} F6000\n", i as f64 * 0.5));
    }
    g
}

#[test]
fn streaming_final_json_is_byte_identical_to_plan() {
    let gcode = zigzag_gcode();
    let full = plan_json(&gcode, CONFIG).unwrap();
    let mut partials: Vec<String> = Vec::new();
    let streamed = plan_json_streaming(&gcode, CONFIG, |p| partials.push(p.to_string())).unwrap();
    assert_eq!(full, streamed);
    assert!(
        !partials.is_empty(),
        "a {}-move file must produce at least one {PARTIAL_BATCH_SEGMENTS}-segment batch",
        200
    );
}

#[test]
fn partials_are_schema_complete_snapshots() {
    let gcode = zigzag_gcode();
    let mut partials: Vec<String> = Vec::new();
    let streamed = plan_json_streaming(&gcode, CONFIG, |p| partials.push(p.to_string())).unwrap();
    let final_snap: serde_json::Value = serde_json::from_str(&streamed).unwrap();
    for partial in &partials {
        let snap: serde_json::Value = serde_json::from_str(partial).unwrap();
        for key in [
            "schema_version",
            "raw_x",
            "raw_y",
            "trajectory",
            "traversal_time_s",
            "seam_max_dp",
            "seam_max_dv",
            "seam_max_da",
            "worst_seams",
        ] {
            assert!(snap.get(key).is_some(), "partial missing {key}");
        }
        assert_eq!(
            snap["schema_version"],
            pipeline_snapshot::SNAPSHOT_SCHEMA_VERSION
        );
        for key in ["spans", "curves", "axes", "t_end"] {
            assert!(
                snap["trajectory"].get(key).is_some(),
                "partial trajectory missing {key}"
            );
        }
        assert_eq!(snap["raw_x"], final_snap["raw_x"]);
        let partial_rows = snap["trajectory"]["axes"][0].as_array().unwrap();
        let final_rows = final_snap["trajectory"]["axes"][0].as_array().unwrap();
        assert!(partial_rows.len() <= final_rows.len());
        assert_eq!(partial_rows[..], final_rows[..partial_rows.len()]);
    }
}

#[test]
fn corner_deviation_replaces_scv_with_identical_output() {
    let gcode = zigzag_gcode();
    // The exact f64 the scv config resolves to (corner_deviation_from_scv),
    // rendered with Rust's round-trip formatting so serde parses it back
    // bit-identically.
    let budget = 5.0_f64 * 5.0 * (std::f64::consts::SQRT_2 - 1.0) / 3000.0;
    let deviation_config = format!(
        r#"{{
        "max_velocity": 300,
        "max_accel": 3000,
        "corner_deviation": {budget},
        "max_jerk": 0
    }}"#
    );
    assert_eq!(
        plan_json(&gcode, CONFIG).unwrap(),
        plan_json(&gcode, &deviation_config).unwrap()
    );
}

#[test]
fn scv_and_corner_deviation_together_are_rejected() {
    let config = r#"{
        "max_velocity": 300,
        "max_accel": 3000,
        "square_corner_velocity": 5,
        "corner_deviation": 0.01,
        "max_jerk": 0
    }"#;
    let err = plan_json("G90\nG1 X10 F6000\n", config).unwrap_err();
    assert!(err.contains("set exactly one"), "got: {err}");
}

#[test]
fn missing_corner_budget_is_rejected() {
    let config = r#"{
        "max_velocity": 300,
        "max_accel": 3000,
        "max_jerk": 0
    }"#;
    let err = plan_json("G90\nG1 X10 F6000\n", config).unwrap_err();
    assert!(
        err.contains("one of square_corner_velocity or corner_deviation"),
        "got: {err}"
    );
}

#[test]
fn streaming_reports_bad_config_as_an_error() {
    let err = plan_json_streaming("G90\nG1 X10 F6000\n", "{", |_| {
        panic!("no partial expected")
    })
    .unwrap_err();
    assert!(err.starts_with("config:"));
}
