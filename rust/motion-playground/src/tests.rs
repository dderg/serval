use crate::plan_core::{PARTIAL_BATCH_SEGMENTS, plan_json, plan_json_streaming};

const CONFIG: &str = r#"{
    "max_velocity": 300,
    "max_accel": 3000,
    "square_corner_velocity": 5,
    "max_jerk": 100000
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
            "raw_x",
            "raw_y",
            "traj_x_pieces",
            "traj_y_pieces",
            "traj_z_pieces",
            "traj_e_pieces",
            "traj_t_end",
            "traversal_time_s",
            "seam_max_dp",
            "seam_max_dv",
            "seam_max_da",
            "worst_seams",
        ] {
            assert!(snap.get(key).is_some(), "partial missing {key}");
        }
        assert_eq!(snap["raw_x"], final_snap["raw_x"]);
        let partial_pieces = snap["traj_x_pieces"].as_array().unwrap();
        let final_pieces = final_snap["traj_x_pieces"].as_array().unwrap();
        assert!(partial_pieces.len() <= final_pieces.len());
        assert_eq!(partial_pieces[..], final_pieces[..partial_pieces.len()]);
    }
}

#[test]
fn streaming_reports_bad_config_as_an_error() {
    let err = plan_json_streaming("G90\nG1 X10 F6000\n", "{", |_| {
        panic!("no partial expected")
    })
    .unwrap_err();
    assert!(err.starts_with("config:"));
}
