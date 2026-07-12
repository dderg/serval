use config_doc::Document;

use crate::from_doc::{ConsumedValue, read_motion_settings};

fn settings(cfg: &str) -> crate::from_doc::MotionSettings {
    let doc = Document::parse(cfg, "test.cfg").expect("parse ok");
    read_motion_settings(&doc).expect("read ok").0
}

fn read_err(cfg: &str) -> String {
    let doc = Document::parse(cfg, "test.cfg").expect("parse ok");
    read_motion_settings(&doc).expect_err("must fail")
}

const MINIMAL: &str = "[printer]\nmax_velocity: 300\nmax_accel: 3000\n";

#[test]
fn printer_defaults_match_motion_setup() {
    let s = settings(MINIMAL);
    let c = s.cartesian;
    assert_eq!(c.max_velocity, 300.0);
    assert_eq!(c.max_accel, 3000.0);
    // Default scv = 5 -> deviation = scv^2 * (sqrt(2)-1) / accel.
    let expected = 25.0 * (std::f64::consts::SQRT_2 - 1.0) / 3000.0;
    assert!((c.corner_deviation - expected).abs() < 1e-15);
    assert_eq!(c.max_jerk, 6000.0);
    assert_eq!(c.max_z_velocity, 300.0);
    assert_eq!(c.max_z_accel, 3000.0);
    assert_eq!(s.fit_tolerance_mm, 0.005);
    assert_eq!(s.fit_tolerance_accel_mm_s2, 50.0);
    assert_eq!(s.max_extrude_only_velocity, None);
}

#[test]
fn scv_and_corner_deviation_are_exclusive() {
    let err = read_err(
        "[printer]\nmax_velocity: 300\nmax_accel: 3000\n\
         square_corner_velocity: 5\ncorner_deviation: 0.01\n",
    );
    assert!(err.contains("set exactly one"), "{err}");
}

#[test]
fn explicit_corner_deviation_wins() {
    let s = settings("[printer]\nmax_velocity: 300\nmax_accel: 3000\ncorner_deviation: 0.02\n");
    assert_eq!(s.cartesian.corner_deviation, 0.02);
}

#[test]
fn scv_converts_via_accel() {
    let s = settings("[printer]\nmax_velocity: 300\nmax_accel: 2000\nsquare_corner_velocity: 8\n");
    let expected = 64.0 * (std::f64::consts::SQRT_2 - 1.0) / 2000.0;
    assert!((s.cartesian.corner_deviation - expected).abs() < 1e-15);
}

#[test]
fn zero_max_jerk_means_unlimited() {
    let s = settings("[printer]\nmax_velocity: 300\nmax_accel: 3000\nmax_jerk: 0\n");
    assert_eq!(s.cartesian.max_jerk, f64::INFINITY);
}

#[test]
fn unsupported_printer_keys_rejected() {
    let err = read_err("[printer]\nmax_velocity: 300\nmax_accel: 3000\nmax_accel_to_decel: 1000\n");
    assert!(err.contains("max_accel_to_decel is not supported"), "{err}");
}

#[test]
fn missing_required_option_errors_with_klippy_wording() {
    let err = read_err("[printer]\nmax_accel: 3000\n");
    assert_eq!(
        err,
        "Option 'max_velocity' in section 'printer' must be specified"
    );
}

#[test]
fn bounds_use_klippy_wording() {
    let err = read_err("[printer]\nmax_velocity: 0\nmax_accel: 3000\n");
    assert_eq!(
        err,
        "Option 'max_velocity' in section 'printer' must be above 0"
    );
    let err = read_err("[printer]\nmax_velocity: 300\nmax_accel: 3000\nmax_z_velocity: 400\n");
    assert_eq!(
        err,
        "Option 'max_z_velocity' in section 'printer' must have maximum of 300"
    );
}

#[test]
fn axis_sections_parse_lists() {
    let s = settings(&format!(
        "{MINIMAL}[axis x]\nmotors: a, b\n[axis e]\nfollows: X , y\nmotors: e0\n"
    ));
    assert_eq!(s.axes.len(), 2);
    assert_eq!(s.axes[0].name, "x");
    assert_eq!(s.axes[0].motors, vec!["a", "b"]);
    assert!(s.axes[0].follows.is_empty());
    // follows lowercased, motors as written.
    assert_eq!(s.axes[1].follows, vec!["x", "y"]);
}

#[test]
fn limit_sections_resolve_and_validate() {
    let s = settings(&format!(
        "{MINIMAL}[axis x]\n[axis y]\n[limit travel]\naxes: X, y\nmax_accel: 500\nmax_jerk: 0\n"
    ));
    assert_eq!(s.limits.len(), 1);
    assert_eq!(s.limits[0].axes, vec!["x", "y"]);
    assert_eq!(s.limits[0].max_accel, Some(500.0));
    assert_eq!(s.limits[0].max_velocity, None);
    assert_eq!(s.limits[0].max_jerk, Some(f64::INFINITY));
    assert_eq!(s.axis_accel_cap("x"), Some(500.0));
    assert_eq!(s.axis_accel_cap("z"), None);

    let err = read_err(&format!("{MINIMAL}[limit t]\naxes: nope\n"));
    assert!(err.contains("undeclared axis 'nope'"), "{err}");
}

#[test]
fn post_processor_sections_and_reference_validation() {
    let s = settings(&format!(
        "{MINIMAL}[axis x]\npost_processors: sm\n[post_processor sm]\ntype: smooth\nfreq: 42.5\n"
    ));
    assert_eq!(s.post_processors.len(), 1);
    assert_eq!(s.post_processors[0].ty, "smooth");
    assert_eq!(s.post_processors[0].params, vec![("freq".to_owned(), 42.5)]);

    let err = read_err(&format!("{MINIMAL}[axis x]\npost_processors: ghost\n"));
    assert!(err.contains("undeclared post_processor 'ghost'"), "{err}");
}

#[test]
fn legacy_and_unsupported_sections_rejected() {
    assert!(read_err(&format!("{MINIMAL}[stepper_x]\n")).contains("role-encoding motor"));
    assert!(read_err(&format!("{MINIMAL}[stepper_z1]\n")).contains("role-encoding motor"));
    assert!(read_err(&format!("{MINIMAL}[servo_x]\n")).contains("role-encoding servo"));
    assert!(
        read_err(&format!("{MINIMAL}[firmware_retraction]\n"))
            .contains("[firmware_retraction] is not supported")
    );
    assert!(
        read_err(&format!("{MINIMAL}[input_shaper]\n")).contains("[input_shaper] is not supported")
    );
    // Free motor names that merely start with stepper_ stay legal.
    settings(&format!("{MINIMAL}[stepper_left]\n"));
}

#[test]
fn extruder_caps_read_when_present() {
    let s = settings(&format!(
        "{MINIMAL}[extruder]\nmax_extrude_only_velocity: 80\n"
    ));
    assert_eq!(s.max_extrude_only_velocity, Some(80.0));
    assert_eq!(s.max_extrude_only_accel, None);
}

#[test]
fn consumed_options_report_values_and_defaults() {
    let doc = Document::parse(
        "[printer]\nmax_velocity: 300\nmax_accel: 3000\n",
        "test.cfg",
    )
    .unwrap();
    let (_, consumed) = read_motion_settings(&doc).unwrap();
    let find = |opt: &str| {
        consumed
            .iter()
            .find(|c| c.section == "printer" && c.option == opt)
            .unwrap_or_else(|| panic!("{opt} not consumed"))
    };
    assert_eq!(find("max_velocity").value, ConsumedValue::Float(300.0));
    // Defaults taken are recorded too, matching _get_wrapper.
    assert_eq!(find("max_jerk").value, ConsumedValue::Float(6000.0));
    assert_eq!(
        find("max_path_deviation").value,
        ConsumedValue::Float(0.005)
    );
}

#[test]
fn interpolation_refs_are_consumed() {
    let doc = Document::parse(
        "[vars]\nspeed: 300\n[printer]\nmax_velocity: ${vars.speed}\nmax_accel: 3000\n",
        "test.cfg",
    )
    .unwrap();
    let (s, consumed) = read_motion_settings(&doc).unwrap();
    assert_eq!(s.cartesian.max_velocity, 300.0);
    assert!(
        consumed
            .iter()
            .any(|c| c.section == "vars" && c.option == "speed")
    );
}

const COREXY_TOPOLOGY: &str = "\
[kinematics]
type: corexy
axis_x: x
axis_y: y
axis_z: z
a_motors: a
b_motors: b
z_motors: z0, z1

[axis x]
[axis y]
[axis z]

[motor a]
drive: stepper
[motor b]
drive: stepper
[motor z0]
drive: stepper
[motor z1]
drive: stepper
";

fn corexy(extra: &str) -> String {
    format!("{MINIMAL}{COREXY_TOPOLOGY}{extra}")
}

#[test]
fn kinematics_absent_reads_as_none() {
    assert!(settings(MINIMAL).kinematics.is_none());
}

#[test]
fn corexy_topology_parses_lanes_and_claimed_axes() {
    use crate::from_doc::Drive;
    let kin = settings(&corexy("")).kinematics.expect("declared");
    assert_eq!(kin.kind, "corexy");
    assert_eq!(kin.claimed_axes(), ["x", "y", "z"]);
    assert_eq!(kin.lanes.len(), 3);
    assert_eq!(kin.lanes[0].lane_idx, 0);
    assert_eq!(kin.lanes[0].axis, "x");
    assert_eq!(kin.lanes[0].motors, ["a"]);
    assert_eq!(kin.lanes[0].drive, Drive::Stepper);
    assert_eq!(kin.lanes[2].motors, ["z0", "z1"]);
    assert!(kin.followers.is_empty());
}

#[test]
fn printer_kinematics_key_rejected() {
    let err = read_err(&format!(
        "[printer]\nmax_velocity: 300\nmax_accel: 3000\nkinematics: corexy\n{COREXY_TOPOLOGY}"
    ));
    assert!(err.contains("declare a [kinematics] section"), "{err}");
}

#[test]
fn unknown_kinematics_type_lists_supported() {
    let err = read_err(&corexy("").replace("type: corexy", "type: hybrid_corexy"));
    assert_eq!(
        err,
        "[kinematics] type 'hybrid_corexy' is not supported (supported: cartesian, corexy)"
    );
}

#[test]
fn role_bound_to_undeclared_axis_rejected() {
    let err = read_err(&corexy("").replace("axis_x: x", "axis_x: w"));
    assert_eq!(
        err,
        "[kinematics] axis_x binds to axis 'w' but no [axis w] section exists"
    );
}

#[test]
fn lane_without_motors_rejected() {
    let err = read_err(&corexy("").replace("a_motors: a", "a_motors:"));
    assert_eq!(
        err,
        "[kinematics] a_motors declares no motors (lane 0 needs at least one motor)"
    );
}

#[test]
fn missing_motor_section_names_the_referencer() {
    let mut cfg = corexy("");
    cfg = cfg.replace("[motor a]\ndrive: stepper\n", "");
    let err = read_err(&cfg);
    assert_eq!(
        err,
        "[kinematics] a_motors references motor 'a' but no [motor a] section exists"
    );
}

#[test]
fn missing_drive_uses_klippy_wording() {
    let err = read_err(&corexy("").replace("[motor a]\ndrive: stepper", "[motor a]"));
    assert_eq!(err, "Option 'drive' in section 'motor a' must be specified");
}

#[test]
fn invalid_drive_uses_klippy_choice_wording() {
    let err =
        read_err(&corexy("").replace("[motor a]\ndrive: stepper", "[motor a]\ndrive: brushless"));
    assert_eq!(
        err,
        "Choice 'brushless' for option 'drive' in section 'motor a' is not a valid choice"
    );
}

#[test]
fn mixed_drive_lane_rejected() {
    let err =
        read_err(&corexy("").replace("[motor z1]\ndrive: stepper", "[motor z1]\ndrive: servo"));
    assert_eq!(
        err,
        "[kinematics] z_motors mixes stepper and servo motors in one lane; a lane must \
         be all-stepper or all-servo"
    );
}

#[test]
fn orphan_motors_rejected_sorted() {
    let err = read_err(&corexy(
        "[motor spare_b]\ndrive: stepper\n[motor spare_a]\ndrive: stepper\n",
    ));
    assert_eq!(
        err,
        "motor(s) [motor spare_a], [motor spare_b] declared but not assigned to any \
         axis (reference them from a [kinematics] role list or [axis <name>] motors:)"
    );
}

#[test]
fn follower_takes_the_free_slot() {
    let kin = settings(&corexy(
        "[axis e]\nfollows: x\nmotors: extruder\n[motor extruder]\ndrive: stepper\n",
    ))
    .kinematics
    .expect("declared");
    assert_eq!(kin.followers.len(), 1);
    assert_eq!(kin.followers[0].axis, "e");
    assert_eq!(kin.followers[0].motors, ["extruder"]);
    assert_eq!(kin.followers[0].slot, 3);
}

#[test]
fn follower_declared_before_spatial_axes_still_gets_the_free_slot() {
    // A follower [axis e] parsed first (e.g. via an [include] processed
    // early) must not shadow a kinematics lane slot.
    let cfg = format!(
        "{MINIMAL}[axis e]\nfollows: x\nmotors: extruder\n[motor extruder]\ndrive: stepper\n{COREXY_TOPOLOGY}"
    );
    let kin = settings(&cfg).kinematics.expect("declared");
    assert_eq!(kin.followers[0].slot, 3);
}

#[test]
fn servo_follower_rejected() {
    let err = read_err(&corexy(
        "[axis e]\nfollows: x\nmotors: extruder\n[motor extruder]\ndrive: servo\n",
    ));
    assert_eq!(
        err,
        "[axis e] motors references 'extruder' with drive: servo — follower axes \
         support stepper motors only"
    );
}

#[test]
fn follower_overflow_rejected() {
    let extra = "[axis e]\nfollows: x\nmotors: m_e\n[motor m_e]\ndrive: stepper\n\
                 [axis f]\nfollows: x\nmotors: m_f\n[motor m_f]\ndrive: stepper\n";
    let err = read_err(&corexy(extra));
    assert_eq!(
        err,
        "2 follower axes declared but only 1 motion slot(s) free of kinematics lanes"
    );
}

#[test]
fn kinematics_options_reported_as_consumed() {
    let doc = Document::parse(&corexy(""), "test.cfg").expect("parse ok");
    let (_, consumed) = read_motion_settings(&doc).expect("read ok");
    let text = |section: &str, option: &str| {
        consumed
            .iter()
            .find(|c| c.section == section && c.option == option)
            .map(|c| c.value.clone())
    };
    assert_eq!(
        text("kinematics", "type"),
        Some(ConsumedValue::Text("corexy".to_owned()))
    );
    assert_eq!(
        text("kinematics", "z_motors"),
        Some(ConsumedValue::Text("z0, z1".to_owned()))
    );
    assert_eq!(
        text("motor z1", "drive"),
        Some(ConsumedValue::Text("stepper".to_owned()))
    );
}
