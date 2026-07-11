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
