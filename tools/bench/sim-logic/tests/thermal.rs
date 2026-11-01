use bench_sim_logic::thermal::{
    Heater, HeaterParams, BED_DEFAULT, EXTRUDER_DEFAULT, RATIO_MAX, RATIO_MIN,
};

fn beta_ratio(params: &HeaterParams, temp_c: f64) -> f64 {
    let inverse = 1.0 / (temp_c + 273.15) - 1.0 / 298.15;
    let r = f64::from(params.r25_ohm) * (f64::from(params.beta) * inverse).exp();
    r / (r + f64::from(params.pullup_ohm))
}

#[test]
fn full_power_follows_the_first_order_solution() {
    let mut heater = Heater::at_ambient(EXTRUDER_DEFAULT);
    let p = EXTRUDER_DEFAULT;
    let tau = p.capacity_j_per_k / p.loss_w_per_k;
    let rise = p.power_w / p.loss_w_per_k;
    let dt = 1e-3;
    for _ in 0..30_000 {
        heater.step(dt, 1.0, 0.0);
    }
    let expected = p.ambient_c + rise * (1.0 - (-30.0f32 / tau).exp());
    assert!(
        (heater.temp_c - expected).abs() < 0.05,
        "{} vs {expected}",
        heater.temp_c
    );
}

#[test]
fn the_fan_cools_toward_ambient_faster() {
    let mut still = Heater {
        params: EXTRUDER_DEFAULT,
        temp_c: 200.0,
    };
    let mut fanned = still;
    for _ in 0..5_000 {
        still.step(1e-3, 0.0, 0.0);
        fanned.step(1e-3, 0.0, 1.0);
    }
    assert!(fanned.temp_c < still.temp_c - 5.0);
    assert!(fanned.temp_c > EXTRUDER_DEFAULT.ambient_c);
}

#[test]
fn the_dac_carries_the_ratio_klippy_inverts() {
    for temp_c in [20.0f32, 25.0, 60.0, 150.0, 210.0, 280.0] {
        let heater = Heater {
            params: EXTRUDER_DEFAULT,
            temp_c,
        };
        let dac = heater.dac();
        assert!(!dac.clamped, "{temp_c} °C must be inside the DAC range");
        let expected = beta_ratio(&EXTRUDER_DEFAULT, f64::from(temp_c)) * 4095.0;
        assert!(
            (f64::from(dac.code) - expected).abs() <= 0.6,
            "{temp_c}: {} vs {expected}",
            dac.code
        );
    }
}

#[test]
fn default_ratios_match_the_documented_range() {
    let ratio = |t| EXTRUDER_DEFAULT.adc_ratio(t);
    assert!((ratio(25.0) - 0.909).abs() < 0.001);
    assert!((ratio(20.0) - 0.922).abs() < 0.001);
    assert!((ratio(280.0) - 0.088).abs() < 0.001);
    assert_eq!(BED_DEFAULT.adc_ratio(25.0), ratio(25.0));
}

#[test]
fn outside_the_buffered_range_the_dac_clamps_and_says_so() {
    let cold = Heater {
        params: EXTRUDER_DEFAULT,
        temp_c: -20.0,
    };
    let hot = Heater {
        params: EXTRUDER_DEFAULT,
        temp_c: 400.0,
    };
    assert!(cold.dac().clamped);
    assert_eq!(cold.dac().code, (RATIO_MAX * 4095.0 + 0.5) as u16);
    assert!(hot.dac().clamped);
    assert_eq!(hot.dac().code, (RATIO_MIN * 4095.0 + 0.5) as u16);
}

#[test]
fn invalid_parameters_are_detected() {
    assert!(EXTRUDER_DEFAULT.valid());
    assert!(!HeaterParams {
        capacity_j_per_k: 0.0,
        ..EXTRUDER_DEFAULT
    }
    .valid());
    assert!(!HeaterParams {
        power_w: f32::NAN,
        ..EXTRUDER_DEFAULT
    }
    .valid());
}

#[test]
fn temperatures_report_in_centidegrees() {
    let heater = Heater {
        params: EXTRUDER_DEFAULT,
        temp_c: 215.456,
    };
    assert_eq!(heater.temp_centi(), 21_546);
}
