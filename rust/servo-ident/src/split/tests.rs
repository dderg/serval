use super::*;

#[test]
fn raw_absolute_value_precedes_segment_filtering() {
    let structure = Structure::new(vec![vec![0.5, 0.5]]);
    let params = PhysicalParams {
        mass: vec![1.0],
        viscous: vec![0.0],
        coulomb: vec![0.0],
    };
    let n = 401;
    let acc: Vec<f64> = (0..n)
        .map(|sample| if sample < n / 2 { -1.0 } else { 1.0 })
        .collect();
    let cap = Capture {
        t: (0..n).map(|sample| sample as f64 * 0.001).collect(),
        acc: vec![acc.clone(), acc],
        vel: vec![vec![0.0; n], vec![0.0; n]],
        vel_act: vec![vec![0.0; n], vec![0.0; n]],
        torque: vec![vec![0.0; n], vec![0.0; n]],
        ferr: vec![vec![0.0; n], vec![0.0; n]],
    };
    let predictor = belt_force_magnitude(&structure, &params, &cap, 0, 60.0);
    assert!(predictor.iter().all(|value| (*value - 1.0).abs() < 1.0e-12));
}
