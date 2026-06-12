use nurbs::VectorNurbs;
use temporal::{AxisSet, GridConfig, GridScheme, LimitSet, Limits, SolveStatus, schedule_segment};

fn gantry_limits() -> Limits {
    Limits::try_new(
        &[
            LimitSet {
                axes: AxisSet::from_indices(&[0, 1]),
                v_max: 300.0,
                a_max: 3000.0,
                j_max: 6000.0,
            },
            LimitSet {
                axes: AxisSet::from_indices(&[2]),
                v_max: 15.0,
                a_max: 100.0,
                j_max: 200.0,
            },
        ],
        temporal::N_SPATIAL,
    )
    .unwrap()
}

fn line_100mm(direction: [f64; 2]) -> VectorNurbs<f64, 3> {
    let end = [direction[0] * 100.0, direction[1] * 100.0, 0.0];
    VectorNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![[0.0; 3], end]).unwrap()
}

fn peak_toolhead_accel(curve: &VectorNurbs<f64, 3>) -> f64 {
    let profile = schedule_segment(
        curve,
        &gantry_limits(),
        &GridConfig {
            scheme: GridScheme::UniformArclength,
            n: 200,
        },
        0.0,
        0.0,
    )
    .expect("schedule");
    assert!(
        matches!(
            profile.status,
            SolveStatus::Solved | SolveStatus::SolvedInexact { .. } | SolveStatus::SolvedSlp { .. }
        ),
        "status: {:?}",
        profile.status
    );
    profile
        .samples
        .iter()
        .map(|s| s.a.abs())
        .fold(0.0_f64, f64::max)
}

#[test]
fn diagonal_move_gets_same_peak_accel_as_axis_aligned() {
    let pure_x = peak_toolhead_accel(&line_100mm([1.0, 0.0]));
    let frac = std::f64::consts::FRAC_1_SQRT_2;
    let diagonal = peak_toolhead_accel(&line_100mm([frac, frac]));

    assert!(
        (pure_x - diagonal).abs() / pure_x < 0.02,
        "gantry accel-norm must be direction-isotropic: pure-X peak {pure_x:.1}, \
         45° peak {diagonal:.1}"
    );
    assert!(
        diagonal <= 3000.0 * 1.02,
        "45° move must not exceed the gantry norm cap: peak {diagonal:.1}"
    );
}
