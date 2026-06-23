use nurbs::VectorNurbs;
use temporal::{GridConfig, GridScheme, Limits, SolveStatus, schedule_segment};

#[test]
fn micro_move_below_cut_floor_is_not_max_iter_slp() {
    let curve = VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [0.03, 0.0, 0.0]],
    )
    .expect("degree-1 line NURBS always valid");

    let limits = Limits::axis_boxes(
        [300.0, 300.0, 15.0],
        [5_000.0, 5_000.0, 350.0],
        [1.0, 1.0, 1.0],
    );

    let profile = schedule_segment(
        &curve,
        &limits,
        &GridConfig {
            scheme: GridScheme::UniformArclength,
            n: 20,
        },
        0.0,
        4e-4_f64,
    )
    .expect("schedule");

    assert!(
        !matches!(profile.status, SolveStatus::MaxIterSlp { .. }),
        "micro-move with all b < SLP_B_CUT_FLOOR must not return MaxIterSlp; \
         got {:?}",
        profile.status,
    );
}
