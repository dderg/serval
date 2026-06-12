use geometry::{CubicSegment, FollowerDemand, GeometryError, SourceRange};
use nurbs::{VectorNurbs, eval::vector_eval};

fn valid_cubic_xyz() -> VectorNurbs<f64, 3> {
    VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
        ],
    )
    .expect("valid cubic")
}

fn dummy_source() -> SourceRange {
    SourceRange {
        start_line: 1,
        end_line: 1,
    }
}

#[test]
fn try_new_rejects_non_cubic() {
    let linear = VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
    )
    .expect("valid linear");
    let result = CubicSegment::try_new(linear, vec![], 100.0, dummy_source(), None);
    assert!(matches!(
        result,
        Err(GeometryError::NotSinglePieceCubic { .. })
    ));
}

#[test]
fn try_new_accepts_valid_travel() {
    let result = CubicSegment::try_new(valid_cubic_xyz(), vec![], 100.0, dummy_source(), None);
    assert!(result.is_ok());
}

#[test]
fn try_new_accepts_follower_with_signed_ratio() {
    let result = CubicSegment::try_new(
        valid_cubic_xyz(),
        vec![FollowerDemand {
            axis_index: 3,
            ratio: -0.05,
        }],
        100.0,
        dummy_source(),
        None,
    );
    assert!(result.is_ok());
}

#[test]
fn try_new_rejects_zero_follower_ratio() {
    let result = CubicSegment::try_new(
        valid_cubic_xyz(),
        vec![FollowerDemand {
            axis_index: 3,
            ratio: 0.0,
        }],
        100.0,
        dummy_source(),
        None,
    );
    assert!(matches!(
        result,
        Err(GeometryError::FollowerInvariantViolation { .. })
    ));
}

#[test]
fn try_new_rejects_duplicate_follower_axis() {
    let result = CubicSegment::try_new(
        valid_cubic_xyz(),
        vec![
            FollowerDemand {
                axis_index: 3,
                ratio: 0.1,
            },
            FollowerDemand {
                axis_index: 3,
                ratio: 0.2,
            },
        ],
        100.0,
        dummy_source(),
        None,
    );
    assert!(matches!(
        result,
        Err(GeometryError::FollowerInvariantViolation { .. })
    ));
}

#[test]
fn live_reduce_rejects_g1() {
    use geometry::{Fatal, FitterParams, GeometryPipeline, Item, TelemetryEvent};

    let mut events: Vec<TelemetryEvent> = vec![];
    let items: Vec<Item> = {
        let mut pipeline = GeometryPipeline::new(FitterParams::default(), vec![]);
        let mut sink = |evt: TelemetryEvent| events.push(evt);
        pipeline.process("G1 X10 Y10 F1000\n", &mut sink).collect()
    };

    assert!(
        items.iter().any(|item| matches!(
            item,
            Item::Fatal(Fatal::UnsupportedGcode {
                gcode_kind: "G0/G1",
                ..
            })
        )),
        "G1 input should produce Item::Fatal(Fatal::UnsupportedGcode {{ gcode_kind: \"G0/G1\" }}); got {items:#?}"
    );
}

#[test]
fn degree_elevation_preserves_curve() {
    use geometry::degree_elevate_2_to_3;

    let q = VectorNurbs::<f64, 3>::try_new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [1.0, 1.0, 0.0], [2.0, 0.0, 0.0]],
    )
    .unwrap();

    let cubic = degree_elevate_2_to_3(&q);

    for i in 0..=100 {
        let u = f64::from(i) / 100.0;
        let q_val = vector_eval(&q, u);
        let c_val = vector_eval(&cubic, u);
        for axis in 0..3 {
            assert!(
                (q_val[axis] - c_val[axis]).abs() < 1e-12,
                "axis {axis} mismatch at u={u}: q={q_val:?} c={c_val:?}",
            );
        }
    }
}

#[test]
fn live_reduce_rejects_g2() {
    use geometry::{Fatal, FitterParams, GeometryPipeline, Item, TelemetryEvent};

    let mut events: Vec<TelemetryEvent> = vec![];
    let items: Vec<Item> = {
        let mut pipeline = GeometryPipeline::new(FitterParams::default(), vec![]);
        let mut sink = |evt: TelemetryEvent| events.push(evt);
        pipeline
            .process("G2 X10 Y10 I5 J5 F1000\n", &mut sink)
            .collect()
    };

    assert!(
        items.iter().any(|item| matches!(
            item,
            Item::Fatal(Fatal::UnsupportedGcode {
                gcode_kind: "G2/G3",
                ..
            })
        )),
        "G2 input should produce Item::Fatal(Fatal::UnsupportedGcode {{ gcode_kind: \"G2/G3\" }}); got {items:#?}"
    );
}

#[test]
fn live_g0_then_g5_aborts_before_emitting_stale_cubic() {
    use geometry::{Fatal, FitterParams, GeometryPipeline, Item, Segment, TelemetryEvent};

    // Pre-fix bug: G0 X10 was rejected without updating state.position, so
    // the subsequent G5 emitted a cubic with cps[0] = [0,0,0] instead of
    // [10,0,0] — silent 10mm geometric corruption. Post-fix: G0 produces
    // Item::Fatal which terminates the iterator before the G5 is processed.
    let mut p = GeometryPipeline::new(FitterParams::default(), vec![]);
    let mut sink = |_e: TelemetryEvent| {};
    let src = "G0 X10 Y0\nG5 X20 Y0 I3 J3 P-3 Q3 F1000\n";
    let items: Vec<_> = p.process(src, &mut sink).collect();

    assert!(
        items.iter().any(|item| matches!(
            item,
            Item::Fatal(Fatal::UnsupportedGcode {
                gcode_kind: "G0/G1",
                ..
            })
        )),
        "expected Item::Fatal(UnsupportedGcode), got {items:#?}"
    );

    let any_cubic = items
        .iter()
        .any(|item| matches!(item, Item::Segment(Segment::Cubic(_))));
    assert!(
        !any_cubic,
        "post-Fatal cubic emission would mean stale-state corruption; got {items:#?}"
    );
}

#[test]
fn z_plus_e_classifies_as_ordinary_move_with_3d_ratio() {
    use geometry::{FitterParams, FollowerWord, GeometryPipeline, Item, Segment, TelemetryEvent};

    // Previously Fatal::HelicalExtrusionUnsupported; the follower ratio is
    // delta over 3D path length, so Z+E is an ordinary move.
    let mut p = GeometryPipeline::new(
        FitterParams::default(),
        vec![FollowerWord {
            letter: b'E',
            axis_index: 3,
        }],
    );
    let mut sink = |_e: TelemetryEvent| {};
    let src = "G5 Z10 E5 I0 J0 P0 Q0 F1500\n";
    let items: Vec<_> = p.process(src, &mut sink).collect();

    let cubic = items
        .iter()
        .find_map(|item| match item {
            Item::Segment(Segment::Cubic(c)) => Some(c.clone()),
            _ => None,
        })
        .expect("pure-Z+E G5 should classify as an ordinary move");
    let path_len = nurbs::arc_length::path_arc_length(&cubic.xyz);
    assert_eq!(cubic.followers.len(), 1);
    assert_eq!(cubic.followers[0].axis_index, 3);
    assert!((cubic.followers[0].ratio - 5.0 / path_len).abs() < 1e-12);
}

#[test]
fn helical_move_with_follower_classifies_and_pipeline_continues() {
    use geometry::{FitterParams, FollowerWord, GeometryPipeline, Item, Segment, TelemetryEvent};

    // Previously the first move was a fatal helical rejection; now both
    // moves classify and the pipeline keeps going.
    let mut p = GeometryPipeline::new(
        FitterParams::default(),
        vec![FollowerWord {
            letter: b'E',
            axis_index: 3,
        }],
    );
    let mut sink = |_e: TelemetryEvent| {};
    let src = "G5 X10 Y0 Z5 I0 J3 P0 Q-3 E2 F1500\nG5 X20 Y0 I3 J3 P-3 Q3 F1500\n";
    let items: Vec<_> = p.process(src, &mut sink).collect();

    let cubics: Vec<_> = items
        .iter()
        .filter_map(|item| match item {
            Item::Segment(Segment::Cubic(c)) => Some(c),
            _ => None,
        })
        .collect();
    assert_eq!(cubics.len(), 2, "expected two cubics, got {items:#?}");
    let path_len = nurbs::arc_length::path_arc_length(&cubics[0].xyz);
    assert_eq!(cubics[0].followers.len(), 1);
    assert!((cubics[0].followers[0].ratio - 2.0 / path_len).abs() < 1e-12);
    assert!(cubics[1].followers.is_empty());
}

#[test]
fn g92_resets_modal_position_for_subsequent_g5() {
    use geometry::{FitterParams, GeometryPipeline, Item, Segment, TelemetryEvent};
    use nurbs::eval::vector_eval;

    // Pre-fix bug: G92 X10 Y20 didn't update state.position. The subsequent
    // G5 emitted P0 = [0,0,0] instead of [10,20,0] — silent geometric
    // corruption. Post-fix: G92 binds params and writes them through to
    // state.position before the marker break.
    let mut p = GeometryPipeline::new(FitterParams::default(), vec![]);
    let mut sink = |_: TelemetryEvent| {};
    let src = "G92 X10 Y20\nG5 X20 Y30 I0 J5 P0 Q-5 F1500\n";
    let items: Vec<_> = p.process(src, &mut sink).collect();

    let cubic = items
        .iter()
        .find_map(|item| match item {
            Item::Segment(Segment::Cubic(c)) => Some(c.clone()),
            _ => None,
        })
        .expect("expected one Segment::Cubic after G92 + G5");

    let p0 = vector_eval(&cubic.xyz, 0.0);
    assert!(
        (p0[0] - 10.0).abs() < 1e-9 && (p0[1] - 20.0).abs() < 1e-9,
        "post-G92 G5 P0 should be [10, 20, *], got {p0:?}"
    );
}

#[test]
fn g92_e_resets_follower_ledger_for_subsequent_g5_delta() {
    use geometry::{FitterParams, FollowerWord, GeometryPipeline, Item, Segment, TelemetryEvent};

    // Pre-fix: G92 E5 didn't update the ledger. The next G5 with E10 computed
    // delta = 10 - <stale>, instead of 10 - 5.
    let mut p = GeometryPipeline::new(
        FitterParams::default(),
        vec![FollowerWord {
            letter: b'E',
            axis_index: 3,
        }],
    );
    let mut sink = |_: TelemetryEvent| {};
    let src = "G92 E5\nG5 X10 Y0 I3 J3 P-3 Q3 E10 F1500\n";
    let items: Vec<_> = p.process(src, &mut sink).collect();

    let cubic = items
        .iter()
        .find_map(|item| match item {
            Item::Segment(Segment::Cubic(c)) => Some(c.clone()),
            _ => None,
        })
        .expect("expected one Segment::Cubic after G92 E + G5");

    let path_len = nurbs::arc_length::path_arc_length(&cubic.xyz);
    assert_eq!(cubic.followers.len(), 1);
    assert!(
        (cubic.followers[0].ratio - 5.0 / path_len).abs() < 1e-12,
        "expected ratio from delta=5, got {}",
        cubic.followers[0].ratio
    );
}

#[test]
fn g18_then_g5_emits_plane_mismatch_recovery() {
    use geometry::{FitterParams, GeometryPipeline, Item, Recovery, TelemetryEvent};

    // Pre-fix: G5 ignored active_plane and emitted a CurveGeom::Cubic as if
    // XY. Post-fix: mirrors G5.1's plane check; emits Recovery::G5PlaneMismatch.
    let mut p = GeometryPipeline::new(FitterParams::default(), vec![]);
    let mut sink = |_: TelemetryEvent| {};
    let src = "G18\nG5 X10 Y0 I3 J3 P-3 Q3 F1500\n";
    let items: Vec<_> = p.process(src, &mut sink).collect();

    assert!(
        items.iter().any(|item| matches!(
            item,
            Item::Recovered(
                _,
                Recovery::G5PlaneMismatch {
                    active_plane_g_code: 18,
                    ..
                }
            )
        )),
        "G18 + G5 should produce Recovery::G5PlaneMismatch {{ 18 }}, got {items:#?}"
    );
}

#[test]
fn g19_then_g5_emits_plane_mismatch_recovery() {
    use geometry::{FitterParams, GeometryPipeline, Item, Recovery, TelemetryEvent};

    let mut p = GeometryPipeline::new(FitterParams::default(), vec![]);
    let mut sink = |_: TelemetryEvent| {};
    let src = "G19\nG5 X10 Y0 I3 J3 P-3 Q3 F1500\n";
    let items: Vec<_> = p.process(src, &mut sink).collect();

    assert!(
        items.iter().any(|item| matches!(
            item,
            Item::Recovered(
                _,
                Recovery::G5PlaneMismatch {
                    active_plane_g_code: 19,
                    ..
                }
            )
        )),
        "G19 + G5 should produce Recovery::G5PlaneMismatch {{ 19 }}, got {items:#?}"
    );
}

#[test]
fn nan_g5_produces_malformed_params_recovery_not_silent_drop() {
    use geometry::{FitterParams, GeometryPipeline, Item, Recovery, TelemetryEvent};

    // Pre-Fix-H: silent ZeroMotion drop. Rust's f64::FromStr accepts "NaN",
    // so the lexer surfaced the move with NaN-poisoned XY params. The
    // pipeline's ZeroMotion classifier then dropped the move (NaN > 1e-6
    // is false), and modal state.position became NaN-poisoned for every
    // subsequent G5 — silent geometric corruption with zero telemetry.
    //
    // Post-Fix-H: lexer rejects NaN as MalformedNumber, the geometry
    // pipeline maps the parse error to Recovery::MalformedParams via the
    // existing handle_event::ParseError path.
    let mut p = GeometryPipeline::new(FitterParams::default(), vec![]);
    let mut sink = |_e: TelemetryEvent| {};
    let src = "G5 XNaN Y0 I0 J3 P0 Q-3 F1500\n";
    let items: Vec<_> = p.process(src, &mut sink).collect();

    assert!(
        items
            .iter()
            .any(|item| matches!(item, Item::Recovered(_, Recovery::MalformedParams { .. }))),
        "expected Item::Recovered(_, MalformedParams), got {items:#?}"
    );
}

#[test]
fn try_new_rejects_non_finite_control_point() {
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [f64::NAN, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
        ],
    )
    .expect("VectorNurbs accepts NaN at the type level; CubicSegment::try_new must catch it");
    let result = CubicSegment::try_new(
        xyz,
        vec![],
        100.0,
        SourceRange {
            start_line: 1,
            end_line: 1,
        },
        None,
    );
    assert!(matches!(
        result,
        Err(GeometryError::NotSinglePieceCubic { .. })
    ));
}

#[test]
fn try_new_rejects_non_finite_feedrate() {
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
        ],
    )
    .unwrap();
    let result = CubicSegment::try_new(
        xyz,
        vec![],
        f64::INFINITY,
        SourceRange {
            start_line: 1,
            end_line: 1,
        },
        None,
    );
    assert!(matches!(
        result,
        Err(GeometryError::FollowerInvariantViolation { .. })
    ));
}

#[test]
fn try_new_rejects_non_finite_follower_ratio() {
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
        ],
    )
    .unwrap();
    let result = CubicSegment::try_new(
        xyz,
        vec![FollowerDemand {
            axis_index: 3,
            ratio: f64::NAN,
        }],
        100.0,
        SourceRange {
            start_line: 1,
            end_line: 1,
        },
        None,
    );
    assert!(matches!(
        result,
        Err(GeometryError::FollowerInvariantViolation { .. })
    ));
}
