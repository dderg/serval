use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};
use std::sync::Arc;

use geometry::path::{Arc as PathArc, Clothoid, CurvatureProfile, Line, PathSegment, Segment};
use geometry::{
    Fade, FollowerDemand, MeshGrid, Move, SourceRange, StraightPhase, SurfaceTransform,
    VelocityLimits,
};

use super::*;

fn close(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() <= 1e-10,
        "expected {expected}, got {actual}"
    );
}

fn move_with(segment: Segment, followers: Vec<FollowerDemand>) -> Move {
    Move {
        segment: PathSegment::try_new(segment, followers).unwrap(),
        feedrate_mm_s: 100.0,
        limits: VelocityLimits::try_new(100.0, 1_000.0, 0.1, f64::INFINITY).unwrap(),
        source: SourceRange {
            start_line: 41,
            end_line: 41,
        },
    }
}

fn analytic_span(
    segment: Segment,
    followers: Vec<FollowerDemand>,
    duration: f64,
    surface: SurfaceMode,
) -> Arc<AnalyticMoveSpan> {
    let length = match &segment {
        Segment::Line(line) => line.length(),
        Segment::Arc(arc) => arc.radius * arc.sweep.abs(),
        Segment::Clothoid(clothoid) => clothoid.length,
    };
    Arc::new(
        AnalyticMoveSpan::try_new(
            move_with(segment, followers),
            Arc::from([StraightPhase {
                t0: 0.0,
                dt: duration,
                s0: 0.0,
                v0: length / duration,
                a0: 0.0,
                j: 0.0,
            }]),
            0.0,
            10.0,
            10.0 + duration,
            Arc::from([0.0, 0.0, 0.0, 7.0]),
            surface,
        )
        .unwrap(),
    )
}

#[test]
fn phase_bounds_remain_finite_at_a_rounded_span_endpoint() {
    let phases = [StraightPhase {
        t0: 0.0,
        dt: 1.0,
        s0: 0.0,
        v0: 2.0,
        a0: 3.0,
        j: 0.0,
    }];
    let rounded_end = f64::from_bits(1.0_f64.to_bits() + 1);

    let bounds = scalar_phase_bounds(&phases, 1.0, rounded_end);
    assert!(bounds.0.is_finite());
    assert!(bounds.1.is_finite());
    assert!(bounds.2.is_finite());
}

fn variable_surface() -> Arc<SurfaceTransform> {
    let mesh = MeshGrid::new(
        0.0,
        0.0,
        1.0,
        1.0,
        4,
        4,
        vec![
            0.0, 0.2, 1.1, 0.4, 0.3, 1.4, -0.2, 2.0, 1.2, -0.7, 2.3, 0.1, -0.4, 1.8, 0.5, 3.0,
        ],
        0.5,
    )
    .unwrap();
    Arc::new(SurfaceTransform::new(
        mesh,
        Fade::new(0.5, 3.5, 0.1).unwrap(),
    ))
}
fn warped_z(
    surface: &SurfaceTransform,
    position: [f64; 3],
    velocity: [f64; 3],
    acceleration: [f64; 3],
) -> Pva {
    let warp = surface.warp(position[0], position[1], position[2]);
    Pva {
        position: position[2] + warp.w,
        velocity: velocity[2]
            + warp.wx * velocity[0]
            + warp.wy * velocity[1]
            + warp.wz * velocity[2],
        acceleration: acceleration[2]
            + warp.wx * acceleration[0]
            + warp.wy * acceleration[1]
            + warp.wz * acceleration[2]
            + warp.wxx * velocity[0] * velocity[0]
            + 2.0 * warp.wxy * velocity[0] * velocity[1]
            + warp.wyy * velocity[1] * velocity[1]
            + 2.0 * warp.wxz * velocity[0] * velocity[2]
            + 2.0 * warp.wyz * velocity[1] * velocity[2],
    }
}

fn independent(axis: ContinuousAxis, source_axis: usize) -> MotorGroup {
    MotorGroup::Independent(MotorTerm {
        source_axis,
        axis,
        scale: 1.0,
    })
}

fn motor_span(axis: ContinuousAxis, t_start: f64, t_end: f64) -> Arc<MotorSpan> {
    Arc::new(
        MotorSpan::try_new(
            Arc::from([independent(axis, 0)]),
            t_start,
            t_end,
            1,
            41,
            false,
        )
        .unwrap(),
    )
}

#[test]
fn analytic_line_arc_and_clothoid_report_exact_pva() {
    let line = analytic_span(
        Segment::Line(Line::try_new([1.0, 2.0, 3.0], [5.0, 2.0, 3.0]).unwrap()),
        vec![],
        2.0,
        SurfaceMode::None,
    );
    assert_eq!(
        line.eval_axis(0, 11.0).unwrap(),
        Pva {
            position: 3.0,
            velocity: 2.0,
            acceleration: 0.0,
        }
    );

    let arc = analytic_span(
        Segment::Arc(
            PathArc::try_new(
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                2.0,
                0.0,
                FRAC_PI_2,
            )
            .unwrap(),
        ),
        vec![],
        PI,
        SurfaceMode::None,
    );
    let arc_x = arc.eval_axis(0, 10.0 + FRAC_PI_2).unwrap();
    close(arc_x.position, 2.0 * libm::cos(FRAC_PI_4));
    close(arc_x.velocity, -libm::sin(FRAC_PI_4));
    close(arc_x.acceleration, -0.5 * libm::cos(FRAC_PI_4));

    let clothoid = analytic_span(
        Segment::Clothoid(
            Clothoid::try_new(
                [2.0, 3.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                0.25,
                0.1,
                4.0,
            )
            .unwrap(),
        ),
        vec![],
        2.0,
        SurfaceMode::None,
    );
    let clothoid_y = clothoid.eval_axis(1, 10.0).unwrap();
    close(clothoid_y.position, 3.0);
    close(clothoid_y.velocity, 0.0);
    close(clothoid_y.acceleration, 1.0);
}

#[test]
fn analytic_phase_distance_origin_maps_to_segment_local_distance() {
    let span = AnalyticMoveSpan::try_new(
        move_with(
            Segment::Line(Line::try_new([10.0, 0.0, 0.0], [12.0, 0.0, 0.0]).unwrap()),
            vec![],
        ),
        Arc::from([StraightPhase {
            t0: 0.0,
            dt: 2.0,
            s0: 7.0,
            v0: 1.0,
            a0: 0.0,
            j: 0.0,
        }]),
        7.0,
        10.0,
        12.0,
        Arc::from([0.0, 0.0, 0.0]),
        SurfaceMode::None,
    )
    .unwrap();
    close(span.eval_axis(0, 11.0).unwrap().position, 11.0);
}

#[test]
fn analytic_phase_distance_gaps_and_overlaps_are_rejected() {
    for second_start in [7.9, 8.1] {
        let second_velocity = 9.0 - second_start;
        let result = AnalyticMoveSpan::try_new(
            move_with(
                Segment::Line(Line::try_new([0.0, 0.0, 0.0], [2.0, 0.0, 0.0]).unwrap()),
                vec![],
            ),
            Arc::from([
                StraightPhase {
                    t0: 0.0,
                    dt: 1.0,
                    s0: 7.0,
                    v0: 1.0,
                    a0: 0.0,
                    j: 0.0,
                },
                StraightPhase {
                    t0: 1.0,
                    dt: 1.0,
                    s0: second_start,
                    v0: second_velocity,
                    a0: 0.0,
                    j: 0.0,
                },
            ]),
            7.0,
            10.0,
            12.0,
            Arc::from([0.0, 0.0, 0.0]),
            SurfaceMode::None,
        );
        assert!(matches!(
            result,
            Err(ContinuousError::PhaseGap { .. } | ContinuousError::PhaseOverlap { .. })
        ));
    }
}

#[test]
fn solver_grade_duration_residual_is_accepted_while_larger_gaps_stay_loud() {
    let cruise = |dt: f64| StraightPhase {
        t0: 0.0,
        dt,
        s0: 0.0,
        v0: 800.0,
        a0: 0.0,
        j: 0.0,
    };
    let build = |dt: f64| {
        AnalyticMoveSpan::try_new(
            move_with(
                Segment::Line(
                    Line::try_new([0.0, 0.0, 0.0], [1.0393208705893557, 0.0, 0.0]).unwrap(),
                ),
                vec![],
            ),
            Arc::from([cruise(dt)]),
            0.0,
            2.162031218590328,
            2.1633303696785617,
            Arc::from([0.0, 0.0, 0.0]),
            SurfaceMode::None,
        )
    };
    assert!(build(0.001299151088234085).is_ok());
    assert!(matches!(
        build(0.001299151088234085 - 1e-9),
        Err(ContinuousError::PhaseEndpointMismatch { .. })
    ));
}

#[test]
fn solver_grade_joint_residual_is_accepted_while_larger_joint_faults_stay_loud() {
    let origin = -2.9558577807620168e-12;
    let cruise_dt = 0.06187712429809089;
    let build = |decel_s0: f64| {
        AnalyticMoveSpan::try_new(
            move_with(
                Segment::Line(
                    Line::try_new([0.0, 0.0, 0.0], [5.5901699437494745, 0.0, 0.0]).unwrap(),
                ),
                vec![],
            ),
            Arc::from([
                StraightPhase {
                    t0: 0.0,
                    dt: cruise_dt,
                    s0: origin,
                    v0: 80.0,
                    a0: 0.0,
                    j: 0.0,
                },
                StraightPhase {
                    t0: cruise_dt,
                    dt: 0.015999999999866357,
                    s0: decel_s0,
                    v0: 80.0,
                    a0: -5000.0,
                    j: 0.0,
                },
            ]),
            origin,
            10.544751256612528,
            10.622628380910486,
            Arc::from([0.0, 0.0, 0.0]),
            SurfaceMode::None,
        )
    };
    assert!(build(4.950169943749529).is_ok());
    assert!(matches!(
        build(4.950169943749529 - 1e-6),
        Err(ContinuousError::PhaseOverlap { .. })
    ));
    assert!(matches!(
        build(4.950169943749529 + 1e-6),
        Err(ContinuousError::PhaseGap { .. })
    ));
}

#[test]
fn analytic_phase_endpoint_mismatch_is_distinct_from_joint_failures() {
    let result = AnalyticMoveSpan::try_new(
        move_with(
            Segment::Line(Line::try_new([0.0, 0.0, 0.0], [2.0, 0.0, 0.0]).unwrap()),
            vec![],
        ),
        Arc::from([StraightPhase {
            t0: 0.0,
            dt: 1.0,
            s0: 7.0,
            v0: 1.0,
            a0: 0.0,
            j: 0.0,
        }]),
        7.0,
        10.0,
        11.0,
        Arc::from([0.0, 0.0, 0.0]),
        SurfaceMode::None,
    );
    assert!(matches!(
        result,
        Err(ContinuousError::PhaseEndpointMismatch { .. })
    ));
}

#[test]
fn stationary_phase_and_roundoff_close_joint_preserve_ordered_coverage() {
    let anchor = 100_000.0;
    let seam = anchor + 32.0 * f64::EPSILON * anchor;
    let span = AnalyticMoveSpan::try_new(
        move_with(
            Segment::Line(Line::try_new([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]).unwrap()),
            vec![],
        ),
        Arc::from([
            StraightPhase {
                t0: 0.0,
                dt: 1.0,
                s0: anchor,
                v0: 0.0,
                a0: 0.0,
                j: 0.0,
            },
            StraightPhase {
                t0: 1.0,
                dt: 1.0,
                s0: seam,
                v0: 1.0,
                a0: 0.0,
                j: 0.0,
            },
        ]),
        anchor,
        10.0,
        12.0,
        Arc::from([0.0, 0.0, 0.0]),
        SurfaceMode::None,
    )
    .unwrap();
    let stationary = span.eval_axis(0, 10.5).unwrap();
    assert_eq!(stationary.position, 0.0);
    assert_eq!(stationary.velocity, 0.0);
    let end = span.eval_axis(0, 12.0).unwrap();
    assert_eq!(end.position, seam + 1.0 - anchor);
    close(end.velocity, 1.0);
}

#[test]
fn analytic_backtracking_is_rejected_as_negative_velocity() {
    let result = AnalyticMoveSpan::try_new(
        move_with(
            Segment::Line(Line::try_new([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]).unwrap()),
            vec![],
        ),
        Arc::from([StraightPhase {
            t0: 0.0,
            dt: 1.0,
            s0: 0.0,
            v0: -1.0,
            a0: 4.0,
            j: 0.0,
        }]),
        0.0,
        10.0,
        11.0,
        Arc::from([0.0, 0.0, 0.0]),
        SurfaceMode::None,
    );
    assert!(matches!(
        result,
        Err(ContinuousError::NegativeVelocity { .. })
    ));
}

#[test]
fn ramped_follower_and_constant_surface_z_are_analytic() {
    let span = analytic_span(
        Segment::Line(Line::try_new([0.0, 0.0, 1.0], [10.0, 0.0, 1.0]).unwrap()),
        vec![FollowerDemand::ramp(3, 0.2, 0.6)],
        2.0,
        SurfaceMode::Constant(2.0),
    );
    assert_eq!(
        span.eval_axis(2, 11.0).unwrap(),
        Pva {
            position: 3.0,
            velocity: 0.0,
            acceleration: 0.0,
        }
    );
    let follower = span.eval_axis(3, 11.0).unwrap();
    close(follower.position, 8.5);
    close(follower.velocity, 2.0);
    close(follower.acceleration, 1.0);
}

#[test]
fn variable_surface_z_reports_chain_rule_pva_inside_mesh_cell() {
    let surface = variable_surface();
    let segment = Segment::Line(Line::try_new([1.2, 1.3, 1.0], [1.8, 1.9, 2.0]).unwrap());
    let length = segment.s_len();
    let span = Arc::new(
        AnalyticMoveSpan::try_new(
            move_with(segment, vec![]),
            Arc::from([StraightPhase {
                t0: 0.0,
                dt: 2.0,
                s0: 0.0,
                v0: length / 4.0,
                a0: length / 4.0,
                j: 0.0,
            }]),
            0.0,
            10.0,
            12.0,
            Arc::from([0.0, 0.0, 0.0, 7.0]),
            SurfaceMode::Variable(Arc::clone(&surface)),
        )
        .unwrap(),
    );
    let expected = warped_z(
        &surface,
        [1.3546875, 1.4546875, 1.2578125],
        [0.2625, 0.2625, 0.4375],
        [0.15, 0.15, 0.25],
    );
    let actual = span.eval_axis(2, 10.75).unwrap();
    close(actual.position, expected.position);
    close(actual.velocity, expected.velocity);
    close(actual.acceleration, expected.acceleration);
    let x = span.eval_axis(0, 10.75).unwrap();
    close(x.position, 1.3546875);
    close(x.velocity, 0.2625);
    close(x.acceleration, 0.15);
    let y = span.eval_axis(1, 10.75).unwrap();
    close(y.position, 1.4546875);
    close(y.velocity, 0.2625);
    close(y.acceleration, 0.15);
}

#[test]
fn variable_surface_z_reports_one_sided_mesh_transition_values() {
    let surface = variable_surface();
    let span = analytic_span(
        Segment::Line(Line::try_new([1.5, 1.4, 1.0], [2.5, 1.4, 2.0]).unwrap()),
        vec![],
        2.0,
        SurfaceMode::Variable(Arc::clone(&surface)),
    );
    let epsilon = 1e-7;
    for sign in [-1.0, 1.0] {
        let offset = sign * epsilon;
        let expected = warped_z(
            &surface,
            [2.0 + 0.5 * offset, 1.4, 1.5 + 0.5 * offset],
            [0.5, 0.0, 0.5],
            [0.0, 0.0, 0.0],
        );
        let actual = span.eval_axis(2, 11.0 + offset).unwrap();
        close(actual.position, expected.position);
        close(actual.velocity, expected.velocity);
        close(actual.acceleration, expected.acceleration);
    }
}

#[test]
fn variable_surface_is_rejected_before_motor_dispatch() {
    let mesh = MeshGrid::new(0.0, 0.0, 1.0, 1.0, 2, 2, vec![0.0; 4], 0.0).unwrap();
    let surface = Arc::new(SurfaceTransform::new(mesh, Fade::disabled()));
    let span = analytic_span(
        Segment::Line(Line::try_new([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]).unwrap()),
        vec![],
        1.0,
        SurfaceMode::Variable(surface),
    );
    let groups = Arc::from([MotorGroup::Analytic {
        span: Arc::clone(&span),
        terms: Arc::from([MotorTerm {
            source_axis: 2,
            axis: ContinuousAxis::Analytic { span, axis: 2 },
            scale: 1.0,
        }]),
    }]);
    assert_eq!(
        MotorSpan::try_new(groups, 10.0, 11.0, 1, 41, false),
        Err(ContinuousError::VariableSurfaceBeforeDispatch)
    );
}

#[test]
fn correlated_corexy_cancellation_has_zero_value_and_bounds() {
    let span = analytic_span(
        Segment::Line(Line::try_new([0.0, 0.0, 0.0], [4.0, 4.0, 0.0]).unwrap()),
        vec![],
        2.0,
        SurfaceMode::None,
    );
    let terms = Arc::from([
        MotorTerm {
            source_axis: 0,
            axis: ContinuousAxis::Analytic {
                span: Arc::clone(&span),
                axis: 0,
            },
            scale: 1.0,
        },
        MotorTerm {
            source_axis: 1,
            axis: ContinuousAxis::Analytic {
                span: Arc::clone(&span),
                axis: 1,
            },
            scale: -1.0,
        },
    ]);
    let motor = MotorSpan::try_new(
        Arc::from([MotorGroup::Analytic { span, terms }]),
        10.0,
        12.0,
        1,
        41,
        false,
    )
    .unwrap();
    assert_eq!(
        motor.eval_pva(11.0).unwrap(),
        Pva {
            position: 0.0,
            velocity: 0.0,
            acceleration: 0.0
        }
    );
    assert_eq!(
        motor.pva_bounds(10.0, 12.0).unwrap(),
        PvaBounds {
            velocity_min: 0.0,
            velocity_max: 0.0,
            acceleration_abs_max: 0.0
        }
    );
}

#[test]
fn analytic_group_cancels_large_scales_before_projection_products() {
    let span = analytic_span(
        Segment::Line(
            Line::try_new([1.0e10, 1.0e10, 0.0], [1.0e10 + 4.0, 1.0e10 + 4.0, 0.0]).unwrap(),
        ),
        vec![],
        2.0,
        SurfaceMode::None,
    );
    let terms = Arc::from([
        MotorTerm {
            source_axis: 0,
            axis: ContinuousAxis::Analytic {
                span: Arc::clone(&span),
                axis: 0,
            },
            scale: 1.0e300,
        },
        MotorTerm {
            source_axis: 1,
            axis: ContinuousAxis::Analytic {
                span: Arc::clone(&span),
                axis: 1,
            },
            scale: -1.0e300,
        },
    ]);
    let motor = MotorSpan::try_new(
        Arc::from([MotorGroup::Analytic { span, terms }]),
        10.0,
        12.0,
        1,
        41,
        false,
    )
    .unwrap();
    assert_eq!(
        motor.eval_pva(11.0).unwrap(),
        Pva {
            position: 0.0,
            velocity: 0.0,
            acceleration: 0.0,
        }
    );
    assert_eq!(motor.pva_bounds(10.0, 12.0).unwrap(), zero_bounds());
}

#[test]
fn multi_revolution_arc_bounds_include_late_extrema_families() {
    let sweep = 200.0 * PI;
    let span = analytic_span(
        Segment::Arc(
            PathArc::try_new(
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                1.0,
                0.0,
                sweep,
            )
            .unwrap(),
        ),
        vec![FollowerDemand::ramp(3, 0.0, 0.5)],
        sweep,
        SurfaceMode::None,
    );
    let terms = Arc::from([
        MotorTerm {
            source_axis: 0,
            axis: ContinuousAxis::Analytic {
                span: Arc::clone(&span),
                axis: 0,
            },
            scale: 1.0,
        },
        MotorTerm {
            source_axis: 3,
            axis: ContinuousAxis::Analytic {
                span: Arc::clone(&span),
                axis: 3,
            },
            scale: 1.0,
        },
    ]);
    let motor = MotorSpan::try_new(
        Arc::from([MotorGroup::Analytic { span, terms }]),
        10.0,
        10.0 + sweep,
        1,
        41,
        false,
    )
    .unwrap();
    let bounds = motor.pva_bounds(10.0, 10.0 + sweep).unwrap();
    let late_peak = motor.eval_pva(10.0 + sweep - FRAC_PI_2).unwrap().velocity;
    assert!(late_peak > 1.49);
    assert!(bounds.velocity_max >= late_peak);
}

#[test]
fn analytic_follower_axes_follow_spatial_axes_in_demand_order() {
    let span = Arc::new(
        AnalyticMoveSpan::try_new(
            move_with(
                Segment::Line(Line::try_new([0.0, 0.0, 0.0], [2.0, 0.0, 0.0]).unwrap()),
                vec![FollowerDemand::constant(6, 0.5)],
            ),
            Arc::from([StraightPhase {
                t0: 0.0,
                dt: 2.0,
                s0: 0.0,
                v0: 1.0,
                a0: 0.0,
                j: 0.0,
            }]),
            0.0,
            10.0,
            12.0,
            Arc::from([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 7.0]),
            SurfaceMode::None,
        )
        .unwrap(),
    );
    assert_eq!(
        span.eval_axis(3, 11.0).unwrap(),
        Pva {
            position: 7.5,
            velocity: 0.5,
            acceleration: 0.0,
        }
    );
    assert!(matches!(
        span.eval_axis(7, 11.0),
        Err(ContinuousError::AxisOutsideMove { axis: 7 })
    ));
}

#[test]
fn clothoid_ramped_followers_cancel_the_shared_spatial_projection() {
    let segment = Segment::Clothoid(
        Clothoid::try_new(
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            0.0,
            0.0,
            4.0,
        )
        .unwrap(),
    );
    let span = Arc::new(
        AnalyticMoveSpan::try_new(
            move_with(
                segment,
                vec![
                    FollowerDemand::ramp(3, -0.75, -1.25),
                    FollowerDemand::ramp(4, -0.25, 0.25),
                ],
            ),
            Arc::from([StraightPhase {
                t0: 0.0,
                dt: 2.0,
                s0: 0.0,
                v0: 2.0,
                a0: 0.0,
                j: 0.0,
            }]),
            0.0,
            10.0,
            12.0,
            Arc::from([0.0, 0.0, 0.0, 0.0, 0.0]),
            SurfaceMode::None,
        )
        .unwrap(),
    );
    let terms = Arc::from([0, 3, 4].map(|source_axis| MotorTerm {
        source_axis,
        axis: ContinuousAxis::Analytic {
            span: Arc::clone(&span),
            axis: source_axis,
        },
        scale: 1.0,
    }));
    let motor = MotorSpan::try_new(
        Arc::from([MotorGroup::Analytic { span, terms }]),
        10.0,
        12.0,
        1,
        41,
        false,
    )
    .unwrap();
    assert_eq!(
        motor.eval_pva(11.0).unwrap(),
        Pva {
            position: 0.0,
            velocity: 0.0,
            acceleration: 0.0,
        }
    );
    assert_eq!(motor.pva_bounds(10.0, 12.0).unwrap(), zero_bounds());
}

#[test]
fn analytic_motor_groups_must_cover_the_motor_time_domain() {
    let span = analytic_span(
        Segment::Line(Line::try_new([0.0, 0.0, 0.0], [2.0, 0.0, 0.0]).unwrap()),
        vec![],
        2.0,
        SurfaceMode::None,
    );
    let group = || MotorGroup::Analytic {
        span: Arc::clone(&span),
        terms: Arc::from([MotorTerm {
            source_axis: 0,
            axis: ContinuousAxis::Analytic {
                span: Arc::clone(&span),
                axis: 0,
            },
            scale: 1.0,
        }]),
    };
    assert!(matches!(
        MotorSpan::try_new(Arc::from([group()]), 9.5, 11.5, 1, 41, false),
        Err(ContinuousError::TimeOutsideSpan { .. })
    ));
    assert!(matches!(
        MotorSpan::try_new(Arc::from([group()]), 10.5, 12.5, 1, 41, false),
        Err(ContinuousError::TimeOutsideSpan { .. })
    ));
}

#[test]
fn holds_use_stream_time_and_reject_outside_samples() {
    let axis = ContinuousAxis::Hold {
        position: 12.5,
        t_start: 7.0,
        t_end: 9.0,
    };
    assert_eq!(
        axis.eval_pva(8.0).unwrap(),
        Pva {
            position: 12.5,
            velocity: 0.0,
            acceleration: 0.0
        }
    );
    assert!(matches!(
        axis.eval_pva(6.0),
        Err(ContinuousError::TimeOutsideSpan { .. })
    ));
    assert!(matches!(
        axis.eval_pva(10.0),
        Err(ContinuousError::TimeOutsideSpan { .. })
    ));
}

#[test]
fn holds_with_unordered_or_non_finite_times_fail_loudly() {
    for (t_start, t_end) in [(f64::NAN, 1.0), (0.0, f64::NAN), (1.0, 0.0)] {
        let axis = ContinuousAxis::Hold {
            position: 4.0,
            t_start,
            t_end,
        };
        assert!(matches!(
            axis.eval_pva(0.5),
            Err(ContinuousError::InvalidSpan { .. })
        ));
        assert!(matches!(
            axis.pva_bounds(0.25, 0.75),
            Err(ContinuousError::InvalidSpan { .. })
        ));
    }
    assert!(matches!(
        MotorSpan::try_new(
            Arc::from([independent(
                ContinuousAxis::Hold {
                    position: 4.0,
                    t_start: 0.0,
                    t_end: 0.0,
                },
                0,
            )]),
            0.0,
            1.0,
            1,
            41,
            true,
        ),
        Err(ContinuousError::InvalidSpan { .. })
    ));
}

#[test]
fn analytic_motor_groups_respect_the_phase_distance_origin() {
    let span = Arc::new(
        AnalyticMoveSpan::try_new(
            move_with(
                Segment::Line(Line::try_new([10.0, 0.0, 0.0], [12.0, 0.0, 0.0]).unwrap()),
                vec![FollowerDemand::ramp(3, 0.0, 2.0)],
            ),
            Arc::from([StraightPhase {
                t0: 0.0,
                dt: 2.0,
                s0: 7.0,
                v0: 1.0,
                a0: 0.0,
                j: 0.0,
            }]),
            7.0,
            10.0,
            12.0,
            Arc::from([0.0, 0.0, 0.0, 5.0]),
            SurfaceMode::None,
        )
        .unwrap(),
    );
    for axis in [0, 3] {
        let terms = [MotorTerm {
            source_axis: axis,
            axis: ContinuousAxis::Analytic {
                span: Arc::clone(&span),
                axis,
            },
            scale: 1.0,
        }];
        let group = analytic_group_pva(&span, &terms, 11.0).unwrap();
        let direct = span.eval_axis(axis, 11.0).unwrap();
        close(group.position, direct.position);
        close(group.velocity, direct.velocity);
        close(group.acceleration, direct.acceleration);
    }
    close(span.eval_axis(3, 11.0).unwrap().position, 5.5);
    let bounds = analytic_group_bounds(&span, std::iter::once((3, 1.0)), 10.0, 12.0).unwrap();
    close(bounds.velocity_min, 0.0);
    close(bounds.velocity_max, 2.0);
    close(bounds.acceleration_abs_max, 1.0);
}

#[test]
fn fractional_clock_mapping_uses_exact_anchor_and_rounds_boundaries() {
    let profile = NudgeProfile::try_new(10.0, 10.0, 0.0, 0.0).unwrap();
    let signal = motor_span(ContinuousAxis::Nudge(profile), 0.0, 1.0);
    let clocked = ClockedMotorSpan::try_new(signal, 0.0, 1.0, 20.0, 21.0, 100.6, 10.0).unwrap();
    assert_eq!((clocked.start_clock, clocked.end_clock), (101, 111));
    assert_eq!(clocked.clock_at_stream_time(0.5).unwrap(), 106);
    close(clocked.stream_time_at_clock(106).unwrap(), 0.54);
    close(clocked.eval_at_clock(106).unwrap().position, 5.4);
    assert_eq!(clocked.stream_time_at_clock(111).unwrap(), 1.0);
    assert_eq!(
        clocked.eval_at_clock(111).unwrap(),
        Pva {
            position: 10.0,
            velocity: 0.0,
            acceleration: 0.0
        }
    );
    assert!(matches!(
        clocked.eval_at_clock(100),
        Err(ContinuousError::ClockOutsideSpan { .. })
    ));
    assert!(matches!(
        clocked.eval_at_clock(112),
        Err(ContinuousError::ClockOutsideSpan { .. })
    ));
}

#[test]
fn exact_clock_endpoints_at_two_to_the_64_are_rejected() {
    let profile = NudgeProfile::try_new(10.0, 10.0, 0.0, 0.0).unwrap();
    let signal = motor_span(ContinuousAxis::Nudge(profile), 0.0, 1.0);
    let boundary = u64::MAX as f64;
    assert!(matches!(
        ClockedMotorSpan::try_new(Arc::clone(&signal), 0.0, 1.0, 20.0, 21.0, boundary, 1.0,),
        Err(ContinuousError::InvalidSpan { .. })
    ));
    assert!(matches!(
        ClockedMotorSpan::try_new(signal, 0.0, 1.0, 20.0, 21.0, boundary - 4096.0, 4096.0,),
        Err(ContinuousError::InvalidSpan { .. })
    ));
}

#[test]
fn split_views_are_at_most_25ms_and_share_one_signal() {
    let signal = motor_span(
        ContinuousAxis::Hold {
            position: 4.0,
            t_start: 0.0,
            t_end: 0.061,
        },
        0.0,
        0.061,
    );
    let clocked = ClockedMotorSpan::try_new(
        Arc::clone(&signal),
        0.0,
        0.061,
        3.0,
        3.061,
        500.25,
        100_000.0,
    )
    .unwrap();
    let views = clocked.split_max_duration().unwrap();
    assert_eq!(views.len(), 3);
    assert!(views
        .iter()
        .all(|view| view.stream_t_end - view.stream_t_start <= MAX_SPAN_SECS));
    assert!(views.iter().all(|view| Arc::ptr_eq(&view.signal, &signal)));
    assert_eq!(views.first().unwrap().stream_t_start, 0.0);
    assert_eq!(views.last().unwrap().stream_t_end, 0.061);
}

#[test]
fn split_views_absorb_a_final_sub_clock_tail() {
    let duration = 2.0 * MAX_SPAN_SECS + 1e-10;
    let signal = motor_span(
        ContinuousAxis::Hold {
            position: 4.0,
            t_start: 0.0,
            t_end: duration,
        },
        0.0,
        duration,
    );
    let clocked = ClockedMotorSpan::try_new(
        signal,
        0.0,
        duration,
        3.0,
        3.0 + duration,
        500.25,
        1_000_000.0,
    )
    .unwrap();
    let views = clocked.split_max_duration().unwrap();
    assert_eq!(views.len(), 2);
    assert_eq!(views.last().unwrap().stream_t_end, duration);
    assert_eq!(views.last().unwrap().end_clock, clocked.end_clock);
}

#[test]
fn nudges_have_exact_endpoints_and_required_breakpoints() {
    let accelerated = NudgeProfile::try_new(10.0, 4.0, 2.0, 5.0).unwrap();
    assert_eq!(accelerated.breakpoints(), &[5.0, 7.0, 7.5, 9.5]);
    assert_eq!(accelerated.eval(5.0).position, 0.0);
    assert_eq!(
        accelerated.eval(9.5),
        super::profiles::ProfileSample {
            position: 10.0,
            velocity: 0.0,
            acceleration: 0.0
        }
    );
    close(accelerated.eval(7.25).position, 5.0);
    close(accelerated.eval(7.25).velocity, 4.0);

    let box_profile = NudgeProfile::try_new(-6.0, 3.0, 0.0, 2.0).unwrap();
    assert_eq!(box_profile.breakpoints(), &[2.0, 4.0]);
    assert_eq!(box_profile.eval(2.0).velocity, 0.0);
    assert_eq!(box_profile.eval(3.0).velocity, -3.0);
    assert_eq!(
        box_profile.eval(4.0),
        super::profiles::ProfileSample {
            position: -6.0,
            velocity: 0.0,
            acceleration: 0.0
        }
    );
}

#[test]
fn profile_result_types_are_nameable_from_the_continuous_facade() {
    let profile: Result<NudgeProfile, crate::continuous::ProfileError> =
        NudgeProfile::try_new(1.0, 1.0, 0.0, 0.0);
    let profile = profile.unwrap();
    let sample: crate::continuous::ProfileSample = profile.eval(0.5);
    assert_eq!(sample.position, 0.5);
}

#[test]
fn buzz_has_ramp_knees_position_continuity_and_exact_zero_endpoints() {
    let buzz = BuzzProfile::try_new(0.5, 5.0, 9.0, 4.0, 0.5, 2.0).unwrap();
    assert_eq!(buzz.breakpoints(), &[2.0, 2.5, 5.5, 6.0]);
    let zero = super::profiles::ProfileSample {
        position: 0.0,
        velocity: 0.0,
        acceleration: 0.0,
    };
    assert_eq!(buzz.eval(2.0), zero);
    assert_eq!(buzz.eval(6.0), zero);
    for knee in [2.5, 5.5] {
        let at = buzz.eval(knee).position;
        assert!((buzz.eval(knee - 1e-9).position - at).abs() < 1e-6);
        assert!((buzz.eval(knee + 1e-9).position - at).abs() < 1e-6);
    }
}

#[test]
fn zero_ramp_buzz_is_a_finite_rectangular_envelope() {
    let amplitude = 0.5;
    let frequency = 5.0;
    let duration = 4.0;
    let buzz = BuzzProfile::try_new(amplitude, frequency, frequency, duration, 0.0, 2.0).unwrap();
    assert_eq!(buzz.breakpoints(), &[2.0, 6.0]);
    let omega = 2.0 * PI * frequency;
    let local = 0.05;
    let sample = buzz.eval(2.0 + local);
    close(sample.position, amplitude * libm::sin(omega * local));
    close(
        sample.velocity,
        amplitude * omega * libm::cos(omega * local),
    );
    close(
        sample.acceleration,
        -amplitude * omega * omega * libm::sin(omega * local),
    );
    let (velocity_min, velocity_max) = buzz.velocity_bounds();
    let (acceleration_min, acceleration_max) = buzz.acceleration_bounds();
    close_relative(velocity_max, amplitude * omega, 1e-6);
    close_relative(velocity_min, -amplitude * omega, 1e-6);
    close_relative(acceleration_max, amplitude * omega * omega, 1e-6);
    close_relative(acceleration_min, -amplitude * omega * omega, 1e-6);
}

#[test]
fn non_finite_and_outside_span_fail_loudly() {
    let bad = ContinuousAxis::Hold {
        position: f64::NAN,
        t_start: 0.0,
        t_end: 1.0,
    };
    assert!(matches!(
        bad.eval_pva(0.5),
        Err(ContinuousError::NonFinite { .. })
    ));
    assert!(matches!(
        bad.eval_pva(f64::NAN),
        Err(ContinuousError::NonFinite { .. })
    ));

    let segment = ContinuousSegment {
        axes: Arc::from([bad]),
        followers: Arc::from([]),
        spatial_path: false,
        t_start: 0.0,
        t_end: 1.0,
        motor_mask: 1,
        source_line: 41,
        rest_at_end: true,
    };
    assert!(matches!(
        segment.eval_axis(0, 0.5),
        Err(ContinuousError::NonFiniteEvaluation { source_axis: 0, .. })
    ));
    assert!(matches!(
        segment.eval_axis(0, 2.0),
        Err(ContinuousError::TimeOutsideSpan { .. })
    ));
    assert!(NudgeProfile::try_new(f64::NAN, 1.0, 1.0, 0.0).is_err());
    assert!(BuzzProfile::try_new(1.0, 1.0, f64::INFINITY, 1.0, 0.1, 0.0).is_err());
}

#[test]
fn motor_breakpoints_are_precomputed_sorted_and_deduplicated() {
    let nudge = ContinuousAxis::Nudge(NudgeProfile::try_new(10.0, 4.0, 2.0, 5.0).unwrap());
    let hold = ContinuousAxis::Hold {
        position: 0.0,
        t_start: 5.0,
        t_end: 9.5,
    };
    let span = MotorSpan::try_new(
        Arc::from([independent(nudge, 0), independent(hold, 1)]),
        5.0,
        9.5,
        1,
        41,
        false,
    )
    .unwrap();
    assert_eq!(span.breakpoints.as_ref(), &[5.0, 7.0, 7.5, 9.5]);
}

const RELATIVE_BASE_MM: f64 = 24.0;
const RELATIVE_T0: f64 = 12.5;
const RELATIVE_VELOCITY_MM_S: f64 = 50.0;
const RELATIVE_ACCEL_MM_S2: f64 = 2e11;

struct TinyRelative {
    curve: Arc<ScalarNurbs>,
    t_end: f64,
    dt: f64,
}

fn tiny_relative_curve() -> TinyRelative {
    let t_end = RELATIVE_T0 + 1e-10;
    let dt = t_end - RELATIVE_T0;
    let first = RELATIVE_VELOCITY_MM_S * dt / 2.0;
    let second = 2.0 * first + RELATIVE_ACCEL_MM_S2 * dt * dt / 2.0;
    let curve = ScalarNurbs::try_new(
        2,
        vec![RELATIVE_T0, RELATIVE_T0, RELATIVE_T0, t_end, t_end, t_end],
        vec![0.0, first, second],
    )
    .unwrap();
    TinyRelative {
        curve: Arc::new(curve),
        t_end,
        dt,
    }
}

fn relative_offset(fixture: &TinyRelative, t: f64) -> f64 {
    let s = (t - RELATIVE_T0) / fixture.dt;
    RELATIVE_VELOCITY_MM_S * s * fixture.dt + 0.5 * RELATIVE_ACCEL_MM_S2 * (s * fixture.dt).powi(2)
}

fn relative_velocity(t: f64) -> f64 {
    RELATIVE_VELOCITY_MM_S + RELATIVE_ACCEL_MM_S2 * (t - RELATIVE_T0).max(0.0)
}

fn close_relative(actual: f64, expected: f64, tolerance: f64) {
    let scale = expected.abs().max(f64::MIN_POSITIVE);
    assert!(
        (actual - expected).abs() <= tolerance * scale,
        "expected {expected}, got {actual}"
    );
}

#[test]
fn relative_spline_adds_base_to_position_only() {
    let fixture = tiny_relative_curve();
    let axis = ContinuousAxis::RelativeSpline {
        base_position: RELATIVE_BASE_MM,
        curve: Arc::clone(&fixture.curve),
    };
    assert_eq!(axis.domain(), (RELATIVE_T0, fixture.t_end));

    for step in 0..=4 {
        let t = RELATIVE_T0 + fixture.dt * f64::from(step) / 4.0;
        let value = axis.eval_pva(t).unwrap();
        assert!(value.position.is_finite());
        assert!(value.velocity.is_finite());
        assert!(value.acceleration.is_finite());
        let offset = relative_offset(&fixture, t);
        close_relative(value.position, RELATIVE_BASE_MM + offset, 1e-15);
        close_relative(value.velocity, relative_velocity(t), 1e-12);
        close_relative(value.acceleration, RELATIVE_ACCEL_MM_S2, 1e-12);
    }

    let zero_base = ContinuousAxis::RelativeSpline {
        base_position: 0.0,
        curve: Arc::clone(&fixture.curve),
    };
    let mid = RELATIVE_T0 + fixture.dt / 2.0;
    let based = axis.eval_pva(mid).unwrap();
    let unbased = zero_base.eval_pva(mid).unwrap();
    assert_eq!(based.velocity, unbased.velocity);
    assert_eq!(based.acceleration, unbased.acceleration);
    assert_eq!(based.position, unbased.position + RELATIVE_BASE_MM);
    assert_eq!(axis.position(mid).unwrap(), based.position);
}

#[test]
fn relative_spline_keeps_derivatives_an_absolute_spline_loses() {
    let fixture = tiny_relative_curve();
    let absolute = ScalarNurbs::try_new(
        2,
        fixture.curve.knots().to_vec(),
        fixture
            .curve
            .control_points()
            .iter()
            .map(|value| RELATIVE_BASE_MM + value)
            .collect(),
    )
    .unwrap();
    let relative = ContinuousAxis::RelativeSpline {
        base_position: RELATIVE_BASE_MM,
        curve: Arc::clone(&fixture.curve),
    };
    let mid = RELATIVE_T0 + fixture.dt / 2.0;
    let relative_value = relative.eval_pva(mid).unwrap();
    let absolute_value = ContinuousAxis::Spline(Arc::new(absolute))
        .eval_pva(mid)
        .unwrap();

    close_relative(relative_value.acceleration, RELATIVE_ACCEL_MM_S2, 1e-12);
    let absolute_error =
        (absolute_value.acceleration - RELATIVE_ACCEL_MM_S2).abs() / RELATIVE_ACCEL_MM_S2;
    assert!(
        absolute_error > 1e-6,
        "absolute encoding unexpectedly kept acceleration: error {absolute_error}"
    );
}

#[test]
fn relative_spline_bounds_ignore_the_base() {
    let fixture = tiny_relative_curve();
    let based = ContinuousAxis::RelativeSpline {
        base_position: RELATIVE_BASE_MM,
        curve: Arc::clone(&fixture.curve),
    };
    let unbased = ContinuousAxis::RelativeSpline {
        base_position: 0.0,
        curve: Arc::clone(&fixture.curve),
    };
    let bounds = based.pva_bounds(RELATIVE_T0, fixture.t_end).unwrap();
    assert_eq!(
        bounds,
        unbased.pva_bounds(RELATIVE_T0, fixture.t_end).unwrap()
    );
    close_relative(bounds.velocity_min, RELATIVE_VELOCITY_MM_S, 1e-9);
    close_relative(
        bounds.velocity_max,
        RELATIVE_VELOCITY_MM_S + RELATIVE_ACCEL_MM_S2 * fixture.dt,
        1e-9,
    );
    close_relative(bounds.acceleration_abs_max, RELATIVE_ACCEL_MM_S2, 1e-9);
}

#[test]
fn coalesced_relative_group_applies_the_base_once_per_scale() {
    let fixture = tiny_relative_curve();
    let span = MotorSpan::try_new(
        Arc::from([MotorGroup::RelativeSpline {
            curve: Arc::clone(&fixture.curve),
            base_position: RELATIVE_BASE_MM,
            summed_scale: 3.0,
        }]),
        RELATIVE_T0,
        fixture.t_end,
        1,
        41,
        false,
    )
    .unwrap();
    assert_eq!(span.breakpoints.as_ref(), &[RELATIVE_T0, fixture.t_end]);

    let mid = RELATIVE_T0 + fixture.dt / 2.0;
    let value = span.eval_pva(mid).unwrap();
    close_relative(
        value.position,
        3.0 * (RELATIVE_BASE_MM + relative_offset(&fixture, mid)),
        1e-12,
    );
    close_relative(value.velocity, 3.0 * relative_velocity(mid), 1e-9);
    close_relative(value.acceleration, 3.0 * RELATIVE_ACCEL_MM_S2, 1e-9);

    let bounds = span.pva_bounds(RELATIVE_T0, fixture.t_end).unwrap();
    close_relative(bounds.velocity_min, 3.0 * RELATIVE_VELOCITY_MM_S, 1e-9);
    close_relative(
        bounds.acceleration_abs_max,
        3.0 * RELATIVE_ACCEL_MM_S2,
        1e-9,
    );
}

#[test]
fn distinct_relative_bases_stay_independent_and_sum() {
    let fixture = tiny_relative_curve();
    let low = ContinuousAxis::RelativeSpline {
        base_position: RELATIVE_BASE_MM,
        curve: Arc::clone(&fixture.curve),
    };
    let high = ContinuousAxis::RelativeSpline {
        base_position: RELATIVE_BASE_MM + 5.0,
        curve: Arc::clone(&fixture.curve),
    };
    let span = MotorSpan::try_new(
        Arc::from([independent(low, 0), independent(high, 1)]),
        RELATIVE_T0,
        fixture.t_end,
        1,
        41,
        false,
    )
    .unwrap();
    let mid = RELATIVE_T0 + fixture.dt / 2.0;
    let value = span.eval_pva(mid).unwrap();
    close_relative(
        value.position,
        2.0 * RELATIVE_BASE_MM + 5.0 + 2.0 * relative_offset(&fixture, mid),
        1e-12,
    );
    close_relative(value.velocity, 2.0 * relative_velocity(mid), 1e-9);
    close_relative(value.acceleration, 2.0 * RELATIVE_ACCEL_MM_S2, 1e-9);
}

#[test]
fn relative_spline_rejects_non_finite_base_and_controls() {
    let fixture = tiny_relative_curve();
    let bad_base = ContinuousAxis::RelativeSpline {
        base_position: f64::NAN,
        curve: Arc::clone(&fixture.curve),
    };
    assert!(matches!(
        bad_base.eval_pva(RELATIVE_T0),
        Err(ContinuousError::NonFinite { .. })
    ));
    assert!(matches!(
        MotorSpan::try_new(
            Arc::from([independent(bad_base, 0)]),
            RELATIVE_T0,
            fixture.t_end,
            1,
            41,
            false,
        ),
        Err(ContinuousError::InvalidSpan { .. })
    ));
    assert!(matches!(
        MotorSpan::try_new(
            Arc::from([MotorGroup::RelativeSpline {
                curve: Arc::clone(&fixture.curve),
                base_position: f64::INFINITY,
                summed_scale: 1.0,
            }]),
            RELATIVE_T0,
            fixture.t_end,
            1,
            41,
            false,
        ),
        Err(ContinuousError::InvalidSpan { .. })
    ));
    assert!(matches!(
        ContinuousAxis::RelativeSpline {
            base_position: RELATIVE_BASE_MM,
            curve: Arc::clone(&fixture.curve),
        }
        .eval_pva(fixture.t_end + 1.0),
        Err(ContinuousError::TimeOutsideSpan { .. })
    ));
}
