use super::*;
use crate::{Item, Recovery, Segment, TelemetryEvent};

fn collect(text: &str) -> Vec<Item> {
    let mut p = GeometryPipeline::new(FitterParams::default(), vec![]);
    let mut sink = |_: crate::TelemetryEvent| {};
    p.process(text, &mut sink).collect()
}

#[test]
fn empty_input_yields_no_items() {
    let mut p = GeometryPipeline::new(FitterParams::default(), vec![]);
    let mut sink = |_e: crate::TelemetryEvent| {};
    let items: Vec<_> = p.process("", &mut sink).collect();
    assert!(items.is_empty());
}

#[test]
fn whitespace_input_yields_no_items() {
    let mut p = GeometryPipeline::new(FitterParams::default(), vec![]);
    let mut sink = |_e: crate::TelemetryEvent| {};
    let items: Vec<_> = p.process("\n\n   \n", &mut sink).collect();
    assert!(items.is_empty());
}

#[test]
fn layer_change_marker_fires_telemetry() {
    let mut events = vec![];
    let mut p = GeometryPipeline::new(FitterParams::default(), vec![]);
    let _items: Vec<_> = {
        let mut sink = |e: TelemetryEvent| events.push(e);
        p.process(";LAYER:5\n", &mut sink).collect()
    };
    assert!(matches!(
        events.as_slice(),
        [TelemetryEvent::LayerChange {
            layer: Some(5),
            line_no: 1
        }]
    ));
}

#[test]
fn tool_change_fires_telemetry() {
    let mut events = vec![];
    let mut p = GeometryPipeline::new(FitterParams::default(), vec![]);
    let _items: Vec<_> = {
        let mut sink = |e: TelemetryEvent| events.push(e);
        p.process("T1\n", &mut sink).collect()
    };
    assert!(matches!(
        events.as_slice(),
        [TelemetryEvent::ToolChange {
            tool: 1,
            line_no: 1
        }]
    ));
}

#[test]
fn g5_emits_cubic_segment() {
    let items = collect("G5 X10 Y0 I3 J3 P-3 Q3\n");
    let cubic_seg = items.iter().find_map(|it| match it {
        Item::Segment(Segment::Cubic(c)) => Some(c),
        _ => None,
    });
    let c = cubic_seg.expect("expected a Segment::Cubic");
    assert_eq!(c.xyz.degree(), 3);
    let cps = c.xyz.control_points();
    assert_eq!(cps.len(), 4);
    let approx = |a: f64, b: f64| (a - b).abs() < 1e-12;
    assert!(approx(cps[0][0], 0.0) && approx(cps[0][1], 0.0));
    assert!(approx(cps[1][0], 3.0) && approx(cps[1][1], 3.0));
    assert!(approx(cps[2][0], 7.0) && approx(cps[2][1], 3.0));
    assert!(approx(cps[3][0], 10.0) && approx(cps[3][1], 0.0));
    let knots = c.xyz.knots();
    assert_eq!(knots, &[0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]);
    assert!(c.followers.is_empty());
    assert!(
        !items
            .iter()
            .any(|it| matches!(it, Item::Segment(Segment::Junction(_)))),
        "G5 must not emit a junction here, got {items:#?}"
    );
}

#[test]
fn g5_1_emits_cubic_via_degree_elevation() {
    let items = collect("G5.1 X10 Y0 I3 J3\n");
    let cubic_seg = items.iter().find_map(|it| match it {
        Item::Segment(Segment::Cubic(c)) => Some(c),
        _ => None,
    });
    let c = cubic_seg.expect("expected a Segment::Cubic from G5.1");
    assert_eq!(c.xyz.degree(), 3);
    assert_eq!(c.xyz.control_points().len(), 4);
    let knots = c.xyz.knots();
    assert_eq!(knots, &[0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]);
    let cps = c.xyz.control_points();
    let approx = |a: f64, b: f64| (a - b).abs() < 1e-12;
    assert!(approx(cps[0][0], 0.0) && approx(cps[0][1], 0.0));
    assert!(approx(cps[3][0], 10.0) && approx(cps[3][1], 0.0));
    assert!(c.followers.is_empty());
}

#[test]
fn g5_1_outside_xy_plane_yields_recovered() {
    let mut events = vec![];
    let mut p = GeometryPipeline::new(FitterParams::default(), vec![]);
    let items: Vec<_> = {
        let mut sink = |e: TelemetryEvent| events.push(e);
        p.process("G18\nG5.1 X10 Z1 I3 J3\n", &mut sink).collect()
    };
    let recovered = items.iter().find_map(|it| match it {
        Item::Recovered(
            _,
            Recovery::G5PlaneMismatch {
                line_no: 2,
                active_plane_g_code: 18,
            },
        ) => Some(()),
        _ => None,
    });
    assert!(
        recovered.is_some(),
        "expected G5PlaneMismatch, got {items:#?}"
    );
    assert!(matches!(
        events.last(),
        Some(TelemetryEvent::Recovery(Recovery::G5PlaneMismatch {
            line_no: 2,
            active_plane_g_code: 18
        }))
    ));
}

fn collect_with_e_follower(text: &str) -> Vec<Item> {
    let mut p = GeometryPipeline::new(
        FitterParams::default(),
        vec![crate::FollowerWord {
            letter: b'E',
            axis_index: 3,
        }],
    );
    let mut sink = |_: crate::TelemetryEvent| {};
    p.process(text, &mut sink).collect()
}

fn single_cubic(items: &[Item]) -> &crate::CubicSegment {
    let cubics: Vec<_> = items
        .iter()
        .filter_map(|it| match it {
            Item::Segment(Segment::Cubic(c)) => Some(c),
            _ => None,
        })
        .collect();
    assert_eq!(cubics.len(), 1, "expected exactly one cubic, got {items:#?}");
    cubics[0]
}

#[test]
fn vase_mode_helix_classifies_with_3d_ratio() {
    let items = collect_with_e_follower("G5 X10 Y0 Z0.3 I3 J0 P-3 Q0 E0.5 F3000\n");
    let seg = single_cubic(&items);
    assert_eq!(seg.followers.len(), 1);
    assert_eq!(seg.followers[0].axis_index, 3);
    let path_len = nurbs::arc_length::path_arc_length(&seg.xyz);
    assert!((seg.followers[0].ratio - 0.5 / path_len).abs() < 1e-12);
}

#[test]
fn z_hop_with_follower_classifies() {
    let items = collect_with_e_follower("G5 X0 Y0 Z2.0 I0.1 J0 P-0.1 Q0 E-3.2 F3000\n");
    let seg = single_cubic(&items);
    let path_len = nurbs::arc_length::path_arc_length(&seg.xyz);
    assert_eq!(seg.followers.len(), 1);
    assert_eq!(seg.followers[0].axis_index, 3);
    assert!((seg.followers[0].ratio - (-3.2) / path_len).abs() < 1e-12);
}

#[test]
fn follower_only_move_is_fatal() {
    let items = collect_with_e_follower(
        "G5 X10 Y0 I3 J0 P-3 Q0 F3000\nG5 X10 Y0 I0 J0 P0 Q0 E5 F3000\n",
    );
    assert!(
        items
            .iter()
            .any(|it| matches!(it, Item::Fatal(Fatal::FollowerOnlyMoveUnsupported { .. }))),
        "expected FollowerOnlyMoveUnsupported, got {items:#?}"
    );
}

#[test]
fn absolute_word_ledger_survives_g92() {
    let items = collect_with_e_follower(
        "G5 X10 Y0 I3 J0 P-3 Q0 E10 F3000\nG92 E0\nG5 X20 Y0 I3 J0 P-3 Q0 E10 F3000\n",
    );
    let cubics: Vec<_> = items
        .iter()
        .filter_map(|it| match it {
            Item::Segment(Segment::Cubic(c)) => Some(c),
            _ => None,
        })
        .collect();
    assert_eq!(cubics.len(), 2, "expected two cubics, got {items:#?}");
    for c in &cubics {
        assert_eq!(c.followers.len(), 1);
        assert!(c.followers[0].ratio > 0.0);
    }
    assert!((cubics[0].followers[0].ratio - cubics[1].followers[0].ratio).abs() < 1e-12);
}
