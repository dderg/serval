use super::*;
use nurbs::VectorNurbs;

#[test]
fn cubic_segment_constructs() {
    let xyz = VectorNurbs::<3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
        ],
    )
    .expect("valid cubic");
    let cs = CubicSegment::try_new(
        xyz,
        vec![],
        100.0,
        SourceRange {
            start_line: 1,
            end_line: 1,
        },
    )
    .expect("valid travel");
    assert!(cs.virtual_path_mm.is_none());
}
