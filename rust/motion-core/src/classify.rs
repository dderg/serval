use geometry::segment::SourceRange;

pub fn build_move(
    start: [f64; 3],
    delta: [f64; 3],
    extruder_axis: usize,
    e_delta: f64,
    limits: geometry::VelocityLimits,
    feedrate_mm_s: f64,
    line_no: u32,
) -> Result<geometry::Move, geometry::FrontendError> {
    let end = [
        start[0] + delta[0],
        start[1] + delta[1],
        start[2] + delta[2],
    ];
    let ctx = geometry::MoveContext {
        extruder_axis,
        feedrate_mm_s,
        limits,
        source: SourceRange {
            start_line: line_no,
            end_line: line_no,
        },
    };
    geometry::line_move(start, end, e_delta, ctx)
}
