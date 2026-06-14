/// Straight-line move as a degenerate cubic Bézier — control points collinear
/// at the 1/3 and 2/3 marks. Relocated from `compat::collinear` so the live
/// engine no longer links the offline preprocessor.
#[must_use]
pub fn to_collinear_bezier(start: [f64; 3], end: [f64; 3]) -> [[f64; 3]; 4] {
    let d = [end[0] - start[0], end[1] - start[1], end[2] - start[2]];
    let p1 = [
        start[0] + d[0] / 3.0,
        start[1] + d[1] / 3.0,
        start[2] + d[2] / 3.0,
    ];
    let p2 = [
        start[0] + 2.0 * d[0] / 3.0,
        start[1] + 2.0 * d[1] / 3.0,
        start[2] + 2.0 * d[2] / 3.0,
    ];
    [start, p1, p2, end]
}

#[cfg(test)]
mod tests;
