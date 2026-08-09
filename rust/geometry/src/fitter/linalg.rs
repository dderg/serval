pub(super) fn solve3(m: [[f64; 3]; 3], b: [f64; 3]) -> Option<[f64; 3]> {
    let det = det3(m);
    if det.abs() < 1e-18 {
        return None;
    }
    let mut out = [0.0_f64; 3];
    for col in 0..3 {
        let mut mc = m;
        for row in 0..3 {
            mc[row][col] = b[row];
        }
        out[col] = det3(mc) / det;
    }
    Some(out)
}

fn det3(m: [[f64; 3]; 3]) -> f64 {
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
}
