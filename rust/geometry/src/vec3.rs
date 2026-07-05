pub(crate) fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub(crate) fn norm_sq(a: [f64; 3]) -> f64 {
    dot(a, a)
}

pub(crate) fn norm(a: [f64; 3]) -> f64 {
    norm_sq(a).sqrt()
}

pub(crate) fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

pub(crate) fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub(crate) fn scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

pub(crate) fn madd(p: [f64; 3], s: f64, d: [f64; 3]) -> [f64; 3] {
    [p[0] + s * d[0], p[1] + s * d[1], p[2] + s * d[2]]
}

pub(crate) fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    norm(sub(a, b))
}

pub(crate) fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

pub(crate) fn normalize(a: [f64; 3]) -> [f64; 3] {
    let n = norm(a);
    [a[0] / n, a[1] / n, a[2] / n]
}

pub(crate) fn signed_angle(from: [f64; 3], to: [f64; 3], plane_normal: [f64; 3]) -> f64 {
    libm::atan2(dot(cross(from, to), plane_normal), dot(from, to))
}

pub(crate) fn turn_normal(t_in: [f64; 3], t_out: [f64; 3]) -> Option<[f64; 3]> {
    let d = dot(t_out, t_in);
    let perp = sub(t_out, scale(t_in, d));
    let n = norm(perp);
    if n < crate::fitter::TURN_NORMAL_EPS {
        None
    } else {
        Some(scale(perp, 1.0 / n))
    }
}
