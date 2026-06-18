use std::f64::consts::TAU;

use super::{DISPLACEMENT_EPSILON, FrontendError};
use crate::path::Arc;

const ARC_RADIUS_ABS_TOL_MM: f64 = 1e-3;
const ARC_RADIUS_REL_TOL: f64 = 1e-6;

const PLANE_U: [f64; 3] = [1.0, 0.0, 0.0];
const PLANE_V: [f64; 3] = [0.0, 1.0, 0.0];

pub(super) fn build_arc(
    start: [f64; 3],
    end: [f64; 3],
    i: f64,
    j: f64,
    ccw: bool,
    line_no: u32,
) -> Result<Arc, FrontendError> {
    if (end[2] - start[2]).abs() > DISPLACEMENT_EPSILON {
        return Err(FrontendError::HelicalArc { line_no });
    }

    let center = [start[0] + i, start[1] + j];
    let radius = i.hypot(j);
    let end_radius = (end[0] - center[0]).hypot(end[1] - center[1]);
    if (end_radius - radius).abs() > radius_tolerance(radius) {
        return Err(FrontendError::ArcRadiusMismatch {
            line_no,
            radius,
            end_radius,
        });
    }

    let start_angle = (start[1] - center[1]).atan2(start[0] - center[0]);
    let end_angle = (end[1] - center[1]).atan2(end[0] - center[0]);
    let sweep = normalize_sweep(start_angle, end_angle, ccw);

    let origin = [center[0], center[1], start[2]];
    Arc::try_new(origin, PLANE_U, PLANE_V, radius, start_angle, sweep)
        .map_err(|source| FrontendError::Segment { line_no, source })
}

fn radius_tolerance(radius: f64) -> f64 {
    ARC_RADIUS_REL_TOL.mul_add(radius, ARC_RADIUS_ABS_TOL_MM)
}

fn normalize_sweep(start_angle: f64, end_angle: f64, ccw: bool) -> f64 {
    let mut sweep = end_angle - start_angle;
    if ccw {
        while sweep <= 0.0 {
            sweep += TAU;
        }
    } else {
        while sweep >= 0.0 {
            sweep -= TAU;
        }
    }
    sweep
}
