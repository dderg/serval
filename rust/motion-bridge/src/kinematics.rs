use crate::dispatch::KINEMATICS_COREXY;
use runtime::segment::KinematicTag;

pub const SPATIAL_AXES: usize = 3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KinematicsKind {
    CoreXy,
    Cartesian,
}

#[derive(Debug, Clone, Copy)]
pub struct KinematicsModule {
    kind: KinematicsKind,
    axis_to_motor: [[f64; SPATIAL_AXES]; SPATIAL_AXES],
    motor_to_axis: [[f64; SPATIAL_AXES]; SPATIAL_AXES],
}

#[derive(Debug, thiserror::Error)]
#[error("unknown kinematics tag {0}; known: 0=corexy, 1=cartesian")]
pub struct UnknownKinematicsTag(pub u8);

const COREXY_AXIS_TO_MOTOR: [[f64; SPATIAL_AXES]; SPATIAL_AXES] =
    [[1.0, 1.0, 0.0], [1.0, -1.0, 0.0], [0.0, 0.0, 1.0]];
const COREXY_MOTOR_TO_AXIS: [[f64; SPATIAL_AXES]; SPATIAL_AXES] =
    [[0.5, 0.5, 0.0], [0.5, -0.5, 0.0], [0.0, 0.0, 1.0]];
const IDENTITY: [[f64; SPATIAL_AXES]; SPATIAL_AXES] =
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

fn matrix_vector(
    matrix: &[[f64; SPATIAL_AXES]; SPATIAL_AXES],
    vector: [f64; SPATIAL_AXES],
) -> [f64; SPATIAL_AXES] {
    let mut out = [0.0; SPATIAL_AXES];
    for (row, slot) in matrix.iter().zip(out.iter_mut()) {
        *slot = row
            .iter()
            .zip(vector.iter())
            .map(|(weight, value)| weight * value)
            .sum();
    }
    out
}

impl KinematicsModule {
    pub fn from_tag(tag: u8) -> Result<Self, UnknownKinematicsTag> {
        if tag == KinematicTag::CoreXy as u8 {
            Ok(Self {
                kind: KinematicsKind::CoreXy,
                axis_to_motor: COREXY_AXIS_TO_MOTOR,
                motor_to_axis: COREXY_MOTOR_TO_AXIS,
            })
        } else if tag == KinematicTag::Cartesian as u8 {
            Ok(Self {
                kind: KinematicsKind::Cartesian,
                axis_to_motor: IDENTITY,
                motor_to_axis: IDENTITY,
            })
        } else {
            Err(UnknownKinematicsTag(tag))
        }
    }

    pub fn kind(&self) -> KinematicsKind {
        self.kind
    }

    pub fn tag(&self) -> u8 {
        match self.kind {
            KinematicsKind::CoreXy => KinematicTag::CoreXy as u8,
            KinematicsKind::Cartesian => KinematicTag::Cartesian as u8,
        }
    }

    pub fn lane_weights(&self, lane: usize) -> [f64; SPATIAL_AXES] {
        self.axis_to_motor[lane]
    }

    pub fn lane_is_identity(&self, lane: usize) -> bool {
        let mut unit = [0.0; SPATIAL_AXES];
        unit[lane] = 1.0;
        self.axis_to_motor[lane] == unit
    }

    pub fn forward(&self, axes: [f64; SPATIAL_AXES]) -> [f64; SPATIAL_AXES] {
        matrix_vector(&self.axis_to_motor, axes)
    }

    pub fn inverse(&self, motors: [f64; SPATIAL_AXES]) -> [f64; SPATIAL_AXES] {
        matrix_vector(&self.motor_to_axis, motors)
    }
}

#[inline]
pub fn forward_corexy(x: f64, y: f64) -> (f64, f64) {
    (x + y, x - y)
}

#[inline]
pub fn inverse_corexy(motor_a: f64, motor_b: f64) -> (f64, f64) {
    (0.5 * (motor_a + motor_b), 0.5 * (motor_a - motor_b))
}

pub fn forward(tag: u8, xyz: [f64; 3]) -> [f64; 4] {
    if tag == KINEMATICS_COREXY {
        let (a, b) = forward_corexy(xyz[0], xyz[1]);
        [a, b, xyz[2], 0.0]
    } else {
        [xyz[0], xyz[1], xyz[2], 0.0]
    }
}

pub fn inverse(tag: u8, motor: [f64; 4]) -> [f64; 3] {
    if tag == KINEMATICS_COREXY {
        let (x, y) = inverse_corexy(motor[0], motor[1]);
        [x, y, motor[2]]
    } else {
        [motor[0], motor[1], motor[2]]
    }
}

#[cfg(test)]
mod tests;
