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

#[cfg(test)]
mod tests;
