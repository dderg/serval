use geometry::curve::to_collinear_bezier;
use geometry::segment::{CubicSegment, FollowerDemand, SourceRange};
use nurbs::VectorNurbs;

const DISPLACEMENT_EPSILON: f64 = 1e-9;

#[derive(Debug)]
pub struct ClassifiedMove {
    pub segment: CubicSegment,
    pub distance_mm: f64,
}

impl ClassifiedMove {
    #[must_use]
    pub fn nominal_duration(&self) -> f64 {
        if self.segment.feedrate_mm_s <= 0.0 {
            return 0.0;
        }
        self.distance_mm / self.segment.feedrate_mm_s
    }
}

pub fn classify_and_build(
    start: [f64; 3],
    dx: f64,
    dy: f64,
    dz: f64,
    followers: &[(usize, f64)],
    feedrate_mm_s: f64,
) -> Result<ClassifiedMove, ClassifyError> {
    let end = [start[0] + dx, start[1] + dy, start[2] + dz];
    let spatial_distance = (dx * dx + dy * dy + dz * dz).sqrt();
    let has_spatial = dx.abs() > DISPLACEMENT_EPSILON
        || dy.abs() > DISPLACEMENT_EPSILON
        || dz.abs() > DISPLACEMENT_EPSILON;
    let active_followers: Vec<(usize, f64)> = followers
        .iter()
        .copied()
        .filter(|&(_, delta)| delta.abs() > DISPLACEMENT_EPSILON)
        .collect();

    if !has_spatial && active_followers.is_empty() {
        return Err(ClassifyError::ZeroDisplacement);
    }

    let source = SourceRange {
        start_line: 0,
        end_line: 0,
    };

    if has_spatial {
        let xyz = build_cubic(to_collinear_bezier(start, end))?;
        let demands = active_followers
            .iter()
            .map(|&(axis_index, delta)| FollowerDemand {
                axis_index,
                ratio: delta / spatial_distance,
            })
            .collect();
        let segment = CubicSegment::try_new(xyz, demands, feedrate_mm_s, source, None)
            .map_err(|e| ClassifyError::SegmentConstruction(format!("{e:?}")))?;
        return Ok(ClassifiedMove {
            segment,
            distance_mm: spatial_distance,
        });
    }

    let virtual_path_mm = active_followers
        .iter()
        .map(|(_, delta)| delta.abs())
        .fold(0.0_f64, f64::max);
    let xyz = build_cubic(to_collinear_bezier(start, start))?;
    let demands = active_followers
        .iter()
        .map(|&(axis_index, delta)| FollowerDemand {
            axis_index,
            ratio: delta / virtual_path_mm,
        })
        .collect();
    let segment =
        CubicSegment::try_new_virtual(xyz, demands, feedrate_mm_s, source, virtual_path_mm)
            .map_err(|e| ClassifyError::SegmentConstruction(format!("{e:?}")))?;
    Ok(ClassifiedMove {
        segment,
        distance_mm: virtual_path_mm,
    })
}

fn build_cubic(cps: [[f64; 3]; 4]) -> Result<VectorNurbs<f64, 3>, ClassifyError> {
    VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        cps.to_vec(),
    )
    .map_err(|e| ClassifyError::NurbsConstruction(format!("{e:?}")))
}

#[derive(Debug)]
pub enum ClassifyError {
    ZeroDisplacement,
    NurbsConstruction(String),
    SegmentConstruction(String),
}

impl std::fmt::Display for ClassifyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ZeroDisplacement => write!(f, "zero displacement move"),
            Self::NurbsConstruction(e) => write!(f, "NURBS construction: {e}"),
            Self::SegmentConstruction(e) => write!(f, "segment construction: {e}"),
        }
    }
}

impl std::error::Error for ClassifyError {}

#[cfg(test)]
mod tests;
