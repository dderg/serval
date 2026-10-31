use crate::{ConstructError, scalar::validate};

#[derive(Debug, Clone, PartialEq)]
pub struct VectorNurbs<const N: usize> {
    degree: u8,
    knots: crate::knot::KnotVector,
    control_points: Vec<[f64; N]>,
}

impl<const N: usize> VectorNurbs<N> {
    pub fn try_new(
        degree: u8,
        knots: Vec<f64>,
        control_points: Vec<[f64; N]>,
    ) -> Result<Self, ConstructError> {
        validate(degree, &knots, control_points.len())?;
        let knot_vector = crate::knot::KnotVector::try_new(knots)
            .expect("validate already ensured monotone + length");
        Ok(Self {
            degree,
            knots: knot_vector,
            control_points,
        })
    }

    #[must_use]
    pub fn degree(&self) -> u8 {
        self.degree
    }
    #[must_use]
    pub fn knots(&self) -> &[f64] {
        self.knots.as_slice()
    }
    #[must_use]
    pub fn control_points(&self) -> &[[f64; N]] {
        &self.control_points
    }
}

#[cfg(test)]
#[allow(clippy::float_cmp)]
mod tests;
