use crate::{ConstructError, Float, MAX_DEGREE, NurbsView};

#[cfg(feature = "host")]
#[derive(Debug, Clone, PartialEq)]
pub struct ScalarNurbs<T: Float> {
    degree: u8,
    knots: crate::knot::KnotVector<T>,
    control_points: Vec<T>,
}

#[cfg(feature = "host")]
impl<T: Float> ScalarNurbs<T> {
    pub fn try_new(
        degree: u8,
        knots: Vec<T>,
        control_points: Vec<T>,
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
    pub fn knots(&self) -> &[T] {
        self.knots.as_slice()
    }
    #[must_use]
    pub fn control_points(&self) -> &[T] {
        &self.control_points
    }

    #[inline]
    #[must_use]
    pub fn as_view(&self) -> ScalarNurbsRef<'_, T> {
        ScalarNurbsRef {
            degree: self.degree,
            knots: self.knots.as_slice(),
            control_points: &self.control_points,
        }
    }
}

#[cfg(feature = "host")]
impl<T: Float> NurbsView<T> for ScalarNurbs<T> {
    #[inline]
    fn degree(&self) -> u8 {
        self.degree
    }
    #[inline]
    fn knots(&self) -> &[T] {
        self.knots.as_slice()
    }
    #[inline]
    fn control_points(&self) -> &[T] {
        &self.control_points
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ScalarNurbsRef<'a, T: Float> {
    pub(crate) degree: u8,
    pub(crate) knots: &'a [T],
    pub(crate) control_points: &'a [T],
}

impl<'a, T: Float> ScalarNurbsRef<'a, T> {
    pub fn try_new(
        degree: u8,
        knots: &'a [T],
        control_points: &'a [T],
    ) -> Result<Self, ConstructError> {
        validate(degree, knots, control_points.len())?;
        Ok(Self {
            degree,
            knots,
            control_points,
        })
    }

    #[must_use]
    pub fn degree(&self) -> u8 {
        self.degree
    }
    #[must_use]
    pub fn knots(&self) -> &[T] {
        self.knots
    }
    #[must_use]
    pub fn control_points(&self) -> &[T] {
        self.control_points
    }
}

impl<T: Float> NurbsView<T> for ScalarNurbsRef<'_, T> {
    #[inline]
    fn degree(&self) -> u8 {
        self.degree
    }
    #[inline]
    fn knots(&self) -> &[T] {
        self.knots
    }
    #[inline]
    fn control_points(&self) -> &[T] {
        self.control_points
    }
}

pub(crate) fn validate<T: Float>(
    degree: u8,
    knots: &[T],
    control_point_count: usize,
) -> Result<(), ConstructError> {
    if (degree as usize) > MAX_DEGREE {
        return Err(ConstructError::DegreeExceeded {
            actual: degree,
            max: MAX_DEGREE as u8,
        });
    }
    let p = degree as usize;
    let expected_knot_count = control_point_count + p + 1;
    if knots.len() != expected_knot_count {
        return Err(ConstructError::KnotCountMismatch {
            expected: expected_knot_count,
            got: knots.len(),
        });
    }
    if knots.len() < 2 * (p + 1) {
        return Err(ConstructError::KnotCountMismatch {
            expected: 2 * (p + 1),
            got: knots.len(),
        });
    }

    let start = knots[0];
    for k in &knots[1..=p] {
        if *k != start {
            return Err(ConstructError::KnotsNotClamped);
        }
    }
    let last_idx = knots.len() - 1;
    let end = knots[last_idx];
    for k in &knots[last_idx - p..last_idx] {
        if *k != end {
            return Err(ConstructError::KnotsNotClamped);
        }
    }

    for window in knots.windows(2) {
        if window[1] < window[0] {
            return Err(ConstructError::KnotsNotMonotone);
        }
    }

    if !(end > start) {
        return Err(ConstructError::DegenerateKnotRange);
    }

    Ok(())
}

#[cfg(all(test, feature = "host"))]
mod tests;
