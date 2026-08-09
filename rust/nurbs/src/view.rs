pub trait NurbsView {
    fn degree(&self) -> u8;
    fn knots(&self) -> &[f64];
    fn control_points(&self) -> &[f64];

    #[inline]
    fn control_point_count(&self) -> usize {
        self.control_points().len()
    }
}

pub trait VectorNurbsView<const N: usize> {
    fn degree(&self) -> u8;
    fn knots(&self) -> &[f64];
    fn control_points(&self) -> &[[f64; N]];

    #[inline]
    fn control_point_count(&self) -> usize {
        self.control_points().len()
    }
}
