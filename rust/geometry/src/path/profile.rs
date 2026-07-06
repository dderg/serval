pub trait CurvatureProfile {
    fn s_len(&self) -> f64;
    fn kappa(&self, s: f64) -> f64;
    fn dkappa_ds(&self, s: f64) -> f64;
    fn kappa_peak(&self) -> (f64, f64);
    fn kappa_endpoints(&self) -> (f64, f64);
}
