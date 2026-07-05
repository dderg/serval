//! Bit-deterministic f64 transcendentals for the motion pipeline.
//!
//! IEEE basic arithmetic and `sqrt` are correctly rounded and identical on
//! every platform, but `f64::sin`/`cos`/`atan2`/... dispatch to the host C
//! libm, whose last ulp differs between macOS and glibc. Those ulps get
//! amplified by discrete decisions downstream (sample-count `ceil`,
//! coefficient trimming, near-tie sorts), so trajectories planned on one
//! platform don't reproduce bit-for-bit on another. Every transcendental on
//! the planning path must go through these `libm`-crate wrappers instead of
//! the inherent `f64` methods.
//!
//! Free functions, not an extension trait: same-name trait methods would be
//! silently shadowed by the inherent `f64` ones at the call site.

pub fn sin(x: f64) -> f64 {
    libm::sin(x)
}

pub fn cos(x: f64) -> f64 {
    libm::cos(x)
}

pub fn sin_cos(x: f64) -> (f64, f64) {
    libm::sincos(x)
}

pub fn asin(x: f64) -> f64 {
    libm::asin(x)
}

pub fn acos(x: f64) -> f64 {
    libm::acos(x)
}

pub fn atan2(y: f64, x: f64) -> f64 {
    libm::atan2(y, x)
}

pub fn hypot(x: f64, y: f64) -> f64 {
    libm::hypot(x, y)
}

pub fn cbrt(x: f64) -> f64 {
    libm::cbrt(x)
}
