pub const N_SPATIAL: usize = 3;
pub const MAX_AXES: usize = 8;
pub const MAX_LIMIT_SETS: usize = 8;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AxisSet(u8);

impl AxisSet {
    #[must_use]
    pub fn from_indices(indices: &[usize]) -> Self {
        let mut bits = 0_u8;
        for &i in indices {
            assert!(i < MAX_AXES, "axis index {i} out of range");
            bits |= 1 << i;
        }
        assert!(bits != 0, "empty axis set");
        Self(bits)
    }
    #[must_use]
    pub fn spatial() -> Self {
        Self((1 << N_SPATIAL) - 1)
    }
    #[must_use]
    pub fn is_spatial(self) -> bool {
        self.0 < (1 << N_SPATIAL)
    }
    #[must_use]
    pub fn is_follower(self) -> bool {
        self.0 >> N_SPATIAL != 0 && self.0 & ((1 << N_SPATIAL) - 1) == 0
    }
    #[must_use]
    pub fn contains(self, axis: usize) -> bool {
        self.0 & (1 << axis) != 0
    }
    pub fn indices(self) -> impl Iterator<Item = usize> {
        (0..MAX_AXES).filter(move |&i| self.contains(i))
    }
    #[must_use]
    pub fn count(self) -> usize {
        self.0.count_ones() as usize
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LimitSet {
    pub axes: AxisSet,
    pub v_max: f64,
    pub a_max: f64,
    pub j_max: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LimitKind {
    #[default]
    Config,
    Feedrate,
    RuntimeCap,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Limits {
    sets: [LimitSet; MAX_LIMIT_SETS],
    kinds: [LimitKind; MAX_LIMIT_SETS],
    n_sets: u8,
    n_axes: u8,
}

#[derive(Debug, thiserror::Error, PartialEq)]
pub enum LimitsError {
    #[error("no limit sets declared")]
    Empty,
    #[error("more than {MAX_LIMIT_SETS} limit sets")]
    TooMany,
    #[error("limit set {set}: caps must be positive (or +inf for undeclared)")]
    BadCap { set: usize },
    #[error("axis {axis}: no limit set declares a finite max_velocity covering it")]
    NoVelocityCoverage { axis: usize },
    #[error("axis {axis}: no limit set declares a finite max_accel covering it")]
    NoAccelCoverage { axis: usize },
    #[error("axis {axis}: no limit set declares a finite max_jerk covering it")]
    NoJerkCoverage { axis: usize },
    #[error("limit set {set}: mixes spatial and follower axes (unsupported)")]
    MixedSpatialFollower { set: usize },
}

impl Limits {
    pub fn try_new(sets: &[LimitSet], n_axes: usize) -> Result<Self, LimitsError> {
        assert!(
            (N_SPATIAL..=MAX_AXES).contains(&n_axes),
            "n_axes {n_axes} out of range"
        );
        if sets.is_empty() {
            return Err(LimitsError::Empty);
        }
        if sets.len() > MAX_LIMIT_SETS {
            return Err(LimitsError::TooMany);
        }
        for (idx, s) in sets.iter().enumerate() {
            let ok = |c: f64| c > 0.0 && !c.is_nan();
            if !(ok(s.v_max) && ok(s.a_max) && ok(s.j_max)) {
                return Err(LimitsError::BadCap { set: idx });
            }
            if !s.axes.is_spatial() && !s.axes.is_follower() {
                return Err(LimitsError::MixedSpatialFollower { set: idx });
            }
        }
        for axis in 0..n_axes {
            let covered = |f: fn(&LimitSet) -> f64| {
                sets.iter()
                    .any(|s| s.axes.contains(axis) && f(s).is_finite())
            };
            if !covered(|s| s.v_max) {
                return Err(LimitsError::NoVelocityCoverage { axis });
            }
            if !covered(|s| s.a_max) {
                return Err(LimitsError::NoAccelCoverage { axis });
            }
            if !covered(|s| s.j_max) {
                return Err(LimitsError::NoJerkCoverage { axis });
            }
        }
        let filler = sets[0];
        let mut arr = [filler; MAX_LIMIT_SETS];
        arr[..sets.len()].copy_from_slice(sets);
        Ok(Self {
            sets: arr,
            kinds: [LimitKind::Config; MAX_LIMIT_SETS],
            n_sets: sets.len() as u8,
            n_axes: n_axes as u8,
        })
    }

    #[must_use]
    pub fn kind(&self, set: usize) -> LimitKind {
        self.kinds.get(set).copied().unwrap_or(LimitKind::Config)
    }

    #[must_use]
    pub fn sets(&self) -> &[LimitSet] {
        &self.sets[..self.n_sets as usize]
    }

    #[must_use]
    pub fn n_axes(&self) -> usize {
        self.n_axes as usize
    }

    pub fn spatial_sets(&self) -> impl Iterator<Item = (usize, &LimitSet)> {
        self.sets()
            .iter()
            .enumerate()
            .filter(|(_, s)| s.axes.is_spatial())
    }

    pub fn follower_sets(&self) -> impl Iterator<Item = (usize, &LimitSet)> {
        self.sets()
            .iter()
            .enumerate()
            .filter(|(_, s)| s.axes.is_follower())
    }

    #[must_use]
    pub fn axis_boxes(v: [f64; 3], a: [f64; 3], j: [f64; 3]) -> Self {
        let sets: Vec<LimitSet> = (0..3)
            .map(|ax| LimitSet {
                axes: AxisSet::from_indices(&[ax]),
                v_max: v[ax],
                a_max: a[ax],
                j_max: j[ax],
            })
            .collect();
        Self::try_new(&sets, N_SPATIAL).expect("axis_boxes: finite positive caps")
    }

    #[must_use]
    pub fn norm_all(v: f64, a: f64, j: f64) -> Self {
        Self::try_new(
            &[LimitSet {
                axes: AxisSet::spatial(),
                v_max: v,
                a_max: a,
                j_max: j,
            }],
            N_SPATIAL,
        )
        .expect("norm_all: finite positive caps")
    }

    #[must_use]
    pub fn mvc_b(&self, c_prime: &[f64; 3], floor: f64) -> f64 {
        let mut bound = f64::INFINITY;
        for (_, s) in self.spatial_sets() {
            if !s.v_max.is_finite() {
                continue;
            }
            let p = restricted_norm(c_prime, s.axes);
            if p > floor {
                let vb = s.v_max / p;
                bound = bound.min(vb * vb);
            }
        }
        bound
    }

    #[must_use]
    pub fn a_tan_cap(&self, c_prime: &[f64; 3], floor: f64) -> f64 {
        self.tan_cap(c_prime, floor, |s| s.a_max)
    }

    #[must_use]
    pub fn j_tan_cap(&self, c_prime: &[f64; 3], floor: f64) -> f64 {
        self.tan_cap(c_prime, floor, |s| s.j_max)
    }

    fn tan_cap(&self, c_prime: &[f64; 3], floor: f64, cap: fn(&LimitSet) -> f64) -> f64 {
        let mut bound = f64::INFINITY;
        for (_, s) in self.spatial_sets() {
            let c = cap(s);
            if !c.is_finite() {
                continue;
            }
            let p = restricted_norm(c_prime, s.axes);
            if p > floor {
                bound = bound.min(c / p);
            }
        }
        bound
    }

    #[must_use]
    pub fn j_path(&self) -> f64 {
        self.spatial_sets()
            .map(|(_, s)| s.j_max)
            .filter(|j| j.is_finite())
            .fold(f64::INFINITY, f64::min)
    }

    #[must_use]
    pub fn v_ceiling(&self) -> f64 {
        self.spatial_sets()
            .map(|(_, s)| s.v_max)
            .filter(|v| v.is_finite())
            .fold(f64::NEG_INFINITY, f64::max)
    }

    #[must_use]
    pub fn axis_velocity_cap(&self, axis: usize) -> f64 {
        self.axis_cap(axis, |s| s.v_max)
    }

    #[must_use]
    pub fn axis_accel_cap(&self, axis: usize) -> f64 {
        self.axis_cap(axis, |s| s.a_max)
    }

    #[must_use]
    pub fn axis_jerk_cap(&self, axis: usize) -> f64 {
        self.axis_cap(axis, |s| s.j_max)
    }

    fn axis_cap(&self, axis: usize, cap: fn(&LimitSet) -> f64) -> f64 {
        self.sets()
            .iter()
            .filter(|s| s.axes.contains(axis))
            .map(cap)
            .fold(f64::INFINITY, f64::min)
    }

    #[must_use]
    pub fn with_sets_mapped(&self, f: impl Fn(&LimitSet) -> LimitSet) -> Self {
        let sets: Vec<LimitSet> = self.sets().iter().map(f).collect();
        let mut out = Self::try_new(&sets, self.n_axes()).expect("set mapping preserves validity");
        out.kinds = self.kinds;
        out
    }

    #[must_use]
    pub fn with_extra_sets(&self, extra: &[LimitSet]) -> Self {
        self.with_extra_sets_of_kind(extra, LimitKind::Config)
    }

    #[must_use]
    pub fn with_extra_sets_of_kind(&self, extra: &[LimitSet], kind: LimitKind) -> Self {
        let mut sets: Vec<LimitSet> = self.sets().to_vec();
        sets.extend_from_slice(extra);
        let mut out =
            Self::try_new(&sets, self.n_axes()).expect("appending sets preserves validity");
        out.kinds = self.kinds;
        for slot in out.kinds[self.n_sets as usize..sets.len()].iter_mut() {
            *slot = kind;
        }
        out
    }

    #[must_use]
    pub fn b_cent_cap(
        &self,
        c_prime: &[f64; 3],
        c_double_prime: &[f64; 3],
        kappa_floor: f64,
    ) -> f64 {
        let mut bound = f64::INFINITY;
        for (_, s) in self.spatial_sets() {
            if !s.a_max.is_finite() {
                continue;
            }
            let k = kappa_set(c_prime, c_double_prime, s.axes, kappa_floor);
            if k > kappa_floor {
                bound = bound.min(s.a_max / k);
            }
        }
        bound
    }
}

#[must_use]
pub fn restricted_norm(v: &[f64; 3], axes: AxisSet) -> f64 {
    debug_assert!(axes.is_spatial());
    axes.indices().map(|i| v[i] * v[i]).sum::<f64>().sqrt()
}

#[must_use]
pub fn kappa_set(c_prime: &[f64; 3], c_double_prime: &[f64; 3], axes: AxisSet, floor: f64) -> f64 {
    debug_assert!(axes.is_spatial());
    let mut pp = 0.0;
    let mut pq = 0.0;
    let mut qq = 0.0;
    for i in axes.indices() {
        pp += c_prime[i] * c_prime[i];
        pq += c_prime[i] * c_double_prime[i];
        qq += c_double_prime[i] * c_double_prime[i];
    }
    if pp.sqrt() <= floor {
        return qq.sqrt();
    }
    (qq - pq * pq / pp).max(0.0).sqrt()
}

#[cfg(test)]
mod tests;
