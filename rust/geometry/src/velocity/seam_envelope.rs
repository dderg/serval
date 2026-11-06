use super::disk;

/// The most seams one side of a move's run keeps: every seam that binds
/// the leaders' speed there at some budget, the rest pruned as dominated.
const ENVELOPE_TERMS_MAX: usize = 16;

/// What holds the leaders at a seam: the feed ceilings either side, and
/// the disk of the sharper curvature there, `√(A/κ)` under a budget `A`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SeamLimit {
    pub ceiling_mm_s: f64,
    pub kappa: f64,
}

impl SeamLimit {
    pub(super) fn ceiling(ceiling_mm_s: f64) -> Self {
        Self {
            ceiling_mm_s,
            kappa: 0.0,
        }
    }

    /// The speed limit under a budget `accel` on both sides.
    pub fn at(&self, accel: f64) -> f64 {
        self.ceiling_mm_s.min(disk::limit_speed(self.kappa, accel))
    }
}

/// `at0 + slope·A`.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Line {
    at0: f64,
    slope: f64,
}

impl Line {
    fn at(&self, accel: f64) -> f64 {
        self.at0 + self.slope * accel
    }
}

/// A seam `run_mm` along the path from a move's edge: under a budget `A`
/// on every move between, the leaders reach the edge from it at no more
/// than `√(w(A)² + 2A·run)`, `w` its [`SeamLimit`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SeamTerm {
    pub limit: SeamLimit,
    pub run_mm: f64,
}

impl SeamTerm {
    const REST: Self = Self {
        limit: SeamLimit {
            ceiling_mm_s: 0.0,
            kappa: 0.0,
        },
        run_mm: 0.0,
    };

    pub fn speed_sq(&self, accel: f64) -> f64 {
        let limit = self.limit.at(accel);
        limit * limit + 2.0 * accel * self.run_mm
    }

    /// `speed_sq` as the lower of two lines in `A`: the ceiling's
    /// `w² + 2A·run`, and the disk's `(1/κ + 2·run)·A` on a curved seam.
    fn lines(&self) -> (Line, Option<Line>) {
        let climb = 2.0 * self.run_mm;
        let ceiling = self.limit.ceiling_mm_s;
        (
            Line {
                at0: ceiling * ceiling,
                slope: climb,
            },
            (self.limit.kappa > 0.0).then(|| Line {
                at0: 0.0,
                slope: 1.0 / self.limit.kappa + climb,
            }),
        )
    }

    /// Whether `speed_sq` stays at or under `line` at every budget: the
    /// difference is concave with one kink, so its start, its kink and its
    /// final slope decide.
    fn under(&self, line: Line) -> bool {
        let (ceiling, disk) = self.lines();
        let start = disk.map_or(ceiling.at0, |disk| disk.at0.min(ceiling.at0));
        let kink_holds = disk.is_none_or(|_| {
            let kink = self.limit.ceiling_mm_s * self.limit.ceiling_mm_s * self.limit.kappa;
            ceiling.at(kink) <= line.at(kink)
        });
        start <= line.at0 && kink_holds && ceiling.slope <= line.slope
    }

    /// Whether this seam holds the leaders at least as tight as `other`
    /// at every budget, so `other` never binds beside it.
    fn dominates(&self, other: &Self) -> bool {
        let (ceiling, disk) = other.lines();
        self.under(ceiling) && disk.is_none_or(|disk| self.under(disk))
    }
}

/// The seams on one side of a move that can bind the leaders' speed at
/// its edge at some budget, measured from that edge: the lower envelope of
/// their [`SeamTerm::speed_sq`]. A seam another dominates is dropped, and
/// carrying the envelope along the path adds the same climb to every term,
/// so a dropped seam never binds again.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SeamEnvelope {
    terms: [SeamTerm; ENVELOPE_TERMS_MAX],
    len: usize,
}

impl SeamEnvelope {
    pub(super) fn of(limit: SeamLimit) -> Self {
        let mut terms = [SeamTerm::REST; ENVELOPE_TERMS_MAX];
        terms[0] = SeamTerm { limit, run_mm: 0.0 };
        Self { terms, len: 1 }
    }

    pub fn terms(&self) -> &[SeamTerm] {
        &self.terms[..self.len]
    }

    /// The envelope with `term` among its seams.
    pub(super) fn with(mut self, term: SeamTerm) -> Self {
        if self.terms().iter().any(|kept| kept.dominates(&term)) {
            return self;
        }
        let mut kept = 0;
        for index in 0..self.len {
            if !term.dominates(&self.terms[index]) {
                self.terms[kept] = self.terms[index];
                kept += 1;
            }
        }
        assert!(
            kept < ENVELOPE_TERMS_MAX,
            "a seam envelope binds at more than {ENVELOPE_TERMS_MAX} seams: {:?} and {term:?}",
            &self.terms[..kept]
        );
        self.terms[kept] = term;
        self.len = kept + 1;
        self
    }

    /// The envelope `length` further along the path from its seams.
    pub(super) fn across(mut self, length: f64) -> Self {
        for term in &mut self.terms[..self.len] {
            term.run_mm += length;
        }
        self
    }

    /// The fastest the leaders reach the edge, squared, under a budget
    /// `accel` on every move between.
    pub fn speed_sq(&self, accel: f64) -> f64 {
        self.terms()
            .iter()
            .map(|term| term.speed_sq(accel))
            .fold(f64::INFINITY, f64::min)
    }
}

/// The fastest a path `length` long is crossed from `entry` to `exit`
/// accelerating at up to `accel`.
pub(super) fn peak_between(entry: f64, exit: f64, accel: f64, length: f64) -> f64 {
    let climb = 2.0 * accel * length;
    (0.5 * (entry * entry + exit * exit + climb))
        .min(entry * entry + climb)
        .min(exit * exit + climb)
        .sqrt()
}

/// The relative margin a move's reachable speed keeps over the envelope it
/// is read off, so a plan that reaches the envelope to the last rounding
/// never rides the ceiling it becomes.
const REACHABLE_SLACK: f64 = 1e-9;

/// The fastest the leaders can cross a move on the planned path,
/// `peak_mm_s`, from the moves' feed ceilings, the disk limits at the seams
/// and the rests, the leaders accelerating no faster than each move's
/// reaching acceleration: whatever acceleration and seam caps the planner
/// derives, its plan of the move stays at or under it. The run around the
/// move, `length_mm` long, is bounded by the seams `behind` its start and
/// `ahead` of its end that can bind at some budget.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReachableSpeed {
    pub peak_mm_s: f64,
    pub behind: SeamEnvelope,
    pub ahead: SeamEnvelope,
    pub length_mm: f64,
}

impl ReachableSpeed {
    /// A move `length` long whose entry the leaders reach from behind at
    /// up to `behind_mm_s` and whose exit they reach from ahead at up to
    /// `ahead_mm_s`, under `ceiling` and accelerating at up to `accel`.
    pub(super) fn across(
        (behind_mm_s, behind): (f64, SeamEnvelope),
        (ahead_mm_s, ahead): (f64, SeamEnvelope),
        (ceiling, accel): (f64, f64),
        length: f64,
    ) -> Self {
        let peak = peak_between(behind_mm_s, ahead_mm_s, accel, length);
        Self {
            peak_mm_s: (peak * (1.0 + REACHABLE_SLACK)).min(ceiling),
            behind,
            ahead,
            length_mm: length,
        }
    }

    /// The fastest the move peaks at with every move on its run planned at
    /// a budget `accel`.
    pub fn run_peak(&self, accel: f64) -> f64 {
        peak_between(
            self.behind.speed_sq(accel).sqrt(),
            self.ahead.speed_sq(accel).sqrt(),
            accel,
            self.length_mm,
        )
    }

    /// The least time the leaders take over the run, from the farthest seam
    /// kept behind to the farthest ahead, with every move on it planned at
    /// a budget `accel`, every seam held to its limit and the move to
    /// `ceiling`. The move lies on `[0, L]`; along the run the fastest
    /// speed squared is, between neighbouring seams, the least of a climb
    /// from the seams before, a brake into the seams after, and on the move
    /// `ceiling²`: linear pieces, each crossed in `2·Δx/(√g₀ + √g₁)`.
    pub fn run_time(&self, ceiling: f64, accel: f64) -> f64 {
        let length = self.length_mm;
        let mut seams = [(0.0, 0.0); 2 * ENVELOPE_TERMS_MAX + 2];
        let mut count = 0;
        let mut add = |at: f64, limit: f64| {
            seams[count] = (at, limit * limit);
            count += 1;
        };
        for term in self.behind.terms() {
            add(-term.run_mm, term.limit.at(accel));
        }
        add(0.0, ceiling);
        add(length, ceiling);
        for term in self.ahead.terms() {
            add(length + term.run_mm, term.limit.at(accel));
        }
        let seams = &mut seams[..count];
        seams.sort_by(|a, b| a.0.total_cmp(&b.0));
        let climb = 2.0 * accel;
        let mut braking = [f64::INFINITY; 2 * ENVELOPE_TERMS_MAX + 2];
        for index in (0..count).rev() {
            let (at, limit_sq) = seams[index];
            let own = limit_sq + climb * at;
            braking[index] = if index + 1 < count {
                own.min(braking[index + 1])
            } else {
                own
            };
        }
        let mut launch = f64::INFINITY;
        let mut time = 0.0;
        for index in 0..count - 1 {
            let (from, limit_sq) = seams[index];
            launch = launch.min(limit_sq - climb * from);
            let to = seams[index + 1].0;
            if to <= from {
                continue;
            }
            let cap = if from >= 0.0 && to <= length {
                ceiling * ceiling
            } else {
                f64::INFINITY
            };
            time += piece_time(from, to, (launch, braking[index + 1], cap), climb);
        }
        time
    }
}

/// The time over `[from, to]` at the speed whose square is the least of
/// `launch + c·x`, `brake − c·x` and `cap`.
fn piece_time(from: f64, to: f64, (launch, brake, cap): (f64, f64, f64), climb: f64) -> f64 {
    let speed_sq = |x: f64| (launch + climb * x).min(brake - climb * x).min(cap).max(0.0);
    let mut cuts = [
        from,
        (brake - launch) / (2.0 * climb),
        (cap - launch) / climb,
        (brake - cap) / climb,
        to,
    ];
    cuts.sort_by(f64::total_cmp);
    let mut time = 0.0;
    for pair in cuts.windows(2) {
        let (lo, hi) = (pair[0].max(from), pair[1].min(to));
        if hi > lo {
            time += 2.0 * (hi - lo) / (speed_sq(lo).sqrt() + speed_sq(hi).sqrt());
        }
    }
    time
}

#[cfg(test)]
mod tests;
