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

    /// The least time the leaders take over the run, from the farthest seam
    /// kept behind to the farthest ahead, every seam held to its limit, the
    /// move planned at a budget `accel` under `ceiling` and the moves around
    /// it at `around_accel`: the move's own seams take its disk at `accel`,
    /// the others theirs at `around_accel`. Along the climb
    /// `y = ∫a dx` the fastest speed squared rises by at most 2 per unit, so
    /// between neighbouring seams it is the least of a launch from the seams
    /// before, a brake into the seams after, and on the move `ceiling²`:
    /// linear pieces, each crossed at an acceleration `a` in
    /// `2·Δy/(a·(√g₀ + √g₁))`.
    pub fn run_time(&self, (ceiling, accel): (f64, f64), around_accel: f64) -> f64 {
        let move_climb = accel * self.length_mm;
        let limit_sq = |term: &SeamTerm| {
            let budget = if term.run_mm == 0.0 {
                accel
            } else {
                around_accel
            };
            let limit = term.limit.at(budget);
            limit * limit
        };
        let mut seams = [(0.0, 0.0); 2 * ENVELOPE_TERMS_MAX + 2];
        let mut count = 0;
        let mut add = |at: f64, limit_sq: f64| {
            seams[count] = (at, limit_sq);
            count += 1;
        };
        for term in self.behind.terms() {
            add(-around_accel * term.run_mm, limit_sq(term));
        }
        add(0.0, ceiling * ceiling);
        add(move_climb, ceiling * ceiling);
        for term in self.ahead.terms() {
            add(move_climb + around_accel * term.run_mm, limit_sq(term));
        }
        let seams = &mut seams[..count];
        seams.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut braking = [f64::INFINITY; 2 * ENVELOPE_TERMS_MAX + 2];
        for index in (0..count).rev() {
            let (at, limit_sq) = seams[index];
            let own = limit_sq + CLIMB_PER_UNIT * at;
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
            launch = launch.min(limit_sq - CLIMB_PER_UNIT * from);
            let to = seams[index + 1].0;
            if to <= from {
                continue;
            }
            let (cap, there_accel) = if from >= 0.0 && to <= move_climb {
                (ceiling * ceiling, accel)
            } else {
                (f64::INFINITY, around_accel)
            };
            time += piece_time(from, to, (launch, braking[index + 1], cap)) / there_accel;
        }
        time
    }
}

/// How fast the speed squared rises per unit of climb `y = ∫a dx`.
const CLIMB_PER_UNIT: f64 = 2.0;

/// The climb `∫dy/√g` over `[from, to]` of the speed whose square `g` is
/// the least of `launch + 2y`, `brake − 2y` and `cap`.
fn piece_time(from: f64, to: f64, (launch, brake, cap): (f64, f64, f64)) -> f64 {
    let climb = CLIMB_PER_UNIT;
    let speed_sq = |y: f64| {
        (launch + climb * y)
            .min(brake - climb * y)
            .min(cap)
            .max(0.0)
    };
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
