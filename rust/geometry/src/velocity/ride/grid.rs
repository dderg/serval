use super::super::disk::disk_rail_accel;
use super::{POS_EPS_MM, Track};

const RANGE_MIN_BLOCK: usize = 64;
const KAPPA_EPS: f64 = 1e-12;

pub(super) struct Grid<'a> {
    pub(super) t: &'a Track<'a>,
    cap_range_min: Vec<f64>,
    /// Per cell: the chord's brake slope is unreachable at the speeds the
    /// cap holds (`slope²/2j` exceeds the cap), while staying within the
    /// accel rail (super-rail descents keep their chord-crossing treatment).
    /// No tangent landing exists on such a chord — the committed brake
    /// anchors on the descent's end node instead.
    wall_cont: Vec<bool>,
    /// Per cell: first wall cell at or after it (`usize::MAX` if none).
    next_wall_cell: Vec<usize>,
    /// Per cell: the ascending mirror of `wall_cont` — swinging the slope up
    /// from the previous chord's gains more speed than the cap allows, so no
    /// ride can follow the chord and no flight landing can match its slope.
    wall_up: Vec<bool>,
    /// Per cell: the last cell of its maximal uniform span — contiguous
    /// straight cells sharing one accel budget and one cap slope-accel (to
    /// integration noise). Flat cruise and constant-decel envelope stretches
    /// merge; a kink or the envelope's jerk swing breaks the span. Within a
    /// span the peel march may step to its tangency aim directly instead of
    /// cell by cell — slope and rail are constant there and the cap is one
    /// linear-in-`v²` chord, so nothing the march reads changes mid-span.
    span_last: Vec<usize>,
}

impl<'a> Grid<'a> {
    pub(super) fn new(t: &'a Track<'a>) -> Self {
        let cap_range_min = t
            .cap_v
            .chunks(RANGE_MIN_BLOCK)
            .map(|c| c.iter().fold(f64::INFINITY, |m, &x| m.min(x)))
            .collect();
        let n = t.s.len();
        let cells = n - 1;
        let straight_cell =
            |c: usize| t.kappa[c].abs() <= KAPPA_EPS && t.kappa[c + 1].abs() <= KAPPA_EPS;
        let mut span_last = vec![0usize; cells];
        span_last[cells - 1] = cells - 1;
        for c in (0..cells.saturating_sub(1)).rev() {
            let mergeable = straight_cell(c)
                && straight_cell(c + 1)
                && (t.cap_a[c + 1] - t.cap_a[c]).abs() <= 1e-9 * (1.0 + t.cap_a[c].abs())
                && t.accel[c] == t.accel[c + 1]
                && t.accel[c + 1] == t.accel[c + 2];
            span_last[c] = if mergeable { span_last[c + 1] } else { c };
        }
        let mut wall_cont = vec![false; cells];
        let mut wall_up = vec![false; cells];
        let mut next_wall_cell = vec![usize::MAX; cells];
        let mut nw = usize::MAX;
        for c in (0..cells).rev() {
            let rail_top = disk_rail_accel(t.accel[c], t.kappa[c], t.cap_v[c]);
            let super_rail = t.cap_a[c] < -rail_top;
            // Riding onto this chord tangentially means swinging the brake
            // slope down from the previous chord's, which sheds
            // `(slope² − prev²)/2j` of speed on top of what the approach
            // already carries; a chord demanding more of that excess than
            // the cap even holds is the sampled shadow of a velocity step —
            // no tangent landing exists on it. Measuring the *step* (not the
            // slope from zero) keeps a continuous brake curve's own steep
            // tail — where the profile arrives already carrying the slope —
            // out of the wall class.
            let prev = t.cap_a[c.saturating_sub(1)].clamp(-rail_top, 0.0);
            let excess_shed = (t.cap_a[c] * t.cap_a[c] - prev * prev) / (2.0 * t.j_max);
            wall_cont[c] = !super_rail && t.cap_a[c] < prev && excess_shed > t.cap_v[c];
            if wall_cont[c] {
                nw = c;
            }
            next_wall_cell[c] = nw;
            let prev_up = t.cap_a[c.saturating_sub(1)].clamp(0.0, rail_top);
            let excess_gain = (t.cap_a[c] * t.cap_a[c] - prev_up * prev_up) / (2.0 * t.j_max);
            wall_up[c] = t.cap_a[c] > prev_up && excess_gain > t.cap_v[c + 1];
        }
        Self {
            t,
            cap_range_min,
            wall_cont,
            next_wall_cell,
            wall_up,
            span_last,
        }
    }

    /// Whether cell `c`'s chord ascends as an unfollowable wall.
    pub(super) fn wall_up(&self, c: usize) -> bool {
        self.wall_up[c.min(self.wall_up.len() - 1)]
    }

    /// First wall-run start at or after cell `c`.
    pub(super) fn next_wall(&self, c: usize) -> Option<usize> {
        let w = self.next_wall_cell[c.min(self.next_wall_cell.len() - 1)];
        (w != usize::MAX).then_some(w)
    }

    /// First node past the contiguous unlandable descent starting at `w` —
    /// the anchor node for the committed wall brake.
    pub(super) fn wall_run_end(&self, w: usize) -> usize {
        let mut k = w;
        while k + 1 < self.wall_cont.len() && self.wall_cont[k + 1] {
            k += 1;
        }
        k + 1
    }

    pub(super) fn n(&self) -> usize {
        self.t.s.len()
    }

    /// Cell whose span contains `s` (clamped).
    pub(super) fn cell(&self, s: f64) -> usize {
        let n = self.n();
        let i = self.t.s.partition_point(|&x| x <= s);
        i.clamp(1, n - 1) - 1
    }

    /// End arc of the maximal uniform span containing cell `c`.
    pub(super) fn span_end_s(&self, c: usize) -> f64 {
        self.t.s[self.span_last[c] + 1]
    }

    /// `cell` walked forward from `hint` to contain `s`. The peel march only
    /// moves forward, so the walk is amortized O(cells crossed) per call
    /// instead of a binary search per lookup.
    pub(super) fn cell_ahead(&self, hint: usize, s: f64) -> usize {
        let last = self.n() - 2;
        let mut c = hint.min(last);
        while c < last && s >= self.t.s[c + 1] {
            c += 1;
        }
        c
    }

    fn lerp_in(&self, arr: &[f64], c: usize, s: f64) -> f64 {
        let span = self.t.s[c + 1] - self.t.s[c];
        let f = if span > POS_EPS_MM {
            ((s - self.t.s[c]) / span).clamp(0.0, 1.0)
        } else {
            0.0
        };
        arr[c] + f * (arr[c + 1] - arr[c])
    }

    pub(super) fn lerp_node(&self, arr: &[f64], s: f64) -> f64 {
        self.lerp_in(arr, self.cell(s), s)
    }

    /// Cap speed within cell `c`, linear in `v²` between nodes: a
    /// constant-decel brake segment of the envelope is linear in `v²`, so
    /// this represents it exactly where a linear-in-`v` chord would sag
    /// below it.
    pub(super) fn cap_in(&self, c: usize, s: f64) -> f64 {
        let span = self.t.s[c + 1] - self.t.s[c];
        let f = if span > POS_EPS_MM {
            ((s - self.t.s[c]) / span).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let (w0, w1) = (
            self.t.cap_v[c] * self.t.cap_v[c],
            self.t.cap_v[c + 1] * self.t.cap_v[c + 1],
        );
        (w0 + f * (w1 - w0)).max(0.0).sqrt()
    }

    pub(super) fn cap_at(&self, s: f64) -> f64 {
        self.cap_in(self.cell(s), s)
    }

    pub(super) fn slope_at(&self, s: f64) -> f64 {
        self.t.cap_a[self.cell(s)]
    }

    pub(super) fn rail_in(&self, c: usize, s: f64, v: f64) -> f64 {
        disk_rail_accel(
            self.lerp_in(self.t.accel, c, s),
            self.lerp_in(self.t.kappa, c, s),
            v,
        )
    }

    pub(super) fn rail_at(&self, s: f64, v: f64) -> f64 {
        self.rail_in(self.cell(s), s, v)
    }

    pub(super) fn kappa_at(&self, s: f64) -> f64 {
        self.lerp_node(self.t.kappa, s)
    }

    pub(super) fn curved_near(&self, s: f64) -> bool {
        let c = self.cell(s);
        self.t.kappa[c].abs() > KAPPA_EPS || self.t.kappa[c + 1].abs() > KAPPA_EPS
    }

    /// Lower bound of the cap over `[s, s + dist]`.
    pub(super) fn cap_min_over(&self, s: f64, dist: f64) -> f64 {
        let lo = self.cell(s);
        let hi = self.cell(s + dist) + 1;
        let (b_lo, b_hi) = (lo / RANGE_MIN_BLOCK, hi / RANGE_MIN_BLOCK);
        let mut m = f64::INFINITY;
        if b_lo == b_hi {
            for &x in &self.t.cap_v[lo..=hi] {
                m = m.min(x);
            }
            return m;
        }
        for &x in &self.t.cap_v[lo..(b_lo + 1) * RANGE_MIN_BLOCK] {
            m = m.min(x);
        }
        for &x in &self.cap_range_min[(b_lo + 1)..b_hi] {
            m = m.min(x);
        }
        for &x in &self.t.cap_v[b_hi * RANGE_MIN_BLOCK..=hi] {
            m = m.min(x);
        }
        m
    }

    pub(super) fn end_s(&self) -> f64 {
        self.t.s[self.n() - 1]
    }
}
