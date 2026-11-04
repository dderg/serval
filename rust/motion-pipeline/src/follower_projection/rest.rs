//! Rests inside a follower's support: stretches where the leaders' speed
//! falls to what their fits cannot resolve. There the direction of the
//! fitted leaders' velocity is their fit noise, and the path speed's
//! curvature `|a⊥|²/|v|` — the fit noise's own turning divided by a
//! vanishing speed — is no toolhead motion. A rest reads the path speed's
//! rate and its derivative along the direction the leaders enter it with,
//! up to its split, and along the direction they leave it with from there
//! on: a stop-and-go reads smooth, and a reversal is a break where the
//! rate jumps by what the two directions read of the acceleration.

use super::{FollowerSignal, GRID_DEDUP_EPS_S, MAX_LEADERS};
use geometry::MIN_DEVICE_INTERVAL_S;

use crate::lowering::FitTol;
use crate::shaper::taylor::{Ball, TaylorModel, Window};
use crate::shaper::velocity_resolution_at_scales;

/// How many halvings a stretch's search for a rest goes to: a rest of a
/// leader at an acceleration `a` is `2·floor/a` wide.
const REST_SEARCH_DEPTH: u32 = 48;

/// Bisection steps placing a rest's ends and its split.
const REST_REFINE_STEPS: u32 = 96;

/// One rest: `[from, to]`, the leaders' speed at most the floor inside, the
/// floor at its ends; an end or the split within [`GRID_DEDUP_EPS_S`] of a
/// break of the leaders or the raw track lies on it. The leaders' unit velocity `entry` at `from` reads
/// the path before `split`, `exit` at `to` from it on.
#[derive(Clone, Copy, Debug)]
pub(super) struct Rest {
    pub(super) from: f64,
    pub(super) split: f64,
    pub(super) to: f64,
    entry: [f64; MAX_LEADERS],
    exit: [f64; MAX_LEADERS],
}

impl Rest {
    /// The direction the rest reads the path along at `t`.
    pub(super) fn direction(&self, t: f64) -> &[f64; MAX_LEADERS] {
        if t < self.split {
            &self.entry
        } else {
            &self.exit
        }
    }

    pub(super) fn holds(&self, t: f64) -> bool {
        self.from <= t && t <= self.to
    }

    /// The rest's ends and split.
    pub(super) fn breaks(&self) -> [f64; 3] {
        [self.from, self.split, self.to]
    }
}

impl FollowerSignal<'_> {
    /// The leaders' speed under which their fits cannot tell their
    /// direction: the velocity resolution of a fit to `leader_tol` at the
    /// scales the leaders reach over the support — what an acceleration
    /// within tolerance carries across the shortest span such a fit holds.
    pub(super) fn rest_speed(&self, leader_tol: FitTol) -> f64 {
        let (mut position_scale, mut velocity_scale) = (0.0_f64, 0.0_f64);
        for &t in &self.grid {
            for index in 0..self.leaders.axes.len() {
                let (position, velocity, _) = self.leaders.pva(index, t);
                position_scale = position_scale.max(position.abs());
                velocity_scale = velocity_scale.max(velocity.abs());
            }
        }
        let time_scale = self.grid[0].abs().max(self.grid[self.grid.len() - 1].abs());
        velocity_resolution_at_scales(position_scale, velocity_scale, time_scale, leader_tol)
    }

    /// The rests of the leaders inside the support, between its end
    /// slivers, where their speed falls to `floor`, in order.
    pub(super) fn find_rests(&self, floor: f64) -> Vec<Rest> {
        let support = (self.grid[0], self.grid[self.grid.len() - 1]);
        let interior = (
            self.start_sliver.map_or(support.0, |sliver| sliver.inner),
            self.end_sliver.map_or(support.1, |sliver| sliver.inner),
        );
        if interior.0 >= interior.1 {
            return Vec::new();
        }
        let Some(breaks) = self.breaks_between(interior.0, interior.1, &[]) else {
            return Vec::new();
        };
        let mut candidates = Vec::new();
        for stretch in breaks.windows(2).filter(|stretch| stretch[0] < stretch[1]) {
            self.rest_candidates((stretch[0], stretch[1]), floor, 0, &mut candidates);
        }
        let mut rests: Vec<Rest> = Vec::new();
        for (lo, hi) in candidates {
            if rests.last().is_some_and(|rest| rest.to >= lo) {
                continue;
            }
            let centre = 0.5 * lo + 0.5 * hi;
            if self.leaders.speed(centre) > floor {
                continue;
            }
            let on_break = |t: f64| {
                let index = breaks.partition_point(|&b| b < t);
                [index.checked_sub(1), Some(index)]
                    .into_iter()
                    .flatten()
                    .filter_map(|index| breaks.get(index).copied())
                    .find(|&b| (b - t).abs() <= GRID_DEDUP_EPS_S)
                    .unwrap_or(t)
            };
            let from = on_break(self.rest_edge(centre, interior.0, floor));
            let to = on_break(self.rest_edge(centre, interior.1, floor));
            rests.push(Rest {
                from,
                split: on_break(self.slowest_between(from, to)),
                to,
                entry: self.unit_velocity(from),
                exit: self.unit_velocity(to),
            });
        }
        rests
    }

    /// Collects the windows of `(lo, hi)` no leader's speed bound rules a
    /// rest out of: halved down to [`REST_SEARCH_DEPTH`], or kept whole
    /// where every leader's speed bound is under the floor throughout.
    fn rest_candidates(
        &self,
        (lo, hi): (f64, f64),
        floor: f64,
        depth: u32,
        out: &mut Vec<(f64, f64)>,
    ) {
        let Some(windows) = self.axis_windows() else {
            return;
        };
        let window = Window::new(lo, hi);
        let speed_squared = windows.leaders.iter().fold(
            TaylorModel::constant(Ball::exact(0.0)),
            |sum, (_, axis)| {
                let [velocity, _, _] = axis
                    .motion_models(window, false)
                    .expect("a stretch between the follower's breaks has one formula per leader");
                sum.add(&velocity.mul(&velocity))
            },
        );
        let (centre, spread) = speed_squared.spread();
        let floor_squared = floor * floor;
        if centre - spread > floor_squared {
            return;
        }
        let resting_throughout = centre + spread <= floor_squared;
        let middle = window.center;
        let unresolved = hi - lo <= MIN_DEVICE_INTERVAL_S;
        if resting_throughout
            || unresolved
            || depth == REST_SEARCH_DEPTH
            || middle <= lo
            || middle >= hi
        {
            out.push((lo, hi));
            return;
        }
        self.rest_candidates((lo, middle), floor, depth + 1, out);
        self.rest_candidates((middle, hi), floor, depth + 1, out);
    }

    /// Where the leaders' speed rises to `floor` going from `inside`, at
    /// most `floor`, toward `bound`; `bound` where it never does.
    fn rest_edge(&self, inside: f64, bound: f64, floor: f64) -> f64 {
        let mut reach = (bound - inside) * f64::EPSILON.sqrt();
        let mut outside = loop {
            let probe = inside + reach;
            let past = if bound > inside {
                probe >= bound
            } else {
                probe <= bound
            };
            if past || reach == 0.0 {
                if self.leaders.speed(bound) <= floor {
                    return bound;
                }
                break bound;
            }
            if self.leaders.speed(probe) > floor {
                break probe;
            }
            reach *= 2.0;
        };
        let mut resting = inside;
        for _ in 0..REST_REFINE_STEPS {
            let middle = 0.5 * resting + 0.5 * outside;
            if middle == resting || middle == outside {
                break;
            }
            if self.leaders.speed(middle) <= floor {
                resting = middle;
            } else {
                outside = middle;
            }
        }
        resting
    }

    /// Where the leaders are slowest on `[from, to]`, by golden-section
    /// search: the time a rest's two directions meet.
    fn slowest_between(&self, from: f64, to: f64) -> f64 {
        let ratio = 0.5 * (5.0_f64.sqrt() - 1.0);
        let (mut lo, mut hi) = (from, to);
        for _ in 0..REST_REFINE_STEPS {
            let left = hi - ratio * (hi - lo);
            let right = lo + ratio * (hi - lo);
            if !(lo < left && left < right && right < hi) {
                break;
            }
            if self.leaders.speed(left) <= self.leaders.speed(right) {
                hi = right;
            } else {
                lo = left;
            }
        }
        0.5 * lo + 0.5 * hi
    }

    /// The leaders' unit velocity at `t`, zero where they stand still.
    fn unit_velocity(&self, t: f64) -> [f64; MAX_LEADERS] {
        let speed = self.leaders.speed(t);
        let mut direction = [0.0; MAX_LEADERS];
        if speed > 0.0 {
            for (index, slot) in direction
                .iter_mut()
                .enumerate()
                .take(self.leaders.axes.len())
            {
                *slot = self.leaders.pva(index, t).1 / speed;
            }
        }
        direction
    }

    /// The rest `t` lies in, if any.
    pub(super) fn rest_at(&self, t: f64) -> Option<&Rest> {
        let index = self.rests.partition_point(|rest| rest.to < t);
        self.rests.get(index).filter(|rest| rest.holds(t))
    }

    /// The path speed's rate and its derivative inside `rest` at `t`: the
    /// leaders' acceleration and jerk read along its direction there.
    pub(super) fn rest_rates(&self, rest: &Rest, t: f64) -> (f64, f64) {
        let direction = rest.direction(t);
        let (states, leaders) = self.leaders.pvaj_states(t);
        states[..leaders]
            .iter()
            .zip(direction)
            .fold((0.0, 0.0), |(rate, second), (&(_, _, a, j), &d)| {
                (rate + d * a, second + d * j)
            })
    }
}
