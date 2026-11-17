use super::hold_time_tests::{BenchChain, line};
use super::*;

const HOME: [f64; 3] = [0.0; 3];
const AWAY: [f64; 3] = [10.0, 0.0, 0.2];
const STROKE_END: [f64; 3] = [50.0, 0.0, 0.2];
const FEED_MM_S: f64 = 50.0;
const PRIME_MM: f64 = 2.0;
const STROKE_E_MM: f64 = 1.5;
/// `M220 S1` on a 100 mm/s stroke (#901).
const CRAWL_FEED_MM_S: f64 = 1.0;
const SLOW_STROKES: [[f64; 3]; 4] = [
    [10.0, 0.0, 0.2],
    [50.0, 0.0, 0.2],
    [50.0, 40.0, 0.2],
    [10.0, 40.0, 0.2],
];

/// Feeds the planned `lines` to a guard standing at [`HOME`].
fn guard_from_home(lines: &[crate::live_config::GcodeLine]) {
    let chain = BenchChain::new();
    let mut guard = TurningGuard::new(chain.config.clone(), &chain.chains, &HOME);
    for item in chain.plan(lines) {
        assert!(guard.feed(item, &mut |_| true));
    }
}

#[test]
#[should_panic(
    expected = "turning guard: discontinuous move at line 1: starts at [10.0, 0.0, 0.2] \
                           but the toolhead is at [0.0, 0.0, 0.0]"
)]
fn a_move_starting_off_the_toolhead_fails_as_a_discontinuity() {
    guard_from_home(&[line(1, AWAY, STROKE_END, STROKE_E_MM, FEED_MM_S)]);
}

#[test]
#[should_panic(
    expected = "turning guard: discontinuous move at line 2: starts at [10.0, 0.0, 0.2] \
                           but the toolhead is at [0.0, 0.0, 0.0]"
)]
fn a_move_off_the_toolhead_after_a_prime_fails_as_a_discontinuity() {
    guard_from_home(&[
        line(1, HOME, HOME, PRIME_MM, FEED_MM_S),
        line(2, AWAY, STROKE_END, STROKE_E_MM, FEED_MM_S),
    ]);
}

/// The guard samples the planned motion where its pending checks and
/// crawl scan read it: the kernels' reach and the valley lead budget
/// around the cells still to prove, twice over for the cells a re-plan
/// re-opens. Strokes of 40 s at 1 mm/s must not grid whole held moves.
#[test]
fn the_guard_grids_slow_strokes_within_its_windows() {
    let most = most_held_over_slow_strokes(|guard| {
        let grid = guard.grid.borrow();
        grid.end() - grid.base()
    });
    let bound = window_cells_bound();
    assert!(
        most <= bound,
        "the guard gridded {most} cells of slow strokes, past the {bound} its windows read"
    );
}

/// The crawl scan keeps the cells it tracks within the same windows: one
/// crawl at 1 mm/s spans every stroke, and its run is not held cell by cell.
#[test]
fn the_guard_tracks_slow_strokes_within_its_windows() {
    let most = most_held_over_slow_strokes(|guard| guard.valleys.track.stored_cells());
    let bound = window_cells_bound();
    assert!(
        most <= bound,
        "the guard tracked {most} cells of slow strokes, past the {bound} its windows read"
    );
}

fn window_cells_bound() -> i64 {
    let chain = BenchChain::new();
    let guard = TurningGuard::new(chain.config.clone(), &chain.chains, &HOME);
    let (back, ahead) = guard.span;
    let budget_cells = (VALLEY_LEAD_BUDGET_S / guard.h()).ceil() as i64;
    2 * (back + ahead + 4 + 2 * budget_cells)
}

/// The most `held` reads of a guard fed [`SLOW_STROKES`] at
/// [`CRAWL_FEED_MM_S`], after each planned item.
fn most_held_over_slow_strokes(held: impl Fn(&TurningGuard) -> i64) -> i64 {
    let chain = BenchChain::new();
    let mut guard = TurningGuard::new(chain.config.clone(), &chain.chains, &HOME);
    let mut lines = vec![line(1, HOME, SLOW_STROKES[0], 0.0, CRAWL_FEED_MM_S)];
    for (seq, ends) in SLOW_STROKES.windows(2).enumerate() {
        lines.push(line(
            seq + 2,
            ends[0],
            ends[1],
            STROKE_E_MM,
            CRAWL_FEED_MM_S,
        ));
    }
    let mut most = 0;
    for item in chain.plan(&lines) {
        assert!(guard.feed(item, &mut |_| true));
        most = most.max(held(&guard));
    }
    most
}

#[test]
#[should_panic(
    expected = "turning guard: the cell at t=0.512500s still asks 30000 mm/s² of the extruder \
                (limit 20000 mm/s²) after 9 back-offs of line 12, the next taking it under \
                0.03125 of its budget; the cell reads [0.507500, 0.520000]s, the frozen seam is \
                at t=0.480000s (line 10)"
)]
fn a_cell_binding_past_the_scale_floor_names_itself_and_the_frozen_seam() {
    Stuck {
        binding: Binding {
            cell: 0.5125,
            from: 0.5075,
            to: 0.52,
            demand: 30_000.0,
            max_accel: 20_000.0,
            asked: Asked::Accel,
        },
        line: 12,
        backoffs: 9,
        frozen: (0.48, 10),
    }
    .fail();
}
