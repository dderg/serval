use bench_sim_logic::dir_merge::{Ambiguous, DirMerge, MIN_DIR_STEP_TICKS};
use bench_sim_logic::extend::Extend16;

#[test]
fn each_step_takes_the_level_of_the_last_edge_before_it() {
    let edges = [(150, false), (300, true)];
    let mut merge = DirMerge::new(&edges, true);
    let signs: Vec<i32> = [100, 200, 250, 400]
        .iter()
        .map(|&t| merge.sign_at(t).unwrap())
        .collect();
    assert_eq!(signs, [1, -1, -1, 1]);
    assert!(merge.finish());
}

#[test]
fn edges_after_the_last_step_still_set_the_carried_level() {
    let edges = [(500, false)];
    let mut merge = DirMerge::new(&edges, true);
    assert_eq!(merge.sign_at(100), Ok(1));
    assert!(!merge.finish());
}

#[test]
fn no_edges_keep_the_level() {
    let mut merge = DirMerge::new(&[], false);
    assert_eq!(merge.sign_at(1), Ok(-1));
    assert!(!merge.finish());
}

#[test]
fn the_extender_counts_through_wraps() {
    let mut counter = Extend16::new(65_000);
    assert_eq!(counter.update(65_500), 500);
    assert_eq!(counter.update(300), 500 + 336);
    assert_eq!(counter.update(300), 836);
    assert_eq!(counter.value(), 836);
}

#[test]
fn a_dir_edge_too_close_before_a_step_is_ambiguous() {
    let edges = [(100, false)];
    let mut merge = DirMerge::new(&edges, true);
    assert_eq!(
        merge.sign_at(100 + MIN_DIR_STEP_TICKS - 1),
        Err(Ambiguous {
            step: 100 + MIN_DIR_STEP_TICKS - 1,
            edge: 100
        })
    );
}

#[test]
fn a_dir_edge_too_close_after_a_step_is_ambiguous() {
    let edges = [(100, false)];
    let mut merge = DirMerge::new(&edges, true);
    assert_eq!(
        merge.sign_at(100 - (MIN_DIR_STEP_TICKS - 1)),
        Err(Ambiguous {
            step: 100 - (MIN_DIR_STEP_TICKS - 1),
            edge: 100
        })
    );
}

#[test]
fn a_dir_edge_at_the_same_tick_as_a_step_is_ambiguous() {
    let edges = [(100, false)];
    let mut merge = DirMerge::new(&edges, true);
    assert!(merge.sign_at(100).is_err());
}

#[test]
fn the_minimum_separation_on_either_side_is_unambiguous() {
    let edges = [(100, false), (200, true)];
    let mut merge = DirMerge::new(&edges, true);
    assert_eq!(merge.sign_at(100 - MIN_DIR_STEP_TICKS), Ok(1));
    assert_eq!(merge.sign_at(100 + MIN_DIR_STEP_TICKS), Ok(-1));
    assert_eq!(merge.sign_at(200 - MIN_DIR_STEP_TICKS), Ok(-1));
    assert_eq!(merge.sign_at(200 + MIN_DIR_STEP_TICKS), Ok(1));
}
