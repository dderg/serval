use bench_sim_logic::dir_merge::DirMerge;
use bench_sim_logic::extend::Extend16;

#[test]
fn each_step_takes_the_level_of_the_last_edge_before_it() {
    let edges = [(150, false), (300, true)];
    let mut merge = DirMerge::new(&edges, true);
    let signs: Vec<i32> = [100, 200, 250, 400]
        .iter()
        .map(|&t| merge.sign_at(t))
        .collect();
    assert_eq!(signs, [1, -1, -1, 1]);
    assert!(merge.finish());
}

#[test]
fn edges_after_the_last_step_still_set_the_carried_level() {
    let edges = [(500, false)];
    let mut merge = DirMerge::new(&edges, true);
    assert_eq!(merge.sign_at(100), 1);
    assert!(!merge.finish());
}

#[test]
fn no_edges_keep_the_level() {
    let mut merge = DirMerge::new(&[], false);
    assert_eq!(merge.sign_at(1), -1);
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
