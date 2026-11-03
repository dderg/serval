use std::fs;

const SUITE_MARKER: &str = "proptest!";
const SEED_KEPT_BY: [&str; 2] = ["FileFailurePersistence", "rng_seed"];

#[test]
fn every_proptest_suite_keeps_the_seed_of_a_failure() {
    let offenders: Vec<_> = source_tree::rust_sources()
        .into_iter()
        .filter(|path| {
            fs::read_to_string(path).is_ok_and(|text| {
                text.lines()
                    .any(|line| line.trim_start().starts_with(SUITE_MARKER))
                    && !SEED_KEPT_BY.iter().any(|keeper| text.contains(keeper))
            })
        })
        .collect();
    assert!(
        offenders.is_empty(),
        "proptest's default persistence cannot find a crate root under nextest, so a failure's \
         seed is lost and rust-fuzz triage cannot replay it; give the suite \
         FileFailurePersistence::Direct(\"proptest-regressions/<file>.txt\") or a fixed \
         rng_seed: {offenders:?}"
    );
}
