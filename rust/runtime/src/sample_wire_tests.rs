#![allow(
    clippy::panic,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing
)]

use super::*;

#[test]
fn command_names_are_the_leading_token() {
    assert_eq!(command_name(SAMPLE_ANCHOR), SAMPLE_ANCHOR_NAME);
    assert_eq!(command_name(SAMPLE_RUN), SAMPLE_RUN_NAME);
    assert_eq!(command_name(SAMPLE_OVERLAY), SAMPLE_OVERLAY_NAME);
    assert_eq!(command_name(SAMPLE_GET_POSITION), SAMPLE_GET_POSITION_NAME);
    assert_eq!(command_name(SAMPLE_POSITION), SAMPLE_POSITION_NAME);
}

#[test]
fn every_command_is_distinct_and_carries_an_oid() {
    let mut names: Vec<&str> = SAMPLE_COMMANDS.iter().copied().map(command_name).collect();
    let count = names.len();
    names.sort_unstable();
    names.dedup();
    assert_eq!(names.len(), count);
    assert!(
        SAMPLE_COMMANDS
            .iter()
            .all(|argstring| argstring.contains("oid=%c")),
        "every sample command addresses a lane by oid"
    );
}

#[cfg(feature = "host")]
#[test]
fn the_c_header_mirrors_every_argstring() {
    let header = std::fs::read_to_string(
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../src/sample_wire.h"),
    )
    .expect("src/sample_wire.h is the mirrored contract");
    let flattened = header.replace("\\\n", "").replace('\n', " ");
    for argstring in SAMPLE_COMMANDS {
        assert!(
            flattened.contains(&format!("\"{argstring}\"")),
            "src/sample_wire.h is missing {argstring}"
        );
    }
    for cap in [
        format!(
            "SAMPLE_RUN_DATA_MAX {}",
            crate::sample_run::SAMPLE_RUN_DATA_MAX
        ),
        format!(
            "SAMPLE_RUN_COUNT_MAX {}",
            crate::sample_run::SAMPLE_RUN_COUNT_MAX
        ),
    ] {
        assert!(
            flattened.contains(&cap),
            "src/sample_wire.h is missing {cap}"
        );
    }
}

#[test]
fn the_fence_pair_agree_on_their_parameters() {
    assert_eq!(
        SAMPLE_BARRIER.trim_start_matches(SAMPLE_BARRIER_NAME),
        SAMPLE_BARRIER_ACK.trim_start_matches(SAMPLE_BARRIER_ACK_NAME),
        "a receipt must carry exactly the fields of the fence it answers"
    );
    for argstring in [SAMPLE_BARRIER, SAMPLE_BARRIER_ACK] {
        assert!(argstring.contains("seq=%u"), "{argstring} carries no seq");
    }
}

#[test]
fn the_readback_pair_agree_on_their_answer() {
    assert!(SAMPLE_GET_POSITION.starts_with(SAMPLE_GET_POSITION_NAME));
    assert!(
        SAMPLE_POSITION.contains("clock=%u") && SAMPLE_POSITION.contains("position=%i"),
        "the readback answers with both the clock it was taken at and the lane position"
    );
}
