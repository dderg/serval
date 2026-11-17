use std::path::{Path, PathBuf};
use std::process::Command;

const EXIT_REPORTED_PANIC: i32 = 70;
const FAILURE_PREFIX: &str = "REPLAY_FAILURE ";
const NOT_A_NUMBER: &str = "not-a-number";

fn replay_bench() -> PathBuf {
    let test_binary = std::env::current_exe().expect("test binary path");
    let profile_dir = test_binary
        .parent()
        .and_then(Path::parent)
        .expect("test binaries live in <profile>/deps");
    profile_dir.join("examples").join("gcode_replay_bench")
}

#[test]
fn a_reported_panic_exits_with_its_code_and_no_signal() {
    let output = Command::new(replay_bench())
        .arg(source_tree::repo_file(
            "rust/motion-core/tests/gcode/neptune_crash_short.gcode",
        ))
        .arg("--config")
        .arg(source_tree::repo_file(
            "rust/motion-core/tests/configs/bench_ramp.cfg",
        ))
        .args([NOT_A_NUMBER, "1", "--check"])
        .output()
        .expect("run the replay bench");

    assert_eq!(
        output.status.code(),
        Some(EXIT_REPORTED_PANIC),
        "a signal leaves no exit code: {:?}",
        output.status
    );
    let stdout = String::from_utf8(output.stdout).expect("utf-8 stdout");
    let record = stdout
        .lines()
        .find_map(|line| line.strip_prefix(FAILURE_PREFIX))
        .expect("the failure record is on stdout");
    let record: serde_json::Value = serde_json::from_str(record).expect("record is json");
    assert_eq!(record["kind"], "panic");
    assert!(
        record["message"]
            .as_str()
            .is_some_and(|message| message.contains("t_lo must be a number")),
        "{record}"
    );
}
