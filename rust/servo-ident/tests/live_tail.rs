//! Live tail contract: chunked tailing of a growing `.scap` must decode the
//! same samples as a one-shot `Scap::load` of the finished file, and the
//! offset handshake (`next_offset` echoed back) must resume exactly where
//! the previous poll stopped — including across a mid-record write, which
//! is what "growing" means at 4 kHz.

use std::io::Read;
use std::path::PathBuf;

use flate2::read::GzDecoder;

use servo_ident::live::{aligned_eof, newest_flat_scap, tail_scap, valid_capture_name};
use servo_ident::scap::Scap;

const FIXTURE: &str = "cal_p880_s550_i2273_20260710_151516.scap.gz";

fn fixture_bytes() -> Vec<u8> {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../test/fixtures/servo_captures")
        .join(FIXTURE);
    let gz = std::fs::read(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    let mut out = Vec::new();
    GzDecoder::new(&gz[..])
        .read_to_end(&mut out)
        .expect("fixture gunzips");
    out
}

fn temp_dir(label: &str) -> PathBuf {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let dir = std::env::temp_dir().join(format!(
        "servo_cal_live_{label}_{}_{nanos}",
        std::process::id()
    ));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn ferr_of(payload: &serde_json::Value, drive: &str) -> Vec<i64> {
    payload["drives"][drive]["ferr"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_i64().unwrap())
        .collect()
}

#[test]
fn chunked_tail_decodes_the_same_samples_as_a_full_load() {
    let bytes = fixture_bytes();
    let dir = temp_dir("chunks");
    let path = dir.join("live_test.scap");

    let full = Scap::from_bytes(&bytes).unwrap();
    let drive = full.drive_names()[0].clone();
    let record_size = full.header.record_size;
    let data_start = bytes.iter().position(|&b| b == b'\n').unwrap() + 1;

    let chunk_records = 1500usize;
    let cut1 = data_start + chunk_records * record_size;
    let cut2 = data_start + 2 * chunk_records * record_size;
    let expected: Vec<i64> =
        full.read_i64(0, "following_error").unwrap()[..2 * chunk_records].to_vec();

    std::fs::write(&path, &bytes[..cut1 + record_size / 2]).unwrap();
    let first = tail_scap(&path, 0).unwrap();
    assert_eq!(
        first["stride"], 1,
        "1500 records sit under the point cap, so no thinning"
    );
    assert_eq!(
        first["n_records"], chunk_records,
        "the trailing partial record must not be decoded"
    );
    let mut got = ferr_of(&first, &drive);
    let next = first["next_offset"].as_u64().unwrap();
    assert_eq!(next, cut1 as u64, "next_offset is record-aligned");

    std::fs::write(&path, &bytes[..cut2]).unwrap();
    let second = tail_scap(&path, next).unwrap();
    got.extend(ferr_of(&second, &drive));
    assert_eq!(second["next_offset"].as_u64().unwrap(), cut2 as u64);
    assert_eq!(second["first_record"], chunk_records);

    assert_eq!(
        got, expected,
        "two-chunk tail must reproduce the full-load following_error stream"
    );

    let drained = tail_scap(&path, cut2 as u64).unwrap();
    assert_eq!(drained["n_records"], 0, "no new bytes -> no records");

    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn attaching_at_eof_streams_only_samples_written_afterwards() {
    let bytes = fixture_bytes();
    let dir = temp_dir("attach_eof");
    let path = dir.join("live_test.scap");

    let full = Scap::from_bytes(&bytes).unwrap();
    let drive = full.drive_names()[0].clone();
    let record_size = full.header.record_size;
    let data_start = bytes.iter().position(|&b| b == b'\n').unwrap() + 1;
    let existing_records = 1500usize;
    let appended_records = 400usize;
    let cut = data_start + existing_records * record_size;

    std::fs::write(&path, &bytes[..cut + record_size / 2]).unwrap();
    let attach = aligned_eof(&path).unwrap();
    assert_eq!(
        attach, cut as u64,
        "attach offset lands on the last complete record boundary"
    );

    let empty = tail_scap(&path, attach).unwrap();
    assert_eq!(empty["n_records"], 0, "attaching replays nothing");
    assert_eq!(empty["next_offset"].as_u64().unwrap(), attach);

    let cut2 = cut + appended_records * record_size;
    std::fs::write(&path, &bytes[..cut2]).unwrap();
    let fresh = tail_scap(&path, attach).unwrap();
    assert_eq!(fresh["n_records"], appended_records);
    assert_eq!(fresh["first_record"], existing_records);
    let expected: Vec<i64> = full.read_i64(0, "following_error").unwrap()
        [existing_records..existing_records + appended_records]
        .to_vec();
    assert_eq!(ferr_of(&fresh, &drive), expected);

    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn misaligned_offset_fails_loud() {
    let bytes = fixture_bytes();
    let dir = temp_dir("misaligned");
    let path = dir.join("live_test.scap");
    std::fs::write(&path, &bytes).unwrap();

    let first = tail_scap(&path, 0).unwrap();
    let aligned = first["next_offset"].as_u64().unwrap();
    let err = tail_scap(&path, aligned + 1).unwrap_err();
    assert!(
        err.contains("not record-aligned"),
        "misaligned offset must name the contract, got: {err}"
    );

    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn newest_flat_scap_skips_run_dirs_and_failed_captures() {
    let bytes = fixture_bytes();
    let dir = temp_dir("newest");

    assert_eq!(newest_flat_scap(&dir).unwrap(), None);

    let run_dir = dir.join("cal_run_20260710_120000");
    std::fs::create_dir_all(&run_dir).unwrap();
    std::fs::write(run_dir.join("step_s550.scap"), &bytes).unwrap();
    std::fs::write(dir.join("broken.failed.scap"), &bytes).unwrap();
    assert_eq!(
        newest_flat_scap(&dir).unwrap(),
        None,
        "run-dir captures and failed captures are not live candidates"
    );

    let old = dir.join("older.scap");
    let new = dir.join("newer.scap");
    std::fs::write(&old, &bytes).unwrap();
    std::fs::write(&new, &bytes).unwrap();
    let past = std::time::SystemTime::now() - std::time::Duration::from_secs(3600);
    let times = std::fs::FileTimes::new().set_modified(past);
    std::fs::File::options()
        .append(true)
        .open(&old)
        .unwrap()
        .set_times(times)
        .unwrap();
    assert_eq!(newest_flat_scap(&dir).unwrap(), Some(new));

    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn capture_name_validation() {
    assert!(valid_capture_name("live_20260710_193000.scap"));
    assert!(valid_capture_name("capture-1.scap"));
    assert!(!valid_capture_name("x.failed.scap"));
    assert!(!valid_capture_name(".scap"));
    assert!(!valid_capture_name("../etc/passwd.scap"));
    assert!(!valid_capture_name("nested/file.scap"));
    assert!(!valid_capture_name("noext"));
}
