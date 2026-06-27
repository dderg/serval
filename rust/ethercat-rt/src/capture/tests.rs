use std::path::{Path, PathBuf};
use std::sync::mpsc::sync_channel;

use super::*;

fn sample(n: i32) -> DriveSample {
    DriveSample {
        target_counts: n,
        position_actual: n + 2,
        velocity_actual: n + 4,
        following_error: -3,
        torque_actual: 42,
        statusword: 0x0627,
        error_code: 0,
        velocity_offset: n + 3,
        torque_offset: -7,
    }
}

fn record_n(cycle: u64, samples: &[DriveSample]) -> CaptureRecord {
    let mut r = CaptureRecord::new(cycle, FLAG_TORQUE_ENABLED | FLAG_MOTION_ACTIVE);
    r.drive_count = samples.len() as u8;
    r.drives[..samples.len()].copy_from_slice(samples);
    r
}

fn record(cycle: u64) -> CaptureRecord {
    record_n(cycle, &[sample(1000)])
}

fn drive_cfg(slot: u8, name: &str) -> CaptureDriveConfig {
    CaptureDriveConfig {
        slot,
        name: name.to_owned(),
        counts_per_mm: 3276.8,
        rotation_distance: 40.0,
    }
}

fn cfg_drives(path: &Path, drives: Vec<CaptureDriveConfig>) -> CaptureConfig {
    CaptureConfig {
        path: path.to_str().unwrap().to_owned(),
        started_utc: "2026-06-10T12:00:00Z".to_owned(),
        drives,
        cycle_ns: 1_000_000,
        started_mono_ns: 7,
    }
}

fn cfg(path: &Path) -> CaptureConfig {
    cfg_drives(path, vec![drive_cfg(0, "x")])
}

fn tmp_path(tag: &str) -> PathBuf {
    std::env::temp_dir().join(format!(
        "kalico-capture-{}-{}.scap",
        tag,
        std::process::id()
    ))
}

fn distinct_sample(seed: i32) -> DriveSample {
    DriveSample {
        target_counts: -2 + seed,
        position_actual: -1 + seed,
        velocity_actual: (0x55667788u32 as i32).wrapping_add(seed),
        following_error: 5 + seed,
        torque_actual: -300i16.wrapping_add(seed as i16),
        statusword: 0x0627,
        error_code: 0x7380,
        velocity_offset: -654321 + seed,
        torque_offset: 250i16.wrapping_add(seed as i16),
    }
}

fn assert_drive_block(block: &[u8], d: &DriveSample) {
    assert_eq!(&block[0..4], &d.target_counts.to_le_bytes());
    assert_eq!(&block[4..8], &d.position_actual.to_le_bytes());
    assert_eq!(&block[8..12], &d.following_error.to_le_bytes());
    assert_eq!(&block[12..14], &d.torque_actual.to_le_bytes());
    assert_eq!(&block[14..16], &d.statusword.to_le_bytes());
    assert_eq!(&block[16..18], &d.error_code.to_le_bytes());
    assert_eq!(&block[18..22], &d.velocity_offset.to_le_bytes());
    assert_eq!(&block[22..24], &d.torque_offset.to_le_bytes());
    assert_eq!(&block[24..28], &d.velocity_actual.to_le_bytes());
}

#[test]
fn record_encodes_to_fixed_little_endian_layout() {
    let d = distinct_sample(0);
    let r = record_n(0x0102030405060708, &[d]);
    let (b, size) = encode_record(&r);
    assert_eq!(size, 37);
    assert_eq!(&b[0..8], &0x0102030405060708u64.to_le_bytes());
    assert_eq!(b[8], FLAG_TORQUE_ENABLED | FLAG_MOTION_ACTIVE);
    assert_drive_block(&b[9..37], &d);
}

#[test]
fn single_drive_record_is_byte_identical_to_pre_change_layout() {
    let d = DriveSample {
        target_counts: -2,
        position_actual: -1,
        velocity_actual: 0x55667788u32 as i32,
        following_error: 5,
        torque_actual: -300,
        statusword: 0x0627,
        error_code: 0x7380,
        velocity_offset: -654321,
        torque_offset: 250,
    };
    let mut r = CaptureRecord::new(0x0102030405060708, 0x03);
    r.drive_count = 1;
    r.drives[0] = d;

    let mut expected = [0u8; 37];
    expected[0..8].copy_from_slice(&0x0102030405060708u64.to_le_bytes());
    expected[8] = 0x03;
    expected[9..13].copy_from_slice(&(-2i32).to_le_bytes());
    expected[13..17].copy_from_slice(&(-1i32).to_le_bytes());
    expected[17..21].copy_from_slice(&5i32.to_le_bytes());
    expected[21..23].copy_from_slice(&(-300i16).to_le_bytes());
    expected[23..25].copy_from_slice(&0x0627u16.to_le_bytes());
    expected[25..27].copy_from_slice(&0x7380u16.to_le_bytes());
    expected[27..31].copy_from_slice(&(-654321i32).to_le_bytes());
    expected[31..33].copy_from_slice(&250i16.to_le_bytes());
    expected[33..37].copy_from_slice(&(0x55667788u32 as i32).to_le_bytes());

    let (b, size) = encode_record(&r);
    assert_eq!(size, 37);
    assert_eq!(&b[..size], &expected[..]);
}

#[test]
fn two_drive_record_packs_blocks_back_to_back() {
    let d0 = distinct_sample(0);
    let d1 = distinct_sample(11);
    let r = record_n(7, &[d0, d1]);
    let (b, size) = encode_record(&r);
    assert_eq!(size, 9 + 2 * 28);
    assert_eq!(&b[0..8], &7u64.to_le_bytes());
    assert_drive_block(&b[9..37], &d0);
    assert_drive_block(&b[37..65], &d1);
}

#[test]
fn header_is_one_json_line_describing_the_record() {
    let path = tmp_path("hdr");
    let h = header_json(&cfg(&path));
    assert!(h.ends_with('\n'));
    assert_eq!(h.lines().count(), 1);
    for needle in [
        "\"version\":1",
        "\"cycle_ns\":1000000",
        "\"record_size\":37",
        "\"started_utc\":\"2026-06-10T12:00:00Z\"",
        "\"started_mono_ns\":7",
        "\"name\":\"x\"",
        "\"counts_per_mm\":3276.8",
        "\"rotation_distance\":40",
        "{\"name\":\"cycle_index\",\"dtype\":\"u64\",\"offset\":0}",
        "{\"name\":\"flags\",\"dtype\":\"u8\",\"offset\":8}",
        "{\"name\":\"target_counts\",\"dtype\":\"i32\",\"offset\":9}",
        "{\"name\":\"position_actual\",\"dtype\":\"i32\",\"offset\":13}",
        "{\"name\":\"following_error\",\"dtype\":\"i32\",\"offset\":17}",
        "{\"name\":\"torque_actual\",\"dtype\":\"i16\",\"offset\":21}",
        "{\"name\":\"statusword\",\"dtype\":\"u16\",\"offset\":23}",
        "{\"name\":\"error_code\",\"dtype\":\"u16\",\"offset\":25}",
        "{\"name\":\"velocity_offset\",\"dtype\":\"i32\",\"offset\":27}",
        "{\"name\":\"torque_offset\",\"dtype\":\"i16\",\"offset\":31}",
        "{\"name\":\"velocity_actual\",\"dtype\":\"i32\",\"offset\":33}",
    ] {
        assert!(h.contains(needle), "header missing {needle}: {h}");
    }
}

#[test]
fn lifecycle_start_push_stop_produces_parseable_file() {
    let path = tmp_path("happy");
    let _ = std::fs::remove_file(&path);
    let mut cap = Capture::new();
    assert!(!cap.is_active());
    assert_eq!(cap.start(cfg(&path)), 0);
    assert!(cap.is_active());
    for i in 0..50u64 {
        cap.push(record(i));
    }
    let out = cap.stop();
    assert_eq!(out.result, 0);
    assert_eq!(out.samples, 50);
    assert_eq!(out.overflow_cycle, None);
    assert!(!cap.is_active());

    let bytes = std::fs::read(&path).unwrap();
    let nl = bytes.iter().position(|&b| b == b'\n').unwrap();
    let header = std::str::from_utf8(&bytes[..nl]).unwrap();
    assert!(header.contains("\"version\":1"));
    let body = &bytes[nl + 1..];
    let rsize = record_size(1);
    assert_eq!(body.len(), 50 * rsize);
    let (buf0, size0) = encode_record(&record(0));
    assert_eq!(&body[..rsize], &buf0[..size0]);
    std::fs::remove_file(&path).unwrap();
}

#[test]
fn multi_drive_round_trip_writes_two_blocks_per_record() {
    let path = tmp_path("multi");
    let _ = std::fs::remove_file(&path);
    let mut cap = Capture::new();
    let cfg = cfg_drives(
        &path,
        vec![drive_cfg(0, "motor_a"), drive_cfg(3, "motor_b")],
    );
    assert_eq!(cap.start(cfg), 0);
    let samples: Vec<(DriveSample, DriveSample)> = (0..20u64)
        .map(|i| (distinct_sample(i as i32), distinct_sample(100 + i as i32)))
        .collect();
    for (i, (a, b)) in samples.iter().enumerate() {
        cap.push(record_n(i as u64, &[*a, *b]));
    }
    let out = cap.stop();
    assert_eq!(out.result, 0);
    assert_eq!(out.samples, 20);

    let bytes = std::fs::read(&path).unwrap();
    let nl = bytes.iter().position(|&b| b == b'\n').unwrap();
    let header = std::str::from_utf8(&bytes[..nl]).unwrap();
    let rsize = record_size(2);
    assert!(header.contains(&format!("\"record_size\":{rsize}")));
    assert!(header.contains("\"name\":\"motor_a\""));
    assert!(header.contains("\"name\":\"motor_b\""));

    let body = &bytes[nl + 1..];
    assert_eq!(body.len(), 20 * rsize);
    for (i, (a, b)) in samples.iter().enumerate() {
        let rec = &body[i * rsize..(i + 1) * rsize];
        assert_eq!(&rec[0..8], &(i as u64).to_le_bytes());
        assert_drive_block(&rec[9..37], a);
        assert_drive_block(&rec[37..65], b);
    }
    std::fs::remove_file(&path).unwrap();
}

#[test]
fn empty_drive_list_rejected_before_touching_disk() {
    let path = tmp_path("emptylist");
    let _ = std::fs::remove_file(&path);
    let mut cap = Capture::new();
    let cfg = cfg_drives(&path, vec![]);
    assert_eq!(cap.start(cfg), ERR_CAPTURE_BAD_DRIVE_LIST);
    assert!(!cap.is_active());
    assert!(!path.exists());
}

#[test]
fn duplicate_slot_rejected_before_touching_disk() {
    let path = tmp_path("duplist");
    let _ = std::fs::remove_file(&path);
    let mut cap = Capture::new();
    let cfg = cfg_drives(&path, vec![drive_cfg(1, "a"), drive_cfg(1, "b")]);
    assert_eq!(cap.start(cfg), ERR_CAPTURE_BAD_DRIVE_LIST);
    assert!(!cap.is_active());
    assert!(!path.exists());
}

#[test]
fn double_start_rejected_and_capture_survives() {
    let path = tmp_path("dbl");
    let _ = std::fs::remove_file(&path);
    let mut cap = Capture::new();
    assert_eq!(cap.start(cfg(&path)), 0);
    assert_eq!(cap.start(cfg(&path)), ERR_CAPTURE_ACTIVE);
    assert!(cap.is_active());
    let out = cap.stop();
    assert_eq!(out.result, 0);
    std::fs::remove_file(&path).unwrap();
}

#[test]
fn stop_without_start_rejected() {
    let mut cap = Capture::new();
    let out = cap.stop();
    assert_eq!(out.result, ERR_CAPTURE_NOT_ACTIVE);
    assert_eq!(out.samples, 0);
}

#[test]
fn unwritable_path_fails_start() {
    let mut cap = Capture::new();
    let c = cfg(&PathBuf::from("/dev/null/nope/x.scap"));
    assert_eq!(cap.start(c), ERR_CAPTURE_FILE);
    assert!(!cap.is_active());
}

#[test]
fn quote_in_drive_name_rejected_before_touching_disk() {
    let path = tmp_path("badname");
    let mut cap = Capture::new();
    let mut c = cfg(&path);
    c.drives[0].name = "x\"evil".to_owned();
    assert_eq!(cap.start(c), ERR_CAPTURE_BAD_ARG);
    assert!(!path.exists());
}

#[test]
fn overflow_kills_capture_and_renames_file() {
    let path = tmp_path("ovf");
    let _ = std::fs::remove_file(&path);
    let failed = path.with_extension("failed.scap");
    let _ = std::fs::remove_file(&failed);

    let (gate_tx, gate_rx) = sync_channel::<()>(1);
    let mut cap = Capture::with_capacity(4);
    assert_eq!(cap.start_gated(cfg(&path), gate_rx), 0);
    for i in 0..10u64 {
        cap.push(record(i));
    }
    gate_tx.send(()).unwrap();
    let out = cap.stop();
    assert_eq!(out.result, ERR_CAPTURE_OVERFLOW);
    assert_eq!(out.overflow_cycle, Some(4));
    assert_eq!(out.samples, 4);
    assert!(!path.exists(), "failed capture must not keep .scap name");
    assert!(failed.exists(), "failed capture must be renamed");
    std::fs::remove_file(&failed).unwrap();
}

#[test]
fn pushes_after_overflow_are_ignored() {
    let path = tmp_path("ovf2");
    let _ = std::fs::remove_file(&path);
    let (gate_tx, gate_rx) = sync_channel::<()>(1);
    let mut cap = Capture::with_capacity(2);
    assert_eq!(cap.start_gated(cfg(&path), gate_rx), 0);
    for i in 0..100u64 {
        cap.push(record(i));
    }
    gate_tx.send(()).unwrap();
    let out = cap.stop();
    assert_eq!(
        out.overflow_cycle,
        Some(2),
        "first refused cycle is recorded"
    );
    let failed = path.with_extension("failed.scap");
    std::fs::remove_file(&failed).unwrap();
}

#[test]
fn writer_death_latches_file_error() {
    let path = tmp_path("wdeath");
    let _ = std::fs::remove_file(&path);
    let failed = path.with_extension("failed.scap");
    let _ = std::fs::remove_file(&failed);

    let mut cap = Capture::with_capacity(4);
    let (start_result, writer_done) = cap.start_writer_fails(cfg(&path));
    assert_eq!(start_result, 0);

    writer_done
        .recv()
        .expect("writer must signal done before failing");

    for i in 0..3u64 {
        cap.push(record(i));
    }
    let out = cap.stop();
    assert_eq!(out.result, ERR_CAPTURE_FILE);
    assert!(!path.exists(), "failed capture must not keep .scap name");
    assert!(failed.exists(), "failed capture must be renamed");
    std::fs::remove_file(&failed).unwrap();
}

#[test]
fn stop_async_returns_while_writer_is_still_finalizing() {
    let path = tmp_path("async");
    let _ = std::fs::remove_file(&path);
    let (gate_tx, gate_rx) = sync_channel::<()>(1);
    let mut cap = Capture::new();
    assert_eq!(cap.start_gated(cfg(&path), gate_rx), 0);
    for i in 0..20u64 {
        cap.push(record(i));
    }

    let started = std::time::Instant::now();
    let pending = cap.stop_async();
    assert!(
        started.elapsed() < std::time::Duration::from_millis(20),
        "stop_async must not wait for the writer"
    );
    assert!(
        pending.try_take().is_none(),
        "outcome must not exist while the writer is gated"
    );
    assert!(!cap.is_active(), "capture slot frees immediately");

    gate_tx.send(()).unwrap();
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    let out = loop {
        if let Some(out) = pending.try_take() {
            break out;
        }
        assert!(
            std::time::Instant::now() < deadline,
            "finalizer never completed"
        );
        std::thread::sleep(std::time::Duration::from_millis(1));
    };
    assert_eq!(out.result, 0);
    assert_eq!(out.samples, 20);
    std::fs::remove_file(&path).unwrap();
}

#[test]
fn stop_async_without_start_resolves_not_active_immediately() {
    let mut cap = Capture::new();
    let out = cap.stop_async().try_take().expect("immediate outcome");
    assert_eq!(out.result, ERR_CAPTURE_NOT_ACTIVE);
}

#[test]
fn any_slot_out_of_range_flags_only_slots_at_or_above_count() {
    assert!(!any_slot_out_of_range(&[0, 1, 2], 3));
    assert!(any_slot_out_of_range(&[0, 3], 3));
    assert!(!any_slot_out_of_range(&[], 0));
    assert!(any_slot_out_of_range(&[0], 0));
}
