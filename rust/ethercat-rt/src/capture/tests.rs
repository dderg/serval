use std::fs::File;
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
        accel_cmd: n as f32 + 0.5,
        vel_cmd: n as f32 - 0.25,
        pin_res_re: n as f32 + 0.125,
        pin_res_im: n as f32 - 0.375,
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
        invert: false,
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
        accel_cmd: 1234.5 + seed as f32,
        vel_cmd: -67.25 + seed as f32,
        pin_res_re: 12.5 + seed as f32,
        pin_res_im: -8.25 + seed as f32,
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
    assert_eq!(&block[28..32], &d.accel_cmd.to_le_bytes());
    assert_eq!(&block[32..36], &d.vel_cmd.to_le_bytes());
    assert_eq!(&block[36..40], &d.pin_res_re.to_le_bytes());
    assert_eq!(&block[40..44], &d.pin_res_im.to_le_bytes());
}

#[test]
fn record_encodes_to_fixed_little_endian_layout() {
    let d = distinct_sample(0);
    let mut r = record_n(0x0102030405060708, &[d]);
    r.skip_count = 3;
    r.late_frames = 9;
    r.frame_lateness_ns = -104_000;
    let (b, size) = encode_record(&r);
    assert_eq!(size, 65);
    assert_eq!(&b[0..8], &0x0102030405060708u64.to_le_bytes());
    assert_eq!(b[8], FLAG_TORQUE_ENABLED | FLAG_MOTION_ACTIVE);
    assert_eq!(&b[9..13], &3u32.to_le_bytes());
    assert_eq!(&b[13..17], &9u32.to_le_bytes());
    assert_eq!(&b[17..21], &(-104_000i32).to_le_bytes());
    assert_drive_block(&b[21..65], &d);
}

#[test]
fn single_drive_record_matches_the_documented_layout() {
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
        accel_cmd: 1234.5,
        vel_cmd: -67.25,
        pin_res_re: 3.5,
        pin_res_im: -1.75,
    };
    let mut r = CaptureRecord::new(0x0102030405060708, 0x03);
    r.skip_count = 1;
    r.late_frames = 2;
    r.frame_lateness_ns = -5;
    r.drive_count = 1;
    r.drives[0] = d;

    let mut expected = [0u8; 65];
    expected[0..8].copy_from_slice(&0x0102030405060708u64.to_le_bytes());
    expected[8] = 0x03;
    expected[9..13].copy_from_slice(&1u32.to_le_bytes());
    expected[13..17].copy_from_slice(&2u32.to_le_bytes());
    expected[17..21].copy_from_slice(&(-5i32).to_le_bytes());
    expected[21..25].copy_from_slice(&(-2i32).to_le_bytes());
    expected[25..29].copy_from_slice(&(-1i32).to_le_bytes());
    expected[29..33].copy_from_slice(&5i32.to_le_bytes());
    expected[33..35].copy_from_slice(&(-300i16).to_le_bytes());
    expected[35..37].copy_from_slice(&0x0627u16.to_le_bytes());
    expected[37..39].copy_from_slice(&0x7380u16.to_le_bytes());
    expected[39..43].copy_from_slice(&(-654321i32).to_le_bytes());
    expected[43..45].copy_from_slice(&250i16.to_le_bytes());
    expected[45..49].copy_from_slice(&(0x55667788u32 as i32).to_le_bytes());
    expected[49..53].copy_from_slice(&1234.5f32.to_le_bytes());
    expected[53..57].copy_from_slice(&(-67.25f32).to_le_bytes());
    expected[57..61].copy_from_slice(&3.5f32.to_le_bytes());
    expected[61..65].copy_from_slice(&(-1.75f32).to_le_bytes());

    let (b, size) = encode_record(&r);
    assert_eq!(size, 65);
    assert_eq!(&b[..size], &expected[..]);
}

#[test]
fn two_drive_record_packs_blocks_back_to_back() {
    let d0 = distinct_sample(0);
    let d1 = distinct_sample(11);
    let r = record_n(7, &[d0, d1]);
    let (b, size) = encode_record(&r);
    assert_eq!(size, 21 + 2 * 44);
    assert_eq!(&b[0..8], &7u64.to_le_bytes());
    assert_drive_block(&b[21..65], &d0);
    assert_drive_block(&b[65..109], &d1);
}

#[test]
fn header_is_one_json_line_describing_the_record() {
    let path = tmp_path("hdr");
    let h = header_json(&cfg(&path));
    assert!(h.ends_with('\n'));
    assert_eq!(h.lines().count(), 1);
    for needle in [
        "\"version\":2",
        "\"cycle_ns\":1000000",
        "\"record_size\":65",
        "\"started_utc\":\"2026-06-10T12:00:00Z\"",
        "\"started_mono_ns\":7",
        "\"name\":\"x\"",
        "\"counts_per_mm\":3276.8",
        "\"rotation_distance\":40",
        "\"invert\":false",
        "{\"name\":\"cycle_index\",\"dtype\":\"u64\",\"offset\":0}",
        "{\"name\":\"flags\",\"dtype\":\"u8\",\"offset\":8}",
        "{\"name\":\"skip_count\",\"dtype\":\"u32\",\"offset\":9}",
        "{\"name\":\"late_frames\",\"dtype\":\"u32\",\"offset\":13}",
        "{\"name\":\"frame_lateness_ns\",\"dtype\":\"i32\",\"offset\":17}",
        "{\"name\":\"target_counts\",\"dtype\":\"i32\",\"offset\":21}",
        "{\"name\":\"position_actual\",\"dtype\":\"i32\",\"offset\":25}",
        "{\"name\":\"following_error\",\"dtype\":\"i32\",\"offset\":29}",
        "{\"name\":\"torque_actual\",\"dtype\":\"i16\",\"offset\":33}",
        "{\"name\":\"statusword\",\"dtype\":\"u16\",\"offset\":35}",
        "{\"name\":\"error_code\",\"dtype\":\"u16\",\"offset\":37}",
        "{\"name\":\"velocity_offset\",\"dtype\":\"i32\",\"offset\":39}",
        "{\"name\":\"torque_offset\",\"dtype\":\"i16\",\"offset\":43}",
        "{\"name\":\"velocity_actual\",\"dtype\":\"i32\",\"offset\":45}",
        "{\"name\":\"accel_cmd\",\"dtype\":\"f32\",\"offset\":49}",
        "{\"name\":\"vel_cmd\",\"dtype\":\"f32\",\"offset\":53}",
        "{\"name\":\"pin_res_re\",\"dtype\":\"f32\",\"offset\":57}",
        "{\"name\":\"pin_res_im\",\"dtype\":\"f32\",\"offset\":61}",
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
    assert!(header.contains("\"version\":2"));
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
        assert_drive_block(&rec[21..65], a);
        assert_drive_block(&rec[65..109], b);
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

#[test]
fn start_consumes_a_prebuilt_record_channel_and_replenishes() {
    let path1 = tmp_path("spare-a");
    let path2 = tmp_path("spare-b");
    let mut c = Capture::with_capacity(8);
    assert_eq!(c.start(cfg(&path1)), 0);
    c.push(record(1));
    assert_eq!(c.stop().result, 0);
    assert_eq!(c.start(cfg(&path2)), 0, "replenished channel must be ready");
    c.push(record(2));
    assert_eq!(c.stop().result, 0);
    let _ = std::fs::remove_file(&path1);
    let _ = std::fs::remove_file(&path2);
}

#[test]
fn start_without_spare_channel_fails_loudly_not_allocating() {
    let path = tmp_path("spare-drained");
    let mut c = Capture::with_capacity(8);
    let _drained = c.spare_channels.recv().expect("initial spare present");
    assert_eq!(c.start(cfg(&path)), ERR_CAPTURE_CHANNEL_NOT_READY);
    assert!(!c.is_active());
    let _ = std::fs::remove_file(&path);
}

#[test]
fn failed_validation_does_not_consume_the_spare_channel() {
    let path = tmp_path("spare-kept");
    let mut c = Capture::with_capacity(8);
    assert_eq!(
        c.start(cfg_drives(&path, vec![])),
        ERR_CAPTURE_BAD_DRIVE_LIST
    );
    assert_eq!(
        c.start(cfg(&path)),
        0,
        "spare must survive a rejected start"
    );
    c.push(record(1));
    assert_eq!(c.stop().result, 0);
    let _ = std::fs::remove_file(&path);
}
/// Little-endian bytes of the zstd magic number 0xFD2FB528.
const ZSTD_MAGIC: [u8; 4] = [0x28, 0xB5, 0x2F, 0xFD];

/// The `.scap.zst` sibling of a `tmp_path` tag.
fn tmp_zst_path(tag: &str) -> PathBuf {
    tmp_path(tag).with_extension("scap.zst")
}

/// The exact byte stream today's raw writer would emit for `cfg` + `records`:
/// the JSON header line followed by each fixed-layout record, back to back.
fn expected_raw_bytes(cfg: &CaptureConfig, records: &[CaptureRecord]) -> Vec<u8> {
    let mut want = header_json(cfg).into_bytes();
    for r in records {
        let (buf, size) = encode_record(r);
        want.extend_from_slice(&buf[..size]);
    }
    want
}

/// Drive a full start/push/stop capture and return the on-disk bytes.
fn capture_to(path: &Path, records: &[CaptureRecord]) -> Vec<u8> {
    let _ = std::fs::remove_file(path);
    let mut cap = Capture::new();
    assert_eq!(cap.start(cfg(path)), 0);
    for r in records {
        cap.push(*r);
    }
    let out = cap.stop();
    assert_eq!(out.result, 0);
    assert_eq!(out.samples, records.len() as u64);
    let bytes = std::fs::read(path).unwrap();
    std::fs::remove_file(path).unwrap();
    bytes
}

#[test]
fn raw_scap_is_byte_identical_to_the_documented_writer_format() {
    let path = tmp_path("raw-identical");
    let records: Vec<CaptureRecord> = (0..50u64).map(record).collect();
    let bytes = capture_to(&path, &records);
    assert_eq!(bytes, expected_raw_bytes(&cfg(&path), &records));
}

#[test]
fn zst_scap_decodes_to_exactly_the_raw_writer_bytes() {
    let records: Vec<CaptureRecord> = (0..50u64).map(record).collect();

    let raw_path = tmp_path("zst-vs-raw");
    let raw_bytes = capture_to(&raw_path, &records);

    let zst_path = tmp_zst_path("zst-vs-raw");
    let zbytes = capture_to(&zst_path, &records);

    assert_eq!(
        &zbytes[..4],
        &ZSTD_MAGIC,
        "compressed capture is a zstd frame"
    );
    let decoded = zstd::decode_all(&zbytes[..]).expect("valid zstd frame");
    assert_eq!(
        decoded, raw_bytes,
        "zst stream decodes to the raw writer bytes"
    );
}

#[test]
fn failed_capture_on_zst_path_keeps_the_zst_name() {
    let path = tmp_zst_path("zst-fail");
    let _ = std::fs::remove_file(&path);
    let failed = super::failed_capture_path(&path);
    let _ = std::fs::remove_file(&failed);
    assert_eq!(
        failed.file_name().unwrap().to_str().unwrap(),
        format!(
            "kalico-capture-zst-fail-{}.failed.scap.zst",
            std::process::id()
        ),
        "renamed capture preserves the .scap.zst suffix"
    );

    let (gate_tx, gate_rx) = sync_channel::<()>(1);
    let mut cap = Capture::with_capacity(4);
    assert_eq!(cap.start_gated(cfg(&path), gate_rx), 0);
    for i in 0..10u64 {
        cap.push(record(i));
    }
    gate_tx.send(()).unwrap();
    let out = cap.stop();
    assert_eq!(out.result, ERR_CAPTURE_OVERFLOW);
    assert!(!path.exists(), "failed capture must not keep the live name");
    assert!(
        failed.exists(),
        "failed capture renamed with .zst preserved"
    );
    std::fs::remove_file(&failed).unwrap();
}

#[test]
fn encoder_error_surfaces_as_capture_file_error() {
    // A read-only handle to the .zst target makes the encoder's underlying
    // flush fail on finalize; the failure must map to the capture-file path.
    let path = tmp_zst_path("zst-encerr");
    let _ = std::fs::remove_file(&path);
    File::create(&path).unwrap();
    let ro = File::open(&path).unwrap();

    let (mut tx, rx) = rtrb::RingBuffer::new(4);
    tx.push(record(0)).unwrap();
    drop(tx);

    let written = super::run_session(ro, &path, header_json(&cfg(&path)), WriterHook::None, rx);
    assert!(
        written.is_err(),
        "encoder finalize on a read-only file must fail"
    );

    let outcome = super::compose_outcome(&path, written, None);
    assert_eq!(outcome.result, ERR_CAPTURE_FILE);
    let failed = super::failed_capture_path(&path);
    let _ = std::fs::remove_file(&failed);
    let _ = std::fs::remove_file(&path);
}
