use bench_sim_logic::aux::{AuxRecord, AUX_RECORD_LEN};
use bench_sim_logic::config::SimConfig;
use bench_sim_logic::tracker::{AuxStamp, McoLost, Tracker, MCO_COUNTS_PER_MS};
use bench_sim_logic::walls::{Wall, Walls};

const TICKS_PER_AUX: u32 = 40;
const MCO_PER_TICK: u16 = (MCO_COUNTS_PER_MS / TICKS_PER_AUX as u64) as u16;

fn probe_config(origin_z: i32, expect_mco: bool) -> SimConfig {
    let mut walls = Walls::default();
    walls
        .push(Wall {
            pin: 3,
            active_high: true,
            at_or_below: true,
            coeffs: [0, 0, 1, 0],
            threshold: 0,
        })
        .unwrap();
    SimConfig {
        id: 1,
        origin: [0, 0, origin_z, 0],
        walls,
        expect_mco,
        ..SimConfig::default()
    }
}

#[test]
fn the_probe_pin_follows_z_from_the_origin() {
    let mut tracker = Tracker::start(&probe_config(3, false), TICKS_PER_AUX, 0);
    assert_eq!(tracker.levels(), 0);
    let mut mco = 0u16;
    let mut levels = Vec::new();
    for _ in 0..5 {
        mco = mco.wrapping_add(MCO_PER_TICK);
        levels.push(tracker.tick(&[0, 0, -1, 0], mco).unwrap().levels);
    }
    assert_eq!(levels, [0, 0, 0b1000, 0b1000, 0b1000]);
    assert_eq!(tracker.positions(), [0, 0, -2, 0]);
}

#[test]
fn a_start_already_past_a_wall_is_triggered_at_once() {
    let tracker = Tracker::start(&probe_config(-1, false), TICKS_PER_AUX, 0);
    assert_eq!(tracker.levels(), 0b1000);
}

#[test]
fn aux_stamps_come_every_period_with_the_extended_mco() {
    let mut tracker = Tracker::start(&probe_config(10, true), TICKS_PER_AUX, 60_000);
    let mut mco = 60_000u16;
    let mut stamps = Vec::new();
    for _ in 0..(3 * TICKS_PER_AUX) {
        mco = mco.wrapping_add(MCO_PER_TICK);
        if let Some(stamp) = tracker.tick(&[0; 4], mco).unwrap().aux {
            stamps.push(stamp);
        }
    }
    let expected: Vec<AuxStamp> = (1..=3)
        .map(|k| AuxStamp {
            mco: k * u64::from(MCO_PER_TICK) * u64::from(TICKS_PER_AUX),
            mco_ok: true,
        })
        .collect();
    assert_eq!(stamps, expected);
}

#[test]
fn a_missing_mco_is_only_fatal_when_expected() {
    let mut relaxed = Tracker::start(&probe_config(10, false), TICKS_PER_AUX, 0);
    let mut strict = Tracker::start(&probe_config(10, true), TICKS_PER_AUX, 0);
    for _ in 1..TICKS_PER_AUX {
        relaxed.tick(&[0; 4], 0).unwrap();
        strict.tick(&[0; 4], 0).unwrap();
    }
    let stamp = relaxed.tick(&[0; 4], 0).unwrap().aux.unwrap();
    assert!(!stamp.mco_ok);
    assert_eq!(strict.tick(&[0; 4], 0), Err(McoLost { delta: 0 }));
}

#[test]
fn the_aux_record_layout_is_stable() {
    let record = AuxRecord {
        time: 0x0102_0304_0506_0708,
        mco: 0x1112_1314,
        samples: 0x2122_2324,
        high: [0x31, 0x32, 0x33],
        temp_centi: [-1, 2_500],
        dac: [3_722, 4_000],
        config_id: 0x4142,
        endstops: 0b1001,
        flags: 0b011,
        dedge: 0b1111,
        tmc_writes: 5,
        tmc_reads: 6,
        tmc_rejects: 7,
    };
    let bytes = record.encode();
    assert_eq!(bytes.len(), AUX_RECORD_LEN);
    let hex = bytes
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect::<Vec<_>>()
        .concat();
    assert_eq!(
        hex,
        "0807060504030201141312112423222131000000320000003300000\
         0ffffc4098a0ea00f424109030f050607"
    );
}
