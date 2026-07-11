use super::{
    PairSync, SyncInputs, SyncParams, SyncStep, ERR_SYNC_FINAL_TORQUE, ERR_SYNC_SETTLE_TIMEOUT,
    ERR_SYNC_TORQUE_RESIDUAL,
};

const CYCLE_NS: u64 = 250_000;
const CPM: f64 = 3276.8;

fn params() -> SyncParams {
    SyncParams {
        torque_ok_tenth_pct: 30,
        settle_timeout_cycles: 4000,
        measure_cycles: 8,
        quiet_cycles: 4,
        dither_amplitude_nm: 100_000,
        dither_freq_millihz: 4_000,
        dither_duration_ms: 200,
    }
}

struct Bench {
    sync: PairSync,
    now_ns: u64,
    torque_primary: i16,
    torque_secondary: i16,
    drift_counts_per_cycle: i32,
    position_secondary: i32,
    primary_targets: Vec<i32>,
    secondary_enabled: bool,
}

impl Bench {
    fn new(base_target: i32) -> Self {
        Bench {
            sync: PairSync::begin(params(), CPM, CPM, base_target).unwrap(),
            now_ns: 1_000_000_000,
            torque_primary: 0,
            torque_secondary: 0,
            drift_counts_per_cycle: 0,
            position_secondary: 0,
            primary_targets: Vec::new(),
            secondary_enabled: true,
        }
    }

    /// Run cycles until the machine emits a non-Idle, non-SetPrimaryTarget
    /// step (recording targets on the way), or panic after a bound.
    fn run_until_step(&mut self) -> SyncStep {
        for _ in 0..2_000_000 {
            self.now_ns += CYCLE_NS;
            self.position_secondary = self
                .position_secondary
                .wrapping_add(self.drift_counts_per_cycle);
            let step = self.sync.poll(&SyncInputs {
                now_ns: self.now_ns,
                torque_primary: self.torque_primary,
                torque_secondary: self.torque_secondary,
                position_secondary: self.position_secondary,
            });
            match step {
                SyncStep::Idle => {}
                SyncStep::SetPrimaryTarget(c) => self.primary_targets.push(c),
                other => return other,
            }
        }
        panic!("sync machine made no progress");
    }

    fn do_disable(&mut self, step: SyncStep) {
        assert_eq!(step, SyncStep::DisableSecondary);
        self.secondary_enabled = false;
    }

    fn do_enable(&mut self, step: SyncStep) {
        assert_eq!(step, SyncStep::EnableSecondary);
        self.secondary_enabled = true;
        self.sync.enable_finished(self.position_secondary);
    }
}

#[test]
fn happy_path_reports_all_phases_and_released_delta() {
    let mut b = Bench::new(50_000);
    b.torque_primary = 80;
    b.torque_secondary = -78;
    b.position_secondary = 20_000;

    let step = b.run_until_step();
    b.do_disable(step);

    // Coast: rotor unwinds 400 counts over a few cycles, then goes quiet;
    // primary keeps a stiction residual.
    for _ in 0..3 {
        b.now_ns += CYCLE_NS;
        b.position_secondary += 133;
        assert_eq!(
            b.sync.poll(&SyncInputs {
                now_ns: b.now_ns,
                torque_primary: b.torque_primary,
                torque_secondary: 0,
                position_secondary: b.position_secondary,
            }),
            SyncStep::Idle
        );
    }
    b.position_secondary = 20_400;
    b.torque_primary = 45;
    b.torque_secondary = 0;

    // Dither breaks the stiction: torque collapses once targets start.
    let step = loop {
        b.now_ns += CYCLE_NS;
        let step = b.sync.poll(&SyncInputs {
            now_ns: b.now_ns,
            torque_primary: b.torque_primary,
            torque_secondary: b.torque_secondary,
            position_secondary: b.position_secondary,
        });
        match step {
            SyncStep::Idle => {}
            SyncStep::SetPrimaryTarget(c) => {
                b.primary_targets.push(c);
                b.torque_primary = 4;
                b.position_secondary = 20_500;
            }
            other => break other,
        }
    };
    assert!(
        !b.primary_targets.is_empty(),
        "dither must command primary targets"
    );
    assert_eq!(
        *b.primary_targets.last().unwrap(),
        50_000,
        "dither must end exactly at the base hold target"
    );

    b.position_secondary = 20_512;
    b.do_enable(step);
    b.torque_primary = 3;
    b.torque_secondary = -2;

    let SyncStep::Done(report) = b.run_until_step() else {
        panic!("expected Done");
    };
    assert_eq!(report.result, 0);
    assert_eq!(report.torque_baseline_primary, 80);
    assert_eq!(report.torque_baseline_secondary, -78);
    assert_eq!(report.torque_released, 45);
    assert_eq!(report.torque_dithered, 4);
    assert_eq!(report.torque_final_primary, 3);
    assert_eq!(report.torque_final_secondary, -2);
    assert_eq!(report.released_delta_counts, 20_512 - 20_000);
    assert!(report.secondary_reseeded);
}

#[test]
fn coast_settle_timeout_reenables_and_fails() {
    let mut b = Bench::new(0);
    let step = b.run_until_step();
    b.do_disable(step);

    // Never goes quiet: keeps creeping more than the quiet band each cycle.
    b.drift_counts_per_cycle = 100;
    let step = b.run_until_step();
    b.do_enable(step);

    let SyncStep::Done(report) = b.run_until_step() else {
        panic!("expected Done");
    };
    assert_eq!(report.result, ERR_SYNC_SETTLE_TIMEOUT);
    assert!(report.secondary_reseeded);
}

#[test]
fn creep_inside_the_quiet_band_settles() {
    let mut b = Bench::new(0);
    let step = b.run_until_step();
    b.do_disable(step);

    // 8 counts/cycle stays within the 33-count quiet band across the 4-cycle
    // quiet window — encoder-noise-scale motion must not block settling.
    b.drift_counts_per_cycle = 8;
    let step = b.run_until_step();
    b.drift_counts_per_cycle = 0;
    b.do_enable(step);

    let SyncStep::Done(report) = b.run_until_step() else {
        panic!("expected Done");
    };
    assert_eq!(report.result, 0);
}

#[test]
fn residual_torque_after_dither_reenables_and_fails() {
    let mut b = Bench::new(0);
    b.torque_primary = 90;
    let step = b.run_until_step();
    b.do_disable(step);

    // Quiet immediately, but the primary torque never collapses — mechanical
    // binding the dither could not shake loose.
    let step = b.run_until_step();
    b.do_enable(step);

    let SyncStep::Done(report) = b.run_until_step() else {
        panic!("expected Done");
    };
    assert_eq!(report.result, ERR_SYNC_TORQUE_RESIDUAL);
    assert_eq!(report.torque_dithered, 90);
    assert!(report.secondary_reseeded);
}

#[test]
fn final_torque_above_threshold_fails_but_reseeds() {
    let mut b = Bench::new(0);
    let step = b.run_until_step();
    b.do_disable(step);

    b.torque_primary = 0;
    let step = b.run_until_step();
    b.do_enable(step);

    // Re-enabled pair immediately fights again.
    b.torque_primary = 60;
    b.torque_secondary = -55;
    let SyncStep::Done(report) = b.run_until_step() else {
        panic!("expected Done");
    };
    assert_eq!(report.result, ERR_SYNC_FINAL_TORQUE);
    assert_eq!(report.torque_final_primary, 60);
    assert_eq!(report.torque_final_secondary, -55);
    assert!(report.secondary_reseeded);
}

#[test]
fn dither_amplitude_reaches_commanded_scale() {
    let mut b = Bench::new(10_000);
    let step = b.run_until_step();
    b.do_disable(step);
    let step = b.run_until_step();
    b.do_enable(step);
    let max_excursion = b
        .primary_targets
        .iter()
        .map(|c| (c - 10_000).abs())
        .max()
        .unwrap();
    // 100 um at 3276.8 counts/mm ≈ 328 counts peak.
    assert!(
        (200..=400).contains(&max_excursion),
        "dither excursion {max_excursion} counts outside expected band"
    );
}
