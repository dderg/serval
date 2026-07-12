use super::{
    SyncInputs, SyncParams, SyncRelease, SyncStep, ERR_SYNC_BAD_MASK, ERR_SYNC_FINAL_TORQUE,
    ERR_SYNC_SETTLE_TIMEOUT, MAX_RELEASE_SLOTS,
};

const CPM: f64 = 3276.8;

fn params() -> SyncParams {
    SyncParams {
        torque_ok_tenth_pct: 30,
        settle_timeout_cycles: 4000,
        measure_cycles: 8,
        quiet_cycles: 4,
    }
}

struct Bench {
    sync: SyncRelease,
    torque: [i16; MAX_RELEASE_SLOTS],
    position: [i32; MAX_RELEASE_SLOTS],
    drift_counts_per_cycle: [i32; MAX_RELEASE_SLOTS],
    enabled: bool,
}

impl Bench {
    fn new(slot_mask: u8) -> Self {
        Bench {
            sync: SyncRelease::begin(params(), slot_mask, [CPM; MAX_RELEASE_SLOTS]).unwrap(),
            torque: [0; MAX_RELEASE_SLOTS],
            position: [0; MAX_RELEASE_SLOTS],
            drift_counts_per_cycle: [0; MAX_RELEASE_SLOTS],
            enabled: true,
        }
    }

    /// Run cycles until the machine emits a non-Idle step, or panic after a
    /// bound.
    fn run_until_step(&mut self) -> SyncStep {
        for _ in 0..2_000_000 {
            for s in 0..MAX_RELEASE_SLOTS {
                self.position[s] = self.position[s].wrapping_add(self.drift_counts_per_cycle[s]);
            }
            let step = self.sync.poll(&SyncInputs {
                torque: self.torque,
                position: self.position,
            });
            if step != SyncStep::Idle {
                return step;
            }
        }
        panic!("sync machine made no progress");
    }

    fn do_disable(&mut self, step: SyncStep) {
        assert_eq!(step, SyncStep::DisableAll);
        self.enabled = false;
    }

    fn do_enable(&mut self, step: SyncStep) {
        assert_eq!(step, SyncStep::EnableAll);
        self.enabled = true;
        let position = self.position;
        self.sync.enable_finished(&position);
    }
}

#[test]
fn happy_path_reports_baselines_finals_and_released_deltas() {
    let mut b = Bench::new(0x0f);
    b.torque = [80, -78, 31, -29];
    b.position = [20_000, -20_000, 5_000, -5_000];

    let step = b.run_until_step();
    b.do_disable(step);

    // Coast: every rotor unwinds over a few cycles, then goes quiet.
    for _ in 0..3 {
        b.position[0] += 133;
        b.position[1] -= 133;
        b.position[2] += 20;
        b.position[3] -= 20;
        assert_eq!(
            b.sync.poll(&SyncInputs {
                torque: [0; MAX_RELEASE_SLOTS],
                position: b.position,
            }),
            SyncStep::Idle
        );
    }
    b.position = [20_512, -20_512, 5_060, -5_060];
    b.torque = [0; MAX_RELEASE_SLOTS];

    let step = b.run_until_step();
    b.do_enable(step);
    b.torque = [3, -2, 1, -1];

    let SyncStep::Done(report) = b.run_until_step() else {
        panic!("expected Done");
    };
    assert_eq!(report.result, 0);
    assert_eq!(report.torque_baseline, [80, -78, 31, -29]);
    assert_eq!(report.torque_final, [3, -2, 1, -1]);
    assert_eq!(report.released_delta_counts, [512, -512, 60, -60]);
    assert!(report.reseeded);
}

#[test]
fn coast_settle_timeout_reenables_and_fails() {
    let mut b = Bench::new(0x03);
    let step = b.run_until_step();
    b.do_disable(step);

    // Slot 1 never goes quiet: keeps creeping more than the quiet band each
    // cycle, so one restless rotor holds the whole release.
    b.drift_counts_per_cycle[1] = 100;
    let step = b.run_until_step();
    b.do_enable(step);

    let SyncStep::Done(report) = b.run_until_step() else {
        panic!("expected Done");
    };
    assert_eq!(report.result, ERR_SYNC_SETTLE_TIMEOUT);
    assert!(report.reseeded);
}

#[test]
fn creep_inside_the_quiet_band_settles() {
    let mut b = Bench::new(0x03);
    let step = b.run_until_step();
    b.do_disable(step);

    // 8 counts/cycle stays within the 33-count quiet band across the 4-cycle
    // quiet window — encoder-noise-scale motion must not block settling.
    b.drift_counts_per_cycle = [8, -8, 0, 0];
    let step = b.run_until_step();
    b.drift_counts_per_cycle = [0; MAX_RELEASE_SLOTS];
    b.do_enable(step);

    let SyncStep::Done(report) = b.run_until_step() else {
        panic!("expected Done");
    };
    assert_eq!(report.result, 0);
}

#[test]
fn final_torque_above_threshold_fails_but_reseeds() {
    let mut b = Bench::new(0x03);
    let step = b.run_until_step();
    b.do_disable(step);

    let step = b.run_until_step();
    b.do_enable(step);

    // Re-enabled pair immediately fights again.
    b.torque = [60, -55, 0, 0];
    let SyncStep::Done(report) = b.run_until_step() else {
        panic!("expected Done");
    };
    assert_eq!(report.result, ERR_SYNC_FINAL_TORQUE);
    assert_eq!(report.torque_final[0], 60);
    assert_eq!(report.torque_final[1], -55);
    assert!(report.reseeded);
}

#[test]
fn final_torque_outside_the_mask_is_ignored() {
    let mut b = Bench::new(0x03);
    let step = b.run_until_step();
    b.do_disable(step);
    let step = b.run_until_step();
    b.do_enable(step);

    b.torque = [1, -1, 90, -90];
    let SyncStep::Done(report) = b.run_until_step() else {
        panic!("expected Done");
    };
    assert_eq!(report.result, 0);
}

#[test]
fn partial_mask_releases_only_masked_slots() {
    let mut b = Bench::new(0x0c);
    b.torque = [70, -70, 25, -25];
    b.position = [1_000, -1_000, 2_000, -2_000];

    let step = b.run_until_step();
    b.do_disable(step);

    // The unmasked pair keeps creeping (it is still energized and streaming
    // is someone else's business) — that must not stall the settle.
    b.drift_counts_per_cycle = [100, -100, 0, 0];
    b.position[2] = 2_040;
    b.position[3] = -2_040;
    let step = b.run_until_step();
    b.do_enable(step);
    b.torque = [70, -70, 2, -2];

    let SyncStep::Done(report) = b.run_until_step() else {
        panic!("expected Done");
    };
    assert_eq!(report.result, 0);
    assert_eq!(report.released_delta_counts[2], 40);
    assert_eq!(report.released_delta_counts[3], -40);
}

#[test]
fn empty_mask_is_rejected() {
    assert_eq!(
        SyncRelease::begin(params(), 0, [CPM; MAX_RELEASE_SLOTS]).err(),
        Some(ERR_SYNC_BAD_MASK)
    );
}

#[test]
fn mask_beyond_the_slot_arrays_is_rejected() {
    assert_eq!(
        SyncRelease::begin(params(), 0x10, [CPM; MAX_RELEASE_SLOTS]).err(),
        Some(ERR_SYNC_BAD_MASK)
    );
}
