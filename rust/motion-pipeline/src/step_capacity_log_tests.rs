use super::*;
use geometry::McuStepLoad;

fn capacity() -> StepCapacity {
    let mcu = |name: &str| McuStepLoad {
        mcu: name.to_string(),
        lane_s_per_mm: [1e-3; 3],
        followers: Vec::new(),
    };
    StepCapacity {
        lane_weights: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        mcus: vec![mcu("mcu"), mcu("toolboard")],
    }
}

fn overload(mcu_index: usize, demand_over_capacity: f64) -> StepCapacityOverload {
    StepCapacityOverload {
        mcu_index,
        demand_over_capacity,
    }
}

#[test]
fn the_first_cap_reports_at_once_and_later_ones_are_counted_until_the_interval() {
    let capacity = capacity();
    let mut log = StepCapacityLog::default();
    let start = Instant::now();
    assert_eq!(
        log.record(&capacity, overload(0, 1.2), start),
        vec![StepCapacityReport {
            mcu: "mcu".into(),
            moves_capped: 1,
            worst_demand_over_capacity: 1.2,
        }]
    );
    let soon = start + Duration::from_secs(1);
    assert!(log.record(&capacity, overload(0, 1.5), soon).is_empty());
    assert!(log.record(&capacity, overload(1, 1.1), soon).is_empty());
    assert!(log.record(&capacity, overload(0, 1.3), soon).is_empty());
    let later = start + STEP_CAPACITY_LOG_INTERVAL;
    assert_eq!(
        log.record(&capacity, overload(0, 1.05), later),
        vec![
            StepCapacityReport {
                mcu: "mcu".into(),
                moves_capped: 3,
                worst_demand_over_capacity: 1.5,
            },
            StepCapacityReport {
                mcu: "toolboard".into(),
                moves_capped: 1,
                worst_demand_over_capacity: 1.1,
            },
        ]
    );
}
