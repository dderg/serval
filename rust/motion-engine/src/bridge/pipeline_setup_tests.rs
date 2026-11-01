use super::pipeline_setup::report_ethercat_credit;
use motion_core::lock_ext::LockExt;

#[test]
fn ethercat_credit_waits_for_delivery_and_the_laggard_motor_playback() {
    use ethercat_setpoint_fill::setpoint_fill::{CLOCK_FREQ_HZ, ChainFiller, LaneSpec};
    use std::sync::{Arc, Mutex};
    use trajectory::{ClockedMotorSpan, ContinuousAxis, MotorGroup, MotorSpan, MotorTerm};

    let signal = Arc::new(
        MotorSpan::try_new(
            Arc::from([MotorGroup::Independent(MotorTerm {
                source_axis: 2,
                axis: ContinuousAxis::Hold {
                    position: 0.0,
                    t_start: 0.0,
                    t_end: 0.001,
                },
                scale: 1.0,
            })]),
            0.0,
            0.001,
            0,
            0,
            true,
        )
        .unwrap(),
    );
    let view =
        ClockedMotorSpan::try_new(signal, 0.0, 0.001, 0.0, 0.001, 0.0, CLOCK_FREQ_HZ).unwrap();
    let end = view.end_clock;
    let mut chain = ChainFiller::new(
        &[
            LaneSpec {
                axis: 2,
                cmd_counts_per_mm: 1000.0,
                ff_lead_ns: 0,
            },
            LaneSpec {
                axis: 2,
                cmd_counts_per_mm: 1000.0,
                ff_lead_ns: 0,
            },
        ],
        None,
        250_000,
        0,
    );
    chain.observe_grid(0, 0).unwrap();
    chain.push_spans(2, &[view]).unwrap();
    let output = chain.pending_sample_runs().unwrap().unwrap();
    let filler = Arc::new(Mutex::new(chain));
    let (tx, rx) = crossbeam_channel::unbounded();
    let report = |clocks: &[u64]| {
        report_ethercat_credit(&tx, 4, &[2, 2], &filler, clocks).unwrap();
        let motion_core::pump::PumpMsg::Heartbeat(credit) = rx.recv().unwrap() else {
            panic!("view credit");
        };
        credit
    };
    let pending = report(&[end, end]);
    assert_eq!(pending.consumed_counts, Some(vec![1]));
    assert_eq!(
        pending.retired_counts,
        vec![0],
        "unsent output cannot retire on an advanced playhead"
    );
    assert!(filler.lock_ok().acknowledge_sample_runs(&output));
    assert_eq!(report(&[end, end - 1]).retired_counts, vec![0]);
    let played = report(&[end, end]);
    assert_eq!(played.axes, vec![2]);
    assert_eq!(
        played.retired_counts,
        vec![1],
        "two motor slots retire one logical view"
    );
    assert!(report_ethercat_credit(&tx, 4, &[2, 2], &filler, &[end]).is_err());
}
