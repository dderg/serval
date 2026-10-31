//! Retirement credit for a lane that two endpoints both speak for.

use super::pump_loop::Pump;
use super::*;
use crate::lock_ext::LockExt;
use std::collections::BTreeMap;
use std::sync::Arc;
use std::time::Duration;
use trajectory::ClockedMotorSpan;

#[derive(Default)]
struct NullSink {
    cuts: std::sync::Mutex<Vec<CutCredit>>,
}

impl SpanSink for NullSink {
    fn send_frame(
        &self,
        _key: AxisKey,
        _spans: &[ClockedMotorSpan],
        _new_head: u32,
        _room: u32,
    ) -> Result<i32, SendError> {
        Ok(mcu_protocol::result_codes::OK)
    }

    fn flush_keys(&self, _keys: &[AxisKey]) -> Result<Vec<CutCredit>, SendError> {
        Ok(std::mem::take(&mut *self.cuts.lock_ok()))
    }
}

const DUAL: AxisKey = AxisKey { mcu_id: 0, axis: 2 };

fn pump_with_pushed(pushed: u32) -> Pump<NullSink> {
    let mut queues = BTreeMap::new();
    let mut q = AxisQueue::new(64);
    q.credit.accept(pushed);
    queues.insert(DUAL, q);
    let mut pump = Pump::new(
        NullSink::default(),
        PumpCallbacks::noop(64),
        None,
        Arc::new(crate::drain::DrainLedger::new()),
    );
    pump.queues = queues;
    pump
}

fn report(pump: &mut Pump<NullSink>, retired_by: RetiredBy, retired: u32) {
    report_split(pump, retired_by, retired, retired);
}

fn report_split(pump: &mut Pump<NullSink>, retired_by: RetiredBy, consumed: u32, retired: u32) {
    pump.handle_control_msg(PumpMsg::Heartbeat(HeartbeatMsg {
        mcu_id: DUAL.mcu_id,
        axes: vec![DUAL.axis],
        consumed_counts: Some(vec![consumed]),
        retired_counts: vec![retired],
        retired_by,
    }));
    pump.publish_ledger();
}

#[test]
fn the_idle_transport_cannot_erase_the_active_transports_credit() {
    let mut pump = pump_with_pushed(5);

    report(&mut pump, RetiredBy::Phase, 5);
    assert!(pump.ledger.drained(), "the phase side finished all 5 spans");

    report(&mut pump, RetiredBy::Pulse, 0);
    assert!(
        pump.ledger.drained(),
        "the pulse endpoint is a member of the same dual lane and keeps reporting its frozen \
         odometer; it must not walk the axis back to unretired: {:?}",
        pump.ledger.lagging_axes()
    );
}

#[test]
fn a_transport_switch_mid_drain_carries_the_credit_already_earned() {
    let mut pump = pump_with_pushed(3);
    report(&mut pump, RetiredBy::Phase, 3);
    assert!(pump.ledger.drained());

    pump.queues.get_mut(&DUAL).unwrap().credit.accept(2);
    report(&mut pump, RetiredBy::Pulse, 0);
    assert!(
        !pump.ledger.drained(),
        "2 spans went out through the transport that just adopted the lane"
    );
    report(&mut pump, RetiredBy::Phase, 3);
    assert!(
        !pump.ledger.drained(),
        "the outgoing transport's final odometer covers only its own 3 spans"
    );

    report(&mut pump, RetiredBy::Pulse, 2);
    assert!(
        pump.ledger.drained(),
        "3 retired before the switch plus 2 after it account for all 5 pushed: {:?}",
        pump.ledger.lagging_axes()
    );
    let q = &pump.queues[&DUAL];
    assert_eq!(
        (q.credit.snapshot().retired, q.credit.snapshot().consumed),
        (5, 5)
    );
}

/// An endpoint counts a view consumed when it releases it — it has converted
/// every sample and can take the successor — and retired only once the mcu has
/// played it back. The two odometers move independently, and only retirement
/// drains the lane.
#[test]
fn consumption_frees_the_ring_while_only_playback_drains_the_lane() {
    let mut pump = pump_with_pushed(4);

    report_split(&mut pump, RetiredBy::Pulse, 4, 0);
    let q = &pump.queues[&DUAL];
    assert_eq!(
        (q.credit.snapshot().consumed, q.credit.snapshot().retired),
        (4, 0),
        "release credit must not imply playback"
    );
    assert_eq!(q.room(), q.ring_depth, "consumed views hold no ring slot");
    assert!(
        !pump.ledger.drained(),
        "nothing has played back yet: {:?}",
        pump.ledger.lagging_axes()
    );

    report_split(&mut pump, RetiredBy::Pulse, 4, 4);
    assert!(
        pump.ledger.drained(),
        "playback of every pushed view drains the lane: {:?}",
        pump.ledger.lagging_axes()
    );
}

#[test]
fn halt_abandonment_preserves_playback_truth_across_delayed_reports_and_resume() {
    let mut pump = pump_with_pushed(100);
    report(&mut pump, RetiredBy::Pulse, 36);
    pump.sink.cuts.lock_ok().push(CutCredit {
        key: DUAL,
        by: RetiredBy::Pulse,
        before: (36, 36),
        after: (100, 100),
    });
    let (ack, _) = std::sync::mpsc::sync_channel(1);
    pump.handle_control_msg(PumpMsg::Halt {
        keys: vec![DUAL],
        ack,
    });
    pump.publish_ledger();
    assert!(pump.ledger.drained());
    assert_eq!(pump.queues[&DUAL].credit.snapshot().pushed, 100);
    assert_eq!(pump.queues[&DUAL].credit.snapshot().retired, 36);
    assert_eq!(pump.queues[&DUAL].credit.snapshot().abandoned, 64);
    report(&mut pump, RetiredBy::Pulse, 70);
    report(&mut pump, RetiredBy::Pulse, 100);
    assert_eq!(
        pump.queues[&DUAL].credit.snapshot().retired,
        36,
        "discard and old receipts are not playback"
    );
    pump.handle_control_msg(PumpMsg::Resume(vec![DUAL]));
    pump.queues.get_mut(&DUAL).unwrap().credit.accept(1);
    pump.publish_ledger();
    assert!(!pump.ledger.drained());
    report(&mut pump, RetiredBy::Pulse, 101);
    assert!(pump.ledger.drained());
    assert_eq!(pump.queues[&DUAL].credit.snapshot().retired, 37);
    report(&mut pump, RetiredBy::Pulse, 100);
    assert_eq!(
        pump.queues[&DUAL].credit.snapshot().retired,
        37,
        "a delayed cut heartbeat cannot undo resumed playback"
    );
    assert_eq!(pump.queues[&DUAL].room(), 64);
}

#[test]
fn repeated_cuts_keep_mixed_transport_progress_separate_across_wraparound() {
    let mut pump = pump_with_pushed(u32::MAX);
    report_split(&mut pump, RetiredBy::Pulse, u32::MAX - 4, u32::MAX - 6);
    report_split(&mut pump, RetiredBy::Phase, 2, 1);
    pump.sink.cuts.lock_ok().push(CutCredit {
        key: DUAL,
        by: RetiredBy::Pulse,
        before: (u32::MAX - 3, u32::MAX - 5),
        after: (1, 1),
    });
    let (ack, _) = std::sync::mpsc::sync_channel(1);
    pump.handle_control_msg(PumpMsg::Halt {
        keys: vec![DUAL],
        ack,
    });
    pump.publish_ledger();
    assert!(pump.ledger.drained());
    assert_eq!(pump.queues[&DUAL].credit.snapshot().abandoned, 4);
    assert_eq!(pump.queues[&DUAL].room(), 64);

    pump.handle_control_msg(PumpMsg::Resume(vec![DUAL]));
    pump.queues.get_mut(&DUAL).unwrap().credit.accept(4);
    report_split(&mut pump, RetiredBy::Pulse, u32::MAX, u32::MAX);
    assert_eq!(pump.queues[&DUAL].room(), 60);
    assert_eq!(pump.queues[&DUAL].credit.outstanding(), 4);
    report_split(&mut pump, RetiredBy::Pulse, 3, 2);
    assert_eq!(pump.queues[&DUAL].room(), 62);
    assert_eq!(pump.queues[&DUAL].credit.outstanding(), 3);
    report_split(&mut pump, RetiredBy::Phase, 4, 3);
    assert_eq!(pump.queues[&DUAL].room(), 64);
    assert!(!pump.ledger.drained());

    pump.sink.cuts.lock_ok().push(CutCredit {
        key: DUAL,
        by: RetiredBy::Pulse,
        before: (3, 2),
        after: (5, 5),
    });
    let (ack, _) = std::sync::mpsc::sync_channel(1);
    pump.handle_control_msg(PumpMsg::Halt {
        keys: vec![DUAL],
        ack,
    });
    pump.publish_ledger();
    assert!(pump.ledger.drained());
    assert_eq!(pump.queues[&DUAL].credit.snapshot().abandoned, 5);
    assert_eq!(pump.queues[&DUAL].credit.snapshot().retired, u32::MAX - 1);

    pump.handle_control_msg(PumpMsg::Resume(vec![DUAL]));
    pump.queues.get_mut(&DUAL).unwrap().credit.accept(2);
    report_split(&mut pump, RetiredBy::Pulse, 3, 2);
    assert_eq!(pump.queues[&DUAL].room(), 62);
    assert_eq!(pump.queues[&DUAL].credit.outstanding(), 2);
    report_split(&mut pump, RetiredBy::Pulse, 6, 6);
    assert_eq!(pump.queues[&DUAL].credit.outstanding(), 1);
    report_split(&mut pump, RetiredBy::Pulse, 7, 7);
    assert!(pump.ledger.drained());
    assert_eq!(pump.queues[&DUAL].credit.snapshot().retired, 0);
    assert_eq!(pump.queues[&DUAL].room(), 64);
}

#[test]
fn an_axis_no_endpoint_speaks_for_never_drains() {
    let pump = pump_with_pushed(5);
    pump.publish_ledger();

    let error = pump
        .ledger
        .wait_drained(Duration::from_millis(20))
        .expect_err("nothing retires a lane no endpoint owns");
    assert!(
        error.contains("mcu0 axis2: pending 0 pushed 5 retired 0"),
        "{error}"
    );
}

#[test]
fn duplicate_flush_keys_reconcile_each_receipt_once() {
    let mut pump = pump_with_pushed(5);
    report_split(&mut pump, RetiredBy::Pulse, 2, 1);
    pump.sink.cuts.lock_ok().push(CutCredit {
        key: DUAL,
        by: RetiredBy::Pulse,
        before: (2, 1),
        after: (5, 5),
    });

    pump.handle_control_msg(PumpMsg::Flush(vec![DUAL, DUAL]));
    pump.publish_ledger();
    assert!(pump.ledger.drained());

    pump.queues.get_mut(&DUAL).unwrap().credit.accept(3);
    report(&mut pump, RetiredBy::Pulse, 8);
    assert!(pump.ledger.drained());
    assert_eq!(pump.queues[&DUAL].credit.snapshot().retired, 4);
}
