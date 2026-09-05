//! The send pass against the mcu's own two refusals: `reset_step_clock` is
//! rejected while the stepper still holds queued steps, and `queue_step` is
//! rejected before a reset has anchored the timeline. Every re-anchor is a
//! race between those two - the fresh epoch's volley heads with a reset while
//! the old epoch is still executing - so the shapes worth randomizing are
//! where in the old stream the resume lands, how much of the send lead is
//! left when it is submitted, and how many lanes share the axis.

use super::*;
use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;

const AXIS: u8 = 2;
const MAX_LANES: usize = 3;

#[derive(Debug, Clone)]
struct Scenario {
    lanes: usize,
    old_views: usize,
    tick_secs: f64,
    /// How far into the old stream the resume is submitted, as a fraction of
    /// its span: below one the old epoch is still mid-flight, above one it
    /// has finished and the resume is an idle re-anchor.
    submit_ratio: f64,
    /// The resume's first step, as a fraction of the send lead past whatever
    /// the wire already holds. Small values leave the reset almost no margin
    /// once it has waited for the old stream's last step.
    resume_margin_ratio: f64,
    resume_views: usize,
    reverse: bool,
}

impl Scenario {
    fn oids(&self) -> Vec<u32> {
        (0..self.lanes as u32).map(|i| 5 + i).collect()
    }

    fn tick_clocks(&self) -> u64 {
        ((H7_FREQ * self.tick_secs) as u64).max(1)
    }

    fn view_clocks(&self) -> u64 {
        (H7_FREQ * RAMP_VIEW_SECS) as u64
    }
}

fn arb_scenario() -> impl Strategy<Value = Scenario> {
    (
        1..=MAX_LANES,
        4..=50usize,
        prop_oneof![
            Just(0.001),
            Just(0.002),
            Just(0.005),
            Just(0.010),
            Just(0.020),
        ],
        0.05..1.6f64,
        0.05..1.6f64,
        2..=20usize,
        any::<bool>(),
    )
        .prop_map(
            |(
                lanes,
                old_views,
                tick_secs,
                submit_ratio,
                resume_margin_ratio,
                resume_views,
                reverse,
            )| Scenario {
                lanes,
                old_views,
                tick_secs,
                submit_ratio,
                resume_margin_ratio,
                resume_views,
                reverse,
            },
        )
}

/// A fatal the pump is *supposed* to raise: the resume could not be delivered
/// with execution margin left, which the mcu would answer with a shutdown, so
/// the host refuses it first. A wedge, a silent drop or an mcu-rule violation
/// is a defect.
fn is_expected_late_refusal(error: &SendError) -> bool {
    match error {
        SendError::Fatal(message) => message.contains("behind the projected mcu clock"),
        _ => false,
    }
}

struct Run {
    mcu: HashMap<u32, McuStepper>,
    refusal: Option<SendError>,
}

/// Drive one re-anchor: an old stream long enough to still be executing, then
/// a fresh epoch submitted `submit_ratio` of the way through it, starting
/// `resume_margin_ratio` of the send lead past the wire's horizon. Every
/// burst is replayed against the modelled mcu at the clock it is handed
/// over, which is what makes the mcu's two refusals observable here.
fn drive(scenario: &Scenario) -> Result<Run, String> {
    let mut h = h7_harness(scenario.oids());
    let mut mcu: HashMap<u32, McuStepper> = HashMap::new();
    let lead = (H7_FREQ * SEND_LEAD_SECONDS) as u64;
    let tick_clocks = scenario.tick_clocks();
    let mut now = (H7_FREQ * 2.0) as u64;
    h.now.store(now, Ordering::Relaxed);

    let old_start = now + lead;
    let old_span = scenario.view_clocks() * scenario.old_views as u64;
    h.endpoint.mark_reanchor(AXIS, old_start, Some(H7_FREQ));
    h.endpoint
        .send_frames(
            MCU_ID,
            &[frame_for_axis(
                AXIS,
                h7_ramp(old_start, scenario.old_views, 0.0, 1.0),
            )],
        )
        .map_err(|e| format!("the first epoch was refused: {e:?}"))?;

    let lead_oid = scenario.oids()[0];
    h.auto_query.store(false, Ordering::Relaxed);
    // The mcu reads back what it was actually handed. The harness's own
    // auto-query answers with the host's compiled total instead, which counts
    // frames still waiting in the backlog.
    let settle = |h: &mut Harness,
                  mcu: &mut HashMap<u32, McuStepper>,
                  now: u64|
     -> Result<Option<SendError>, String> {
        let frames: Vec<StepFrame> = std::mem::take(&mut h.sent.lock_ok());
        play_frames_on_mcu(&frames, mcu, now)?;
        let executed = mcu.get(&lead_oid).map_or(0, |stepper| stepper.position);
        h.query_count.store(executed, Ordering::Relaxed);
        match h.ack_sent_barriers_result() {
            Ok(()) => Ok(None),
            Err(error) => Ok(Some(error)),
        }
    };

    let submit_at = old_start + (old_span as f64 * scenario.submit_ratio) as u64;
    let mut resume_end = u64::MAX;
    let mut submitted = false;
    while now < resume_end.saturating_add(lead) {
        now = (now + tick_clocks).min(if submitted { u64::MAX } else { submit_at });
        h.now.store(now, Ordering::Relaxed);
        if let Err(error) = h.endpoint.tick() {
            settle(&mut h, &mut mcu, now)?;
            return refusal(mcu, error);
        }
        if let Some(error) = settle(&mut h, &mut mcu, now)? {
            return refusal(mcu, error);
        }
        if submitted || now < submit_at {
            continue;
        }
        submitted = true;
        // The host cannot recall a step it has already handed over, so a
        // re-anchor always lands at or past the wire's own horizon; how far
        // past is the margin the reset gets once it has waited the old
        // stream out.
        let horizon = h.endpoint.lanes[0]
            .last_sent_boundary
            .unwrap_or(now)
            .max(now);
        let resume_at = horizon + (lead as f64 * scenario.resume_margin_ratio) as u64;
        resume_end = resume_at + scenario.view_clocks() * scenario.resume_views as u64;
        let direction = if scenario.reverse { -1.0 } else { 1.0 };
        let position = h.endpoint.shim.commanded_position(0);
        h.endpoint.mark_reanchor(AXIS, resume_at, Some(H7_FREQ));
        if let Err(error) = h.endpoint.send_frames(
            MCU_ID,
            &[frame_for_axis(
                AXIS,
                h7_ramp(resume_at, scenario.resume_views, position, direction),
            )],
        ) {
            return refusal(mcu, error);
        }
    }
    if !h.endpoint.backlog.is_empty() {
        return Err(format!(
            "the backlog never drained: {} frames still held at clock {now}, a whole \
             send lead past the resume's last step at {resume_end}",
            h.endpoint.backlog.len()
        ));
    }
    verify_mcu_bases(&h.endpoint.lanes, &mcu)?;
    Ok(Run { mcu, refusal: None })
}

fn refusal(mcu: HashMap<u32, McuStepper>, error: SendError) -> Result<Run, String> {
    if is_expected_late_refusal(&error) {
        Ok(Run {
            mcu,
            refusal: Some(error),
        })
    } else {
        Err(format!(
            "the pump failed with an unexpected error: {error:?}"
        ))
    }
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 192,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/stepcompress_sink_fuzz.txt",
        ))),
        ..ProptestConfig::default()
    })]

    /// The whole contract of the re-anchor send path: whatever the residual
    /// execution and whatever the resume's margin, the frames that reach the
    /// mcu are always legal for it to execute. A reset that overtakes the
    /// stream it replaces is the `Can't reset time when stepper active`
    /// shutdown; a volley that overtakes its own reset is the mirror image.
    /// Either the resume is delivered in full or it is refused out loud -
    /// never both, and never neither.
    #[test]
    fn a_reanchor_never_resets_a_stepper_the_mcu_is_still_stepping(
        scenario in arb_scenario(),
    ) {
        let run = drive(&scenario).map_err(TestCaseError::fail)?;
        if run.refusal.is_none() {
            prop_assert!(
                !run.mcu.is_empty()
                    && run.mcu.values().all(|stepper| !stepper.need_reset),
                "the resume was delivered in full, so every lane must be anchored: {:?}",
                run.mcu
                    .iter()
                    .map(|(oid, stepper)| (*oid, stepper.need_reset))
                    .collect::<Vec<_>>()
            );
        }
    }
}
