use super::*;

fn drains_while_observing(runways_s: &[f64], reserve_s: f64) -> Vec<usize> {
    let mut watch = BusyRunwayWatch::default();
    runways_s
        .iter()
        .enumerate()
        .filter(|&(_, &runway_s)| watch.observe(runway_s, reserve_s))
        .map(|(index, _)| index)
        .collect()
}

#[test]
fn a_busy_stream_gaining_runway_under_a_widened_reserve_never_brakes() {
    let reserve_s = 2.4;
    let sawtooth_growth: Vec<f64> = (0..200)
        .map(|step| 0.01 * f64::from(step) - if step % 4 == 0 { 0.0 } else { 0.05 })
        .map(|runway_s: f64| runway_s.max(0.0))
        .collect();
    assert_eq!(
        drains_while_observing(&sawtooth_growth, reserve_s),
        Vec::<usize>::new()
    );
}

#[test]
fn a_busy_stream_that_never_built_runway_has_nothing_to_brake() {
    assert_eq!(drains_while_observing(&[0.0; 50], 0.5), Vec::<usize>::new());
}

#[test]
fn a_busy_stream_brakes_once_it_loses_half_the_reserve_below_its_best_runway() {
    let reserve_s = 1.0;
    let hovering = [0.9, 0.8, 0.7, 0.5, 0.41, 0.39, 0.3];
    assert_eq!(drains_while_observing(&hovering, reserve_s), vec![5, 6]);
}

#[test]
fn a_busy_stream_that_held_the_reserve_brakes_half_the_reserve_below_it() {
    let reserve_s = 1.0;
    let falling = [3.0, 0.9, 0.8, 0.7, 0.5, 0.41, 0.39, 0.3];
    assert_eq!(
        drains_while_observing(&falling, reserve_s),
        vec![4, 5, 6, 7]
    );
}

#[test]
fn a_busy_stream_that_falls_from_the_reserve_past_half_of_it_unobserved_brakes_at_once() {
    let reserve_s = 1.0;
    let coarse = [1.4, 1.1, 0.2];
    assert_eq!(drains_while_observing(&coarse, reserve_s), vec![2]);
}

#[test]
fn the_reserve_covers_a_brake_after_the_most_runway_lost_planning_blind() {
    let mut pacer = Pacer::default();
    pacer.note_drain(0.3);
    pacer.note_step(1.0, 0.6);
    pacer.note_step(0.6, 0.9);
    assert!(
        (pacer.reserve_secs() - DRAIN_RESERVE_SAFETY * (0.3 + 0.4 + LOW_MARGIN_WARN_SECS)).abs()
            < 1e-12
    );
}

#[test]
fn a_stream_that_regains_the_reserve_brakes_only_after_losing_half_of_it_again() {
    let reserve_s = 1.0;
    let recovering = [0.9, 0.45, 1.2, 0.8, 0.6, 0.55];
    assert_eq!(
        drains_while_observing(&recovering, reserve_s),
        Vec::<usize>::new()
    );
}

const COMMIT_S: f64 = 0.01;
const BRAKE_S: f64 = 0.1;
const SIMULATED_S: f64 = 60.0;
const ANALYZER_PACE_TOLERANCE_S: f64 = 20e-3;
const ANALYZER_PACE_TOLERANCE_PPM: f64 = 500.0;

/// One planned segment as the dispatcher anchored it.
struct Dispatched {
    stream_start: f64,
    host_start: f64,
}

/// A producer behind its playhead: it plans `realtime_factor` of realtime
/// in `COMMIT_S` commits after a first commit of `start_commit_s`, so the
/// ingress finds its inbox empty between commits and takes the idle path.
/// A brake to rest takes `drain_latency_s` to drain, its first segment
/// dispatched halfway through; a requested stop of the same latency may
/// have been measured before the print starts.
#[derive(Clone, Copy)]
struct Starvation {
    realtime_factor: f64,
    start_commit_s: f64,
    drain_latency_s: f64,
    measured_before_start: bool,
}

/// The bench's starved trident print: a print-start buffer, then a
/// producer at 0.9x realtime whose brakes to rest take 0.53 s.
const BENCH_TRIDENT_ONE_CORE: Starvation = Starvation {
    realtime_factor: 0.9,
    start_commit_s: 2.0,
    drain_latency_s: 0.53,
    measured_before_start: false,
};

/// A stream starting from rest after a requested stop measured a 0.2 s
/// drain, with a single commit in flight and a producer at a fifth of
/// realtime: only braking before the prime runs out leaves a brake's
/// worth of runway.
const STARVED_FROM_THE_FIRST_COMMIT: Starvation = Starvation {
    realtime_factor: 0.2,
    start_commit_s: COMMIT_S,
    drain_latency_s: 0.2,
    measured_before_start: true,
};

/// A starvation played on virtual time against the real anchor and pacer.
/// Every segment is classified before it is anchored, so an underrun fails
/// the simulation with its verdict instead of aborting.
struct StarvedPrint {
    starvation: Starvation,
    base: Instant,
    now: f64,
    stream_t: f64,
    next_commit: f64,
    anchor: crate::anchor::Anchor,
    pacer: Pacer,
    dispatched: Vec<Dispatched>,
    brakes: usize,
}

impl StarvedPrint {
    fn new(starvation: Starvation) -> Self {
        let mut pacer = Pacer::default();
        if starvation.measured_before_start {
            pacer.note_drain(starvation.drain_latency_s);
        }
        Self {
            starvation,
            base: Instant::now(),
            now: 0.0,
            stream_t: 0.0,
            next_commit: 0.0,
            anchor: crate::anchor::Anchor::new(),
            pacer,
            dispatched: Vec::new(),
            brakes: 0,
        }
    }

    fn instant(&self, at: f64) -> Instant {
        self.base + Duration::from_secs_f64(at)
    }

    fn runway_s(&self) -> f64 {
        self.anchor
            .committed_end_host()
            .map_or(0.0, |end| (end - self.now).max(0.0))
    }

    fn dispatch(&mut self, duration_s: f64) -> Result<(), String> {
        let seg_t_start = self.stream_t;
        if self.anchor.planned_start(seg_t_start, self.now).is_none() {
            return Err(format!(
                "segment at stream {seg_t_start:.3}s overran the playhead at {:.3}s after {} \
                 brakes (committed end {:?})",
                self.now,
                self.brakes,
                self.anchor.committed_end_host()
            ));
        }
        let (t0, _) = self
            .anchor
            .anchor_segment(seg_t_start, seg_t_start + duration_s, self.now);
        self.dispatched.push(Dispatched {
            stream_start: seg_t_start,
            host_start: t0 + seg_t_start,
        });
        self.stream_t += duration_s;
        Ok(())
    }

    fn brake(&mut self) -> Result<(), String> {
        let in_flight = !self.anchor.is_parked();
        let latency_s = self.starvation.drain_latency_s;
        self.now += latency_s / 2.0;
        self.dispatch(BRAKE_S)?;
        self.now += latency_s / 2.0;
        self.anchor.mark_parked();
        self.pacer.note_drain(latency_s);
        if in_flight {
            self.anchor.require_resume_lead(self.pacer.reserve_secs())?;
        }
        self.brakes += 1;
        Ok(())
    }

    fn run(&mut self) -> Result<(), String> {
        self.pacer.note_move(self.instant(self.now));
        self.dispatch(self.starvation.start_commit_s)?;
        while self.now < SIMULATED_S {
            if self.now >= self.next_commit {
                self.pacer.note_move(self.instant(self.now));
                self.dispatch(COMMIT_S)?;
                self.next_commit = self.now + COMMIT_S / self.starvation.realtime_factor;
                continue;
            }
            let at_rest = self.anchor.is_parked();
            match self
                .pacer
                .idle_verdict(self.runway_s(), at_rest, self.instant(self.now))
            {
                IdleVerdict::Wait(wait) => {
                    self.now = (self.now + wait.as_secs_f64().max(1e-6)).min(self.next_commit);
                }
                IdleVerdict::Brake => self.brake()?,
            }
        }
        Ok(())
    }

    fn worst_pace_lag(&self) -> (f64, f64) {
        let first = &self.dispatched[0];
        self.dispatched
            .iter()
            .map(|d| {
                let stream_s = d.stream_start - first.stream_start;
                let lag_s = (d.host_start - first.host_start) - stream_s;
                let allowed_s =
                    ANALYZER_PACE_TOLERANCE_S + ANALYZER_PACE_TOLERANCE_PPM * 1e-6 * stream_s;
                (lag_s, allowed_s)
            })
            .fold((0.0, f64::INFINITY), |worst, (lag_s, allowed_s)| {
                if lag_s - allowed_s > worst.0 - worst.1 {
                    (lag_s, allowed_s)
                } else {
                    worst
                }
            })
    }
}

#[test]
fn a_producer_slower_than_realtime_brakes_and_resumes_without_ever_overrunning_the_playhead() {
    let mut print = StarvedPrint::new(BENCH_TRIDENT_ONE_CORE);
    print.run().unwrap_or_else(|underrun| panic!("{underrun}"));
    assert!(
        print.brakes >= 10,
        "a producer at {}x realtime braked {} times in {SIMULATED_S}s; the pacer must \
         stop it and resume again and again",
        BENCH_TRIDENT_ONE_CORE.realtime_factor,
        print.brakes
    );
    let (lag_s, allowed_s) = print.worst_pace_lag();
    assert!(
        lag_s > allowed_s,
        "the brakes paused the machine, yet the dispatched starts lag the stream by only \
         {lag_s}s ({allowed_s}s allowed): the bench pace check could not flag the stops"
    );
}

#[test]
fn a_starved_stream_in_flight_brakes_before_its_prime_runs_out() {
    let mut print = StarvedPrint::new(STARVED_FROM_THE_FIRST_COMMIT);
    print.run().unwrap_or_else(|underrun| panic!("{underrun}"));
    assert!(print.brakes > 0);
}

#[test]
fn a_stream_in_flight_waits_out_its_prime_only_while_one_brake_still_fits() {
    let base = Instant::now();
    let mut pacer = Pacer::default();
    pacer.note_drain(0.2);
    pacer.note_step(0.6, 0.55);
    pacer.note_move(base);
    let one_brake_s = 0.2 + 0.05 + LOW_MARGIN_WARN_SECS;
    assert_eq!(
        pacer.idle_verdict(0.0, true, base),
        IdleVerdict::Wait(Duration::from_secs_f64(STARTUP_PRIME_S)),
        "nothing in flight: the prime waits for the stream to build runway"
    );
    assert_eq!(
        pacer.idle_verdict(one_brake_s - 0.01, false, base),
        IdleVerdict::Brake,
        "motion in flight with less runway than one brake: waiting out the prime would \
         let the playhead overrun it"
    );
    assert_eq!(
        pacer.idle_verdict(one_brake_s + 0.1, false, base),
        IdleVerdict::Wait(Duration::from_secs_f64(0.1)),
        "the prime waits only until the runway reaches one brake"
    );
    assert_eq!(
        pacer.idle_verdict(0.3, false, base + Duration::from_secs(1)),
        IdleVerdict::Brake,
        "an expired prime brakes under the reserve"
    );
}
