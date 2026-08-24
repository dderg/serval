//! Offline throughput bench for the streaming planner: drives the real
//! `FitStage` + `Planner` with the production streaming config over realistic
//! print paths and reports wall time against the planned motion time. A
//! ratio below 1.0 means the planner cannot keep up with the print and the
//! toolhead stalls.
//!
//! Run: cargo run --release --example planner_bench

use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Instant;

use crossbeam_channel::{bounded, unbounded};
use geometry::MoveVelocity;
use motion_pipeline::fit_stage::FitStage;
use motion_pipeline::planner::Planner;
use motion_pipeline::{PlannedItem, StreamConfig, StreamInput};

const PRODUCTION_MAX_BUFFER_MOVES: usize = 128;
const PRODUCTION_INTEGRATION_TOL: f64 = 1e-4;

const MAX_VELOCITY: f64 = 2800.0;
const MAX_ACCEL: f64 = 100_000.0;
const SCV: f64 = 70.0;

use pipeline_snapshot::waypoints::Waypoint;

static PLAN_COUNT: AtomicU64 = AtomicU64::new(0);
static PLAN_US_TOTAL: AtomicU64 = AtomicU64::new(0);
static PLAN_US_MAX: AtomicU64 = AtomicU64::new(0);

struct PlanCounter;

impl tracing::Subscriber for PlanCounter {
    fn enabled(&self, _: &tracing::Metadata<'_>) -> bool {
        true
    }
    fn new_span(&self, _: &tracing::span::Attributes<'_>) -> tracing::span::Id {
        tracing::span::Id::from_u64(1)
    }
    fn record(&self, _: &tracing::span::Id, _: &tracing::span::Record<'_>) {}
    fn record_follows_from(&self, _: &tracing::span::Id, _: &tracing::span::Id) {}
    fn event(&self, event: &tracing::Event<'_>) {
        #[derive(Default)]
        struct FindEvent {
            is_plan: bool,
            plan_us: u64,
        }
        impl tracing::field::Visit for FindEvent {
            fn record_debug(&mut self, _: &tracing::field::Field, _: &dyn std::fmt::Debug) {}
            fn record_str(&mut self, field: &tracing::field::Field, value: &str) {
                if field.name() == "event" && value == "pipe_plan" {
                    self.is_plan = true;
                }
            }
            fn record_u64(&mut self, field: &tracing::field::Field, value: u64) {
                if field.name() == "plan_us" {
                    self.plan_us = value;
                }
            }
            fn record_i64(&mut self, field: &tracing::field::Field, value: i64) {
                if field.name() == "plan_us" {
                    self.plan_us = value.max(0) as u64;
                }
            }
        }
        let mut finder = FindEvent::default();
        event.record(&mut finder);
        if finder.is_plan {
            PLAN_COUNT.fetch_add(1, Ordering::Relaxed);
            PLAN_US_TOTAL.fetch_add(finder.plan_us, Ordering::Relaxed);
            PLAN_US_MAX.fetch_max(finder.plan_us, Ordering::Relaxed);
        }
    }
    fn enter(&self, _: &tracing::span::Id) {}
    fn exit(&self, _: &tracing::span::Id) {}
}

fn faceted_circles(loops: usize, radius_mm: f64, chord_mm: f64) -> Vec<Waypoint> {
    let facets = ((2.0 * std::f64::consts::PI * radius_mm) / chord_mm).ceil() as usize;
    let mut wp = Vec::with_capacity(loops * facets + 1);
    let mut e = 0.0;
    wp.push((radius_mm, 0.0, 0.2, e, MAX_VELOCITY, MAX_ACCEL));
    for k in 1..=loops * facets {
        let theta = 2.0 * std::f64::consts::PI * (k as f64) / (facets as f64);
        e += chord_mm * 0.04;
        wp.push((
            radius_mm * libm::cos(theta),
            radius_mm * libm::sin(theta),
            0.2,
            e,
            MAX_VELOCITY,
            MAX_ACCEL,
        ));
    }
    wp
}

fn zigzag_infill(lines: usize, line_mm: f64, pitch_mm: f64) -> Vec<Waypoint> {
    let mut wp = Vec::with_capacity(2 * lines + 1);
    let mut e = 0.0;
    wp.push((0.0, 0.0, 0.2, e, MAX_VELOCITY, MAX_ACCEL));
    for i in 0..lines {
        let y = (i as f64) * pitch_mm;
        let (x0, x1) = if i % 2 == 0 {
            (0.0, line_mm)
        } else {
            (line_mm, 0.0)
        };
        e += line_mm * 0.04;
        wp.push((x1, y, 0.2, e, MAX_VELOCITY, MAX_ACCEL));
        let _ = x0;
        e += pitch_mm * 0.04;
        wp.push((x1, y + pitch_mm, 0.2, e, MAX_VELOCITY, MAX_ACCEL));
    }
    wp
}

fn move_time_s(mv: &MoveVelocity) -> f64 {
    if !mv.phases.is_empty() {
        mv.phases.iter().map(|p| p.dt).sum()
    } else {
        mv.samples
            .windows(2)
            .map(|w| {
                let ds = w[1].s - w[0].s;
                let v_sum = w[0].v + w[1].v;
                if v_sum > 0.0 { 2.0 * ds / v_sum } else { 0.0 }
            })
            .sum()
    }
}

fn stream_config() -> StreamConfig {
    let limits = geometry::VelocityLimits::try_new(MAX_VELOCITY, MAX_ACCEL, SCV, f64::INFINITY)
        .expect("valid limits");
    StreamConfig {
        corner: geometry::CornerFitConfig::default(),
        integration_tol: PRODUCTION_INTEGRATION_TOL,
        max_extrude_only_velocity_mm_s: f64::INFINITY,
        max_extrude_only_accel_mm_s2: f64::INFINITY,
        fit_tol_mm: 0.005,
        fit_tol_accel_mm_s2: 50.0,
        max_buffer_moves: PRODUCTION_MAX_BUFFER_MOVES,
        limits,
    }
}

fn run_fitter(moves: &[geometry::Move], config: &StreamConfig) -> Vec<StreamInput> {
    let (raw_tx, raw_rx) = unbounded();
    for m in moves.iter().cloned() {
        raw_tx.send(m.into()).expect("unbounded send");
    }
    drop(raw_tx);
    let (fitted_tx, fitted_rx) = unbounded();
    FitStage::new(config.corner).run(raw_rx, fitted_tx);
    fitted_rx.into_iter().collect()
}

struct BenchResult {
    planned_moves: usize,
    plans: u64,
    plan_us_total: u64,
    plan_us_max: u64,
    wall_s: f64,
    motion_s: f64,
}

fn run_planner(items: Vec<StreamInput>, config: StreamConfig, trickle: bool) -> BenchResult {
    let (in_tx, in_rx) = if trickle { bounded(1) } else { unbounded() };
    let (out_tx, out_rx) = unbounded();
    let plans_before = PLAN_COUNT.load(Ordering::Relaxed);
    let plan_us_before = PLAN_US_TOTAL.load(Ordering::Relaxed);
    PLAN_US_MAX.store(0, Ordering::Relaxed);

    let feeder = std::thread::spawn(move || {
        for item in items {
            if in_tx.send(item).is_err() {
                return;
            }
        }
    });
    let start = Instant::now();
    Planner::new(config).run(in_rx, out_tx);
    let wall_s = start.elapsed().as_secs_f64();
    feeder.join().expect("feeder thread");

    let planned: Vec<PlannedItem> = out_rx.into_iter().collect();
    let motion_s: f64 = planned
        .iter()
        .map(|item| match item {
            PlannedItem::Move(pm) => move_time_s(&pm.velocity),
            _ => 0.0,
        })
        .sum();
    let planned_moves = planned
        .iter()
        .filter(|i| matches!(i, PlannedItem::Move(_)))
        .count();
    BenchResult {
        planned_moves,
        plans: PLAN_COUNT.load(Ordering::Relaxed) - plans_before,
        plan_us_total: PLAN_US_TOTAL.load(Ordering::Relaxed) - plan_us_before,
        plan_us_max: PLAN_US_MAX.load(Ordering::Relaxed),
        wall_s,
        motion_s,
    }
}

fn bench(name: &str, waypoints: &[Waypoint], trickle: bool) {
    let config = stream_config();
    let moves = pipeline_snapshot::build_moves(waypoints, config.limits).expect("valid waypoints");
    let fit_start = Instant::now();
    let fitted = run_fitter(&moves, &config);
    let fit_wall = fit_start.elapsed().as_secs_f64();
    let r = run_planner(fitted, stream_config(), trickle);
    let ratio = r.motion_s / r.wall_s;
    let plan_mean_ms = if r.plans > 0 {
        r.plan_us_total as f64 / r.plans as f64 / 1e3
    } else {
        0.0
    };
    println!(
        "{name:<18} feed={:<9} moves_in={:<4} planned={:<4} plans={:<4} \
         fit={:>6.1}ms plan_wall={:>9.1}ms plan_mean={:>8.1}ms plan_max={:>8.1}ms \
         motion={:>7.1}ms realtime_x={ratio:>7.2} {}",
        if trickle { "trickle" } else { "saturated" },
        moves.len(),
        r.planned_moves,
        r.plans,
        fit_wall * 1e3,
        r.wall_s * 1e3,
        plan_mean_ms,
        r.plan_us_max as f64 / 1e3,
        r.motion_s * 1e3,
        if ratio < 1.0 { "<-- STALLS" } else { "" }
    );
}

fn main() {
    tracing::subscriber::set_global_default(PlanCounter).expect("set subscriber");
    let circles = faceted_circles(2, 15.0, 0.4);
    let zigzag = zigzag_infill(100, 40.0, 0.4);

    for &trickle in &[false, true] {
        bench("faceted-circles", &circles, trickle);
        bench("zigzag-infill", &zigzag, trickle);
        println!();
    }
}
