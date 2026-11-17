// Replays a gcode file through the production streaming pipeline, built from
// a printer.cfg exactly as the live path builds it (stream config, post-
// processor chains, per-move limits including the z caps, shaper on its own
// thread), and reports producer throughput as a sliding-window realtime
// factor over stream time, the deepest run of moves absorbed without the
// planner committing, and where the wall time went stage by stage.
//
//   cargo run --release -p motion-core --example gcode_replay_bench -- \
//       <file.gcode> --config <printer.cfg> [t_lo t_hi] \
//       [--min-worst-x <x>] [--max-held-moves <n>] [--digest] [--synchronous] \
//       [--mesh <bed_mesh.json>] [--check]
//
// With t_lo/t_hi only moves whose stream window overlaps [t_lo, t_hi] stream
// seconds are interesting; the whole file still replays (stream time only
// exists downstream), but the report highlights that range. --digest hashes
// every emitted segment, so a throughput change can be held to an identical
// trajectory. --synchronous drives the plain synchronous Pipeline on the
// calling thread with no shaper thread; under `taskset -c 0` the worker pool
// has no helpers either, which makes the run single-threaded and its
// instruction count deterministic (scripts/throughput_gate.py). The planner
// commits only show up as shaped output there, so --max-held-moves needs the
// staged pipeline.
//
// Stops at rest (G4, M400, M84, ...) reach the pipeline as the live engine
// sends them. --mesh activates a bed mesh before the first move, as klippy's
// `BED_MESH_PROFILE LOAD` does with the toolhead at the origin: the JSON is
// the request `bed_mesh` hands `set_bed_mesh`, and the G-code starts from
// the Z the activation renamed. --check fails on any emitted segment that
// does not evaluate to finite positions. Any failure prints one
// `REPLAY_FAILURE {json}` line for scripts/preflight.py; a panic then exits
// with EXIT_REPORTED_PANIC instead of aborting, so it leaves no core dump.

use std::env;
use std::fs;
use std::hash::{Hash, Hasher};
use std::io::Write;
use std::panic;
use std::process;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock, PoisonError};
use std::time::Instant;

use geometry::{GcodePos, MachinePos, MeshGridSpec, ProbedMesh, SurfaceTransform};
use motion_core::seam_test_harness::{LiveScript, LiveScriptError};
use motion_pipeline::timing::stage_totals;
use motion_pipeline::{Control, Pipeline, StreamInput, TrajectoryItem};
use pipeline_snapshot::{PlanStep, SnapshotError};
use trajectory::ContinuousSegment;

const EXIT_FAILURE: i32 = 1;
const EXIT_REPORTED_PANIC: i32 = 70;

#[derive(Clone, Copy)]
enum FailureKind {
    Config,
    Mesh,
    GcodeStopsPrint,
    GcodeUnplannable,
    MoveRefused,
    NothingToPlan,
    Panic,
    NonFinite,
    PipelineClosed,
}

impl FailureKind {
    fn name(self) -> &'static str {
        match self {
            Self::Config => "config",
            Self::Mesh => "mesh",
            Self::GcodeStopsPrint => "gcode_stops_print",
            Self::GcodeUnplannable => "gcode_unplannable",
            Self::MoveRefused => "move_refused",
            Self::NothingToPlan => "nothing_to_plan",
            Self::Panic => "panic",
            Self::NonFinite => "non_finite",
            Self::PipelineClosed => "pipeline_closed",
        }
    }

    fn of(error: &LiveScriptError) -> Self {
        match error {
            LiveScriptError::Config(_) => Self::Config,
            LiveScriptError::Gcode(e) if e.stops_the_live_print() => Self::GcodeStopsPrint,
            LiveScriptError::Gcode(_) => Self::GcodeUnplannable,
            LiveScriptError::Plan(SnapshotError::InvalidMove { .. }) => Self::MoveRefused,
            LiveScriptError::Plan(_) => Self::NothingToPlan,
        }
    }
}

/// Where the replay stands: the sequence numbers of the last move fed and
/// of the last segment emitted, and that segment's end.
#[derive(Clone, Copy)]
struct Progress {
    fed_seq: Option<u32>,
    emitted_seq: Option<u32>,
    stream_t: Option<f64>,
}

static PROGRESS: Mutex<Progress> = Mutex::new(Progress {
    fed_seq: None,
    emitted_seq: None,
    stream_t: None,
});
static MOVE_LINES: OnceLock<Vec<u32>> = OnceLock::new();
static FAILED: AtomicBool = AtomicBool::new(false);

fn progress() -> MutexGuard<'static, Progress> {
    PROGRESS.lock().unwrap_or_else(PoisonError::into_inner)
}

fn source_line(seq: Option<u32>) -> Option<u32> {
    MOVE_LINES.get()?.get(seq? as usize).copied()
}

/// The first failure's record; later ones are its consequences.
fn record_failure(kind: FailureKind, message: &str, location: Option<&str>) {
    if FAILED.swap(true, Ordering::SeqCst) {
        return;
    }
    let at = *progress();
    let record = serde_json::json!({
        "kind": kind.name(),
        "message": message,
        "location": location,
        "fed_line": source_line(at.fed_seq),
        "emitted_line": source_line(at.emitted_seq),
        "stream_t": at.stream_t,
    });
    println!("REPLAY_FAILURE {record}");
    let _ = std::io::stdout().flush();
}

/// A panic on any thread records its failure from the hook; the record is
/// flushed by then, so the process exits at once.
fn record_panics() {
    let default_hook = panic::take_hook();
    panic::set_hook(Box::new(move |info| {
        let payload = info.payload();
        let message = payload
            .downcast_ref::<&str>()
            .map(|text| (*text).to_owned())
            .or_else(|| payload.downcast_ref::<String>().cloned())
            .unwrap_or_default();
        let location = info.location().map(ToString::to_string);
        record_failure(FailureKind::Panic, &message, location.as_deref());
        default_hook(info);
        process::exit(EXIT_REPORTED_PANIC);
    }));
}

fn fail(kind: FailureKind, message: &str) -> ! {
    record_failure(kind, message, None);
    eprintln!("FAIL {}: {message}", kind.name());
    process::exit(EXIT_FAILURE);
}

fn take_flag(args: &mut Vec<String>, flag: &str) -> Option<String> {
    let i = args.iter().position(|a| a == flag)?;
    args.remove(i);
    Some(args.remove(i))
}

fn take_switch(args: &mut Vec<String>, switch: &str) -> bool {
    let Some(i) = args.iter().position(|a| a == switch) else {
        return false;
    };
    args.remove(i);
    true
}

fn note_fed(input: &StreamInput) {
    if let StreamInput::Move(m) = input {
        progress().fed_seq = Some(m.source.start_line);
    }
}

/// The deepest run of inputs absorbed without the planner committing, or
/// `None` when the pipeline closed.
fn replay_staged(
    pipeline: Pipeline,
    inputs: Vec<StreamInput>,
    sink: impl FnMut(TrajectoryItem) -> bool + Send + 'static,
) -> Option<u64> {
    let mut pipeline = pipeline.into_staged(sink);
    let mut held = 0u64;
    let mut deepest_held = 0u64;
    for input in inputs {
        note_fed(&input);
        let planned_before = pipeline.segments_planned();
        if !pipeline.feed(input) {
            return None;
        }
        held = if pipeline.segments_planned() > planned_before {
            0
        } else {
            held + 1
        };
        deepest_held = deepest_held.max(held);
    }
    pipeline.finish().then_some(deepest_held)
}

fn replay_synchronous(
    mut pipeline: Pipeline,
    inputs: Vec<StreamInput>,
    mut sink: impl FnMut(TrajectoryItem) -> bool,
) -> bool {
    for input in inputs {
        note_fed(&input);
        if !pipeline.feed(input, &mut sink) {
            return false;
        }
    }
    pipeline.finish(&mut sink)
}

fn planner_config(path: &str) -> planner_config::PlannerConfig {
    let text = fs::read_to_string(path)
        .unwrap_or_else(|e| fail(FailureKind::Config, &format!("cannot read {path}: {e}")));
    planner_config::from_doc::planner_config_from_text(&text)
        .unwrap_or_else(|e| fail(FailureKind::Config, &format!("{path}: {e}")))
}

/// The request klippy's `bed_mesh` hands `set_bed_mesh`.
#[derive(serde::Deserialize)]
struct BedMeshRequest {
    points: Vec<f64>,
    x_min: f64,
    y_min: f64,
    dx: f64,
    dy: f64,
    nx: usize,
    ny: usize,
    tension: f64,
    fade: Option<(f64, f64, f64)>,
    zero_ref_x: f64,
    zero_ref_y: f64,
}

fn load_mesh(path: &str) -> Result<SurfaceTransform, String> {
    let text = fs::read_to_string(path).map_err(|e| format!("cannot read {path}: {e}"))?;
    let request: BedMeshRequest =
        serde_json::from_str(&text).map_err(|e| format!("{path}: {e}"))?;
    SurfaceTransform::activate(ProbedMesh {
        grid: MeshGridSpec {
            x_min: request.x_min,
            y_min: request.y_min,
            dx: request.dx,
            dy: request.dy,
            nx: request.nx,
            ny: request.ny,
            tension: request.tension,
        },
        points: request.points,
        fade: request.fade,
        zero_reference: (request.zero_ref_x, request.zero_ref_y),
    })
    .map_err(|e| format!("set_bed_mesh: {e}"))
}

fn stream_inputs(
    steps: Vec<PlanStep>,
    mesh: Option<Arc<SurfaceTransform>>,
    gcode_z_rebase: f64,
) -> Vec<StreamInput> {
    let mut inputs = Vec::with_capacity(steps.len() + 1);
    if let Some(mesh) = mesh {
        inputs.push(StreamInput::Control(Control::SetMesh {
            mesh: Some(mesh),
            gcode_z_rebase,
        }));
    }
    for step in steps {
        step.feed(|input| {
            inputs.push(input);
            true
        });
    }
    inputs
}

fn non_finite(seg: &ContinuousSegment) -> Option<String> {
    if !(seg.t_start.is_finite() && seg.t_end.is_finite()) {
        return Some(format!(
            "segment span [{}, {}] is not finite",
            seg.t_start, seg.t_end
        ));
    }
    for axis in 0..seg.axes.len() {
        for t in [seg.t_start, seg.t_end] {
            match seg.eval_axis(axis, t) {
                Ok(pva)
                    if pva.position.is_finite()
                        && pva.velocity.is_finite()
                        && pva.acceleration.is_finite() => {}
                Ok(pva) => return Some(format!("axis {axis} at t={t}: {pva:?} is not finite")),
                Err(e) => return Some(format!("axis {axis} at t={t} does not evaluate: {e}")),
            }
        }
    }
    None
}

fn main() {
    record_panics();
    let mut args: Vec<String> = env::args().collect();
    let min_worst_x = take_flag(&mut args, "--min-worst-x")
        .map(|v| v.parse::<f64>().expect("--min-worst-x takes a number"));
    let max_held_moves = take_flag(&mut args, "--max-held-moves")
        .map(|v| v.parse::<u64>().expect("--max-held-moves takes a count"));
    let digest = take_switch(&mut args, "--digest");
    let synchronous = take_switch(&mut args, "--synchronous");
    let check = take_switch(&mut args, "--check");
    let mesh_path = take_flag(&mut args, "--mesh");
    let Some(config_path) = take_flag(&mut args, "--config") else {
        eprintln!(
            "usage: gcode_replay_bench <in.gcode> --config <printer.cfg> [t_lo t_hi] \
             [--min-worst-x <x>] [--max-held-moves <n>] [--digest] [--synchronous] \
             [--mesh <bed_mesh.json>] [--check]"
        );
        process::exit(1);
    };
    if synchronous && max_held_moves.is_some() {
        eprintln!("--max-held-moves measures the staged pipeline; drop --synchronous");
        process::exit(1);
    }
    if args.len() < 2 {
        eprintln!("usage: gcode_replay_bench <in.gcode> --config <printer.cfg> [t_lo t_hi]");
        process::exit(1);
    }
    let source = fs::read_to_string(&args[1]).unwrap_or_else(|e| {
        eprintln!("cannot read {}: {e}", args[1]);
        process::exit(1);
    });
    let highlight: Option<(f64, f64)> = (args.len() >= 4).then(|| {
        (
            args[2].parse().expect("t_lo must be a number"),
            args[3].parse().expect("t_hi must be a number"),
        )
    });

    let planner = planner_config(&config_path);
    let mesh = mesh_path
        .map(|path| Arc::new(load_mesh(&path).unwrap_or_else(|e| fail(FailureKind::Mesh, &e))));
    let gcode_z_rebase = mesh
        .as_deref()
        .map_or(0.0, |mesh| MachinePos([0.0; 3]).to_gcode(Some(mesh)).z());
    let mut script = LiveScript::new(
        &source,
        &planner,
        None,
        GcodePos([0.0, 0.0, gcode_z_rebase]),
    )
    .unwrap_or_else(|e| fail(FailureKind::of(&e), &e.to_string()));
    MOVE_LINES
        .set(std::mem::take(&mut script.move_lines))
        .expect("the move lines are set once");
    let inputs = stream_inputs(std::mem::take(&mut script.steps), mesh, gcode_z_rebase);

    let wall = Instant::now();
    let (arrival_tx, arrival_rx) = crossbeam_channel::unbounded::<(f64, f64)>();
    let (digest_tx, digest_rx) = crossbeam_channel::unbounded::<ContinuousSegment>();
    let sink = move |item| {
        if let TrajectoryItem::Seg(seg) = item {
            arrival_tx
                .send((wall.elapsed().as_secs_f64(), seg.t_end))
                .expect("bench collects every arrival");
            {
                let mut at = progress();
                at.emitted_seq = Some(seg.source_line);
                at.stream_t = Some(seg.t_end);
            }
            if let Some(problem) = check.then(|| non_finite(&seg)).flatten() {
                record_failure(FailureKind::NonFinite, &problem, None);
                return false;
            }
            if digest {
                digest_tx.send(seg).expect("bench digests every segment");
            }
        }
        true
    };

    let submitted = inputs.len() as u64;
    let pipeline = Pipeline::new(
        script.config.clone(),
        script.chains.clone(),
        script.home.clone(),
        0.0,
    );
    let outcome = if synchronous {
        replay_synchronous(pipeline, inputs, sink).then_some(None)
    } else {
        replay_staged(pipeline, inputs, sink).map(Some)
    };
    let wall_s = wall.elapsed().as_secs_f64();
    let arrivals: Vec<(f64, f64)> = arrival_rx.try_iter().collect();
    let Some(deepest_held) = outcome else {
        if FAILED.load(Ordering::SeqCst) {
            process::exit(EXIT_FAILURE);
        }
        fail(FailureKind::PipelineClosed, "the pipeline closed its input");
    };
    let stream_s = arrivals.last().map_or(0.0, |&(_, t)| t);
    let held_field =
        deepest_held.map_or(String::new(), |held| format!(" deepest_held_moves={held}"));
    println!(
        "moves={submitted} segments={} stream_secs={stream_s:.3} wall_secs={wall_s:.1} overall_x={:.2}{held_field}",
        arrivals.len(),
        stream_s / wall_s
    );
    if digest {
        let mut hasher = std::hash::DefaultHasher::new();
        for seg in digest_rx.try_iter() {
            format!("{seg:?}").hash(&mut hasher);
        }
        println!("trajectory_digest={:016x}", hasher.finish());
    }
    if check {
        println!("check ok: {} segments evaluate finite", arrivals.len());
    }
    println!("stage breakdown (wall share, calls, per move):");
    for total in stage_totals() {
        let secs = total.total_us as f64 * 1e-6;
        println!(
            "  {:<28} {secs:8.3}s {:6.1}%  calls={:<8} {:7.1} us/move",
            total.stage.name(),
            100.0 * secs / wall_s,
            total.calls,
            total.total_us as f64 / submitted as f64
        );
    }

    // Sliding stream-time windows: wall time the producer spent emitting each.
    const WINDOW_S: f64 = 5.0;
    let mut windows: Vec<(f64, f64, f64)> = Vec::new();
    let mut lo = 0usize;
    for hi in 0..arrivals.len() {
        while arrivals[hi].1 - arrivals[lo].1 > WINDOW_S {
            lo += 1;
        }
        if hi > lo {
            let stream = arrivals[hi].1 - arrivals[lo].1;
            let wall = arrivals[hi].0 - arrivals[lo].0;
            if wall > 1e-9 && stream >= WINDOW_S * 0.8 {
                windows.push((arrivals[lo].1, arrivals[hi].1, stream / wall));
            }
        }
    }
    windows.sort_by(|a, b| a.2.total_cmp(&b.2));
    println!("worst stream windows (t_lo..t_hi realtime_x):");
    let mut reported: Vec<(f64, f64, f64)> = Vec::new();
    for &(t_lo, t_hi, x) in &windows {
        if reported
            .iter()
            .any(|&(r_lo, r_hi, _)| t_lo < r_hi + WINDOW_S && t_hi > r_lo - WINDOW_S)
        {
            continue;
        }
        reported.push((t_lo, t_hi, x));
        println!("  {t_lo:9.2}..{t_hi:9.2}  {x:6.2}x");
        if reported.len() >= 10 {
            break;
        }
    }
    if let Some((h_lo, h_hi)) = highlight {
        let mut worst = f64::INFINITY;
        for &(t_lo, t_hi, x) in &windows {
            if t_lo < h_hi && t_hi > h_lo {
                worst = worst.min(x);
            }
        }
        println!("highlight {h_lo:.1}..{h_hi:.1}: worst realtime_x={worst:.2}");
    }
    if let Some(floor) = min_worst_x {
        let worst = windows.first().map_or(f64::INFINITY, |&(_, _, x)| x);
        if !(worst >= floor) {
            eprintln!("FAIL: worst window {worst:.2}x is below the {floor:.2}x floor");
            process::exit(1);
        }
        println!("budget ok: worst window {worst:.2}x >= {floor:.2}x floor");
    }
    if let Some(ceiling) = max_held_moves {
        let deepest_held = deepest_held.expect("--max-held-moves runs the staged pipeline");
        if deepest_held > ceiling {
            eprintln!(
                "FAIL: the planner absorbed {deepest_held} moves in a row without committing \
                 (ceiling {ceiling})"
            );
            process::exit(1);
        }
        println!("commit flow ok: deepest held run {deepest_held} <= {ceiling} moves");
    }
}
