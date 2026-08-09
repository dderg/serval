# Serval architecture

Serval is a **host-planned, MCU-executed** motion system built on the Kalico/Klipper host and firmware estate. It deliberately replaces the conventional host step-queue path for configured motion axes. This page is an implementation-oriented map for operators and contributors; it describes the current `sota-motion` code, not a compatibility promise.

> **Safety boundary:** the host, native extensions, and flashed MCU firmware are one protocol version. Do not mix a Serval host with firmware built from another branch. Rebuild the native artifacts and reflash every participating MCU after switching versions. See [Quickstart](Quickstart.md).

## System boundary

| Layer | Main locations | Responsibility |
| --- | --- | --- |
| Configuration and control plane | `klippy/`, especially `configfile.py`, `motion_setup.py`, `motion_kinematics.py` | Parses configuration, accepts G-code, owns printer lifecycle, validates topology, and bridges Python objects to the native engine. |
| Native host engine | `rust/motion-engine`, `rust/motion-core`, `rust/motion-pipeline` | Owns streamed motion, geometry fitting, velocity planning, lowering, post-processing, dispatch, pump, timing, and recovery bookkeeping. `motion-engine` exposes the PyO3 module loaded as `klippy._motion_engine`. |
| Transport and protocol | `rust/mcu-protocol`, `rust/mcu-transport`, `klippy/mcu.py` | Converts planned pieces and control traffic to live MCU/endpoint connections and observes clock/ring state. |
| Execution firmware | `src/`, `rust/c-api`, `rust/runtime` | Receives trajectory pieces, evaluates continuous positions at the configured sample cadence, and drives step/dir or phase outputs. |
| Optional servo endpoint | `rust/ethercat-rt`, `klippy/extras/servo_*.py` | Supplies position targets and drive control for EtherCAT servo nodes; it is separate from step-pulse MCU execution. |

## Motion data path

1. G-code and printer objects submit geometric moves through the Python motion integration.
2. `motion-core::worker::setup_pipeline` starts the pipeline once at boot. It wires **fit → planner → lowerer → shaper → dispatcher → pump**; the worker entry is `rust/motion-core/src/worker.rs`.
3. The fitter turns input line/curve geometry into a smooth path under the configured deviation budget. The planner applies path, velocity, acceleration, and (when enabled) jerk constraints over its look-ahead. The lowerer produces per-axis polynomial position tracks.
4. Each axis's configured post-processor chain transforms its track. This includes smoothing/input-shaping-like kernels and pressure-advance operators. Limits apply to the resulting motor command, not merely the nominal path.
5. The dispatcher anchors segments into each endpoint's clock domain and routes axis lanes. The pump maintains lead, sends pieces, handles drain/fence traffic, and treats missed timing as a fault rather than silently retiming motion.
6. Firmware stores and evaluates the pieces at its selected sample rate. A stepper output turns the evaluated position into step/dir edges or phase commands; a servo endpoint consumes position targets.

The critical implication is that the MCU has a trajectory buffer, not a pre-expanded list of step times. A host stall is therefore tolerable only while buffered lead remains. Homing deliberately uses a much shorter window and is correspondingly more sensitive to host and transport stalls; see [Feature status](Feature_Status.md).

## Configuration model

The model separates **axes** (planned coordinates) from **motors** (physical actuators) and **kinematics** (the mapping between them):

- `[kinematics]` declares the supported mapping (`cartesian` or `corexy`) and the motors for its X/Y/Z lanes.
- `[axis <name>]` declares a coordinate. Kinematic rails carry travel/homing settings. A follower can derive its displacement from other axes.
- `[motor <name>]` declares a stepper or EtherCAT servo actuator.
- `[post_processor <name>]` declares a reusable operator chain element attached to an axis.

An extruder is normally a follower axis, rather than a special planner type. This lets its requested displacement follow the actual path length of its declared axes. The complete accepted schema, defaults, and validation rules live in [Motion configuration reference](Config_Reference_Motion.md); use [Config migration](Config_Migration.md) when moving a classic configuration.

## Concurrency, timing, and failure model

The pipeline stages run as dedicated streaming stages. The pump and dispatcher cross endpoint clock domains; the native bridge also runs endpoint calls without blocking Klippy's reactor. Bounded channels intentionally create backpressure instead of unbounded queued latency. In particular, `INPUT_CHANNEL_CAP` in `worker.rs` bounds incoming motion commands.

This is not a best-effort real-time system. Serval locks the host process memory before its motion threads start, and a production service needs `LimitMEMLOCK=infinity`. Disk swap should be disabled. These are operational requirements, not performance tuning; follow [Installation: host memory requirements](Installation.md#host-memory-requirements).

When the system cannot preserve the timing contract—an exhausted trajectory lead, a dead endpoint, an invalid topology, a protocol mismatch, or a timing error—it should stop and report the fault. Do not treat a fault as a request to retry a print blindly: inspect the log, correct the cause, home again, and verify position before resuming.

## Build artifacts and source map

`scripts/build-native.sh` is the supported entry point for host artifacts. Its normal build creates:

- `klippy/_config_doc.so` — native configuration document/parser support;
- `klippy/_motion_engine.so` — the PyO3 host motion engine; and
- `klippy/_shaper_ident.so` — resonance-identification numeric core.

The Rust workspace is declared in `rust/Cargo.toml`. Useful ownership landmarks are `geometry` (path/velocity geometry), `trajectory` (tracks and algorithms), `planner-config` (motion schema), `motion-core` (streaming orchestration), `runtime`/`c-api` (embedded runtime boundary), and `pipeline-snapshot` (offline regression snapshots). This map is intentionally high-level: module APIs are internal unless their configuration or G-code surface is documented elsewhere.

## Related reading

- [README](../README.md) — rationale, pipeline overview, and supported concepts.
- [Quickstart](Quickstart.md) — installation/upgrade sequence.
- [Feature status](Feature_Status.md) — support tiers and known limits.
- [Developer guide](Development.md) — build, test, and review workflows.
- `docs/rewrite/` — design and bring-up notes. These are valuable engineering records but are not stable operator instructions.
