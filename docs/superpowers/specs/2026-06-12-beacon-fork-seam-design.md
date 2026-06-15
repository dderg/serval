# Beacon Fork Seam: Design

Rewrite of the `dderg/beacon_klipper` integration layer onto this tree's
primitives, plus full capability validation (this merges the survey's
Spec D and Spec E into one project). Supersedes the per-seam resolutions
in [`beacon-fork-survey.md`](../../rewrite/beacon-fork-survey.md) where they differ;
the survey's audit ledger remains the touchpoint inventory.

## Scope

Full port, nothing staged out: proximity (eddy) homing, contact homing,
streaming/scanning, probe interface, accelerometer, temperature
compensation, model management, calibration flows. Every Beacon hardware
revision — the device-protocol code that varies by revision is the ~85%
of beacon.py this design does not touch.

Out of scope: Cartographer (`scanner.py`) — same seams, ported later by
repeating this recipe; any kalico-repo feature work (see Contract gaps
below).

## Repo layout and merge strategy

- Work happens on `master` of `dderg/beacon_klipper` (the fork's and
  upstream's default branch — both `master`). No project-name branch
  yet; naming is deferred. Upstream comparison/merges work regardless:
  add `upstream = beacon3d/beacon_klipper` as a remote, vendor updates
  arrive as `git merge upstream/master`.
- One new file, `beacon_kalico.py`, holds all new integration code.
- `beacon.py` keeps the device-protocol bulk untouched. Seam call sites
  become one-line delegations into `beacon_kalico`. The dead upstream
  integration classes are deleted outright: `BeaconEndstopShared`,
  `BeaconEndstopWrapper`, `BeaconContactEndstopWrapper` (trsync /
  trdispatch ceremony), `BeaconHomingState`, and the
  `homing:home_rails_*` listener pair.
- `install.sh` symlinks both files into `klippy/extras`.
- Merge audit property this preserves: upstream changes to
  device-protocol code merge clean; upstream changes at seam call sites
  conflict exactly where we edited; new upstream usage of deleted klippy
  APIs fails loudly at call time because the APIs don't exist here.

## Seam 1 — homing/endstop (the core)

`beacon_kalico.py` provides a provider class implementing the six-hook
virtual-endstop contract (`klippy/extras/homing.py:158-199`,
reference implementation `klippy/extras/sim_remote_endstop.py`):

- `setup_bridge_endstop(pin_params, axis)` — validates
  `probe:z_virtual_endstop` on Z, returns
  `RemoteMotionEndstop(printer, beacon_mcu, trsync_oid)`
  (`klippy/bridge_endstop.py:62`). One trsync total, allocated on the
  beacon MCU at config time with `config_trsync` sent directly
  (`MCU_trsync` is not reused — its constructor demands a trdispatch).
  The secondary per-stepper trsyncs and the trdispatch C relay are gone;
  the Rust RX-thread interceptor (`arm_remote_trigger`,
  `rust/motion-engine/src/bridge.rs:3217`) is the relay.
- `trip_move_begin(entry)` — device arming, mode-dependent.
  Proximity: `trsync_start` + `beacon_home trsync_oid=… trigger_reason=…
  trigger_invert=…`. Contact: latency commands +
  `beacon_contact_home trsync_oid=…`. Both: a reactor-timer
  `trsync_set_timeout` heartbeat feeding the beacon-side deadman, as
  upstream does during homing.
- `trip_move_end(entry)` — stop the heartbeat, send
  `beacon_stop_home` / `beacon_contact_stop_home`, poll the terminal
  `trsync_state` reason (provider registers its own `trsync_state`
  response handler at config time, the sim_remote_endstop pattern), fail
  loudly on any reason other than `REASON_ENDSTOP_HIT`.
- `measured_trip_position(axis, trip_pos, final_pos)` — where
  upstream's two post-home corrections move (their old home —
  the `homing:home_rails_end` listener — is dead on our G28).
  Proximity: take a distance `_sample()` after the trip, return measured
  Z. Contact: query `beacon_contact_query` for `detect_clock`, validate
  cruise phase via
  `bridge.motion_state_at(beacon_mcu, clock=detect_clock64)["z"]`
  acceleration ≈ 0 (replaces `trapq_extract_old`), return Z derived from
  the exact detect-time position. The device latches detect time before
  trsync dispatch latency, so this is more precise than the doorbell
  clock.
- `get_position_endstop()` — `z_offset` / contact equivalent, per mode.

Upstream's three `HomingMove` descents (contact probe at beacon.py:622,
BEACON_POKE at 1443, AUTO_CALIBRATE at 1539) become calls to
`phoming.trip_move(...)` (`klippy/extras/homing.py:352`) with this
provider in the entry — the same primitive `probe.py` uses — returning
`(trip_pos, final_pos)` in place of `epos`.

Mode selection (proximity vs contact) follows upstream's existing
`home_method` configuration; the provider arms whichever mode the
current operation requests.

## Seam 2 — streaming / motion history

`_get_position_at_time` (beacon.py:1032, the per-sample backbone) is
replaced by one call:
`bridge.motion_state_at(beacon_mcu, clock=sample_clock64)` —
beacon-domain ticks passed straight in; cross-MCU clock conversion
happens inside the bridge (`clock_between_mcus`,
`rust/motion-engine/src/motion_history.rs:218`). The per-stepper
`get_past_mcu_position` → `mcu_to_commanded_position` →
`kin.calc_position` round trip disappears.

The `toolhead.get_trapq()` readiness guard (beacon.py:402) becomes a
connect flag set on `klippy:connect`.

Per-sample error policy:

| Error | Meaning here | Handling |
|---|---|---|
| `QueryInFuture` | Sample arrived before host caught up | Re-query on next flush; sample is not lost |
| `BeforeRetainedWindow` | Ring wrapped (sample older than retention) | Drop sample, count it; warn on first occurrence per stream session |
| `NoHistoryForAxis` | Printer has not moved since connect | Position-less sample (distance/temp still valid), matching upstream's behavior for pre-motion samples |
| anything else | Bug | Propagate loudly |

## Seam 3 — probe wrapper and events

`BeaconProbeWrapper` already implements the restored probe interface
exactly (`run_probe`, `get_offsets`, `get_lift_speed`,
`multi_probe_begin/end` — `klippy/extras/probe.py` Spec C surface). It
stays. The unused mainline session methods (`start_probe_session` etc.)
stay too — inert here, and keeping them minimizes upstream-merge churn.

Beacon's self-sent `homing:home_rails_begin/end` events around
auto-calibrate (beacon.py:1521, 1593) keep working — our listeners
(gcode_move, z_thermal_adjust, …) are alive. The listening side
(beacon.py:2246-2275) is deleted; its job moved into
`measured_trip_position`.

## Contract gaps

Planned kalico-repo changes: zero. If the port uncovers a genuine gap in
the provider contract / `motion_state_at` / probe surface, that becomes
a separate small PR to `beacon-support` with its own justification —
never silent glue in the fork.

## Validation (the merged Spec E)

Sim-first against `tools/kalico-sim/emulators/beacon_mcu.py` via the
existing `third_party/beacon_klipper` symlink path in the runner.
First step is an emulator gap assessment: which of the flows below the
emulator already supports (likely: proximity homing, streaming) and
which need emulator extension (likely: contact mode, poke, NVM/temp-comp
reads). Extensions are part of this project.

Validation matrix — each row gets a sim test:

1. Module load + connect (fork imports cleanly, no dead-API references).
2. G28 Z, proximity mode.
3. G28 Z, contact mode (including cruise-phase validation path).
4. PROBE + PROBE_ACCURACY, and `ProbePointsHelper` consumers
   (Z_TILT_ADJUST path).
5. Streaming session producing positioned samples
   (`motion_state_at`-backed), including the three error-policy rows.
6. Scanned bed mesh.
7. BEACON_POKE.
8. AUTO_CALIBRATE flow end-to-end.
9. Accelerometer / resonance session.
10. Temperature-compensation model load and application.

Bench validation comes last, interactively (motion commands per-command
approved). The capability matrix above must be green in sim before any
bench session is proposed.

## Testing approach

- Fork-side: the sim matrix above is the functional suite. Pure-Python
  seam units (error-policy table, clock expansion, mode dispatch) get
  unit tests in the fork repo, separate files from the tested code.
- Kalico-side: no changes planned, so no new tests; existing
  `test_bridge_endstop.py` / `test_homing_trip_verify.py` already cover
  the contract the fork consumes.
