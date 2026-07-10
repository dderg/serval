# servo-cal dashboard (`servo-cal serve`)

Part 3 of
[servo-calibration-automation.md](../plans/servo-calibration-automation.md):
the bench calibration dashboard. Wire formats and endpoint contracts are
binding per [servo-cal-contracts.md](servo-cal-contracts.md) — this page is
the how-to-run companion.

A run directory (`manifest.json` + `results.json` + `plot_series.json`,
[schema](servo-cal-contracts.md)) is a self-contained record of one
calibration attempt. `servo-cal serve` turns a folder of them into a
journal: every attempt as a row, the ambient SDO parameters that changed
since the previous attempt, headline metrics per step, and a form to fire
the next attempt without leaving the page.

## Run it on the bench

```sh
cd rust && cargo build --release -p servo-ident
rust/target/release/servo-cal serve --dir ~/printer_data/logs/servo_captures --port 8085
```

Open `http://<bench-host>:8085/` in a browser. The journal polls
`/api/runs` every 5 s, so a sweep's `results.json` landing on disk shows up
without a manual refresh.

The drive tuning panel reads `<captures_root>/drive_state.json` — written by
`SERVO_DUMP_TUNING` (see
[servo-tuning-profiles.md](servo-tuning-profiles.md#tuning-panel-backend)) —
and writes back through `SERVO_TUNE`, one register at a time, applied to
every motor unless the panel had to expand a mixed row per motor. Both the
panel's Apply and the sweep row's Run, like the re-run form, issue G-code
through Moonraker, not through `servo-cal` — add the dashboard's origin to
the bench's `moonraker.conf`:

```ini
[authorization]
cors_domains: http://<bench-host>:8085
```

The dashboard's Moonraker URL field defaults to
`http://<page-hostname>:7125` and persists per-browser in `localStorage`.

## Demo it without a bench

`servo-cal demo` builds three run directories from the committed fixtures
(`test/fixtures/servo_captures/*.scap.gz`, a real two-step gain sweep —
`s550` safe, `s700` target — recorded 2026-07-10), simulating three
notch-tuning attempts that share the same captures but differ in the
`0x2001.0x31` ambient value their manifests journal (3, then 1, then 0),
then runs `analyze` on each:

```sh
rust/target/release/servo-cal demo /tmp/servo-cal-demo
rust/target/release/servo-cal serve --dir /tmp/servo-cal-demo --port 8085
```

Open `http://127.0.0.1:8085/` — the journal shows three rows, newest first,
each an `s700`-recommended clean gain sweep, with the ambient diff column
reading the notch value change between consecutive rows. Tick two rows and
click "overlay selected" to see their `s550`/`s700` following-error traces
drawn together.

`servo-cal demo` also writes a `drive_state.json` for four AWD corexy
motors (`motor_a`/`motor_a1`/`motor_b`/`motor_b1`) mirroring the shipped
`PANEL_PARAMS` (gains 880/550/2273, `freq_cutoff` 220, `gain_mode`/
`inertia_ratio` 0/150 pinned as if set by `[motor] params:`, three
unidentified experimental registers), so the drive tuning panel has
something plausible to render and Apply against without a bench — Apply
still tries to reach Moonraker, so on a bench-less demo it will report a
connection error in the panel's status line and session log, which is
expected.

`servo-cal demo` resolves the fixtures directory relative to the running
binary (`<repo>/rust/target/<profile>/servo-cal` -> `<repo>/test/fixtures`);
pass `--fixtures <dir>` if the binary has been copied elsewhere.

## Endpoints

| Method | Path                              | Behavior                                                                 |
|--------|-----------------------------------|---------------------------------------------------------------------------|
| GET    | `/`, `/app.js`, `/app.css`        | the embedded single-page dashboard (no build step, no CDN)                 |
| GET    | `/api/runs`                       | run directories under `--dir` holding a `manifest.json`, newest mtime first: name, `mtime_utc`, experiment, tag, axis, `has_results`, and a verdict summary (`recommended_step`, `reason`, `flags`) when `results.json` exists, else `null` |
| GET    | `/api/runs/<name>/manifest`       | raw `manifest.json`; 404 with a JSON `{"error": ...}` body if missing      |
| GET    | `/api/runs/<name>/results`        | raw `results.json`; 404 if missing                                        |
| GET    | `/api/runs/<name>/plot_series`    | raw `plot_series.json`; 404 if missing                                    |
| POST   | `/api/runs/<name>/analyze`        | re-analyzes if `results.json` is missing or older than any capture/manifest file in the run dir, then returns the (possibly freshly written) `results.json` |
| GET    | `/api/drive_state`                | raw `<captures_root>/drive_state.json` (`SERVO_DUMP_TUNING`'s output, [servo-tuning-profiles.md](servo-tuning-profiles.md#tuning-panel-backend)) with one field added, `age_s` — seconds since the file's mtime, recomputed fresh on every request, never cached; 404 with a JSON `{"error": ...}` reason if the file doesn't exist yet (`SERVO_DUMP_TUNING` hasn't run against this `captures_root`) |

`<name>` is validated against `[A-Za-z0-9_-]+`; anything else is rejected
before it ever reaches the filesystem.

## Drive tuning panel

Renders purely from `/api/drive_state`, grouped into the sections
`PANEL_PARAMS` assigns (`gains`, `filters`, `notch`, `load`,
`experimental`), plus an `other` section for any `extra_params:` group the
panel doesn't otherwise recognize — nothing from the dump is ever dropped.

- **One field per param.** When every motor reads the same raw value the
  row is a single input, pre-filled with the display value (`raw / scale`)
  and a small "raw `<n>`" hint; when motors disagree the row shows a
  "mixed" badge that expands to one input per motor on click.
- **Config-pinned params** (present in a motor's `[motor] params:` block or
  `tuning_profile`, per `drive_state.json`'s `config_pins`) get a pin badge
  showing the pinned value, titled "restart re-applies this" — editing the
  live value here does not survive a restart until the config is updated
  too.
- **Autofill.** Editing `speed_gain` live-derives `position_gain`
  (`round(raw * 1.6)`) and `integral_time` (`round(1250000 / raw)`) unless
  the operator has edited that field directly this session (dirty-tracked
  per field); a "re-derive" link restores the linkage.
- **Staleness banner.** Shows the drive state's age (ticking client-side
  between fetches) and a Refresh button that sends `SERVO_DUMP_TUNING`
  through Moonraker, then polls `/api/drive_state` until `age_s` drops,
  then re-renders — the operator's cue that an out-of-band edit (e.g. a
  hand-typed `SERVO_TUNE` on the console) is now reflected in the panel.
- **Apply** builds the minimal `SERVO_TUNE` command list — only params that
  actually changed, one call per param when every motor gets the same
  value, one call per touched motor (`MOTORS=<name>`) when a row was
  expanded — previews the exact lines in the session's G-code textarea,
  sends them through the existing Moonraker plumbing, appends the batch to
  a scrollable, timestamped "sent this session" log, then refreshes the
  drive state.
- **Sweep row.** Sits under the panel: the reconstructed sweep command
  (same `manifest.json`-derived reconstruction the old re-run form used)
  with its own Run button, so the loop reads tweak panel -> apply -> run
  sweep -> journal updates.
- The G-code textarea stays as the manual escape hatch for arbitrary lines
  and doubles as the last-sent preview; every batch sent from Apply, Run
  sweep, or the textarea's own Run button lands in the same session log.

## Implementation notes

- The HTTP server (`rust/servo-ident/src/http.rs`) is hand-rolled —
  `std::net::TcpListener` plus a minimal HTTP/1.1 GET/POST parser,
  thread-per-connection, no keep-alive. No new external dependency; the
  crate's existing `flate2` (previously dev-only, used to gunzip the
  committed `.scap.gz` fixtures) is now a normal dependency so `servo-cal
  demo` can unpack fixtures at runtime.
- The dashboard (`rust/servo-ident/src/web/{index.html,app.js,app.css}`) is
  embedded into the binary via `include_str!` — vanilla DOM + `<canvas>`,
  no framework, no build step.
- The overlay drill-down never re-runs a sweep; it only draws
  `plot_series.json` already on disk for the ticked rows.
- The sweep row reconstructs its G-code from `manifest.json`'s
  `experiment`/`steps`/`stroke_plan` — a best-effort rendering the operator
  can edit before sending, not a guarantee of exact parameter fidelity.
- The drive panel's pure logic (display/raw unit conversion, autofill
  derivation, changed-param diffing) is a handful of plain functions in
  `app.js` (`rawToDisplay`, `displayToRaw`, `deriveGainPositionFromSpeed`,
  `deriveGainIntegralFromSpeed`, `groupParams`, `motorRawValues`,
  `valuesAgree`, `pinnedEntries`, `diffChangedParams`,
  `buildServoTuneCommands`) rather than behind a Node toolchain this crate
  doesn't otherwise need; `rust/servo-ident/tests/drive_panel_spa.rs` is
  the substitute test rig, asserting the served `app.js` still defines them
  and that the demo's `drive_state.json` matches what they assume.
