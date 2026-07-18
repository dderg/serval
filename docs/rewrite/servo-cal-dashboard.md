# servo-cal dashboard (`servo-cal serve`)

Part 3 of
[servo-calibration-automation.md](../plans/servo-calibration-automation.md):
the bench calibration dashboard. Wire formats and endpoint contracts are
binding per [servo-cal-contracts.md](servo-cal-contracts.md) — this page is
the how-to-run companion.

A run directory (`manifest.json` + `results.json` + `plot_series.json`,
[schema](servo-cal-contracts.md)) is a self-contained record of one
calibration attempt. `servo-cal serve` turns a folder of them into a set
of task pages — **gains** (gains + notches, always tuned together), **observers**, **dynamics**,
**journal** — each serving one calibration activity with only the tools
that activity needs (the page design lives in the plan's second demo
review amendment). Every page shares the same spine: a strip of that
page's runs with the ambient SDO diffs between consecutive attempts, the
relevant slice of the drive tuning grid, and the G-code console with its
session log.

## Run it on the bench

One-off (SSH session):

```sh
cd rust && cargo build --profile snapshot -p servo-ident
rust/target/snapshot/servo-cal serve --dir ~/printer_data/logs/servo_captures --port 8085
```

Permanent: from the dev machine,

```sh
scripts/install-servo-cal.sh dderg@trident.local
```

which copies
[`config/servo-cal/`](../../config/servo-cal/servo-cal.service)'s unit +
launcher into `~/servo-cal/` on the host (outside the repo, so they
survive branch switches), rewrites `User=`/paths for the remote user, and
enables the unit via the symlink form the benches' systemctl-only sudoers
allows. Re-running the script updates and restarts.

The service is branch-aware: the launcher idles (rechecking every 60 s)
when the checked-out `~/klipper` has no `rust/servo-ident`, builds and
serves when it does, and exits whenever HEAD moves — `Restart=always`
turns a flash-script pull or branch switch into an automatic
rebuild-and-restart. The binary it builds is the same one klippy resolves
for `servo-cal analyze` (`~/klipper/rust/target/snapshot/servo-cal`), so
one build feeds both.

Open `http://<bench-host>:8085/` in a browser. The run strips poll
`/api/runs` every 5 s, so a sweep's `results.json` landing on disk shows up
without a manual refresh.

The drive tuning grid reads `<captures_root>/drive_state.json` — written by
`SERVO_DUMP_TUNING` (see
[servo-tuning-profiles.md](servo-tuning-profiles.md#tuning-panel-backend)) —
and writes back through `SERVO_TUNE`, always with an explicit `MOTORS=`
list. The grid's Apply and the console both issue G-code through
Moonraker, not through `servo-cal` — add the
dashboard's origin to the bench's `moonraker.conf`:

```ini
[authorization]
cors_domains: http://<bench-host>:8085
```

The dashboard's Moonraker URL field defaults to
`http://<page-hostname>:7125` and persists per-browser in `localStorage`.
A health badge next to the field polls `GET /server/info` every few
seconds: green shows the klippy state, red says the URL is wrong,
Moonraker is down, or the origin is missing from `cors_domains` — the
three ways every button on every page silently stops working.

The topbar's STOP button fires `POST /printer/emergency_stop` on the
first click — no confirmation, since an accidental stop costs a
`FIRMWARE_RESTART` while a dialog in a real emergency costs the machine.
The click lands in the session log and the health badge flips to
klippy's shutdown state.

## Demo it without a bench

`servo-cal demo` builds three run directories from the committed fixtures
(`test/fixtures/servo_captures/*.scap.gz`, a real two-step gain sweep —
`s550` safe, `s700` target — recorded 2026-07-10), simulating three
notch-tuning attempts that share the same captures but differ in the
`0x2001.0x31` ambient value their manifests journal (3, then 1, then 0),
then runs `analyze` on each:

```sh
rust/target/snapshot/servo-cal demo /tmp/servo-cal-demo
rust/target/snapshot/servo-cal serve --dir /tmp/servo-cal-demo --port 8085
```

Open `http://127.0.0.1:8085/` — the gains page preselects the newest
analyzed run, so the PSD overlay (every gain step in its own color) and
the prefilled console command are populated before any clicking; the
ambient diff column reads the notch value change between consecutive
rows.

`servo-cal demo` also writes a `drive_state.json` for four AWD corexy
motors (`motor_a`/`motor_a1`/`motor_b`/`motor_b1`) mirroring the shipped
`PANEL_PARAMS` (gains 880/550/2273, the full notch bank with
slots 1–3 on bench-noted values and `notch_1_freq` deliberately drifted on
`motor_b` to show the grid's drift highlight, both observers, `inertia_ratio`
pinned as if set by `[motor] params:`), so every page's
grid has something plausible to render and Apply against without a
bench — Apply still tries to reach Moonraker, so on a bench-less demo it
will report a connection error in the session log, which is expected.

`servo-cal demo` resolves the fixtures directory relative to the running
binary (`<repo>/rust/target/<profile>/servo-cal` -> `<repo>/test/fixtures`);
pass `--fixtures <dir>` if the binary has been copied elsewhere.

## Endpoints

| Method | Path                              | Behavior                                                                 |
|--------|-----------------------------------|---------------------------------------------------------------------------|
| GET    | `/`, `/<bundle>.js`, `/<bundle>.css`, ... | the embedded dashboard bundle, built by bun at compile time (hash-routed task pages; no CDN) |
| GET    | `/api/runs`                       | run directories under `--dir` holding a `manifest.json`, newest mtime first: name, `mtime_utc`, experiment, tag, axis, `has_results`, and a verdict summary (`recommended_step`, `reason`, `flags`) when `results.json` exists, else `null` |
| GET    | `/api/runs/<name>/manifest`       | raw `manifest.json`; 404 with a JSON `{"error": ...}` body if missing      |
| GET    | `/api/runs/<name>/results`        | raw `results.json`; 404 if missing                                        |
| GET    | `/api/runs/<name>/plot_series`    | raw `plot_series.json`; 404 if missing                                    |
| POST   | `/api/runs/<name>/analyze`        | re-analyzes if `results.json` is missing or older than any capture/manifest file in the run dir, then returns the (possibly freshly written) `results.json` |
| GET    | `/api/drive_state`                | raw `<captures_root>/drive_state.json` (`SERVO_DUMP_TUNING`'s output, [servo-tuning-profiles.md](servo-tuning-profiles.md#tuning-panel-backend)) with one field added, `age_s` — seconds since the file's mtime, recomputed fresh on every request, never cached; 404 with a JSON `{"error": ...}` reason if the file doesn't exist yet (`SERVO_DUMP_TUNING` hasn't run against this `captures_root`) |
| GET    | `/api/live`                       | newest top-level `.scap` in `--dir` (flat captures only — run-dir and `.failed.scap` files are never candidates): `{capture: {name, size_bytes, age_s}}`, or `{capture: null}` |
| GET    | `/api/live/<file>?offset=<bytes>` | incremental decode of a (possibly growing) capture from a record-aligned byte offset: per-drive `ferr`/`torque`, `moving`, `stride` (thinned to ≤2000 points), `fs_hz`, `first_record`, and the `next_offset` to poll from; `offset=0` means "from the first record", `offset=end` attaches at the last complete record boundary (new samples only), any other offset must be a prior response's `next_offset` (misaligned fails loud), at most 5 s of backlog per response |
| GET    | `/api/live_tap?since_cycle=<u64>` | the file-less live stream (what the live page draws): relays the ethercat-rt tap at `--live-sock`, holding the last ~30 s in memory while polls keep coming (the reader hangs up ~10 s after the last poll, which turns the RT-side tap off). Always 200 with `status` `connecting` / `unreachable` (+`reason`) / `streaming`; without `since_cycle` returns just the `next_cycle` cursor ("attach now"); with it, samples strictly after the cursor — `drives:{name:{ferr,torque}}`, `moving`, `first_cycle`, `stride` (≤2000 points) — where sample `i` sits exactly at cycle `first_cycle + i*stride`: a response never spans a `cycle_index` hole, so a drop shows up as the next response's `first_cycle` jumping past the cursor and the page draws a gap |

`<name>` is validated against `[A-Za-z0-9_-]+`; anything else is rejected
before it ever reaches the filesystem.

## Task pages

Hash-routed (`#/gains`, `#/observers`, `#/dynamics`,
`#/journal`); the tuning loop is navigation between pages, not scrolling
within one. Non-journal pages are a two-column workspace: charts and the
page's run strip on the left, the page's slice of the drive tuning grid
plus the console in a sticky right rail.

Run rows select exclusively on click (click the sole selected row again to
clear); shift+click adds/removes a run from the overlay. The 📌 toggle on a
row (visible on hover) pins it: pinned runs survive plain clicks, so a
reference run stays in the overlay while single clicks switch the run being
compared against it; shift+click-deselecting a pinned run unpins it. Step
chips use the same grammar with an **all** chip as the default: every step
draws at once, click a chip to isolate that step, shift+click to add/remove
one. Trace colors carry the disambiguation: with one run selected each step
gets its own palette color (the all-gains-of-this-sweep view); with several
runs each keeps its table-swatch hue and its steps ramp toward white, so
the chips are the clutter valve when overlaying sweeps. A run's color is
assigned when it is selected and held until it is deselected — changing the
rest of the selection never reshuffles the colors of runs already on the
chart.

- **gains** — the gain and notch loop live on one page because they are
  always tuned together: the resonances the PSD shows are what keep gains
  from going higher. Gain-sweep/refine/ladder plus tracking runs (the
  page's own template launches `SERVO_MEASURE_TRACKING`, so its
  before/after strokes must land here), following-error PSD overlay (step
  chips, 20–450 Hz band marked, per-trace peak annotations), the `gains`
  and notch grids, and a detected-peak list. The spectrum
  charts (here and the accelerometer box) draw linear amplitude from a
  zero floor, clipped to 0–500 Hz — the old report's resonance-zoom view,
  where a peak is a spike, not a bump on a log floor. Following error
  converts to µm via the manifest's `counts_per_mm`; both convert the
  analyzer's Welch PSD to tone amplitude as `sqrt(2 · psd · ENBW)`
  (Hann ENBW = 1.5·Δf). The tracking metrics table heat-tints its µm
  columns (red intensity scaled between each column's best and worst
  visible value), so the low-overshoot step reads without comparing four
  drives' digits per step. Between the metrics table and the PSD sits the
  **metrics-vs-gain chart** — the old gain-report PNG's bottom-left panel:
  each selected sweep run draws worst-drive overshoot (solid, dotted
  markers), ferr rms (dashed), and ferr peak (dotted) in µm against the
  swept value (the `speed` gain when that's what varies), with a red
  dashed rung at every step flagged `resonance_detected` or
  `torque_saturated` (settle truncation stays a table badge — it's a
  capture-length artifact, not a gain quality signal). Hovering the chart
  snaps a crosshair to the nearest swept step and reads out that step's
  exact swept value plus each run's overshoot / ferr rms / ferr peak in µm
  — the values are exact instead of eyeballed off the axis. Runs whose
  manifests sweep nothing — tracking's single step — stay off the chart
  and are read in the metrics table instead. The detected-peak list (top
  spaced peaks in the band, from the newest selected run's recommended
  step when visible, else its last visible step) sits under the PSD; "→
  notch n" pushes a peak's frequency into that slot's pending edits for
  all motors (width/depth stay operator-chosen). The notch grid ships
  compact and per-motor views — nothing is written until Apply.
- **observers** — torque filter, speed observer, disturbance observer
  grids; time-domain following-error overlay (disturbance rejection is a
  time-domain signal).
- **dynamics** — `SERVO_FIT_DYNAMICS` runner and the `load` grid
  (`inertia_ratio`). Differential belt runs
  (`experiment: "differential"`, the anti-phase chirp on one AWD belt
  pair) land here too: selecting one adds a four-box FRF stack over the
  sweep band — magnitude (dB), phase (deg), coherence (0–1.05 with a
  dashed line at the analyzer's `coherence_min`), torque FRF (dB) — all
  sharing the PSD charts' hover readout (nearest sample's exact frequency,
  value, run). Dashed vertical markers on the magnitude chart flag each
  fitted mode, labeled `<freq> Hz ζ=<damping>` (ζ omitted when the fit
  didn't converge), and a compact mode table (freq / |H| dB / damping /
  coherence) sits under the charts with the drive pair and Welch segment
  count from `results.json`. Multiple selected runs overlay on the same
  boxes; markers, threshold, and table follow the newest one. The console
  prefill reconstructs `SERVO_MEASURE_DIFFERENTIAL BELT=... FREQ_START=...
  FREQ_END=... AMPLITUDE=... DURATION=... RAMP=... DWELL_MS=...
  NAME=<tag>` from the manifest's `stroke_plan` (no SPEED/ACCEL — the
  carriage never moves).
- **live** — the vendor's USB scope without the USB: opening the page
  streams following error straight from the drives, one stacked chart
  per motor on a shared y-scale (the noisy motor stands out), over a
  2–30 s slider-set window. No capture, no file, no G-code: the server
  relays the ethercat-rt live tap (`<control-socket>.live`,
  `ethercat-rt/src/live_tap.rs`) through `/api/live_tap`, and the tap
  only builds records while someone is watching. Drops under
  backpressure and tap reconnects arrive as `cycle_index` jumps and draw
  as gaps — never stale data pretending to be live. Chart titles map the
  tap's `slot<N>` names to motor names via `drive_state.json`'s `slots`
  object once a dump has run. A separate "record to file" box wraps
  `SERVO_CAPTURE_START`/`STOP` for when the session should leave an
  analyzable `.scap` behind.
- **journal** — every run across experiments, full width.

## Drive tuning grid

Renders purely from `/api/drive_state` as a param × motor table — one
column per motor plus an **all** setter — filtered to the current page's
groups (`gains`, `filters`, `notch`, `speed_observer`,
`disturbance_observer`, `load`; any unrecognized `extra_params:` group
lands in an `other` section on every grid page — nothing from the dump is
ever dropped). Cells show the raw register value with no display
conversion; the unit label names the register's LSB (e.g. "0.1 Hz"), the
same convention as the vendor manual and the drive's front panel.

- **Per-motor cells, explicit scope.** Every motor's value is always
  visible; the "all" column writes every motor at once (showing "mixed"
  when they disagree); a cell that differs from its siblings gets a drift
  highlight, a cell with an unapplied edit a pending highlight. Params
  with an `options` enum render as labeled selects.
- **The notch group is transposed by default** — one column per notch,
  freq/width/depth rows, one input per cell that stages the value for
  every motor: notches are per-axis physics, so on corexy a per-motor
  notch table is noise. A "per-motor view" toggle restores the param ×
  motor rows for drives that genuinely disagree (a mixed cell names the
  per-motor values in its tooltip either way).
- **Config-pinned params** (present in a motor's `[motor] params:` block or
  `tuning_profile`, per `drive_state.json`'s `config_pins`) get a pin badge
  showing the pinned value — editing the live value here does not survive
  a restart until the config is updated too.
- **Staleness banner** (topbar, all pages). Shows the drive state's age
  (ticking client-side between fetches) and a Refresh button that sends
  `SERVO_DUMP_TUNING` through Moonraker, then polls `/api/drive_state`
  until `age_s` drops, then re-renders — the operator's cue that an
  out-of-band edit (e.g. a hand-typed `SERVO_TUNE` on the console) is now
  reflected in the grid.
- **Apply** previews the exact pending `SERVO_TUNE PARAM=... VALUE=...
  MOTORS=<explicit list>` lines above the button (motors grouped per
  value), sends them through Moonraker, appends the batch to the
  timestamped session log, then reloads `drive_state.json` — `SERVO_TUNE`
  readback-verifies each write and patches the mapped value into
  `drive_state.json` in place, so the grid refreshes from the file in
  milliseconds; the full `SERVO_DUMP_TUNING` drive re-read stays behind
  the refresh button.
- **Console.** One terminal-style G-code line under the session log on
  every page — sweeps, manual commands, and multi-line pastes all go
  through it. Enter runs (shift+enter for a newline, `;` lines are
  skipped), ↑/↓ or ctrl+p/ctrl+n walk the history, ctrl+r
  reverse-searches it, ctrl+c clears the line; history persists in
  `localStorage` (500 entries, consecutive duplicates collapsed). It
  prefills from the newest run of the page's experiment (or any run's
  "→ console" button) and from the page's template buttons, so the loop
  reads tweak grid -> apply -> run sweep -> run strip updates. Every
  batch from Apply or the console lands in the same session log, which
  survives page switches; clicking any logged line inserts it back into
  the console. Command output is echoed under each sent line:
  `respond_info` text only travels Moonraker's websocket, so after every
  blocking `/printer/gcode/script` call the console diffs
  `/server/gcode_store` against its pre-send tail (server-side
  timestamps, so no clock agreement needed) and renders the new
  `response` entries — `SERVO_MEASURE_TRACKING`'s ferr/overshoot summary
  reads in the dashboard, not just in mainsail. `!!`-prefixed lines tint
  red; concurrent responses from another console may ride along.

## Implementation notes

- The HTTP server (`rust/servo-ident/src/http.rs`) is hand-rolled —
  `std::net::TcpListener` plus a minimal HTTP/1.1 GET/POST parser,
  thread-per-connection, no keep-alive. No new external dependency; the
  crate's existing `flate2` (previously dev-only, used to gunzip the
  committed `.scap.gz` fixtures) is now a normal dependency so `servo-cal
  demo` can unpack fixtures at runtime.
- The dashboard is a TypeScript + preact/htm SPA in `rust/servo-ident/web/`
  (`index.html`, `app.css`, `src/*.ts`, wire types generated by ts-rs in
  `src/generated/`). `build.rs` bundles it with bun at compile time and the
  resulting assets are embedded into the binary (`src/assets.rs`), so the
  server always serves the built output of the checked-in sources.
- Charts never re-run a sweep; they only draw `plot_series.json` already
  on disk for the selected rows (cached per run mtime, so reselecting is
  free).
- The console prefill reconstructs its G-code from `manifest.json`'s
  `experiment`/`steps`/`stroke_plan` — a best-effort rendering the operator
  can edit before sending, not a guarantee of exact parameter fidelity.
- The drive panel's pure logic (changed-param diffing, SERVO_TUNE line
  building) is a handful of plain functions in the `web/src` modules
  (`groupParams`, `motorRawValues`, `valuesAgree`, `pinnedEntries`,
  `diffChangedParams`, `buildServoTuneCommands`);
  `rust/servo-ident/tests/drive_panel_spa.rs` asserts the modules still
  define them and that the demo's `drive_state.json` matches what they
  assume, alongside the bun test suite in `web/test/`.
