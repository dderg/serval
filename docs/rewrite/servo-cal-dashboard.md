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

The re-run form issues G-code through Moonraker
(`POST /printer/gcode/script`), not through `servo-cal` — add the
dashboard's origin to the bench's `moonraker.conf`:

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

`<name>` is validated against `[A-Za-z0-9_-]+`; anything else is rejected
before it ever reaches the filesystem.

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
- The re-run form reconstructs the sweep's G-code from
  `manifest.json`'s `experiment`/`steps`/`stroke_plan` — a best-effort
  rendering the operator can edit before sending, not a guarantee of exact
  parameter fidelity.
