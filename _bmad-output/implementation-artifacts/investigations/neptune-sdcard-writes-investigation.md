# Investigation: Neptune EtherCAT bench — excessive SD-card writes

## Hand-off Brief

1. **What happened.** The Neptune EtherCAT bench (`ethercatpi5.local`) saturates its slow SD card with write bursts during prints and recently filled to 89% — the user attributes this to a triple-write of motion telemetry (Kalico JSONL → Vector disk buffer → VictoriaLogs).
2. **Where the case stands.** The triple-write is **Confirmed architecturally** but is bounded and buffered (not an fsync-storm). Evidence shows the capacity fill and a large share of *burst* writes come from a **coredump storm** — ~80 MB cores from a repeatedly-crashing `push-pieces-pum` thread, landing on the SD every ~10–25 min and still accumulating right now. Two more uncounted writers exist (`servo_captures` 245 MB, `ec-rt-capture.log` 46 MB).
3. **What's needed next.** Re-run the read-only disk-I/O monitor **during a print** to attribute live KB/s across the four writers (JSONL, Vector buffer `/var/lib/vector`, VictoriaLogs `/var/lib/victorialogs`, coredumps `printer_data/logs/coredumps`) — this is the one missing piece needed to rank the write sources by actual bandwidth.

## Case Info

| Field            | Value                                                                                          |
| ---------------- | ---------------------------------------------------------------------------------------------- |
| Ticket           | N/A                                                                                            |
| Date opened      | 2026-06-26                                                                                      |
| Status           | Active                                                                                          |
| System           | `ethercatpi5` (Raspberry Pi 5, IgH/Kalico EtherCAT host); root fs `/dev/mmcblk0p2` 28 G, 62% used; klipper/vector/victorialogs/moonraker all active; swap=zram, journald volatile |
| Evidence sources | Read-only SSH probes (df, du, ls, vector.toml, systemctl, /proc); local source (`rust/motion-engine/src/logging/`); disk-I/O monitor (Mac-side, currently stopped) |

## Problem Statement

(User-reported; treated as hypothesis.) The bench does far too much SD writing — ~5 MB/s write bursts pin disk util at ~100% on a slow card during prints; the fs recently filled to 89% (mostly accumulated coredumps), crashing prints a few layers in until space was freed to ~62%. A significant part of the load is claimed to be a **triple-write of motion telemetry**: Kalico writes `printer_data/logs/events/*.jsonl`; Vector tails those into an on-disk buffer; VictoriaLogs persists them again — three SD writes per motion event. (swap=zram, journald volatile → neither touches the card.)

## Evidence Inventory

| Source | Status | Notes |
| ------ | ------ | ----- |
| `vector.toml` (`/etc/kalico/vector.toml`) | Available | Confirms tail→disk-buffer→VL pipeline; 256 MiB disk buffer, `when_full=block` |
| JSONL writer source (`rust/motion-engine/src/logging/writer.rs`) | Available | 32 MiB rotation, 5 backups, fsync every 15 s, non-blocking appender |
| `df` / `du` on bench | Available | Capacity breakdown captured (see Findings) |
| VictoriaLogs service flags | Available | `-retentionPeriod=30d -retention.maxDiskSpaceUsageBytes=2GB` |
| Coredump dir + `core_pattern` | Available | Cores land on SD at `printer_data/logs/coredumps`; actively growing |
| **Live per-writer KB/s during a print** | **Missing** | The disk-I/O monitor is stopped; needed to rank writers by actual bandwidth |
| Coredump root-cause (`push-pieces-pum` crash) | Partial | Out of scope here; see related case files |

## Investigation Backlog

| # | Path to Explore | Priority | Status | Notes |
| - | --------------- | -------- | ------ | ----- |
| 1 | Run disk monitor during a print; attribute KB/s per writer dir | High | Open | The decisive measurement; the monitor only reads on the Pi |
| 2 | Confirm the monitor's coredump counter watches `printer_data/logs/coredumps`, not `/var/lib/systemd/coredump` | High | Open | `/var/lib/systemd/coredump` is empty (count 0); cores actually land on SD via `core_pattern` — a wrong watch dir under-reports |
| 3 | Quantify VictoriaLogs background-merge write amplification vs raw ingest | Medium | Open | LSM-style compaction rewrites data; a candidate for the steady-state portion of bursts |
| 4 | Decide retention/sizing for `servo_captures` (245 MB) and `ec-rt-capture.log` (46 MB) | Medium | Open | Capacity, mostly static; not in the original triple-write premise |
| 5 | Root-cause the `push-pieces-pum` crash storm | High (separate case) | Open | Cross-ref `neptune-print-crash-investigation.md`, `neptune-second-run-starvation-investigation.md` |

## Timeline of Events

| Time (2026-06-26) | Event | Source | Confidence |
| ----------------- | ----- | ------ | ---------- |
| 17:57–19:01 | `host-rust.jsonl` rotated 5× (32 MB each); 3 rotations in the 18:58–19:01 window (~99 MB / 3 min) | `ls -la events/` | Confirmed |
| 18:38 | coredump `core.push-pieces-pum.2725` (~82 MB) | `ls coredumps/` | Confirmed |
| 19:02 | coredump `core.push-pieces-pum.2842` (~88 MB) | `ls coredumps/` | Confirmed |
| 19:13 | coredump `core.python.15030` (~76 MB) | `ls coredumps/` | Confirmed |
| 19:17 | coredump `core.push-pieces-pum.21314` (~83 MB) | `ls coredumps/` | Confirmed |
| 19:15 (probe) | Root fs at 62% used; load avg 2.80 | `df -h`, `uptime` | Confirmed |

## Confirmed Findings

### Finding 1: The telemetry triple-write exists exactly as described — but each hop is buffered, not synchronous

**Evidence:** `/etc/kalico/vector.toml` — `[sources.kalico_events]` tails `/home/dderg/printer_data/logs/events/*.jsonl`; `[sinks.victorialogs.buffer] type="disk", max_size=268435488 (256 MiB), when_full="block"`, `data_dir="/var/lib/vector"`; sink posts to `http://127.0.0.1:9428/insert/jsonline` (VictoriaLogs, `-storageDataPath=/var/lib/victorialogs`).

**Detail:** A motion event hits the SD three times: (1) Kalico append to `events/*.jsonl`; (2) Vector's on-disk queue at `/var/lib/vector/buffer` (every event passes through the disk buffer, not only on backpressure); (3) VictoriaLogs persistence at `/var/lib/victorialogs`. Current on-disk sizes: events **351 MB**, vector buffer **66 MB**, VictoriaLogs **316 MB**. All three are size-bounded (JSONL: 6×32 MiB per stream ≈ 192 MiB × 2 streams; Vector: 256 MiB; VL: 2 GB / 30 d cap).

### Finding 2: The Kalico JSONL writer is SD-friendly — buffered append, fsync batched every 15 s

**Evidence:** `rust/motion-engine/src/logging/writer.rs:6-8` — `DEFAULT_MAX_BYTES = 32 MiB`, `DEFAULT_BACKUP_COUNT = 5`, `FSYNC_INTERVAL = 15 s`. `writer.rs:95-102` — `flush()` only calls `sync_all()` once ≥15 s have elapsed. `mod.rs:43-45` — wrapped in `tracing_appender::non_blocking` (lossy=false). `layer.rs:121-124` — one `write_all` per event, no per-event fsync.

**Detail:** Write #1 is a sequential append flushed to page cache, with fsync coalesced to once per 15 s. This rules out an fsync-per-event storm from Kalico as the saturation cause. Write volume during verbose prints is still high (≈99 MB / 3 min observed ≈ ~0.5 MB/s on the rust stream alone in bursts), but the *character* is benign sequential I/O, not sync thrash.

### Finding 3: A coredump storm is actively writing ~80 MB bursts to the SD and is the capacity-fill driver

**Evidence:** `/proc/sys/kernel/core_pattern = /home/dderg/printer_data/logs/coredumps/core.%e.%p.%t` (cores land **on the SD**, not on volatile/zram). `ls coredumps/`: 4 cores from today — `push-pieces-pum` at 18:38/19:02/19:17 and `python` at 19:13, ~76–88 MB apparent each (du-measured 131 MB real; sparse). Cadence ~10–25 min and accelerating during this session. `du` of `~/printer_data/logs` top consumers: events 351 MB, `servo_captures` 245 MB, coredumps 131 MB, `ec-rt-capture.log` 46 MB.

**Detail:** Each ~80 MB core is a near-synchronous kernel write completed in seconds — a multi-MB/s burst that alone can pin disk util, independent of the telemetry pipeline. Repeated at this cadence it both saturates bandwidth and is the most plausible driver of the 89% capacity fill the user attributed to "accumulated coredumps." The crashes themselves (`push-pieces-pum` = the motion-engine piece-push pump) are a **separate defect**; here they matter as a write source.

### Finding 4: `/var/lib/systemd/coredump` is empty — a measurement trap for the disk monitor

**Evidence:** `ls /var/lib/systemd/coredump/ | wc -l` → 0; real cores are at `printer_data/logs/coredumps` per `core_pattern`.

**Detail:** If the Mac-side monitor's "coredump file count" watches `/var/lib/systemd/coredump`, it will always report 0 and miss the storm entirely. The counter must point at `printer_data/logs/coredumps`.

## Deduced Conclusions

### Deduction 1: The dominant burst-write source is the coredump storm, not steady-state telemetry

**Based on:** Findings 2, 3.

**Reasoning:** Telemetry write #1 is buffered/coalesced (Finding 2); even ×3 amplification of a ~0.5 MB/s sequential stream is well under the observed ~5 MB/s burst peak. An ~80 MB synchronous core written in a few seconds is a single-event multi-MB/s burst that matches the observed saturation, and occurs every ~10–25 min.

**Conclusion:** The user's premise is *partially* correct — the triple-write is real and adds sustained load — but the headline "~5 MB/s burst" symptom and the 89% capacity fill are most consistent with the coredump storm. The two are independent and both need addressing; ranking requires the live measurement (Backlog #1).

## Hypothesized Paths

### Hypothesis 1: Triple-write telemetry is the *significant* write-load driver (user's premise)

**Status:** Open (partially supported)

**Theory:** Motion telemetry written three times dominates SD write bandwidth during prints.

**Supporting indicators:** Architecture confirmed (Finding 1); three size-bounded on-disk stores totalling ~730 MB; fast JSONL rotation during activity.

**Would confirm:** Live monitor during a print showing the summed KB/s into `events/`, `/var/lib/vector`, `/var/lib/victorialogs` is the largest contributor and approaches the saturation ceiling.

**Would refute:** Live monitor showing telemetry KB/s is a minor fraction while coredump-write spikes account for the saturation peaks.

### Hypothesis 2: VictoriaLogs background-merge amplification inflates the persist hop above 1×

**Status:** Open

**Theory:** VL's LSM-style compaction periodically rewrites stored data, so write #3 costs more SD bandwidth than raw ingest.

**Supporting indicators:** 316 MB store with 30 d / 2 GB retention; compaction is inherent to VL.

**Would confirm:** Monitor shows write spikes to `/var/lib/victorialogs` decoupled from (and exceeding) ingest rate.

**Would refute:** Writes to `/var/lib/victorialogs` track ingest 1:1 with no independent merge spikes.

## Missing Evidence

| Gap | Impact | How to Obtain |
| --- | ------ | ------------- |
| Live per-writer KB/s during a print | Ranks the four writers by actual bandwidth; settles H1 vs Deduction 1 | Re-run the Mac-side monitor during a print; add per-dir `du`/diskstats attribution |
| Monitor's coredump watch dir | If wrong, the storm is invisible in the monitor output | Verify it counts `printer_data/logs/coredumps` (Finding 4) |
| VL merge vs ingest write ratio | Settles H2 | Sample writes to `/var/lib/victorialogs` vs ingest count over a window |

## Source Code Trace

| Element | Detail |
| ------- | ------ |
| Telemetry write #1 origin | `rust/motion-engine/src/logging/layer.rs:121-124` (per-event `write_all`) via `writer.rs` `RotatingJsonlWriter` |
| Rotation/fsync policy | `rust/motion-engine/src/logging/writer.rs:6-8` (32 MiB, 5 backups, 15 s fsync), `:85-103` |
| Writer install | `rust/motion-engine/src/logging/mod.rs:32-58` (`init_logging`, non-blocking appender) |
| Write #2 (Vector buffer) | `/etc/kalico/vector.toml` `[sinks.victorialogs.buffer]` → `/var/lib/vector/buffer` |
| Write #3 (VictoriaLogs) | systemd `victorialogs` → `-storageDataPath=/var/lib/victorialogs` |
| Coredump write path | `/proc/sys/kernel/core_pattern` → `printer_data/logs/coredumps/` (crashing `push-pieces-pum` / `python`) |

## Conclusion

**Confidence:** High (attribution measured live; see Follow-up 2026-06-26)

Live measurement during a print resolves the ranking. **Steady-state SD writes are dominated by the telemetry pipeline (~176 KB/s logical), but it is a raw *double*-write — JSONL 78 KB/s + Vector disk buffer 93 KB/s — with VictoriaLogs persist negligible at 4.5 KB/s (Finding 5).** The Vector disk buffer is redundant with the durable JSONL and movable to memory, cutting ~53% of telemetry SD writes at no durability cost (Deduction 2). **Peak bursts are dominated by coredump writes** (~10 MB/s, util 92%, ~80 MB each), which the user scopes out as crash-driven but which also drive the capacity fill; cores land on the SD at `printer_data/logs/coredumps` (`/var/lib/systemd/coredump` is empty — Finding 4, a monitor trap). The Kalico JSONL writer itself is benign buffered append with 15 s-coalesced fsync (Finding 2).

## Recommended Next Steps

### Diagnostic

1. Re-run the read-only disk-I/O monitor **during a print**, attributing write KB/s to `events/`, `/var/lib/vector`, `/var/lib/victorialogs`, and `printer_data/logs/coredumps` separately. This settles H1 vs Deduction 1.
2. Verify the monitor's coredump counter watches `printer_data/logs/coredumps` (Finding 4).

### Fix direction (after measurement confirms ranking)

- **Coredump storm (capacity + burst):** stop the bleed — cap/rotate `coredumps/`, and treat the `push-pieces-pum` crash as the upstream defect (separate case). This is the highest-leverage write reduction if the live data confirms it.
- **Telemetry triple-write (sustained):** the redundancy is by design (JSONL is the durable source of truth; Vector+VL add two more on-disk copies on the *same* card). Options to evaluate without losing the source-of-truth posture: relocate Vector's `data_dir` and/or VictoriaLogs `-storageDataPath` off the SD (zram/tmpfs is volatile — would violate durability; an external/USB volume would not), or reconsider whether VL needs to persist on this bench at all vs. ship to an off-box collector.
- **Static capacity:** define retention for `servo_captures` (245 MB) and `ec-rt-capture.log` (46 MB).

## Reproduction / Verification Plan

Start a representative print, run the monitor for the first several layers, and capture: free space trend, read/write KB/s, disk util%, coredump count delta. Expected if Deduction 1 holds: util-pinning spikes coincide with new ~80 MB files appearing in `coredumps/`, with telemetry contributing a lower, steadier baseline.

## Follow-up: 2026-06-26

### New Evidence — live monitor during a print (19:33:01–19:34:54)

Read-only monitor (`scratchpad/sdmon.sh`) ran across an idle baseline, a real print, and the crash. Per-writer du-growth (logical bytes/s) attributed to each of the four dirs, plus device-level diskstats.

**Telemetry attribution, mean over the print window (21 samples):**

| Writer | Dir | Mean during print | vs source |
| ------ | --- | ----------------- | --------- |
| Write #1 — Kalico JSONL | `events/` | **78 KB/s** | 1.00× |
| Write #2 — Vector disk buffer | `/var/lib/vector` | **93 KB/s** | **1.20×** |
| Write #3 — VictoriaLogs persist | `/var/lib/victorialogs` | **4.5 KB/s** | 0.06× |

Total telemetry logical ≈ **176 KB/s**, of which **~97% is raw double-write** (JSONL + Vector buffer) and only ~3% is the VL persist. Idle baseline for all three combined was ~1.4 KB/s.

**Device-level (diskstats mmcblk0) during the print:** steady writes were low (~100–400 KB/s) punctuated by **~15 s-spaced flush bursts of 1.7–3.1 MB/s** at ~85–87% util — the 15 s `FSYNC_INTERVAL` (Finding 2) surfacing as coalesced writeback. The single largest spike was the **coredump write at the crash: 9991 KB/s device / ~16 MB/s into `coredumps/`, util 92%**, core count 4→5 (~80 MB core).

### Finding 5: VictoriaLogs is NOT the SD-write problem; the Vector disk buffer is

**Evidence:** Follow-up table — `vl` persist is 4.5 KB/s (compresses raw telemetry to ~6%), while `vec` is 93 KB/s, **1.20× the source JSONL it duplicates**.

**Detail:** The "triple-write" is in practice a **raw double-write** (JSONL + Vector's on-disk buffer, both uncompressed, Vector slightly *larger* due to buffer-format overhead) plus a negligible compressed third. The Vector disk buffer is the single most reducible steady-state writer.

### Deduction 2: Vector's disk buffer is redundant with the durable JSONL and can move to memory

**Based on:** Finding 1 (JSONL is the durable source of truth; Vector checkpoints its read offset in `data_dir`), Finding 5.

**Reasoning:** `vector.toml`'s own header states the JSONL "survive[s] plug-pull; Vector resumes from its on-disk checkpoint so restarts neither lose nor duplicate lines." The `disk` sink buffer (`when_full=block`) exists for backpressure durability when VL is down — but that durability is *already* guaranteed by the JSONL-as-source-of-truth plus the read checkpoint. A `memory` buffer with `when_full=block` keeps the same backpressure (Vector blocks, JSONL keeps accumulating, checkpoint un-advanced) without re-writing every raw line to the card.

**Conclusion:** Switching `[sinks.victorialogs.buffer] type = "disk"` → `"memory"` removes ≈93 KB/s — **~53% of steady-state telemetry SD writes** — at no durability cost. This is the highest-leverage, lowest-risk reduction the data supports.

### Hypothesis 1 — RESOLVED (partially supported, re-ranked)

**Status:** Confirmed-partial. **Resolution:** The triple-write is real and is the dominant *steady-state* SD writer during a print (~176 KB/s logical, ~1.5–3 MB/s device bursts at fsync). But it is a **raw double-write** (JSONL + Vector buffer), not a true 3× — VL persist is negligible (4.5 KB/s). The largest *instantaneous* burst remains the coredump write (~10 MB/s, util 92%), which the user scopes out as crash-driven. Net: telemetry dominates sustained load and is cheaply halvable (Deduction 2); coredumps dominate peak bursts and capacity.

## Side Findings

- `servo_captures/` holds **245 MB** of `.scap` traces (mostly dated Jun 11–25, static) — capacity, not throughput. (`du`)
- `ec-rt-capture.log` is **46 MB**, owned by root, last written Jun 24 — capacity, static. (`ls -la`)
- `klippy.log` daily rotations run 8–23 MB/day — a minor, benign sequential writer. (`du`)
- Related existing cases likely covering the `push-pieces-pum` crash root cause: `neptune-print-crash-investigation.md`, `neptune-second-run-starvation-investigation.md`.
