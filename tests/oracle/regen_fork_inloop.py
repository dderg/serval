#!/usr/bin/env python3
"""Fork-side oracle adapter — in-loop sim path.

Runs the fork's klipper.elf (Linux MCU with Kalico Rust runtime) + klippy
connected to it in a Docker container for each oracle input, captures the
`[bridge-trace] move:` records emitted by `klippy/motion_toolhead.py`, and
reconstructs a 100 µs trajectory CSV with the same schema as `expected/*.csv`.

This replaces the batch-mode klippy path in `regen_fork.py` with a LIVE
klipper.elf-backed run, so the Rust engine actually receives and processes
every move (`bridge_is_none=False` in the trace lines).

Prerequisites:
  - Docker installed and `kalico-sim:latest` image built (see
    `tools/sim_klippy/Dockerfile` and `tools/sim_klippy/run_local.sh`).
  - `klipper.elf` and `motion_bridge_native.so` built inside the container
    (first run builds them; subsequent runs are incremental via cargo cache).

Invocation:
  python3 tests/oracle/regen_fork_inloop.py

Fallback: if Docker is unavailable or a run fails, the script falls back to
the batch-mode reconstruction path (`regen_fork.py`'s algorithm). Set
KALICO_ORACLE_NO_DOCKER=1 to force the batch-mode fallback.
"""
from __future__ import annotations
import math
import os
import re
import socket
import subprocess
import sys
import time
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parent.parent
INPUTS_DIR = ROOT / "inputs"
ACTUAL_DIR = ROOT / "actual_fork"
CFG = ROOT / "cfg" / "oracle.cfg"          # used only by batch fallback
CFG_LINUX = ROOT / "cfg" / "oracle_linux.cfg"  # used by the in-loop Docker path
KLIPPER_DICT = Path.home() / "Developer" / "klipper-sim" / "test" / "fixtures" / "klipper.dict"
PY = sys.executable

# Trajectory grid + cfg constants (must match cfg/oracle.cfg).
DT = 1e-4              # 100 µs sample grid
MAX_V = 300.0          # mm/s
MAX_A = 5000.0         # mm/s²
SCV = 5.0              # square_corner_velocity, mm/s
JUNCTION_DEVIATION = SCV * SCV * (math.sqrt(2.0) - 1.0) / MAX_A

MOVE_RE = re.compile(
    r"\[bridge-trace\] move: newpos=\[([^\]]+)\] speed=[\d.\-eE]+ "
    r"dx=([\d.\-eE]+) dy=([\d.\-eE]+) dz=([\d.\-eE]+) de=([\d.\-eE]+) "
    r"feedrate=([\d.\-eE]+) bridge_is_none=(True|False)"
)

IMG = "kalico-sim:latest"

# ─── Docker-based in-loop sim ──────────────────────────────────────────────

_INLOOP_SCRIPT = r"""
set -e

# Build klipper.elf (incremental).
make -j$(nproc) 2>&1 | grep -E 'Linking|error:|warning.*mismatch' || true

# Build motion_bridge_native.so (incremental).
make -f Makefile.kalico motion-bridge 2>&1 | tail -3 || true

# Remove stale macOS .dylib / misnamed .so artifacts.
rm -f klippy/chelper/c_helper.so.dSYM 2>/dev/null || true
rm -f klippy/motion_bridge.so 2>/dev/null || true

# Pre-build klippy's C helper module so klippy starts immediately without a
# ~100 s compilation delay on first launch.  Klippy checks for the .so at
# startup and skips the build if it already exists.
if [ ! -f klippy/chelper/c_helper.so ]; then
    echo "Pre-building c_helper.so ..."
    python3 -c "
import sys
sys.path.insert(0, 'klippy')
import chelper
chelper.get_ffi()
print('c_helper.so built OK')
" 2>&1 | tail -5 || true
fi

# Ensure the output directory on the volume mount exists.
mkdir -p tests/oracle/actual_fork

run_one() {
    local stem="$1"
    local gcode="$2"
    # Write the klippy log directly to the bind-mounted volume so it survives
    # container exit (the log is the only artifact we need from this run).
    local log_out="tests/oracle/actual_fork/${stem}.log"
    local elf_log="/tmp/${stem}_elf.log"
    local sim_socket="/tmp/klipper_oracle_${stem}"
    local api_socket="/tmp/klipper_api_${stem}"

    rm -f "$sim_socket" "$api_socket" "$log_out" "$elf_log"

    # Start klipper.elf (Linux MCU) — with CONFIG_KALICO_SIM=y GPIO is no-op.
    out/klipper.elf -I "$sim_socket" >"$elf_log" 2>&1 &
    local elf_pid=$!

    # Wait for the socket (klipper.elf creates it on bind).
    for i in $(seq 1 50); do
        [ -e "$sim_socket" ] && break
        sleep 0.1
    done
    if [ ! -e "$sim_socket" ]; then
        echo "[${stem}] FAIL: klipper.elf did not create socket — last elf log:"
        tail -20 "$elf_log" || true
        kill $elf_pid 2>/dev/null || true
        return 1
    fi

    # Patch oracle_linux.cfg serial path to this stem's socket.
    local cfg_tmp="/tmp/${stem}.cfg"
    sed "s|/tmp/klippy_oracle_socket|${sim_socket}|g" \
        tests/oracle/cfg/oracle_linux.cfg > "$cfg_tmp"

    # Start klippy connected to klipper.elf; log goes to the volume mount.
    python3 klippy/klippy.py "$cfg_tmp" \
        -l "$log_out" \
        -a "$api_socket" &
    local klippy_pid=$!

    # Wait for the api socket (klippy is ready).
    local api_up=0
    for i in $(seq 1 200); do
        [ -e "$api_socket" ] && api_up=1 && break
        sleep 0.1
        if ! kill -0 $klippy_pid 2>/dev/null; then
            echo "[${stem}] FAIL: klippy exited before api socket appeared"
            tail -30 "$log_out" 2>/dev/null || true
            echo "--- elf log ---"
            tail -10 "$elf_log" 2>/dev/null || true
            kill $elf_pid 2>/dev/null || true
            return 1
        fi
    done
    if [ "$api_up" -eq 0 ]; then
        echo "[${stem}] FAIL: klippy api socket never appeared"
        tail -30 "$log_out" 2>/dev/null || true
        kill $klippy_pid $elf_pid 2>/dev/null || true
        return 1
    fi

    # Give klippy a moment to complete initialisation (MCU config_send phase).
    sleep 1.5

    # Send the G-code script.  The oracle inputs already contain
    # SET_KINEMATIC_POSITION so no separate homing step is needed.
    python3 -c "
import socket, json, sys
sock = '$api_socket'
script = sys.argv[1]
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
s.settimeout(30.0)
try:
    s.connect(sock)
    msg = json.dumps({'id':1,'method':'gcode/script','params':{'script': script}}).encode() + b'\x03'
    s.sendall(msg)
    buf = b''
    while True:
        chunk = s.recv(4096)
        if not chunk: break
        buf += chunk
        if b'\x03' in buf: break
except Exception as e:
    print('send error:', e, file=sys.stderr)
finally:
    s.close()
" "$(cat $gcode)" 2>/dev/null || true

    # Let motion settle.
    sleep 2.5

    # Tear down.
    kill $klippy_pid 2>/dev/null || true
    wait $klippy_pid 2>/dev/null || true
    kill $elf_pid 2>/dev/null || true
    wait $elf_pid 2>/dev/null || true

    if [ -f "$log_out" ]; then
        local n_bridge
        n_bridge=$(grep -c 'bridge-trace.*move:' "$log_out" 2>/dev/null || echo 0)
        echo "[${stem}] done — ${n_bridge} bridge-trace move lines in log"
    else
        echo "[${stem}] FAIL: log not written to ${log_out}"
        return 1
    fi
}

"""

def _docker_available() -> bool:
    try:
        r = subprocess.run(
            ["docker", "image", "inspect", IMG],
            capture_output=True, timeout=5
        )
        return r.returncode == 0
    except Exception:
        return False


# ─── Trajectory reconstruction (same as regen_fork.py) ───────────────────

def parse_moves(log_path: Path) -> list[tuple]:
    moves = []
    inloop_bridge = False
    for line in log_path.read_text(errors="replace").splitlines():
        m = MOVE_RE.search(line)
        if m:
            _, dx, dy, dz, de, fr, is_none = m.groups()
            if is_none == "False":
                inloop_bridge = True
            moves.append((float(dx), float(dy), float(dz), float(de), float(fr)))
    return moves, inloop_bridge


def junction_v2(prev_axes_r, axes_r, prev_delta_v2, delta_v2):
    if prev_axes_r is None:
        return 0.0
    cos_theta = -sum(a * b for a, b in zip(prev_axes_r[:3], axes_r[:3]))
    sin_d2 = math.sqrt(max(0.5 * (1.0 - cos_theta), 0.0))
    cos_d2 = math.sqrt(max(0.5 * (1.0 + cos_theta), 0.0))
    if (1.0 - sin_d2) <= 0.0 or cos_d2 <= 0.0:
        return 0.0
    R_jd = sin_d2 / (1.0 - sin_d2)
    jd_v2 = R_jd * JUNCTION_DEVIATION * MAX_A
    qt = 0.25 * sin_d2 / cos_d2
    return min(jd_v2, delta_v2 * qt, prev_delta_v2 * qt)


def plan_velocities(moves):
    n = len(moves)
    md = [math.sqrt(sum(c * c for c in m[:4])) for m in moves]
    cruise = [min(m[4], MAX_V) for m in moves]
    axes_r = [tuple(c / d if d > 0 else 0.0 for c in m[:4]) for m, d in zip(moves, md)]
    delta_v2 = [2.0 * MAX_A * d for d in md]
    start_v2 = [0.0] * n
    for i in range(1, n):
        start_v2[i] = junction_v2(axes_r[i - 1], axes_r[i], delta_v2[i - 1], delta_v2[i])
    end_v2 = [0.0] * n
    for i in range(n - 1, -1, -1):
        end_v2[i] = 0.0 if i == n - 1 else start_v2[i + 1]
    for i in range(n):
        if i > 0:
            start_v2[i] = min(start_v2[i], end_v2[i - 1] + delta_v2[i])
        end_v2[i] = min(end_v2[i], start_v2[i] + delta_v2[i])
    return md, cruise, start_v2, end_v2, axes_r


def sample_trajectory(moves, md, cruise, start_v2, end_v2, axes_r):
    rows = [(0.0,) + (0.0,) * 12]
    t_cursor = 0.0
    pos = [0.0, 0.0, 0.0, 0.0]
    for _, (m, d, vcr, sv2, ev2, ar) in enumerate(zip(moves, md, cruise, start_v2, end_v2, axes_r)):
        if d <= 0:
            continue
        vs = math.sqrt(sv2); ve = math.sqrt(ev2)
        vc = min(vcr, math.sqrt(max(sv2, ev2) + 2 * MAX_A * d))
        d_acc = max(0.0, (vc * vc - sv2) / (2 * MAX_A))
        d_dec = max(0.0, (vc * vc - ev2) / (2 * MAX_A))
        d_cru = d - d_acc - d_dec
        if d_cru < 0:
            vc = math.sqrt((2 * MAX_A * d + sv2 + ev2) / 2.0)
            d_acc = (vc * vc - sv2) / (2 * MAX_A)
            d_dec = (vc * vc - ev2) / (2 * MAX_A)
            d_cru = 0.0
        t_acc = (vc - vs) / MAX_A if MAX_A > 0 else 0.0
        t_cru = d_cru / vc if vc > 0 else 0.0
        t_dec = (vc - ve) / MAX_A if MAX_A > 0 else 0.0
        t_move = t_acc + t_cru + t_dec
        t_end = t_cursor + t_move
        last_t = rows[-1][0]
        k = int(round(last_t / DT)) + 1
        while True:
            t = k * DT
            if t > t_end + 1e-9:
                break
            tau = t - t_cursor
            if tau < t_acc:
                v = vs + MAX_A * tau
                a = MAX_A
                s = vs * tau + 0.5 * MAX_A * tau * tau
            elif tau < t_acc + t_cru:
                v = vc; a = 0.0
                s = d_acc + vc * (tau - t_acc)
            else:
                td = tau - t_acc - t_cru
                v = vc - MAX_A * td
                a = -MAX_A
                s = d_acc + d_cru + vc * td - 0.5 * MAX_A * td * td
            x = pos[0] + ar[0] * s; y = pos[1] + ar[1] * s
            z = pos[2] + ar[2] * s; e = pos[3] + ar[3] * s
            vx, vy, vz, ve_ = (ar[0] * v, ar[1] * v, ar[2] * v, ar[3] * v)
            ax, ay, az, ae_ = (ar[0] * a, ar[1] * a, ar[2] * a, ar[3] * a)
            rows.append((round(t, 7), x, y, z, e, vx, vy, vz, ve_, ax, ay, az, ae_))
            k += 1
        pos = [pos[0] + ar[0] * d, pos[1] + ar[1] * d, pos[2] + ar[2] * d, pos[3] + ar[3] * d]
        t_cursor = t_end
    if rows and t_cursor > rows[-1][0]:
        rows.append((round(t_cursor, 7), pos[0], pos[1], pos[2], pos[3]) + (0.0,) * 8)
    return rows


def write_csv(path: Path, rows):
    cols = ("t", "x", "y", "z", "e", "vx", "vy", "vz", "ve", "ax", "ay", "az", "ae")
    with path.open("w") as f:
        f.write(",".join(cols) + "\n")
        for r in rows:
            f.write(",".join(f"{v:.6g}" for v in r) + "\n")


# ─── Batch fallback (regen_fork.py algorithm) ─────────────────────────────

def run_batch_fallback(stem: str) -> bool:
    """Fall back to klippy batch mode if Docker is unavailable."""
    input_gcode = INPUTS_DIR / f"{stem}.gcode"
    log_out = ACTUAL_DIR / f"{stem}.log"
    out_bin = Path("/tmp") / f"oracle_fork_{stem}.out"
    out_bin.unlink(missing_ok=True)
    log_out.unlink(missing_ok=True)
    cmd = [PY, str(REPO / "klippy" / "klippy.py"), str(CFG),
           "-i", str(input_gcode), "-o", str(out_bin),
           "-d", str(KLIPPER_DICT), "-l", str(log_out)]
    rc = subprocess.run(cmd, cwd=str(REPO), capture_output=True, text=True).returncode
    return rc == 0


# ─── Main ─────────────────────────────────────────────────────────────────

def process_log(stem: str, log_path: Path) -> bool:
    if not log_path.exists():
        print(f"[{stem}] no log found at {log_path}")
        return False
    moves, is_inloop = parse_moves(log_path)
    if not moves:
        print(f"[{stem}] no [bridge-trace] move: lines found in log")
        return False
    source = "in-loop (bridge_is_none=False)" if is_inloop else "batch (bridge_is_none=True)"
    md, cruise, sv2, ev2, ar = plan_velocities(moves)
    rows = sample_trajectory(moves, md, cruise, sv2, ev2, ar)
    csv_out = ACTUAL_DIR / f"{stem}.csv"
    write_csv(csv_out, rows)
    print(f"[{stem}] OK {len(moves)} moves -> {len(rows)} samples | source={source}")
    return True


def main():
    ACTUAL_DIR.mkdir(parents=True, exist_ok=True)
    stems = sorted(p.stem for p in INPUTS_DIR.glob("*.gcode"))
    if not stems:
        print("No oracle inputs found")
        return

    force_batch = os.environ.get("KALICO_ORACLE_NO_DOCKER", "") == "1"

    if not force_batch and _docker_available():
        print(f"[inloop] Docker available — running in-loop sim for: {stems}")
        # Logs are written directly to the volume-mounted actual_fork/ directory
        # inside the container (path tests/oracle/actual_fork/<stem>.log relative
        # to /work), so they survive container exit without a copy step.
        run_calls = ""
        for stem in stems:
            # Use the container-relative path (repo is mounted as /work).
            gcode_container = f"tests/oracle/inputs/{stem}.gcode"
            run_calls += f"run_one '{stem}' '{gcode_container}' || true\n"

        bash_script = _INLOOP_SCRIPT + run_calls + "\necho 'ALL_INLOOP_DONE'\n"

        try:
            result = subprocess.run(
                [
                    "docker", "run", "--rm",
                    "-v", f"{REPO}:/work",
                    "-w", "/work",
                    "--tmpfs", "/tmp:exec",
                    IMG,
                    "bash", "-c", bash_script,
                ],
                timeout=600,
            )
            if result.returncode != 0:
                print(f"[inloop] Docker run failed (rc={result.returncode}); falling back to batch")
                force_batch = True
        except subprocess.TimeoutExpired:
            print("[inloop] Docker run timed out; falling back to batch")
            force_batch = True
        except Exception as e:
            print(f"[inloop] Docker error: {e}; falling back to batch")
            force_batch = True

    if force_batch:
        print("[batch] Running klippy batch mode for each input")
        for stem in stems:
            run_batch_fallback(stem)

    # Process whatever logs we have.
    for stem in stems:
        log_path = ACTUAL_DIR / f"{stem}.log"
        process_log(stem, log_path)


if __name__ == "__main__":
    main()
