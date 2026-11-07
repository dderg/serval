"""Pi-side recorder for one bench run (stdlib only; runs under the Pi's python3).

    python3 pi_agent.py OUT_DIR --pid PID [--steppers stepper_x,...]

Samples the klippy process's CPU time (whole process and per thread, from
/proc) every PERIOD_S into OUT_DIR/cpu.jsonl. With --steppers it subscribes
to klippy's motion_report/dump_stepper endpoint over the API socket for each
stepper and appends every message verbatim to OUT_DIR/tap.jsonl: the
queue_step schedule the live host sent, the commanded side of the
MCU-execution comparison. Runs until SIGTERM, then flushes and exits 0.
"""

import argparse
import json
import os
import signal
import socket
import threading
import time

PERIOD_S = 0.25
ETX = b"\x03"


def proc_times(path):
    with open(path) as f:
        text = f.read()
    comm_end = text.rindex(")")
    comm = text[text.index("(") + 1 : comm_end]
    fields = text[comm_end + 2 :].split()
    return comm, int(fields[11]), int(fields[12])


def sample(pid):
    _, utime, stime = proc_times("/proc/%d/stat" % pid)
    tasks = {}
    for tid in os.listdir("/proc/%d/task" % pid):
        try:
            tasks[tid] = proc_times("/proc/%d/task/%s/stat" % (pid, tid))
        except FileNotFoundError:
            continue
    with open("/proc/loadavg") as f:
        load = f.read().split()[0]
    return {
        "mono": time.clock_gettime(time.CLOCK_MONOTONIC_RAW),
        "utime": utime,
        "stime": stime,
        "tasks": tasks,
        "loadavg": float(load),
    }


def cpu_loop(pid, out_path, stop):
    with open(out_path, "w", buffering=1) as out:
        out.write(json.dumps({"clk_tck": os.sysconf("SC_CLK_TCK")}) + "\n")
        while not stop.is_set():
            out.write(json.dumps(sample(pid)) + "\n")
            stop.wait(PERIOD_S)
        out.write(json.dumps(sample(pid)) + "\n")


def tap_loop(uds, steppers, out_path, stop, ready):
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.connect(uds)
    for index, name in enumerate(steppers):
        request = {
            "id": index + 1,
            "method": "motion_report/dump_stepper",
            "params": {"name": name, "response_template": {"tap": name}},
        }
        sock.sendall(json.dumps(request).encode() + ETX)
    sock.settimeout(0.2)
    pending = b""
    answered = set()
    with open(out_path, "wb") as out:
        while not stop.is_set():
            try:
                chunk = sock.recv(1 << 16)
            except socket.timeout:
                continue
            if not chunk:
                raise RuntimeError("klippy closed the API socket")
            pending += chunk
            *messages, pending = pending.split(ETX)
            for message in messages:
                decoded = json.loads(message)
                if "id" in decoded:
                    if "error" in decoded:
                        raise RuntimeError("subscription failed: %r" % decoded)
                    answered.add(decoded["id"])
                    if len(answered) == len(steppers):
                        ready.set()
                out.write(message + b"\n")
    sock.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("out_dir")
    parser.add_argument("--pid", type=int, required=True)
    parser.add_argument("--steppers", default="")
    parser.add_argument("--uds", default="/tmp/klippy_uds")
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    failures = []

    def guarded(target, *a):
        def run():
            try:
                target(*a)
            except BaseException as exc:
                failures.append(repr(exc))
                stop.set()

        return threading.Thread(target=run)

    threads = [
        guarded(
            cpu_loop, args.pid, os.path.join(args.out_dir, "cpu.jsonl"), stop
        )
    ]
    ready = threading.Event()
    steppers = [s for s in args.steppers.split(",") if s]
    if steppers:
        threads.append(
            guarded(
                tap_loop,
                args.uds,
                steppers,
                os.path.join(args.out_dir, "tap.jsonl"),
                stop,
                ready,
            )
        )
    else:
        ready.set()
    for thread in threads:
        thread.start()
    while not stop.is_set():
        if ready.is_set():
            with open(os.path.join(args.out_dir, "ready"), "w") as f:
                f.write("ok\n")
            ready.clear()
        stop.wait(0.05)
    for thread in threads:
        thread.join()
    with open(os.path.join(args.out_dir, "agent.json"), "w") as f:
        json.dump({"failures": failures}, f)


if __name__ == "__main__":
    main()
