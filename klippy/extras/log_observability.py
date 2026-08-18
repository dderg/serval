import logging
import os

from .. import structured_log

HEARTBEAT_INTERVAL = 30.0
LAG_CHECK_INTERVAL = 60.0
LAG_THRESHOLD_BYTES = 8 * 1024 * 1024


def host_memory_snapshot(proc="/proc"):
    """Host and own-process memory numbers for the heartbeat. A page-fault
    stall of the klippy process freezes the motion pump and lands queued
    motion in the MCU's past, so memory pressure must be visible in the
    event stream. Anything this kernel does not expose (PSI on older
    kernels, macOS dev boxes) is omitted rather than faked - telemetry
    must not take klippy down."""
    fields = {}
    try:
        mem = {}
        with open(os.path.join(proc, "meminfo")) as f:
            for line in f:
                parts = line.split()
                if len(parts) >= 2:
                    mem[parts[0].rstrip(":")] = int(parts[1])
        fields["mem_available_kb"] = mem["MemAvailable"]
        fields["swap_used_kb"] = mem["SwapTotal"] - mem["SwapFree"]
    except (OSError, KeyError, ValueError):
        pass
    try:
        with open(os.path.join(proc, "self", "status")) as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    fields["own_rss_kb"] = int(line.split()[1])
                elif line.startswith("VmSwap:"):
                    fields["own_swap_kb"] = int(line.split()[1])
    except (OSError, ValueError, IndexError):
        pass
    try:
        with open(os.path.join(proc, "pressure", "memory")) as f:
            for line in f:
                parts = line.split()
                avg10 = next(p for p in parts[1:] if p.startswith("avg10="))
                fields["psi_mem_%s_avg10" % (parts[0],)] = float(
                    avg10.split("=", 1)[1]
                )
    except (OSError, StopIteration, ValueError, IndexError):
        pass
    return fields


def swapped_out_since(prev_own_swap_kb, fields):
    """kB of this process newly written to swap since the last heartbeat,
    or None. Growth here is the precise precursor of an emission-stall
    crash: the kernel just evicted klippy pages, and the next burst of
    activity pays for it in page faults."""
    cur = fields.get("own_swap_kb")
    if prev_own_swap_kb is None or cur is None or cur <= prev_own_swap_kb:
        return None
    return cur - prev_own_swap_kb


def emit_heartbeat(fields=None):
    structured_log.event(
        "observability",
        "heartbeat",
        level=logging.INFO,
        msg="pipeline heartbeat",
        **(fields or {}),
    )


def check_lag(bytes_behind, threshold=LAG_THRESHOLD_BYTES):
    return bytes_behind > threshold


class LogObservability:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.events_dir = self.printer.get_start_args().get("log_events_dir")
        self._last_stale = False
        self._last_own_swap_kb = None
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    def _handle_ready(self):
        now = self.reactor.monotonic()
        self.reactor.register_timer(
            self._heartbeat_timer, now + HEARTBEAT_INTERVAL
        )
        self.reactor.register_timer(self._lag_timer, now + LAG_CHECK_INTERVAL)

    def _heartbeat_timer(self, eventtime):
        fields = host_memory_snapshot()
        emit_heartbeat(fields)
        grew = swapped_out_since(self._last_own_swap_kb, fields)
        if grew is not None:
            structured_log.event(
                "observability",
                "host_memory_pressure",
                level=logging.WARNING,
                msg="klippy pages were swapped out since the last heartbeat "
                "- host memory pressure can stall the motion pump "
                "(anchor_underrun risk)",
                own_swap_grew_kb=grew,
                own_swap_kb=fields.get("own_swap_kb", -1),
                mem_available_kb=fields.get("mem_available_kb", -1),
            )
        self._last_own_swap_kb = fields.get("own_swap_kb")
        return eventtime + HEARTBEAT_INTERVAL

    def _vector_bytes_behind(self):
        return None

    def _lag_timer(self, eventtime):
        behind = self._vector_bytes_behind()
        if behind is not None:
            stale = check_lag(behind)
            if stale and not self._last_stale:
                logging.warning(
                    "observability: Vector shipper lagging %d bytes behind "
                    "events files — logs may not be reaching VictoriaLogs",
                    behind,
                )
                structured_log.event(
                    "observability",
                    "shipper_lag",
                    level=logging.WARNING,
                    msg="vector shipper lagging",
                    bytes_behind=behind,
                )
            self._last_stale = stale
        return eventtime + LAG_CHECK_INTERVAL


def load_config(config):
    return LogObservability(config)
