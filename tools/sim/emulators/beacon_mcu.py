from __future__ import annotations

import logging
import mmap
import os
import pty
import select
import struct
import threading
import time
from typing import Optional

from klippy import msgproto
from tools.sim.emulators.beacon_identify_dict import (
    CLOCK_FREQ,
    IDENTIFY_BLOB,
)


def _build_nvm_image() -> bytes:
    nvm = bytearray(65536 + 8)
    # beacon.py decodes offset 0 as f_count(u32)+adc_count(u16); 0xFFFFFFFF /
    # 0xFFFF are its "no calibration data" sentinels. Offset 12 is the model
    # version byte; 0 selects the V0 (no-params) branch.
    struct.pack_into("<IH", nvm, 0, 0xFFFFFFFF, 0xFFFF)
    nvm[12] = 0x00
    mcu_temp_cal_offset = 65534
    temp_room_c, temp_hot_c = 20, 60
    adc_room, adc_hot = 1000, 3000
    lower = temp_room_c | (temp_hot_c << 12)
    upper = (adc_room << 8) | (adc_hot << 20)
    struct.pack_into("<II", nvm, mcu_temp_cal_offset, lower, upper)
    return bytes(nvm)


NVM_IMAGE = _build_nvm_image()

DEFAULT_FREQUENCY_HZ = 5_400_000

DEFAULT_TEMP_RAW = 2048

APPROACH_FROM_ABOVE_Z_MM = 10.0


class BeaconMcuStub:
    SAMPLE_RATE_HZ = 1600.0
    BATCH_HZ = 200.0
    SAMPLES_PER_BATCH = 8
    BATCH_PERIOD_S = 1.0 / BATCH_HZ
    IDENTIFY_BLOB = IDENTIFY_BLOB
    CLOCK_FREQ = CLOCK_FREQ
    STUB_NAME = "beacon-stub"

    def __init__(
        self,
        pty_path: str,
        log_path: Optional[str] = None,
        step_sock_path: Optional[str] = None,
        z_steps_per_mm: float = 800.0,
        z_step_line: int = 15,
        z_step_sign: int = 1,
        vtime_shm_name: Optional[str] = None,
    ) -> None:
        # The stub's MCU clock must tick with the world's virtual clock,
        # not the wall clock: virtual time legally slips against real time
        # (pacer floors hold it while the motion tick catches up), and this
        # clock is what klippy's clocksync maps trigger clocks through. A
        # wall-clock stub drifts against the steppers by the accumulated
        # slip, which lands reconstructed contact positions seconds away.
        self._vtime_map: Optional[mmap.mmap] = None
        if vtime_shm_name:
            with open("/dev/shm" + vtime_shm_name, "r+b") as f:
                self._vtime_map = mmap.mmap(f.fileno(), 8)
        self._pty_path = pty_path
        self._log_path = log_path
        self._step_sock_path = step_sock_path
        self._z_steps_per_mm = z_steps_per_mm
        self._z_step_line = z_step_line
        self._z_step_sign = z_step_sign
        self._z_anchor_mm = 10.0
        self._z_anchor_steps = 0
        self._steps_now = 0
        self._step_tracking = False
        self._step_thread: Optional[threading.Thread] = None
        self._z_line_locked = False
        self._line_baselines = {}
        self._z_target: float = 10.0
        self._stream_en: bool = False
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._sample_thread: Optional[threading.Thread] = None
        self._master_fd: Optional[int] = None
        self._slave_fd: Optional[int] = None
        self._t0: float = time.monotonic()
        self._send_lock = threading.Lock()
        self._host_recv_seq: int = 1
        self._inbuf = bytearray()
        self._parser = msgproto.MessageParser(warn_prefix=self.STUB_NAME + ": ")
        self._parser.process_identify(self.IDENTIFY_BLOB, decompress=True)
        self._handlers = self._build_handlers()
        self.rx_byte_count: int = 0
        self.tx_sample_count: int = 0
        self.tx_frame_count: int = 0
        self._threshold_trigger: int = 0
        self._threshold_untrigger: int = 0
        self._home_active: bool = False
        self._home_trsync_oid: int = 0
        self._home_trigger_reason: int = 0
        self._home_trigger_invert: int = 0
        self._is_configured: bool = False
        self._committed_crc: int = 0
        self._trsync_oids: set = set()
        self._trsync_can_trigger: dict = {}
        self._trsync_trigger_reason: dict = {}
        self._accel_stream_en: bool = False
        self._accel_scale_id: int = 0
        self._accel_thread: Optional[threading.Thread] = None
        self._accel_clock_at_last_emit: int = 0
        self._sample_index: int = 0
        self._next_batch_vt: float = 0.0
        self._last_batch_vt: Optional[float] = None
        self._clock_origin = self._monotonic()

        self._homing_trigger_delay: float = 0.5
        self._homing_trigger_timer: Optional[threading.Event] = None
        self._contact_homing_active: bool = False
        self._contact_armed_clock: int = 0
        self._contact_trsync_oid: int = 0
        self._contact_trigger_reason: int = 1
        self._contact_trigger_clock: int = 0
        self._contact_trigger_sample: int = 0
        self._contact_trigger_freq: int = 0
        self._contact_triggered: bool = False
        self.contact_latch_commit_delay: float = 0.0
        self._contact_latch_timer: Optional[threading.Event] = None

        self._z_current: float = 10.0
        self._prev_poll_time: Optional[float] = None
        self._prev_poll_z: float = 10.0
        self._prev2_poll_time: Optional[float] = None
        self._prev2_poll_z: float = 10.0
        self._freq_base: int = 5_183_000
        self._freq_coeff: float = 763_000.0
        self._freq_offset: float = 2.857
        self._homing_approach_speed: float = 5.0
        self._homing_start_z: float = 10.0
        self._homing_start_time: float = 0.0

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        master_fd, slave_fd = pty.openpty()
        slave_name = os.ttyname(slave_fd)
        # Raw mode: the line discipline must not cook (echo, NL->CRLF) the
        # binary serial wire.
        import termios as _termios
        import tty as _tty

        try:
            _tty.setraw(slave_fd, _termios.TCSANOW)
        except _termios.error:
            pass
        try:
            os.unlink(self._pty_path)
        except FileNotFoundError:
            pass
        os.symlink(slave_name, self._pty_path)
        self._slave_fd = slave_fd
        self._master_fd = master_fd
        # Non-blocking master: a slow/absent slave reader must not deadlock
        # the reactor.
        import fcntl as _fcntl

        flags = _fcntl.fcntl(master_fd, _fcntl.F_GETFL)
        _fcntl.fcntl(master_fd, _fcntl.F_SETFL, flags | os.O_NONBLOCK)
        self._thread = threading.Thread(
            target=self._reactor_loop, name="beacon-stub-rx", daemon=True
        )
        self._thread.start()
        if self._step_sock_path and self._step_thread is None:
            self._step_thread = threading.Thread(
                target=self._step_poll_loop,
                name="beacon-stub-steps",
                daemon=True,
            )
            self._step_thread.start()

    def start_sample_stream(
        self, z_target_mm: float, rate_hz: float = SAMPLE_RATE_HZ
    ) -> None:
        self._z_target = z_target_mm
        if self._thread is None or not self._thread.is_alive():
            self.start()

    def set_z(self, z_mm: float) -> None:
        self._z_target = z_mm
        self._z_current = z_mm

    def stop(self) -> None:
        self._stop.set()
        # Closing the master fd unblocks the select() in the reactor.
        if self._master_fd is not None:
            try:
                os.close(self._master_fd)
            except OSError:
                pass
            self._master_fd = None
        if self._slave_fd is not None:
            try:
                os.close(self._slave_fd)
            except OSError:
                pass
            self._slave_fd = None
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._sample_thread is not None:
            self._sample_thread.join(timeout=2.0)
            self._sample_thread = None
        if self._accel_thread is not None:
            self._accel_thread.join(timeout=2.0)
            self._accel_thread = None
        try:
            os.unlink(self._pty_path)
        except FileNotFoundError:
            pass

    def _step_poll_loop(self) -> None:
        import socket as _socket

        sock = None
        buf = b""
        while not self._stop.is_set():
            if sock is None:
                try:
                    sock = _socket.socket(_socket.AF_UNIX, _socket.SOCK_STREAM)
                    sock.settimeout(1.0)
                    sock.connect(self._step_sock_path)
                except OSError:
                    sock = None
                    time.sleep(0.25)
                    continue
            try:
                if self._z_line_locked:
                    probe_lines = [self._z_step_line]
                else:
                    probe_lines = [15, 18, 7]
                readings = {}
                reading_vts = {}
                for ln in probe_lines:
                    sock.sendall(b"get_steps line=%d\n" % ln)
                    while b"\n" not in buf:
                        chunk = sock.recv(64)
                        if not chunk:
                            raise OSError("closed")
                        buf += chunk
                    resp, _, buf = buf.partition(b"\n")
                    if resp.startswith(b"steps="):
                        fields = dict(
                            kv.split(b"=", 1)
                            for kv in resp.split()
                            if b"=" in kv
                        )
                        readings[ln] = int(fields[b"steps"])
                        if b"vt" in fields:
                            reading_vts[ln] = int(fields[b"vt"]) / 1e9
                if not self._z_line_locked:
                    for ln, val in readings.items():
                        if val != self._line_baselines.get(ln, val):
                            self._z_step_line = ln
                            self._z_line_locked = True
                            logging.info(
                                "beacon-stub: z step line locked to gpio%d",
                                ln,
                            )
                            break
                        self._line_baselines[ln] = val
                line_vt = reading_vts.get(self._z_step_line)
                line = (
                    b"steps=%d" % readings[self._z_step_line]
                    if self._z_step_line in readings
                    else b""
                )
            except OSError:
                try:
                    sock.close()
                except OSError:
                    pass
                sock = None
                continue
            if line.startswith(b"steps="):
                sampled_at = (
                    line_vt if line_vt is not None else self._monotonic()
                )
                steps = int(line[6:])
                self._steps_now = steps
                if self._z_line_locked and not self._step_tracking:
                    self._step_tracking = True
                    self._z_anchor_steps = steps
                if self._step_tracking:
                    z = (
                        self._z_anchor_mm
                        + self._z_step_sign
                        * (steps - self._z_anchor_steps)
                        / self._z_steps_per_mm
                    )
                    self._z_current = z
                    if self._contact_homing_active and z <= 0.0:
                        self._fire_contact_trigger(
                            self._bed_crossing_time(sampled_at, z)
                        )
                    self._prev2_poll_time = self._prev_poll_time
                    self._prev2_poll_z = self._prev_poll_z
                    self._prev_poll_time = sampled_at
                    self._prev_poll_z = z
            time.sleep(0.001)

    def _bed_crossing_time(self, sampled_at: float, z: float) -> float:
        """The poll that sees z <= 0 runs up to one poll period after the
        true crossing; the descent is constant-speed until the trigger
        fires, so interpolating between the last two polls recovers the
        crossing time exactly. Detecting late is fine, but reporting the
        detection time as the trigger clock biases every reconstructed
        contact position downward by poll latency."""
        prev_t, prev_z = self._prev_poll_time, self._prev_poll_z
        if prev_t is None or prev_z <= 0.0 or prev_z <= z:
            return sampled_at
        return prev_t + (sampled_at - prev_t) * prev_z / (prev_z - z)

    def _monotonic(self) -> float:
        if self._vtime_map is not None:
            return struct.unpack_from("<Q", self._vtime_map, 0)[0] / 1e9
        return time.monotonic()

    def _start_virtual_timer(self, delay_s: float, fn) -> threading.Event:
        """threading.Timer on the virtual clock: these delays emulate
        physical latencies, so they must elapse with the simulated world,
        not the wall clock. Returns the cancel event."""
        cancelled = threading.Event()
        deadline = self._monotonic() + delay_s

        def poll() -> None:
            while not cancelled.is_set() and not self._stop.is_set():
                if self._monotonic() >= deadline:
                    if not cancelled.is_set():
                        fn()
                    return
                time.sleep(0.002)

        threading.Thread(target=poll, daemon=True).start()
        return cancelled

    def _now_clock(self) -> int:
        return self._clock_at(self._monotonic())

    def _clock_at(self, monotonic_time: float) -> int:
        elapsed = monotonic_time - self._clock_origin
        return int(elapsed * self.CLOCK_FREQ) & 0xFFFFFFFF

    def _now_clock_high(self) -> int:
        elapsed = self._monotonic() - self._clock_origin
        return (int(elapsed * self.CLOCK_FREQ) >> 32) & 0xFFFFFFFF

    def _send_msg(self, msgformat: str, **kwargs) -> None:
        # Framing is open-coded rather than via msgproto.encode_msgblock:
        # that helper appends the CRC list as one element instead of
        # extending it.
        if self._master_fd is None:
            return
        try:
            cmd = self._parser.lookup_command(msgformat).encode_by_name(
                **kwargs
            )
        except msgproto.error:
            logging.exception("beacon-stub: unknown msgformat %r", msgformat)
            return
        with self._send_lock:
            seq_byte = (
                self._host_recv_seq & msgproto.MESSAGE_SEQ_MASK
            ) | msgproto.MESSAGE_DEST
            payload = [msgproto.MESSAGE_MIN + len(cmd), seq_byte] + list(cmd)
            crc = msgproto.crc16_ccitt(payload)
            payload.extend(crc)
            payload.append(msgproto.MESSAGE_SYNC)
            try:
                os.write(self._master_fd, bytes(payload))
            except (BlockingIOError, OSError):
                # Drop on a full kernel buffer; serialqueue NAKs and we
                # retransmit on the next dispatch tick.
                return
            self.tx_frame_count += 1
            self._log("tx", bytes(payload), msgformat, kwargs)

    def _send_ack(self) -> None:
        if self._master_fd is None:
            return
        with self._send_lock:
            seq_byte = (
                self._host_recv_seq & msgproto.MESSAGE_SEQ_MASK
            ) | msgproto.MESSAGE_DEST
            payload = [msgproto.MESSAGE_MIN, seq_byte]
            crc = msgproto.crc16_ccitt(payload)
            payload.extend(crc)
            payload.append(msgproto.MESSAGE_SYNC)
            try:
                os.write(self._master_fd, bytes(payload))
            except (BlockingIOError, OSError):
                return
            self._log("tx-ack", bytes(payload))

    def _reactor_loop(self) -> None:
        master_fd = self._master_fd
        while not self._stop.is_set():
            if master_fd is None:
                break
            try:
                r, _, _ = select.select([master_fd], [], [], 0.05)
            except (ValueError, OSError):
                break
            if not r:
                continue
            try:
                chunk = os.read(master_fd, 4096)
            except OSError:
                break
            if not chunk:
                continue
            self.rx_byte_count += len(chunk)
            self._log("rx-raw", bytes(chunk))
            self._inbuf.extend(chunk)
            self._drain_inbuf()

    def _drain_inbuf(self) -> None:
        while True:
            msglen = self._parser.check_packet(self._inbuf)
            if msglen == 0:
                return
            if msglen < 0:
                logging.info(
                    "beacon-stub: RESYNC bad framing, buf head=%s",
                    bytes(self._inbuf[:24]).hex(),
                )
                idx = self._inbuf.find(msgproto.MESSAGE_SYNC)
                if idx < 0:
                    self._inbuf.clear()
                    return
                del self._inbuf[: idx + 1]
                continue
            frame = list(self._inbuf[:msglen])
            del self._inbuf[:msglen]
            frame_seq = frame[1] & msgproto.MESSAGE_SEQ_MASK
            if frame_seq != self._host_recv_seq & msgproto.MESSAGE_SEQ_MASK:
                logging.info(
                    "beacon-stub: DROP frame seq=%d expected=%d len=%d",
                    frame_seq,
                    self._host_recv_seq & msgproto.MESSAGE_SEQ_MASK,
                    len(frame),
                )
                self._send_ack()
                continue
            self._host_recv_seq = (self._host_recv_seq + 1) & 0xFFFF
            try:
                params = self._parser.parse(frame)
            except msgproto.error:
                logging.exception("beacon-stub: parse failed")
                self._send_ack()
                continue
            tx_before = self.tx_frame_count
            self._dispatch(params, frame)
            if self.tx_frame_count == tx_before:
                self._send_ack()

    def _dispatch(self, params: dict, frame: list) -> None:
        name = params.get("#name")
        self._log_inbound(name, params)
        handler = self._handlers.get(name)
        if handler is None:
            return
        try:
            handler(params)
        except Exception:
            logging.exception("beacon-stub: handler %r raised", name)

    def _build_handlers(self) -> dict:
        return {
            "identify": self._handle_identify,
            "get_uptime": self._handle_get_uptime,
            "get_clock": self._handle_get_clock,
            "get_config": self._handle_get_config,
            "allocate_oids": self._handle_noop,
            "finalize_config": self._handle_finalize_config,
            "emergency_stop": self._handle_noop,
            "clear_shutdown": self._handle_noop,
            "debug_nop": self._handle_noop,
            "debug_ping": self._handle_debug_ping,
            "debug_read": self._handle_debug_read,
            "debug_write": self._handle_noop,
            "beacon_stream": self._handle_beacon_stream,
            "beacon_set_threshold": self._handle_beacon_set_threshold,
            "beacon_home": self._handle_beacon_home,
            "beacon_stop_home": self._handle_beacon_stop_home,
            "beacon_nvm_read": self._handle_beacon_nvm_read,
            "beacon_contact_home": self._handle_beacon_contact_home,
            "beacon_contact_query": self._handle_beacon_contact_query,
            "beacon_contact_stop_home": self._handle_beacon_contact_stop_home,
            "beacon_contact_set_latency_min": self._handle_noop,
            "beacon_contact_set_sensitivity": self._handle_noop,
            "config_trsync": self._handle_config_trsync,
            "trsync_start": self._handle_trsync_start,
            "trsync_set_timeout": self._handle_noop,
            "trsync_trigger": self._handle_trsync_trigger,
            "stepper_stop_on_trigger": self._handle_noop,
            "beacon_accel_stream": self._handle_beacon_accel_stream,
        }

    def _handle_noop(self, params: dict) -> None:
        return

    def _handle_identify(self, params: dict) -> None:
        offset = params["offset"]
        count = params["count"]
        if offset >= len(self.IDENTIFY_BLOB):
            data = b""
        else:
            data = self.IDENTIFY_BLOB[offset : offset + count]
        self._send_msg(
            "identify_response offset=%u data=%.*s",
            offset=offset,
            data=list(data),
        )

    def _handle_get_uptime(self, params: dict) -> None:
        self._send_msg(
            "uptime high=%u clock=%u",
            high=self._now_clock_high(),
            clock=self._now_clock(),
        )

    def _handle_get_clock(self, params: dict) -> None:
        self._send_msg("clock clock=%u", clock=self._now_clock())

    def _handle_get_config(self, params: dict) -> None:
        self._send_msg(
            "config is_config=%c crc=%u is_shutdown=%c move_count=%hu",
            is_config=1 if self._is_configured else 0,
            crc=self._committed_crc,
            is_shutdown=0,
            move_count=0,
        )

    def _handle_finalize_config(self, params: dict) -> None:
        self._committed_crc = params["crc"]
        self._is_configured = True

    def _handle_debug_ping(self, params: dict) -> None:
        self._send_msg("pong data=%*s", data=list(params["data"]))

    def _handle_debug_read(self, params: dict) -> None:
        self._send_msg("debug_result val=%u", val=0)

    def _handle_beacon_stream(self, params: dict) -> None:
        en = params["en"]
        self._stream_en = bool(en)
        logging.info("beacon-stub: stream en=%d", en)
        if self._stream_en:
            self._start_sample_thread()
        else:
            self._stop_sample_thread()

    def _handle_beacon_set_threshold(self, params: dict) -> None:
        self._threshold_trigger = params["trigger"]
        self._threshold_untrigger = params["untrigger"]

    def _handle_beacon_home(self, params: dict) -> None:
        self._home_trsync_oid = params["trsync_oid"]
        self._home_trigger_reason = params["trigger_reason"]
        self._home_trigger_invert = params["trigger_invert"]
        if not self._home_active:
            if not self._step_tracking:
                self._z_current = APPROACH_FROM_ABOVE_Z_MM
            self._homing_start_z = self._z_current
            self._homing_start_time = self._monotonic()
            self._home_active = True
            self._start_homing_monitor()
        logging.info(
            "beacon-stub: beacon_home trsync_oid=%d z=%.2f threshold=%d",
            self._home_trsync_oid,
            self._z_current,
            self._threshold_trigger,
        )

    def _handle_beacon_stop_home(self, params: dict) -> None:
        self._home_active = False

    def _start_homing_monitor(self) -> None:
        # Separate from _sample_loop because the trigger must be checked
        # during homing, when sample streaming is off.
        t = threading.Thread(
            target=self._homing_monitor_loop,
            name="beacon-stub-homing",
            daemon=True,
        )
        t.start()

    def _homing_monitor_loop(self) -> None:
        CHECK_HZ = 200.0
        period = 1.0 / CHECK_HZ
        iter_count = 0
        while not self._stop.is_set() and self._home_active:
            time.sleep(period)
            if not self._step_tracking:
                elapsed = self._monotonic() - self._homing_start_time
                self._z_current = max(
                    0.0,
                    self._homing_start_z
                    - elapsed * self._homing_approach_speed,
                )
            freq = self._z_to_frequency(self._z_current)
            count = self._freq_to_count(freq)
            iter_count += 1
            if iter_count % 40 == 0:
                logging.info(
                    "beacon-stub: homing monitor z=%.2f freq=%d "
                    "count=%d threshold=%d",
                    self._z_current,
                    freq,
                    count,
                    self._threshold_trigger,
                )
            if self._threshold_trigger > 0:
                if self._home_trigger_invert:
                    triggered = count <= self._threshold_untrigger
                else:
                    triggered = count >= self._threshold_trigger
                if triggered:
                    logging.info(
                        "beacon-stub: TRIGGER z=%.2f count=%d threshold=%d",
                        self._z_current,
                        count,
                        self._threshold_trigger,
                    )
                    self._fire_homing_trigger()
                    return

    def _handle_beacon_nvm_read(self, params: dict) -> None:
        length = params["len"]
        offset = params["offset"]
        end = offset + length
        if end > len(NVM_IMAGE):
            # 0xFF is beacon's "uncalibrated" sentinel for unpacked NVM.
            data = NVM_IMAGE[offset:] + b"\xff" * (end - len(NVM_IMAGE))
        else:
            data = NVM_IMAGE[offset:end]
        self._send_msg(
            "beacon_nvm_data bytes=%*s offset=%hu",
            bytes=list(data),
            offset=offset,
        )

    def _handle_beacon_contact_home(self, params: dict) -> None:
        self._contact_homing_active = True
        self._contact_armed_clock = self._now_clock()
        self._contact_trsync_oid = params["trsync_oid"]
        self._contact_trigger_reason = params["trigger_reason"]

        if self._homing_trigger_timer is not None:
            self._homing_trigger_timer.set()
        step_tracking_will_fire_at_bed_contact = self._step_thread is not None
        if not step_tracking_will_fire_at_bed_contact:
            self._homing_trigger_timer = self._start_virtual_timer(
                self._homing_trigger_delay, self._fire_contact_trigger
            )

    def _fire_contact_trigger(
        self, trigger_time: Optional[float] = None
    ) -> None:
        if not self._contact_homing_active:
            return
        self._contact_homing_active = False
        if self.contact_latch_commit_delay <= 0.0:
            self._contact_triggered = True
        else:
            self._contact_latch_timer = self._start_virtual_timer(
                self.contact_latch_commit_delay, self._commit_contact_latch
            )
        self._contact_trigger_clock = (
            self._now_clock()
            if trigger_time is None
            else self._clock_at(trigger_time)
        )
        contact_line = (
            f"CONTACT steps={self._steps_now} z={self._z_current:.6f}"
            f" trigger_time={trigger_time!r}"
            f" clock={self._contact_trigger_clock}\n"
        )
        logging.info("beacon-stub: %s", contact_line.strip())
        if self._log_path:
            with open(self._log_path, "a") as f:
                f.write(contact_line)
        self._contact_trigger_sample = self._sample_index
        self._contact_trigger_freq = self._z_to_frequency(0.0)
        self._trsync_can_trigger[self._contact_trsync_oid] = False
        self._trsync_trigger_reason[self._contact_trsync_oid] = (
            self._contact_trigger_reason
        )
        self._send_msg(
            "beacon_contact armed_clock=%u trigger_clock=%u"
            " detect_clock=%u latency=%c error=%c",
            armed_clock=self._contact_armed_clock,
            trigger_clock=self._contact_trigger_clock,
            detect_clock=self._contact_trigger_clock,
            latency=0,
            error=0,
        )
        self._send_msg(
            "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u",
            oid=self._contact_trsync_oid,
            can_trigger=0,
            trigger_reason=self._contact_trigger_reason,
            clock=self._contact_trigger_clock,
        )

    def _commit_contact_latch(self) -> None:
        self._contact_latch_timer = None
        self._contact_triggered = True

    def _handle_beacon_contact_stop_home(self, params: dict) -> None:
        self._contact_homing_active = False
        if self._homing_trigger_timer is not None:
            self._homing_trigger_timer.set()
            self._homing_trigger_timer = None
        if self._contact_latch_timer is not None:
            self._contact_latch_timer.set()
            self._contact_latch_timer = None

    def _handle_beacon_contact_query(self, params: dict) -> None:
        self._send_msg(
            "beacon_contact_state triggered=%c detect_clock=%u",
            triggered=1 if self._contact_triggered else 0,
            detect_clock=self._contact_trigger_clock,
        )

    def _handle_config_trsync(self, params: dict) -> None:
        self._trsync_oids.add(params["oid"])
        self._trsync_can_trigger = {}

    def _handle_trsync_start(self, params: dict) -> None:
        self._trsync_oids.add(params["oid"])
        self._trsync_can_trigger[params["oid"]] = True

    def _handle_trsync_trigger(self, params: dict) -> None:
        oid = params["oid"]
        reason = params["reason"]
        if self._trsync_can_trigger.get(oid, False):
            self._trsync_can_trigger[oid] = False
            self._trsync_trigger_reason[oid] = reason
        else:
            reason = self._trsync_trigger_reason.get(oid, reason)
        self._send_msg(
            "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u",
            oid=oid,
            can_trigger=0,
            trigger_reason=reason,
            clock=self._now_clock(),
        )

    def _handle_beacon_accel_stream(self, params: dict) -> None:
        en = bool(params["en"])
        self._accel_scale_id = params["scale"]
        was_en = self._accel_stream_en
        self._accel_stream_en = en
        if en and not was_en:
            self._start_accel_thread()

    def _start_accel_thread(self) -> None:
        if self._accel_thread is not None and self._accel_thread.is_alive():
            return
        self._accel_thread = threading.Thread(
            target=self._accel_loop, name="beacon-stub-accel", daemon=True
        )
        self._accel_thread.start()

    def _accel_loop(self) -> None:
        SAMPLES_PER_BATCH = 6
        BATCH_PERIOD_S = 0.001
        z_raw = 16384
        sample_bytes = bytes(
            [
                0x00,
                0x00,
                0x00,
                0x00,
                z_raw & 0xFF,
                (z_raw >> 8) & 0xFF,
            ]
        )
        batch_payload = sample_bytes * SAMPLES_PER_BATCH
        next_tick = time.monotonic()
        last_clock = self._now_clock()
        while not self._stop.is_set() and self._accel_stream_en:
            now = time.monotonic()
            sleep_for = next_tick - now
            if sleep_for > 0:
                time.sleep(min(sleep_for, BATCH_PERIOD_S))
                continue
            next_tick += BATCH_PERIOD_S
            cur_clock = self._now_clock()
            delta = (cur_clock - last_clock) & 0xFFFFFFFF
            self._send_msg(
                "beacon_accel_data start_clock=%u delta_clock=%u data=%*s",
                start_clock=last_clock,
                delta_clock=delta,
                data=list(batch_payload),
            )
            last_clock = cur_clock

    def _start_sample_thread(self) -> None:
        if self._sample_thread is not None and self._sample_thread.is_alive():
            return
        self._sample_thread = threading.Thread(
            target=self._sample_loop, name="beacon-stub-tx", daemon=True
        )
        self._sample_thread.start()

    def _stop_sample_thread(self) -> None:
        # No-op: the loop exits when _stream_en flips false, and stop() joins.
        return

    def _project_z(self, at_vt: float) -> float:
        last_t, last_z = self._prev_poll_time, self._prev_poll_z
        prev_t, prev_z = self._prev2_poll_time, self._prev2_poll_z
        if last_t is None or prev_t is None or last_t <= prev_t:
            return self._z_current
        if at_vt <= last_t:
            return last_z
        horizon = min(at_vt - last_t, last_t - prev_t)
        slope = (last_z - prev_z) / (last_t - prev_t)
        return max(0.0, last_z + slope * horizon)

    def _query_steps_now(self, sock) -> tuple:
        sock.sendall(b"get_steps line=%d\n" % self._z_step_line)
        buf = b""
        while b"\n" not in buf:
            chunk = sock.recv(64)
            if not chunk:
                raise OSError("closed")
            buf += chunk
        resp, _, _ = buf.partition(b"\n")
        if not resp.startswith(b"steps="):
            raise OSError("bad get_steps response")
        fields = dict(kv.split(b"=", 1) for kv in resp.split() if b"=" in kv)
        return int(fields[b"steps"]), int(fields[b"vt"]) / 1e9

    def _batch_sock(self):
        import socket as _socket

        sock = _socket.socket(_socket.AF_UNIX, _socket.SOCK_STREAM)
        sock.settimeout(1.0)
        sock.connect(self._step_sock_path)
        return sock

    def _due_batch_vt(self, now_vt: float) -> float:
        """A stall that swallows whole batch periods cannot be replayed:
        the samples those periods would have carried were never taken.
        Replaying the backlog emits several batches at one clock, each
        stamped with the current Z, which reads downstream as scheduled
        history. The schedule resynchronizes to now exactly once
        instead."""
        if now_vt - self._next_batch_vt >= self.BATCH_PERIOD_S:
            return now_vt
        return self._next_batch_vt

    def _commit_batch_vt(self, batch_vt: float) -> None:
        last = self._last_batch_vt
        min_advance = self.BATCH_PERIOD_S / 2
        if last is not None and batch_vt - last < min_advance:
            raise RuntimeError(
                "beacon-stub: batch clocks collided: "
                f"{batch_vt!r} follows {last!r}, less than half of the "
                f"{self.BATCH_PERIOD_S}s batch period apart"
            )
        self._last_batch_vt = batch_vt
        self._next_batch_vt = batch_vt + self.BATCH_PERIOD_S

    def _sample_loop(self) -> None:
        STATUS_HZ = 10.0
        status_period = 1.0 / STATUS_HZ

        self._next_batch_vt = self._monotonic()
        self._last_batch_vt = None
        next_status = time.monotonic()
        loop_iter_count = 0
        batch_sock = None

        while not self._stop.is_set() and self._stream_en:
            now_vt = self._monotonic()
            now = time.monotonic()
            if now_vt < self._next_batch_vt and now < next_status:
                time.sleep(0.001)
                continue

            if now_vt >= self._next_batch_vt:
                batch_vt = self._due_batch_vt(now_vt)

                z_at_batch = None
                if self._step_tracking:
                    try:
                        if batch_sock is None:
                            batch_sock = self._batch_sock()
                        steps, batch_vt = self._query_steps_now(batch_sock)
                        z_at_batch = (
                            self._z_anchor_mm
                            + self._z_step_sign
                            * (steps - self._z_anchor_steps)
                            / self._z_steps_per_mm
                        )
                    except OSError:
                        if batch_sock is not None:
                            try:
                                batch_sock.close()
                            except OSError:
                                pass
                            batch_sock = None
                self._commit_batch_vt(batch_vt)
                if z_at_batch is None:
                    if self._home_active and not self._step_tracking:
                        elapsed = batch_vt - self._homing_start_time
                        self._z_current = max(
                            0.0,
                            self._homing_start_z
                            - elapsed * self._homing_approach_speed,
                        )
                        z_at_batch = self._z_current
                    else:
                        z_at_batch = self._project_z(batch_vt)
                start_clock = self._clock_at(batch_vt)

                freq = self._z_to_frequency(z_at_batch)
                data_value = self._freq_to_count(freq)

                buf = bytearray()
                decoder_baseline = 0
                last_data_value = decoder_baseline
                for i in range(self.SAMPLES_PER_BATCH):
                    delta = data_value - last_data_value
                    fits_two_byte_twos_complement = -16384 <= delta <= 16383
                    if fits_two_byte_twos_complement:
                        encoded = delta & 0x7FFF
                        buf.append((encoded >> 8) & 0x7F)
                        buf.append(encoded & 0xFF)
                    else:
                        four_byte_absolute_flag = 0x80
                        buf.append(
                            four_byte_absolute_flag
                            | ((data_value >> 24) & 0x7F)
                        )
                        buf.append((data_value >> 16) & 0xFF)
                        buf.append((data_value >> 8) & 0xFF)
                        buf.append(data_value & 0xFF)
                    last_data_value = data_value

                delta_clock = (
                    int(
                        self.CLOCK_FREQ
                        / (self.BATCH_HZ * self.SAMPLES_PER_BATCH)
                    )
                    * self.SAMPLES_PER_BATCH
                )
                self._send_msg(
                    "beacon_data data=%*s samples=%c start_clock=%u delta_clock=%u",
                    data=list(buf),
                    samples=self.SAMPLES_PER_BATCH,
                    start_clock=start_clock,
                    delta_clock=delta_clock,
                )
                self.tx_sample_count += self.SAMPLES_PER_BATCH
                loop_iter_count += 1

                # Thresholds arrive from klippy already in counts, not Hz.
                if self._home_active and self._threshold_trigger > 0:
                    count = data_value
                    if self._home_trigger_invert:
                        triggered = count <= self._threshold_untrigger
                    else:
                        triggered = count >= self._threshold_trigger
                    if loop_iter_count % 40 == 0:
                        logging.info(
                            "beacon-stub: trigger check z=%.2f "
                            "count=%d threshold=%d triggered=%s",
                            self._z_current,
                            count,
                            self._threshold_trigger,
                            triggered,
                        )
                    if triggered:
                        logging.info(
                            "beacon-stub: TRIGGER FIRED z=%.2f "
                            "count=%d threshold=%d",
                            self._z_current,
                            count,
                            self._threshold_trigger,
                        )
                        self._fire_homing_trigger()

            if now >= next_status:
                next_status += status_period
                self._send_msg(
                    "beacon_status mcu_temp=%u supply_voltage=%u coil_temp=%u status=%u",
                    mcu_temp=2048,
                    supply_voltage=3300,
                    coil_temp=143_640,  # ~25C at BEACON_ADC_SMOOTH_COUNT=200
                    status=0,
                )
        if batch_sock is not None:
            try:
                batch_sock.close()
            except OSError:
                pass

    def _z_to_frequency(self, z_mm: float) -> int:
        if z_mm < 0:
            z_mm = 0
        return int(
            self._freq_base + self._freq_coeff / (z_mm + self._freq_offset)
        )

    def _freq_to_count(self, freq_hz: int) -> int:
        return int(freq_hz * (2**28) / self.CLOCK_FREQ)

    def _fire_homing_trigger(self) -> None:
        if not self._home_active:
            return
        self._home_active = False
        oid = self._home_trsync_oid
        reason = self._home_trigger_reason
        self._trsync_can_trigger[oid] = False
        self._trsync_trigger_reason[oid] = reason
        oid = self._home_trsync_oid
        reason = self._home_trigger_reason
        self._send_msg(
            "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u",
            oid=oid,
            can_trigger=0,
            trigger_reason=reason,
            clock=self._now_clock(),
        )

    def _log(
        self,
        direction: str,
        data: bytes,
        msgformat: Optional[str] = None,
        kwargs: Optional[dict] = None,
    ) -> None:
        if self._log_path is None:
            return
        try:
            log_dir = os.path.dirname(self._log_path)
            if log_dir:
                os.makedirs(log_dir, exist_ok=True)
            with open(self._log_path, "ab") as f:
                ts = f"{time.monotonic() - self._t0:.6f}"
                trailer = ""
                if msgformat is not None:
                    name = msgformat.split()[0]
                    args_repr = ""
                    if kwargs:
                        parts = []
                        for k, v in kwargs.items():
                            if (
                                isinstance(v, list)
                                and v
                                and isinstance(v[0], int)
                            ):
                                parts.append(f"{k}=<{len(v)} bytes>")
                            else:
                                parts.append(f"{k}={v}")
                        args_repr = " " + " ".join(parts)
                    trailer = f"  {name}{args_repr}"
                line = f"[{ts}][{direction}] {data.hex()}{trailer}\n".encode()
                f.write(line)
        except (OSError, ValueError):
            pass

    def _log_inbound(self, name: Optional[str], params: dict) -> None:
        if self._log_path is None or name is None:
            return
        try:
            log_dir = os.path.dirname(self._log_path)
            if log_dir:
                os.makedirs(log_dir, exist_ok=True)
            with open(self._log_path, "ab") as f:
                ts = f"{time.monotonic() - self._t0:.6f}"
                light = {
                    k: (
                        f"<{len(v)} bytes>"
                        if isinstance(v, (bytes, bytearray))
                        else v
                    )
                    for k, v in params.items()
                    if not k.startswith("#")
                }
                line = f"[{ts}][rx-msg] {name} {light}\n".encode()
                f.write(line)
        except (OSError, ValueError):
            pass


BeaconSerialStub = BeaconMcuStub
