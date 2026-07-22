# Micro-controller clock synchronization
#
# Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import traceback

from .motion_engine import native_class

RTT_AGE = 0.000010 / (60.0 * 60.0)
DECAY = 1.0 / 30.0
SYNC_STABLE_FREQ_PPM = 5e-6
SYNC_STABLE_SAMPLES = 3


class ClockSync:
    def __init__(self, reactor):
        self.reactor = reactor
        self.serial = None
        self.get_clock_timer = reactor.register_timer(self._get_clock_event)
        self.queries_pending = 0
        self.mcu_freq = 1.0
        self.clock_est = (0.0, 0.0, 0.0)
        self._est = native_class("ClockSyncEstimator")(
            DECAY, RTT_AGE, SYNC_STABLE_FREQ_PPM, SYNC_STABLE_SAMPLES
        )
        self._clock_est_callback = None

    @property
    def last_clock(self):
        return self._est.last_clock

    @last_clock.setter
    def last_clock(self, v):
        self._est.last_clock = int(v)

    @property
    def time_avg(self):
        return self._est.time_avg

    @time_avg.setter
    def time_avg(self, v):
        self._est.time_avg = v

    @property
    def clock_avg(self):
        return self._est.clock_avg

    @clock_avg.setter
    def clock_avg(self, v):
        self._est.clock_avg = v

    @property
    def time_variance(self):
        return self._est.time_variance

    @time_variance.setter
    def time_variance(self, v):
        self._est.time_variance = v

    @property
    def clock_covariance(self):
        return self._est.clock_covariance

    @clock_covariance.setter
    def clock_covariance(self, v):
        self._est.clock_covariance = v

    @property
    def prediction_variance(self):
        return self._est.prediction_variance

    @prediction_variance.setter
    def prediction_variance(self, v):
        self._est.prediction_variance = v

    @property
    def last_prediction_time(self):
        return self._est.last_prediction_time

    @last_prediction_time.setter
    def last_prediction_time(self, v):
        self._est.last_prediction_time = v

    @property
    def min_half_rtt(self):
        return self._est.min_half_rtt

    @min_half_rtt.setter
    def min_half_rtt(self, v):
        self._est.min_half_rtt = v

    @property
    def min_rtt_time(self):
        return self._est.min_rtt_time

    @min_rtt_time.setter
    def min_rtt_time(self, v):
        self._est.min_rtt_time = v

    @property
    def _sync_stable_count(self):
        return self._est.sync_stable_count

    @_sync_stable_count.setter
    def _sync_stable_count(self, v):
        self._est.sync_stable_count = int(v)

    @property
    def _synced(self):
        return self._est.synced

    @_synced.setter
    def _synced(self, v):
        self._est.synced = bool(v)

    def set_clock_est_callback(self, cb):
        # cb(freq, offset, last_clock); invoked from the serial-reader thread on
        # every published regression update.
        self._clock_est_callback = cb
        if cb is not None and self.last_clock:
            try:
                cb(
                    self.clock_est[2],
                    self.time_avg + self.min_half_rtt,
                    int(self.clock_avg),
                )
            except Exception:
                logging.exception("clocksync: initial set_clock_est callback")

    def disconnect(self):
        self.reactor.update_timer(self.get_clock_timer, self.reactor.NEVER)

    def connect(self, serial):
        self.serial = serial
        self.mcu_freq = serial.msgparser.get_constant_float("CLOCK_FREQ")
        # Load initial clock and frequency
        params = serial.send_with_response("get_uptime", "uptime")
        self.last_clock = (params["high"] << 32) | params["clock"]
        self.clock_avg = self.last_clock
        self.time_avg = params["#sent_time"]
        self.clock_est = (self.time_avg, self.clock_avg, self.mcu_freq)
        self.prediction_variance = (0.001 * self.mcu_freq) ** 2
        self._sync_stable_count = 0
        self._synced = False
        # Enable periodic get_clock timer
        for i in range(8):
            self.reactor.pause(self.reactor.monotonic() + 0.050)
            self.last_prediction_time = -9999.0
            params = serial.send_with_response("get_clock", "clock")
            self._handle_clock(params)
        serial.register_response(self._handle_clock, "clock")
        self._sync_stable_count = 0
        self.reactor.update_timer(self.get_clock_timer, self.reactor.NOW)

    def connect_file(self, serial, pace=False):
        self.serial = serial
        self.mcu_freq = serial.msgparser.get_constant_float("CLOCK_FREQ")
        self.clock_est = (0.0, 0.0, self.mcu_freq)
        self._synced = True
        freq = 1000000000000.0
        if pace:
            freq = self.mcu_freq
        serial.set_clock_est(freq, self.reactor.monotonic(), 0, 0)

    # MCU clock querying (_handle_clock is invoked from background thread)
    def _get_clock_event(self, eventtime):
        self.serial.engine_get_clock_async()
        self.queries_pending += 1
        # Use an unusual time for the next event so clock messages
        # don't resonate with other periodic events.
        return eventtime + 0.9839

    def _handle_clock(self, params):
        self.queries_pending = 0
        est = self._est.handle_clock(
            params["clock"] & 0xFFFFFFFF,
            params["#sent_time"],
            params["#receive_time"],
            self.mcu_freq,
            self.clock_est[2],
        )
        if est is None:
            return
        new_freq, offset, clock_avg = est
        self.clock_est = (offset, clock_avg, new_freq)
        cb = self._clock_est_callback
        if cb is not None:
            try:
                cb(new_freq, offset, int(clock_avg))
            except Exception:
                logging.exception("clocksync: set_clock_est callback")

    # clock frequency conversions
    def print_time_to_clock(self, print_time):
        return int(print_time * self.mcu_freq)

    def clock_to_print_time(self, clock):
        return clock / self.mcu_freq

    # system time conversions
    def get_clock(self, eventtime):
        sample_time, clock, freq = self.clock_est
        return int(clock + (eventtime - sample_time) * freq)

    def estimate_clock_systime(self, reqclock):
        sample_time, clock, freq = self.clock_est
        return float(reqclock - clock) / freq + sample_time

    def estimated_print_time(self, eventtime):
        return self.clock_to_print_time(self.get_clock(eventtime))

    # misc commands
    def clock32_to_clock64(self, clock32):
        last_clock = self.last_clock
        clock_diff = (clock32 - last_clock) & 0xFFFFFFFF
        clock_diff -= (clock_diff & 0x80000000) << 1
        return last_clock + clock_diff

    def is_active(self):
        return self.queries_pending <= 4

    def is_synced(self):
        return self._synced

    def dump_debug(self):
        sample_time, clock, freq = self.clock_est
        return (
            "clocksync state: mcu_freq=%d last_clock=%d"
            " clock_est=(%.3f %d %.3f) min_half_rtt=%.6f min_rtt_time=%.3f"
            " time_avg=%.3f(%.3f) clock_avg=%.3f(%.3f)"
            " pred_variance=%.3f"
            % (
                self.mcu_freq,
                self.last_clock,
                sample_time,
                clock,
                freq,
                self.min_half_rtt,
                self.min_rtt_time,
                self.time_avg,
                self.time_variance,
                self.clock_avg,
                self.clock_covariance,
                self.prediction_variance,
            )
        )

    def stats(self, eventtime):
        sample_time, clock, freq = self.clock_est
        return "freq=%d" % (freq,)

    def calibrate_clock(self, print_time, eventtime):
        return (0.0, self.mcu_freq)


# Clock syncing code for secondary MCUs (whose clocks are sync'ed to a
# primary MCU)
class SecondarySync(ClockSync):
    def __init__(self, reactor, main_sync):
        ClockSync.__init__(self, reactor)
        self.main_sync = main_sync
        self.clock_adj = (0.0, 1.0)
        self.last_sync_time = 0.0

    def connect(self, serial):
        ClockSync.connect(self, serial)
        self.clock_adj = (0.0, self.mcu_freq)
        curtime = self.reactor.monotonic()
        main_print_time = self.main_sync.estimated_print_time(curtime)
        local_print_time = self.estimated_print_time(curtime)
        self.clock_adj = (main_print_time - local_print_time, self.mcu_freq)
        self.calibrate_clock(0.0, curtime)

    def connect_file(self, serial, pace=False):
        ClockSync.connect_file(self, serial, pace)
        self.clock_adj = (0.0, self.mcu_freq)

    def is_synced(self):
        return self._synced and self.main_sync.is_synced()

    # clock frequency conversions
    def print_time_to_clock(self, print_time):
        if self.clock_adj[1] == 1.0:
            logging.warning(
                "Clock not yet synchronized, clock is untrustworthy"
            )
            for line in traceback.format_stack():
                logging.warning(line.strip())
        adjusted_offset, adjusted_freq = self.clock_adj
        return int((print_time - adjusted_offset) * adjusted_freq)

    def clock_to_print_time(self, clock):
        if self.clock_adj[1] == 1.0:
            logging.warning(
                "Clock not yet synchronized, print time is untrustworthy"
            )
            for line in traceback.format_stack():
                logging.warning(line.strip())
        adjusted_offset, adjusted_freq = self.clock_adj
        return clock / adjusted_freq + adjusted_offset

    # misc commands
    def dump_debug(self):
        adjusted_offset, adjusted_freq = self.clock_adj
        return "%s clock_adj=(%.3f %.3f)" % (
            ClockSync.dump_debug(self),
            adjusted_offset,
            adjusted_freq,
        )

    def stats(self, eventtime):
        adjusted_offset, adjusted_freq = self.clock_adj
        return "%s adj=%d" % (ClockSync.stats(self, eventtime), adjusted_freq)

    def calibrate_clock(self, print_time, eventtime):
        # Calculate: est_print_time = main_sync.estimatated_print_time()
        ser_time, ser_clock, ser_freq = self.main_sync.clock_est
        main_mcu_freq = self.main_sync.mcu_freq
        est_main_clock = (eventtime - ser_time) * ser_freq + ser_clock
        est_print_time = est_main_clock / main_mcu_freq
        # Determine sync1_print_time and sync2_print_time
        sync1_print_time = max(print_time, est_print_time)
        sync2_print_time = max(
            sync1_print_time + 4.0,
            self.last_sync_time,
            print_time + 2.5 * (print_time - est_print_time),
        )
        # Calc sync2_sys_time (inverse of main_sync.estimatated_print_time)
        sync2_main_clock = sync2_print_time * main_mcu_freq
        sync2_sys_time = ser_time + (sync2_main_clock - ser_clock) / ser_freq
        # Adjust freq so estimated print_time will match at sync2_print_time
        sync1_clock = self.print_time_to_clock(sync1_print_time)
        sync2_clock = self.get_clock(sync2_sys_time)
        adjusted_freq = (sync2_clock - sync1_clock) / (
            sync2_print_time - sync1_print_time
        )
        adjusted_offset = sync1_print_time - sync1_clock / adjusted_freq
        # Apply new values
        self.clock_adj = (adjusted_offset, adjusted_freq)
        self.last_sync_time = sync2_print_time
        return self.clock_adj
