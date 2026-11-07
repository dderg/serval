# Micro-controller clock synchronization
#
# Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import math
import traceback

RTT_AGE = 0.000010 / (60.0 * 60.0)
DECAY = 1.0 / 30.0
TRANSMIT_EXTRA = 0.001
CALIBRATION_SAMPLES = 8
MAX_FREQ_DEVIATION = 0.05


class ClockSyncError(Exception):
    pass


class ClockSync:
    def __init__(self, reactor):
        self.reactor = reactor
        self.serial = None
        self.get_clock_timer = reactor.register_timer(self._get_clock_event)
        self.get_clock_cmd = self.cmd_queue = None
        self.queries_pending = 0
        self.mcu_freq = 1.0
        self.last_clock = 0
        self.clock_est = (0.0, 0.0, 0.0)
        # Minimum round-trip-time tracking
        self.min_half_rtt = 999999999.9
        self.min_rtt_time = 0.0
        # Linear regression of mcu clock and system sent_time
        self.time_avg = self.time_variance = 0.0
        self.clock_avg = self.clock_covariance = 0.0
        self.prediction_variance = 0.0
        self.last_prediction_time = 0.0

    def disconnect(self):
        self.reactor.update_timer(self.get_clock_timer, self.reactor.NEVER)

    def connect(self, serial):
        self.serial = serial
        self.mcu_freq = serial.msgparser.get_constant_float("CLOCK_FREQ")
        # get_uptime only extends the clock to 64 bits: its reply time is not
        # bounded (it follows identify), so it must not anchor the estimate
        params = serial.send_with_response("get_uptime", "uptime")
        self.last_clock = (params["high"] << 32) | params["clock"]
        self.clock_est = (params["#sent_time"], self.last_clock, self.mcu_freq)
        samples = []
        for i in range(CALIBRATION_SAMPLES):
            self.reactor.pause(self.reactor.monotonic() + 0.050)
            params = serial.send_with_response("get_clock", "clock")
            clock = self._extend_clock(params["clock"])
            self._update_best_rtt(params["#sent_time"], params["#receive_time"])
            samples.append((params["#sent_time"], clock))
        self._calibrate(samples)
        self.get_clock_cmd = serial.get_msgparser().create_command("get_clock")
        self.cmd_queue = serial.alloc_command_queue()
        serial.register_response(self._handle_clock, "clock")
        self.reactor.update_timer(self.get_clock_timer, self.reactor.NOW)

    def _calibrate(self, samples):
        """Least squares over the calibration samples, which seeds the
        running regression."""
        count = len(samples)
        time_avg = sum(t for t, c in samples) / count
        clock_avg = sum(c for t, c in samples) / count
        time_variance = sum((t - time_avg) ** 2 for t, c in samples) / count
        clock_covariance = (
            sum((t - time_avg) * (c - clock_avg) for t, c in samples) / count
        )
        freq = clock_covariance / time_variance
        deviation = freq / self.mcu_freq - 1.0
        if abs(deviation) > MAX_FREQ_DEVIATION:
            raise ClockSyncError(
                "MCU clock measured at %.0f Hz, %+.2f%% off its CLOCK_FREQ"
                " %.0f Hz: the firmware's clock configuration does not match"
                " the hardware, or the host could not time its clock queries"
                % (freq, 100.0 * deviation, self.mcu_freq)
            )
        self.time_avg = time_avg
        self.clock_avg = clock_avg
        self.time_variance = time_variance
        self.clock_covariance = clock_covariance
        self.prediction_variance = (0.001 * self.mcu_freq) ** 2
        self.last_prediction_time = samples[-1][0]
        self._update_estimate(samples[-1][1])

    def _extend_clock(self, clock32):
        clock_delta = (clock32 - self.last_clock) & 0xFFFFFFFF
        self.last_clock += clock_delta
        return self.last_clock

    def _update_best_rtt(self, sent_time, receive_time):
        half_rtt = 0.5 * (receive_time - sent_time)
        aged_rtt = (sent_time - self.min_rtt_time) * RTT_AGE
        if half_rtt < self.min_half_rtt + aged_rtt:
            self.min_half_rtt = half_rtt
            self.min_rtt_time = sent_time
            logging.debug(
                "new minimum rtt %.3f: hrtt=%.6f freq=%d",
                sent_time,
                half_rtt,
                self.clock_est[2],
            )

    def connect_file(self, serial, pace=False):
        self.serial = serial
        self.mcu_freq = serial.msgparser.get_constant_float("CLOCK_FREQ")
        self.clock_est = (0.0, 0.0, self.mcu_freq)
        freq = 1000000000000.0
        if pace:
            freq = self.mcu_freq
        serial.set_clock_est(freq, self.reactor.monotonic(), 0, 0)

    # MCU clock querying (_handle_clock is invoked from background thread)
    def _get_clock_event(self, eventtime):
        self.serial.raw_send(self.get_clock_cmd, 0, 0, self.cmd_queue)
        self.queries_pending += 1
        # Use an unusual time for the next event so clock messages
        # don't resonate with other periodic events.
        return eventtime + 0.9839

    def _handle_clock(self, params):
        self.queries_pending = 0
        clock = self._extend_clock(params["clock"])
        sent_time = params["#sent_time"]
        if not sent_time:
            return
        receive_time = params["#receive_time"]
        self._update_best_rtt(sent_time, receive_time)
        # Filter out samples that are extreme outliers
        exp_clock = (sent_time - self.time_avg) * self.clock_est[
            2
        ] + self.clock_avg
        clock_diff2 = (clock - exp_clock) ** 2
        if (
            clock_diff2 > 25.0 * self.prediction_variance
            and clock_diff2 > (0.000500 * self.mcu_freq) ** 2
        ):
            # A sample that went out late reads the clock late: it may lead
            # the prediction by at most its own round trip. A larger lead
            # means the estimate is wrong, and ignoring it would freeze it.
            if (
                clock > exp_clock
                and sent_time < self.last_prediction_time + 10.0
                and clock - exp_clock
                <= (receive_time - sent_time) * self.mcu_freq
            ):
                logging.debug(
                    "Ignoring clock sample %.3f: freq=%d diff=%d stddev=%.3f",
                    sent_time,
                    self.clock_est[2],
                    clock - exp_clock,
                    math.sqrt(self.prediction_variance),
                )
                return
            logging.info(
                "Resetting prediction variance %.3f:"
                " freq=%d diff=%d stddev=%.3f",
                sent_time,
                self.clock_est[2],
                clock - exp_clock,
                math.sqrt(self.prediction_variance),
            )
            self.prediction_variance = (0.001 * self.mcu_freq) ** 2
        else:
            self.last_prediction_time = sent_time
            self.prediction_variance = (1.0 - DECAY) * (
                self.prediction_variance + clock_diff2 * DECAY
            )
        # Add clock and sent_time to linear regression
        diff_sent_time = sent_time - self.time_avg
        self.time_avg += DECAY * diff_sent_time
        self.time_variance = (1.0 - DECAY) * (
            self.time_variance + diff_sent_time**2 * DECAY
        )
        diff_clock = clock - self.clock_avg
        self.clock_avg += DECAY * diff_clock
        self.clock_covariance = (1.0 - DECAY) * (
            self.clock_covariance + diff_sent_time * diff_clock * DECAY
        )
        self._update_estimate(clock)

    def _update_estimate(self, clock):
        new_freq = self.clock_covariance / self.time_variance
        pred_stddev = math.sqrt(self.prediction_variance)
        self.serial.set_clock_est(
            new_freq,
            self.time_avg + TRANSMIT_EXTRA,
            int(self.clock_avg - 3.0 * pred_stddev),
            clock,
        )
        self.clock_est = (
            self.time_avg + self.min_half_rtt,
            self.clock_avg,
            new_freq,
        )

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
        if self.clock_est[2] <= 0.0:
            # Never received a clock estimate from this mcu (for example a
            # non-critical mcu that was not reachable at startup). The
            # calculation below would degenerate and eventually divide by
            # zero, so keep the current (unsynchronized) adjustment.
            return self.clock_adj
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
