"""Cartographer scanner MCU stub.

Subclasses BeaconMcuStub for the serial framing, virtual clock, trsync
bookkeeping, and Z step tracking; replaces the device surface with the
cartographer command set (dderg/cartographer-klipper ``kalico-seam``
branch, ``sensor: cartographer``):

* ``cartographer_data clock=%u data=%u temp=%u`` — one message per
  sample (no beacon-style delta batching).
* ``cartographer_home`` carries ``threshold`` and ``trigger_method``:
  SCAN (0) triggers on the count thresholds armed by
  ``cartographer_set_threshold`` exactly like beacon proximity homing;
  TOUCH (1) triggers when the tracked toolhead Z reaches the bed at 0,
  like beacon contact — but there is no contact-state query protocol,
  so only the trsync fires.
* ``cartographer_base_read`` serves the "no factory calibration"
  sentinels, so the temperature-compensation model stays disabled.

The analytic Z->frequency model is inherited from the beacon stub; the
count encoding is adjusted for scanner.py deriving
``sensor_freq = CLOCK_FREQ / 2`` at 20 MHz, so the beacon saved-model
polynomial works verbatim in the ``[scanner model default]`` config
block.
"""

from __future__ import annotations

import logging
import struct
import time

from tools.sim.emulators.beacon_mcu import BeaconMcuStub
from tools.sim.emulators.cartographer_identify_dict import (
    CLOCK_FREQ,
    IDENTIFY_BLOB,
    SENSOR_FREQ,
)

TRIGGER_METHOD_SCAN = 0
TRIGGER_METHOD_TOUCH = 1

# 10k pullup / 47k-at-25C beta-4041 thermistor (scanner.py hardcodes the
# coefficients): adc_fraction(25C) = 47/(47+10), raw = fraction * ADC_MAX
# * CARTOGRAPHER_ADC_SMOOTH_COUNT.
TEMP_RAW_25C = int(47000.0 / 57000.0 * 4095 * 16)

BASE_DATA_UNCALIBRATED = struct.pack("<IH", 0xFFFFFFFF, 0xFFFF)


class CartographerMcuStub(BeaconMcuStub):
    IDENTIFY_BLOB = IDENTIFY_BLOB
    CLOCK_FREQ = CLOCK_FREQ
    STUB_NAME = "cartographer-stub"

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
            "cartographer_stream": self._handle_beacon_stream,
            "cartographer_set_threshold": self._handle_beacon_set_threshold,
            "cartographer_home": self._handle_cartographer_home,
            "cartographer_stop_home": self._handle_cartographer_stop_home,
            "cartographer_base_read": self._handle_base_read,
            "config_trsync": self._handle_config_trsync,
            "trsync_start": self._handle_trsync_start,
            "trsync_set_timeout": self._handle_noop,
            "trsync_trigger": self._handle_trsync_trigger,
            "stepper_stop_on_trigger": self._handle_noop,
        }

    def _freq_to_count(self, freq_hz: int) -> int:
        return int(freq_hz * (2**28) / SENSOR_FREQ)

    def _handle_cartographer_home(self, params: dict) -> None:
        method = params["trigger_method"]
        if method == TRIGGER_METHOD_TOUCH:
            self._contact_homing_active = True
            self._contact_trsync_oid = params["trsync_oid"]
            self._contact_trigger_reason = params["trigger_reason"]
            if self._homing_trigger_timer is not None:
                self._homing_trigger_timer.set()
            step_tracking_will_fire_at_bed_contact = (
                self._step_thread is not None
            )
            if not step_tracking_will_fire_at_bed_contact:
                self._homing_trigger_timer = self._start_virtual_timer(
                    self._homing_trigger_delay, self._fire_contact_trigger
                )
            logging.info(
                "cartographer-stub: touch home trsync_oid=%d z=%.2f",
                self._contact_trsync_oid,
                self._z_current,
            )
            return
        self._handle_beacon_home(params)

    def _handle_cartographer_stop_home(self, params: dict) -> None:
        self._home_active = False
        self._contact_homing_active = False
        if self._homing_trigger_timer is not None:
            self._homing_trigger_timer.set()
            self._homing_trigger_timer = None

    def _fire_contact_trigger(self, trigger_time=None) -> None:
        # Unlike beacon contact there is no detect-clock protocol: the
        # trsync report is the whole trigger.
        if not self._contact_homing_active:
            return
        self._contact_homing_active = False
        trigger_clock = (
            self._now_clock()
            if trigger_time is None
            else self._clock_at(trigger_time)
        )
        self._trsync_can_trigger[self._contact_trsync_oid] = False
        self._trsync_trigger_reason[self._contact_trsync_oid] = (
            self._contact_trigger_reason
        )
        logging.info(
            "cartographer-stub: TOUCH TRIGGER z=%.3f clock=%d",
            self._z_current,
            trigger_clock,
        )
        self._send_msg(
            "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u",
            oid=self._contact_trsync_oid,
            can_trigger=0,
            trigger_reason=self._contact_trigger_reason,
            clock=trigger_clock,
        )

    def _handle_base_read(self, params: dict) -> None:
        length = params["len"]
        offset = params["offset"]
        end = offset + length
        image = BASE_DATA_UNCALIBRATED
        if end > len(image):
            data = image[offset:] + b"\xff" * (end - len(image))
        else:
            data = image[offset:end]
        self._send_msg(
            "cartographer_base_data bytes=%*s offset=%hu",
            bytes=list(data),
            offset=offset,
        )

    def _sample_loop(self) -> None:
        SAMPLE_HZ = 200.0
        period = 1.0 / SAMPLE_HZ
        next_sample = time.monotonic()

        while not self._stop.is_set() and self._stream_en:
            now = time.monotonic()
            sleep_for = next_sample - now
            if sleep_for > 0:
                time.sleep(min(sleep_for, period))
                continue
            next_sample += period

            if self._home_active and not self._step_tracking:
                elapsed = self._monotonic() - self._homing_start_time
                self._z_current = max(
                    0.0,
                    self._homing_start_z
                    - elapsed * self._homing_approach_speed,
                )

            freq = self._z_to_frequency(self._z_current)
            data_value = self._freq_to_count(freq)
            self._send_msg(
                "cartographer_data clock=%u data=%u temp=%u",
                clock=self._now_clock(),
                data=data_value,
                temp=TEMP_RAW_25C,
            )
            self.tx_sample_count += 1
