"""The hotend's temperature, which the sim does not model: a first-order
thermal model driven by the heater pin's duty and written to the
thermistor's ADC channel, so M109 and TEMPERATURE_WAIT wait out a
heat-up as on a printer.

    heater pin --(sampled duty)--> model --(Steinhart-Hart)--> ADC count
"""

from __future__ import annotations

import dataclasses
import math
import threading
from typing import Optional

from tools.sim.fuzzing.printer import HotendPins
from tools.sim.world import HarnessPacer, SimError

AMBIENT_C = 25.0
FULL_POWER_C_PER_S = 4.0
COOLING_TAU_S = 120.0
SAMPLE_S = 0.013
REASSERT_S = 1.0
ADC_MAX = 4095
PULLUP_OHM = 4700.0
KELVIN = 273.15
HEATER_CHIP = 0
THERMISTOR_POINTS = {
    "EPCOS 100K B57560G104F": ((25.0, 100000.0), (150.0, 1641.9), (250.0, 226.15)),
}


class Thermistor:
    """klippy/extras/thermistor.py's Steinhart-Hart fit of three
    (temperature, resistance) points behind a pullup, inverted to the ADC
    fraction klippy reads for a temperature."""

    def __init__(self, points):
        inv_t = [1.0 / (t + KELVIN) for t, _ in points]
        ln_r = [math.log(r) for _, r in points]
        ln3_r = [value**3 for value in ln_r]
        inv_t12, inv_t13 = inv_t[0] - inv_t[1], inv_t[0] - inv_t[2]
        ln_r12, ln_r13 = ln_r[0] - ln_r[1], ln_r[0] - ln_r[2]
        ln3_r12, ln3_r13 = ln3_r[0] - ln3_r[1], ln3_r[0] - ln3_r[2]
        self.c3 = (inv_t12 - inv_t13 * ln_r12 / ln_r13) / (
            ln3_r12 - ln3_r13 * ln_r12 / ln_r13
        )
        self.c2 = (inv_t12 - self.c3 * ln3_r12) / ln_r12
        self.c1 = inv_t[0] - self.c2 * ln_r[0] - self.c3 * ln3_r[0]

    def temp_c(self, fraction: float) -> float:
        ln_r = math.log(PULLUP_OHM * fraction / (1.0 - fraction))
        inv_t = self.c1 + self.c2 * ln_r + self.c3 * ln_r**3
        return 1.0 / inv_t - KELVIN

    def fraction(self, temp_c: float) -> float:
        inv_t = 1.0 / (temp_c + KELVIN)
        y = (self.c1 - inv_t) / (2.0 * self.c3)
        x = math.sqrt((self.c2 / (3.0 * self.c3)) ** 3 + y**2)
        ln_r = math.pow(x - y, 1.0 / 3.0) - math.pow(x + y, 1.0 / 3.0)
        resistance = math.exp(ln_r)
        return resistance / (PULLUP_OHM + resistance)

    def adc_count(self, temp_c: float) -> int:
        return round(self.fraction(temp_c) * ADC_MAX)


def thermistor(sensor_type: str) -> Thermistor:
    if sensor_type not in THERMISTOR_POINTS:
        raise ValueError(f"no thermal model for sensor_type {sensor_type!r}")
    return Thermistor(THERMISTOR_POINTS[sensor_type])


@dataclasses.dataclass
class HotendModel:
    """Heats at FULL_POWER_C_PER_S times the duty and loses heat to the
    ambient with time constant COOLING_TAU_S: a hotend's few degrees per
    second, which the configs' PID gains were tuned for."""

    temp_c: float = AMBIENT_C

    def step(self, duty: float, dt_s: float) -> None:
        loss = (self.temp_c - AMBIENT_C) / COOLING_TAU_S
        self.temp_c += dt_s * (FULL_POWER_C_PER_S * duty - loss)


class Hotend:
    """A paced harness thread that samples the heater pin every SAMPLE_S
    of printer time (off the 0.1 s soft-PWM cycle, so samples sweep its
    phase), steps the model and writes the ADC. A restarting MCU answers
    nothing (the heater counts as off) and comes back reading the shim's
    default ADC, so the count is rewritten every REASSERT_S."""

    def __init__(self, control, clock, pins: HotendPins):
        self._control = control
        self._clock = clock
        self._pins = pins
        self._thermistor = thermistor(pins.sensor_type)
        self.model = HotendModel()
        self.error: Optional[str] = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2)

    def _heater_on(self) -> int:
        try:
            return self._control.gpio_output(HEATER_CHIP, self._pins.heater_line)
        except SimError:
            return 0

    def _write(self, count: int) -> None:
        self._control.set_adc(self._pins.adc_channel, count)

    def _run(self) -> None:
        try:
            self._paced_loop()
        except Exception as error:
            self.error = f"hotend model: {error!r}"

    def _paced_loop(self) -> None:
        written = None
        since_write_s = 0.0
        with HarnessPacer(self._clock) as pacer:
            while pacer.sleep(SAMPLE_S, self._stop):
                self.model.step(self._heater_on(), SAMPLE_S)
                count = self._thermistor.adc_count(self.model.temp_c)
                since_write_s += SAMPLE_S
                if count == written and since_write_s < REASSERT_S:
                    continue
                self._write(count)
                written, since_write_s = count, 0.0
