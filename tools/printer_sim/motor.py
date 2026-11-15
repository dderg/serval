"""Hybrid stepper on a current-chopper driver (module 2).

Per motor, electrical angle phi = p * theta (p rotor teeth):
  v_a = R i_a + L di_a/dt - K w sin(phi)
  v_b = R i_b + L di_b/dt + K w cos(phi)
  T   = K (i_b cos(phi) - i_a sin(phi)) - T_d sin(4 phi)
The driver commands i* = I (cos phi_cmd, sin phi_cmd) and applies, per
phase and per integration step, the voltage that would reach i* at the
step's end (dead-beat), clipped to +-supply. While the supply suffices the
current follows the command and T = K I sin(delta), delta = phi_cmd - phi
the load angle; at speed the back-EMF and the inductance eat the supply and
the current falls behind. The rotor slips a pole when delta passes +-pi:
it then settles 2 pi (4 full steps) away, a permanent position loss.
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np

from . import parts as parts_mod

SQUARE_WAVE_FUNDAMENTAL = 4.0 / math.pi
PULLOUT_GRID = 2001


@dataclasses.dataclass(frozen=True)
class Electrical:
    resistance: float
    inductance: float
    torque_constant: float
    pole_pairs: int
    detent_torque: float
    supply_voltage: float
    current_amplitude: float
    rotor_inertia: float

    @classmethod
    def from_parts(cls, parts: parts_mod.Parts) -> Electrical:
        return cls.of(parts.motor, parts.driver)

    @classmethod
    def of(cls, m: parts_mod.Motor, driver: parts_mod.Driver) -> Electrical:
        return cls(
            resistance=m.resistance,
            inductance=m.inductance,
            torque_constant=m.torque_constant,
            pole_pairs=m.pole_pairs,
            detent_torque=m.detent_torque,
            supply_voltage=driver.supply_voltage,
            current_amplitude=driver.current_amplitude,
            rotor_inertia=m.rotor_inertia,
        )

    @property
    def peak_torque(self) -> float:
        """Torque at delta = pi/2 while the current follows the command."""
        return self.torque_constant * self.current_amplitude

    @property
    def magnetic_stiffness(self) -> float:
        """dT/dtheta at delta = 0 (N m/rad of rotor)."""
        return self.peak_torque * self.pole_pairs


def pullout_torque(e: Electrical, speed, voltage_amplitude) -> float:
    """Largest steady torque at rotor speed `speed` (rad/s) with sinusoidal
    phase voltages of amplitude <= voltage_amplitude and current amplitude
    <= the commanded one: max K*I*sin(psi) over current I and its angle
    psi ahead of the rotor, subject to |(R + j w_e L) I e^{j psi} + j K w|
    <= V (the phasor form of the voltage equations at w_e = p*w).
    Detent torque averages out at speed and is not included."""
    w_e = e.pole_pairs * speed
    z = complex(e.resistance, w_e * e.inductance)
    emf = 1j * e.torque_constant * speed
    current = np.linspace(0.0, e.current_amplitude, PULLOUT_GRID)[:, None]
    psi = np.linspace(0.0, math.pi, PULLOUT_GRID)[None, :]
    volts = np.abs(z * current * np.exp(1j * psi) + emf)
    torque = np.where(
        volts <= voltage_amplitude,
        e.torque_constant * current * np.sin(psi),
        0.0,
    )
    return float(torque.max())


def pullout_bracket(e: Electrical, speed):
    """The chopper clips each phase at +-supply: its fundamental lies
    between a sine of that amplitude and a square wave's 4/pi times it, so
    the clipped motor's pull-out torque lies between these two."""
    return (
        pullout_torque(e, speed, e.supply_voltage),
        pullout_torque(e, speed, SQUARE_WAVE_FUNDAMENTAL * e.supply_voltage),
    )
