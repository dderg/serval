# Execution modes, timing, and recovery

Serval normally sends polynomial trajectory **pieces** to each MCU. Firmware evaluates continuous position at its configured sample rate and emits step/dir or phase output. This is different from the inherited host-generated step-time queue. Host, native artifacts, and every MCU firmware image must be built from the same revision.

## Select an execution mode

| Mode | Use | Restrictions |
|---|---|---|
| `piece` | Normal supported streaming executor | Required for phase stepping and EtherCAT position endpoints. MCU sample rate and ring storage are real-time build constraints. |
| `stepcompress` | Compatibility path for constrained step/dir MCU topology | Host sends interval queues; cannot drive phase-stepped motors or EtherCAT endpoints. Configure its MCU options in the generic `[mcu]` reference. |

Do not select a mode merely to suppress a capacity error. Check pulse width, microsteps, rotation distance, motor speed, sample rate, and target support first. F4/G0/H7 are the printing families; F103 is explicitly not print-supported. See [Hardware support](Hardware_Support.md).

## First motion and timing gate

Before first motion, Serval waits for participating MCU clock synchronization. Failure to converge within 60 seconds is an error, not a state to bypass. The dispatcher anchors pieces in endpoint clocks and the pump keeps buffered lead. Normal streaming targets roughly seconds of lead; homing intentionally uses a short drip window (about 100 ms), so scheduler or transport stalls that are harmless during a print can abort homing.

The host must have locked memory available (`LimitMEMLOCK=infinity`) and should not disk-swap; see [Installation](Installation.md#host-memory-requirements). A `piece in past`, start-time-in-past, endpoint failure, buffer underrun, or clock fault is fail-stop: stop, collect diagnostics, correct the host/transport/firmware cause, verify physical position, and home. Never retry a job at an assumed coordinate.

## Phase stepping

`phase_stepping: True` is a `[motor]` opt-in for compatible supported hardware and required compatible TMC setup. Firmware capability, a maximum per-MCU motor count, and driver support are checked. Phase stepping is simulator-verified but not recently hardware-validated; switch-endstop homing in this mode is not simulator-covered. It cannot coexist with `stepcompress` on that MCU.

## M84/M18 and physical position

`M84`/`M18` disables registered motors. On an EtherCAT servo axis this may make the physical position diverge from the previously commanded coordinate. Before later motion, Serval attempts to reseed parked servo axes from live encoder position; an unavailable endpoint/query is a loud error and requires a safe recovery/home. Do not use motor-disable commands as a casual pause mechanism on a servo machine.

`M400` waits for both planner drain and MCU execution, including queued dwell. `M114` reports the commanded G-code position. `GET_POSITION` also reports engine-measured axes when available; `ERR` means no usable measured value. Treat a disagreement as an investigation, not an offset to ignore.
