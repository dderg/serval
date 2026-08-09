# EtherCAT servos

> **Support boundary:** EtherCAT servo motion is solid on the test bench, not a generic drop-in stepper replacement. It needs an appropriately prepared EtherCAT endpoint host, supported drives, conservative bring-up, and a recovery plan. Read [Feature status](Feature_Status.md) first.

Serval uses an independent native EtherCAT real-time endpoint; EtherCAT is not an MCU transport. Servo motors use `drive: servo`, consume planned position targets, and are configured through an `[ethercat_node <name>]` plus their `[motor]` sections.

## Endpoint node reference

```ini
[ethercat_node drive_bus]
socket: /run/serval-ethercat.sock
interface: eth0
#endpoint: rust/target/release/ethercat-rt
#cycle_us: 1000
#late_tolerance_us: 0
#group_delay_us: 1000
```

| Option | Rule |
| --- | --- |
| `socket` | Required endpoint socket path. |
| `interface` | Required dedicated EtherCAT network interface. Do not use a general LAN interface. |
| `endpoint` | Optional endpoint executable; default is `rust/target/release/ethercat-rt`. |
| `cycle_us` | Positive multiple of 250 µs. |
| `late_tolerance_us` | Optional, default 0: strict late-cycle policy. |
| `group_delay_us` | Optional, default `cycle_us`. |

A node supports at most eight distinct chain indices. Use each `ethercat_chain_index` once. A node-level dynamics profile and per-motor dynamics profiles are mutually exclusive; coupled profiles also require consistent `velocity_ff` and `ff_max_torque` across participating motors.

## Motor binding

```ini
[motor x_servo]
drive: servo
protocol: ethercat
node: drive_bus
ethercat_chain_index: 0
rotation_distance: 40
encoder_counts_per_rev: 4096
#velocity_ff: False
#ff_max_torque: 30
#following_error:
#max_torque:
```

`protocol`, `node`, `ethercat_chain_index`, `rotation_distance`, and `encoder_counts_per_rev` are required. Bounds and homing-only settings are defined in [Motion configuration reference](Config_Reference_Motion.md). Configure following-error and torque limits deliberately from drive documentation and measured mechanics; they are protection settings, not performance knobs.

## Bring-up safety

Build the endpoint only on the prepared bench/host (`./scripts/build-native.sh --bench --ethercat hw`). Its real hardware link needs the IgH stack; `make -f Makefile.rust setcap-ethercat` grants raw-network, real-time scheduling, and memory-lock capabilities after each binary rebuild. This is privileged hardware work—do not run it casually.

Before enabling motion: isolate the network, verify chain order and encoder direction, validate fault state and torque limits, test encoder reporting at standstill, home at conservative speed, then test a short attended move. `M84` can make servo position non-authoritative; see [Execution and timing](Execution_and_Timing.md).

## Service commands

The following modules are optional and must be configured/understood before use:

- `SERVO_CAPTURE_START [AXIS=<axis>|SERVO=<motor[,motor...]>] [NAME=<tag>]` starts one DC-rate `.scap` capture; the selectors are exclusive. Use `M400`, then `SERVO_CAPTURE_STOP` to finish and report path/sample duration.
- `SERVO_PARAM SERVO=<motor> GET=<0xINDEX.SUB> [TYPE=u8|u16|u32|i8|i16|i32]` reads an SDO. Replace `GET` with `SET=<address> VALUE=<integer>` to write; exactly one operation is allowed. Raw SDO writes can damage drive configuration—record originals and use vendor documentation.
- `SERVO_SYNC [AXIS=X|Y] [TORQUE_OK=<pct>] [SETTLE=<seconds>] [RETRIES=<n>]` reseeds supported non-Z dual-servo belt pairs.
- `SERVO_DIFF_TRIM BELT=A|B|AB [GAIN=...] [MAX_OFFSET_UM=...] [LPF_HZ=...] [SETTLE_MS=...] [REMOVE=1] [SAVE=1]` controls standstill differential trim. `SAVE=1` writes configuration state; review it before keeping it.
- `SERVO_STRAIN_COMP [ENABLE=0|1]` applies or ramp-clears a validated strain map. It requires exactly two dual-servo non-Z belt axes and a matching map file.

Capture, sync, trim, strain compensation, torque homing, feed-forward, and dynamics profiles are advanced bench procedures. Keep an operator at the machine, collect structured logs, and treat drive faults or encoder disagreement as stop conditions.
