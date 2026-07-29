# CANBUS

This document describes Kalico's CAN bus support.

## Device Hardware

Kalico currently supports CAN on stm32, SAME5x, and rp2040 chips. In
addition, the micro-controller chip must be on a board that has a CAN
transceiver.

To compile for CAN, run `make menuconfig` and select "CAN bus" as the
communication interface. Finally, compile the micro-controller code
and flash it to the target board.

## Host Hardware

In order to use a CAN bus, it is necessary to have a host adapter. It
is recommended to use a "USB to CAN adapter". There are many different
USB to CAN adapters available from different manufacturers. When
choosing one, we recommend verifying that the firmware can be updated
on it. (Unfortunately, we've found some USB adapters run defective
firmware and are locked down, so verify before purchasing.) Look for
adapters that can run Kalico directly (in its "USB to CAN bridge
mode") or that run the
[candlelight firmware](https://github.com/candle-usb/candleLight_fw).

It is also necessary to configure the host operating system to use the
adapter. This is typically done by creating a new file named
`/etc/network/interfaces.d/can0` with the following contents:
```
allow-hotplug can0
iface can0 can static
    bitrate 1000000
    up ip link set $IFACE txqueuelen 128
```

## Terminating Resistors

A CAN bus should have two 120 ohm resistors between the CANH and CANL
wires. Ideally, one resistor located at each the end of the bus.

Note that some devices have a builtin 120 ohm resistor that can not be
easily removed. Some devices do not include a resistor at all. Other
devices have a mechanism to select the resistor (typically by
connecting a "pin jumper"). Be sure to check the schematics of all
devices on the CAN bus to verify that there are two and only two 120
Ohm resistors on the bus.

To test that the resistors are correct, one can remove power to the
printer and use a multi-meter to check the resistance between the CANH
and CANL wires - it should report ~60 ohms on a correctly wired CAN
bus.

## ⚠️ Finding the canbus_uuid for new micro-controllers

Each micro-controller on the CAN bus is assigned a unique id based on
the factory chip identifier encoded into each micro-controller. To
find each micro-controller device id, make sure the hardware is
powered and wired correctly, and then run:
```
~/klippy-env/bin/python ~/klipper/scripts/canbus_query.py can0
```

If CAN devices are detected the above command will
report lines like the following:
```
Found canbus_uuid=11aa22bb33cc, Application: Klipper, Unassigned
Found canbus_uuid=11aa22bb33cc, Application: Kalico, Assigned: 77
```

Each device will have a unique identifier. In the above example,
`11aa22bb33cc` is the micro-controller's "canbus_uuid".

Note that the `canbus_query.py` tool will only report uninitialized
devices - if Kalico (or a similar tool) configures the device then it
will no longer appear in the list.

⚠️ Note that only devices flashed with a Kalico firmware will
respond while assigned a device node ID. Devices using a Klipper firmware
will no longer appear in the list once configured

## Configuring Kalico

Update the Kalico [mcu configuration](Config_Reference.md#mcu) to use
the CAN bus to communicate with the device - for example:
```
[mcu my_can_mcu]
canbus_uuid: 11aa22bb33cc
#canbus_interface: can0
```

`restart_method` is always `command` on a CAN micro-controller; the
`arduino`, `cheetah` and `rpi_usb` methods act on a USB or serial
connection a CAN node does not have.

### Step-rate ceiling on toolhead boards

The motion engine emits step pulses from its sample ISR, so a single axis
cannot exceed `CONFIG_MOTION_SAMPLE_RATE_HZ` steps per second. Toolhead
boards are usually STM32G0, whose default sample rate is 2 kHz - a
bring-up rate, not a throughput rate. A CAN toolhead configured with a
desktop microstepping value reaches that ceiling almost immediately: at
80 steps/mm, 25 mm/s already asks for 2000 steps/s, and the engine faults
with `StepQueueOverflow` rather than silently dropping steps.

Size `microsteps` and `rotation_distance` against the sample rate of the
micro-controller that drives the motor, and raise
`CONFIG_MOTION_SAMPLE_RATE_HZ` (menuconfig) only as far as that
micro-controller's CPU allows.

How many axes the toolhead drives matters as much as the step rate. On a
bench an STM32G0B1 at the default 2 kHz sustained a single axis of
continuous streamed motion for ten minutes without a fault, while driving
three axes from the same micro-controller faulted within a minute with
`StepQueueOverflow` and `Rescheduled timer in the past`. That failure
reproduces identically over USB, so it is a limit of the micro-controller
and its sample rate, not of the CAN link. Keep a G0 toolhead to the axes
it physically drives - normally just the extruder - and leave the machine
axes on the main board.

## CAN-FD

CAN-FD carries up to 64 bytes per frame instead of 8, and switches to a
faster bit rate for the data phase. Enable it by setting a non-zero
"CAN-FD data phase speed" (`CONFIG_CANBUS_DATA_FREQUENCY`) in
`make menuconfig` when building the micro-controller. Zero, the default,
is classic CAN.

Requirements: an FDCAN-capable micro-controller (STM32G0B1, G4, H7 - the
older bxCAN parts such as STM32F4 cannot do FD), a transceiver rated for
the data bit rate, a host adapter whose firmware advertises FD, and the
interface brought up in FD mode, for example:

```
ip link set can0 up type can bitrate 1000000 dbitrate 2000000 fd on
```

`ip link` control modes are sticky. A `loopback on` left over from a
previous test survives a `down`/`up` cycle and silently detaches the
controller from the bus, so clear it explicitly with `loopback off`.

Framing is negotiated, never assumed. The micro-controller reports its
data phase to the host, the link starts in classic framing, and it moves
to 64-byte frames only once the host has seen a non-zero data phase from
the micro-controller and the interface itself is FD-capable. A classic
micro-controller on an FD-capable bus therefore stays on classic frames
instead of being flooded with frames it would reject. Messages of eight
bytes or fewer, including node discovery, stay classic in both
directions.

All of this is verified on a test bench and has never driven a real
print. See [Feature status](Feature_Status.md).

## USB to CAN bus bridge mode

Some micro-controllers support selecting "USB to CAN bus bridge" mode
during Kalico's "make menuconfig". This mode may allow one to use a
micro-controller as both a "USB to CAN bus adapter" and as a Kalico
node.

When Kalico uses this mode the micro-controller appears as a "USB CAN
bus adapter" under Linux. The "Kalico bridge mcu" itself will appear
as if it was on this CAN bus - it can be identified via
`canbus_query.py` and it must be configured like other CAN bus Kalico
nodes.

Bridge mode is classic CAN only: it has no CAN-FD support, so a bridge
board cannot carry an FD data path.

Some important notes when using this mode:

* It is necessary to configure the `can0` (or similar) interface in
  Linux in order to communicate with the bus. However, Linux CAN bus
  speed and CAN bus bit-timing options are ignored by Kalico.
  Currently, the CAN bus frequency is specified during "make
  menuconfig" and the bus speed specified in Linux is ignored.

* Whenever the "bridge mcu" is reset, Linux will disable the
  corresponding `can0` interface. To ensure proper handling of
  FIRMWARE_RESTART and RESTART commands, it is recommended to use
  `allow-hotplug` in the `/etc/network/interfaces.d/can0` file. For
  example:
```
allow-hotplug can0
iface can0 can static
    bitrate 1000000
    up ip link set $IFACE txqueuelen 128
```

* The "bridge mcu" is not actually on the CAN bus. Messages to and
  from the bridge mcu will not be seen by other adapters that may be
  on the CAN bus.

* The available bandwidth to both the "bridge mcu" itself and all
  devices on the CAN bus is effectively limited by the CAN bus
  frequency. As a result, it is recommended to use a CAN bus frequency
  of 1000000 when using "USB to CAN bus bridge mode".

  Even at a CAN bus frequency of 1000000, there may not be sufficient
  bandwidth to run a `SHAPER_CALIBRATE` test if both the XY steppers
  and the accelerometer all communicate via a single "USB to CAN bus"
  interface.

* A USB to CAN bridge board will not appear as a USB serial device, it
  will not show up when running `ls /dev/serial/by-id`, and it can not
  be configured in Kalico's printer.cfg file with a `serial:`
  parameter. The bridge board appears as a "USB CAN adapter" and it is
  configured in the printer.cfg as a [CAN node](#configuring-kalico).

## Tips for troubleshooting

See the [CAN bus troubleshooting](CANBUS_Troubleshooting.md) document.
