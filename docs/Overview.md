# Documentation overview

Serval documentation has two layers: the Serval motion stack and inherited Kalico/Klipper subsystem documentation. Start with [Documentation guide](Documentation_Guide.md) to understand their authority. For a printer using Serval motion, begin with [Quickstart](Quickstart.md), not the generic installation page.

## Serval motion

- [README](../README.md): rationale and high-level capabilities.
- [Quickstart](Quickstart.md): supported-board gate, host build, firmware matching, and first startup.
- [Feature status](Feature_Status.md): real-hardware, simulator, and exploratory support tiers.
- [Config migration](Config_Migration.md): transition from classic role-encoded configuration.
- [Motion configuration reference](Config_Reference_Motion.md): complete current motion schema.
- [Serval configuration examples](Serval_Configuration_Examples.md): annotated topology patterns.
- [Motion operations and G-code](Motion_Operations.md): Serval-specific move, limit, curve, and tuning commands.
- [Architecture](Architecture.md): host, native pipeline, protocol, and firmware boundaries.
- [Developer guide](Development.md): builds, tests, simulator, and contribution workflow.

## Inherited subsystem documentation

The following references cover the broader Kalico-derived feature estate. Their workflow descriptions may name Kalico because they are inherited; where a page conflicts with Serval motion requirements, the Serval pages above win.

## General information

- [Features](Features.md): A high-level list of features in Kalico.
- [FAQ](FAQ.md): Frequently asked questions.
- [Config changes](Config_Changes.md): Recent software changes that
may require users to update their printer config file.
- [Contact](Contact.md): Information on bug reporting and general
communication with the Kalico developers.

## Installation and Configuration

- [Installation](Installation.md): Guide to installing Kalico.
  - [Octoprint](OctoPrint.md): Guide to installing Octoprint with Kalico.
  - [Migrating from Klipper](Migrating_from_Klipper.md): Moving from
    Klipper's configuration and motion model.
  - [Config migration](Config_Migration.md): Current configuration model
    and migration steps.
- [Config Reference](Config_Reference.md): Description of config
  parameters.
  - [Rotation Distance](Rotation_Distance.md): Calculating the
    rotation_distance stepper parameter.
- [Config checks](Config_checks.md): Verify basic pin settings in the
  config file.
- [Bed level](Bed_Level.md): Information on "bed leveling" in Kalico.
  - [Probe calibrate](Probe_Calibrate.md): Calibration of automatic Z
    probes.
  - [BL-Touch](BLTouch.md): Configure a "BL-Touch" Z probe.
  - [Manual level](Manual_Level.md): Calibration of Z endstops (and
    similar).
  - [Bed Mesh](Bed_Mesh.md): Bed height correction based on XY
    locations.
  - [Axis Twist Compensation](Axis_Twist_Compensation.md): A tool to compensate
    for inaccurate probe readings due to twist in X gantry.
- [Resonance compensation](Resonance_Compensation.md): A tool to
  reduce ringing in prints.
  - [Measuring resonances](Measuring_Resonances.md): Information on
    using adxl345 accelerometer hardware to measure resonance.
- [Pressure advance](Pressure_Advance.md): Calibrate extruder
  pressure.
- [G-Codes](G-Codes.md): Information on commands supported by Kalico.
- [Command Templates](Command_Templates.md): G-Code macros and
  conditional evaluation.
  - [Status Reference](Status_Reference.md): Information available to
    macros (and similar).
- [TMC Drivers](TMC_Drivers.md): Using Trinamic stepper motor drivers
  with Kalico.
- [Multi-MCU Homing](Multi_MCU_Homing.md): Homing and probing using multiple micro-controllers.
- [Slicers](Slicers.md): Configure "slicer" software for Kalico.
- [Skew correction](Skew_Correction.md): Adjustments for axes not
  perfectly square.
- [PWM tools](Using_PWM_Tools.md): Guide on how to use PWM controlled
  tools such as lasers or spindles.
- [Exclude Object](Exclude_Object.md): The guide to the Exclude Objects
  implementation.

## Developer Documentation

- [Motion configuration reference](Config_Reference_Motion.md): Current
  `[printer]`, `[kinematics]`, `[motor]`, `[axis]`, and post-processor schema.
- [Architecture](Architecture.md): Serval pipeline and execution boundaries.
- [Execution and timing](Execution_and_Timing.md): piece/stepcompress modes, timing, phase, and recovery.
- [Homing and endstops](Homing_and_Endstops.md): guarded homing and multi-motor squaring.
- [EtherCAT servos](EtherCAT_Servos.md): endpoint, servo configuration, and advanced operations.
- [Diagnostics and observability](Diagnostics_and_Observability.md): structured-log incident workflow.
- [Developer guide](Development.md): Native build, test, simulator, and review workflows.
- [Documentation guide](Documentation_Guide.md): scope, authority, and maintenance rules.
- [API Server](API_Server.md): Information on Kalico's command and
  control API.
- [CAN bus protocol](CANBUS_protocol.md): Kalico CAN bus message
  format.
- [Contributing](CONTRIBUTING.md): Information on how to submit
  improvements to Kalico.
- [Packaging](Packaging.md): Information on building OS packages.

## Device Specific Documents

- [Example configs](Example_Configs.md): Information on adding an
  example config file to Kalico.
- [SDCard Updates](SDCard_Updates.md): Flash a micro-controller by
  copying a binary to an sdcard in the micro-controller.
- [Raspberry Pi as Micro-controller](RPi_microcontroller.md): Details
  for controlling devices wired to the GPIO pins of a Raspberry Pi.
- [Beaglebone](Beaglebone.md): Details for running Kalico on the
  Beaglebone PRU.
- [Bootloaders](Bootloaders.md): Developer information on
  micro-controller flashing.
- [Bootloader Entry](Bootloader_Entry.md): Requesting the bootloader.
- [CAN bus](CANBUS.md): Information on using CAN bus with Kalico.
  - [CAN bus troubleshooting](CANBUS_Troubleshooting.md): Tips for
    troubleshooting CAN bus.
- [TSL1401CL filament width sensor](TSL1401CL_Filament_Width_Sensor.md)
- [Hall filament width sensor](Hall_Filament_Width_Sensor.md)
- [Load Cells](Load_Cell.md)
