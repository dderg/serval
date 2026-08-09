# Installation

> **Serval motion-path notice:** this inherited installation guide contains general Kalico/Klipper subsystem material. For a printer that will execute Serval motion, use [Quickstart](Quickstart.md) as the canonical conversion and upgrade procedure. Serval requires matching host native artifacts and firmware on every MCU; it supports only the board families listed in [Hardware support](Hardware_Support.md). Do not follow a legacy board or flashing recipe if it conflicts with those pages.

These instructions assume the software will run on a Linux-based host
running a Kalico-compatible front end. It is recommended that a
SBC(Small Board Computer) such as a Raspberry Pi or Debian-based Linux
device be used as the host machine (see the
[FAQ](FAQ.md#can-i-run-kalico-on-something-other-than-a-raspberry-pi-3)
for other options).

For the purposes of these instructions, host relates to the Linux device and
mcu relates to the printer board. SBC relates to the term Small Board Computer
such as the Raspberry Pi.

## Obtain a Kalico Configuration File

The repository's `config/` directory is **not** a catalogue of current Serval printer configurations; it currently contains deployment/observability material. Preserve and migrate a known-good configuration with [Config migration](Config_Migration.md) and validate it against [Motion configuration reference](Config_Reference_Motion.md). Do not assume an inherited `printer-*` or `generic-*` example is compatible with the Serval motion protocol.

Most Kalico settings are determined by a "printer configuration file"
printer.cfg, that will be stored on the host. An appropriate configuration
file can often be found by looking in the Kalico
[config directory](../config/) for a file starting with a "printer-"
prefix that corresponds to the target printer. The Kalico
configuration file contains technical information about the printer
that will be needed during the installation.

If there isn't an appropriate printer configuration file in the
Kalico config directory then try searching the printer manufacturer's
website to see if they have an appropriate Kalico configuration file.

If no configuration file for the printer can be found, but the type of
printer control board is known, then look for an appropriate
[config file](../config/) starting with a "generic-" prefix. These
example printer board files should allow one to successfully complete
the initial installation, but will require some customization to
obtain full printer functionality.

It is also possible to define a new printer configuration from
scratch. However, this requires significant technical knowledge about
the printer and its electronics. It is recommended that most users
start with an appropriate configuration file. If creating a new custom
printer configuration file, then start with the closest example
[config file](../config/) and use the Kalico
[config reference](Config_Reference.md) for further information.

## Interacting with Kalico

Kalico is a 3d printer firmware, so it needs some way for the user to
interact with it.

Currently the best choices are front ends that retrieve information through
the [Moonraker web API](https://moonraker.readthedocs.io/) and there is also
the option to use [Octoprint](https://octoprint.org/) to control Kalico.

The choice is up to the user on what to use, but the underlying Kalico is the
same in all cases. We encourage users to research the options available and
make an informed decision.

## Obtaining an OS image for SBC's

There are many ways to obtain an OS image for Kalico for SBC use, most depend on
what front end you wish to use. Some manufacturers of these SBC boards also provide
their own Klipper-centric images, which are also compatible with Kalico.

The two main Moonraker-based front ends are [Fluidd](https://docs.fluidd.xyz/)
and [Mainsail](https://docs.mainsail.xyz/), the latter of which has a premade install
image ["MainsailOS"](https://docs-os.mainsail.xyz/), this has the option for Raspberry Pi
and some OrangePi variants.

Fluidd can be installed via KIAUH(Klipper Install And Update Helper), which
is explained below and is a 3rd party installer for all things Kalico.

OctoPrint can be installed via the popular OctoPi image or via KIAUH, this
process is explained in [OctoPrint.md](OctoPrint.md)

## Installing via KIAUH

Normally you would start with a base image for your SBC, RPiOS Lite for example,
or in the case of an x86 Linux device, Ubuntu Server. Please note that Desktop
variants are not recommended due to certain helper programs that can stop some
Kalico functions from working and even mask access to some printer boards.

KIAUH can be used to install Kalico and its associated programs on a variety
of Linux-based systems that run a form of Debian. More information can be found
at https://github.com/dw-0/kiauh

## Host memory requirements

Kalico's motion pipeline runs on the host, and its worker threads have to
keep feeding the micro-controller ahead of real time. If the kernel has
paged those threads out, waking them costs a disk round trip: on a
memory-pressured SBC we have measured pipeline threads sitting
unscheduled for 0.3 to 1.9 seconds while their pages faulted back in.
Homing and probing hand work to the micro-controller in 100ms slices, so
a stall of that size arrives after the slice it belongs to has already
played, and Kalico aborts with a "start time in the past" error rather
than quietly moving the toolhead at the wrong moment.

To make that impossible, the Kalico host process locks all of its current
and future memory (`mlockall(MCL_CURRENT | MCL_FUTURE)`) once, immediately
before the planner, pump, and position-polling threads are spawned.
Locking cannot be partially applied, so if it fails the printer does not
start at all: Kalico reports the operating system error and names the
setting to change. A host whose locked-memory budget is too small fails
with `Cannot allocate memory` (`ENOMEM`) — that is a configuration
problem on the host, not a fault in the printer.

### Required: raise LimitMEMLOCK on the service

Most distributions give a service a locked-memory allowance of only a few
megabytes, which is far below what the host needs. The unit that runs
`klippy.py` (`klipper.service` in a KIAUH, MainsailOS, or OctoPi install)
therefore needs an unlimited allowance. Create a drop-in:

```
sudo systemctl edit klipper.service
```

and add:

```
[Service]
LimitMEMLOCK=infinity
```

Then apply it:

```
sudo systemctl daemon-reload
sudo systemctl restart klipper.service
```

Verify that the unit asks for it, and that the running process actually
got it:

```
systemctl show -p LimitMEMLOCK klipper.service
grep 'Max locked memory' /proc/$(pgrep -f klippy.py)/limits
```

The first should report `LimitMEMLOCK=infinity`, the second `unlimited`
in both the soft and hard columns. A small number in `/proc` means the
drop-in was not applied or the service was not restarted.

If Kalico is started by hand instead of by systemd, the same allowance
comes from `ulimit -l unlimited` in the shell that launches it, which
requires either root or a matching `memlock` entry in
`/etc/security/limits.conf`.

Hosts driving an EtherCAT servo bus already have this: the drop-in in the
[EtherCAT installation guide](rewrite/ethercat-igh-macb-install.md) sets
`LimitMEMLOCK=infinity` together with `AmbientCapabilities=CAP_IPC_LOCK`,
which lifts the limit outright, for the endpoint's own memory lock.

### Recommended: no disk swap

Locking the host process protects the host process. It does not stop the
rest of the machine from thrashing, and a printer that is swapping is a
printer whose front end, camera streamer, and log writers are all stalling
on the same disk. Production printers should run without disk swap:

```
sudo systemctl disable --now dphys-swapfile   # Raspberry Pi OS
sudo swapoff -a                               # other distributions
```

On non-Raspberry Pi systems also remove the swap entry from
`/etc/fstab` so it does not come back at the next boot. Confirm with
`swapon --show`, which should print nothing.

If the host is tight on memory and you would rather absorb pressure than
let the out-of-memory killer pick a victim, a compressed in-RAM swap
device (`zram`) is a reasonable safety valve, and it is much faster than
swapping to an SD card. It is not a replacement for the memory lock: the
lock is what guarantees the motion threads are never evicted at all,
while zram only makes evicting *other* processes cheaper. Keep both.

## Building and flashing the micro-controller

To compile the micro-controller code, start by running these commands
on your host device:

```
cd ~/klipper/
make menuconfig
```

The comments at the top of the
[printer configuration file](#obtain-a-kalico-configuration-file)
should describe the settings that need to be set during "make
menuconfig". Open the file in a web browser or text editor and look
for these instructions near the top of the file. Once the appropriate
"menuconfig" settings have been configured, press "Q" to exit, and
then "Y" to save. Then run:

```
make
```

If the comments at the top of the
[printer configuration file](#obtain-a-kalico-configuration-file)
describe custom steps for "flashing" the final image to the printer
control board, then follow those steps and then proceed to
[configuring OctoPrint](OctoPrint.md#configuring-octoprint-to-use-kalico).

Otherwise, the following steps are often used to "flash" the printer
control board. First, it is necessary to determine the serial port
connected to the micro-controller. Run the following:

```
ls /dev/serial/by-id/*
```

It should report something similar to the following:

```
/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0
```

It's common for each printer to have its own unique serial port name.
This unique name will be used when flashing the micro-controller. It's
possible there may be multiple lines in the above output - if so,
choose the line corresponding to the micro-controller. If many
items are listed and the choice is ambiguous, unplug the board and
run the command again, the missing item will be your print board(see the
[FAQ](FAQ.md#wheres-my-serial-port) for more information).

For common micro-controllers with STM32 or clone chips, LPC chips and
others, it is usual that these need an initial Kalico flash via SD card.

When flashing with this method, it is important to make sure that the
print board is not connected with USB to the host, due to some boards
being able to feed power back to the board and stopping a flash from
occurring.

For common micro-controllers using Atmega chips, for example the 2560,
the code can be flashed with something
similar to:

```
sudo service klipper stop
make flash FLASH_DEVICE=/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0
sudo service klipper start
```

Be sure to update the FLASH_DEVICE with the printer's unique serial
port name.

For common micro-controllers using RP2040 chips, the code can be flashed
with something similar to:

```
sudo service klipper stop
make flash FLASH_DEVICE=first
sudo service klipper start
```

It is important to note that RP2040 chips may need to be put into Boot mode
before this operation.


## Configuring Kalico

The next step is to copy the
[printer configuration file](#obtain-a-kalico-configuration-file) to
the host.

Arguably the easiest way to set the Kalico configuration file is using the
built-in editors in Mainsail or Fluidd. These will allow the user to open
the configuration examples and save them to be printer.cfg.

Another option is to use a desktop editor that supports editing files
over the "scp" and/or "sftp" protocols. There are freely available tools
that support this (eg, Notepad++, WinSCP, and Cyberduck).
Load the printer config file in the editor and then save it as a file
named "printer.cfg" in the home directory of the pi user
(ie, /home/pi/printer.cfg).

Alternatively, one can also copy and edit the file directly on the
host via SSH. That may look something like the following (be
sure to update the command to use the appropriate printer config
filename):

```
cp ~/klipper/config/example-cartesian.cfg ~/printer.cfg
nano ~/printer.cfg
```

It's common for each printer to have its own unique name for the
micro-controller. The name may change after flashing Kalico, so rerun
these steps again even if they were already done when flashing. Run:

```
ls /dev/serial/by-id/*
```

It should report something similar to the following:

```
/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0
```

Then update the config file with the unique name. For example, update
the `[mcu]` section to look something similar to:

```
[mcu]
serial: /dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0
```

After creating and editing the file, it will be necessary to issue a
"restart" command in the command console to load the config. A
"status" command will report that the printer is ready if the Kalico
config file is successfully read and the micro-controller is
successfully found and configured.

When customizing the printer config file, it is not uncommon for
Kalico to report a configuration error. If an error occurs, make any
necessary corrections to the printer config file and issue "restart"
until "status" reports the printer is ready.

Kalico reports error messages via the command console and pop-ups in
Fluidd and Mainsail. The "status" command can be used to re-report error
messages. A log is available and usually located at
`~/printer_data/logs/klippy.log`.

After Kalico reports that the printer is ready, proceed to the
[config check document](Config_checks.md) to perform some basic checks
on the definitions in the config file. See the main
[documentation reference](Overview.md) for other information.
