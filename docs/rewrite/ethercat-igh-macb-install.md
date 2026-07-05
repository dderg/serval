# Installing the IgH EtherCAT master with the native `ec_macb` driver (Raspberry Pi 5)

> Portable, bench-agnostic setup guide for the EtherCAT servo stack: a
> PREEMPT_RT kernel, the IgH (EtherLab) EtherCAT master built with a **native
> `ec_macb` device driver** for the Pi 5's on-RP1 gigabit MAC, and the systemd
> glue that hands the NIC to EtherCAT at boot.
>
> This is the infrastructure the servo path sits on. Once it is in place, follow
> [`ethercat-bench-bringup.md`](ethercat-bench-bringup.md) to bring an actual
> drive up (`[ethercat_node]` / `[servo]` config, homing, feedforward, capture).
>
> Substitute your own values for everything in `<angle brackets>` — this guide
> names no specific host.

## Why a *native* driver (and not the generic one)

IgH ships a `generic` device driver that talks to any NIC through the normal
Linux network stack. It works, but every EtherCAT frame traverses the kernel net
path, which adds latency and jitter. For a servo running a **1 kHz distributed-
clock (DC) loop**, that jitter makes the drive miss its SYNC0 window under load
and latch a **synchronization-loss fault** (CiA `0x8700`, panel `ErC1.1`,
EtherCAT AL status `0x001a`). Because the drive is usually on its own always-on
supply, that latch survives host reboots — only a drive power-cycle clears it.

A **native** EtherCAT device driver poll-drives the NIC hardware directly,
bypassing the net stack, and holds cadence even while the rest of the machine is
busy. On the Raspberry Pi 5 the gigabit MAC is the **Cadence GEM** on the RP1
(`macb` kernel driver, platform device `1f00100000.ethernet`), so the native
driver is a purpose-built `ec_macb`. This is what this guide installs.

> **Not on a Pi 5?** The `ec_macb` driver is specific to the RP1 Cadence GEM.
> IgH already carries native drivers for other NICs (`e1000e`, `igb`, `r8169`,
> `ccat`, …). Pick the one matching your NIC, or use `generic` as a starting
> point and accept the jitter tradeoff. The rest of this guide (RT kernel, core
> isolation, systemd glue, endpoint) is NIC-independent.

## What you need

- **A Raspberry Pi 5** (the `ec_macb` driver targets its RP1 Cadence GEM).
- **Debian 13 "trixie"** (or newer). This matters: the `ec_macb` source is a
  *verbatim fork of the Linux 6.18.33 `macb` driver* plus EtherCAT hooks, so it
  must be built against a 6.18.33 kernel tree and the trixie `gcc-14` toolchain.
  Bookworm's kernel and toolchain are too old — see
  [Kernel version pinning](#kernel-version-pinning-important) below. If you are
  on bookworm, `sudo apt full-upgrade` to trixie first (repoint your apt sources
  bookworm→trixie), and rebuild any Python virtualenvs afterward (trixie moves
  Python 3.11→3.13).
- **A dedicated Ethernet port for EtherCAT.** `eth0` is handed entirely to the
  EtherCAT master and gets *no IP address*, so put the Pi's LAN/SSH on Wi-Fi
  (`wlan0`) or a second NIC. Confirm which you are on before you start:
  `ip route get 1.1.1.1` should **not** go out `eth0`.
- **The EtherCAT drive** wired to that port (only needed for the final drive
  bring-up, not for building the stack).
- Root (sudo) and ~2 GB free disk for the kernel + build tree.

## Components you will end up with

| Piece | Where | What it does |
|---|---|---|
| PREEMPT_RT kernel | `kernel8_rt.img` | deterministic scheduling for the DC loop |
| Isolated CPU core | `isolcpus=…` on cmdline | contention-free core the endpoint pins to |
| IgH master + `ec_macb` | `/opt/etherlab`, `/lib/modules/<uname -r>/ethercat` | the EtherCAT master + native NIC driver |
| `ethercat-macb.service` | systemd | hands `eth0` to `ec_macb` and starts the master at boot |
| udev rule | `/etc/udev/rules.d/99-ethercat.rules` | makes `/dev/EtherCAT0` group-accessible |
| klipper RT drop-in | `/etc/systemd/system/klipper.service.d/` | grants the endpoint `SCHED_FIFO` + `mlockall` |
| kalico endpoint | `rust/target/release/ethercat-rt` | the RT process klippy spawns to drive the servo |

The endpoint is **spawned by klippy** at `mcu_identify`; you never launch it by
hand. It opens `/dev/EtherCAT0` (created by the master), not a raw socket.

---

## Step 1 — PREEMPT_RT kernel + an isolated core

Install the RT kernel and its headers (the headers are required to build the
IgH modules later):

```sh
sudo apt update
sudo apt install linux-image-rpi-v8-rt linux-headers-rpi-v8-rt
```

To match the `ec_macb` driver's pinned kernel exactly, install the specific
6.18.33 build and hold it so a later `apt upgrade` cannot bump it out from under
the driver (see [pinning](#kernel-version-pinning-important)):

```sh
sudo apt install linux-image-6.18.33+rpt-rpi-v8-rt linux-headers-6.18.33+rpt-rpi-v8-rt
sudo apt-mark hold linux-image-6.18.33+rpt-rpi-v8-rt linux-headers-6.18.33+rpt-rpi-v8-rt
```

Select the RT image and isolate a core. Back up both files first.

`/boot/firmware/config.txt` — add under the final `[all]`:

```ini
kernel=kernel8_rt.img
```

`/boot/firmware/cmdline.txt` — append to the single line (isolate core **3**;
the endpoint pins to CPU 3 by default — pick any core and pass `--rt-cpu` to
match if you choose a different one):

```
isolcpus=domain,managed_irq,3 nohz_full=3 rcu_nocbs=3 threadirqs
```

Reboot and verify:

```sh
uname -r                 # …-rt   (PREEMPT_RT)
nproc                    # one fewer than physical cores (the isolated one is out of the pool)
cat /proc/cmdline        # shows isolcpus=…,3 nohz_full=3 …
```

## Step 2 — Get the IgH source + the `ec_macb` driver

The master is **IgH EtherLab `stable-1.6`** plus the `ec_macb` native driver.
Both live together in a **published fork** of the upstream IgH repo — `ec_macb`
is a fork addition proposed back to upstream through an open merge request. Clone
the fork (not `gitlab.com/etherlab.org/ethercat.git`, which does not yet carry
`ec_macb`):

```sh
git clone <fork-url> -b <fork-branch> ~/ethercat-igh
```

- **Fork:** `<fork-url>`
- **Merge request (upstream):** `<merge-request-url>`

The `ec_macb` driver is already in that tree at `devices/macb/` — you add no
files by hand. It carries the 6.18.33 file set (`macb.h`, `macb_main.c`,
`macb_ptp.c` as `*-6.18-orig.*` and `*-6.18-ethercat.*`, plus `Kbuild`,
`Makefile.am`, `update.sh`, and `PORTING-NOTES.md`). To (re)generate it for a
different kernel, see
[Obtaining or regenerating `ec_macb`](#obtaining-or-regenerating-ec_macb) — but note
the driver is pinned to 6.18.33 for a reason.

## Step 3 — Build and install the master

```sh
cd ~/ethercat-igh
./bootstrap           # autotools (only needed on a fresh git clone)

# KMM = kernel major.minor of the macb file set (6.18); KREL = running kernel.
./configure --prefix=/opt/etherlab \
            --enable-generic --enable-userlib --enable-tool \
            --enable-macb --with-macb-kernel=6.18 \
            --with-linux-dir=/lib/modules/$(uname -r)/build

make -j"$(nproc)"           # userspace: libethercat, the ethercat tool, init.d
make modules -j"$(nproc)"   # kernel modules: ec_master, ec_generic, ec_macb (Kbuild)
sudo make modules_install install
sudo depmod -a

# Put libethercat on the dynamic linker's search path, or the endpoint fails at
# runtime with "libethercat.so.1: cannot open shared object file".
echo /opt/etherlab/lib | sudo tee /etc/ld.so.conf.d/etherlab.conf
sudo ldconfig
```

**`make` alone builds only userspace** — the kernel modules (`ec_master.ko`,
`ec_macb.ko`, …) come from the separate `make modules` target, so both are
required before `modules_install`. After it, confirm the driver was built against
your running kernel:

```sh
/usr/sbin/modinfo -F vermagic /lib/modules/$(uname -r)/ethercat/devices/macb/ec_macb.ko
# must match `uname -r` (…-rt … preempt_rt …). A mismatch means it built against
# the wrong kernel tree — fix --with-linux-dir and rebuild.
```

`configure` **fails loudly** if `devices/macb/macb_main-<X.Y>-orig.c` for the
`--with-macb-kernel` value is missing — that is the guard that stops you building
the driver against the wrong kernel.

Point the master at your NIC. `/opt/etherlab/etc/sysconfig/ethercat`:

```sh
MASTER0_DEVICE="<eth0 MAC, lowercase, e.g. 2c:cf:67:7d:37:1b>"
DEVICE_MODULES="macb"
```

(`ethercat --version` should now report `IgH EtherCAT master 1.6.x`.)

## Step 4 — Hand `eth0` to `ec_macb` at boot

The in-tree `macb` driver claims the NIC at boot, so a boot service must hand the
platform device over to `ec_macb` before starting the master. Install this as
`/usr/local/sbin/ethercat-macb-up.sh` (root, `chmod 755`):

```bash
#!/bin/bash
# Hand the RP1 Cadence GEM (1f00100000.ethernet) from the builtin macb driver to
# ec_macb, then start the IgH master. Idempotent.
set -u
DEV=1f00100000.ethernet
SYS=/sys/bus/platform

# driver_override pins the device to ec_macb; unbind the builtin macb if it grabbed
# it at boot. init.d loads the modules with main_devices=$MASTER0_DEVICE so the
# master registers for this MAC.
[ -e "$SYS/devices/$DEV/driver_override" ] && echo ec_macb > "$SYS/devices/$DEV/driver_override"
cur=$(basename "$(readlink "$SYS/devices/$DEV/driver" 2>/dev/null)" 2>/dev/null || true)
if [ -n "$cur" ] && [ "$cur" != ec_macb ]; then
    echo "$DEV" > "$SYS/drivers/$cur/unbind" 2>/dev/null || true
fi

/opt/etherlab/etc/init.d/ethercat start

# The endpoint runs unprivileged and opens /dev/EtherCAT0.
sleep 1
chgrp <your-user> /dev/EtherCAT0 2>/dev/null || true
chmod 0660 /dev/EtherCAT0 2>/dev/null || true
```

`/etc/systemd/system/ethercat-macb.service`:

```ini
[Unit]
Description=EtherCAT master on native ec_macb (Pi 5 RP1 Cadence GEM)
DefaultDependencies=no
After=sysinit.target
Wants=sysinit.target
Before=klipper.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/local/sbin/ethercat-macb-up.sh
ExecStop=/opt/etherlab/etc/init.d/ethercat stop

[Install]
WantedBy=multi-user.target
```

Make `/dev/EtherCAT0` accessible to your user
(`/etc/udev/rules.d/99-ethercat.rules`):

```
KERNEL=="EtherCAT[0-9]*", MODE="0660", GROUP="<your-user>"
```

**If you use NetworkManager**, stop it from managing the EtherCAT NIC (otherwise
it fights the handover at boot). `/etc/NetworkManager/conf.d/99-ethercat-unmanaged.conf`:

```ini
[keyfile]
unmanaged-devices=interface-name:eth0
```

Enable and start:

```sh
sudo udevadm control --reload-rules
sudo systemctl daemon-reload
sudo systemctl enable --now ethercat-macb.service
```

Verify the master is live on the native driver:

```sh
lsmod | grep -E 'ec_macb|ec_master'          # both loaded, ec_master used by ec_macb
ls -l /dev/EtherCAT0                          # exists, group = your user
ethercat master                               # "Ethernet devices … Link: UP"
```

## Step 5 — RT capabilities for the endpoint

The endpoint's DC loop **must** run `SCHED_FIFO`, `mlockall`, pinned to the
isolated core — otherwise it aborts the claim loudly (no silent `SCHED_OTHER`
fallback). Grant the capabilities on the klipper service so the spawned endpoint
inherits them (ambient caps survive endpoint rebuilds, unlike a per-inode file
`setcap`). `/etc/systemd/system/klipper.service.d/10-ethercat-rt.conf`:

```ini
[Service]
AmbientCapabilities=CAP_SYS_NICE CAP_IPC_LOCK
LimitRTPRIO=infinity
LimitMEMLOCK=infinity
```

`sudo systemctl daemon-reload`. (Equivalently, `setcap
cap_sys_nice,cap_ipc_lock+ep` on the binary — but re-run it after **every**
rebuild, since `cargo` writes a fresh inode and drops file-caps.)

## Step 6 — Build the kalico endpoint

From the repo, on the Pi (the `hw` build compiles the IgH C shim and links
`libethercat` from `/opt/etherlab` — never cross-compile):

```sh
make -f Makefile.rust ethercat-endpoint-hw     # -> rust/target/release/ethercat-rt
```

For a **drive-off dry run** build the stub instead (`make -f Makefile.rust
ethercat-stub`) and point `[ethercat_node].endpoint` at
`rust/target/release/ethercat-rt-stub`.

## Step 7 — Configure klippy and bring the drive up

That is the boundary of this guide. Add `[ethercat_node]` + your servo config and
follow [`ethercat-bench-bringup.md`](ethercat-bench-bringup.md) for the drive
bring-up, homing, feedforward, and telemetry capture. Minimal node config:

```ini
[ethercat_node node]
socket: /tmp/kalico-ethercat.sock
interface: eth0
```

A `[ethercat_node]` is the EtherCAT bus/master, not an axis — one node carries
one or more drives, each selected by its `ethercat_chain_index`. Name it for the
bus (`node`), not an axis.

### Verify RT is actually in force

A warm restart only proves the caps took; only a **cold reboot** proves the loop
holds cadence under boot load. With the servo claimed:

```sh
pid=$(pgrep -f release/ethercat-rt)
chrt -p "$pid"                          # SCHED_FIFO priority 80
grep Cpus_allowed_list /proc/$pid/status  # the isolated core (e.g. 3)
sudo journalctl -b | grep -c 'al=0x001a'  # 0  (any hits = DC sync loss)
```

---

## Kernel version pinning (important)

The `ec_macb` `*-orig.{c,h}` files are **verbatim from Linux 6.18.33**
(`drivers/net/ethernet/cadence/{macb_main.c,macb.h,macb_ptp.c}`), and the
`*-ethercat.*` files are those plus the EtherCAT hooks. The upstream `macb`
driver changes materially between point releases (macb.h/macb_main.c gained
hundreds of lines between 6.18.33 and 6.18.36), so **the driver only builds
cleanly against a 6.18.33 kernel tree**. This is why:

- The bench targets **6.18.33-rt specifically** and `apt-mark hold`s it.
- Trixie is required — the 6.18.x kernels and the `gcc-14` toolchain they build
  against live there, not in bookworm.
- `--with-macb-kernel=6.18` selects the 6.18 file set; `configure` errors if it
  is absent.

To ride a different kernel you must regenerate the driver for it (next section)
and accept that you are off the tested path.

## Obtaining or regenerating `ec_macb`

The driver was ported from the Pi 4 `genet` native-driver recipe. The hooks are
**additive and gated by `get_ecdev(bp)`** so a non-EtherCAT load is unaffected:

- **Registration** — `ecdev_offer(dev, ec_poll, THIS_MODULE)` before
  `macb_init()` (macb requests IRQs there); `register_netdev` → `ecdev_open`.
- **No IRQ / NAPI in EtherCAT mode** — the master polls, so `devm_request_irq`,
  the link-up IER enable, and `napi_enable`/scheduling are gated off.
- **TX/RX** — TX keeps the DMA map but never frees the skb (persistent ring); RX
  hands the raw frame (MAC header intact, before `eth_type_trans`) to
  `ecdev_receive(...)`.
- **Link** — phylink `mac_link_up/down` → `ecdev_set_link(...)`.
- **`ec_poll`** — per cycle: `macb_tx_complete()` + `gem_rx()` for every queue.
- Driver `.name` → `ec_macb`.

`devices/macb/update.sh <kernel-src-dir> <prev-ver> <new-ver>` regenerates the
`-orig`/`-ethercat` pair for a new kernel by copying the mainline files and
re-applying the previous version's diff, then you fix up any rejects by hand and
add them to `Makefile.am`. Full detail is in `devices/macb/PORTING-NOTES.md`.

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `configure: … kernel 6.XX not available for macb driver` | no `macb_main-6.XX-orig.c` in `devices/macb/` | install the matching file set, or `--with-macb-kernel=6.18` and run the 6.18.33 kernel |
| `linux-headers-…-rt : Depends: gcc-14-for-host but it is not installable` | building on bookworm | upgrade to trixie (the 6.18 kernel + `gcc-14` live there) |
| `ec_macb` won't bind / `eth0` still a normal netdev | the builtin `macb` grabbed the NIC, or NetworkManager re-claimed it | run `ethercat-macb-up.sh` (driver_override + unbind); mark `eth0` unmanaged in NM |
| endpoint aborts claim `rc=-10/-11/-12` | missing `CAP_IPC_LOCK` / isolated core / `CAP_SYS_NICE` | install the klipper RT drop-in (Step 5); confirm the isolated core exists |
| drive latches `ErC1.1` / `0x8700` / `al=0x001a`, "works once connected" | DC loop not truly `SCHED_FIFO` on an isolated core under cold-boot load | verify RT (Step 5); **power-cycle the drive** to clear the latch, then fix the RT cause |
| bringup `rc=-2` "no slaves responding" | drive powered off or cable | power the drive, check the cable, `FIRMWARE_RESTART` |

## See also

- [`ethercat-bench-bringup.md`](ethercat-bench-bringup.md) — drive bring-up, config, homing, faults.
- [`servo-feedforward.md`](servo-feedforward.md) — velocity/torque feedforward.
- [`servo-telemetry-capture.md`](servo-telemetry-capture.md) — `.scap` capture + dynamics fitting.
- `devices/macb/PORTING-NOTES.md` (in the IgH tree) — `ec_macb` provenance and porting.
