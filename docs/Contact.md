# Contact

This document provides contact information for Kalico.
Kalico a community-maintained fork of the Kalico firmware.

## Discord

Kalico has a dedicated Discord server where you can chat with the 
developers and users of Kalico in real-time.

You can join the server here:
[kalico.gg/discord](https://kalico.gg/discord)

## I have a question about Kalico

Many questions we receive are already answered in the
[Overview](Overview.md). Please be sure to to read the
documentation and follow the directions provided there.

If you are interested in sharing your knowledge and experience with
other Kalico users then you can join the Kalico [Discord server](#discord)

If you have a general question or are experiencing general printing
problems, then also consider a general 3d-printing forum or a forum
dedicated to the printer hardware.

## I have a feature request

All new features require someone interested and able to implement that
feature. If you are interested in helping to implement or test a new
feature, you can search for ongoing developments on the 
[GitHub issues](https://github.com/KalicoCrew/kalico/issues) page and 
[pull requests](https://github.com/KalicoCrew/kalico/pulls) page

There also are discussions between collaborators on the Kalico [Discord server](#discord).

## Help! It doesn't work!

If you are experiencing problems we recommend you carefully read the
[Overview](Overview.md) and double check that all steps
were followed.

If you are experiencing a printing problem, then we recommend
carefully inspecting the printer hardware (all joints, wires, screws,
etc.) and verify nothing is abnormal. We find most printing problems
are not related to the Kalico software. If you do find a problem with
the printer hardware then consider searching general 3d-printing
forums or forums dedicated to the printer hardware.

## I found a bug in the Kalico software

Kalico is an open-source project and we appreciate when collaborators
diagnose errors in the software.

Problems should be reported on the [Discord server](#discord)

There is important information that will be needed in order to fix a
bug. Please follow these steps:
1. Make sure you are running unmodified code from
   [https://github.com/KalicoCrew/kalico](https://github.com/KalicoCrew/kalico).
   If the code has been modified or is obtained from another source,
   then you should reproduce the problem on the unmodified code from
   [https://github.com/KalicoCrew/kalico](https://github.com/KalicoCrew/kalico)
   prior to reporting.
2. If possible, run an `M112` command immediately after the
   undesirable event occurs. This causes Kalico to go into a
   "shutdown state" and it will cause additional debugging information
   to be written to the log file.
3. Run `CREATE_SUPPORT_BUNDLE` after the failure. It creates a
   `serval-support-<timestamp>.tar.gz` archive in the same directory as
   `klippy.log`. The default bundle covers the previous 30 minutes; use
   `CREATE_SUPPORT_BUNDLE SINCE=2h` when the failure happened earlier.
   The archive includes the text log, structured host and MCU events, and the
   Klipper service journal when available.
   1. Download the archive from the logs page in the printer's web interface.
      Otherwise, copy it from `~/printer_data/logs/` with an `scp` or `sftp`
      utility.
   2. Attach the complete, unmodified archive to the issue report. It may
      contain printer configuration, file names, and diagnostic data.
   3. On older versions without `CREATE_SUPPORT_BUNDLE`, attach the complete
      unmodified `~/printer_data/logs/klippy.log` file instead.
4. Open a new thread on the [Discord server](#discord)
   and provide a clear description of the problem. Other Kalico
   contributors will need to understand what steps were taken, what
   the desired outcome was, and what outcome actually occurred. The
   support bundle should be attached to that topic.

## I am making changes that I'd like to include in Kalico

Kalico is open-source software and we appreciate new contributions.

See the [CONTRIBUTING document](CONTRIBUTING.md) for information.

There are several
[documents for developers](Overview.md#developer-documentation). If
you have questions on the code then you can also ask on the [Discord server](#discord)
