; Oracle input 01: simplest possible non-zero motion.
; A single X-axis jog. This is the case the fork currently breaks on
; (jog issued; no motors move on bench).
;
; NOTE: G28 is intentionally omitted. klipper-sim's batch-mode klippy has
; no endstops attached, and a G28 in that environment becomes a no-op
; trajectory that walks the steppers for ~268 s of wallclock-equivalent
; trajectory time, completely swamping the actual G1 in the CSV output.
; SET_KINEMATIC_POSITION forces the toolhead to a known origin without
; physically homing.
G90
SET_KINEMATIC_POSITION X=0 Y=0 Z=0
G1 X10 F600
M400
M84
