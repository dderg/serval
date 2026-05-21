; Oracle input 03: diagonal XY jog — exercises CoreXY mixing
; on both A and B steppers simultaneously.
G90
SET_KINEMATIC_POSITION X=0 Y=0 Z=0
G1 X20 Y20 F3000
M400
M84
