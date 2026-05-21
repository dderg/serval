; Oracle input 04: right-angle corner — exercises the planner's
; junction-deviation cornering logic, where lookahead matters.
G90
SET_KINEMATIC_POSITION X=0 Y=0 Z=0
G1 X30 Y0 F6000
G1 X30 Y30 F6000
M400
M84
