; the same square traced twice: corner_deviation promises identical corner
; rounding regardless of acceleration, so both passes must overlap exactly
G1 X0 Y0 F12000
SET_VELOCITY_LIMIT ACCEL=1000
G1 X20 Y0
G1 X20 Y20
G1 X0 Y20
G1 X0 Y0
SET_VELOCITY_LIMIT ACCEL=25000
G1 X20 Y0
G1 X20 Y20
G1 X0 Y20
G1 X0 Y0
