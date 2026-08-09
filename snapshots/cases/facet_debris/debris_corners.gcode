; A 30mm square whose corners carry different flavors of slicer debris:
;   corner 1: single 50um 45-deg chamfer      -> one-pair consumption
;   corner 2: two 60um facets at 30/30/30     -> one pair consumes both
;   corner 3: 60um + 100um facets at 35/10/45 -> split blend (two pairs)
;   corner 4: single 400um chamfer            -> roomy: keeps pairwise blends
M83 ; relative extrusion
G1 X10 Y10
G1 F9000
G1 X40.000000 Y10.000000 E1.500000
G1 X40.035355 Y10.035355 E0.002500
G1 X40.035355 Y40.035355 E1.500000
G1 X40.005355 Y40.087317 E0.003000
G1 X39.953394 Y40.117317 E0.003000
G1 X9.953394 Y40.117317 E1.500000
G1 X9.904245 Y40.082902 E0.003000
G1 X9.833534 Y40.012192 E0.005000
G1 X9.833534 Y10.012192 E1.500000
G1 X10.116377 Y9.729349 E0.020000
G1 X20.116377 Y9.729349 E0.500000
