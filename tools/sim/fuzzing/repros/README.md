# Scenario-fuzzer repros of open product bugs

One directory per GitHub issue: the minimized scenario (`scenario.json`,
`timeline.txt`) the scenario fuzzer saved. Each still fails, by design, so
none is in the suite; a fix PR moves its directory into the regression set
(a sim test that replays it and expects a pass) and deletes it here.

Replay one (the image copies the tree, run.sh maps the path):

```
tools/sim/run.sh fuzz --replay tools/sim/fuzzing/repros/issue-893
tools/sim/run.sh fuzz --replay tools/sim/fuzzing/repros/issue-* --repeat 3
```

| issue | replay | expected failure |
|---|---|---|
| #893 | `tools/sim/run.sh fuzz --replay tools/sim/fuzzing/repros/issue-893` | pin_order: motors move within 0.3 s of the mark pin, inside the `G4 P1500` after it |
| #895 | `tools/sim/run.sh fuzz --replay tools/sim/fuzzing/repros/issue-895` | panic `crawl_sides.rs:347` "the leaders ask N mm/s² of the follower" |
| #896 | `tools/sim/run.sh fuzz --replay tools/sim/fuzzing/repros/issue-896` | panic `ratio_ramp.rs` assert_weight_continuous "the advance valley ... steps its weight" |
| #897 | `tools/sim/run.sh fuzz --replay tools/sim/fuzzing/repros/issue-897` | panic `crawl_sides.rs:568` assert_flow_fits "the ratio steps ... ask N mm/s² of the follower's flow" |
| #898 | `tools/sim/run.sh fuzz --replay tools/sim/fuzzing/repros/issue-898` | panic `ratio_ramp.rs` place_valley "the crawl valley ... the projection has taken the path" |
| #902 | `tools/sim/run.sh fuzz --replay tools/sim/fuzzing/repros/issue-902` | panic `turning_guard.rs` "the cell at t=Ns still asks N mm/s² of the extruder" |
| #903 | `tools/sim/run.sh fuzz --replay tools/sim/fuzzing/repros/issue-903` | panic `turning_guard.rs` "the crawl ... still lays a valley side" (4 travel moves, no E) |
| #929 | `tools/sim/run.sh fuzz --replay tools/sim/fuzzing/repros/issue-929` | fatal_event `anchor_underrun` after PAUSE + jog; klippy exits |
| #932 | `tools/sim/run.sh fuzz --replay tools/sim/fuzzing/repros/issue-932` | panic `shaper.rs` "advanced_follower_source span ... neither exact polynomials nor an enclosure" |
| #940 | `tools/sim/run.sh fuzz --replay tools/sim/fuzzing/repros/issue-940 --repeat 10` | intermittent: pacer_brake `RunwayExhausted` at print start; not minimized |
| #993 | `tools/sim/run.sh fuzz --replay tools/sim/fuzzing/repros/issue-993` | idle_timeout: `state 'Printing', extruder target 150.0` 8.5 s after the second print, which followed a 40 s idle gap |
