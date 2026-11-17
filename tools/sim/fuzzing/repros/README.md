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
| #897 | `tools/sim/run.sh fuzz --replay tools/sim/fuzzing/repros/issue-897` | panic `crawl_sides.rs:568` assert_flow_fits "the ratio steps ... ask N mm/s² of the follower's flow" |
| #902 | `tools/sim/run.sh fuzz --replay tools/sim/fuzzing/repros/issue-902` | panic `turning_guard.rs` "the cell at t=Ns still asks N mm/s² of the extruder" |
| #903 | `tools/sim/run.sh fuzz --replay tools/sim/fuzzing/repros/issue-903` | panic `turning_guard.rs` "the crawl ... still lays a valley side" (4 travel moves, no E) |
| #988 | `tools/sim/run.sh fuzz --replay tools/sim/fuzzing/repros/issue-988` | panic `ratio_ramp.rs:2462` "the crawl valley ... reaches X mm back, but the projection has taken the path to Y mm" with X − Y ≈ 1e-13 |
