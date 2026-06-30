# Mcu clock syncronisation

Problem: Clocks on different chips drift relative to each other. If we don't do anything about it, axis driven by different mcus will get out of sync. (or if we simply queue next piece at the position of previous piece end). Often it can be just some jitter back and forth. And sometimes clock just have different speeds.


Current solution (2026-06-30): To simply shift the scheduled time start for the next piece by the drift duration. This introduces a problem of inconsistent motion, every time the sync occurs, we either add a gap to the motion, or we overlap pieces, often triggering -308 error code. 


Proposed solution: To acknowledge the difference in speed of the clock, and to speed up or slow down the new pushed pieces. We should be careful with how we approach the gain of this time multiplier. so we don't hunt, but actually apply a meaningful correction. We should have a fail loud case for when we are not able to sync the axis properly and they drift too far apart. I suggest we store a timeMultiplier per mcu and update it when we sync the clock. We should keep in mind that we will only be able to tell if the sync is working after the adjusted pieces are consumed. since the effect is somewhat delayed. but the fail loud should be immediate, so we know we should tune the algorithm rather than silently try to compensate for way out of sync clocks.
