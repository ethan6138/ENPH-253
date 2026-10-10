# Drivetrain Tuning Guide

ALL FROM CLAUDE i sent in my chicken scratch code and it figured it out for me
 
## Tuning WheelPID
 
Use `testDriveBlindRpmAccuracy()` for this — fixed target RPM, watching `errLeft`/`errRight` converge.
 
1. Set `kd = 0` in `WheelPID.cpp`. Leave `kp` at your current starting value (2.0).
2. Flash, run the test, watch the error columns. Look at how `errLeft`/`errRight` behave from a cold start (motor at 0) up to steady state.
3. If response is sluggish (takes a long time to approach 0, error shrinks very slowly) — **raise `kp`** (try roughly doubling: 2.0 → 4.0), reflash, retest.
4. Keep raising `kp` until you see the error overshoot past 0 and swing to the opposite sign before settling (e.g., measured RPM shoots past target, dips below, etc.) — that's the oscillation threshold.
5. Back `kp` off slightly from that threshold — maybe 70–80% of the value where oscillation started. This is roughly your best P-only response.
6. Now bring `kd` up from 0 (try 0.25 × your final `kp` as a starting point). Reflash, retest.
7. If overshoot is still visible, raise `kd` further — small steps, since D is sensitive to noise. Watch specifically whether the approach to target becomes smoother (less overshoot) without introducing jitter/twitchiness once it's near steady-state.
8. If the reading looks jittery/noisy near steady-state (not overshooting, just twitchy) — `kd` is too high, back it off.
9. Repeat steps 3–8, alternating small `kp`/`kd` adjustments, until you get: fast approach to target, minimal/no overshoot, stable once settled.
10. Test at multiple target RPMs (low, medium, near-max) — confirm the tuning holds reasonably well across your actual operating range, not just the one value you tuned at. If it behaves very differently at different speeds, that's a sign your feedforward table needs more data points (run the characterization sweep if you haven't yet).
---
 
## Tuning TapeFollow
 
Use `testDriveFollowTape()` for this — needs the robot physically over tape, watching how it steers as tape position shifts under the sensors.
 
1. Set `kd = 0` in `TapeFollow.cpp`. Leave `kp` at your starting value (8.0).
2. Place tape centered under the sensors, then slowly slide it left/right by hand (or physically move the robot off-center) while watching `targetLeft`/`targetRight` from the serial output.
3. Confirm direction is correct first — if you shift tape to the robot's left, the robot should steer left to follow it (i.e., `targetLeft` should decrease relative to `targetRight`, or vice versa depending on your sign convention — just confirm it corrects *toward* the tape, not away). If it's backwards, that's the `TAPE_DETECTED_LEVEL` polarity issue — flip `HIGH`/`LOW` and retest before tuning anything else.
4. Once direction is confirmed correct, let the robot actually run on tape (real driving, not just holding tape under stationary sensors) and watch behavior: does it track smoothly, or does it oscillate side-to-side across the tape?
5. If it barely reacts to being off-center — raise `kp`.
6. If it swings hard, overshoots the tape, and oscillates back and forth — that's the oscillation threshold, same as before. Back `kp` off slightly from where oscillation starts.
7. Bring `kd` up from 0 to dampen any remaining overshoot — same small-step approach as WheelPID.
8. Test on a curve, not just a straight line — tape following behavior often looks fine on straights but reveals tuning issues on turns (either cutting corners too sharply or overshooting the curve and re-catching the line late).
9. Test the "lost tape" recovery — deliberately move the robot off tape entirely, confirm it steers hard back in the correct last-known direction and reacquires the line, rather than spinning the wrong way or not reacting at all.
10. Once WheelPID and TapeFollow are each individually reasonable, test them together at realistic driving speed — a WheelPID that looks great tracking a fixed RPM target might behave differently under TapeFollow's constantly changing targets, since now the inner loop never gets to fully settle before the outer loop moves the goalposts again. If oscillation reappears only in this combined test, you may need to back off gains slightly from your individually-tuned values.
