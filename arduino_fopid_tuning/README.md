# arduino_fopid_tuning

Standalone fractional-order PID (PI^lambda D^mu) tuning tool. This is a
**separate sketch** from both `arduino_ros2_base_controller/arduino_ros2_base_controller.ino`
and `arduino_motor_calibration/arduino_motor_calibration.ino` -- it does not
include, import, or modify either, so nothing about the production
controller changes just by running this.

Purpose: hand-tune wheel-speed PID gains with the PID loop actually
engaged, targeting **odometry accuracy** specifically -- not just how well
one wheel tracks its own commanded speed, but how closely all four wheels
match each other when commanded the same speed. A robot where every wheel
is individually accurate but mismatched relative to the others still veers
off a straight line, and that mismatch is what corrupts odometry over
distance. Two trial types:

- **STEP** -- single-motor closed-loop dynamic step response (PID engaged).
  Directly comparable to `arduino_motor_calibration`'s `OPENLOOP` trial at
  the same target speed: `steady_err_mm_s` here should be much smaller than
  `OPENLOOP`'s feed-forward-only baseline once gains are reasonable.
- **MATCH** -- all four motors driven closed-loop at the same target speed
  simultaneously. Reports each wheel's steady-state speed plus cross-wheel
  mismatch (`left_right_diff_mm_s`, `max_pair_diff_mm_s`, `spread_mm_s`) --
  these are the numbers that predict odometry drift, and the main thing to
  minimize.
- **ROTATE** -- single-motor *position* move (not speed) to a target angle,
  e.g. one full turn. Meant to be watched by eye: mark the wheel, run it,
  see how far past the mark it actually stops. The motor is released (not
  braked) at the end specifically so you can hand-correct it back to the
  mark and re-run with adjusted gains -- a visual, hands-on check that
  catches both PID overshoot and encoder tick-per-rev calibration error at
  once (see the command doc below for how to tell them apart).
- **AUTOTUNE** -- automatic twiddle (coordinate-descent) search over one
  motor's `kp`/`ki`/`kd`/`lambda`/`mu`, evaluating each candidate with a
  silent `STEP` trial (no per-tick output) and minimizing the same cost
  `STEP_RESULT` reports. **Staged, not interleaved**: `kp` is twiddled to
  convergence before `ki` is touched at all, then `kd`, then `lambda`, then
  `mu` -- mirrors the classic manual "raise kp first" procedure. Runs
  unattended for many trials and leaves the best-found gains loaded when
  done.

`PID`/`PIDM`/`POSGAIN` set gains by hand; `STEP`/`MATCH`/`ROTATE` report
what happened for you to act on; `AUTOTUNE` searches automatically for one
motor at a time, using `STEP`'s own cost as the objective.

## Hardware

QGPMaker motor shield + four QGPMaker quadrature encoders. No IMU is used
by this tool -- the odometry-accuracy proxy here is cross-wheel speed
matching, not IMU-measured heading drift. Serial: 500000 baud, 8-N-1.
Commands are ASCII lines terminated by `\n`, `\r` or `:`.

**SAFETY: lift all four wheels off the ground before running STEP, MATCH,
ROTATE, or AUTOTUNE.** MATCH drives all four wheels at once -- on the
ground the robot will crawl forward at full commanded speed. AUTOTUNE runs
one motor unattended through many trials in a row; `STOP`/`ESTOP` still
interrupts it mid-search, but don't assume a bad gain combination can't
occur just because it's automated. Motor 4 has noticeably
harder/higher-torque gearing than the other three and keeps drifting for
longer after RELEASE, so STEP/MATCH/AUTOTUNE brake (not release) between
runs. ROTATE is the one exception: it releases (not brakes) when it
finishes, on purpose, so you can hand-correct the wheel back to its mark.

## Commands

### `PID,<kp>,<ki>,<kd>[,<lambda>,<mu>]`

Sets the same fractional-order PID gains on all four motors. `lambda`/`mu`
default to `1.0` (classic PID) when omitted.

### `PIDM,<motor 1-4>,<kp>,<ki>,<kd>[,<lambda>,<mu>]`

Sets gains on one motor only.

### `GAINS`

Prints each motor's current `kp`/`ki`/`kd`/`lambda`/`mu`:

```text
GAINS,motor=<n>,kp=...,ki=...,kd=...,lambda=...,mu=...
```

### `STEP,<motor 1-4>[,target_mm_s[,duration_ms]]`

Single-wheel closed-loop dynamic step trial at `target_mm_s`. A negative
`target_mm_s` runs the step in reverse (`SAMPLE` speeds print signed;
`STEP_RESULT` metrics stay in magnitude terms so forward and reverse runs
compare directly). Defaults: `target_mm_s=220`, `duration_ms=1200`. Only
the chosen motor moves.

Output per control tick:

```text
SAMPLE,trial=<n>,motor=<n>,t_ms=...,target_mm_s=...,measured_mm_s=...,
  error_mm_s=...,ff_pwm=...,correction_pwm=...,pwm=...
```

Summary line:

```text
STEP_RESULT,trial=<n>,motor=<n>,target_mm_s=...,rise_s=...,
  overshoot_pct=...,steady_err_mm_s=...,cost=...
```

Ends with `INFO,STEP_DONE,motor=<n>`.

### `MATCH[,target_mm_s[,duration_ms]]`

Runs all four motors closed-loop at the same `target_mm_s` simultaneously.
A negative `target_mm_s` runs the trial in reverse -- this is the check
for backward straight-line driving, which has its own per-motor
feed-forward curve (`FF_*_REV`) because brushed gearmotors are
direction-asymmetric; a robot can MATCH well forward and still drift badly
in reverse until the reverse curves are calibrated. Speeds print signed
(all negative on a reverse trial); the mismatch metrics read the same way.
Defaults: `target_mm_s=220`, `duration_ms=1200`. **All four motors move --
wheels must be off the ground.**

Output per control tick:

```text
SAMPLE,trial=<n>,t_ms=...,target_mm_s=...,
  m1_mm_s=...,m2_mm_s=...,m3_mm_s=...,m4_mm_s=...,
  m1_pwm=...,m2_pwm=...,m3_pwm=...,m4_pwm=...
```

Summary line:

```text
MATCH_RESULT,trial=<n>,target_mm_s=...,
  m1_ss_mm_s=...,m2_ss_mm_s=...,m3_ss_mm_s=...,m4_ss_mm_s=...,
  m1_err_mm_s=...,m2_err_mm_s=...,m3_err_mm_s=...,m4_err_mm_s=...,
  left_avg_mm_s=...,right_avg_mm_s=...,left_right_diff_mm_s=...,
  max_pair_diff_mm_s=...,spread_mm_s=...
```

`left_right_diff_mm_s = left_avg - right_avg` (signed -- positive means the
left side is running fast relative to the right, which would curl the
robot to the right, i.e. clockwise). `spread_mm_s` is `max - min` across
all four steady-state speeds. Both are the numbers to drive toward zero.
Ends with `INFO,MATCH_DONE`.

Motor order/sides (matches `arduino_ros2_base_controller.ino`): 1
front-left, 2 front-right, 3 rear-right, 4 rear-left. Left = motors 1 & 4,
right = motors 2 & 3.

### `POSGAIN[,<kp_pos>[,max_speed_mm_s]]`

With no args, prints the current `ROTATE` position-loop gain and speed cap.
With args, sets them. Defaults: `kp_pos=3.0`, `max_speed_mm_s=150`.

```text
POSGAIN,kp_pos=...,max_speed_mm_s=...
```

`kp_pos` converts position error (mm of wheel travel) into a speed
setpoint (mm/s) for `ROTATE`'s outer loop -- higher values approach the
target faster but decelerate later (more overshoot risk); lower values are
gentler but slower to settle.

### `ROTATE,<motor 1-4>[,target_deg[,timeout_ms]]`

Position-control move: rotates one wheel by `target_deg`, using a
proportional outer position loop (gain `kp_pos`, from `POSGAIN`) wrapped
around the same per-motor speed `WheelPID` used by `STEP`/`MATCH` -- the
speed setpoint shrinks (decelerating the wheel) as it nears the target
angle, capped at `max_speed_mm_s`. Defaults: `target_deg=360` (one full
turn), `timeout_ms=3000`. Only the chosen motor moves.

**Mark the wheel before running this** (tape, paint pen, whatever's
visible) so you can see where it actually stops relative to where it
started.

Output per control tick:

```text
SAMPLE,trial=<n>,motor=<n>,t_ms=...,target_deg=...,position_deg=...,
  error_deg=...,speed_setpoint_mm_s=...,ff_pwm=...,correction_pwm=...,pwm=...
```

Summary line:

```text
ROTATE_RESULT,trial=<n>,motor=<n>,target_deg=...,final_deg=...,
  overshoot_deg=...,settle_s=...,settled=<0_or_1>
```

`overshoot_deg` is what the *encoder* measured, always >= 0. Compare it
against what you see by eye:

- Encoder and eye roughly agree -> real PID/position-loop overshoot. Lower
  `kp_pos` (gentler approach) or check `kp`/`ki` on that motor via `STEP`.
- Encoder says small/no overshoot but the wheel visibly stopped well past
  (or short of) the mark -> that gap is `MEASURED_COUNTS_PER_REV`
  calibration error for that motor, not a PID problem -- re-derive it by
  hand-rotating the wheel exactly one turn and comparing tick counts (see
  `arduino_ros2_base_controller.ino`'s comments on that constant).

`settled=0` means it hit `timeout_ms` without holding inside tolerance
(~3 degrees) for 150ms straight -- raise `timeout_ms` or lower `kp_pos` if
this happens often. Ends with `INFO,ROTATE_DONE,motor=<n>`, motor released
so you can hand-correct it.

### `AUTOTUNE,<motor 1-4>[,target_mm_s[,max_iterations]]`

Twiddle (coordinate-descent) search over one motor's
`kp`/`ki`/`kd`/`lambda`/`mu`, starting from whatever's currently configured
for that motor. Each candidate is evaluated with a silent `STEP` trial
(same trial, same cost formula, just no per-tick `SAMPLE` output) at
`target_mm_s`. Defaults: `target_mm_s=220`, `max_iterations=10`.

Bounds (won't search outside these): `kp` in `[0.5, 1.0]`, `ki` in
`[0.0, 0.05]`, `kd` in `[0.0, 0.02]`, `lambda`/`mu` in `[0.4, 1.4]`
(lambda/mu range matches the original, pre-removal AUTOTUNE search --
stays clear of the degenerate order-0 case and the order-2 stability edge).

**Staged, not interleaved**: `kp` is twiddled first, up to `max_iterations`
steps -- or fewer, if its own step size shrinks below its convergence
threshold first -- before `ki` is touched at all. Then `kd`, then
`lambda`, then `mu`, each fully staged the same way, mirroring the classic
manual "raise kp first, then ki, then kd" procedure rather than nudging
all five gains a little each round. Each step costs up to 2 `STEP` trials
(~2.2s/trial including the rest hold), so `max_iterations=10` means up to
20 trials per stage worst-case (100 trials across all 5 stages) -- several
minutes total, often less since a stage can converge and move on early.

Output per candidate:

```text
AUTOTUNE_EVAL,motor=<n>,iter=<i>,param=<kp|ki|kd|lambda|mu>,
  kp=...,ki=...,kd=...,lambda=...,mu=...,cost=...,best_cost=...
```

Summary line:

```text
AUTOTUNE_RESULT,motor=<n>,kp=...,ki=...,kd=...,lambda=...,mu=...,cost=...
```

Ends with `INFO,AUTOTUNE_DONE,motor=<n>`. The winning gains are already
loaded into that motor's `WheelPID` when this finishes -- run `STEP`
immediately after to see the result for yourself, or `GAINS` to just read
the numbers back. AUTOTUNE optimizes one motor in isolation (same cost
`STEP` uses) -- it does **not** know about cross-wheel matching, so still
run `MATCH` afterward; a motor that autotunes to a great individual
`STEP_RESULT` can still be a bad match for its neighbors.

### `STOP` / `ESTOP`

Aborts any running trial and releases all motors.

### `PING`

Replies `PONG`.

## Tuning procedure

1. **Lift all four wheels off the ground.**
2. Per motor, either run `AUTOTUNE,<motor>,<target_mm_s>` to search
   automatically (leaves the best gains loaded, then confirm with
   `STEP,<motor>,<target_mm_s>`), or set gains by hand: `PIDM,<motor>,<kp>,
   <ki>,<kd>[,<lambda>,<mu>]`, then `STEP,<motor>,<target_mm_s>`. Either
   way, compare `steady_err_mm_s` against `arduino_motor_calibration`'s
   `OPENLOOP` baseline at the same `target_mm_s` -- PID should shrink it
   close to zero without introducing much overshoot/ringing. If hand-tuning:
   raise `kp` first, add a small `ki` to kill remaining offset, add `kd`
   only if you see overshoot `ki` alone doesn't settle.
3. Mark that same wheel and run `ROTATE,<motor>,360` as a visual spot-check.
   Watch where it actually stops relative to the mark. If it overshoots (by
   eye and by `overshoot_deg` agreeing), nudge `kp`/`ki` down slightly (or
   lower `POSGAIN`'s `kp_pos`) and re-run `STEP` and `ROTATE` again on that
   motor until both look right. If `overshoot_deg` is small but the eye
   disagrees, that's `MEASURED_COUNTS_PER_REV` calibration error -- see the
   `ROTATE` command doc above. Repeat steps 2-3 for all four motors.
4. Once all four look reasonable in isolation, run
   `MATCH,<target_mm_s>` at your typical operating speed. Then run
   `MATCH,-<target_mm_s>` for the reverse direction -- reverse has its own
   feed-forward curve (`FF_*_REV`, fit from `arduino_motor_calibration`'s
   `CALIBRATE,...,R` sweeps), and a robot that matches well forward can
   still drift in reverse if those curves haven't been calibrated yet.
5. Read `left_right_diff_mm_s` and `spread_mm_s`. If one side is
   consistently faster, raise `kp`/`ki` slightly on the slower side's
   motor(s) (or lower it on the faster side) and re-run `MATCH` -- small
   steps, since these loops interact once running together. Iterate until
   both numbers are as close to zero as the hardware allows.
6. Use `GAINS` any time to confirm what's currently loaded before
   recording a result.
7. Once `MATCH` and `ROTATE` both look good, copy each motor's final
   `kp`/`ki`/`kd`/`lambda`/`mu` into `arduino_ros2_base_controller.ino`'s
   `setup()` (`wheelPID[i].configure(...)` calls). That sketch is the only
   place these gains actually drive the real robot.

## Relationship to the other two sketches

- `arduino_motor_calibration.ino` -- PID-free. Fits the feed-forward curve
  (`CALIBRATE`) and measures open-loop dynamic response (`OPENLOOP`). Do
  that first; this tool's feed-forward constants must match its result.
- `arduino_fopid_tuning.ino` (this sketch) -- PID engaged, used to search
  for gains by hand and check cross-wheel matching before touching
  production.
- `arduino_ros2_base_controller.ino` -- production. Runs the real FOPID
  loop continuously on live `/cmd_vel` commands; gains found here get
  pasted into its `setup()`.

Keep the feed-forward constants (`FF_SLOPE_MM_S_PER_PWM_FWD/REV` +
`FF_INTERCEPT_MM_S_FWD/REV`), encoder calibration
(`MEASURED_COUNTS_PER_REV`), and the `WheelPID` implementation itself in
sync across all three sketches manually -- there's no shared header
between them.
