# arduino_motor_calibration

Standalone per-wheel raw calibration data tool. This is a **separate sketch**
from `arduino_ros2_base_controller/arduino_ros2_base_controller.ino` -- it
does not include, import, or modify that sketch, so nothing about the
production controller changes just by running this.

Purpose: gather raw, PID-free per-motor data for manual analysis and
hand-tuning. No automatic gain search runs here (an earlier `AUTOTUNE`
twiddle search was removed -- see below). Two data sources:

- **CALIBRATE** -- steady-state PWM-to-speed sweep (open-loop, fixed PWM).
- **OPENLOOP** -- dynamic step response at a target speed (open-loop,
  feed-forward PWM only, zero PID correction).

Both drive motors directly off the feed-forward curve/fixed PWM with no
correction term, so the data reflects only real motor + feed-forward
behaviour -- nothing from a control loop can be masking or fixing it.

## Hardware

QGPMaker motor shield + four QGPMaker quadrature encoders. No IMU is used by
this tool. Serial: 500000 baud, 8-N-1. Commands are ASCII lines terminated by
`\n`, `\r` or `:`.

**SAFETY: lift all four wheels off the ground before running CALIBRATE or
OPENLOOP.** Each trial repeatedly drives one wheel; on the ground the robot
will crawl into whatever is in front of it.

## Commands

### `CALIBRATE,<motor 1-4>[,<pwm_step>[,<hold_ms>]]`

Open-loop (no PID) PWM sweep from `MIN_EFFECTIVE_PWM` to `MAX_DRIVE_PWM` on
one motor, step size `pwm_step`, holding each step for `hold_ms` and
measuring steady-state speed from the encoder. Reports each step's speed
against what that motor's own calibrated feed-forward curve
(`FF_SLOPE_MM_S_PER_PWM`/`FF_INTERCEPT_MM_S`, used by `speedFeedForwardPWM`)
predicts for that PWM, so any remaining gap -- e.g. from the real response
saturating at high PWM, which a straight-line fit doesn't capture -- is
visible directly.

Defaults: `pwm_step=10`, `hold_ms=1000`. Only the chosen motor moves.

Output per step:

```text
CAL,motor=<n>,pwm=...,measured_mm_s=...,expected_mm_s=...,error_mm_s=...,error_pct=...
```

Ends with `INFO,CALIBRATE_DONE,motor=<n>`.

### `OPENLOOP,<motor 1-4>[,<target_mm_s>]`

Runs one dynamic step test at `target_mm_s`, feed-forward PWM only
(correction forced to 0 -- no PID involved at all). Defaults:
`target_mm_s=220`. Only the chosen motor moves.

Output, one line per 10ms control tick (raw step response, for offline
analysis/plotting):

```text
SAMPLE,trial=<n>,motor=<n>,t_ms=...,target_mm_s=...,measured_mm_s=...,
  error_mm_s=...,ff_pwm=...,correction_pwm=0.0,pwm=...
```

`trial=` matches the `trial=` field on the summary line printed right after
it, so a raw response stream can be joined back to the run it belongs to.
`correction_pwm` is always 0 here (kept in the output for format parity with
a PID-driven trial); `pwm` is just the clamped `ff_pwm`.

Summary line:

```text
OPENLOOP_RESULT,trial=<n>,motor=<n>,target_mm_s=...,rise_s=...,
  overshoot_pct=...,steady_err_mm_s=...,cost=...
```

Ends with `INFO,OPENLOOP_DONE,motor=<n>`.

### `STOP` / `ESTOP`

Aborts any running trial and releases all motors.

### `PING`

Replies `PONG`.

## Calibration procedure

1. **Lift all four wheels off the ground.**
2. Run `CALIBRATE,<motor>,10,1000` for each of the 4 motors, one at a time.
3. Fit each motor's own linear model from the `measured_mm_s`/`pwm` pairs:
   `measured_mm_s = slope*pwm + intercept`. Update
   `FF_SLOPE_MM_S_PER_PWM`/`FF_INTERCEPT_MM_S` in **both**
   `arduino_motor_calibration.ino` and `arduino_ros2_base_controller.ino`
   (they must be kept in sync manually -- there's no shared header between
   the two independent sketches).
4. Re-run `CALIBRATE` on all 4 motors again. `expected_mm_s` now reflects the
   new fit, so `error_mm_s`/`error_pct` show you the residual directly --
   confirm it's small and stable before moving on. If a second independent
   run shows the same residual pattern and magnitude, that's real motor
   nonlinearity (not measurement noise) and refitting further won't help
   much.
5. Run `OPENLOOP,<motor>,220` (or whatever your typical operating speed is)
   on each of the 4 motors. This is your open-loop dynamic baseline --
   record `rise_s`, `overshoot_pct`, `steady_err_mm_s` per motor.
6. Move to `arduino_ros2_base_controller.ino` and hand-tune PID per motor
   (see below), using step 5's numbers as the "before PID" reference to
   compare closed-loop behaviour against.

## Worked example (2026-07-04)

This is real data from this robot, kept here as a concrete reference for
what "good" looks like and how to read the output. Numbers will differ on
different hardware/wiring/gearing -- don't copy these coefficients onto a
different robot.

### Feed-forward coefficients derived from CALIBRATE

Two independent `CALIBRATE,<motor>,10,1000` runs (PWM 48-208, then 50-210)
gave consistent linear fits:

| Motor | Slope (mm/s per PWM) | Intercept | Residual RMSE |
|---|---|---|---|
| 1 | 2.557 | -83.25 | ~15-25 mm/s |
| 2 | 2.631 | -143.56 | ~20-22 mm/s |
| 3 | 2.4945 | -149.38 | ~20-21 mm/s |
| 4 | 2.3404 | 6.14 | ~26-27 mm/s |

A single **shared** linear curve (the original design) was off by 50-500% --
motor 4 needs less than half the PWM of motors 2/3 for the same speed. The
per-motor fit above is a large improvement; the remaining ~15-27 mm/s
residual is real response nonlinearity (a soft start at low PWM, mild
saturation at high PWM) that a straight-line model can't capture. Refitting
from a second independent run barely moved these numbers, confirming it's
real motor behaviour, not run-to-run noise.

### OPENLOOP baseline at 220 mm/s (feed-forward only, zero PID)

| Motor | ff_pwm | rise_s | overshoot_pct | steady_err_mm_s | cost |
|---|---|---|---|---|---|
| 1 | 118.6 | 0.331 | 15.5% | -27.2 | 243.1 |
| 2 | 138.2 | 0.492 | 23.5% | -36.2 | 334.8 |
| 3 | 148.1 | 0.451 | 7.7% | -11.0 | 110.0 |
| 4 | 91.4 | 0.452 | 17.7% | -30.4 | 274.4 |

All four motors drift *above* the 220 mm/s target under feed-forward alone
(consistent with the positive-residual bump in the CALIBRATE fit around this
PWM range) -- this is what PID has to correct. Motor 3 is the easiest
(smallest gap, lowest cost); motor 2 is the hardest (slowest rise, largest
drift, worst cost). This ranking is a useful prior for how aggressively each
motor's `kp` will need to be pushed during hand-tuning.

## Hand-tuning PID on the production controller

Once the feed-forward and OPENLOOP baseline above look right, PID gains are
tuned on `arduino_ros2_base_controller.ino`, not here (this sketch has no
PID controller at all -- see next section). Suggested procedure per motor:

1. Set `PIDM,<motor>,<kp>,0,0` (ki=kd=0, P-only).
2. Command that motor via `FORWARD,<target_mm_s>` and watch the `T,...`
   telemetry line's `speed<n>` field.
3. Raise `kp` until steady-state error shrinks close to zero without
   oscillating. Motors with a bigger OPENLOOP gap (e.g. motor 2 above) will
   likely need a higher `kp` than motors with a small gap (e.g. motor 3).
4. Add a small `ki` to kill any remaining steady-state offset.
5. Add `kd` only if you see overshoot/ringing that `ki` alone doesn't settle.
6. Compare the closed-loop response against this sketch's OPENLOOP numbers
   for the same motor/target -- it should settle much faster and tighter.

## History: AUTOTUNE removal

This sketch originally included an `AUTOTUNE` command (twiddle/coordinate-
descent search over kp, ki, kd, lambda, mu, using a fractional-order
`WheelPID`) and a `MATCH` command (four-motor group speed-matching search).
Both were removed:

- `MATCH` was dropped because a single-motor search was judged more useful
  than a synchronized four-motor one.
- `AUTOTUNE` was dropped in favor of hand-tuning: no PID controller runs in
  this sketch anymore, and `OPENLOOP` replaced it as a way to still get a
  dynamic (not just steady-state) view of motor behaviour, minus the search.

If you're looking for the fractional-order PID implementation itself, it
only exists in `arduino_ros2_base_controller.ino` now.

**Note on `CalibrationTypes.h`**: this header defines `TrialResult` and
`CalStepResult`, which `OPENLOOP` and `CALIBRATE` both require to compile,
even though no PID/AUTOTUNE exists in this sketch. It was previously named
`AutotuneTypes.h` and got deleted by mistake more than once on the
assumption it was autotune leftover cruft -- it was renamed specifically to
stop that. Don't delete it.
