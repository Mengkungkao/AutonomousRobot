/*
  mDetect standalone FO-PID wheel-tuning tool

  This is a SEPARATE sketch from arduino_ros2_base_controller.ino and from
  arduino_motor_calibration.ino. It does not include, import, or modify
  either -- all hardware driver sources here are independent copies living
  only in this folder, so nothing about the production controller changes
  by running this.

  Purpose: hand-tune the fractional-order PID (PI^lambda D^mu) wheel-speed
  gains with the PID loop actually engaged, targeting ODOMETRY accuracy --
  not just how well one wheel tracks its own target, but how closely all
  four wheels match each other when commanded the same speed. A robot where
  every wheel is individually accurate but mismatched relative to the others
  still drifts off a straight line, which is what corrupts odometry over
  distance. Two trial types:

    STEP  -- single-motor closed-loop dynamic step response (PID engaged).
             Same shape as arduino_motor_calibration's OPENLOOP trial, so the
             two are directly comparable: STEP's steady_err_mm_s should be
             much smaller than OPENLOOP's feed-forward-only baseline once
             gains are reasonable.
    MATCH -- all four motors driven closed-loop at the same target speed at
             once. Reports each wheel's steady-state speed plus cross-wheel
             mismatch (left/right average difference, max pairwise
             difference, spread) -- this is the number that predicts
             odometry drift, and the main thing to minimize here.
    ROTATE -- single-motor POSITION move (not speed) to a target angle, e.g.
             one full turn. Meant to be watched by eye: mark the wheel,
             command ROTATE, see how far past the mark it actually stops.
             The motor is released (not braked) at the end specifically so
             you can hand-correct it back to the mark and re-run with
             adjusted gains -- this catches both PID overshoot and encoder
             tick-per-rev calibration error (MEASURED_COUNTS_PER_REV), since
             the two look identical in the SAMPLE/ROTATE_RESULT numbers but
             only calibration error will disagree with what your eye sees.
    AUTOTUNE -- automatic twiddle (coordinate-descent) search over one
             motor's kp/ki/kd/lambda/mu, using silent STEP trials (no
             per-tick SAMPLE spam) as the cost evaluator -- same cost
             formula STEP_RESULT reports, so an AUTOTUNE cost and a manual
             STEP cost mean the same thing. Runs unattended for many
             trials; leaves the best-found gains loaded when done.

  PID/PIDM/POSGAIN set gains by hand; STEP/MATCH/ROTATE report what
  happened for you to act on; AUTOTUNE searches kp/ki/kd/lambda/mu
  automatically for one motor at a time using STEP's cost as the objective.

  SAFETY: lift all four wheels off the ground before running STEP, MATCH,
  ROTATE, or AUTOTUNE. MATCH drives all four wheels at once -- on the
  ground the robot will crawl forward at full commanded speed. AUTOTUNE
  runs one motor unattended for many trials in a row -- don't leave the
  room assuming a bad gain combination can't happen; STOP/ESTOP still
  works mid-search. Motor 4 has noticeably harder/higher-torque gearing
  than the other three and keeps drifting for longer after RELEASE, so
  trials brake (not release) between runs.

  Hardware: QGPMaker motor shield + four QGPMaker quadrature encoders. No
  IMU is used by this tool -- the odometry-accuracy proxy used here is
  cross-wheel speed matching, not IMU-measured heading drift.

  Serial: 500000 baud, 8-N-1. Commands are ASCII lines terminated by
  \n, \r or ':'.

  Commands:
    PID,<kp>,<ki>,<kd>[,<lambda>,<mu>]
      Set the same fractional-order PID gains on all four motors. lambda/mu
      default to 1.0 (classic PID) when omitted.
    PIDM,<motor 1-4>,<kp>,<ki>,<kd>[,<lambda>,<mu>]
      Set gains on one motor only.
    GAINS
      Prints each motor's current kp/ki/kd/lambda/mu.
    STEP,<motor 1-4>[,target_mm_s[,duration_ms]]
      Single-wheel closed-loop dynamic step trial (PID engaged) at
      target_mm_s, printing one SAMPLE line per 10ms control tick plus a
      STEP_RESULT summary. A negative target_mm_s runs the step in
      reverse (SAMPLE speeds print signed; STEP_RESULT metrics stay in
      magnitude terms so forward and reverse runs compare directly).
      Defaults: target_mm_s=220, duration_ms=1200. Only the chosen motor
      moves.
    MATCH[,target_mm_s[,duration_ms]]
      Runs all four motors closed-loop at the same target_mm_s
      simultaneously, printing one combined SAMPLE line per control tick
      plus a MATCH_RESULT summary with cross-wheel mismatch metrics. A
      negative target_mm_s runs the trial in reverse -- use this to check
      straight-line matching for backward driving, which has its own
      feed-forward curve (brushed gearmotors are direction-asymmetric).
      Defaults: target_mm_s=220, duration_ms=1200. All four motors move --
      wheels MUST be off the ground.
    POSGAIN[,<kp_pos>[,max_speed_mm_s]]
      With no args, prints the current ROTATE position-loop gain and speed
      cap. With args, sets them. Defaults: kp_pos=3.0, max_speed_mm_s=150.
    ROTATE,<motor 1-4>[,target_deg[,timeout_ms]]
      Position-control move: rotates one wheel by target_deg (outer
      proportional position loop, gain kp_pos, wrapped around the same
      per-motor speed WheelPID as STEP/MATCH) and stops. Meant to be
      watched by eye -- mark the wheel first. Releases the motor at the
      end (not braked) so it can be hand-corrected back to the mark.
      Defaults: target_deg=360 (one full turn), timeout_ms=3000. Only the
      chosen motor moves.
    AUTOTUNE,<motor 1-4>[,target_mm_s[,max_iterations]]
      Twiddle search over that motor's kp/ki/kd/lambda/mu (starting from
      its currently configured gains), evaluating each candidate with a
      silent STEP trial at target_mm_s and minimizing the same cost
      STEP_RESULT reports. Prints one AUTOTUNE_EVAL line per trial plus a
      final AUTOTUNE_RESULT, and leaves the best gains loaded. Defaults:
      target_mm_s=220, max_iterations=10 (up to 10 candidates per
      parameter, 5 parameters -- can run for several minutes). Only the
      chosen motor moves.
    STOP / ESTOP
      Aborts any running trial and releases all motors.
    PING
      Replies PONG.

  Output during STEP (one line per control tick):
    SAMPLE,trial=<n>,motor=<n>,t_ms=...,target_mm_s=...,measured_mm_s=...,
      error_mm_s=...,ff_pwm=...,correction_pwm=...,pwm=...

  Output after STEP:
    STEP_RESULT,trial=<n>,motor=<n>,target_mm_s=...,rise_s=...,
      overshoot_pct=...,steady_err_mm_s=...,cost=...
    Ends with INFO,STEP_DONE,motor=<n>.

  Output during MATCH (one line per control tick):
    SAMPLE,trial=<n>,t_ms=...,target_mm_s=...,
      m1_mm_s=...,m2_mm_s=...,m3_mm_s=...,m4_mm_s=...,
      m1_pwm=...,m2_pwm=...,m3_pwm=...,m4_pwm=...

  Output after MATCH:
    MATCH_RESULT,trial=<n>,target_mm_s=...,
      m1_ss_mm_s=...,m2_ss_mm_s=...,m3_ss_mm_s=...,m4_ss_mm_s=...,
      m1_err_mm_s=...,m2_err_mm_s=...,m3_err_mm_s=...,m4_err_mm_s=...,
      left_avg_mm_s=...,right_avg_mm_s=...,left_right_diff_mm_s=...,
      max_pair_diff_mm_s=...,spread_mm_s=...
    left_right_diff_mm_s = left_avg - right_avg (signed -- positive means
    the left side is running fast relative to the right, which would curl
    the robot to the right). spread_mm_s = max - min across all four
    steady-state speeds. Both are the numbers to drive toward zero.
    Ends with INFO,MATCH_DONE.

  Output during ROTATE (one line per control tick):
    SAMPLE,trial=<n>,motor=<n>,t_ms=...,target_deg=...,position_deg=...,
      error_deg=...,speed_setpoint_mm_s=...,ff_pwm=...,correction_pwm=...,
      pwm=...

  Output after ROTATE:
    ROTATE_RESULT,trial=<n>,motor=<n>,target_deg=...,final_deg=...,
      overshoot_deg=...,settle_s=...,settled=<0_or_1>
    overshoot_deg is what the encoder measured (compare against what you
    saw by eye -- a mismatch means MEASURED_COUNTS_PER_REV needs
    re-calibrating, not that the PID gains are wrong). settled=0 means it
    hit timeout_ms without holding inside tolerance; increase timeout_ms or
    lower kp_pos if this happens often. Ends with INFO,ROTATE_DONE,motor=<n>.

  Output during AUTOTUNE (one line per candidate evaluated):
    AUTOTUNE_EVAL,motor=<n>,iter=<i>,param=<kp|ki|kd|lambda|mu>,
      kp=...,ki=...,kd=...,lambda=...,mu=...,cost=...,best_cost=...

  Output after AUTOTUNE:
    AUTOTUNE_RESULT,motor=<n>,kp=...,ki=...,kd=...,lambda=...,mu=...,cost=...
    Ends with INFO,AUTOTUNE_DONE,motor=<n>. The winning gains are already
    loaded into that motor's WheelPID -- run STEP right after to confirm,
    or GAINS to just read them back.

  Suggested workflow: use PIDM to set one motor's gains, run STEP on it and
  compare against arduino_motor_calibration's OPENLOOP baseline at the same
  target_mm_s to confirm PID is actually helping (smaller steady_err_mm_s,
  reasonable overshoot). Repeat per motor. Once all four look reasonable in
  isolation, run MATCH at your typical operating speed and adjust each
  motor's kp/ki (mostly kp, small ki trim) via PIDM to shrink
  left_right_diff_mm_s and spread_mm_s -- that's what actually improves
  odometry, since a straight-drive command should produce near-identical
  wheel speeds. Use ROTATE per motor as a visual spot-check: mark the wheel,
  run ROTATE,<motor>,360, see how far past the mark it stops, hand-correct
  it back, and re-run after nudging kp/ki (or POSGAIN's kp_pos, or
  MEASURED_COUNTS_PER_REV if the encoder's own number disagrees with your
  eye) until it reliably stops on the mark. Once MATCH and ROTATE both look
  good, paste the final per-motor kp/ki/kd/lambda/mu into
  arduino_ros2_base_controller.ino's setup().
*/

#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "QGPMaker_MotorShield.h"
#include "QGPMaker_Encoder.h"
#include "FoPidTuningTypes.h"

// -----------------------------------------------------------------------------
// Constants copied from arduino_ros2_base_controller.ino so tuned gains
// transfer directly. Keep these in sync with that sketch (and with
// arduino_motor_calibration.ino's copy) if robot geometry, encoder
// calibration, or the feed-forward fit ever changes.
// -----------------------------------------------------------------------------

const uint8_t MOTOR_COUNT = 4;
const uint32_t SERIAL_BAUD = 500000UL;
const uint16_t CONTROL_PERIOD_MS = 10;   // 100 Hz control, matches production loop

const float WHEEL_DIAMETER_MM = 80.5f;

// Measured by hand-rotating each wheel exactly one full turn and reading the
// tick count -- see arduino_ros2_base_controller.ino for the full history.
// hand-measured count 4193.0f, 4176.0f, 4119.0f, 4330.0f
const float MEASURED_COUNTS_PER_REV[MOTOR_COUNT] = {4193.0f, 4176.0f, 4119.0f, 4330.0f};
const float MM_PER_COUNT[MOTOR_COUNT] = {
  (PI * WHEEL_DIAMETER_MM) / MEASURED_COUNTS_PER_REV[0],
  (PI * WHEEL_DIAMETER_MM) / MEASURED_COUNTS_PER_REV[1],
  (PI * WHEEL_DIAMETER_MM) / MEASURED_COUNTS_PER_REV[2],
  (PI * WHEEL_DIAMETER_MM) / MEASURED_COUNTS_PER_REV[3],
};

const float MAX_WHEEL_MM_S = 250.0f;   // matches production MAX_WHEEL_MM_S
const uint8_t MIN_EFFECTIVE_PWM = 50;
const uint8_t MAX_DRIVE_PWM = 210;

// Motor order: 1 front-right, 2 front-left, 3 rear-left, 4 rear-right.
const int8_t ENCODER_SIGN[MOTOR_COUNT] = {-1, 1, 1, -1};
const bool LEFT_SIDE[MOTOR_COUNT] = {false, true, true, false};

// Per-motor, per-direction open-loop PWM-to-speed linear fit
// (|measured_mm_s| = slope*pwm + intercept). Keep in sync with
// arduino_ros2_base_controller.ino's and arduino_motor_calibration.ino's
// copies of the same constants.
const float FF_SLOPE_MM_S_PER_PWM_FWD[MOTOR_COUNT] = {2.557f, 2.631f, 2.4945f, 2.3404f};
const float FF_INTERCEPT_MM_S_FWD[MOTOR_COUNT] = {-83.25f, -143.56f, -149.38f, 6.14f};
// Reverse fit from CALIBRATE,<motor>,10,1000,R sweeps (2026-07-05, PWM
// 50-210, stalled deadband points excluded). RMSE 5-17 mm/s.
const float FF_SLOPE_MM_S_PER_PWM_REV[MOTOR_COUNT] = {2.4662f, 2.1564f, 1.9853f, 2.5093f};
const float FF_INTERCEPT_MM_S_REV[MOTOR_COUNT] = {-118.32f, -86.97f, -92.43f, -104.95f};

// -----------------------------------------------------------------------------
// Trial knobs
// -----------------------------------------------------------------------------

const float DEFAULT_TARGET_MM_S = 220.0f;
const uint16_t DEFAULT_DURATION_MS = 1200;
const uint16_t MIN_DURATION_MS = 300;

// Active brake hold between trials, not a coast -- a released, higher-torque
// motor (e.g. motor 4's harder gearing) keeps drifting for longer than the
// others after RELEASE, so trials could start from inconsistent, still-moving
// wheels. Braking forces every motor to a real stop first.
const uint16_t REST_DURATION_MS = 1000;
const float STEADY_WINDOW_FRACTION = 0.3f;   // last 30% of trial used for steady-state avg

// Used to compute the "cost" field in STEP_RESULT, and reused unmodified as
// AUTOTUNE's search objective below (same formula, so a manual STEP number
// and an AUTOTUNE eval number mean the same thing).
const float OVERSHOOT_COST_WEIGHT = 4.0f;
const float STEADY_ERROR_COST_WEIGHT = 6.0f;
const float NO_RISE_PENALTY = 5000.0f;

// -----------------------------------------------------------------------------
// ROTATE position-loop knobs. kp_pos/max_speed are mutable (set via
// POSGAIN) since they're independent of the speed-loop kp/ki/kd -- this is
// a proportional outer loop around the same per-motor WheelPID used by
// STEP/MATCH, producing a speed setpoint that shrinks (and decelerates the
// wheel) as it nears the target angle.
// -----------------------------------------------------------------------------

float posKp = 3.0f;               // (mm/s of speed setpoint) per mm of position error
float posMaxSpeedMMs = 150.0f;    // slower than typical straight-line speed for controllability

const float DEFAULT_TARGET_DEG = 360.0f;   // one full wheel turn
const uint16_t DEFAULT_ROTATE_TIMEOUT_MS = 3000;
const uint16_t MIN_ROTATE_TIMEOUT_MS = 300;
const float ROTATE_SETTLE_TOLERANCE_DEG = 3.0f;
const uint16_t ROTATE_SETTLE_HOLD_MS = 150;

// -----------------------------------------------------------------------------
// AUTOTUNE: twiddle (coordinate-descent) search over kp/ki/kd/lambda/mu per
// motor, using STEP's own cost formula as the objective. Bounds keep
// lambda/mu inside [0.4, 1.4] -- away from the degenerate order-0 case and
// well short of the order-2 stability edge, since twiddle could otherwise
// walk a live motor into an unstable combination mid-search.
// -----------------------------------------------------------------------------

const uint16_t DEFAULT_AUTOTUNE_ITERATIONS = 10;
// Index order throughout AUTOTUNE: 0=kp, 1=ki, 2=kd, 3=lambda, 4=mu.
const float AUTOTUNE_MIN[5]          = {0.50f, 0.0f,   0.0f,   0.5f, 0.5f};
const float AUTOTUNE_MAX[5]          = {1.00f,  0.05f,  0.02f,  1.4f, 1.4f};
const float AUTOTUNE_INITIAL_STEP[5] = {0.01f, 0.005f, 0.001f, 0.1f, 0.1f};

// -----------------------------------------------------------------------------
// Fractional-order PID: PI^lambda D^mu.
//
// Identical design to arduino_ros2_base_controller.ino's WheelPID (see that
// file for the full derivation) -- kept as an independent copy per this
// repo's "separate sketch, independent copies" convention. The integral and
// derivative keep their classic, unbounded/instantaneous form (true pole at
// the origin, so steady-state error still converges to zero); the
// fractional order only reshapes what feeds them via a Grunwald-Letnikov
// (GL) short-memory filter of the residual order (lambda-1 / mu-1). At
// lambda == mu == 1.0 this is byte-for-byte classic PID.
// -----------------------------------------------------------------------------
struct WheelPID {
  static const uint8_t GL_MEMORY_LENGTH = 20;

  float kp;
  float ki;
  float kd;
  float lambda;
  float mu;
  float integral;
  float previousShapedError;
  float previousTarget;
  float errorHistory[GL_MEMORY_LENGTH];  // errorHistory[0] is the newest sample
  uint8_t historyLen;

  void configure(float p, float i, float d, float lam, float m) {
    kp = p;
    ki = i;
    kd = d;
    lambda = lam;
    mu = m;
    reset();
  }

  void reset() {
    integral = 0.0f;
    previousShapedError = 0.0f;
    previousTarget = 0.0f;
    for (uint8_t j = 0; j < GL_MEMORY_LENGTH; ++j) errorHistory[j] = 0.0f;
    historyLen = 0;
  }

  // GL binomial coefficient recursion: c0 = 1, cj = c(j-1) * (1 - (alpha+1)/j).
  // dtScale converts the unitless GL sum to physical units (dt^alpha).
  float glShape(float alpha, float dtScale) const {
    float coeff = 1.0f;
    float sum = errorHistory[0];
    for (uint8_t j = 1; j < historyLen; ++j) {
      coeff *= (1.0f - (alpha + 1.0f) / (float)j);
      sum += coeff * errorHistory[j];
    }
    return sum * dtScale;
  }

  float update(float target, float measured, float dt) {
    if (dt <= 0.0f) return 0.0f;

    if (fabs(target) < 1.0f || (target * previousTarget < 0.0f)) {
      reset();
    }
    previousTarget = target;

    const float error = target - measured;
    for (uint8_t j = GL_MEMORY_LENGTH - 1; j > 0; --j) errorHistory[j] = errorHistory[j - 1];
    errorHistory[0] = error;
    if (historyLen < GL_MEMORY_LENGTH) ++historyLen;

    const float shapedForIntegral = glShape(lambda - 1.0f, pow(dt, 1.0f - lambda));
    integral += shapedForIntegral * dt;
    // Anti-windup: cap the integral by its output contribution (ki*integral,
    // in PWM), not a fixed error-seconds bound. The old +/-500 cap allowed
    // only ~1.5-4 PWM of trim at ki~0.003-0.008, so feed-forward error could
    // never be integrated away -- wheels plateaued off-target no matter how
    // long they ran.
    const float MAX_INTEGRAL_PWM = 80.0f;
    if (ki > 0.0001f) {
      const float limit = MAX_INTEGRAL_PWM / ki;
      integral = constrain(integral, -limit, limit);
    } else {
      integral = constrain(integral, -500.0f, 500.0f);
    }

    const float shapedForDerivative = glShape(mu - 1.0f, pow(dt, 1.0f - mu));
    const float derivative = (shapedForDerivative - previousShapedError) / dt;
    previousShapedError = shapedForDerivative;

    return kp * error + ki * integral + kd * derivative;
  }
};

WheelPID wheelPID[MOTOR_COUNT];

// -----------------------------------------------------------------------------
// Hardware
// -----------------------------------------------------------------------------

QGPMaker_MotorShield motorShield;
QGPMaker_Encoder encoder1(1);
QGPMaker_Encoder encoder2(2);
QGPMaker_Encoder encoder3(3);
QGPMaker_Encoder encoder4(4);

// Map 0-based motor index to the shield's 1-based motor ports.
QGPMaker_DCMotor *getMotor(uint8_t index) {
  return motorShield.getMotor(index + 1);
}

// Read the tick delta since the previous call and clear the counter, with
// interrupts disabled so the ISR cannot update the count mid-read.
int32_t readEncoderAndResetAtomic(uint8_t index) {
  noInterrupts();
  int32_t value;
  switch (index) {
    case 0: value = encoder1.readAndReset(); break;
    case 1: value = encoder2.readAndReset(); break;
    case 2: value = encoder3.readAndReset(); break;
    default: value = encoder4.readAndReset(); break;
  }
  interrupts();
  return value;
}

// Cut power to all motors and let them coast.
void releaseAllMotors() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    QGPMaker_DCMotor *m = getMotor(i);
    m->setSpeed(0);
    m->run(RELEASE);
  }
}

// Active brake (shorts the H-bridge outputs) rather than RELEASE (coast).
// Used between trials so a higher-torque wheel can't keep drifting after
// its target drops to zero.
void brakeAllMotors() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    QGPMaker_DCMotor *m = getMotor(i);
    m->setSpeed(0);
    m->run(BRAKE);
  }
}

// Signed PWM: positive drives FORWARD, negative drives BACKWARD, so STEP
// and MATCH can run reverse trials through the same path.
void applyMotorOutputSigned(uint8_t index, float signedPWM) {
  uint8_t pwm = (uint8_t)constrain((int)(fabs(signedPWM) + 0.5f), 0, 255);
  QGPMaker_DCMotor *m = getMotor(index);
  if (pwm < 1) {
    m->setSpeed(0);
    m->run(RELEASE);
    return;
  }
  m->run((signedPWM >= 0.0f) ? FORWARD : BACKWARD);
  m->setSpeed(pwm);
}

// Returns the PWM magnitude for a signed target speed, using the fit for
// whichever direction the sign selects.
float speedFeedForwardPWM(float targetMMs, uint8_t motorIndex) {
  const bool reverse = targetMMs < 0.0f;
  const float slope = reverse ? FF_SLOPE_MM_S_PER_PWM_REV[motorIndex]
                              : FF_SLOPE_MM_S_PER_PWM_FWD[motorIndex];
  const float intercept = reverse ? FF_INTERCEPT_MM_S_REV[motorIndex]
                                  : FF_INTERCEPT_MM_S_FWD[motorIndex];
  const float pwm = (fabs(targetMMs) - intercept) / slope;
  // No shared lower floor (matches arduino_ros2_base_controller): each
  // motor's fit encodes its own dead zone, and flooring everyone at
  // MIN_EFFECTIVE_PWM=50 forced motor 4 to ~123 mm/s minimum forward.
  return constrain(pwm, 0.0f, (float)MAX_DRIVE_PWM);
}

// -----------------------------------------------------------------------------
// Serial command handling
// -----------------------------------------------------------------------------

char commandBuffer[96];
uint8_t commandLength = 0;
bool abortRequested = false;

// Incremented once per trial so a trial's raw SAMPLE stream can be joined
// back to its *_RESULT summary line by "trial=".
uint16_t trialCounter = 0;

void printReady() {
  Serial.println(F("READY,FoPid_tuning_v1"));
}

// Called continuously during a trial so STOP/ESTOP can interrupt a run
// that's mid-flight. Any other command received while a trial is running
// is dropped.
bool pollAbort() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r' || c == ':') {
      if (commandLength > 0) {
        commandBuffer[commandLength] = '\0';
        commandLength = 0;
        for (char *p = commandBuffer; *p; ++p) {
          if (*p >= 'a' && *p <= 'z') *p = *p - ('a' - 'A');
        }
        if (strcmp(commandBuffer, "STOP") == 0 || strcmp(commandBuffer, "ESTOP") == 0) {
          abortRequested = true;
        }
      }
    } else if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = c;
    } else {
      commandLength = 0;
    }
  }
  return abortRequested;
}

// Common cleanup when a trial is interrupted by STOP/ESTOP.
void finishAborted() {
  releaseAllMotors();
  Serial.println(F("ABORTED,STOPPED"));
}

// -----------------------------------------------------------------------------
// STEP: single-motor closed-loop (PID engaged) dynamic step trial
// -----------------------------------------------------------------------------

// Run one closed-loop step test on a single motor: PID + feed-forward drive
// for durationMs, optional per-tick SAMPLE output (AUTOTUNE runs it silent),
// then rise/overshoot/steady-error/cost metrics and a braked rest.
StepResult runStepTrial(uint8_t motorIndex, float targetMMs, uint16_t durationMs, bool emitSamples) {
  StepResult result = {0.0f, -1.0f, 0.0f, 0.0f, false, ++trialCounter};

  // Trials run signed: dir flips the drive direction and maps measured
  // speed back into magnitude space so rise/overshoot/steady metrics mean
  // the same thing forward and reverse. The PID itself works in signed
  // space, same as production.
  const float dir = (targetMMs < 0.0f) ? -1.0f : 1.0f;
  const float targetMag = fabs(targetMMs);

  readEncoderAndResetAtomic(motorIndex);
  wheelPID[motorIndex].reset();
  float filteredSpeed = 0.0f;

  const uint32_t start = millis();
  uint32_t last = start;
  float peak = 0.0f;
  bool riseRecorded = false;
  const float riseThreshold = targetMag * 0.9f;

  float steadySum = 0.0f;
  uint32_t steadyCount = 0;
  const uint32_t steadyStartMs = (uint32_t)(durationMs * (1.0f - STEADY_WINDOW_FRACTION));

  while (millis() - start < durationMs) {
    if (pollAbort()) {
      result.aborted = true;
      break;
    }

    uint32_t now = millis();
    if (now - last >= CONTROL_PERIOD_MS) {
      float dt = (now - last) * 0.001f;
      last = now;
      dt = constrain(dt, 0.005f, 0.050f);

      // Encoder ticks -> mm/s, low-pass filtered like the production loop.
      const int32_t rawTicks = readEncoderAndResetAtomic(motorIndex);
      const float signedTicks = rawTicks * ENCODER_SIGN[motorIndex];
      const float rawSpeed = (signedTicks * MM_PER_COUNT[motorIndex]) / dt;
      filteredSpeed += 0.35f * (rawSpeed - filteredSpeed);

      // Positive when moving in the commanded direction, whichever it is.
      const float speedMag = dir * filteredSpeed;

      const float error = targetMMs - filteredSpeed;
      const float feedForward = speedFeedForwardPWM(targetMMs, motorIndex);
      const float correction = wheelPID[motorIndex].update(targetMMs, filteredSpeed, dt);
      // Same shape as production: correction is signed, feed-forward is a
      // magnitude, output magnitude clamps at 0 (no direction flip).
      const float out = constrain(feedForward + dir * correction, 0.0f, 255.0f);
      applyMotorOutputSigned(motorIndex, dir * out);

      const uint32_t elapsedMs = now - start;

      if (emitSamples) {
        Serial.print(F("SAMPLE,trial="));
        Serial.print(result.trialId);
        Serial.print(F(",motor="));
        Serial.print(motorIndex + 1);
        Serial.print(F(",t_ms="));
        Serial.print(elapsedMs);
        Serial.print(F(",target_mm_s="));
        Serial.print(targetMMs, 2);
        Serial.print(F(",measured_mm_s="));
        Serial.print(filteredSpeed, 2);
        Serial.print(F(",error_mm_s="));
        Serial.print(error, 2);
        Serial.print(F(",ff_pwm="));
        Serial.print(dir * feedForward, 1);
        Serial.print(F(",correction_pwm="));
        Serial.print(correction, 1);
        Serial.print(F(",pwm="));
        Serial.println(dir * out, 1);
      }

      result.cost += fabs(error) * (elapsedMs * 0.001f) * dt;

      if (speedMag > peak) peak = speedMag;
      if (!riseRecorded && speedMag >= riseThreshold) {
        result.riseTimeS = elapsedMs * 0.001f;
        riseRecorded = true;
      }

      if (elapsedMs >= steadyStartMs) {
        steadySum += speedMag;
        ++steadyCount;
      }
    }
  }

  brakeAllMotors();

  // Summary metrics: overshoot relative to the target magnitude, steady-state
  // error from the averaged tail window (or last sample if aborted early),
  // plus the weighted cost terms AUTOTUNE minimizes.
  result.overshootPct = (peak > targetMag) ? ((peak - targetMag) / targetMag * 100.0f) : 0.0f;
  if (steadyCount > 0) {
    result.steadyStateErrorMMs = targetMag - (steadySum / steadyCount);
  } else {
    result.steadyStateErrorMMs = targetMag - dir * filteredSpeed;
  }
  if (!riseRecorded) {
    result.cost += NO_RISE_PENALTY;
  }
  result.cost += OVERSHOOT_COST_WEIGHT * result.overshootPct;
  result.cost += STEADY_ERROR_COST_WEIGHT * fabs(result.steadyStateErrorMMs);

  // Braked rest so the wheel is truly stopped before the next trial.
  if (!result.aborted) {
    const uint32_t restStart = millis();
    while (millis() - restStart < REST_DURATION_MS) {
      if (pollAbort()) {
        result.aborted = true;
        break;
      }
    }
  }

  return result;
}

// Print the STEP_RESULT summary line for one finished step test.
void printStepResult(uint8_t motorIndex, float targetMMs, const StepResult &r) {
  Serial.print(F("STEP_RESULT,trial="));
  Serial.print(r.trialId);
  Serial.print(F(",motor="));
  Serial.print(motorIndex + 1);
  Serial.print(F(",target_mm_s="));
  Serial.print(targetMMs, 1);
  Serial.print(F(",rise_s="));
  Serial.print(r.riseTimeS, 3);
  Serial.print(F(",overshoot_pct="));
  Serial.print(r.overshootPct, 1);
  Serial.print(F(",steady_err_mm_s="));
  Serial.print(r.steadyStateErrorMMs, 2);
  Serial.print(F(",cost="));
  Serial.println(r.cost, 3);
}

// Top-level STEP command: run one verbose step trial on one motor and report.
void runStep(uint8_t motorIndex, float targetMMs, uint16_t durationMs) {
  abortRequested = false;
  releaseAllMotors();

  Serial.print(F("INFO,STEP_START,motor="));
  Serial.print(motorIndex + 1);
  Serial.print(F(",target_mm_s="));
  Serial.println(targetMMs, 1);

  StepResult result = runStepTrial(motorIndex, targetMMs, durationMs, true);
  if (result.aborted) {
    finishAborted();
    return;
  }

  releaseAllMotors();
  printStepResult(motorIndex, targetMMs, result);
  Serial.print(F("INFO,STEP_DONE,motor="));
  Serial.println(motorIndex + 1);
}

// -----------------------------------------------------------------------------
// AUTOTUNE: twiddle search over kp/ki/kd/lambda/mu for one motor, using a
// silent (no per-tick SAMPLE output) STEP trial as the cost evaluator.
// -----------------------------------------------------------------------------

// Load a candidate gain set (index order kp/ki/kd/lambda/mu) into the motor's
// PID and run one silent step trial; the trial's cost is the search objective.
float evaluateAutotuneCost(uint8_t motorIndex, const float *params, float targetMMs, bool *aborted) {
  wheelPID[motorIndex].configure(params[0], params[1], params[2], params[3], params[4]);
  StepResult r = runStepTrial(motorIndex, targetMMs, DEFAULT_DURATION_MS, false);
  *aborted = r.aborted;
  return r.cost;
}

// Print one AUTOTUNE_EVAL line describing a just-evaluated candidate.
void printAutotuneEval(uint8_t motorIndex, uint16_t iter, const char *paramName,
                        const float *params, float cost, float bestCost) {
  Serial.print(F("AUTOTUNE_EVAL,motor="));
  Serial.print(motorIndex + 1);
  Serial.print(F(",iter="));
  Serial.print(iter);
  Serial.print(F(",param="));
  Serial.print(paramName);
  Serial.print(F(",kp="));
  Serial.print(params[0], 4);
  Serial.print(F(",ki="));
  Serial.print(params[1], 4);
  Serial.print(F(",kd="));
  Serial.print(params[2], 4);
  Serial.print(F(",lambda="));
  Serial.print(params[3], 3);
  Serial.print(F(",mu="));
  Serial.print(params[4], 3);
  Serial.print(F(",cost="));
  Serial.print(cost, 3);
  Serial.print(F(",best_cost="));
  Serial.println(bestCost, 3);
}

// Print the final AUTOTUNE_RESULT line with the winning gain set.
void printAutotuneResult(uint8_t motorIndex, const float *params, float cost) {
  Serial.print(F("AUTOTUNE_RESULT,motor="));
  Serial.print(motorIndex + 1);
  Serial.print(F(",kp="));
  Serial.print(params[0], 4);
  Serial.print(F(",ki="));
  Serial.print(params[1], 4);
  Serial.print(F(",kd="));
  Serial.print(params[2], 4);
  Serial.print(F(",lambda="));
  Serial.print(params[3], 3);
  Serial.print(F(",mu="));
  Serial.print(params[4], 3);
  Serial.print(F(",cost="));
  Serial.println(cost, 3);
}

// Top-level AUTOTUNE command: twiddle (coordinate descent) starting from the
// motor's current gains. For each parameter in turn, try +dp; if that doesn't
// beat the best cost, try -dp; grow dp on success, shrink it on failure.
void runAutotune(uint8_t motorIndex, float targetMMs, uint16_t maxIterations) {
  abortRequested = false;
  releaseAllMotors();

  static const char *AUTOTUNE_PARAM_NAMES[5] = {"kp", "ki", "kd", "lambda", "mu"};

  Serial.print(F("INFO,AUTOTUNE_START,motor="));
  Serial.print(motorIndex + 1);
  Serial.print(F(",target_mm_s="));
  Serial.println(targetMMs, 1);

  float bestParams[5] = {
    wheelPID[motorIndex].kp, wheelPID[motorIndex].ki, wheelPID[motorIndex].kd,
    wheelPID[motorIndex].lambda, wheelPID[motorIndex].mu
  };
  float dp[5];
  for (uint8_t i = 0; i < 5; ++i) dp[i] = AUTOTUNE_INITIAL_STEP[i];

  // Baseline cost of the starting gains; every candidate must beat this.
  bool aborted = false;
  float bestCost = evaluateAutotuneCost(motorIndex, bestParams, targetMMs, &aborted);

  for (uint16_t iter = 0; iter < maxIterations && !aborted; ++iter) {
    for (uint8_t i = 0; i < 5 && !aborted; ++i) {
      // Probe upward first: parameter i + dp[i], clamped to the safe bounds.
      float trialParams[5];
      memcpy(trialParams, bestParams, sizeof(trialParams));
      trialParams[i] = constrain(bestParams[i] + dp[i], AUTOTUNE_MIN[i], AUTOTUNE_MAX[i]);

      float cost = evaluateAutotuneCost(motorIndex, trialParams, targetMMs, &aborted);
      printAutotuneEval(motorIndex, iter, AUTOTUNE_PARAM_NAMES[i], trialParams, cost, bestCost);
      if (aborted) break;

      if (cost < bestCost) {
        // Improvement: keep it and widen this parameter's search step.
        bestCost = cost;
        memcpy(bestParams, trialParams, sizeof(bestParams));
        dp[i] *= 1.1f;
        continue;
      }

      // Upward probe failed: try the same distance downward.
      trialParams[i] = constrain(bestParams[i] - dp[i], AUTOTUNE_MIN[i], AUTOTUNE_MAX[i]);
      cost = evaluateAutotuneCost(motorIndex, trialParams, targetMMs, &aborted);
      printAutotuneEval(motorIndex, iter, AUTOTUNE_PARAM_NAMES[i], trialParams, cost, bestCost);
      if (aborted) break;

      if (cost < bestCost) {
        bestCost = cost;
        memcpy(bestParams, trialParams, sizeof(bestParams));
        dp[i] *= 1.1f;
      } else {
        // Neither direction helped: narrow this parameter's search step.
        dp[i] *= 0.9f;
      }
    }
  }

  if (aborted) {
    finishAborted();
    return;
  }

  // Leave the best-found gains loaded so STEP/MATCH can validate them
  // immediately without re-entering them by hand.
  wheelPID[motorIndex].configure(bestParams[0], bestParams[1], bestParams[2], bestParams[3], bestParams[4]);
  releaseAllMotors();
  printAutotuneResult(motorIndex, bestParams, bestCost);
  Serial.print(F("INFO,AUTOTUNE_DONE,motor="));
  Serial.println(motorIndex + 1);
}

// -----------------------------------------------------------------------------
// MATCH: all four motors closed-loop at the same target speed at once.
// Cross-wheel mismatch (not any single motor's own error) is the proxy for
// odometry drift used here -- a straight-drive command should produce
// near-identical wheel speeds.
// -----------------------------------------------------------------------------

// Drive all four motors closed-loop at the same target for durationMs,
// streaming combined SAMPLE lines, then compute per-wheel steady-state speeds
// and the cross-wheel mismatch metrics (left/right diff, max pair diff, spread).
MatchResult runMatchTrial(float targetMMs, uint16_t durationMs) {
  MatchResult result;
  result.aborted = false;
  result.trialId = ++trialCounter;

  // Negative target runs the whole trial in reverse. Speeds and PWMs are
  // reported signed; the mismatch metrics work the same either way.
  const float dir = (targetMMs < 0.0f) ? -1.0f : 1.0f;
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    result.steadyMMs[i] = 0.0f;
    result.errorMMs[i] = 0.0f;
  }

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    readEncoderAndResetAtomic(i);
    wheelPID[i].reset();
  }
  float filteredSpeed[MOTOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
  float steadySum[MOTOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
  uint32_t steadyCount = 0;

  const uint32_t start = millis();
  uint32_t last = start;
  const uint32_t steadyStartMs = (uint32_t)(durationMs * (1.0f - STEADY_WINDOW_FRACTION));

  while (millis() - start < durationMs) {
    if (pollAbort()) {
      result.aborted = true;
      break;
    }

    uint32_t now = millis();
    if (now - last >= CONTROL_PERIOD_MS) {
      float dt = (now - last) * 0.001f;
      last = now;
      dt = constrain(dt, 0.005f, 0.050f);
      const uint32_t elapsedMs = now - start;

      // Same per-wheel pipeline as STEP, applied to all four motors in one
      // tick: measure, filter, feed-forward + PID, drive.
      float pwmOut[MOTOR_COUNT];
      for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        const int32_t rawTicks = readEncoderAndResetAtomic(i);
        const float signedTicks = rawTicks * ENCODER_SIGN[i];
        const float rawSpeed = (signedTicks * MM_PER_COUNT[i]) / dt;
        filteredSpeed[i] += 0.35f * (rawSpeed - filteredSpeed[i]);

        const float feedForward = speedFeedForwardPWM(targetMMs, i);
        const float correction = wheelPID[i].update(targetMMs, filteredSpeed[i], dt);
        const float out = constrain(feedForward + dir * correction, 0.0f, 255.0f);
        applyMotorOutputSigned(i, dir * out);
        pwmOut[i] = dir * out;

        if (elapsedMs >= steadyStartMs) {
          steadySum[i] += filteredSpeed[i];
        }
      }
      if (elapsedMs >= steadyStartMs) ++steadyCount;

      Serial.print(F("SAMPLE,trial="));
      Serial.print(result.trialId);
      Serial.print(F(",t_ms="));
      Serial.print(elapsedMs);
      Serial.print(F(",target_mm_s="));
      Serial.print(targetMMs, 2);
      for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        Serial.print(F(",m")); Serial.print(i + 1); Serial.print(F("_mm_s="));
        Serial.print(filteredSpeed[i], 2);
      }
      for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        Serial.print(F(",m")); Serial.print(i + 1); Serial.print(F("_pwm="));
        Serial.print(pwmOut[i], 1);
      }
      Serial.println();
    }
  }

  brakeAllMotors();

  // Steady-state speed per wheel from the tail-window average, and each
  // wheel's own tracking error against the shared target.
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    result.steadyMMs[i] = (steadyCount > 0) ? (steadySum[i] / steadyCount) : filteredSpeed[i];
    result.errorMMs[i] = targetMMs - result.steadyMMs[i];
  }

  // Cross-wheel mismatch metrics: side averages, signed left-right
  // difference, and max-min spread across all four wheels.
  float leftSum = 0.0f, rightSum = 0.0f;
  uint8_t leftCount = 0, rightCount = 0;
  float maxSpeed = result.steadyMMs[0], minSpeed = result.steadyMMs[0];
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    if (LEFT_SIDE[i]) { leftSum += result.steadyMMs[i]; ++leftCount; }
    else { rightSum += result.steadyMMs[i]; ++rightCount; }
    if (result.steadyMMs[i] > maxSpeed) maxSpeed = result.steadyMMs[i];
    if (result.steadyMMs[i] < minSpeed) minSpeed = result.steadyMMs[i];
  }
  result.leftAvgMMs = (leftCount > 0) ? (leftSum / leftCount) : 0.0f;
  result.rightAvgMMs = (rightCount > 0) ? (rightSum / rightCount) : 0.0f;
  result.leftRightDiffMMs = result.leftAvgMMs - result.rightAvgMMs;
  result.spreadMMs = maxSpeed - minSpeed;

  // Largest speed gap between any two wheels.
  float maxPairDiff = 0.0f;
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    for (uint8_t j = i + 1; j < MOTOR_COUNT; ++j) {
      const float diff = fabs(result.steadyMMs[i] - result.steadyMMs[j]);
      if (diff > maxPairDiff) maxPairDiff = diff;
    }
  }
  result.maxPairDiffMMs = maxPairDiff;

  if (!result.aborted) {
    const uint32_t restStart = millis();
    while (millis() - restStart < REST_DURATION_MS) {
      if (pollAbort()) {
        result.aborted = true;
        break;
      }
    }
  }

  return result;
}

// Print the MATCH_RESULT summary line for one finished four-wheel trial.
void printMatchResult(float targetMMs, const MatchResult &r) {
  Serial.print(F("MATCH_RESULT,trial="));
  Serial.print(r.trialId);
  Serial.print(F(",target_mm_s="));
  Serial.print(targetMMs, 1);
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    Serial.print(F(",m")); Serial.print(i + 1); Serial.print(F("_ss_mm_s="));
    Serial.print(r.steadyMMs[i], 2);
  }
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    Serial.print(F(",m")); Serial.print(i + 1); Serial.print(F("_err_mm_s="));
    Serial.print(r.errorMMs[i], 2);
  }
  Serial.print(F(",left_avg_mm_s=")); Serial.print(r.leftAvgMMs, 2);
  Serial.print(F(",right_avg_mm_s=")); Serial.print(r.rightAvgMMs, 2);
  Serial.print(F(",left_right_diff_mm_s=")); Serial.print(r.leftRightDiffMMs, 2);
  Serial.print(F(",max_pair_diff_mm_s=")); Serial.print(r.maxPairDiffMMs, 2);
  Serial.print(F(",spread_mm_s=")); Serial.println(r.spreadMMs, 2);
}

// Top-level MATCH command: run one four-wheel matching trial and report.
void runMatch(float targetMMs, uint16_t durationMs) {
  abortRequested = false;
  releaseAllMotors();

  Serial.print(F("INFO,MATCH_START,target_mm_s="));
  Serial.println(targetMMs, 1);

  MatchResult result = runMatchTrial(targetMMs, durationMs);
  if (result.aborted) {
    finishAborted();
    return;
  }

  releaseAllMotors();
  printMatchResult(targetMMs, result);
  Serial.println(F("INFO,MATCH_DONE"));
}

// -----------------------------------------------------------------------------
// ROTATE: single-motor position move to a target angle, meant to be
// watched by eye. A proportional outer position loop (gain posKp) produces
// a speed setpoint fed into the same per-motor speed WheelPID used by
// STEP/MATCH -- the setpoint naturally shrinks (decelerating the wheel) as
// position error shrinks, capped at posMaxSpeedMMs. The motor is released
// (not braked) at the end so it can be hand-corrected back to the mark.
// -----------------------------------------------------------------------------

// Run one position move: P position loop -> speed setpoint -> speed PID,
// until the wheel holds within tolerance for ROTATE_SETTLE_HOLD_MS or the
// timeout expires. Position is tracked in rim-mm (angle * mm-per-degree).
RotateResult runRotateTrial(uint8_t motorIndex, float targetDeg, uint16_t timeoutMs) {
  RotateResult result = {0.0f, 0.0f, -1.0f, false, false, ++trialCounter};

  const float mmPerDeg = (PI * WHEEL_DIAMETER_MM) / 360.0f;
  const float targetMM = targetDeg * mmPerDeg;
  const float toleranceMM = ROTATE_SETTLE_TOLERANCE_DEG * mmPerDeg;

  readEncoderAndResetAtomic(motorIndex);
  wheelPID[motorIndex].reset();

  float positionMM = 0.0f;
  float filteredSpeed = 0.0f;
  float peakPositionMM = 0.0f;
  bool withinTolerance = false;
  uint32_t withinToleranceStartMs = 0;

  const uint32_t start = millis();
  uint32_t last = start;

  while (millis() - start < timeoutMs) {
    if (pollAbort()) {
      result.aborted = true;
      break;
    }

    uint32_t now = millis();
    if (now - last >= CONTROL_PERIOD_MS) {
      float dt = (now - last) * 0.001f;
      last = now;
      dt = constrain(dt, 0.005f, 0.050f);

      // Integrate encoder travel into position; speed is filtered as usual.
      const int32_t rawTicks = readEncoderAndResetAtomic(motorIndex);
      const float deltaMM = (rawTicks * ENCODER_SIGN[motorIndex]) * MM_PER_COUNT[motorIndex];
      positionMM += deltaMM;
      filteredSpeed += 0.35f * ((deltaMM / dt) - filteredSpeed);
      if (positionMM > peakPositionMM) peakPositionMM = positionMM;

      const float positionError = targetMM - positionMM;
      // Forward-only trial (matches STEP/MATCH convention): never ask for
      // backward motion even if position overshot the target.
      float speedSetpoint = constrain(posKp * positionError, 0.0f, posMaxSpeedMMs);

      float feedForward = 0.0f;
      float correction = 0.0f;
      float out = 0.0f;
      if (speedSetpoint < 1.0f) {
        // At/past target: stop driving and let the speed loop relax so it
        // doesn't wind up integral against a wheel that's no longer moving.
        wheelPID[motorIndex].reset();
      } else {
        feedForward = speedFeedForwardPWM(speedSetpoint, motorIndex);
        correction = wheelPID[motorIndex].update(speedSetpoint, filteredSpeed, dt);
        out = constrain(feedForward + correction, 0.0f, 255.0f);
      }
      // ROTATE stays forward-only (setpoint clamped >= 0 above), so the
      // signed drive helper always gets a non-negative PWM here.
      applyMotorOutputSigned(motorIndex, out);

      const uint32_t elapsedMs = now - start;

      Serial.print(F("SAMPLE,trial="));
      Serial.print(result.trialId);
      Serial.print(F(",motor="));
      Serial.print(motorIndex + 1);
      Serial.print(F(",t_ms="));
      Serial.print(elapsedMs);
      Serial.print(F(",target_deg="));
      Serial.print(targetDeg, 1);
      Serial.print(F(",position_deg="));
      Serial.print(positionMM / mmPerDeg, 2);
      Serial.print(F(",error_deg="));
      Serial.print(positionError / mmPerDeg, 2);
      Serial.print(F(",speed_setpoint_mm_s="));
      Serial.print(speedSetpoint, 2);
      Serial.print(F(",ff_pwm="));
      Serial.print(feedForward, 1);
      Serial.print(F(",correction_pwm="));
      Serial.print(correction, 1);
      Serial.print(F(",pwm="));
      Serial.println(out, 1);

      // Settled = held inside the angle tolerance continuously for
      // ROTATE_SETTLE_HOLD_MS; leaving tolerance restarts the clock.
      if (fabs(positionError) <= toleranceMM) {
        if (!withinTolerance) {
          withinTolerance = true;
          withinToleranceStartMs = now;
        }
        if (now - withinToleranceStartMs >= ROTATE_SETTLE_HOLD_MS) {
          result.settled = true;
          result.settleTimeS = elapsedMs * 0.001f;
          break;
        }
      } else {
        withinTolerance = false;
      }
    }
  }

  // Released, not braked -- lets the wheel be hand-corrected back to the
  // mark before the next trial.
  releaseAllMotors();

  // Final angle and encoder-measured overshoot past the target; if it never
  // settled, report the full elapsed time instead.
  result.finalAngleDeg = positionMM / mmPerDeg;
  const float overshootMM = peakPositionMM - targetMM;
  result.overshootDeg = (overshootMM > 0.0f) ? (overshootMM / mmPerDeg) : 0.0f;
  if (!result.settled) result.settleTimeS = (millis() - start) * 0.001f;

  return result;
}

// Print the ROTATE_RESULT summary line for one finished position move.
void printRotateResult(uint8_t motorIndex, float targetDeg, const RotateResult &r) {
  Serial.print(F("ROTATE_RESULT,trial="));
  Serial.print(r.trialId);
  Serial.print(F(",motor="));
  Serial.print(motorIndex + 1);
  Serial.print(F(",target_deg="));
  Serial.print(targetDeg, 1);
  Serial.print(F(",final_deg="));
  Serial.print(r.finalAngleDeg, 2);
  Serial.print(F(",overshoot_deg="));
  Serial.print(r.overshootDeg, 2);
  Serial.print(F(",settle_s="));
  Serial.print(r.settleTimeS, 3);
  Serial.print(F(",settled="));
  Serial.println(r.settled ? 1 : 0);
}

// Top-level ROTATE command: run one position move on one motor and report.
void runRotate(uint8_t motorIndex, float targetDeg, uint16_t timeoutMs) {
  abortRequested = false;
  releaseAllMotors();

  Serial.print(F("INFO,ROTATE_START,motor="));
  Serial.print(motorIndex + 1);
  Serial.print(F(",target_deg="));
  Serial.println(targetDeg, 1);

  RotateResult result = runRotateTrial(motorIndex, targetDeg, timeoutMs);
  if (result.aborted) {
    finishAborted();
    return;
  }

  printRotateResult(motorIndex, targetDeg, result);
  Serial.print(F("INFO,ROTATE_DONE,motor="));
  Serial.println(motorIndex + 1);
}

// -----------------------------------------------------------------------------
// Command parser
// -----------------------------------------------------------------------------

// Print every motor's current kp/ki/kd/lambda/mu (the GAINS command).
void printGains() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    Serial.print(F("GAINS,motor="));
    Serial.print(i + 1);
    Serial.print(F(",kp=")); Serial.print(wheelPID[i].kp, 4);
    Serial.print(F(",ki=")); Serial.print(wheelPID[i].ki, 4);
    Serial.print(F(",kd=")); Serial.print(wheelPID[i].kd, 4);
    Serial.print(F(",lambda=")); Serial.print(wheelPID[i].lambda, 3);
    Serial.print(F(",mu=")); Serial.println(wheelPID[i].mu, 3);
  }
}

// Parse and execute one complete command line (command set in the header
// comment). Tokenizes in place with strtok_r; the command token is upper-cased
// for case-insensitive matching.
void processCommand(char *line) {
  char *savePtr = NULL;
  char *command = strtok_r(line, ",", &savePtr);
  if (command == NULL) return;

  for (char *p = command; *p; ++p) {
    if (*p >= 'a' && *p <= 'z') *p = *p - ('a' - 'A');
  }

  if (strcmp(command, "PID") == 0) {
    char *pText = strtok_r(NULL, ",", &savePtr);
    char *iText = strtok_r(NULL, ",", &savePtr);
    char *dText = strtok_r(NULL, ",", &savePtr);
    char *lamText = strtok_r(NULL, ",", &savePtr);
    char *muText = strtok_r(NULL, ",", &savePtr);
    if (pText && iText && dText) {
      const float lambda = lamText ? atof(lamText) : 1.0f;
      const float mu = muText ? atof(muText) : 1.0f;
      for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        wheelPID[i].configure(atof(pText), atof(iText), atof(dText), lambda, mu);
      }
      Serial.println(F("ACK,PID_ALL"));
    } else {
      Serial.println(F("ERR,PID_FORMAT"));
    }
  }
  else if (strcmp(command, "PIDM") == 0) {
    char *motorText = strtok_r(NULL, ",", &savePtr);
    char *pText = strtok_r(NULL, ",", &savePtr);
    char *iText = strtok_r(NULL, ",", &savePtr);
    char *dText = strtok_r(NULL, ",", &savePtr);
    char *lamText = strtok_r(NULL, ",", &savePtr);
    char *muText = strtok_r(NULL, ",", &savePtr);
    int motorNum = motorText ? atoi(motorText) : -1;
    if (motorNum >= 1 && motorNum <= MOTOR_COUNT && pText && iText && dText) {
      const float lambda = lamText ? atof(lamText) : 1.0f;
      const float mu = muText ? atof(muText) : 1.0f;
      wheelPID[motorNum - 1].configure(atof(pText), atof(iText), atof(dText), lambda, mu);
      Serial.println(F("ACK,PID_MOTOR"));
    } else {
      Serial.println(F("ERR,PIDM_FORMAT"));
    }
  }
  else if (strcmp(command, "GAINS") == 0) {
    printGains();
  }
  else if (strcmp(command, "STEP") == 0) {
    char *motorText = strtok_r(NULL, ",", &savePtr);
    char *targetText = strtok_r(NULL, ",", &savePtr);
    char *durationText = strtok_r(NULL, ",", &savePtr);

    int motorNum = motorText ? atoi(motorText) : -1;
    if (motorNum < 1 || motorNum > MOTOR_COUNT) {
      Serial.println(F("ERR,STEP_FORMAT,expected_STEP_motor_1to4"));
      return;
    }

    // Negative target = reverse step.
    float targetMMs = targetText ? atof(targetText) : DEFAULT_TARGET_MM_S;
    const float stepMag = constrain(fabs(targetMMs), 30.0f, MAX_WHEEL_MM_S);
    targetMMs = (targetMMs < 0.0f) ? -stepMag : stepMag;

    uint16_t durationMs = durationText ? (uint16_t)atoi(durationText) : DEFAULT_DURATION_MS;
    if (durationMs < MIN_DURATION_MS) durationMs = DEFAULT_DURATION_MS;

    runStep((uint8_t)(motorNum - 1), targetMMs, durationMs);
  }
  else if (strcmp(command, "AUTOTUNE") == 0) {
    char *motorText = strtok_r(NULL, ",", &savePtr);
    char *targetText = strtok_r(NULL, ",", &savePtr);
    char *iterText = strtok_r(NULL, ",", &savePtr);

    int motorNum = motorText ? atoi(motorText) : -1;
    if (motorNum < 1 || motorNum > MOTOR_COUNT) {
      Serial.println(F("ERR,AUTOTUNE_FORMAT,expected_AUTOTUNE_motor_1to4"));
      return;
    }

    // Negative target = evaluate candidates with reverse step trials.
    float targetMMs = targetText ? atof(targetText) : DEFAULT_TARGET_MM_S;
    const float tuneMag = constrain(fabs(targetMMs), 30.0f, MAX_WHEEL_MM_S);
    targetMMs = (targetMMs < 0.0f) ? -tuneMag : tuneMag;

    uint16_t maxIterations = iterText ? (uint16_t)atoi(iterText) : DEFAULT_AUTOTUNE_ITERATIONS;
    if (maxIterations < 1) maxIterations = DEFAULT_AUTOTUNE_ITERATIONS;

    runAutotune((uint8_t)(motorNum - 1), targetMMs, maxIterations);
  }
  else if (strcmp(command, "MATCH") == 0) {
    char *targetText = strtok_r(NULL, ",", &savePtr);
    char *durationText = strtok_r(NULL, ",", &savePtr);

    // Negative target = reverse trial (checks backward straight-line
    // matching against the reverse feed-forward curve).
    float targetMMs = targetText ? atof(targetText) : DEFAULT_TARGET_MM_S;
    const float matchMag = constrain(fabs(targetMMs), 30.0f, MAX_WHEEL_MM_S);
    targetMMs = (targetMMs < 0.0f) ? -matchMag : matchMag;

    uint16_t durationMs = durationText ? (uint16_t)atoi(durationText) : DEFAULT_DURATION_MS;
    if (durationMs < MIN_DURATION_MS) durationMs = DEFAULT_DURATION_MS;

    runMatch(targetMMs, durationMs);
  }
  else if (strcmp(command, "POSGAIN") == 0) {
    char *kpText = strtok_r(NULL, ",", &savePtr);
    char *maxSpeedText = strtok_r(NULL, ",", &savePtr);
    if (kpText) {
      posKp = atof(kpText);
      if (maxSpeedText) posMaxSpeedMMs = atof(maxSpeedText);
      Serial.println(F("ACK,POSGAIN"));
    }
    Serial.print(F("POSGAIN,kp_pos="));
    Serial.print(posKp, 3);
    Serial.print(F(",max_speed_mm_s="));
    Serial.println(posMaxSpeedMMs, 1);
  }
  else if (strcmp(command, "ROTATE") == 0) {
    char *motorText = strtok_r(NULL, ",", &savePtr);
    char *targetText = strtok_r(NULL, ",", &savePtr);
    char *timeoutText = strtok_r(NULL, ",", &savePtr);

    int motorNum = motorText ? atoi(motorText) : -1;
    if (motorNum < 1 || motorNum > MOTOR_COUNT) {
      Serial.println(F("ERR,ROTATE_FORMAT,expected_ROTATE_motor_1to4"));
      return;
    }

    float targetDeg = targetText ? atof(targetText) : DEFAULT_TARGET_DEG;
    targetDeg = fabs(targetDeg);
    if (targetDeg < 1.0f) targetDeg = DEFAULT_TARGET_DEG;

    uint16_t timeoutMs = timeoutText ? (uint16_t)atoi(timeoutText) : DEFAULT_ROTATE_TIMEOUT_MS;
    if (timeoutMs < MIN_ROTATE_TIMEOUT_MS) timeoutMs = DEFAULT_ROTATE_TIMEOUT_MS;

    runRotate((uint8_t)(motorNum - 1), targetDeg, timeoutMs);
  }
  else if (strcmp(command, "STOP") == 0 || strcmp(command, "ESTOP") == 0) {
    releaseAllMotors();
    Serial.println(F("ACK,STOPPED"));
  }
  else if (strcmp(command, "PING") == 0) {
    Serial.println(F("PONG"));
  }
  else {
    Serial.print(F("ERR,UNKNOWN_COMMAND,"));
    Serial.println(command);
  }
}

// Accumulate serial bytes into commandBuffer and dispatch a command whenever
// a terminator (\n, \r or ':') arrives. Oversized lines are discarded.
void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r' || c == ':') {
      if (commandLength > 0) {
        commandBuffer[commandLength] = '\0';
        processCommand(commandBuffer);
        commandLength = 0;
      }
    } else if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = c;
    } else {
      commandLength = 0;
      Serial.println(F("ERR,COMMAND_TOO_LONG"));
    }
  }
}

// -----------------------------------------------------------------------------
// Arduino setup/loop
// -----------------------------------------------------------------------------

// One-time init: serial link, I2C, motor shield at 500 Hz PWM, motors
// released, encoder counters cleared, production gains loaded. No IMU here.
void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  Wire.setClock(400000UL);

  motorShield.begin(500);
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) getMotor(i);
  releaseAllMotors();

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) readEncoderAndResetAtomic(i);

  // Starting point: current arduino_ros2_base_controller.ino production
  // gains (lambda/mu = 1.0, i.e. classic PID -- see that sketch's setup()).
  // Re-sync these if production's gains change independently, and paste
  // your final tuned values back there when done.
  // ki raised to 0.02 (2026-07-06), kept identical to production
  // arduino_ros2_base_controller setup() -- see the comment there.
  wheelPID[0].configure(0.3420f, 0.0200f, 0.0030f, 1.0f, 1.0f);
  wheelPID[1].configure(0.2900f, 0.0200f, 0.0030f, 1.0f, 1.0f);
  wheelPID[2].configure(0.3400f, 0.0200f, 0.0030f, 1.0f, 1.0f);
  wheelPID[3].configure(0.3000f, 0.0200f, 0.0035f, 1.0f, 1.0f);

  printReady();
  Serial.println(F("INFO,LIFT_ALL_WHEELS_OFF_THE_GROUND_BEFORE_RUNNING_TRIALS"));
}

// Idle loop: everything happens inside the blocking trial runners, which
// poll for STOP/ESTOP themselves via pollAbort().
void loop() {
  readSerialCommands();
}
