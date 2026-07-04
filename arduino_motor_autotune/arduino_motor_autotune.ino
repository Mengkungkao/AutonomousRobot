/*
  mDetect standalone per-wheel PID autotune tool

  This is a SEPARATE sketch from arduino_ros2_base_controller.ino. It does
  not include, import, or modify that sketch or any of its files -- all
  hardware driver sources here are independent copies living only in this
  folder, so nothing about the production controller changes.

  Purpose: find fractional-order PID gains (kp, ki, kd, lambda, mu -- see
  PI^lambda D^mu in WheelPID, AutotuneTypes.h) for a single wheel's speed
  loop by running repeated step-response trials at a target speed and
  adjusting gains with a twiddle (coordinate descent) search that minimises
  a cost combining settling speed, overshoot and steady-state error. Same
  PID struct, feed-forward curve and constants as the production sketch, so
  results are meant to be pasted straight back into it.

  SAFETY: lift all four wheels off the ground before running AUTOTUNE.
  Each trial repeatedly drives one wheel up to the target speed; on the
  ground the robot will crawl into whatever is in front of it. Also note
  motor 4 has noticeably harder/higher-torque gearing than the other three,
  so it keeps drifting for longer after RELEASE -- that's why trials brake
  (not release) between runs (see REST_DURATION_MS/brakeMotor below).

  Hardware: QGPMaker motor shield + four QGPMaker quadrature encoders.
  No IMU is used by this tool.

  Serial: 500000 baud, 8-N-1. Commands are ASCII lines terminated by
  \n, \r or ':'.

  Commands:
    AUTOTUNE,<motor 1-4>[,<target_mm_s>[,<iterations>]]
      Runs the single-motor autotune search over kp,ki,kd,lambda,mu.
      Defaults: target_mm_s=220, iterations=8. Prints a TRIAL line per step
      test and a RESULT line with the final gains when done. Only the
      chosen motor moves.
    MATCH,[<target_mm_s>[,<accel_mm_s2>[,<max_wheel_mm_s>[,<hold_ms>[,<rounds>]]]]]
      Runs all four motors together through the same commanded ramp-up
      (at accel_mm_s2 to target_mm_s) then holds for hold_ms, and tunes
      each motor's kp,ki,kd,lambda,mu to minimise its speed-vs-time AND
      final-distance divergence from the other three -- i.e. matched
      acceleration, speed, timing and distance, not just an accurate
      individual step response. Defaults: target_mm_s=220, accel_mm_s2=150,
      max_wheel_mm_s=250, hold_ms=1000, rounds=6. Seeds from the per-motor
      gains already found by AUTOTUNE (see MATCH_SEED_GAINS below) rather
      than starting fresh.
    STOP / ESTOP
      Aborts any running autotune/match run and releases all motors.
    PING
      Replies PONG.

  Output after a completed AUTOTUNE run:
    RESULT,motor=<n>,kp=...,ki=...,kd=...,lambda=...,mu=...,cost=...
    PASTE,wheelPID[<index>].configure(kp, ki, kd, lambda, mu);
    TEST_CMD,PIDM,<n>,<kp>,<ki>,<kd>,<lambda>,<mu>
      Send TEST_CMD's payload verbatim to the production firmware's serial
      port to try the gains live (without reflashing) before committing
      them to arduino_ros2_base_controller.ino.

  Output after a completed MATCH run: one RESULT/PASTE/TEST_CMD triple per
  motor as above, plus:
    REPORT,max_distance_spread_mm=...,expected_distance_mm=...
      max_distance_spread_mm is the worst-case distance mismatch between any
      two wheels over the test run -- directly predicts how much the robot
      will drift off a straight line over that distance. expected_distance_mm
      is the analytic distance the commanded ramp/hold profile should have
      produced; compare it against each motor's own final_distance_mm to
      see whether the group is overshooting/undershooting the actual
      target, not just agreeing with each other.
*/

#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "QGPMaker_MotorShield.h"
#include "QGPMaker_Encoder.h"
#include "AutotuneTypes.h"

// -----------------------------------------------------------------------------
// Constants copied from arduino_ros2_base_controller.ino so results transfer
// directly. Keep these in sync with that sketch if the robot geometry or
// motor shield timing ever changes.
// -----------------------------------------------------------------------------

const uint8_t MOTOR_COUNT = 4;
const uint32_t SERIAL_BAUD = 500000UL;
const uint16_t CONTROL_PERIOD_MS = 10;   // 100 Hz control, matches production loop

const float WHEEL_DIAMETER_MM = 80.5f;

// Measured by hand-rotating each wheel exactly one full turn and reading the
// tick count (2026-07-02): 4326, 4226, 4138, 4334 -- motors 2 and 3 undercount
// noticeably relative to the nominal 4320, which (uncorrected) would make
// their measured speed under-report their true speed, so PID would keep
// driving them physically faster than commanded to compensate. Per-wheel
// values here instead of a single shared constant fix that at the source.
//by hand 1st time :4326.0f, 4226.0f, 4138.0f, 4334.0f
//by hand 2nd time : 4193.0f, 4176.0f, 4119.0f, 4330.0f
const float MEASURED_COUNTS_PER_REV[MOTOR_COUNT] = {4193.0f, 4176.0f, 4119.0f, 4330.0f};
const float MM_PER_COUNT[MOTOR_COUNT] = {
  (PI * WHEEL_DIAMETER_MM) / MEASURED_COUNTS_PER_REV[0],
  (PI * WHEEL_DIAMETER_MM) / MEASURED_COUNTS_PER_REV[1],
  (PI * WHEEL_DIAMETER_MM) / MEASURED_COUNTS_PER_REV[2],
  (PI * WHEEL_DIAMETER_MM) / MEASURED_COUNTS_PER_REV[3],
};

const float MAX_WHEEL_MM_S = 300.0f;
const uint8_t MIN_EFFECTIVE_PWM = 48;
const uint8_t MAX_DRIVE_PWM = 210;

// Motor order: 1 front-left, 2 front-right, 3 rear-right, 4 rear-left.
const int8_t ENCODER_SIGN[MOTOR_COUNT] = {-1, 1, 1, -1};

// -----------------------------------------------------------------------------
// Autotune tuning knobs
// -----------------------------------------------------------------------------

const float DEFAULT_TARGET_MM_S = 220.0f;
const uint8_t DEFAULT_ITERATIONS = 8;
const uint8_t MAX_ITERATIONS_ALLOWED = 30;

const uint16_t TRIAL_DURATION_MS = 1200;
// Active brake hold between trials, not a coast -- a released, higher-torque
// motor (e.g. motor 4's harder gearing) keeps drifting for longer than the
// others after RELEASE, so trials could start from inconsistent, still-moving
// wheels. Braking for a full second forces every motor to a real stop first.
const uint16_t REST_DURATION_MS = 1000;
const float STEADY_WINDOW_FRACTION = 0.3f;    // last 30% of trial used for steady-state avg

const float OVERSHOOT_COST_WEIGHT = 4.0f;
const float STEADY_ERROR_COST_WEIGHT = 6.0f;
const float NO_RISE_PENALTY = 5000.0f;
const float CONVERGENCE_THRESHOLD = 0.001f;   // sum of deltas below this stops early

// Start the search from the production sketch's shared baseline gains.
// lambda/mu start at 1.0 (classic PID) and let twiddle explore the
// fractional-order space from there.
const float INITIAL_KP = 0.25f;
const float INITIAL_KI = 0.034f;
const float INITIAL_KD = 0.003f;
const float INITIAL_LAMBDA = 1.0f;
const float INITIAL_MU = 1.0f;
const float INITIAL_DELTA_KP = 0.05f;
const float INITIAL_DELTA_KI = 0.01f;
const float INITIAL_DELTA_KD = 0.001f;
const float INITIAL_DELTA_LAMBDA = 0.05f;
const float INITIAL_DELTA_MU = 0.05f;

const float MIN_GAIN = 0.0f;
const float MAX_KP = 3.0f;
const float MAX_KI = 1.0f;
const float MAX_KD = 0.05f;
// Fractional orders are kept away from 0 (degenerate) and bounded well short
// of the classic stability limit (order 2) since twiddle can otherwise walk
// a live motor into an unstable combination mid-run.
const float MIN_LAMBDA = 0.4f;
const float MAX_LAMBDA = 1.4f;
const float MIN_MU = 0.4f;
const float MAX_MU = 1.4f;

// Number of tuned parameters per wheel: kp, ki, kd, lambda, mu.
const uint8_t PARAM_COUNT = 5;

// -----------------------------------------------------------------------------
// MATCH (group) tuning knobs
// -----------------------------------------------------------------------------

const float DEFAULT_MATCH_TARGET_MM_S = 220.0f;
const float DEFAULT_MATCH_ACCEL_MM_S2 = 150.0f;
const float DEFAULT_MATCH_MAX_WHEEL_MM_S = 250.0f;
const uint16_t DEFAULT_MATCH_HOLD_MS = 1000;
const uint8_t DEFAULT_MATCH_ROUNDS = 6;
const uint8_t MAX_MATCH_ROUNDS_ALLOWED = 15;

// How much a wheel's final cumulative distance is allowed to diverge from
// the other three's average before it dominates that wheel's cost -- this is
// the term that actually captures "same distance", since integrated speed
// error already captures "same acceleration/speed/timing".
const float MATCH_DISTANCE_WEIGHT = 8.0f;

// Anchors the group to the actual commanded ramp/hold profile, not just to
// each other. Without these, four wheels overshooting the target by roughly
// the same amount score just as well as four wheels tracking it accurately
// -- the search has no reason to prefer "right" over "wrong but agreeing".
const float MATCH_ABSOLUTE_TICK_WEIGHT = 1.0f;
const float MATCH_ABSOLUTE_DISTANCE_WEIGHT = 8.0f;

// Seeded from the results of running AUTOTUNE on each motor individually
// (2026-07-02), not from scratch -- refines rather than re-discovers.
// lambda/mu seeded at 1.0 (classic PID) until AUTOTUNE has been re-run with
// the fractional-order search and produces real per-motor values.
const float MATCH_SEED_GAINS[4][PARAM_COUNT] = {
  {0.3000f, 0.0340f, 0.0030f, 1.0f, 1.0f},  // motor 1 (front-left)
  {0.3000f, 0.0340f, 0.0030f, 1.0f, 1.0f},  // motor 2 (front-right)
  {0.3000f, 0.0240f, 0.0030f, 1.0f, 1.0f},  // motor 3 (rear-right)
  {0.2500f, 0.0240f, 0.006f, 1.0f, 1.0f},  // motor 4 (rear-left)
};
// Smaller than AUTOTUNE's initial deltas since MATCH is refining an
// already-decent starting point rather than searching from a blank slate.
const float MATCH_INITIAL_DELTA[PARAM_COUNT] = {0.01f, 0.001f, 0.0001f, 0.02f, 0.02f};

// -----------------------------------------------------------------------------
// Hardware
// -----------------------------------------------------------------------------

QGPMaker_MotorShield motorShield;
QGPMaker_Encoder encoder1(1);
QGPMaker_Encoder encoder2(2);
QGPMaker_Encoder encoder3(3);
QGPMaker_Encoder encoder4(4);

QGPMaker_DCMotor *getMotor(uint8_t index) {
  return motorShield.getMotor(index + 1);
}

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

void releaseAllMotors() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    QGPMaker_DCMotor *m = getMotor(i);
    m->setSpeed(0);
    m->run(RELEASE);
  }
}

// Active brake (shorts the H-bridge outputs) rather than RELEASE (coast).
// Used between trials so a higher-torque wheel can't keep drifting after
// its target drops to zero -- RELEASE alone lets it coast for longer than
// the other three, leaving trials starting from inconsistent wheel speeds.
void brakeMotor(uint8_t index) {
  QGPMaker_DCMotor *m = getMotor(index);
  m->setSpeed(0);
  m->run(BRAKE);
}

void brakeAllMotors() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) brakeMotor(i);
}

// Autotune only ever drives a wheel forward -- direction sign isn't part of
// what we're tuning here.
void applyMotorOutputForward(uint8_t index, float magnitudePWM) {
  uint8_t pwm = (uint8_t)constrain((int)(magnitudePWM + 0.5f), 0, 255);
  QGPMaker_DCMotor *m = getMotor(index);
  if (pwm < 1) {
    m->setSpeed(0);
    m->run(RELEASE);
    return;
  }
  m->run(FORWARD);
  m->setSpeed(pwm);
}

float speedFeedForwardPWM(float targetMMs, float maxWheelMMs) {
  float ratio = constrain(targetMMs / maxWheelMMs, 0.0f, 1.0f);
  return MIN_EFFECTIVE_PWM + ratio * (MAX_DRIVE_PWM - MIN_EFFECTIVE_PWM);
}

// -----------------------------------------------------------------------------
// PID controller (identical shape to the production WheelPID; struct is in
// AutotuneTypes.h so it's visible to the IDE's auto-generated prototypes)
// -----------------------------------------------------------------------------

// One instance per wheel: AUTOTUNE only ever drives wheelPID[motorIndex]
// (the other three stay idle); MATCH drives all four simultaneously.
WheelPID wheelPID[MOTOR_COUNT];

// -----------------------------------------------------------------------------
// Serial command handling
// -----------------------------------------------------------------------------

char commandBuffer[96];
uint8_t commandLength = 0;
bool abortRequested = false;

void printReady() {
  Serial.println(F("READY,Motor_autotune_v1"));
}

// Called continuously during a trial so STOP/ESTOP can interrupt a run that's
// mid-flight. Any other command received while a trial is running is dropped.
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

// -----------------------------------------------------------------------------
// Step-response trial (TrialResult struct lives in AutotuneTypes.h)
// -----------------------------------------------------------------------------

TrialResult runStepTrial(uint8_t motorIndex, float targetMMs) {
  TrialResult result = {0.0f, -1.0f, 0.0f, 0.0f, false};

  wheelPID[motorIndex].reset();
  readEncoderAndResetAtomic(motorIndex);
  float filteredSpeed = 0.0f;

  const uint32_t start = millis();
  uint32_t last = start;
  float peak = 0.0f;
  bool riseRecorded = false;
  const float riseThreshold = targetMMs * 0.9f;

  float steadySum = 0.0f;
  uint32_t steadyCount = 0;
  const uint32_t steadyStartMs = (uint32_t)(TRIAL_DURATION_MS * (1.0f - STEADY_WINDOW_FRACTION));

  while (millis() - start < TRIAL_DURATION_MS) {
    if (pollAbort()) {
      result.aborted = true;
      break;
    }

    uint32_t now = millis();
    if (now - last >= CONTROL_PERIOD_MS) {
      float dt = (now - last) * 0.001f;
      last = now;
      dt = constrain(dt, 0.005f, 0.050f);

      const int32_t rawTicks = readEncoderAndResetAtomic(motorIndex);
      const float signedTicks = rawTicks * ENCODER_SIGN[motorIndex];
      const float rawSpeed = (signedTicks * MM_PER_COUNT[motorIndex]) / dt;
      filteredSpeed += 0.35f * (rawSpeed - filteredSpeed);

      const float error = targetMMs - filteredSpeed;
      const float correction = wheelPID[motorIndex].update(targetMMs, filteredSpeed, dt);
      const float feedForward = speedFeedForwardPWM(targetMMs, MAX_WHEEL_MM_S);
      const float out = constrain(feedForward + correction, 0.0f, 255.0f);
      applyMotorOutputForward(motorIndex, out);

      const uint32_t elapsedMs = now - start;
      result.cost += fabs(error) * (elapsedMs * 0.001f) * dt;

      if (filteredSpeed > peak) peak = filteredSpeed;
      if (!riseRecorded && filteredSpeed >= riseThreshold) {
        result.riseTimeS = elapsedMs * 0.001f;
        riseRecorded = true;
      }

      if (elapsedMs >= steadyStartMs) {
        steadySum += filteredSpeed;
        ++steadyCount;
      }
    }
  }

  brakeMotor(motorIndex);

  result.overshootPct = (peak > targetMMs) ? ((peak - targetMMs) / targetMMs * 100.0f) : 0.0f;
  if (steadyCount > 0) {
    result.steadyStateErrorMMs = targetMMs - (steadySum / steadyCount);
  } else {
    result.steadyStateErrorMMs = targetMMs - filteredSpeed;
  }
  if (!riseRecorded) {
    result.cost += NO_RISE_PENALTY;
  }
  result.cost += OVERSHOOT_COST_WEIGHT * result.overshootPct;
  result.cost += STEADY_ERROR_COST_WEIGHT * fabs(result.steadyStateErrorMMs);

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

float evaluate(uint8_t motorIndex, float targetMMs, float kp, float ki, float kd,
               float lambda, float mu, TrialResult *outResult) {
  wheelPID[motorIndex].configure(kp, ki, kd, lambda, mu);
  TrialResult r = runStepTrial(motorIndex, targetMMs);
  if (outResult) *outResult = r;
  return r.cost;
}

// -----------------------------------------------------------------------------
// Twiddle (coordinate descent) search and reporting
// -----------------------------------------------------------------------------

void printTrial(int iter, int paramIndex, const float params[PARAM_COUNT], const TrialResult &r, float cost, bool isBaseline) {
  const char *name = "base";
  if (paramIndex == 0) name = "kp";
  else if (paramIndex == 1) name = "ki";
  else if (paramIndex == 2) name = "kd";
  else if (paramIndex == 3) name = "lambda";
  else if (paramIndex == 4) name = "mu";

  Serial.print(isBaseline ? F("BASELINE,") : F("TRIAL,"));
  Serial.print(F("iter=")); Serial.print(iter);
  Serial.print(F(",param=")); Serial.print(name);
  Serial.print(F(",kp=")); Serial.print(params[0], 4);
  Serial.print(F(",ki=")); Serial.print(params[1], 4);
  Serial.print(F(",kd=")); Serial.print(params[2], 4);
  Serial.print(F(",lambda=")); Serial.print(params[3], 4);
  Serial.print(F(",mu=")); Serial.print(params[4], 4);
  Serial.print(F(",cost=")); Serial.print(cost, 3);
  Serial.print(F(",rise_s=")); Serial.print(r.riseTimeS, 3);
  Serial.print(F(",overshoot_pct=")); Serial.print(r.overshootPct, 1);
  Serial.print(F(",steady_err_mm_s=")); Serial.println(r.steadyStateErrorMMs, 2);
}

void finishAborted() {
  releaseAllMotors();
  Serial.println(F("ABORTED,AUTOTUNE_STOPPED"));
}

void runAutotune(uint8_t motorIndex, float targetMMs, uint8_t maxIterations) {
  abortRequested = false;
  releaseAllMotors();

  float params[PARAM_COUNT] = {INITIAL_KP, INITIAL_KI, INITIAL_KD, INITIAL_LAMBDA, INITIAL_MU};
  float deltas[PARAM_COUNT] = {INITIAL_DELTA_KP, INITIAL_DELTA_KI, INITIAL_DELTA_KD,
                                INITIAL_DELTA_LAMBDA, INITIAL_DELTA_MU};
  const float minParam[PARAM_COUNT] = {MIN_GAIN, MIN_GAIN, MIN_GAIN, MIN_LAMBDA, MIN_MU};
  const float maxParam[PARAM_COUNT] = {MAX_KP, MAX_KI, MAX_KD, MAX_LAMBDA, MAX_MU};

  Serial.print(F("INFO,AUTOTUNE_START,motor="));
  Serial.print(motorIndex + 1);
  Serial.print(F(",target_mm_s="));
  Serial.print(targetMMs, 1);
  Serial.print(F(",iterations="));
  Serial.println(maxIterations);

  TrialResult baseline;
  float bestCost = evaluate(motorIndex, targetMMs, params[0], params[1], params[2], params[3], params[4], &baseline);
  printTrial(0, -1, params, baseline, bestCost, true);
  if (baseline.aborted) {
    finishAborted();
    return;
  }

  for (uint8_t iter = 0; iter < maxIterations; ++iter) {
    for (uint8_t i = 0; i < PARAM_COUNT; ++i) {
      const float original = params[i];

      params[i] = constrain(original + deltas[i], minParam[i], maxParam[i]);
      TrialResult trialUp;
      float costUp = evaluate(motorIndex, targetMMs, params[0], params[1], params[2], params[3], params[4], &trialUp);
      printTrial(iter, i, params, trialUp, costUp, false);
      if (trialUp.aborted) {
        finishAborted();
        return;
      }

      if (costUp < bestCost) {
        bestCost = costUp;
        deltas[i] *= 1.1f;
        continue;
      }

      params[i] = constrain(original - deltas[i], minParam[i], maxParam[i]);
      TrialResult trialDown;
      float costDown = evaluate(motorIndex, targetMMs, params[0], params[1], params[2], params[3], params[4], &trialDown);
      printTrial(iter, i, params, trialDown, costDown, false);
      if (trialDown.aborted) {
        finishAborted();
        return;
      }

      if (costDown < bestCost) {
        bestCost = costDown;
        deltas[i] *= 1.05f;
      } else {
        params[i] = original;
        deltas[i] *= 0.9f;
      }
    }

    float deltaSum = 0.0f;
    for (uint8_t i = 0; i < PARAM_COUNT; ++i) deltaSum += deltas[i];
    if (deltaSum < CONVERGENCE_THRESHOLD) {
      Serial.println(F("INFO,AUTOTUNE_CONVERGED_EARLY"));
      break;
    }
  }

  releaseAllMotors();

  Serial.print(F("RESULT,motor="));
  Serial.print(motorIndex + 1);
  Serial.print(F(",kp="));
  Serial.print(params[0], 4);
  Serial.print(F(",ki="));
  Serial.print(params[1], 4);
  Serial.print(F(",kd="));
  Serial.print(params[2], 4);
  Serial.print(F(",lambda="));
  Serial.print(params[3], 4);
  Serial.print(F(",mu="));
  Serial.print(params[4], 4);
  Serial.print(F(",cost="));
  Serial.println(bestCost, 3);

  Serial.print(F("PASTE,wheelPID["));
  Serial.print(motorIndex);
  Serial.print(F("].configure("));
  Serial.print(params[0], 4);
  Serial.print(F(", "));
  Serial.print(params[1], 4);
  Serial.print(F(", "));
  Serial.print(params[2], 4);
  Serial.print(F(", "));
  Serial.print(params[3], 4);
  Serial.print(F(", "));
  Serial.print(params[4], 4);
  Serial.println(F(");"));

  Serial.print(F("TEST_CMD,PIDM,"));
  Serial.print(motorIndex + 1);
  Serial.print(F(","));
  Serial.print(params[0], 4);
  Serial.print(F(","));
  Serial.print(params[1], 4);
  Serial.print(F(","));
  Serial.print(params[2], 4);
  Serial.print(F(","));
  Serial.print(params[3], 4);
  Serial.print(F(","));
  Serial.println(params[4], 4);
}

// -----------------------------------------------------------------------------
// MATCH (group) trial and twiddle search
//
// Runs all four wheels together through the same commanded ramp-up/hold
// profile. Each motor's cost is its divergence -- summed over the whole run,
// plus a heavily-weighted final-distance term -- from the *other three*
// wheels' average, not from the commanded target directly. That's what
// makes this "match the group" instead of "hit my own setpoint".
// -----------------------------------------------------------------------------

MatchTrialResult runMatchTrial(float targetMMs, float accelMMs2, float maxWheelMMs, uint16_t holdMs) {
  MatchTrialResult result;
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    result.cost[i] = 0.0f;
    result.finalDistanceMM[i] = 0.0f;
  }
  result.aborted = false;

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    wheelPID[i].reset();
    readEncoderAndResetAtomic(i);
  }

  float filteredSpeed[MOTOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
  float distanceMM[MOTOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
  float rampedTarget = 0.0f;

  const uint32_t rampDurationMs = (accelMMs2 > 0.0f)
    ? (uint32_t)((targetMMs / accelMMs2) * 1000.0f)
    : 0;
  const uint32_t totalDurationMs = rampDurationMs + holdMs;

  // Analytic ground truth for the commanded profile: area under the ramp
  // (a triangle, 0 to targetMMs) plus the hold at full target speed.
  const float expectedDistanceMM =
    0.5f * targetMMs * (rampDurationMs * 0.001f) + targetMMs * (holdMs * 0.001f);

  const uint32_t start = millis();
  uint32_t last = start;

  while (millis() - start < totalDurationMs) {
    if (pollAbort()) {
      result.aborted = true;
      break;
    }

    uint32_t now = millis();
    if (now - last >= CONTROL_PERIOD_MS) {
      float dt = (now - last) * 0.001f;
      last = now;
      dt = constrain(dt, 0.005f, 0.050f);

      const float maxStep = accelMMs2 * dt;
      if (rampedTarget < targetMMs) {
        rampedTarget = min(rampedTarget + maxStep, targetMMs);
      }

      for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        const int32_t rawTicks = readEncoderAndResetAtomic(i);
        const float signedTicks = rawTicks * ENCODER_SIGN[i];
        const float distStep = signedTicks * MM_PER_COUNT[i];
        distanceMM[i] += distStep;
        const float rawSpeed = distStep / dt;
        filteredSpeed[i] += 0.35f * (rawSpeed - filteredSpeed[i]);
      }

      const uint32_t elapsedMs = now - start;
      for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        float sumOthers = 0.0f;
        for (uint8_t j = 0; j < MOTOR_COUNT; ++j) {
          if (j != i) sumOthers += filteredSpeed[j];
        }
        const float othersMean = sumOthers / (MOTOR_COUNT - 1);
        const float diff = filteredSpeed[i] - othersMean;
        result.cost[i] += fabs(diff) * (elapsedMs * 0.001f) * dt;

        // Pulls toward the actual commanded ramp, so the group can't drift
        // to a mutually-agreeing but wrong speed.
        result.cost[i] += MATCH_ABSOLUTE_TICK_WEIGHT * fabs(filteredSpeed[i] - rampedTarget) * dt;
      }

      for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        const float correction = wheelPID[i].update(rampedTarget, filteredSpeed[i], dt);
        const float feedForward = speedFeedForwardPWM(rampedTarget, maxWheelMMs);
        const float out = constrain(feedForward + correction, 0.0f, 255.0f);
        applyMotorOutputForward(i, out);
      }
    }
  }

  brakeAllMotors();

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    float sumOthersDist = 0.0f;
    for (uint8_t j = 0; j < MOTOR_COUNT; ++j) {
      if (j != i) sumOthersDist += distanceMM[j];
    }
    const float othersMeanDist = sumOthersDist / (MOTOR_COUNT - 1);
    result.finalDistanceMM[i] = distanceMM[i];
    result.cost[i] += MATCH_DISTANCE_WEIGHT * fabs(distanceMM[i] - othersMeanDist);
    result.cost[i] += MATCH_ABSOLUTE_DISTANCE_WEIGHT * fabs(distanceMM[i] - expectedDistanceMM);
  }

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

MatchTrialResult runMatchTrialWithParams(float params[4][PARAM_COUNT], float targetMMs, float accelMMs2,
                                          float maxWheelMMs, uint16_t holdMs) {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    wheelPID[i].configure(params[i][0], params[i][1], params[i][2], params[i][3], params[i][4]);
  }
  return runMatchTrial(targetMMs, accelMMs2, maxWheelMMs, holdMs);
}

void printMatchTrial(int round, int motorIndex, const float params[4][PARAM_COUNT], const MatchTrialResult &r, bool isBaseline) {
  Serial.print(isBaseline ? F("MBASELINE,") : F("MTRIAL,"));
  Serial.print(F("round=")); Serial.print(round);
  Serial.print(F(",motor="));
  if (motorIndex < 0) Serial.print(F("-"));
  else Serial.print(motorIndex + 1);

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    Serial.print(F(",kp")); Serial.print(i + 1); Serial.print(F("=")); Serial.print(params[i][0], 4);
    Serial.print(F(",ki")); Serial.print(i + 1); Serial.print(F("=")); Serial.print(params[i][1], 4);
    Serial.print(F(",kd")); Serial.print(i + 1); Serial.print(F("=")); Serial.print(params[i][2], 4);
    Serial.print(F(",lambda")); Serial.print(i + 1); Serial.print(F("=")); Serial.print(params[i][3], 4);
    Serial.print(F(",mu")); Serial.print(i + 1); Serial.print(F("=")); Serial.print(params[i][4], 4);
    Serial.print(F(",cost")); Serial.print(i + 1); Serial.print(F("=")); Serial.print(r.cost[i], 2);
  }
  Serial.println();
}

void runMatchTune(float targetMMs, float accelMMs2, float maxWheelMMs, uint16_t holdMs, uint8_t rounds) {
  abortRequested = false;
  releaseAllMotors();

  float params[MOTOR_COUNT][PARAM_COUNT];
  float deltas[MOTOR_COUNT][PARAM_COUNT];
  float bestCost[MOTOR_COUNT];
  const float minParam[PARAM_COUNT] = {MIN_GAIN, MIN_GAIN, MIN_GAIN, MIN_LAMBDA, MIN_MU};
  const float maxParam[PARAM_COUNT] = {MAX_KP, MAX_KI, MAX_KD, MAX_LAMBDA, MAX_MU};

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    for (uint8_t k = 0; k < PARAM_COUNT; ++k) {
      params[i][k] = MATCH_SEED_GAINS[i][k];
      deltas[i][k] = MATCH_INITIAL_DELTA[k];
    }
  }

  Serial.print(F("INFO,MATCH_START,target_mm_s="));
  Serial.print(targetMMs, 1);
  Serial.print(F(",accel_mm_s2="));
  Serial.print(accelMMs2, 1);
  Serial.print(F(",max_wheel_mm_s="));
  Serial.print(maxWheelMMs, 1);
  Serial.print(F(",hold_ms="));
  Serial.print(holdMs);
  Serial.print(F(",rounds="));
  Serial.println(rounds);

  MatchTrialResult baseline = runMatchTrialWithParams(params, targetMMs, accelMMs2, maxWheelMMs, holdMs);
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) bestCost[i] = baseline.cost[i];
  printMatchTrial(-1, -1, params, baseline, true);
  if (baseline.aborted) {
    finishAborted();
    return;
  }

  for (uint8_t round = 0; round < rounds; ++round) {
    for (uint8_t m = 0; m < MOTOR_COUNT; ++m) {
      for (uint8_t k = 0; k < PARAM_COUNT; ++k) {
        const float original = params[m][k];

        params[m][k] = constrain(original + deltas[m][k], minParam[k], maxParam[k]);
        MatchTrialResult trialUp = runMatchTrialWithParams(params, targetMMs, accelMMs2, maxWheelMMs, holdMs);
        printMatchTrial(round, m, params, trialUp, false);
        if (trialUp.aborted) {
          finishAborted();
          return;
        }

        if (trialUp.cost[m] < bestCost[m]) {
          bestCost[m] = trialUp.cost[m];
          deltas[m][k] *= 1.1f;
          continue;
        }

        params[m][k] = constrain(original - deltas[m][k], minParam[k], maxParam[k]);
        MatchTrialResult trialDown = runMatchTrialWithParams(params, targetMMs, accelMMs2, maxWheelMMs, holdMs);
        printMatchTrial(round, m, params, trialDown, false);
        if (trialDown.aborted) {
          finishAborted();
          return;
        }

        if (trialDown.cost[m] < bestCost[m]) {
          bestCost[m] = trialDown.cost[m];
          deltas[m][k] *= 1.05f;
        } else {
          params[m][k] = original;
          deltas[m][k] *= 0.9f;
        }
      }
    }

    float deltaSum = 0.0f;
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
      for (uint8_t k = 0; k < PARAM_COUNT; ++k) deltaSum += deltas[i][k];
    }
    if (deltaSum < CONVERGENCE_THRESHOLD * MOTOR_COUNT) {
      Serial.println(F("INFO,MATCH_CONVERGED_EARLY"));
      break;
    }
  }

  releaseAllMotors();

  // One clean confirmation run at the converged gains, used for the report.
  MatchTrialResult final = runMatchTrialWithParams(params, targetMMs, accelMMs2, maxWheelMMs, holdMs);
  releaseAllMotors();

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    Serial.print(F("RESULT,motor="));
    Serial.print(i + 1);
    Serial.print(F(",kp="));
    Serial.print(params[i][0], 4);
    Serial.print(F(",ki="));
    Serial.print(params[i][1], 4);
    Serial.print(F(",kd="));
    Serial.print(params[i][2], 4);
    Serial.print(F(",lambda="));
    Serial.print(params[i][3], 4);
    Serial.print(F(",mu="));
    Serial.print(params[i][4], 4);
    Serial.print(F(",cost="));
    Serial.print(final.cost[i], 3);
    Serial.print(F(",final_distance_mm="));
    Serial.println(final.finalDistanceMM[i], 2);

    Serial.print(F("PASTE,wheelPID["));
    Serial.print(i);
    Serial.print(F("].configure("));
    Serial.print(params[i][0], 4);
    Serial.print(F(", "));
    Serial.print(params[i][1], 4);
    Serial.print(F(", "));
    Serial.print(params[i][2], 4);
    Serial.print(F(", "));
    Serial.print(params[i][3], 4);
    Serial.print(F(", "));
    Serial.print(params[i][4], 4);
    Serial.println(F(");"));

    Serial.print(F("TEST_CMD,PIDM,"));
    Serial.print(i + 1);
    Serial.print(F(","));
    Serial.print(params[i][0], 4);
    Serial.print(F(","));
    Serial.print(params[i][1], 4);
    Serial.print(F(","));
    Serial.print(params[i][2], 4);
    Serial.print(F(","));
    Serial.print(params[i][3], 4);
    Serial.print(F(","));
    Serial.println(params[i][4], 4);
  }

  float maxDist = final.finalDistanceMM[0];
  float minDist = final.finalDistanceMM[0];
  for (uint8_t i = 1; i < MOTOR_COUNT; ++i) {
    if (final.finalDistanceMM[i] > maxDist) maxDist = final.finalDistanceMM[i];
    if (final.finalDistanceMM[i] < minDist) minDist = final.finalDistanceMM[i];
  }
  const uint32_t rampDurationMs = (accelMMs2 > 0.0f)
    ? (uint32_t)((targetMMs / accelMMs2) * 1000.0f)
    : 0;
  const float expectedDistanceMM =
    0.5f * targetMMs * (rampDurationMs * 0.001f) + targetMMs * (holdMs * 0.001f);

  Serial.print(F("REPORT,max_distance_spread_mm="));
  Serial.print(maxDist - minDist, 2);
  Serial.print(F(",expected_distance_mm="));
  Serial.println(expectedDistanceMM, 2);
}

// -----------------------------------------------------------------------------
// Command parser
// -----------------------------------------------------------------------------

void processCommand(char *line) {
  char *savePtr = NULL;
  char *command = strtok_r(line, ",", &savePtr);
  if (command == NULL) return;

  for (char *p = command; *p; ++p) {
    if (*p >= 'a' && *p <= 'z') *p = *p - ('a' - 'A');
  }

  if (strcmp(command, "AUTOTUNE") == 0) {
    char *motorText = strtok_r(NULL, ",", &savePtr);
    char *targetText = strtok_r(NULL, ",", &savePtr);
    char *iterText = strtok_r(NULL, ",", &savePtr);

    int motorNum = motorText ? atoi(motorText) : -1;
    if (motorNum < 1 || motorNum > MOTOR_COUNT) {
      Serial.println(F("ERR,AUTOTUNE_FORMAT,expected_AUTOTUNE_motor_1to4"));
      return;
    }

    float targetMMs = targetText ? atof(targetText) : DEFAULT_TARGET_MM_S;
    targetMMs = constrain(fabs(targetMMs), 30.0f, MAX_WHEEL_MM_S);

    int iterations = iterText ? atoi(iterText) : DEFAULT_ITERATIONS;
    if (iterations < 1) iterations = DEFAULT_ITERATIONS;
    if (iterations > MAX_ITERATIONS_ALLOWED) iterations = MAX_ITERATIONS_ALLOWED;

    runAutotune((uint8_t)(motorNum - 1), targetMMs, (uint8_t)iterations);
  }
  else if (strcmp(command, "MATCH") == 0) {
    char *targetText = strtok_r(NULL, ",", &savePtr);
    char *accelText = strtok_r(NULL, ",", &savePtr);
    char *maxWheelText = strtok_r(NULL, ",", &savePtr);
    char *holdText = strtok_r(NULL, ",", &savePtr);
    char *roundsText = strtok_r(NULL, ",", &savePtr);

    float maxWheelMMs = maxWheelText ? atof(maxWheelText) : DEFAULT_MATCH_MAX_WHEEL_MM_S;
    if (maxWheelMMs < 10.0f) maxWheelMMs = DEFAULT_MATCH_MAX_WHEEL_MM_S;

    float targetMMs = targetText ? atof(targetText) : DEFAULT_MATCH_TARGET_MM_S;
    targetMMs = constrain(fabs(targetMMs), 10.0f, maxWheelMMs);

    float accelMMs2 = accelText ? atof(accelText) : DEFAULT_MATCH_ACCEL_MM_S2;
    if (accelMMs2 < 10.0f) accelMMs2 = DEFAULT_MATCH_ACCEL_MM_S2;

    uint16_t holdMs = holdText ? (uint16_t)atoi(holdText) : DEFAULT_MATCH_HOLD_MS;
    if (holdMs < 100) holdMs = DEFAULT_MATCH_HOLD_MS;

    int rounds = roundsText ? atoi(roundsText) : DEFAULT_MATCH_ROUNDS;
    if (rounds < 1) rounds = DEFAULT_MATCH_ROUNDS;
    if (rounds > MAX_MATCH_ROUNDS_ALLOWED) rounds = MAX_MATCH_ROUNDS_ALLOWED;

    runMatchTune(targetMMs, accelMMs2, maxWheelMMs, holdMs, (uint8_t)rounds);
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

void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  Wire.setClock(400000UL);

  motorShield.begin(500);
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) getMotor(i);
  releaseAllMotors();

  // Idle-time default so wheelPID isn't left zero-initialised; AUTOTUNE and
  // MATCH both reconfigure whichever motors they run before driving them.
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    wheelPID[i].configure(MATCH_SEED_GAINS[i][0], MATCH_SEED_GAINS[i][1], MATCH_SEED_GAINS[i][2],
                          MATCH_SEED_GAINS[i][3], MATCH_SEED_GAINS[i][4]);
  }

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) readEncoderAndResetAtomic(i);

  printReady();
  Serial.println(F("INFO,LIFT_ALL_WHEELS_OFF_THE_GROUND_BEFORE_AUTOTUNE"));
}

void loop() {
  readSerialCommands();
}
