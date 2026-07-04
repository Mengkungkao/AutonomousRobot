/*
  mDetect standalone per-wheel raw calibration data tool

  This is a SEPARATE sketch from arduino_ros2_base_controller.ino. It does
  not include, import, or modify that sketch or any of its files -- all
  hardware driver sources here are independent copies living only in this
  folder, so nothing about the production controller changes.

  Purpose: gather raw, PID-free per-motor data for manual analysis and
  hand-tuning -- no automatic gain search runs here. Two data sources:
  CALIBRATE (steady-state PWM-to-speed sweep) and OPENLOOP (dynamic step
  response at a target speed, feed-forward only). Both drive motors
  directly off the feed-forward curve/fixed PWM with zero PID correction,
  so the data reflects only real motor+feed-forward behaviour.

  SAFETY: lift all four wheels off the ground before running CALIBRATE or
  OPENLOOP. Each trial repeatedly drives one wheel; on the ground the robot
  will crawl into whatever is in front of it. Also note motor 4 has
  noticeably harder/higher-torque gearing than the other three, so it keeps
  drifting for longer after RELEASE -- that's why trials brake (not
  release) between runs (see REST_DURATION_MS/brakeMotor below).

  Hardware: QGPMaker motor shield + four QGPMaker quadrature encoders.
  No IMU is used by this tool.

  Serial: 500000 baud, 8-N-1. Commands are ASCII lines terminated by
  \n, \r or ':'.

  Commands:
    CALIBRATE,<motor 1-4>[,<pwm_step>[,<hold_ms>]]
      Open-loop (no PID) PWM sweep from MIN_EFFECTIVE_PWM to MAX_DRIVE_PWM
      on one motor, step size pwm_step, holding each step for hold_ms and
      measuring steady-state speed from the encoder. Reports each step's
      speed against what that motor's own calibrated feed-forward curve
      (FF_SLOPE_MM_S_PER_PWM/FF_INTERCEPT_MM_S, used by
      speedFeedForwardPWM) predicts for that PWM, so any remaining gap
      -- e.g. from the real response saturating at high PWM, which a
      straight-line fit doesn't capture -- is visible directly. Re-run
      after updating those constants to check the new fit. Defaults:
      pwm_step=20, hold_ms=1000. Only the chosen motor moves.
    OPENLOOP,<motor 1-4>[,<target_mm_s>]
      Runs one dynamic step test at target_mm_s, feed-forward PWM only
      (correction forced to 0 -- no PID involved at all), printing one
      SAMPLE line per 10ms control tick (raw step response, for offline
      analysis/plotting) plus a summary OPENLOOP_RESULT line. Defaults:
      target_mm_s=220. Only the chosen motor moves.
    STOP / ESTOP
      Aborts any running trial and releases all motors.
    PING
      Replies PONG.

  Output during each OPENLOOP step test (one line per control tick):
    SAMPLE,trial=<n>,motor=<n>,t_ms=...,target_mm_s=...,measured_mm_s=...,
      error_mm_s=...,ff_pwm=...,correction_pwm=0.0,pwm=...
      trial= matches the "trial=" field on the OPENLOOP_RESULT summary
      line printed right after this step test. correction_pwm is always 0
      here (kept in the output for format parity with a PID-driven trial);
      pwm is just the clamped ff_pwm.

  Output after each OPENLOOP step test:
    OPENLOOP_RESULT,trial=<n>,motor=<n>,target_mm_s=...,rise_s=...,
      overshoot_pct=...,steady_err_mm_s=...,cost=...
      Same shape as AUTOTUNE's old per-trial summary, minus the gains --
      there's nothing being searched, this is purely descriptive.

  Output after each CALIBRATE step:
    CAL,motor=<n>,pwm=...,measured_mm_s=...,expected_mm_s=...,
      error_mm_s=...,error_pct=...
      expected_mm_s is what speedFeedForwardPWM's linear PWM-to-speed
      assumption predicts for that PWM; error_mm_s/error_pct is
      measured_mm_s's deviation from it. Ends with INFO,CALIBRATE_DONE.
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
const uint8_t MIN_EFFECTIVE_PWM = 50;
const uint8_t MAX_DRIVE_PWM = 210;

// Motor order: 1 front-left, 2 front-right, 3 rear-right, 4 rear-left.
const int8_t ENCODER_SIGN[MOTOR_COUNT] = {-1, 1, 1, -1};

// Per-motor open-loop PWM-to-speed linear fit (measured_mm_s = slope*pwm +
// intercept), from a CALIBRATE,<motor>,10,1000 sweep (2026-07-04, PWM
// 48-208). Keep in sync with arduino_ros2_base_controller.ino's copy of
// the same constants -- real motors diverge hugely from a single shared
// curve (motor 4 needs less than half the PWM of motors 2/3 for the same
// speed), so each gets its own inverse mapping in speedFeedForwardPWM
// below. Linear fit residual RMSE is ~15-27 mm/s (real response saturates
// a bit at high PWM); PID still corrects the remainder.
const float FF_SLOPE_MM_S_PER_PWM[MOTOR_COUNT] = {2.557f, 2.631f, 2.4945f, 2.3404f};
const float FF_INTERCEPT_MM_S[MOTOR_COUNT] = {-83.25f, -143.56f, -149.38f, 6.14f};

// -----------------------------------------------------------------------------
// OPENLOOP step-trial knobs
// -----------------------------------------------------------------------------

const float DEFAULT_TARGET_MM_S = 220.0f;

const uint16_t TRIAL_DURATION_MS = 1200;
// Active brake hold between trials, not a coast -- a released, higher-torque
// motor (e.g. motor 4's harder gearing) keeps drifting for longer than the
// others after RELEASE, so trials could start from inconsistent, still-moving
// wheels. Braking for a full second forces every motor to a real stop first.
const uint16_t REST_DURATION_MS = 1000;
const float STEADY_WINDOW_FRACTION = 0.3f;    // last 30% of trial used for steady-state avg

// Used only to compute the descriptive "cost" field in OPENLOOP_RESULT --
// no search runs here, so these just weight the summary metric.
const float OVERSHOOT_COST_WEIGHT = 4.0f;
const float STEADY_ERROR_COST_WEIGHT = 6.0f;
const float NO_RISE_PENALTY = 5000.0f;

// -----------------------------------------------------------------------------
// CALIBRATE tuning knobs
// -----------------------------------------------------------------------------

const uint8_t DEFAULT_CAL_PWM_STEP = 20;
const uint16_t DEFAULT_CAL_HOLD_MS = 1000;
// Last fraction of each step's hold used for the steady-state speed average,
// same idea as STEADY_WINDOW_FRACTION above.
const float CAL_STEADY_WINDOW_FRACTION = 0.3f;

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

// Trials only ever drive a wheel forward -- direction sign isn't part of
// what's being measured here.
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

float speedFeedForwardPWM(float targetMMs, uint8_t motorIndex) {
  const float pwm = (targetMMs - FF_INTERCEPT_MM_S[motorIndex]) / FF_SLOPE_MM_S_PER_PWM[motorIndex];
  return constrain(pwm, MIN_EFFECTIVE_PWM, MAX_DRIVE_PWM);
}

// -----------------------------------------------------------------------------
// Serial command handling
// -----------------------------------------------------------------------------

char commandBuffer[96];
uint8_t commandLength = 0;
bool abortRequested = false;

// Incremented once per trial (OPENLOOP step test) so each trial's raw
// SAMPLE stream can be joined back to its OPENLOOP_RESULT summary line by
// "trial=".
uint16_t trialCounter = 0;

void printReady() {
  Serial.println(F("READY,Motor_calibration_v1"));
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

void finishAborted() {
  releaseAllMotors();
  Serial.println(F("ABORTED,STOPPED"));
}

// -----------------------------------------------------------------------------
// OPENLOOP: dynamic step-response trial, feed-forward only (no PID at all)
// -----------------------------------------------------------------------------

TrialResult runOpenLoopTrial(uint8_t motorIndex, float targetMMs) {
  TrialResult result = {0.0f, -1.0f, 0.0f, 0.0f, false, ++trialCounter};

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
      const float feedForward = speedFeedForwardPWM(targetMMs, motorIndex);
      const float out = constrain(feedForward, 0.0f, 255.0f);
      applyMotorOutputForward(motorIndex, out);

      const uint32_t elapsedMs = now - start;

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
      Serial.print(feedForward, 1);
      Serial.print(F(",correction_pwm="));
      Serial.print(0.0f, 1);
      Serial.print(F(",pwm="));
      Serial.println(out, 1);

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

void printOpenLoopResult(uint8_t motorIndex, float targetMMs, const TrialResult &r) {
  Serial.print(F("OPENLOOP_RESULT,trial="));
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

void runOpenLoop(uint8_t motorIndex, float targetMMs) {
  abortRequested = false;
  releaseAllMotors();

  Serial.print(F("INFO,OPENLOOP_START,motor="));
  Serial.print(motorIndex + 1);
  Serial.print(F(",target_mm_s="));
  Serial.println(targetMMs, 1);

  TrialResult result = runOpenLoopTrial(motorIndex, targetMMs);
  if (result.aborted) {
    finishAborted();
    return;
  }

  releaseAllMotors();
  printOpenLoopResult(motorIndex, targetMMs, result);
  Serial.print(F("INFO,OPENLOOP_DONE,motor="));
  Serial.println(motorIndex + 1);
}

// -----------------------------------------------------------------------------
// CALIBRATE: open-loop PWM sweep and error reporting
//
// Drives one motor directly at a fixed PWM (no PID) and measures the speed
// it actually settles at. This is ground truth for how much the production
// feed-forward curve's linear PWM<->speed assumption (speedFeedForwardPWM)
// diverges from a given motor's real response -- basic error data for
// judging how accurately that motor can be controlled open-loop before PID
// correction ever engages.
// -----------------------------------------------------------------------------

CalStepResult runCalibrationStep(uint8_t motorIndex, uint8_t pwm, uint16_t holdMs) {
  CalStepResult result = {0.0f, false};

  readEncoderAndResetAtomic(motorIndex);
  float filteredSpeed = 0.0f;
  applyMotorOutputForward(motorIndex, pwm);

  const uint32_t start = millis();
  uint32_t last = start;

  float steadySum = 0.0f;
  uint32_t steadyCount = 0;
  const uint32_t steadyStartMs = (uint32_t)(holdMs * (1.0f - CAL_STEADY_WINDOW_FRACTION));

  while (millis() - start < holdMs) {
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

      const uint32_t elapsedMs = now - start;
      if (elapsedMs >= steadyStartMs) {
        steadySum += filteredSpeed;
        ++steadyCount;
      }
    }
  }

  brakeMotor(motorIndex);
  result.measuredMMs = (steadyCount > 0) ? (steadySum / steadyCount) : filteredSpeed;

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

void runCalibration(uint8_t motorIndex, uint8_t pwmStep, uint16_t holdMs) {
  abortRequested = false;
  releaseAllMotors();

  Serial.print(F("INFO,CALIBRATE_START,motor="));
  Serial.print(motorIndex + 1);
  Serial.print(F(",pwm_step="));
  Serial.print(pwmStep);
  Serial.print(F(",hold_ms="));
  Serial.println(holdMs);

  for (uint16_t pwm = MIN_EFFECTIVE_PWM; pwm <= MAX_DRIVE_PWM; pwm += pwmStep) {
    CalStepResult step = runCalibrationStep(motorIndex, (uint8_t)pwm, holdMs);
    if (step.aborted) {
      finishAborted();
      return;
    }

    const float expectedMMs =
      FF_SLOPE_MM_S_PER_PWM[motorIndex] * (float)pwm + FF_INTERCEPT_MM_S[motorIndex];
    const float errorMMs = step.measuredMMs - expectedMMs;
    const float errorPct = (expectedMMs > 1.0f) ? (errorMMs / expectedMMs * 100.0f) : 0.0f;

    Serial.print(F("CAL,motor="));
    Serial.print(motorIndex + 1);
    Serial.print(F(",pwm="));
    Serial.print(pwm);
    Serial.print(F(",measured_mm_s="));
    Serial.print(step.measuredMMs, 2);
    Serial.print(F(",expected_mm_s="));
    Serial.print(expectedMMs, 2);
    Serial.print(F(",error_mm_s="));
    Serial.print(errorMMs, 2);
    Serial.print(F(",error_pct="));
    Serial.println(errorPct, 1);
  }

  releaseAllMotors();
  Serial.print(F("INFO,CALIBRATE_DONE,motor="));
  Serial.println(motorIndex + 1);
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

  if (strcmp(command, "CALIBRATE") == 0) {
    char *motorText = strtok_r(NULL, ",", &savePtr);
    char *stepText = strtok_r(NULL, ",", &savePtr);
    char *holdText = strtok_r(NULL, ",", &savePtr);

    int motorNum = motorText ? atoi(motorText) : -1;
    if (motorNum < 1 || motorNum > MOTOR_COUNT) {
      Serial.println(F("ERR,CALIBRATE_FORMAT,expected_CALIBRATE_motor_1to4"));
      return;
    }

    int pwmStep = stepText ? atoi(stepText) : DEFAULT_CAL_PWM_STEP;
    if (pwmStep < 1) pwmStep = DEFAULT_CAL_PWM_STEP;

    uint16_t holdMs = holdText ? (uint16_t)atoi(holdText) : DEFAULT_CAL_HOLD_MS;
    if (holdMs < 100) holdMs = DEFAULT_CAL_HOLD_MS;

    runCalibration((uint8_t)(motorNum - 1), (uint8_t)pwmStep, holdMs);
  }
  else if (strcmp(command, "OPENLOOP") == 0) {
    char *motorText = strtok_r(NULL, ",", &savePtr);
    char *targetText = strtok_r(NULL, ",", &savePtr);

    int motorNum = motorText ? atoi(motorText) : -1;
    if (motorNum < 1 || motorNum > MOTOR_COUNT) {
      Serial.println(F("ERR,OPENLOOP_FORMAT,expected_OPENLOOP_motor_1to4"));
      return;
    }

    float targetMMs = targetText ? atof(targetText) : DEFAULT_TARGET_MM_S;
    targetMMs = constrain(fabs(targetMMs), 30.0f, MAX_WHEEL_MM_S);

    runOpenLoop((uint8_t)(motorNum - 1), targetMMs);
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

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) readEncoderAndResetAtomic(i);

  printReady();
  Serial.println(F("INFO,LIFT_ALL_WHEELS_OFF_THE_GROUND_BEFORE_RUNNING_TRIALS"));
}

void loop() {
  readSerialCommands();
}
