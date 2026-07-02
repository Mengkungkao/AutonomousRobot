/*
  mDetect standalone per-wheel PID autotune tool

  This is a SEPARATE sketch from arduino_ros2_base_controller.ino. It does
  not include, import, or modify that sketch or any of its files -- all
  hardware driver sources here are independent copies living only in this
  folder, so nothing about the production controller changes.

  Purpose: find PID gains (kp, ki, kd) for a single wheel's speed loop by
  running repeated step-response trials at a target speed and adjusting
  gains with a twiddle (coordinate descent) search that minimises a cost
  combining settling speed, overshoot and steady-state error. Same PID
  struct, feed-forward curve and constants as the production sketch, so
  results are meant to be pasted straight back into it.

  SAFETY: lift all four wheels off the ground before running AUTOTUNE.
  Each trial repeatedly drives one wheel up to the target speed; on the
  ground the robot will crawl into whatever is in front of it.

  Hardware: QGPMaker motor shield + four QGPMaker quadrature encoders.
  No IMU is used by this tool.

  Serial: 500000 baud, 8-N-1. Commands are ASCII lines terminated by
  \n, \r or ':'.

  Commands:
    AUTOTUNE,<motor 1-4>[,<target_mm_s>[,<iterations>]]
      Runs the autotune search on one motor. Defaults: target_mm_s=200,
      iterations=8. Prints a TRIAL line per step test and a RESULT line
      with the final gains when done.
    STOP / ESTOP
      Aborts any running autotune and releases all motors.
    PING
      Replies PONG.

  Output after a completed run:
    RESULT,motor=<n>,kp=...,ki=...,kd=...,cost=...
    PASTE,wheelPID[<index>].configure(kp, ki, kd);
    TEST_CMD,PIDM,<n>,<kp>,<ki>,<kd>
      Send TEST_CMD's payload verbatim to the production firmware's serial
      port to try the gains live (without reflashing) before committing
      them to arduino_ros2_base_controller.ino.
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
const float COUNTS_PER_REV = 4320.0f;
const float MM_PER_COUNT = (PI * WHEEL_DIAMETER_MM) / COUNTS_PER_REV;

const float MAX_WHEEL_MM_S = 350.0f;
const uint8_t MIN_EFFECTIVE_PWM = 48;
const uint8_t MAX_DRIVE_PWM = 210;

// Motor order: 1 front-left, 2 front-right, 3 rear-right, 4 rear-left.
const int8_t ENCODER_SIGN[MOTOR_COUNT] = {-1, 1, 1, -1};

// -----------------------------------------------------------------------------
// Autotune tuning knobs
// -----------------------------------------------------------------------------

const float DEFAULT_TARGET_MM_S = 200.0f;
const uint8_t DEFAULT_ITERATIONS = 8;
const uint8_t MAX_ITERATIONS_ALLOWED = 30;

const uint16_t TRIAL_DURATION_MS = 1200;
const uint16_t REST_DURATION_MS = 600;
const float STEADY_WINDOW_FRACTION = 0.3f;    // last 30% of trial used for steady-state avg

const float OVERSHOOT_COST_WEIGHT = 4.0f;
const float STEADY_ERROR_COST_WEIGHT = 6.0f;
const float NO_RISE_PENALTY = 5000.0f;
const float CONVERGENCE_THRESHOLD = 0.001f;   // sum of deltas below this stops early

// Start the search from the production sketch's shared baseline gains.
const float INITIAL_KP = 0.25f;
const float INITIAL_KI = 0.034f;
const float INITIAL_KD = 0.003f;
const float INITIAL_DELTA_KP = 0.05f;
const float INITIAL_DELTA_KI = 0.01f;
const float INITIAL_DELTA_KD = 0.001f;

const float MIN_GAIN = 0.0f;
const float MAX_KP = 3.0f;
const float MAX_KI = 1.0f;
const float MAX_KD = 0.05f;

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

float speedFeedForwardPWM(float targetMMs) {
  float ratio = constrain(targetMMs / MAX_WHEEL_MM_S, 0.0f, 1.0f);
  return MIN_EFFECTIVE_PWM + ratio * (MAX_DRIVE_PWM - MIN_EFFECTIVE_PWM);
}

// -----------------------------------------------------------------------------
// PID controller (identical shape to the production WheelPID; struct is in
// AutotuneTypes.h so it's visible to the IDE's auto-generated prototypes)
// -----------------------------------------------------------------------------

WheelPID testPID;

// -----------------------------------------------------------------------------
// Serial command handling
// -----------------------------------------------------------------------------

char commandBuffer[96];
uint8_t commandLength = 0;
bool abortRequested = false;

void printReady() {
  Serial.println(F("READY,mDetect_motor_autotune_v1"));
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

  testPID.reset();
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
      const float rawSpeed = (signedTicks * MM_PER_COUNT) / dt;
      filteredSpeed += 0.35f * (rawSpeed - filteredSpeed);

      const float error = targetMMs - filteredSpeed;
      const float correction = testPID.update(targetMMs, filteredSpeed, dt);
      const float feedForward = speedFeedForwardPWM(targetMMs);
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

  applyMotorOutputForward(motorIndex, 0.0f);

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

float evaluate(uint8_t motorIndex, float targetMMs, float kp, float ki, float kd, TrialResult *outResult) {
  testPID.configure(kp, ki, kd);
  TrialResult r = runStepTrial(motorIndex, targetMMs);
  if (outResult) *outResult = r;
  return r.cost;
}

// -----------------------------------------------------------------------------
// Twiddle (coordinate descent) search and reporting
// -----------------------------------------------------------------------------

void printTrial(int iter, int paramIndex, const float params[3], const TrialResult &r, float cost, bool isBaseline) {
  const char *name = "base";
  if (paramIndex == 0) name = "kp";
  else if (paramIndex == 1) name = "ki";
  else if (paramIndex == 2) name = "kd";

  Serial.print(isBaseline ? F("BASELINE,") : F("TRIAL,"));
  Serial.print(F("iter=")); Serial.print(iter);
  Serial.print(F(",param=")); Serial.print(name);
  Serial.print(F(",kp=")); Serial.print(params[0], 4);
  Serial.print(F(",ki=")); Serial.print(params[1], 4);
  Serial.print(F(",kd=")); Serial.print(params[2], 4);
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

  float params[3] = {INITIAL_KP, INITIAL_KI, INITIAL_KD};
  float deltas[3] = {INITIAL_DELTA_KP, INITIAL_DELTA_KI, INITIAL_DELTA_KD};
  const float maxParam[3] = {MAX_KP, MAX_KI, MAX_KD};

  Serial.print(F("INFO,AUTOTUNE_START,motor="));
  Serial.print(motorIndex + 1);
  Serial.print(F(",target_mm_s="));
  Serial.print(targetMMs, 1);
  Serial.print(F(",iterations="));
  Serial.println(maxIterations);

  TrialResult baseline;
  float bestCost = evaluate(motorIndex, targetMMs, params[0], params[1], params[2], &baseline);
  printTrial(0, -1, params, baseline, bestCost, true);
  if (baseline.aborted) {
    finishAborted();
    return;
  }

  for (uint8_t iter = 0; iter < maxIterations; ++iter) {
    for (uint8_t i = 0; i < 3; ++i) {
      const float original = params[i];

      params[i] = constrain(original + deltas[i], MIN_GAIN, maxParam[i]);
      TrialResult trialUp;
      float costUp = evaluate(motorIndex, targetMMs, params[0], params[1], params[2], &trialUp);
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

      params[i] = constrain(original - deltas[i], MIN_GAIN, maxParam[i]);
      TrialResult trialDown;
      float costDown = evaluate(motorIndex, targetMMs, params[0], params[1], params[2], &trialDown);
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

    if (deltas[0] + deltas[1] + deltas[2] < CONVERGENCE_THRESHOLD) {
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
  Serial.println(F(");"));

  Serial.print(F("TEST_CMD,PIDM,"));
  Serial.print(motorIndex + 1);
  Serial.print(F(","));
  Serial.print(params[0], 4);
  Serial.print(F(","));
  Serial.print(params[1], 4);
  Serial.print(F(","));
  Serial.println(params[2], 4);
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
  Serial.println(F("INFO,LIFT_ALL_WHEELS_OFF_THE_GROUND_BEFORE_AUTOTUNE"));
}

void loop() {
  readSerialCommands();
}
