/*
  mDetect ROS2 low-level base controller

  Hardware:
  - Arduino Uno
  - QGPMaker motor shield with four DC motors
  - Four QGPMaker quadrature encoders
  - MPU6050 on I2C

  Responsibility split:
  - Arduino: 100 Hz encoder/IMU acquisition, four independent wheel-speed PID
    loops, odometry integration, command watchdog and latched emergency stop.
  - Raspberry Pi: ROS2 serial bridge, /cmd_vel, /odom, /imu/data, /joint_states.
  - Ubuntu workstation: RViz, SLAM Toolbox, Nav2, costmaps, planner and
    Regulated Pure Pursuit controller.

  ROS coordinate convention:
  - +X forward
  - +Y left
  - +yaw counter-clockwise

  Serial settings:
  - 500000 baud, 8-N-1
  - Commands are ASCII lines terminated by \n, \r or ':'

  Main command:
    VEL,<linear_mm_s>,<angular_deg_s>

  Test aliases:
    FORWARD,<speed_mm_s>
    REVERSE,<speed_mm_s>
    LEFT,<angular_deg_s>
    RIGHT,<angular_deg_s>
    STOP
    ESTOP
    CLEAR_ESTOP
    RESET_ODOM
    ZERO_YAW
    CAL_IMU
    PID,<kp>,<ki>,<kd>
    PIDM,<motor_1_to_4>,<kp>,<ki>,<kd>
    PING

  Telemetry at 20 Hz:
    T,time_ms,x_mm,y_mm,yaw_deg,vx_mm_s,wz_deg_s,
      tick1,tick2,tick3,tick4,
      speed1,speed2,speed3,speed4,
      pwm1,pwm2,pwm3,pwm4,estop,watchdog
*/

#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "QGPMaker_MotorShield.h"
#include "QGPMaker_Encoder.h"
#include <MPU6050_tockn.h>

// -----------------------------------------------------------------------------
// Robot geometry and timing
// -----------------------------------------------------------------------------

const uint8_t MOTOR_COUNT = 4;
const uint32_t SERIAL_BAUD = 500000UL;
const uint16_t CONTROL_PERIOD_MS = 10;      // 100 Hz motor control
const uint16_t TELEMETRY_PERIOD_MS = 50;    // 20 Hz telemetry
const uint16_t COMMAND_WATCHDOG_MS = 500;   // stop if Pi heartbeat disappears

const float WHEEL_DIAMETER_MM = 80.5f;
const float WHEEL_RADIUS_MM = WHEEL_DIAMETER_MM * 0.5f;
const float COUNTS_PER_REV = 4320.0f;
const float MM_PER_COUNT = (PI * WHEEL_DIAMETER_MM) / COUNTS_PER_REV;
const float TRACK_WIDTH_MM = 210.0f;        // left-to-right wheel centre distance

// Limits should stay conservative until the robot has been tested on blocks.
const float MAX_LINEAR_MM_S = 250.0f;
const float MAX_ANGULAR_DEG_S = 120.0f;
const float MAX_WHEEL_MM_S = 350.0f;
const float LINEAR_ACCEL_MM_S2 = 350.0f;
const float ANGULAR_ACCEL_DEG_S2 = 220.0f;

const uint8_t MIN_EFFECTIVE_PWM = 48;
const uint8_t MAX_DRIVE_PWM = 210;

// Existing encoder installation signs. Forward motion must produce positive
// speed for all four wheels. Change one sign if that wheel reports backwards.
const int8_t ENCODER_SIGN[MOTOR_COUNT] = {-1, 1, 1, -1};

// Motor order used throughout this sketch and the ROS URDF:
// 1 front-left, 2 front-right, 3 rear-right, 4 rear-left.
const bool LEFT_SIDE[MOTOR_COUNT] = {true, false, false, true};

// Keep all feed-forward scales equal initially. PID will independently correct
// motor-to-motor differences. These can be adjusted later only if a motor reaches
// integral saturation during steady driving.
const float PWM_FEEDFORWARD_SCALE[MOTOR_COUNT] = {1.0f, 1.0f, 1.0f, 1.0f};

// MPU sign: a physical left turn must make ROS yaw increase. Set to -1.0 if the
// yaw decreases during a left-turn test.
const float IMU_YAW_SIGN = 1.0f;

// -----------------------------------------------------------------------------
// Hardware
// -----------------------------------------------------------------------------

QGPMaker_MotorShield motorShield;
QGPMaker_Encoder encoder1(1);
QGPMaker_Encoder encoder2(2);
QGPMaker_Encoder encoder3(3);
QGPMaker_Encoder encoder4(4);
MPU6050 mpu6050(Wire);

// -----------------------------------------------------------------------------
// PID controller
// -----------------------------------------------------------------------------

struct WheelPID {
  float kp;
  float ki;
  float kd;
  float integral;
  float previousError;
  float previousTarget;

  void configure(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
    reset();
  }

  void reset() {
    integral = 0.0f;
    previousError = 0.0f;
    previousTarget = 0.0f;
  }

  float update(float target, float measured, float dt) {
    if (dt <= 0.0f) return 0.0f;

    // Clear accumulated integral when stopped or when wheel direction changes.
    if (fabs(target) < 1.0f || (target * previousTarget < 0.0f)) {
      integral = 0.0f;
      previousError = 0.0f;
    }

    const float error = target - measured;
    integral += error * dt;
    integral = constrain(integral, -500.0f, 500.0f);

    const float derivative = (error - previousError) / dt;
    previousError = error;
    previousTarget = target;

    return kp * error + ki * integral + kd * derivative;
  }
};

WheelPID wheelPID[MOTOR_COUNT];

// -----------------------------------------------------------------------------
// State
// -----------------------------------------------------------------------------

float commandLinearMMs = 0.0f;
float commandAngularDegS = 0.0f;
float rampedLinearMMs = 0.0f;
float rampedAngularDegS = 0.0f;

float wheelTargetMMs[MOTOR_COUNT] = {0};
float wheelMeasuredMMs[MOTOR_COUNT] = {0};
float wheelFilteredMMs[MOTOR_COUNT] = {0};
float wheelPWM[MOTOR_COUNT] = {0};
int32_t cumulativeTicks[MOTOR_COUNT] = {0};

float poseXMM = 0.0f;
float poseYMM = 0.0f;
float yawDeg = 0.0f;
float yawRateDegS = 0.0f;
float yawZeroDeg = 0.0f;
float linearVelocityMMs = 0.0f;

bool emergencyStopLatched = false;
bool watchdogStopped = true;

uint32_t lastCommandMs = 0;
uint32_t lastControlMs = 0;
uint32_t lastTelemetryMs = 0;

char commandBuffer[96];
uint8_t commandLength = 0;

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

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

void resetEncoderDeltas() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    readEncoderAndResetAtomic(i);
    wheelMeasuredMMs[i] = 0.0f;
    wheelFilteredMMs[i] = 0.0f;
  }
}

float normalizeDegrees(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

float moveToward(float current, float target, float maximumStep) {
  if (current < target) return min(current + maximumStep, target);
  if (current > target) return max(current - maximumStep, target);
  return current;
}

float speedFeedForwardPWM(float targetMMs, uint8_t motorIndex) {
  const float magnitude = fabs(targetMMs);
  if (magnitude < 1.0f) return 0.0f;

  float ratio = magnitude / MAX_WHEEL_MM_S;
  ratio = constrain(ratio, 0.0f, 1.0f);
  float pwm = MIN_EFFECTIVE_PWM + ratio * (MAX_DRIVE_PWM - MIN_EFFECTIVE_PWM);
  return pwm * PWM_FEEDFORWARD_SCALE[motorIndex];
}

void releaseAllMotors() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    QGPMaker_DCMotor *m = getMotor(i);
    m->setSpeed(0);
    m->run(RELEASE);
    wheelPWM[i] = 0.0f;
    wheelTargetMMs[i] = 0.0f;
    wheelPID[i].reset();
  }
}

void brakeAllMotors() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    QGPMaker_DCMotor *m = getMotor(i);
    m->setSpeed(0);
    m->run(BRAKE);
    wheelPWM[i] = 0.0f;
    wheelTargetMMs[i] = 0.0f;
    wheelPID[i].reset();
  }
}

void applyMotorOutput(uint8_t index, float signedPWM) {
  if (fabs(wheelTargetMMs[index]) < 1.0f || fabs(signedPWM) < 1.0f) {
    QGPMaker_DCMotor *m = getMotor(index);
    m->setSpeed(0);
    m->run(RELEASE);
    wheelPWM[index] = 0.0f;
    return;
  }

  uint8_t pwm = (uint8_t)constrain((int)(fabs(signedPWM) + 0.5f), 0, 255);
  uint8_t direction = (wheelTargetMMs[index] >= 0.0f) ? FORWARD : BACKWARD;

  // Run direction first so the library's internal MDIR is initialised before
  // setSpeed() calls run(MDIR).
  QGPMaker_DCMotor *m = getMotor(index);
  m->run(direction);
  m->setSpeed(pwm);
  wheelPWM[index] = (wheelTargetMMs[index] >= 0.0f) ? pwm : -((float)pwm);
}

void setVelocityCommand(float linearMMs, float angularDegS) {
  commandLinearMMs = constrain(linearMMs, -MAX_LINEAR_MM_S, MAX_LINEAR_MM_S);
  commandAngularDegS = constrain(angularDegS, -MAX_ANGULAR_DEG_S, MAX_ANGULAR_DEG_S);
  lastCommandMs = millis();
  watchdogStopped = false;
}

void stopCommand(bool brake) {
  commandLinearMMs = 0.0f;
  commandAngularDegS = 0.0f;
  rampedLinearMMs = 0.0f;
  rampedAngularDegS = 0.0f;
  if (brake) brakeAllMotors();
  else releaseAllMotors();
}

void resetOdometry() {
  poseXMM = 0.0f;
  poseYMM = 0.0f;
  linearVelocityMMs = 0.0f;
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) cumulativeTicks[i] = 0;
  resetEncoderDeltas();
  yawZeroDeg = IMU_YAW_SIGN * mpu6050.getAngleZ();
  yawDeg = 0.0f;
}

void zeroYaw() {
  yawZeroDeg = IMU_YAW_SIGN * mpu6050.getAngleZ();
  yawDeg = 0.0f;
}

// -----------------------------------------------------------------------------
// Command parser
// -----------------------------------------------------------------------------

void printReady() {
  Serial.println(F("READY,mDetect_ROS2_low_level_v1"));
}

void processCommand(char *line) {
  char *savePtr = NULL;
  char *command = strtok_r(line, ",", &savePtr);
  if (command == NULL) return;

  // Make command comparison case insensitive.
  for (char *p = command; *p; ++p) {
    if (*p >= 'a' && *p <= 'z') *p = *p - ('a' - 'A');
  }

  if (strcmp(command, "VEL") == 0) {
    char *linearText = strtok_r(NULL, ",", &savePtr);
    char *angularText = strtok_r(NULL, ",", &savePtr);
    if (linearText && angularText && !emergencyStopLatched) {
      setVelocityCommand(atof(linearText), atof(angularText));
      Serial.println(F("ACK,VEL"));
    } else if (emergencyStopLatched) {
      Serial.println(F("ERR,ESTOP_LATCHED"));
    } else {
      Serial.println(F("ERR,VEL_FORMAT"));
    }
  }
  else if (strcmp(command, "FORWARD") == 0) {
    char *value = strtok_r(NULL, ",", &savePtr);
    if (!emergencyStopLatched) setVelocityCommand(value ? fabs(atof(value)) : 100.0f, 0.0f);
  }
  else if (strcmp(command, "REVERSE") == 0) {
    char *value = strtok_r(NULL, ",", &savePtr);
    if (!emergencyStopLatched) setVelocityCommand(-(value ? fabs(atof(value)) : 100.0f), 0.0f);
  }
  else if (strcmp(command, "LEFT") == 0) {
    char *value = strtok_r(NULL, ",", &savePtr);
    if (!emergencyStopLatched) setVelocityCommand(0.0f, value ? fabs(atof(value)) : 45.0f);
  }
  else if (strcmp(command, "RIGHT") == 0) {
    char *value = strtok_r(NULL, ",", &savePtr);
    if (!emergencyStopLatched) setVelocityCommand(0.0f, -(value ? fabs(atof(value)) : 45.0f));
  }
  else if (strcmp(command, "STOP") == 0) {
    stopCommand(false);
    lastCommandMs = millis();
    watchdogStopped = false;
    Serial.println(F("ACK,STOP"));
  }
  else if (strcmp(command, "ESTOP") == 0) {
    emergencyStopLatched = true;
    stopCommand(true);
    Serial.println(F("ACK,ESTOP_LATCHED"));
  }
  else if (strcmp(command, "CLEAR_ESTOP") == 0) {
    emergencyStopLatched = false;
    stopCommand(false);
    lastCommandMs = millis();
    watchdogStopped = false;
    Serial.println(F("ACK,ESTOP_CLEARED"));
  }
  else if (strcmp(command, "RESET_ODOM") == 0) {
    stopCommand(false);
    resetOdometry();
    Serial.println(F("ACK,ODOM_RESET"));
  }
  else if (strcmp(command, "ZERO_YAW") == 0) {
    zeroYaw();
    Serial.println(F("ACK,YAW_ZEROED"));
  }
  else if (strcmp(command, "CAL_IMU") == 0) {
    stopCommand(true);
    Serial.println(F("INFO,KEEP_ROBOT_STILL_CALIBRATING_IMU"));
    mpu6050.calcGyroOffsets(false);
    zeroYaw();
    Serial.println(F("ACK,IMU_CALIBRATED"));
  }
  else if (strcmp(command, "PID") == 0) {
    char *pText = strtok_r(NULL, ",", &savePtr);
    char *iText = strtok_r(NULL, ",", &savePtr);
    char *dText = strtok_r(NULL, ",", &savePtr);
    if (pText && iText && dText) {
      const float kp = atof(pText);
      const float ki = atof(iText);
      const float kd = atof(dText);
      for (uint8_t i = 0; i < MOTOR_COUNT; ++i) wheelPID[i].configure(kp, ki, kd);
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
    int motorIndex = motorText ? atoi(motorText) - 1 : -1;
    if (motorIndex >= 0 && motorIndex < MOTOR_COUNT && pText && iText && dText) {
      wheelPID[motorIndex].configure(atof(pText), atof(iText), atof(dText));
      Serial.println(F("ACK,PID_MOTOR"));
    } else {
      Serial.println(F("ERR,PIDM_FORMAT"));
    }
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
// Control and odometry
// -----------------------------------------------------------------------------

void updateIMU() {
  mpu6050.update();
  yawDeg = normalizeDegrees(IMU_YAW_SIGN * mpu6050.getAngleZ() - yawZeroDeg);
  yawRateDegS = IMU_YAW_SIGN * mpu6050.getGyroZ();
}

void updateControl(float dt) {
  // Communication loss is always handled locally on the Arduino.
  if (!emergencyStopLatched && millis() - lastCommandMs > COMMAND_WATCHDOG_MS) {
    watchdogStopped = true;
    stopCommand(false);
  }

  if (emergencyStopLatched) {
    brakeAllMotors();
    return;
  }

  rampedLinearMMs = moveToward(
    rampedLinearMMs,
    commandLinearMMs,
    LINEAR_ACCEL_MM_S2 * dt
  );
  rampedAngularDegS = moveToward(
    rampedAngularDegS,
    commandAngularDegS,
    ANGULAR_ACCEL_DEG_S2 * dt
  );

  const float angularRadS = rampedAngularDegS * DEG_TO_RAD;
  const float leftTarget = rampedLinearMMs - angularRadS * TRACK_WIDTH_MM * 0.5f;
  const float rightTarget = rampedLinearMMs + angularRadS * TRACK_WIDTH_MM * 0.5f;

  float leftDistanceMM = 0.0f;
  float rightDistanceMM = 0.0f;
  uint8_t leftCount = 0;
  uint8_t rightCount = 0;

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    const int32_t rawTicks = readEncoderAndResetAtomic(i);
    const int32_t signedTicks = rawTicks * ENCODER_SIGN[i];
    cumulativeTicks[i] += signedTicks;

    const float distanceMM = signedTicks * MM_PER_COUNT;
    const float rawSpeedMMs = distanceMM / dt;

    // Low-pass encoder speed while preserving signed direction.
    wheelFilteredMMs[i] += 0.35f * (rawSpeedMMs - wheelFilteredMMs[i]);
    wheelMeasuredMMs[i] = wheelFilteredMMs[i];

    if (LEFT_SIDE[i]) {
      leftDistanceMM += distanceMM;
      ++leftCount;
      wheelTargetMMs[i] = constrain(leftTarget, -MAX_WHEEL_MM_S, MAX_WHEEL_MM_S);
    } else {
      rightDistanceMM += distanceMM;
      ++rightCount;
      wheelTargetMMs[i] = constrain(rightTarget, -MAX_WHEEL_MM_S, MAX_WHEEL_MM_S);
    }

    const float feedForward = speedFeedForwardPWM(wheelTargetMMs[i], i);
    const float correction = wheelPID[i].update(wheelTargetMMs[i], wheelMeasuredMMs[i], dt);
    float outputMagnitude = feedForward + ((wheelTargetMMs[i] >= 0.0f) ? correction : -correction);
    outputMagnitude = constrain(outputMagnitude, 0.0f, 255.0f);
    applyMotorOutput(i, (wheelTargetMMs[i] >= 0.0f) ? outputMagnitude : -outputMagnitude);
  }

  if (leftCount > 0) leftDistanceMM /= leftCount;
  if (rightCount > 0) rightDistanceMM /= rightCount;
  const float centreDistanceMM = 0.5f * (leftDistanceMM + rightDistanceMM);

  const float yawRad = yawDeg * DEG_TO_RAD;
  poseXMM += centreDistanceMM * cos(yawRad);
  poseYMM += centreDistanceMM * sin(yawRad);
  linearVelocityMMs = centreDistanceMM / dt;
}

void publishTelemetry() {
  Serial.print(F("T,"));
  Serial.print(millis());
  Serial.print(','); Serial.print(poseXMM, 2);
  Serial.print(','); Serial.print(poseYMM, 2);
  Serial.print(','); Serial.print(yawDeg, 3);
  Serial.print(','); Serial.print(linearVelocityMMs, 2);
  Serial.print(','); Serial.print(yawRateDegS, 3);

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    Serial.print(','); Serial.print(cumulativeTicks[i]);
  }
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    Serial.print(','); Serial.print(wheelMeasuredMMs[i], 2);
  }
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    Serial.print(','); Serial.print(wheelPWM[i], 1);
  }

  Serial.print(','); Serial.print(emergencyStopLatched ? 1 : 0);
  Serial.print(','); Serial.println(watchdogStopped ? 1 : 0);
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

  // Same initial PID gains for all wheels. Each motor receives the same speed
  // target for straight motion, while its own PID output corrects speed mismatch.
  // for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
  //   wheelPID[i].configure(0.25f, 0.034f, 0.003f);
  // }
  wheelPID[0].configure(0.3420f,0.0290f,0.0035f);
  wheelPID[1].configure(0.2056f,0.0292f,0.0030f);
  wheelPID[2].configure(0.2598f,0.0183f,0.0042f);
  wheelPID[3].configure(0.4543f,0.0603f,0.0035f);

  mpu6050.begin();
  Serial.println(F("INFO,KEEP_ROBOT_STILL_IMU_STARTUP_CALIBRATION"));
  mpu6050.calcGyroOffsets(false);

  releaseAllMotors();
  resetEncoderDeltas();
  zeroYaw();

  lastCommandMs = millis();
  lastControlMs = millis();
  lastTelemetryMs = millis();
  printReady();
}

void loop() {
  readSerialCommands();
  updateIMU();

  const uint32_t now = millis();
  if (now - lastControlMs >= CONTROL_PERIOD_MS) {
    float dt = (now - lastControlMs) * 0.001f;
    lastControlMs = now;
    dt = constrain(dt, 0.005f, 0.050f);
    updateControl(dt);
  }

  if (now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = now;
    publishTelemetry();
  }
}
