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
    PID,<kp>,<ki>,<kd>[,<lambda>,<mu>]
    PIDM,<motor_1_to_4>,<kp>,<ki>,<kd>[,<lambda>,<mu>]
    PING

  Wheel speed loops are fractional-order PID (PI^lambda D^mu). lambda/mu
  default to 1.0 (classic PID) when omitted from PID/PIDM.

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

// Measured by hand-rotating each wheel exactly one full turn and reading the
// tick count (2026-07-02): 4326, 4226, 4138, 4334 -- motors 2 and 3 undercount
// noticeably relative to the nominal 4320, which (uncorrected) made their
// measured speed under-report their true speed, so PID kept driving them
// physically faster than commanded to compensate. Per-wheel values here
// instead of a single shared constant fix that at the source.
//by hand 1st time :4326.0f, 4226.0f, 4138.0f, 4334.0f
//by hand 2nd time : 4193.0f, 4176.0f, 4119.0f, 4330.0f
//by hand 3rd time:  4201.0f, 4154.0f, 4128.0f, 4235.0f
const float MEASURED_COUNTS_PER_REV[MOTOR_COUNT] = {4193.0f, 4176.0f, 4119.0f, 4330.0f};
const float MM_PER_COUNT[MOTOR_COUNT] = {
  (PI * WHEEL_DIAMETER_MM) / MEASURED_COUNTS_PER_REV[0],
  (PI * WHEEL_DIAMETER_MM) / MEASURED_COUNTS_PER_REV[1],
  (PI * WHEEL_DIAMETER_MM) / MEASURED_COUNTS_PER_REV[2],
  (PI * WHEEL_DIAMETER_MM) / MEASURED_COUNTS_PER_REV[3],
};
const float TRACK_WIDTH_MM = 210.0f;        // left-to-right wheel centre distance

// Limits should stay conservative until the robot has been tested on blocks.
const float MAX_LINEAR_MM_S = 250.0f;
const float MAX_ANGULAR_DEG_S = 120.0f;
const float MAX_WHEEL_MM_S = 250.0f;
const float LINEAR_ACCEL_MM_S2 = 150.0f;
const float ANGULAR_ACCEL_DEG_S2 = 120.0f;

const uint8_t MIN_EFFECTIVE_PWM = 50;
const uint8_t MAX_DRIVE_PWM = 210;

// Existing encoder installation signs. Forward motion must produce positive
// speed for all four wheels. Change one sign if that wheel reports backwards.
const int8_t ENCODER_SIGN[MOTOR_COUNT] = {-1, 1, 1, -1};

// Motor order used throughout this sketch and the ROS URDF:
// 1 front-right, 2 front-left, 3 rear-left, 4 rear-right.
const bool LEFT_SIDE[MOTOR_COUNT] = {false, true, true, false};

// Per-motor, per-direction open-loop PWM-to-speed linear fit
// (|measured_mm_s| = slope*pwm + intercept). Forward coefficients are from
// arduino_motor_calibration's CALIBRATE,<motor>,10,1000 sweep (2026-07-04,
// PWM 48-208). Real motors diverge hugely from a single shared curve --
// motor 4 needs less than half the PWM of motors 2/3 for the same speed --
// so each motor gets its own inverse mapping in speedFeedForwardPWM below.
// Brushed gearmotors are also direction-asymmetric, which showed up on the
// robot as forward driving straight while reverse drifted, so reverse gets
// its own fit from a CALIBRATE,<motor>,10,1000,R sweep. Linear fit residual
// RMSE is ~15-27 mm/s (real response saturates a bit at high PWM); PID
// still corrects the remainder.
const float FF_SLOPE_MM_S_PER_PWM_FWD[MOTOR_COUNT] = {2.557f, 2.631f, 2.4945f, 2.3404f};
const float FF_INTERCEPT_MM_S_FWD[MOTOR_COUNT] = {-83.25f, -143.56f, -149.38f, 6.14f};
// Reverse fit from CALIBRATE,<motor>,10,1000,R sweeps (2026-07-05, PWM
// 50-210, stalled deadband points excluded). Very different from forward
// -- e.g. motor 4 needs PWM ~130 for 220 mm/s in reverse vs ~91 forward --
// which is what made the robot drift when backing up. RMSE 5-17 mm/s.
const float FF_SLOPE_MM_S_PER_PWM_REV[MOTOR_COUNT] = {2.4662f, 2.1564f, 1.9853f, 2.5093f};
const float FF_INTERCEPT_MM_S_REV[MOTOR_COUNT] = {-118.32f, -86.97f, -92.43f, -104.95f};

// MPU sign: a physical left turn must make ROS yaw increase (REP-103,
// CCW-positive). This MPU reports clockwise-positive (hand left-turn test read
// -50 deg), so the sign is flipped here. Set back to 1.0 only if a left-turn
// test shows yaw decreasing again after a hardware change.
const float IMU_YAW_SIGN = -1.0f;

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

// Fractional-order PID: PI^lambda D^mu. The integral and derivative each keep
// their classic, unbounded/instantaneous form -- that's what gives a true
// pole at the origin, so a constant wheel-speed error still gets integrated
// away to zero over time, the same guarantee the original classic PID gave.
// The fractional order only reshapes what feeds them: the error is passed
// through a Grunwald-Letnikov (GL) short-memory filter of the *residual*
// order (lambda-1) before the integral, and (mu-1) before the derivative.
// Both residual orders are small (lambda/mu are kept within [0.4, 1.4], see
// MIN/MAX_LAMBDA/MU in arduino_motor_autotune.ino) so short-memory
// truncation only affects fine shaping, never whether steady-state error
// converges to zero. At lambda == mu == 1.0 the GL filter is exactly the
// identity (order-0 GL binomial coefficients are 1,0,0,...), so this
// reduces byte-for-byte to the original classic PID. Identical shape to
// arduino_motor_autotune's WheelPID (in AutotuneTypes.h) so autotune
// results transfer directly.
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

    // Clear accumulated state when stopped or when wheel direction changes.
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

// Discard any pending encoder ticks and clear the speed estimates, so stale
// counts don't produce a velocity spike on the next control cycle.
void resetEncoderDeltas() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    readEncoderAndResetAtomic(i);
    wheelMeasuredMMs[i] = 0.0f;
    wheelFilteredMMs[i] = 0.0f;
  }
}

// Wrap an angle into (-180, 180] degrees.
float normalizeDegrees(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

// Step 'current' toward 'target' by at most 'maximumStep' — the slew-rate
// limiter used for the acceleration ramps.
float moveToward(float current, float target, float maximumStep) {
  if (current < target) return min(current + maximumStep, target);
  if (current > target) return max(current - maximumStep, target);
  return current;
}

// Invert the per-motor, per-direction PWM-to-speed linear fit to get the
// open-loop PWM expected to produce the target speed; PID then only has to
// correct the residual. Returns an unsigned PWM magnitude.
float speedFeedForwardPWM(float targetMMs, uint8_t motorIndex) {
  const float magnitude = fabs(targetMMs);
  if (magnitude < 1.0f) return 0.0f;

  const bool reverse = targetMMs < 0.0f;
  const float slope = reverse ? FF_SLOPE_MM_S_PER_PWM_REV[motorIndex]
                              : FF_SLOPE_MM_S_PER_PWM_FWD[motorIndex];
  const float intercept = reverse ? FF_INTERCEPT_MM_S_REV[motorIndex]
                                  : FF_INTERCEPT_MM_S_FWD[motorIndex];
  const float pwm = (magnitude - intercept) / slope;
  // No shared lower floor here: each motor's fit already encodes its own
  // dead zone (below -intercept/slope the fit says zero speed). Flooring
  // everyone at MIN_EFFECTIVE_PWM=50 forced motor 4 (fwd intercept +6.14,
  // turning from near PWM 0) to ~123 mm/s minimum -- about twice a typical
  // rotate/approach wheel target -- which made it visibly outrun the other
  // three at low speeds. PID adds PWM if a motor still sticks below its fit.
  return constrain(pwm, 0.0f, (float)MAX_DRIVE_PWM);
}

// Stop with motors freewheeling (coast) and clear all controller state.
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

// Stop with active braking (motor terminals shorted) — used for e-stop.
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

// Drive one motor with a signed PWM value: sign selects direction (taken from
// the wheel target), magnitude sets duty. Near-zero target or output releases
// the motor instead of stalling it at a tiny PWM.
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

// Accept a new body-velocity setpoint (clamped to limits) and feed the
// command watchdog.
void setVelocityCommand(float linearMMs, float angularDegS) {
  commandLinearMMs = constrain(linearMMs, -MAX_LINEAR_MM_S, MAX_LINEAR_MM_S);
  commandAngularDegS = constrain(angularDegS, -MAX_ANGULAR_DEG_S, MAX_ANGULAR_DEG_S);
  lastCommandMs = millis();
  watchdogStopped = false;
}

// Zero the setpoints and ramps, then stop the motors (brake or coast).
void stopCommand(bool brake) {
  commandLinearMMs = 0.0f;
  commandAngularDegS = 0.0f;
  rampedLinearMMs = 0.0f;
  rampedAngularDegS = 0.0f;
  if (brake) brakeAllMotors();
  else releaseAllMotors();
}

// Zero the integrated pose, tick totals, and yaw reference.
void resetOdometry() {
  poseXMM = 0.0f;
  poseYMM = 0.0f;
  linearVelocityMMs = 0.0f;
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) cumulativeTicks[i] = 0;
  resetEncoderDeltas();
  yawZeroDeg = IMU_YAW_SIGN * mpu6050.getAngleZ();
  yawDeg = 0.0f;
}

// Make the current heading the new zero by capturing the IMU's absolute
// integrated angle as the reference offset.
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

// Parse and execute one complete command line (see header comment for the
// command set). Tokenizes in place with strtok_r and replies with ACK/ERR.
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
    if (!emergencyStopLatched) setVelocityCommand(value ? fabs(atof(value)) : 220.0f, 0.0f);
  }
  else if (strcmp(command, "REVERSE") == 0) {
    char *value = strtok_r(NULL, ",", &savePtr);
    if (!emergencyStopLatched) setVelocityCommand(-(value ? fabs(atof(value)) : 220.0f), 0.0f);
  }
  else if (strcmp(command, "LEFT") == 0) {
    char *value = strtok_r(NULL, ",", &savePtr);
    if (!emergencyStopLatched) setVelocityCommand(0.0f, value ? fabs(atof(value)) : 15.0f);
  }
  else if (strcmp(command, "RIGHT") == 0) {
    char *value = strtok_r(NULL, ",", &savePtr);
    if (!emergencyStopLatched) setVelocityCommand(0.0f, -(value ? fabs(atof(value)) : 15.0f));
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
    char *lamText = strtok_r(NULL, ",", &savePtr);
    char *muText = strtok_r(NULL, ",", &savePtr);
    if (pText && iText && dText) {
      const float kp = atof(pText);
      const float ki = atof(iText);
      const float kd = atof(dText);
      const float lambda = lamText ? atof(lamText) : 1.0f;
      const float mu = muText ? atof(muText) : 1.0f;
      for (uint8_t i = 0; i < MOTOR_COUNT; ++i) wheelPID[i].configure(kp, ki, kd, lambda, mu);
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
    int motorIndex = motorText ? atoi(motorText) - 1 : -1;
    if (motorIndex >= 0 && motorIndex < MOTOR_COUNT && pText && iText && dText) {
      const float lambda = lamText ? atof(lamText) : 1.0f;
      const float mu = muText ? atof(muText) : 1.0f;
      wheelPID[motorIndex].configure(atof(pText), atof(iText), atof(dText), lambda, mu);
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
// Control and odometry
// -----------------------------------------------------------------------------

// Refresh yaw angle (relative to the zero reference) and yaw rate from the
// MPU6050's integrated gyro.
void updateIMU() {
  mpu6050.update();
  yawDeg = normalizeDegrees(IMU_YAW_SIGN * mpu6050.getAngleZ() - yawZeroDeg);
  yawRateDegS = IMU_YAW_SIGN * mpu6050.getGyroZ();
}

// 100 Hz control step: watchdog/e-stop handling, acceleration ramps,
// differential-drive kinematics, per-wheel feed-forward + PID, and odometry
// integration (encoder distance + IMU heading).
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

  // Slew-limit the commanded velocities so wheel targets change smoothly.
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

  // Differential-drive kinematics: body (v, w) -> left/right wheel rim speeds.
  const float angularRadS = rampedAngularDegS * DEG_TO_RAD;
  const float leftTarget = rampedLinearMMs - angularRadS * TRACK_WIDTH_MM * 0.5f;
  const float rightTarget = rampedLinearMMs + angularRadS * TRACK_WIDTH_MM * 0.5f;

  float leftDistanceMM = 0.0f;
  float rightDistanceMM = 0.0f;
  uint8_t leftCount = 0;
  uint8_t rightCount = 0;

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    // Measure this cycle's travel from the encoder tick delta.
    const int32_t rawTicks = readEncoderAndResetAtomic(i);
    const int32_t signedTicks = rawTicks * ENCODER_SIGN[i];
    cumulativeTicks[i] += signedTicks;

    const float distanceMM = signedTicks * MM_PER_COUNT[i];
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

    // Output = open-loop feed-forward + PID correction. The correction is
    // flipped for reverse targets because the PID works in signed speed while
    // the output here is a magnitude.
    const float feedForward = speedFeedForwardPWM(wheelTargetMMs[i], i);
    const float correction = wheelPID[i].update(wheelTargetMMs[i], wheelMeasuredMMs[i], dt);
    float outputMagnitude = feedForward + ((wheelTargetMMs[i] >= 0.0f) ? correction : -correction);
    outputMagnitude = constrain(outputMagnitude, 0.0f, 255.0f);
    applyMotorOutput(i, (wheelTargetMMs[i] >= 0.0f) ? outputMagnitude : -outputMagnitude);
  }

  // Odometry: average per-side wheel travel, take the body-centre distance,
  // and integrate along the IMU heading (encoder distance + gyro yaw fusion).
  if (leftCount > 0) leftDistanceMM /= leftCount;
  if (rightCount > 0) rightDistanceMM /= rightCount;
  const float centreDistanceMM = 0.5f * (leftDistanceMM + rightDistanceMM);

  const float yawRad = yawDeg * DEG_TO_RAD;
  poseXMM += centreDistanceMM * cos(yawRad);
  poseYMM += centreDistanceMM * sin(yawRad);
  linearVelocityMMs = centreDistanceMM / dt;
}

// Emit one 'T,...' CSV telemetry line (field layout in the header comment);
// consumed by the ROS 2 serial bridge.
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

// One-time init: serial link, I2C at 400 kHz, motor shield at 500 Hz PWM,
// per-wheel PID gains, and IMU gyro calibration (robot must be still).
void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  Wire.setClock(400000UL);

  motorShield.begin(500);
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) getMotor(i);

  // Same initial PID gains for all wheels. Each motor receives the same speed
  // target for straight motion, while its own PID output corrects speed mismatch.
  // for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
  //   wheelPID[i].configure(0.25f, 0.034f, 0.003f, 1.0f, 1.0f);
  // }
  // lambda/mu left at 1.0 (classic PID) until AUTOTUNE/MATCH is re-run with
  // fractional-order search and pastes real per-motor values here.
  // ki raised 0.003-0.008 -> 0.02 (2026-07-06): with the integral output
  // clamp fixed, the old tiny ki gave ~0.2 PWM/s of trim -- far too slow to
  // push a humming wheel through static friction or pull an overspeeding
  // one back. 0.02 is still well inside the 0.05 autotune bound; verify
  // with the tuning sketch's STEP/MATCH and re-trim kp if wheels oscillate.
  wheelPID[0].configure(0.3420f, 0.0200f, 0.0030f, 1.0f, 1.0f);
  wheelPID[1].configure(0.2900f, 0.0200f, 0.0030f, 1.0f, 1.0f);
  wheelPID[2].configure(0.3400f, 0.0200f, 0.0030f, 1.0f, 1.0f);
  wheelPID[3].configure(0.3000f, 0.0200f, 0.0035f, 1.0f, 1.0f);

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

// Main loop: poll serial and IMU every pass; run control at 100 Hz and
// telemetry at 20 Hz off millis() schedules.
void loop() {
  readSerialCommands();
  updateIMU();

  const uint32_t now = millis();
  if (now - lastControlMs >= CONTROL_PERIOD_MS) {
    // Use the true elapsed time, clamped so a hiccup can't blow up the
    // integrator or derivative.
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
