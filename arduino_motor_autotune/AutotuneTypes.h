#ifndef AutotuneTypes_h_
#define AutotuneTypes_h_

// Kept in a header so these types are visible to the Arduino IDE's
// auto-generated function prototypes, which are inserted above the .ino's
// own struct definitions but below any #include.

struct WheelPID {
  float kp, ki, kd;
  float integral, previousError, previousTarget;

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

struct TrialResult {
  float cost;
  float riseTimeS;
  float overshootPct;
  float steadyStateErrorMMs;
  bool aborted;
};

#endif
