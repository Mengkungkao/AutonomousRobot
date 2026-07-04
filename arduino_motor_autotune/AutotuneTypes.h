#ifndef AutotuneTypes_h_
#define AutotuneTypes_h_

// Kept in a header so these types are visible to the Arduino IDE's
// auto-generated function prototypes, which are inserted above the .ino's
// own struct definitions but below any #include.

// Fractional-order PID: PI^lambda D^mu. The integral and derivative each keep
// their classic, unbounded/instantaneous form -- that's what gives a true
// pole at the origin, so a constant wheel-speed error still gets integrated
// away to zero over time, the same guarantee the original classic PID gave.
// The fractional order only reshapes what feeds them: the error is passed
// through a Grunwald-Letnikov (GL) short-memory filter of the *residual*
// order (lambda-1) before the integral, and (mu-1) before the derivative.
// Both residual orders are small (lambda/mu are kept within [0.4, 1.4], see
// MIN/MAX_LAMBDA/MU in the autotune sketch) so short-memory truncation only
// affects fine shaping, never whether steady-state error converges to zero.
// At lambda == mu == 1.0 the GL filter is exactly the identity (order-0 GL
// binomial coefficients are 1,0,0,...), so this reduces byte-for-byte to the
// original classic PID -- verified by direct comparison against the old
// implementation over a target sequence with direction flips and stops.
struct WheelPID {
  static const uint8_t GL_MEMORY_LENGTH = 20;

  float kp, ki, kd;
  float lambda, mu;
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

    // Clear accumulated state when stopped or when wheel direction changes,
    // same trigger as the classic PID's integral reset.
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
    integral = constrain(integral, -500.0f, 500.0f);

    const float shapedForDerivative = glShape(mu - 1.0f, pow(dt, 1.0f - mu));
    const float derivative = (shapedForDerivative - previousShapedError) / dt;
    previousShapedError = shapedForDerivative;

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
