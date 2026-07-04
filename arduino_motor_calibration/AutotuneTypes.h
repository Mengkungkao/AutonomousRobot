#ifndef AutotuneTypes_h_
#define AutotuneTypes_h_

// Kept in a header so these types are visible to the Arduino IDE's
// auto-generated function prototypes, which are inserted above the .ino's
// own struct definitions but below any #include.
//
// No PID controller lives in this sketch (removed along with AUTOTUNE) --
// OPENLOOP and CALIBRATE both drive motors directly off the feed-forward
// curve/fixed PWM, no correction term. The production PID (fractional-order
// WheelPID, PI^lambda D^mu) lives only in arduino_ros2_base_controller.ino.

struct TrialResult {
  float cost;
  float riseTimeS;
  float overshootPct;
  float steadyStateErrorMMs;
  bool aborted;
  // Matches the "trial=" field on this trial's per-tick SAMPLE lines, so
  // the raw step-response stream can be joined back to the
  // OPENLOOP_RESULT summary line printed right after it.
  uint16_t trialId;
};

// Result of one open-loop PWM step in CALIBRATE: steady-state speed measured
// at a fixed, directly-commanded PWM (no PID correction).
struct CalStepResult {
  float measuredMMs;
  bool aborted;
};

#endif
