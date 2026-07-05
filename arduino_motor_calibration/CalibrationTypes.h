#ifndef CalibrationTypes_h_
#define CalibrationTypes_h_

// Kept in a header so these types are visible to the Arduino IDE's
// auto-generated function prototypes, which are inserted above the .ino's
// own struct definitions but below any #include.
//
// No PID controller lives in this sketch -- OPENLOOP and CALIBRATE both
// drive motors directly off the feed-forward curve/fixed PWM, no
// correction term. The production PID (fractional-order WheelPID,
// PI^lambda D^mu) lives only in arduino_ros2_base_controller.ino.
//
// NOTE: this file is REQUIRED by arduino_motor_calibration.ino (TrialResult
// and CalStepResult below are used throughout it) even though neither
// AUTOTUNE nor any PID exists in this sketch anymore -- don't delete it as
// autotune leftover cruft. (This file used to be named AutotuneTypes.h,
// which is exactly why it kept getting deleted by mistake; renamed to
// avoid that.)

// Result of one OPENLOOP step test: descriptive step-response metrics for a
// single motor driven by feed-forward only (rise to 90% of target, peak
// overshoot, steady-state error over the tail window, and a weighted cost).
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
