#ifndef FoPidTuningTypes_h_
#define FoPidTuningTypes_h_

// Kept in a header so these types are visible to the Arduino IDE's
// auto-generated function prototypes, which are inserted above the .ino's
// own struct definitions but below any #include. Same convention as
// arduino_motor_calibration's CalibrationTypes.h -- don't delete this file,
// StepResult/MatchResult are used throughout arduino_fopid_tuning.ino.

// Result of one STEP trial: single-motor closed-loop (PID engaged) dynamic
// step response. Same shape as arduino_motor_calibration's TrialResult so
// the two are directly comparable (STEP's steady_err_mm_s should be much
// smaller than the matching OPENLOOP baseline once gains are reasonable).
struct StepResult {
  float cost;
  float riseTimeS;
  float overshootPct;
  float steadyStateErrorMMs;
  bool aborted;
  uint16_t trialId;
};

// Result of one MATCH trial: all four motors driven closed-loop at the same
// target speed simultaneously. The cross-wheel fields (not any single
// motor's own error) are what predict odometry drift -- a robot with every
// wheel individually accurate but mismatched relative to each other will
// still veer off a straight line.
struct MatchResult {
  float steadyMMs[4];
  float errorMMs[4];
  float leftAvgMMs;
  float rightAvgMMs;
  float leftRightDiffMMs;
  float maxPairDiffMMs;
  float spreadMMs;
  bool aborted;
  uint16_t trialId;
};

// Result of one ROTATE trial: single-motor position-control move to a
// target angle, meant to be watched by eye. overshootDeg is what the
// encoder itself measured (>= 0, clamped at the target); if this disagrees
// with what you actually see the wheel do, that gap is encoder/tick-per-rev
// calibration error, not PID behaviour.
struct RotateResult {
  float finalAngleDeg;
  float overshootDeg;
  float settleTimeS;
  bool settled;
  bool aborted;
  uint16_t trialId;
};

#endif
