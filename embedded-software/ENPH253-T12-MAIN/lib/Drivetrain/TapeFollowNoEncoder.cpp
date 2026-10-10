#include "TapeFollowNoEncoder.h"
#include "TapeFollowEncoder.h" // for computeWeightedError() from 4 tape sensors

#include <cmath>

// Drives duty cycle directly from the tape error

float baseDutyCycle{3800.0f}; // placeholder, tune to reasonable value
// NOTE: gains are in different units than TapeFollowEncoder's kp and kd,
// (error based on duty counts, not error based on rpm), so tune from scratch
float dutyKp{80.0f};
float dutyKd{20.0f};

float lastErrorNoEncoder{0.0f};

DutyCycleTargets tapeFollowNoEncoder() {
  float error = computeWeightedError();
  float p = dutyKp * error;
  float d = dutyKd * (error - lastErrorNoEncoder);
  float turnEffort = p + d;

  lastErrorNoEncoder = error;

  DutyCycleTargets targets{};

  int leftDuty = static_cast<int>(baseDutyCycle + turnEffort);
  int rightDuty = static_cast<int>(baseDutyCycle - turnEffort);

  targets.dutyLeft = leftDuty;
  targets.dutyRight = rightDuty;
  targets.forwardLeft = true;
  targets.forwardRight = true;
  return targets;
}

void tapeFollowNoEncoderSetKp(float newKp) { dutyKp = newKp; }
void tapeFollowNoEncoderSetKd(float newKd) { dutyKd = newKd; }
void setTapeFollowNoEncoderBaseDuty(float newBaseDuty) {
  baseDutyCycle = newBaseDuty;
}