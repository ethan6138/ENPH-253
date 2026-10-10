#include "WheelPID.h"
#include "FeedForwardTable.h"

#include <Pose.h>

#include <types.h>

#include <cmath>

namespace {
constexpr float defaultMaxIntegralContribution{1000.0f};
constexpr float targetJumpThresholdRpm{10.0f};
constexpr float defaultMaxIntegralAccum{22000.0f}; // TODO: tune on hardware
} // namespace

void wheelPID_init(WheelPIDState &state, float kp, float kd, float ki,
                   Wheels::WheelSide side) {
  state.kp = kp;
  state.kd = kd;
  state.ki = ki;
  state.lastError = 0.0f;
  state.integral = 0.0f;
  state.lastTargetRpm = 0.0f;
  state.maxDutyCycle = maxDutyCycle;
  state.maxIntegralContribution = defaultMaxIntegralContribution;
  state.maxIntegralAccum = defaultMaxIntegralAccum;
  state.side = side;
}

WheelOutput wheelPID(WheelPIDState &state, float targetRpm, float measuredRpm) {
  constexpr float zeroRpmThreshold{0.5f}; // matches dutyCycleFromRpm's cutoff
  if (fabs(targetRpm) < zeroRpmThreshold) {
    state.integral = 0.0f;
    state.lastError = 0.0f;
    state.lastTargetRpm = 0.0f;

    WheelOutput output;
    output.dutyCycle = 0;
    output.forward = true;
    return output;
  }

  bool forward{targetRpm >= 0.0f};
  float targetRpmAbs{fabs(targetRpm)};
  float measuredRpmAbs{fabs(measuredRpm)};
  float feedForwardDuty{};

  feedForwardDuty = (getRampBottomDetected() && !getRampTopDetected())
                        ? dutyCycleFromRpmBoost(targetRpmAbs, state.side)
                        : dutyCycleFromRpm(targetRpmAbs, state.side);

  float error{targetRpmAbs - measuredRpmAbs};

  if (fabs(targetRpm - state.lastTargetRpm) > targetJumpThresholdRpm) {
    state.integral = 0.0f;
    state.lastError = error;
  }

  state.lastTargetRpm = targetRpm;

  float p = state.kp * error;
  float d = state.kd * (error - state.lastError);
  state.lastError = error;

  state.integral += error;
  float i = state.ki * state.integral;
  if (i > state.maxIntegralContribution) {
    i = state.maxIntegralContribution;
    state.integral = i / state.ki;
  } else if (i < -state.maxIntegralContribution) {
    i = -state.maxIntegralContribution;
    state.integral = i / state.ki;
  }

  // clmaps intergal term with max integral ocntrobutions + accumulation
  //  state.integral += error;
  //  state.integral = constrain(state.integral, -state.maxIntegralAccum,
  //                             state.maxIntegralAccum);
  //  float i = state.ki * state.integral;
  //  if (i > state.maxIntegralContribution) {
  //    i = state.maxIntegralContribution;
  //  } else if (i < -state.maxIntegralContribution) {
  //    i = -state.maxIntegralContribution;
  //  }

  float correctedDuty = feedForwardDuty + p + d + i;
  if (correctedDuty < 0.0f)
    correctedDuty = 0.0f;
  if (correctedDuty > state.maxDutyCycle)
    correctedDuty = static_cast<float>(state.maxDutyCycle);

  WheelOutput output;
  output.dutyCycle = static_cast<int>(correctedDuty);
  output.forward = forward;
  return output;
}