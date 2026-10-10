#pragma once

#include <types.h>

#include <Arduino.h>

struct WheelPIDState {
  float kp{};
  float kd{};
  float ki{};

  float lastError{};
  float integral{};
  float lastTargetRpm{};

  int maxDutyCycle{};
  float maxIntegralContribution{};
  float maxIntegralAccum{};
  Wheels::WheelSide side{};
};

struct WheelOutput {
  int dutyCycle{};
  bool forward{};
};

void wheelPID_init(WheelPIDState &state, float kp, float kd, float ki,
                   Wheels::WheelSide side);
WheelOutput wheelPID(WheelPIDState &state, float targetRpm, float measuredRpm);