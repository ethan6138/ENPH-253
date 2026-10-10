#pragma once

#include <types.h>

// Duty Cycle from RPM
constexpr int leftDutyCycleMin{1180};  //  MIN No Load 2170 : Load 1250
constexpr int rightDutyCycleMin{1320}; //  MIN No Load 2350 : Load 1250

constexpr float leftWheelRpmAtFullDuty{
    320.0f}; // MAX No Load: 627.0f Load: 400.0f
constexpr float rightWheelRpmAtFullDuty{
    260.0f}; // MAX No Load: 627.0f Load 400.0f
constexpr int maxDutyCycle{4095};

constexpr int boostedLeftDutyCycleMin{leftDutyCycleMin + 1500};
constexpr int boostedRightDutyCycleMin{rightDutyCycleMin + 1500};

constexpr float boostedLeftWheelRpmAtFullDuty{leftWheelRpmAtFullDuty - 150.0f};
constexpr float boostedRightWheelRpmAtFullDuty{rightWheelRpmAtFullDuty -
                                               150.0f};

inline int dutyCycleFromRpm(float targetRpmMag, Wheels::WheelSide side) {
  // targetRpmMag should always be non-negative
  // direction is handled in different functions with a bool.
  float mag = fabs(targetRpmMag);
  float duty{};
  int minDuty =
      (side == Wheels::WheelSide::LEFT) ? leftDutyCycleMin : rightDutyCycleMin;
  float wheelRpmAtFullDuty = (side == Wheels::WheelSide::LEFT)
                                 ? leftWheelRpmAtFullDuty
                                 : rightWheelRpmAtFullDuty;
  float ratio = mag / wheelRpmAtFullDuty;

  if (mag < 0.5f) {
    return 0;
  }

  if (ratio > 1.0f)
    ratio = 1.0f;

  duty = minDuty + ratio * (maxDutyCycle - minDuty);

  if (duty > static_cast<float>(maxDutyCycle))
    duty = static_cast<float>(maxDutyCycle);

  return static_cast<int>(duty);
}

//  Boosted for ramp, using boosted min rpms
inline int dutyCycleFromRpmBoost(float targetRpmMag, Wheels::WheelSide side) {
  float mag = fabs(targetRpmMag);
  float duty{};

  int minDuty = (side == Wheels::WheelSide::LEFT) ? boostedLeftDutyCycleMin
                                                  : boostedRightDutyCycleMin;
  float wheelRpmAtFullDuty = (side == Wheels::WheelSide::LEFT)
                                 ? boostedLeftWheelRpmAtFullDuty
                                 : boostedRightWheelRpmAtFullDuty;
  float ratio = mag / wheelRpmAtFullDuty;

  if (mag < 0.5f) {
    return 0;
  }

  if (ratio > 1.0f)
    ratio = 1.0f;

  duty = minDuty + ratio * (maxDutyCycle - minDuty);

  if (duty > static_cast<float>(maxDutyCycle))
    duty = static_cast<float>(maxDutyCycle);

  return static_cast<int>(duty);
}