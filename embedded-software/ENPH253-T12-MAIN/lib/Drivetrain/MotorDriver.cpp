#include "MotorDriver.h"
#include "FeedForwardTable.h"

#include <pins.h>
#include <types.h>

#include <cmath>

constexpr int pwmFrequencyHz{100};
constexpr int pwmResolutionBits{12};
// constexpr int minDutyCycle{2170}; // no load min 2525, load 2700

void motorDriver_init() {
  ledcAttach(PIN_PWM_LMD_0, pwmFrequencyHz, pwmResolutionBits);
  ledcAttach(PIN_PWM_LMD_1, pwmFrequencyHz, pwmResolutionBits);
  ledcAttach(PIN_PWM_RMD_0, pwmFrequencyHz, pwmResolutionBits);
  ledcAttach(PIN_PWM_RMD_1, pwmFrequencyHz, pwmResolutionBits);

  // Start fully stopped on all channels
  ledcWrite(PIN_PWM_LMD_0, 0);
  ledcWrite(PIN_PWM_LMD_1, 0);
  ledcWrite(PIN_PWM_RMD_0, 0);
  ledcWrite(PIN_PWM_RMD_1, 0);
}

void setMotorDutyCycle(Wheels::WheelSide wheel, int dutyCycle, bool forward) {
  if (dutyCycle > 0 && dutyCycle < rightDutyCycleMin &&
      wheel == Wheels::WheelSide::RIGHT) {
    dutyCycle = rightDutyCycleMin;
  } else if (dutyCycle > 0 && dutyCycle < leftDutyCycleMin &&
             wheel == Wheels::WheelSide::LEFT) {
    dutyCycle = leftDutyCycleMin;
  }

  if (dutyCycle < 0)
    dutyCycle = 0;
  if (dutyCycle > maxDutyCycle)
    dutyCycle = maxDutyCycle;

  bool inverted{(wheel == Wheels::WheelSide::LEFT ? LEFT_WHEEL_INVERTED
                                                  : RIGHT_WHEEL_INVERTED)};
  bool actualForward = inverted ? !forward : forward;

  int forwardPin =
      (wheel == Wheels::WheelSide::LEFT) ? PIN_PWM_LMD_0 : PIN_PWM_RMD_0;
  int reversePin =
      (wheel == Wheels::WheelSide::LEFT) ? PIN_PWM_LMD_1 : PIN_PWM_RMD_1;

  if (actualForward) {
    ledcWrite(reversePin, 0);
    ledcWrite(forwardPin, dutyCycle);
  } else {
    ledcWrite(forwardPin, 0);
    ledcWrite(reversePin, dutyCycle);
  }
}

void setMotorRpm(Wheels::WheelSide wheel, float targetRpm) {
  bool forward{targetRpm >= 0.0f};
  float targetRpmAbs{fabs(targetRpm)};

  int dutyCycle{dutyCycleFromRpm(targetRpmAbs, wheel)};
  setMotorDutyCycle(wheel, dutyCycle, forward);
}