#include "Claw.h"
#include "Arm.h"

#include <helpers.h>
#include <pins.h>

#include <Arduino.h>

namespace Claw{
void openClaw() {
  ledcWrite(PIN_PWM_CLAW, anglePos(ArmParameters::ArmDimensions::openClaw));
}

void closeClaw() {
  ledcWrite(PIN_PWM_CLAW, anglePos(ArmParameters::ArmDimensions::closeClaw));
}

void clampClaw() {
    ledcWrite(PIN_PWM_CLAW, anglePos(ArmParameters::ArmDimensions::clampClaw));
}

}