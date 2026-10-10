#pragma once

#include <types.h>

#include <Arduino.h>

void motorDriver_init();

// 12 bit, 0- 4095.
void setMotorDutyCycle(Wheels::WheelSide wheel, int dutyCycle,
                       bool forward);
void setMotorRpm(Wheels::WheelSide wheel, float targetRpm);