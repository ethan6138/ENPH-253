#pragma once

#include <Arduino.h>

constexpr int TICKS_PER_WHEEL_REV{240}; // 96 ticks * gear ratio

void encoder_init();

float getEncoderRpmLeft();
float getEncoderRpmRight();

long getEncoderTicksLeft();
long getEncoderTicksRight();
