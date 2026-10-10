#pragma once

#include <cstdint>

void oscillator_init();

float detectOscillatorFrequency();

float getOscillatorFrequency();

void setOscillatorSampling(bool enabled); // app.cpp arms/disarms sampling per phase

bool getOscillatorDetected();

void setOscillatorThreshold(float threshold); // if we want to set the threshold in app.cpp