#pragma once

#include <types.h>

#include <Arduino.h>

constexpr unsigned long defaultDoubleTapeCheckTime{500000}; 

constexpr unsigned long defaultTapeCrossingWindowMs{250};

// constexpr unsigned long defaultSideBlipMaxGapMs{5000};

struct TapeSensorStates {
  bool s1, s2, s3, s4, s5;
};

float computeWeightedError();

void tapeFollowReset();

bool tapeCheckStart(int pin, unsigned long timeInterval = defaultDoubleTapeCheckTime);

void waitForTapeCrossing(unsigned long crossingWindowMs = defaultTapeCrossingWindowMs);

void tapeFollow_init();
Wheels::RpmTargets tapeFollow();

void tapeFollowSetKp(float newKp);
void tapeFollowSetKd(float newKd);
void setApproachRpm(float newApproachRpm);
void setBaseSpeedRpm(float newBaseSpeed);

TapeSensorStates getTapeSensorStates();

// void startSideTapeWatch(unsigned long maxGapMs = defaultSideBlipMaxGapMs);
// void stopSideTapeWatch();
// bool sideTapeWatchState();
// // Debug helper - how many blips have been counted so far (0, 1, 2)
// int getSideTapeBlipCount();

