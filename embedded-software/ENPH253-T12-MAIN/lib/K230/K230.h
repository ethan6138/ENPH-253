#pragma once

#include <Arduino.h>

struct Detection {
  String label{};
  int classId{};
  int x1{}, y1{}, x2{}, y2{};
  float score{};
};

struct TeletubbySighting {
  uint32_t timestampMs{};
  float score{};
  int x1{}, y1{}, x2{}, y2{};
};

constexpr int maxDetections{8};

void k230_init(void);

int getDetectionCount(); 
Detection getDetection(int index); 

void k230_setPhase(int phase);
void k230_blinkLight();

void k230_startTeletubbyWatch();
float k230_stopTeletubbyWatch();

void k230_setTeletubbyXMarginFraction(float frac);
void k230_setTeletubbyYMarginFraction(float frac);

void k230_setTeletubbyForegroundMinAreaPx(float areaPx);
void k230_setTeletubbyMiddleXTargetPx(int px);
void k230_setTeletubbyMiddleXBandPx(int px);