#pragma once

#include <cstdint>

#include "TapeFollowEncoder.h" // RpmTargets

enum class AlignTargetClass { ROCK, TELETUBBY, LOOP, HABITAT };

enum class AlignEdge { TOP, BOTTOM };

void driveAlignToObject(AlignTargetClass targetClass);
bool driveAlignToObjectAndWait(AlignTargetClass targetClass);

void k230Align_init();

void k230AlignSetTargetClass(AlignTargetClass targetClass);

void k230AlignSetTargetPosition(int x, int y);

void k230AlignSetKpX(float kp);
void k230AlignSetKdX(float kd);
void k230AlignSetKpY(float kp);
void k230AlignSetKdY(float kd);

void k230AlignSetBaseApproachRpm(float rpm);

void k230AlignSetDeadbandPxX(float px);
void k230AlignSetDeadbandPxY(float px);
void k230AlignSetMinRpm(float rpm);

void k230AlignSetAlignEdge(AlignEdge edge);

void k230AlignSetStableDurationMs(uint32_t ms);

bool k230AlignIsAligned();

Wheels::RpmTargets k230AlignUpdate();