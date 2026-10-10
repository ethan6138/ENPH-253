#pragma once
 
#include <K230Align.h> // for AlignTargetClass
 
#include <cstdint>
  
void k230TurretAlignSetTargetPixel(float xPixel);
void k230TurretAlignSetDeadbandPx(float deadbandPx);
void k230TurretAlignSetKpX(float kp);
void k230TurretAlignSetKdX(float kd);
void k230TurretAlignSetMaxDegPerCycle(float maxDeg);
 
bool k230TurretAlignToObjectAndWait(AlignTargetClass target,
                                    uint32_t timeoutMs = 4000);