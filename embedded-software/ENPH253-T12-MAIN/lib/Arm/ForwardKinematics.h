#pragma once

#include <types.h>

Vector3D computeForwardKinematics(float alpha, float beta, float gamma);
Vector3D computeForwardKinematics(ArmParameters::ArmAngles armAngles);
