#pragma once

#include "Arm.h"

#include <types.h>

ArmParameters::ArmAngles
computeInverseKinematics(float x, float y, float z,
                         ArmParameters::ArmAngles currentAngles);
ArmParameters::ArmAngles
computeInverseKinematics(Vector3D position,
                         ArmParameters::ArmAngles currentAngles);