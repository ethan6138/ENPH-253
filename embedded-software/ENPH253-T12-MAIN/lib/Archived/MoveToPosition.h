#pragma once

#include <types.h>

Vector3D getPresetPosition(ArmParameters::ArmPreset preset);
void moveArmToPosition(Vector3D target);
void moveArmToPosition(ArmParameters::ArmAngles target);
void moveArmToPositionAndWait(Vector3D target);
void moveArmToPositionAndWait(ArmParameters::ArmAngles target);