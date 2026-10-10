#include "MoveToPosition.h"
#include "InverseKinematics.h"

#include <cmath>

Vector3D getPresetPosition(ArmParameters::ArmPreset preset) {
  return ArmParameters::ArmDimensions::presetPositions[+preset];
}

void moveArmToPosition(Vector3D target) {
  if (std::isnan(target.x) || std::isnan(target.y) || std::isnan(target.z)) {
    return; // TODO error handler
  }

  ArmParameters::ArmAngles current{getCurrentArmAngles()};
  ArmParameters::ArmAngles solved{computeInverseKinematics(target, current)};
  setArmTargetAngles(solved);
}

void moveArmToPosition(ArmParameters::ArmAngles target) {
  setArmTargetAngles(target);
}

namespace {
void waitUntilArmAtTarget() {
  while (!isArmAtTarget()) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
} // namespace

void moveArmToPositionAndWait(Vector3D target) {
  moveArmToPosition(target);
  waitUntilArmAtTarget();
}

void moveArmToPositionAndWait(ArmParameters::ArmAngles target) {
  moveArmToPosition(target);
  waitUntilArmAtTarget();
}