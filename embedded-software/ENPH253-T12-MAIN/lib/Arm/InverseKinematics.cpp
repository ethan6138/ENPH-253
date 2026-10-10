#include "InverseKinematics.h"
#include "Arm.h"

#include <helpers.h>
#include <types.h>

#include <cmath>

ArmParameters::ArmAngles
computeInverseKinematics(float x, float y, float z,
                         ArmParameters::ArmAngles currentAngles) {
  namespace arm = ArmParameters::ArmDimensions;

  float radius{sqrtf(std::pow(x, 2.0f) + std::pow(y, 2.0f))};
  float zPrime{z - arm::turretHeight};

  float distance{sqrt(std::pow(radius, 2.0f) + std::pow(zPrime, 2.0f))};

  if (distance > (arm::bicepLength + arm::forearmLength) || distance < 0.0f)
    return currentAngles;

  float phi{acosf((std::pow(distance, 2) + std::pow(arm::bicepLength, 2) -
                   std::pow(arm::forearmLength, 2)) /
                  (2.0f * arm::bicepLength * distance))};

  float psi{acosf((std::pow(arm::bicepLength, 2) + std::pow(arm::forearmLength, 2) -
                   std::pow(distance, 2)) /
                  (2.0f * arm::bicepLength * arm::forearmLength))};

  float theta{asinf(zPrime / distance)};

  // in rads
  float alpha{phi + theta};
  float beta{alpha - static_cast<float>(M_PI) + psi};
  float gamma{atan2f(-x, y)};

  return ArmParameters::ArmAngles{radToDeg(alpha), radToDeg(beta),
                                  radToDeg(gamma)};
}

ArmParameters::ArmAngles
computeInverseKinematics(Vector3D position,
                         ArmParameters::ArmAngles currentAngles) {
  return computeInverseKinematics(position.x, position.y, position.z,
                                  currentAngles);
}
