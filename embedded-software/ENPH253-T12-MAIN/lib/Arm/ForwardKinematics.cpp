#include "ForwardKinematics.h"
#include "Arm.h"

#include <helpers.h>
#include <types.h>

#include <cmath>

Vector3D computeForwardKinematics(float alpha, float beta, float gamma) {
  namespace arm = ArmParameters::ArmDimensions;

  float radAlpha{degToRad(alpha)};
  float radBeta{degToRad(beta)};
  float radGamma{degToRad(gamma)};

  float radius =
      arm::bicepLength * cosf(radAlpha) + arm::forearmLength * cosf(radBeta);
  float z = arm::bicepLength * sinf(radAlpha) +
            arm::forearmLength * sinf(radBeta) + arm::turretHeight;

  float x = -radius * sinf(radGamma);
  float y = radius * cosf(radGamma);

  return Vector3D{x, y, z};
}

Vector3D computeForwardKinematics(ArmParameters::ArmAngles armAngles) {
  return computeForwardKinematics(armAngles.alpha, armAngles.beta,
                                  armAngles.gamma);
}