#pragma once

#include <types.h>

Wheels::RpmTargets driveToPose(const Vector3D &target, bool reverse = false);
Wheels::RpmTargets driveToDeltaIsh(const float distance, const float theta,
                                   const Pose::PoseState &start);

bool isAtTargetPos(void);
void driveToPoseReset(void);

namespace Pose {
void setDriveToPoseKp(float newKp);
void setDriveToPoseKd(float newKd);

void setPoseTolerance(float newTolerance);
void setPoseRpm(float newBaseRpm);

} // namespace Pose