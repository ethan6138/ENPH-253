#include "DriveToPose.h"

#include <Pose.h>

#include <helpers.h>
#include <types.h>

#include <cmath>

namespace {
float toleranceCm{2.5f};
float baseRpm{50};
volatile float lastError{};
volatile float lastTurnError{};
volatile float lastRemainingDistance{};
volatile bool targetReached{false};

float kp{83.0f};
float kd{};

float kpTurn{80.0f};
float kdTurn{20.0f};
float minTurnEffort{35.0f};

Wheels::RpmTargets internalTurn(const float theta) {
  Pose::PoseState position{getPose()};

  float targetAngleRad{theta};
  float positionYawRad{degToRad(position.yaw)};

  float error{targetAngleRad - positionYawRad};
  error = wrapAngleRad(error);

  float errorDelta{wrapAngleRad(error - lastTurnError)};
  float turnEffort{kpTurn * error + kdTurn * errorDelta};
  lastTurnError = error;

  if (fabsf(turnEffort) < minTurnEffort) {
    turnEffort = copysignf(minTurnEffort, turnEffort);
  }

  if (fabsf(error) < degToRad(2.0f)) {
    turnEffort = 0.0f;
    targetReached = true; 
  }

  Wheels::RpmTargets targets{-turnEffort, turnEffort};
  return targets;
}
} // namespace

Wheels::RpmTargets driveToPose(const Vector3D &target, bool reverse) {
  Pose::PoseState position{getPose()};

  float dx{target.x - position.x};
  float dy{target.y - position.y};
  float remainingDistance{sqrt(dx * dx + dy * dy)};
  lastRemainingDistance = remainingDistance;

  if (remainingDistance < toleranceCm) {
    lastError = 0.0f;
    targetReached = true;
    return Wheels::RpmTargets(0.0f, 0.0f);
  }

  if (remainingDistance < (toleranceCm + 10)) {
    kp = 0;
  } else {
    kp = 83;
  }

  float targetAngleRad{atan2(dx, dy)};
  if (reverse) {
    // For driving backwards
    targetAngleRad += M_PI;
  }
  float positionYawRad{degToRad(position.yaw)};

  float error{targetAngleRad - positionYawRad};
  error = wrapAngleRad(error);

  float errorDelta{wrapAngleRad(error - lastError)};
  float turnEffort{kp * error + kd * errorDelta};
  lastError = error;

  float directionalBaseRpm{reverse ? -baseRpm : baseRpm};
  Wheels::RpmTargets targets{directionalBaseRpm - turnEffort,
                             directionalBaseRpm + turnEffort};
  return targets;
}

Wheels::RpmTargets driveToDeltaIsh(const float distance, const float theta,
                                   const Pose::PoseState &start) {
  float thetaRad{degToRad(start.yaw + theta)};
  float targetX{start.x + distance * sin(thetaRad)};
  float targetY{start.y + distance * cos(thetaRad)};
  float absDistance(fabsf(distance));
  return (absDistance <= 0.00)
             ? internalTurn(thetaRad)
             : driveToPose(Vector3D{targetX, targetY, 0.0f}, distance < 0.0f);
}

bool isAtTargetPos() { return targetReached; }

void Pose::setDriveToPoseKp(float newKp) { kp = newKp; }
void Pose::setDriveToPoseKd(float newKd) { kd = newKd; }
void Pose::setPoseTolerance(float newTolerance) {toleranceCm = newTolerance; }
void Pose::setPoseRpm(float newBaseRpm) {baseRpm = newBaseRpm; }

void driveToPoseReset() {
  lastError = 0.0f;
  lastTurnError = 0.0f;
  lastRemainingDistance = 0.0f;
  targetReached = false;
}