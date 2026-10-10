#include "Pose.h"

#include <Encoder.h>
#include <Imu.h>
#include <Kalman.h> // imu

#include <helpers.h>
#include <types.h>

#include <Arduino.h>

namespace {
constexpr float trackWidthCm{23.00f};
constexpr float wheelDiameterCm{8.00f};

constexpr TickType_t poseTaskPeriodMs{20};

constexpr float rampPitchThresholdBottomDeg{6.0f};
constexpr float rampPitchThresholdTopDeg{2.0f};

constexpr long stationaryTickThreshold{
    -1}; // temporarily at -1, should be like 5

// TODO fill in from measured field geometries
// x,y,theta
constexpr Pose::PoseState startPose{0.0f, 0.0f, 0.0f};
constexpr Pose::PoseState rampBottomPose{91.44, 182.88, 180.0f};
constexpr Pose::PoseState rampTopPose{91.44, 60.96, 180.0f};

// imu
Kalman kalmanYaw;

Pose::PoseState currentPose{};
SemaphoreHandle_t poseMutex{nullptr};

long lastLeftTicks{};
long lastRightTicks{};
float encoderHeadingAccumDeg{};

volatile bool rampBottomDetected{false};
volatile bool rampTopDetected{false};

void setPoseInternal(const Pose::PoseState &pose) {
  if (xSemaphoreTake(poseMutex, portMAX_DELAY) == pdTRUE) {
    currentPose = pose;
    xSemaphoreGive(poseMutex);
  }
}

Pose::PoseState lookupResetPose(Pose::ResetPoint point) {
  switch (point) {
  case Pose::ResetPoint::Start:
    return startPose;
  case Pose::ResetPoint::RampBottom:
    return rampBottomPose;
  case Pose::ResetPoint::RampTop:
    return rampTopPose;
  default:
    return startPose;
  }
}

void applyReset(Pose::ResetPoint point) {
  Pose::PoseState poseDeg = lookupResetPose(point);
  setPoseInternal({poseDeg.x, poseDeg.y, degToRad(poseDeg.yaw)});
  encoderHeadingAccumDeg = poseDeg.yaw;
  // kalmanYaw.setAngle(poseDeg.yaw);
}

// internal only
Pose::PoseState getPoseInternally() {
  Pose::PoseState result{};
  if (xSemaphoreTake(poseMutex, portMAX_DELAY) == pdTRUE) {
    result = currentPose;
    xSemaphoreGive(poseMutex);
  }
  return result;
}

float deltaCm(long ticks) {
  return static_cast<float>(ticks) / TICKS_PER_WHEEL_REV *
         (M_PI * wheelDiameterCm);
}

void poseTask(void *pvParameters) {
  lastLeftTicks = getEncoderTicksLeft();
  lastRightTicks = getEncoderTicksRight();

  for (;;) {
    long currentLeft = getEncoderTicksLeft();
    long currentRight = getEncoderTicksRight();

    long deltaLeftTicks{currentLeft - lastLeftTicks};
    long deltaRightTicks{currentRight - lastRightTicks};
    lastLeftTicks = currentLeft;
    lastRightTicks = currentRight;

    bool isStationary = (labs(deltaLeftTicks) <= stationaryTickThreshold) &&
                        (labs(deltaRightTicks) <= stationaryTickThreshold);

    float deltaLeftCm{deltaCm(deltaLeftTicks)};
    float deltaRightCm{deltaCm(deltaRightTicks)};

    float dThetaEncoderDeg{
        radToDeg((deltaRightCm - deltaLeftCm) / trackWidthCm)};
    encoderHeadingAccumDeg += dThetaEncoderDeg;

    // imu
    float yawRateDeg{isStationary ? 0.0f : getImuYawRate()};

    float dt{poseTaskPeriodMs * Conversions::msToUs};

    float fusedThetaDeg{
        kalmanYaw.getAngle(encoderHeadingAccumDeg, yawRateDeg, dt)};

    // float fusedThetaDeg{encoderHeadingAccumDeg};

    float dCenterCm = (deltaLeftCm + deltaRightCm) / 2.0f;
    float fusedThetaRad = degToRad(fusedThetaDeg);

    // yaw measured from +y, CCW pos
    // heading vector = (sin(yaw), cos(yaw))
    Pose::PoseState prev = getPoseInternally();
    Pose::PoseState pose{};
    pose.x = prev.x + dCenterCm * sin(fusedThetaRad);
    pose.y = prev.y + dCenterCm * cos(fusedThetaRad);
    pose.yaw = fusedThetaRad; // stored in radians
    setPoseInternal(pose);

    // auto ramp
    float pitchDeg = getImuAngles().pitch;

    if (!rampBottomDetected && (fabs(pitchDeg) > rampPitchThresholdBottomDeg)) {
      rampBottomDetected = true;
      applyReset(Pose::ResetPoint::RampBottom);
    }
    if (rampBottomDetected && !rampTopDetected &&
        (fabs(pitchDeg) < rampPitchThresholdTopDeg)) {
      rampTopDetected = true;
      applyReset(Pose::ResetPoint::RampTop);
    }

    vTaskDelay(pdMS_TO_TICKS(poseTaskPeriodMs));
  }
}
} // namespace

void pose_init() {
  poseMutex = xSemaphoreCreateMutex();
  applyReset(Pose::ResetPoint::Start);

  xTaskCreatePinnedToCore(poseTask, "poseTask", 4096, nullptr, 1, nullptr, 0);
}

Pose::PoseState getPose() {
  Pose::PoseState result{};
  if (xSemaphoreTake(poseMutex, portMAX_DELAY) == pdTRUE) {
    result = currentPose;
    xSemaphoreGive(poseMutex);
  }

  return {result.x, result.y, wrapAngleDeg(radToDeg(result.yaw))};
}

void setPose(void) {
  if (xSemaphoreTake(poseMutex, portMAX_DELAY) == pdTRUE) {
    currentPose = {};
    xSemaphoreGive(poseMutex);
  }
}

void resetPose(Pose::ResetPoint point) {
  applyReset(point);
  if (point == Pose::ResetPoint::Start) {
    rampBottomDetected = false;
    rampTopDetected = false;
  }
}

bool getRampBottomDetected(void) { return rampBottomDetected; }

bool getRampTopDetected(void) { return rampTopDetected; }