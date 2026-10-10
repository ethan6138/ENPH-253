#include "K230Align.h"
#include "DriveTrain.h" // driveBasedOnCamera()
#include "K230.h"       // Detection, getDetectionCount(), getDetection()

#include <Arduino.h> // millis()
#include <cmath>

namespace {

AlignTargetClass targetClass{AlignTargetClass::ROCK};

AlignEdge alignEdge{AlignEdge::BOTTOM};

// Default target pixel to align to
int targetX{420};
int targetY{440};

// Need to tune all values on the robot
float kpX{0.1f};
float kdX{0.02f};
float kpY{0.15f};
float kdY{0.6f};

// Optional forward approach speed
float baseApproachRpm{0.0f};

// Pixel deadband, provides effective end condition
float deadbandPxX{20.0f};
float deadbandPxY{20.0f};

// minimum rpm we provide for significant errors
float minMoveRpm{8.0f};

// Lower alpha = more smoothing/lag, higher alpha = snappier but noisier.
float smoothingAlpha{0.5f};

float smoothedCenterX{0.0f};
// float smoothedCenterY{0.0f};
float smoothedEdgeY{0.0f};
bool hasValidTarget{false};

// end condition
volatile bool isAligned{false};

// must hold inside the deadband this long (continuously) before isAligned
// latches true - guards against a single noisy frame reporting alignment
uint32_t stableDurationMs{100};
uint32_t stableStartMs{0};
bool wasStable{false};

// the max missed cycles is how many consecutive frames have to be dropped to
// dismiss the object
int maxMissedCycles{10};
int missCount{0};

constexpr float maxSteerRpm{20.0f};
constexpr float maxForwardRpm{50.0f};

float lastErrorX{0.0f};
float lastErrorY{0.0f};

const char *targetClassLabel() {
  switch (targetClass) {
  case AlignTargetClass::ROCK:
    return "Rock";
  case AlignTargetClass::TELETUBBY:
    return "Teletubby";
  case AlignTargetClass::LOOP:
    return "Loop";
  default:
    return "";
  }
}

float clampf(float value, float limit) {
  if (value > limit)
    return limit;
  if (value < -limit)
    return -limit;
  return value;
}

float applyMinRpm(float rpm, float minRpm) {
  if (rpm == 0.0f)
    return 0.0f;
  float mag = std::fabs(rpm);
  if (mag < minRpm)
    return std::copysign(minRpm, rpm);
  return rpm;
}

// Returns object of target class in K230 detections with the largest detection
// area Returns false if no object of target class is visible
bool findBestDetection(Detection &best) {
  const char *label = targetClassLabel();
  int count = getDetectionCount();

  bool found = false;
  long bestArea = -1;

  for (int i = 0; i < count; i++) {
    Detection d = getDetection(i);
    if (!(d.label.equals(label)))
      continue;

    long width = d.x2 - d.x1;
    long height = d.y2 - d.y1;
    long area = width * height;

    if (area > bestArea) {
      bestArea = area;
      best = d;
      found = true;
    }
  }

  return found;
}

} // namespace

void k230Align_init() {
  lastErrorX = 0.0f;
  lastErrorY = 0.0f;
  wasStable = false;
}

void driveAlignToObject(AlignTargetClass newTargetClass) {
  isAligned = false;
  wasStable = false;
  k230AlignSetTargetClass(newTargetClass);
  driveBasedOnCamera();
}

bool driveAlignToObjectAndWait(AlignTargetClass newTargetClass) {

  driveAlignToObject(newTargetClass);

  while (!k230AlignIsAligned()) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  return true;
}

void k230AlignSetTargetClass(AlignTargetClass newTargetClass) {
  targetClass = newTargetClass;
}

void k230AlignSetTargetPosition(int x, int y) {
  targetX = x;
  targetY = y;
}

void k230AlignSetKpX(float kp) { kpX = kp; }
void k230AlignSetKdX(float kd) { kdX = kd; }
void k230AlignSetKpY(float kp) { kpY = kp; }
void k230AlignSetKdY(float kd) { kdY = kd; }
void k230AlignSetBaseApproachRpm(float rpm) { baseApproachRpm = rpm; }
void k230AlignSetDeadbandPxX(float px) { deadbandPxX = px; }
void k230AlignSetDeadbandPxY(float px) { deadbandPxY = px; }
void k230AlignSetMinRpm(float rpm) { minMoveRpm = rpm; }
void k230AlignSetAlignEdge(AlignEdge edge) { alignEdge = edge; }
void k230AlignSetStableDurationMs(uint32_t ms) { stableDurationMs = ms; }

bool k230AlignIsAligned() { return isAligned; }

// Called from driveTask() each cycle while in DriveMode::CAMERA
Wheels::RpmTargets k230AlignUpdate() {
  Wheels::RpmTargets targets{};

  Detection best{};
  bool detected = findBestDetection(best);

  if (detected) {
    missCount = 0;

    float rawCenterX =
        (best.x1 + best.x2) / 2.0f; // align to the lateral center
    // float rawCenterY = (best.y1 + best.y2) / 2.0f;
    float rawEdgeY = (alignEdge == AlignEdge::BOTTOM) ? best.y2 : best.y1;

    if (!hasValidTarget) {
      // First detection
      smoothedCenterX = rawCenterX;
      // smoothedCenterY = rawCenterY;
      smoothedEdgeY = rawEdgeY;
      hasValidTarget = true;
    } else {
      smoothedCenterX = smoothingAlpha * rawCenterX +
                        (1.0f - smoothingAlpha) * smoothedCenterX;
      // smoothedCenterY = smoothingAlpha * rawCenterY + (1.0f - smoothingAlpha)
      // * smoothedCenterY;
      smoothedEdgeY = smoothingAlpha * rawEdgeY +
                        (1.0f - smoothingAlpha) * smoothedEdgeY;
    }
  } else if (hasValidTarget) {
    // Dropped frames, coast on the last known smoothed position
    missCount++;
    if (missCount >= maxMissedCycles) {
      // Target has genuinely been gone for a while now - stop for real
      hasValidTarget = false;
      missCount = 0;
      lastErrorX = 0.0f;
      lastErrorY = 0.0f;
      wasStable = false;
      isAligned = false;
      return targets;
    }
  } else {
    // Never had a target to begin with (or declared lost), nothing to coast on
    lastErrorX = 0.0f;
    lastErrorY = 0.0f;
    wasStable = false;
    isAligned = false;
    return targets;
  }

  float errorX = static_cast<float>(targetX) - smoothedCenterX;
  float errorY = static_cast<float>(targetY) - smoothedEdgeY;

  // if (!isAligned) {
  //   Serial.print("targetX: ");
  //   Serial.println(targetX);
  //   Serial.print("targetY: ");
  //   Serial.println(targetY);

  //   Serial.print("centerX: ");
  //   Serial.println(smoothedCenterX);
  //   Serial.print("edgeY: ");
  //   Serial.println(smoothedEdgeY);

  //   Serial.print("errorX: ");
  //   Serial.println(errorX);
  //   Serial.print("errorY: ");
  //   Serial.println(errorY);
  // }

  if (std::fabs(errorX) < deadbandPxX)
    errorX = 0.0f;
  if (std::fabs(errorY) < deadbandPxY)
    errorY = 0.0f;

  bool inDeadband = (errorX == 0.0f && errorY == 0.0f);

  // Track how long we've continuously held inside the deadband. Only once
  // that holds for stableDurationMs do we trust it and call it "aligned" -
  // this filters out a single noisy frame briefly crossing into deadband.
  uint32_t now = millis();
  if (inDeadband) {
    if (!wasStable) {
      wasStable = true;
      stableStartMs = now;
    }
  } else {
    wasStable = false;
  }
  bool aligned = wasStable && (now - stableStartMs >= stableDurationMs);

  float steerEffort = kpX * errorX + kdX * (errorX - lastErrorX);
  float forwardEffort =
      baseApproachRpm + kpY * errorY + kdY * (errorY - lastErrorY);

  lastErrorX = errorX;
  lastErrorY = errorY;

  steerEffort = clampf(steerEffort, maxSteerRpm);
  float forwardRpm = clampf(forwardEffort, maxForwardRpm);

  // Might need to flip signs
  targets.rpmLeft = forwardRpm - steerEffort;
  targets.rpmRight = forwardRpm + steerEffort;

  if (inDeadband) {
    // Both axes within deadband, so stop
    targets.rpmLeft = 0.0f;
    targets.rpmRight = 0.0f;
  } else {
    // Outside the deadband, so don't let a small PD output ask for less RPM
    // than the motors can actually deliver
    targets.rpmLeft = applyMinRpm(targets.rpmLeft, minMoveRpm);
    targets.rpmRight = applyMinRpm(targets.rpmRight, minMoveRpm);
  }

  isAligned = aligned;

  // if (!isAligned) {
  //   Serial.print("Target Left RPM cam: ");
  //   Serial.println(targets.rpmLeft);
  //   Serial.print("Target Right RPM cam: ");
  //   Serial.println(targets.rpmRight);

  //   Serial.print("are we aligned?: ");
  //   Serial.println(isAligned);
  // }

  return targets;
}