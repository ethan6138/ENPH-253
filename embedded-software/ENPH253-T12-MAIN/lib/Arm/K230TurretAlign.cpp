#include "K230TurretAlign.h"

#include <Arm.h>
#include <K230.h>

#include <Arduino.h>
#include <cmath>

namespace {

AlignTargetClass targetClass{AlignTargetClass::ROCK};

float targetPixel{400.0f};
float deadbandPx{10.0f};
float kpX{0.008f}; // tune on robot, 0.008
float kdX{0.0f};
float maxDegPerCycle{3.0f}; // safety clamp per control loop iteration

constexpr uint32_t controlPeriodMs{100};
constexpr uint32_t stableDurationMs{
    500}; // must hold inside deadband this long to count as "aligned"
// should be 500 ms, but set to like 300000 if testing

const char *targetClassLabel() {
  switch (targetClass) {
  case AlignTargetClass::ROCK:
    return "Rock";
  case AlignTargetClass::TELETUBBY:
    return "Teletubby";
  case AlignTargetClass::LOOP:
    return "Loop";
  case AlignTargetClass::HABITAT:
    return "Habitat";
  default:
    return "";
  }
}

// Returns object of target class in K230 detections with the largest detection
// area Returns false if no object of target class is visible
bool findBestDetection(Detection &best) {
  const char *label = targetClassLabel();
  // Serial.println(label);
  int count = getDetectionCount();
  // Serial.println(count);

  bool found = false;
  long bestArea = -1;

  for (int i = 0; i < count; i++) {
    Detection d = getDetection(i);
    // Serial.println(d.label);
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

void k230TurretAlignSetTargetPixel(float xPixel) { targetPixel = xPixel; }
void k230TurretAlignSetDeadbandPx(float px) { deadbandPx = px; }
void k230TurretAlignSetKpX(float kp) { kpX = kp; }
void k230TurretAlignSetKdX(float kd) { kdX = kd; }
void k230TurretAlignSetMaxDegPerCycle(float maxDeg) { maxDegPerCycle = maxDeg; }

bool k230TurretAlignToObjectAndWait(AlignTargetClass target,
                                    uint32_t timeoutMs) {
  targetClass = target;
  uint32_t startTime = millis();
  uint32_t stableStart = 0;
  bool wasStable = false;
  float prevError = 0.0f;
  bool havePrevError = false;

  // Serial.println("made it here");

  while (millis() - startTime < timeoutMs) {
    // Serial.println("made it here 2");
    Detection det{};
    if (findBestDetection(det)) {
      // Serial.println(findBestDetection(det));
      float centerX = (det.x1 + det.x2) / 2.0f;
      float error = (centerX - targetPixel);

      float derivative = error - prevError;
      prevError = error;
      havePrevError = true;

      if (fabsf(error) <= (deadbandPx)) {
        error = 0.0f;
        if (!wasStable) {
          wasStable = true;
          stableStart = millis();
        } else if (millis() - stableStart >= stableDurationMs) {
          return true; // aligned and holding
        }
      } else {
        wasStable = false;

        float deltaGamma = kpX * error + kdX * derivative;

        deltaGamma = constrain(deltaGamma, -maxDegPerCycle, maxDegPerCycle);

        // Serial.print("deltaGamma: ");
        // Serial.println(deltaGamma);

        ArmParameters::ArmAngles target = getCurrentArmAngles();

        moveToPositionGamma(target.gamma - deltaGamma);
      }
    } else {
      // no matching detection this cycle
      // Serial.println("unfortunately here");
      wasStable = false;
      havePrevError = false;
    }

    vTaskDelay(pdMS_TO_TICKS(controlPeriodMs));
  }

  return false; // timed out without a stable lock
}