#pragma once

#include <types.h>

#include <Arduino.h>
#include <cmath>

namespace ServoParameters {
constexpr float minPulseUs{500.0f};
constexpr float maxPulseUs{2500.0f};
constexpr float travelDegrees{270.0f};
constexpr uint32_t pwmResolution{4095UL};
constexpr uint32_t pwmPeriodUs{10000UL};
} // namespace ServoParameters

namespace Conversions {
constexpr float degAndRadConversions{180.0f};
constexpr float usToS{1000000.0f}; 
constexpr float msToUs{0.001f}; 
}

inline float degToRad(float degree) {
  return degree * M_PI / Conversions::degAndRadConversions;
}
inline float radToDeg(float degree) {
  return degree / M_PI * Conversions::degAndRadConversions;
}

inline float wrapAngleDeg(float angleDeg) {
    while (angleDeg > 180.0f)  angleDeg -= 360.0f;
    while (angleDeg < -180.0f) angleDeg += 360.0f;
    return angleDeg;
}

inline float wrapAngleRad(float angleRad) {
    while (angleRad > M_PI)  angleRad -= 2.0f * M_PI;
    while (angleRad < -(M_PI)) angleRad += 2.0f * M_PI;
    return angleRad;
}

inline uint32_t anglePos(float angle) {
  float pulseUs =
      ServoParameters::minPulseUs +
      (angle * (ServoParameters::maxPulseUs - ServoParameters::minPulseUs) /
       ServoParameters::travelDegrees);
  return static_cast<uint32_t>(pulseUs * ServoParameters::pwmResolution /
                                   ServoParameters::pwmPeriodUs +
                               0.5f);
}

// inline const char *targetClassLabel(AlignTargetClass targetClass) {
//   switch (targetClass) {
//   case ROCK:
//     return "Rock";
//   case TELETUBBY:
//     return "Teletubby";
//   case LOOP:
//     return "Loop";
//   default:
//     return "";
//   }
// }