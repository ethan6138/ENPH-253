// shared_state.cpp
#include "sharedState.h"

#include <types.h>

#include <Arduino.h>

namespace {
float turretAngle = 0.0f;
SemaphoreHandle_t turretMutex{nullptr};
} // namespace

void sharedState_init() { turretMutex = xSemaphoreCreateMutex(); }

namespace Turret {
void setTurretAngle(float angle) {
  if (xSemaphoreTake(turretMutex, portMAX_DELAY) == pdTRUE) {
    turretAngle = angle;
    xSemaphoreGive(turretMutex);
  }
}
float getTurretAngle() {
  float result = 0.0f;
  if (xSemaphoreTake(turretMutex, portMAX_DELAY) == pdTRUE) {
    result = turretAngle;
    xSemaphoreGive(turretMutex);
  }
  return result;
}
} // namespace Turret