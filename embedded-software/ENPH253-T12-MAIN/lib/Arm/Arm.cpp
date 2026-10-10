#include "Arm.h"
#include "Claw.h"
#include "ForwardKinematics.h"
#include "InverseKinematics.h"

#include <helpers.h>
#include <pins.h>
#include <types.h>

#include <Arduino.h>
#include <FastAccelStepper.h>
#include <cmath>

namespace {
// private armTask statics
static SemaphoreHandle_t armMutex;
static ArmParameters::ArmAngles currentAngles{};
static ArmParameters::ArmAngles targetAngles{};

enum class ClawState { Open, Closed, Clamped };
ClawState clawState{ClawState::Closed};

// stepper motor engine
FastAccelStepperEngine engine{FastAccelStepperEngine()};
FastAccelStepper *fastStepper = NULL;

static float stepToward(float current, float target, float maxStep) {
  float diff = target - current;
  if (diff > maxStep)
    diff = maxStep;
  if (diff < -maxStep)
    diff = -maxStep;
  return current + diff;
}

struct ServoCalibration {
  float referenceLogicalDeg{};
  float referenceRawDeg{};
  bool inverted{};
};

constexpr ServoCalibration alphaCal{90.0f, 135.0f, false};
constexpr ServoCalibration betaCal{0.0f, 135.0f, true};

float toRawServoAngle(float logicalAngleDeg, const ServoCalibration &cal) {
  float delta = logicalAngleDeg - cal.referenceLogicalDeg;
  return cal.inverted ? (cal.referenceRawDeg - delta)
                      : (cal.referenceRawDeg + delta);
}

// robot bounding box! this is so we dont collide with the bot when we move the
// arm.
struct FootprintBox {
  float xMin{};
  float xMax{};
  float yMin{};
  float yMax{};
};

constexpr FootprintBox footprintRectangleOscillator{-8.5f, 8.5f, 3.5f, 10.0f};
constexpr FootprintBox footprintRectangleChassi{-18.0f, 18.0f, -16.5f, 3.5f};

bool isInsideBox(const FootprintBox &box, float x, float y) {
  return x >= box.xMin && x <= box.xMax && y >= box.yMin && y <= box.yMax;
}

bool insideBotBoundingBox(float x, float y) {
  return isInsideBox(footprintRectangleChassi, x, y) ||
         isInsideBox(footprintRectangleOscillator, x, y);
}

struct AngleLimits {
  float alphaMin, alphaMax;
  float betaMin, betaMax;
};

constexpr AngleLimits insideBoxLimits{6.0f, 145.0f, -80.0f, 48.0f};
constexpr AngleLimits outsideBoxLimits{6.0f, 130.0f, -70.0f, 48.0f};

ArmParameters::ArmAngles
clampToMechanicalLimits(ArmParameters::ArmAngles target) {
  Vector3D pos = computeForwardKinematics(target);
  const AngleLimits &limits =
      insideBotBoundingBox(pos.x, pos.y) ? insideBoxLimits : outsideBoxLimits;
  target.alpha = constrain(target.alpha, limits.alphaMin, limits.alphaMax);
  target.beta = constrain(target.beta, limits.betaMin, limits.betaMax);
  return target;
}

// waypoints!
constexpr int maxWaypoints = 2;
ArmParameters::ArmAngles waypointQueue[maxWaypoints];
int waypointCount{1};
int waypointIndex{0};

int planSafePath(ArmParameters::ArmAngles from, ArmParameters::ArmAngles to,
                 ArmParameters::ArmAngles *outQueue) {
  Vector3D fromPos = computeForwardKinematics(from);
  Vector3D toPos = computeForwardKinematics(to);

  Vector3D waypoint;

  if (insideBotBoundingBox(fromPos.x, fromPos.y)) {
    waypoint = {toPos.x, toPos.y, fromPos.z};
  } else {
    waypoint = {fromPos.x, fromPos.y, toPos.z};
  }

  ArmParameters::ArmAngles waypointAngles =
      computeInverseKinematics(waypoint, from);
  waypointAngles = clampToMechanicalLimits(waypointAngles);
  outQueue[0] = waypointAngles;
  outQueue[1] = to;
  return 2;
}

void commitTargetAnglesRaw(ArmParameters::ArmAngles target) {
  target = clampToMechanicalLimits(target);
  xSemaphoreTake(armMutex, portMAX_DELAY);
  waypointQueue[0] = target;
  waypointCount = 1;
  waypointIndex = 0;
  targetAngles = target;
  xSemaphoreGive(armMutex);
}

void moveToPositionRawAndWait(Vector3D target) {
  ArmParameters::ArmAngles current;
  xSemaphoreTake(armMutex, portMAX_DELAY);
  current = currentAngles;
  xSemaphoreGive(armMutex);
  commitTargetAnglesRaw(computeInverseKinematics(target, current));
  while (!isArmAtTarget())
    vTaskDelay(pdMS_TO_TICKS(10));
}

static void armTask(void *pvParameters) {
  const TickType_t period{pdMS_TO_TICKS(5)};
  TickType_t lastWake{xTaskGetTickCount()};

  for (;;) {
    xSemaphoreTake(armMutex, portMAX_DELAY);

    if (waypointIndex < waypointCount - 1 &&
        fabsf(currentAngles.alpha - targetAngles.alpha) < 0.05f &&
        fabsf(currentAngles.beta - targetAngles.beta) < 0.05f &&
        fabsf(currentAngles.gamma - targetAngles.gamma) < 0.05f) {
      waypointIndex++;
      targetAngles = waypointQueue[waypointIndex];
    }

    currentAngles.alpha =
        stepToward(currentAngles.alpha, targetAngles.alpha,
                   ArmParameters::ArmDimensions::maxDegPerTick);
    currentAngles.beta =
        stepToward(currentAngles.beta, targetAngles.beta,
                   ArmParameters::ArmDimensions::maxDegPerTick);
    currentAngles.gamma =
        stepToward(currentAngles.gamma, targetAngles.gamma,
                   ArmParameters::ArmDimensions::maxDegPerTick);

    xSemaphoreGive(armMutex);

    ledcWrite(PIN_PWM_ALPHA,
              anglePos(toRawServoAngle(currentAngles.alpha, alphaCal)));
    ledcWrite(PIN_PWM_BETA,
              anglePos(toRawServoAngle(currentAngles.beta, betaCal)));

    if (fastStepper) {
      int32_t rawTargetSteps =
          static_cast<int32_t>(currentAngles.gamma * Stepper::stepsPerDegree);
      int32_t targetSteps =
          Stepper::gammaInverted ? -rawTargetSteps : rawTargetSteps;
      fastStepper->moveTo(targetSteps); // absolute angle to 0 that we set
    }

    switch (clawState) {
    case ClawState::Open:
      Claw::openClaw();
      break;
    case ClawState::Closed:
      Claw::closeClaw();
      break;
    case ClawState::Clamped:
      Claw::clampClaw();
      break;
    }

    vTaskDelayUntil(&lastWake, period);
  }
}

} // namespace

void armInit(void) {
  ledcAttach(PIN_PWM_CLAW, Servos::frequency, Servos::resolution);
  ledcAttach(PIN_PWM_ALPHA, Servos::frequency, Servos::resolution);
  ledcAttach(PIN_PWM_BETA, Servos::frequency, Servos::resolution);

  // // for stepper on gamma
  engine.init();

  fastStepper = engine.stepperConnectToPin(PIN_GPIO_STEP);

  if (fastStepper) {
    fastStepper->setDirectionPin(PIN_GPIO_DIR);
    // fastStepper->setEnablePin(PIN_GPIO_ENA);
    fastStepper->setAutoEnable(true);

    fastStepper->setSpeedInHz(160);
    fastStepper->setAcceleration(1000);
    fastStepper->setCurrentPosition(0); // hold arm straight during startup
  }

  armMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(armTask, "arm", 8192, nullptr, 1, nullptr, 1);
}

ArmParameters::ArmAngles getCurrentArmAngles() {
  xSemaphoreTake(armMutex, portMAX_DELAY);
  ArmParameters::ArmAngles copy = currentAngles;
  xSemaphoreGive(armMutex);
  return copy;
}

ArmParameters::ArmAngles getCurrentArmTargetAngles() {
  xSemaphoreTake(armMutex, portMAX_DELAY);
  ArmParameters::ArmAngles copy = waypointQueue[waypointCount - 1];
  xSemaphoreGive(armMutex);
  return copy;
}

namespace {
void commitTargetAngles(ArmParameters::ArmAngles target) {
  target = clampToMechanicalLimits(target);
  xSemaphoreTake(armMutex, portMAX_DELAY);
  waypointCount = planSafePath(currentAngles, target, waypointQueue);
  waypointIndex = 0;
  targetAngles = waypointQueue[0];
  xSemaphoreGive(armMutex);
}
} // namespace

void moveToPosition(ArmParameters::ArmAngles target) {
  commitTargetAngles(target);
}

void moveToPosition(Vector3D target) {
  if (std::isnan(target.x) || std::isnan(target.y) || std::isnan(target.z)) {
    return; // TODO error handler
  }
  commitTargetAngles(computeInverseKinematics(target, getCurrentArmAngles()));
}

void moveToPositionAndWait(ArmParameters::ArmAngles target) {
  moveToPosition(target);
  while (!isArmAtTarget())
    vTaskDelay(pdMS_TO_TICKS(10));
}

void moveToPositionAndWait(Vector3D target) {
  moveToPosition(target);
  while (!isArmAtTarget())
    vTaskDelay(pdMS_TO_TICKS(10));
}

void moveToPositionX(float x) {
  Vector3D target = getArmTargetPosition();
  target.x = x;
  moveToPosition(target);
}

void moveToPositionY(float y) {
  Vector3D target = getArmTargetPosition();
  target.y = y;
  moveToPosition(target);
}

void moveToPositionZ(float z) {
  Vector3D target = getArmTargetPosition();
  target.z = z;
  moveToPosition(target);
}

void moveToPositionXAndWait(float x) {
  Vector3D target = getArmTargetPosition();
  target.x = x;
  moveToPositionAndWait(target);
}

void moveToPositionYAndWait(float y) {
  Vector3D target = getArmTargetPosition();
  target.y = y;
  moveToPositionAndWait(target);
}

void moveToPositionZAndWait(float z) {
  Vector3D target = getArmTargetPosition();
  target.z = z;
  moveToPositionAndWait(target);
}

void moveToPositionAlpha(float alpha) {
  ArmParameters::ArmAngles target = getCurrentArmTargetAngles();
  target.alpha = alpha;
  moveToPosition(target);
}

void moveToPositionBeta(float beta) {
  ArmParameters::ArmAngles target = getCurrentArmTargetAngles();
  target.beta = beta;
  moveToPosition(target);
}

void moveToPositionGamma(float gamma) {
  ArmParameters::ArmAngles target = getCurrentArmTargetAngles();
  target.gamma = gamma;
  moveToPosition(target);
}

bool isArmAtTarget() {
  xSemaphoreTake(armMutex, portMAX_DELAY);
  bool result = waypointIndex == waypointCount - 1 &&
                fabsf(currentAngles.alpha - targetAngles.alpha) < 0.5f &&
                fabsf(currentAngles.beta - targetAngles.beta) < 0.5f &&
                fabsf(currentAngles.gamma - targetAngles.gamma) < 0.5f;
  xSemaphoreGive(armMutex);
  return result;
}

Vector3D getArmPosition() {
  return computeForwardKinematics(getCurrentArmAngles());
}

Vector3D getArmTargetPosition() {
  return computeForwardKinematics(getCurrentArmTargetAngles());
}

void openClaw() { clawState = ClawState::Open; }

void closeClaw() { clawState = ClawState::Closed; }

void clampClaw() { clawState = ClawState::Clamped; }

void armHoldGammaPosition(void) {
  if (fastStepper) {
    fastStepper->enableOutputs();
  }
}

namespace {

constexpr float clearanceAlpha{90.0f};
constexpr float clearanceBeta{46.0f};

void runDropOffSequence() {
  ArmParameters::ArmAngles current = getCurrentArmAngles();

  ArmParameters::ArmAngles home =
      computeInverseKinematics(ArmParameters::ArmDimensions::presetPositions
                                   [+ArmParameters::ArmPreset::Home],
                               current);

  ArmParameters::ArmAngles dropOff =
      computeInverseKinematics(ArmParameters::ArmDimensions::presetPositions
                                   [+ArmParameters::ArmPreset::DropOff],
                               current);

  // 1: alpha + beta go to home's retracted pose, gamma stays wherever it
  // currently is
  ArmParameters::ArmAngles semiHome{home.alpha, home.beta, current.gamma};
  moveToPositionAndWait(semiHome);

  // 2: alpha + beta rise to the clearance pose, gamma still untouched
  commitTargetAnglesRaw({clearanceAlpha, clearanceBeta, current.gamma});
  while (!isArmAtTarget())
    vTaskDelay(pdMS_TO_TICKS(10));

  // 3: gamma spins to the bin-facing angle, alpha/beta hold at clearance
  commitTargetAnglesRaw({clearanceAlpha, clearanceBeta, dropOff.gamma});
  while (!isArmAtTarget())
    vTaskDelay(pdMS_TO_TICKS(10));

  // 4: alpha + beta lower to the actual drop-off height, gamma holds
  commitTargetAnglesRaw(dropOff);
  while (!isArmAtTarget())
    vTaskDelay(pdMS_TO_TICKS(10));
  // turret didnt fully drop then claw opened
  vTaskDelay(pdMS_TO_TICKS(1000));
  openClaw();
  vTaskDelay(pdMS_TO_TICKS(200));

  // 5: alpha + beta rise back to clearance pose, gamma still at bin-facing
  // angle
  commitTargetAnglesRaw({clearanceAlpha, clearanceBeta, dropOff.gamma});
  while (!isArmAtTarget())
    vTaskDelay(pdMS_TO_TICKS(10));

  // 6: gamma spins back to home's gamma, alpha/beta hold at clearance
  commitTargetAnglesRaw({clearanceAlpha, clearanceBeta, home.gamma});
  while (!isArmAtTarget())
    vTaskDelay(pdMS_TO_TICKS(10));

  // 7: alpha + beta return fully to home, gamma holds
  commitTargetAnglesRaw(home);
  closeClaw();
  while (!isArmAtTarget())
    vTaskDelay(pdMS_TO_TICKS(10));
}
} // namespace

void moveToPreset(ArmParameters::ArmPreset preset) {
  using namespace ArmParameters;
  if (preset == ArmParameters::ArmPreset::DropOff) {
    runDropOffSequence();
    return;
  }

  switch (preset) {
  case ArmPreset::RockPickup:
    // openClaw();
    break;
  case ArmPreset::SolarPanel:
    openClaw();
    break;
  case ArmPreset::HabitatPiece:
    break;
  case ArmPreset::DropOff:
    runDropOffSequence();
    break;
  case ArmPreset::Home:
    // openClaw();
    break;
  case ArmPreset::DriveHome:
    openClaw();
    break;
  default:
    break;
  }

  Vector3D presetPos = ArmParameters::ArmDimensions::presetPositions[+preset];
  Vector3D target = getArmTargetPosition();

  if (!std::isnan(presetPos.x))
    target.x = presetPos.x;
  if (!std::isnan(presetPos.y))
    target.y = presetPos.y;
  if (!std::isnan(presetPos.z))
    target.z = presetPos.z;

  // moveToPosition(target);

  moveToPositionAndWait(target);
  vTaskDelay(500);
  switch (preset) {
  case ArmPreset::RockPickup:
    // moveToPositionYAndWait(20);
    // closeClaw();
    break;
  case ArmPreset::SolarPanel:
    closeClaw();
    break;
  case ArmPreset::HabitatPiece:
    break;
  case ArmPreset::DropOff:
    runDropOffSequence();
    break;
  case ArmPreset::Home:
    break;
  case ArmPreset::DriveHome:
    openClaw();
    break;
  default:
    break;
  }
}