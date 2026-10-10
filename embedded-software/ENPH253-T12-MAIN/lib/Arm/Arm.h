#pragma once

#include <types.h>

#include <Arduino.h>
#include <cmath>

// ---------------------- Arm Geometry ------------------
namespace ArmParameters::ArmDimensions {
constexpr float bicepLength{12.7f}; // in cm
constexpr float forearmLength{12.7f};
constexpr float turretHeight{13.335f}; // subject to change, need to be measured
constexpr float maxDegPerTick{1.0f};   // armTask step sizes

// FILL IN VALUES HERE PLEASE
constexpr Vector3D presetPositions[] = {
    /* RockPickup    */ {NAN, 18.0, 3.5f}, // y overridden by sonar at call site
    /* SolarPanel    */ {NAN, 10.0f, 20.0f},
    /* HabitatPiece  */ {NAN, 17.5, 6.5f},
    /* DropOff       */ {-3.0, -2.5f, 33.4f},
    /* Home          */ {0.0f, 9.0f, 13.5f},
    /* Drive Home    */ {0.0f, 1.5f, 25.0f}
}; // namespace ArmParameters::ArmDimensions

constexpr float openClaw{140};
constexpr float closeClaw{45};
constexpr float clampClaw{25};

} // namespace ArmParameters::ArmDimensions

// ---------------------- Stepper --------------------

namespace Stepper {
constexpr int motorStepsPerRev{200};    // QSH4218 defualt no m1-3
constexpr int gearRatioNumerator{60};   // turret gear
constexpr int gearRatioDenominator{16}; // motor gear
constexpr float stepsPerDegree =
    (motorStepsPerRev * static_cast<float>(gearRatioNumerator) /
     gearRatioDenominator) /
    360.0f;
constexpr bool gammaInverted{false};
} // namespace Stepper

// ---------------------- Claw Servo --------------------

namespace Servos {
constexpr uint32_t frequency{100};
constexpr uint8_t resolution{12}; // range of 0 - 4095
} // namespace Servos

// ---------------------- Function decl ---------------------
void armInit(void);

// public mutex protected accessors.
ArmParameters::ArmAngles getCurrentArmAngles(void);
void setArmTargetAngles(ArmParameters::ArmAngles target);
bool isArmAtTarget(void);

void moveToPosition(Vector3D target);
void moveToPosition(ArmParameters::ArmAngles target);

void moveToPositionAndWait(Vector3D target);
void moveToPositionAndWait(ArmParameters::ArmAngles target);

void moveToPositionX(float x);
void moveToPositionY(float y);
void moveToPositionZ(float z);

void moveToPositionXAndWait(float x);
void moveToPositionYAndWait(float y);
void moveToPositionZAndWait(float z);
void moveToPositionGammaAndWait(float gamma);

void moveToPositionAlpha(float alpha);
void moveToPositionBeta(float beta);
void moveToPositionGamma(float gamma);

void moveToPreset(ArmParameters::ArmPreset preset);
void armHoldGammaPosition(void);

Vector3D getArmPosition(void);
Vector3D getArmTargetPosition(void);

void openClaw(void);
void closeClaw(void);
void clampClaw(void);