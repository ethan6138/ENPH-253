#include "DriveTrain.h"
#include "DriveToPose.h"
#include "MotorDriver.h"
#include "TapeFollowEncoder.h"
#include "TapeFollowNoEncoder.h"
#include "WheelPID.h"

#include <K230Align.h>
#include <Pose.h>

#include <Encoder.h>

#include <types.h>

namespace {
WheelPIDState leftPIDState;
WheelPIDState rightPIDState;

volatile float targetRpmLeft{0.0f};
volatile float targetRpmRight{0.0f};

volatile int targetDutyLeft{0};
volatile int targetDutyRight{0};

volatile int currentDutyLeft{0};
volatile int currentDutyRight{0};

volatile int rawDutyLeft{0};
volatile bool rawForwardLeft{true};
volatile int rawDutyRight{0};
volatile bool rawForwardRight{true};

volatile float targetPoseX{0.0f};
volatile float targetPoseY{0.0f};
volatile float targetPoseYaw{0.0f};

volatile bool isDelta{};
Pose::PoseState startingPose{};

constexpr TickType_t driveTaskPeriodMs{20}; // must match encoderTaskPeriodMs

DriveMode currentMode{DriveMode::IDLE};

void driveTask(void *pvParameters) {
  for (;;) {
    switch (currentMode) {
    case DriveMode::TAPE: {
      Wheels::RpmTargets t = tapeFollow();
      targetRpmLeft = t.rpmLeft;
      targetRpmRight = t.rpmRight;
      break;
    }
    case DriveMode::TAPE_NO_ENCODER: {
      // Drives duty cycle directly from tape error
      // Skip shared closed-loop tail below entirely
      DutyCycleTargets d = tapeFollowNoEncoder();
      setMotorDutyCycle(Wheels::WheelSide::LEFT, d.dutyLeft, d.forwardLeft);
      setMotorDutyCycle(Wheels::WheelSide::RIGHT, d.dutyRight, d.forwardRight);
      vTaskDelay(pdMS_TO_TICKS(driveTaskPeriodMs));
      continue;
    }
    case DriveMode::POSITION: {
      Wheels::RpmTargets t =
          (isDelta) ? driveToDeltaIsh(targetPoseX, targetPoseY, startingPose)
                    : driveToPose(Vector3D{targetPoseX, targetPoseY, 0.0f});
      targetRpmLeft = t.rpmLeft;
      targetRpmRight = t.rpmRight;
      break;
    }
    case DriveMode::BLIND: {
      break;
    }
    case DriveMode::IDLE: {
      break;
    }
    case DriveMode::CAMERA: {
      Wheels::RpmTargets t = k230AlignUpdate();
      targetRpmLeft = t.rpmLeft;
      targetRpmRight = t.rpmRight;
      break;
    }
    case DriveMode::RAW:
      break;
    }

    // tail
    if (currentMode == DriveMode::RAW) {
      setMotorDutyCycle(Wheels::WheelSide::LEFT, rawDutyLeft, rawForwardLeft);
      setMotorDutyCycle(Wheels::WheelSide::RIGHT, rawDutyRight,
                        rawForwardRight);
      currentDutyLeft = rawDutyLeft;
      currentDutyRight = rawDutyRight;
    } else if (currentMode == DriveMode::TAPE_NO_ENCODER) {
      continue;
    } else {
      float measuredLeft = getEncoderRpmLeft();
      float measuredRight = getEncoderRpmRight();

      WheelOutput leftOut = wheelPID(leftPIDState, targetRpmLeft, measuredLeft);
      WheelOutput rightOut =
          wheelPID(rightPIDState, targetRpmRight, measuredRight);

      setMotorDutyCycle(Wheels::WheelSide::LEFT, leftOut.dutyCycle,
                        leftOut.forward);
      setMotorDutyCycle(Wheels::WheelSide::RIGHT, rightOut.dutyCycle,
                        rightOut.forward);

      currentDutyLeft = leftOut.dutyCycle;
      currentDutyRight = rightOut.dutyCycle;
    }

    vTaskDelay(pdMS_TO_TICKS(driveTaskPeriodMs));
  }
}
} // namespace

void driveFollowTape() {
  tapeFollowReset();
  currentMode = DriveMode::TAPE;
}

void driveFollowTapeNoEncoder() { currentMode = DriveMode::TAPE_NO_ENCODER; }

void driveToPose(float x, float y, float yaw) {
  driveToPoseReset();
  currentMode = DriveMode::POSITION;
  isDelta = false;
  targetPoseX = x;
  targetPoseY = y;
}

void driveToPoseAndWait(float x, float y, float yaw) {
  driveToPose(x, y, yaw);

  while (!isAtTargetPos()) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void driveToDelta(float r, float yaw) {
  driveToPoseReset();
  currentMode = DriveMode::POSITION;
  isDelta = true;
  startingPose = getPose();
  targetPoseX = r;
  targetPoseY = yaw;
}

void driveToDeltaAndWait(float r, float yaw) {
  driveToDelta(r, yaw);

  while (!isAtTargetPose()) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void driveBasedOnCamera() { currentMode = DriveMode::CAMERA; }

void driveBlind(float rpmLeft, float rpmRight) {
  currentMode = DriveMode::BLIND;
  targetRpmLeft = rpmLeft;
  targetRpmRight = rpmRight;
}

void driveStop() {
  currentMode = DriveMode::IDLE;
  targetRpmLeft = 0;
  targetRpmRight = 0;
}

void setDriveToPosX(float x) {
  currentMode = DriveMode::POSITION;
  targetPoseX = x;
}
void setDriveToPosY(float y) {
  currentMode = DriveMode::POSITION;
  targetPoseY = y;
}

void setRawDutyCycle(int dutyLeft, bool forwardLeft, int dutyRight,
                     bool forwardRight) {
  currentMode = DriveMode::RAW;
  rawDutyLeft = dutyLeft;
  rawForwardLeft = forwardLeft;
  rawDutyRight = dutyRight;
  rawForwardRight = forwardRight;
}

bool isAtTargetPose() { return ::isAtTargetPos(); }

void setTargetRpm(float rpmLeft, float rpmRight) {
  targetRpmLeft = rpmLeft;
  targetRpmRight = rpmRight;
}

void setWheelPidKpLeft(float kp) { leftPIDState.kp = kp; }
void setWheelPidKdLeft(float kd) { leftPIDState.kd = kd; }
void setWheelPidKpRight(float kp) { rightPIDState.kp = kp; }
void setWheelPidKdRight(float kd) { rightPIDState.kd = kd; }
void setWheelPidKiLeft(float ki) { leftPIDState.ki = ki; }
void setWheelPidKiRight(float ki) { rightPIDState.ki = ki; }

void setTapeFollowKp(float kp) { tapeFollowSetKp(kp); }
void setTapeFollowKd(float kd) { tapeFollowSetKd(kd); }
void setTapeFollowBaseSpeed(float rpm) { setBaseSpeedRpm(rpm); }
void setTapeFollowNoEncoderKp(float kp) { tapeFollowNoEncoderSetKp(kp); }
void setTapeFollowNoEncoderKd(float kd) { tapeFollowNoEncoderSetKd(kd); }
void setTapeFollowNoEncoderBaseSpeed(float duty) {
  setTapeFollowNoEncoderBaseDuty(duty);
}

void driveTrain_init() {
  motorDriver_init();
  tapeFollow_init();

  // NO LOAD: 15, 5.0, 0.25
  // WITH LOAD: 10.00, 2.00, 0.1
  wheelPID_init(leftPIDState, 9.0f, 2.0f, 0.5f, Wheels::WheelSide::LEFT);

  // NO LOAD: 7, 1.5, 0.15
  // WITH LOAD: 10.0, 2.0, 0.1
  wheelPID_init(rightPIDState, 10.0f, 2.0f, 0.5f, Wheels::WheelSide::RIGHT);

  xTaskCreatePinnedToCore(driveTask, "driveTask", 4096, nullptr, 1, nullptr, 1);
}

float getTargetRpmLeft() { return targetRpmLeft; }
float getTargetRpmRight() { return targetRpmRight; }
int getTargetDutyLeft() { return targetDutyLeft; }
int getTargetDutyRight() { return targetDutyRight; }

int getCurrentDutyLeft() { return currentDutyLeft; }
int getCurrentDutyRight() { return currentDutyRight; }

const char *getDriveModeString() {
  switch (currentMode) {
  case DriveMode::IDLE:
    return "IDLE";
  case DriveMode::BLIND:
    return "BLIND";
  case DriveMode::TAPE:
    return "TAPE";
  case DriveMode::TAPE_NO_ENCODER:
    return "TAPE_NO_ENCODER";
  case DriveMode::POSITION:
    return "POSITION";
  case DriveMode::CAMERA:
    return "CAMERA";
  case DriveMode::RAW:
    return "RAW";
  default:
    return "UNKNOWN";
  }
}

TapeSensorStates getTapeSensorState() { return ::getTapeSensorStates(); }