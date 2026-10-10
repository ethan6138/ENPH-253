#pragma once

#include "TapeFollowEncoder.h"

#include <Arduino.h>

enum class DriveMode {
  IDLE,
  BLIND,
  TAPE,
  TAPE_NO_ENCODER,
  POSITION,
  CAMERA,
  RAW
};

void driveTrain_init();

// drive modes
void driveFollowTape();
void driveFollowTapeNoEncoder();

void driveToPose(float x, float y, float yaw = 0.0f);
void driveToPoseAndWait(float x, float y, float yaw = 0.0f);

void driveToDelta(float r, float yaw = 0.0f);
void driveToDeltaAndWait(float r, float yaw = 0.0f);

void driveBasedOnCamera();
void driveBlind(float rpmLeft, float rpmRight);
void driveStop();
void setDriveToPosX(float x);
void setDriveToPosY(float y);
void setRawDutyCycle(int dutyLeft, bool forwardLeft, int dutyRight,
                     bool forwardRight);

bool isAtTargetPose();
void setTargetRpm(float rpmLeft, float rpmRight);

// Per-wheel live tuning
void setWheelPidKpLeft(float kp);
void setWheelPidKdLeft(float kd);
void setWheelPidKpRight(float kp);
void setWheelPidKdRight(float kd);
void setWheelPidKiLeft(float ki);
void setWheelPidKiRight(float ki);

void setTapeFollowKp(float kp);
void setTapeFollowKd(float kd);
void setTapeFollowBaseSpeed(float rpm);

void setTapeFollowNoEncoderKp(float kp);
void setTapeFollowNoEncoderKd(float kd);
void setTapeFollowNoEncoderBaseSpeed(float duty);

float getTargetRpmLeft();
float getTargetRpmRight();
int getTargetDutyLeft();
int getTargetDutyRight();

int getCurrentDutyLeft();
int getCurrentDutyRight();

const char *getDriveModeString();
TapeSensorStates getTapeSensorState();