#pragma once

#include "K230Align.h" // for AlignTargetClass

//set arm to 0 deg and 90 deg, claw open 
void testArmCenterServos(void);

// raw input
void testRawDutyCycle(int leftDuty, int rightDuty);

// test changing duty cycle (to check for H-field coupling)
void testChangingDutyCycle(void);

// test driving forward at set rpm
void testDriveBlindForward(void);

// same as above, just easier to update
void testDriveBlindRpmAccuracy(void);

// drive to a position 
void driveToPositionAndWait(float x, float y, float yaw = 0.0f);

// tape follow
void testDriveFollowTape(void);

// tape follow no encoder
void testDriveFollowTapeNoEncoder(void);

// align to object detected by K230
void testDriveAlignToObject(AlignTargetClass targetClass);

// detect oscillator frequency
void testOscillatorFrequency(void);

// depricated
// void testDashboard(void);

// void testDashboardAllFields(void);