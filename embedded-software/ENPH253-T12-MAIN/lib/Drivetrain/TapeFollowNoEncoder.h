#pragma once
 
struct DutyCycleTargets {
  int dutyLeft;
  bool forwardLeft;
  int dutyRight;
  bool forwardRight;
};
 
DutyCycleTargets tapeFollowNoEncoder();
void tapeFollowNoEncoderSetKp(float newKp);
void tapeFollowNoEncoderSetKd(float newKd);
void setTapeFollowNoEncoderBaseDuty(float newBaseDuty);