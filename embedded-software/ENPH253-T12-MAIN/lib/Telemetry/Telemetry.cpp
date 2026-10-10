#include "Telemetry.h"

#include <Arm.h>
#include <DriveTrain.h>
#include <Encoder.h>
#include <Imu.h>
#include <K230.h>
#include <Oscillator.h>
#include <Pose.h>
// #include <Sonar.h>  // uncomment once Sonar is in

#include <Arduino.h>

constexpr TickType_t telemetryTaskPeriodMs{150};

void readCommands() {
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    int c = line.indexOf(':');
    if (c <= 0)
      continue;

    String key = line.substring(0, c);
    float val = line.substring(c + 1).toFloat();

    if (key == "WPKPL")
      setWheelPidKpLeft(val);
    else if (key == "WPKDL")
      setWheelPidKdLeft(val);
    else if (key == "WPKPR")
      setWheelPidKpRight(val);
    else if (key == "WPKDR")
      setWheelPidKdRight(val);
    else if (key == "WPKIL")
      setWheelPidKiLeft(val);
    else if (key == "WPKIR")
      setWheelPidKiRight(val);
    else if (key == "TARGETRPM")
      driveBlind(val, val);
    else if (key == "TFKP")
      setTapeFollowKp(val);
    else if (key == "TFKD")
      setTapeFollowKd(val);
    else if (key == "BASE")
      setTapeFollowBaseSpeed(val);
    else if (key == "TFNEKP")
      setTapeFollowNoEncoderKp(val);
    else if (key == "TFNEKD")
      setTapeFollowNoEncoderKd(val);
    else if (key == "TFNEBASE")
      setTapeFollowNoEncoderBaseSpeed(val);
    else if (key == "RESETPOSE")
      resetPose(Pose::ResetPoint::Start);
    else if (key == "DRIVETOPOSX")
      setDriveToPosX(val);
    else if (key == "DRIVETOPOSY")
      setDriveToPosY(val);
    // else if (key == "DRIVETOPOSD")
    //   setDriveToPosD(val);
    // else if (key == "DRIVETOPOSTHETA")
    //   setDriveToPosTheta(val);
    else if (key == "ARMX")
      moveToPositionX(val);
    else if (key == "ARMY")
      moveToPositionY(val);
    else if (key == "ARMZ")
      moveToPositionZ(val);
    else if (key == "ARMALPHA")
      moveToPositionAlpha(val);
    else if (key == "ARMBETA")
      moveToPositionBeta(val);
    else if (key == "ARMGAMMA")
      moveToPositionGamma(val);
    else if (key == "ARMPRESET")
      moveToPreset(
          static_cast<ArmParameters::ArmPreset>(static_cast<int>(val)));
    else if (key == "CLAWOPEN")
      openClaw();
    else if (key == "CLAWCLOSE")
      closeClaw();
  }
}

void sendTelemetry() {
  Vector3D armPos = getArmPosition();
  ArmParameters::ArmAngles armAngles = getCurrentArmAngles();
  TapeSensorStates tape = getTapeSensorState();
  Imu::IMUAngles angles = getImuAngles();
  Pose::PoseState pose = getPose();

  String line;
  line.reserve(300);

  line += "RollDeg:" + String(angles.roll);
  line += " PitchDeg:" + String(angles.pitch);
  line += " YawDeg:" + String(pose.yaw);

  line += " PoseX:" + String(pose.x);
  line += " PoseY:" + String(pose.y);
  line += " PoseTheta:" + String(pose.yaw);

  line += " Alpha:" + String(armAngles.alpha);
  line += " Beta:" + String(armAngles.beta);
  line += " Gamma:" + String(armAngles.gamma);
  line += " ArmX:" + String(armPos.x);
  line += " ArmY:" + String(armPos.y);
  line += " ArmZ:" + String(armPos.z);

  line += " TargetRpmLeft:" + String(getTargetRpmLeft());
  line += " RpmLeft:" + String(getEncoderRpmLeft());
  line += " TargetRpmRight:" + String(getTargetRpmRight());
  line += " RpmRight:" + String(getEncoderRpmRight());
  line += " DutyLeft:" + String(getCurrentDutyLeft());
  line += " DutyRight:" + String(getCurrentDutyRight());
  line += " TicksLeft:" + String(getEncoderTicksLeft());
  line += " TicksRight:" + String(getEncoderTicksRight());
  line += " Mode:" + String(getDriveModeString());
  line += " TargetDutyLeft:" + String(getTargetDutyLeft());
  line += " TargetDutyRight:" + String(getTargetDutyRight());

  line += " Tape1:" + String(tape.s1 ? 1 : 0);
  line += " Tape2:" + String(tape.s2 ? 1 : 0);
  line += " Tape3:" + String(tape.s3 ? 1 : 0);
  line += " Tape4:" + String(tape.s4 ? 1 : 0);
  line += " Tape5:" + String(tape.s5 ? 1 : 0);

  line += " Oscillator:" + String(getOscillatorFrequency());
  line += " OscillatorDetected:" + String(getOscillatorDetected() ? 1 : 0);
  // line += " SonarCm:" + String(getSonarDistanceCm());

  int detectionCount = getDetectionCount();
  line += " K230Count:" + String(detectionCount);

  if (detectionCount > 0) {
    Detection primary = getDetection(0);
    int centerX = (primary.x1 + primary.x2) / 2;
    int centerY = (primary.y1 + primary.y2) / 2;

    line += " K230Label:" + primary.label;
    line += " K230Score:" + String(primary.score);
    line += " K230X:" + String(centerX);
    line += " K230Y:" + String(centerY);
  } else {
    line += " K230Label:none";
    line += " K230Score:0";
    line += " K230X:0";
    line += " K230Y:0";
  }

  Serial.println(line);
}

void telemetryTask(void *pvParameters) {
  for (;;) {
    readCommands();
    sendTelemetry();
    vTaskDelay(pdMS_TO_TICKS(telemetryTaskPeriodMs));
  }
}

void telemetry_init() {
  xTaskCreatePinnedToCore(telemetryTask, "telemetryTask", 4096, nullptr, 1,
                          nullptr, 0);
}