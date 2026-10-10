#include "app.h"
#include "test.h"

#include "K230TurretAlign.h"
#include <Arm.h>
#include <DriveTrain.h>
#include <Encoder.h>
#include <Imu.h>
#include <K230.h>
#include <K230Align.h>
#include <Oscillator.h>
#include <Pose.h>
#include <Telemetry.h>

#include <DriveToPose.h>

#include <pins.h>
#include <sharedState.h>
#include <types.h>

#include <Arduino.h>

// starting block 32, 27

namespace {

void app_init(void) {
  // call all init functions here
  // such as beacon::init, arm::init, etc
  Serial.begin(115200);
  sharedState_init();

  armInit();
  encoder_init(); /// HAS TO HAPPEN BEFORE DRIVETRAIN_INIT
  driveTrain_init();
  imu_init();
  k230_init();
  k230Align_init();
  oscillator_init();
  pose_init();

  telemetry_init(); // telemetry to serial written as a task
}

void k230RockParameters(void){
  k230_setPhase(1);

  // set alignment parameters
  k230AlignSetTargetPosition(400, 440);
  k230AlignSetMinRpm(2.0f); // set to 2.0
  k230AlignSetDeadbandPxX(15.0f);
  k230AlignSetDeadbandPxY(10.0f);
  k230AlignSetKpX(0.3f); // set to 0.1
  k230AlignSetKdX(0.1f); // set to 0.1
  k230AlignSetKpY(0.5f); // set to 0.15
  k230AlignSetKdY(2.0f); // set to 0.6
}

void rockHandle(int rockNumber = 0) {
  if (rockNumber == 5) {
    k230AlignSetAlignEdge(AlignEdge::TOP);
    k230AlignSetTargetPosition(400, 250);
  } else {
    k230AlignSetAlignEdge(AlignEdge::BOTTOM);
    k230AlignSetTargetPosition(400, 440);
  }

  k230_startTeletubbyWatch();

  driveAlignToObjectAndWait(AlignTargetClass::ROCK);
  driveStop();
  vTaskDelay(100);

  if (rockNumber == 5) {
    driveToDeltaAndWait(21.0f, 0.0f);
  } else {
    driveToDeltaAndWait(25.0f, 0.0f);
  }
  driveStop();
  vTaskDelay(100);

  // TODO: run more tests on the confidence threshold
  float confidence = k230_stopTeletubbyWatch();
  // Serial.print("confidence: ");
  // Serial.println(confidence);
  if (confidence > 10.0f) {
    k230_blinkLight();
  }

  setOscillatorSampling(true);
  vTaskDelay(500);

  if (getOscillatorDetected()) {
    // driveBlind(-60, -60);
    driveToDeltaAndWait(-25.0f);
    // vTaskDelay(5000);
    driveStop();
    vTaskDelay(100);

    moveToPreset(ArmParameters::ArmPreset::Home);
    vTaskDelay(100);
    k230TurretAlignToObjectAndWait(AlignTargetClass::ROCK, 500);
    vTaskDelay(1000);
    moveToPreset(ArmParameters::ArmPreset::RockPickup);
    vTaskDelay(250);

    if (rockNumber == 5) {
      driveToDeltaAndWait(11.5f);
    } else {
      driveToDeltaAndWait(15.0f);
    }

    driveStop();
    closeClaw();
    vTaskDelay(500);

    moveToPreset(ArmParameters::ArmPreset::DropOff);
    moveToPreset(ArmParameters::ArmPreset::DriveHome);
  } else {
    // driveBlind(-50, -50);
    driveToDeltaAndWait(-20.0f);
    // vTaskDelay(1000);
    driveStop();
    vTaskDelay(100);
  }
  setOscillatorSampling(false);
}

void rockSequencePt1(void) {
  // Sample to determine threshold frequency at the start
  setOscillatorSampling(true);
  vTaskDelay(1000);
  float round_thresh = getOscillatorDetected();
  // setOscillatorThreshold(round_thresh);
  setOscillatorSampling(false);

  // ---------------------------- ROCK 1 ---------------------------------
  moveToPreset(ArmParameters::ArmPreset::DriveHome);
  // // wait for the arm to move before aligning
  vTaskDelay(500);

  rockHandle(); // first rock

  vTaskDelay(100);

  driveBlind(-80.0f, -80.0f);
  vTaskDelay(400);
  driveStop();

  vTaskDelay(100);

  driveToDeltaAndWait(0.0f, 30.0f);
  driveStop();

  vTaskDelay(100);

  driveToDeltaAndWait(40.0f, 0.0f);
  driveStop();

  // ---------------------------- ROCK 2 ---------------------------------
  moveToPreset(ArmParameters::ArmPreset::DriveHome);

  rockHandle(); // second rock

  vTaskDelay(100);

  driveBlind(-80.0f, -80.0f);
  vTaskDelay(500);
  driveStop();

  driveToDeltaAndWait(0.0f, -35.0f);
  driveStop();

  driveToDeltaAndWait(30.0f, 0.0f);
  driveStop();

  // ---------------------------- ROCK 3 ---------------------------------
  moveToPreset(ArmParameters::ArmPreset::DriveHome);

  rockHandle();

  vTaskDelay(100);

  driveToDeltaAndWait(20.0f, 0.0f);
  driveStop();

  driveToDeltaAndWait(0.0f, 90.0f);
  driveStop();

  vTaskDelay(100);

  // ---------------------------- ROCK 4 ---------------------------------
  moveToPreset(ArmParameters::ArmPreset::DriveHome);

  rockHandle();

  driveStop();
  vTaskDelay(100);

  driveBlind(-60, -60);
  vTaskDelay(1500);
  driveStop();
  vTaskDelay(1500);
}

void rockSequencePt2(void) { 
  driveFollowTape();
  while (!(getRampBottomDetected() && getRampTopDetected())) {
    vTaskDelay(10);
  }
  driveStop();
  vTaskDelay(300);

  // ---------------------------- ROCK 5 ---------------------------------
  moveToPreset(ArmParameters::ArmPreset::DriveHome);

  rockHandle(5);
  driveStop();
  vTaskDelay(100);

  driveToDeltaAndWait(0.0f, -90.0f);
  driveStop();
  vTaskDelay(100);

  driveBlind(-60.0f, -60.0f);
  vTaskDelay(500);
  driveStop();
  vTaskDelay(100);

  // ---------------------------- ROCK 6 ---------------------------------
  moveToPreset(ArmParameters::ArmPreset::DriveHome);

  rockHandle();

  vTaskDelay(100);

  driveToDeltaAndWait(0.0f, 35.0f);
  driveStop();
  vTaskDelay(100);

  k230_setPhase(2); // for the solar panel later

  driveFollowTape();
  while (getPose().y < 110.0f) {
    vTaskDelay(10);
  }
  driveStop();
  vTaskDelay(100);

  float currentAngle = getPose().yaw;
  driveToDeltaAndWait(0.0f, (-90.0f - currentAngle));
  driveStop();
  vTaskDelay(100);
}

void solarPanel(void) {
  // set alignment parameters
  k230AlignSetTargetPosition(450, 200);
  k230AlignSetMinRpm(10.0f); // set to 2.0
  k230AlignSetDeadbandPxX(10.0f);
  k230AlignSetDeadbandPxY(10.0f);
  k230AlignSetKpX(0.3f);  // set to 0.1
  k230AlignSetKdX(0.05f); // set to 0.1
  k230AlignSetKpY(0.5f);  // set to 0.15
  k230AlignSetKdY(2.0f);  // set to 0.6

  moveToPreset(ArmParameters::ArmPreset::DriveHome);
  vTaskDelay(100);

  driveAlignToObjectAndWait(AlignTargetClass::LOOP);
  driveStop();
  vTaskDelay(100);

  driveToDeltaAndWait(8.0f, 0.0f);
  driveStop();
  vTaskDelay(100);

  k230TurretAlignSetTargetPixel(450.0f);
  k230TurretAlignSetDeadbandPx(10.0f);
  k230TurretAlignSetKpX(0.008f);
  k230TurretAlignSetKdX(0.0f);
  k230TurretAlignToObjectAndWait(AlignTargetClass::LOOP);
  float loopPullAngle = getCurrentArmAngles().gamma;

  driveToDeltaAndWait(6.8f, 0.0f);
  driveStop();
  vTaskDelay(100);

  moveToPreset(ArmParameters::ArmPreset::Home);
  moveToPositionZ(12.5f);
  moveToPositionGamma(loopPullAngle);
  vTaskDelay(250);
  clampClaw();
  vTaskDelay(500);

  driveBlind(-80, -80);
  vTaskDelay(1300);
  driveStop();
  vTaskDelay(100);

  moveToPositionGamma(-45.0f);
  vTaskDelay(300);

  openClaw();
  driveToDeltaAndWait(0.0f, 55.0f);
  moveToPositionGamma(0.0f);
  vTaskDelay(250);
  // moveToPreset(ArmParameters::ArmPreset::Home);
  moveToPreset(ArmParameters::ArmPreset::DriveHome);
  driveStop();
  vTaskDelay(100);
}

void habitat(void) {
  setApproachRpm(40.0f);
  setTapeFollowBaseSpeed(40.0f);
  // while (!(digitalRead(PIN_GPIO_TAPE_1) && digitalRead(PIN_GPIO_TAPE_2) &&
  // digitalRead(PIN_GPIO_TAPE_3) && digitalRead(PIN_GPIO_TAPE_4))) {
  //     driveFollowTape();
  // }
  driveFollowTape();
  waitForTapeCrossing(250);
  driveStop();
  vTaskDelay(100);

  Pose::setPoseTolerance(1.0);
  Pose::setPoseRpm(40.0);

  driveToDeltaAndWait(-9.0f);
  vTaskDelay(1500);
  driveStop();
  vTaskDelay(100);

  driveToDeltaAndWait(0.0f, 93.0f);
  driveStop();
  vTaskDelay(100);

  driveToDeltaAndWait(12.0f, 0.0f);
  driveStop();
  vTaskDelay(100);

  moveToPreset(ArmParameters::ArmPreset::HabitatPiece);
  vTaskDelay(100);

  moveToPositionGamma(-90.0f); // test this out, adjust angle if necessary
  vTaskDelay(1000);

  // float currentBeta = getCurrentArmAngles().beta;
  moveToPositionBeta(-38.0f);
  vTaskDelay(1000);
  // pdMS_TO_TICKS(3000);

  closeClaw();
  vTaskDelay(1000);

  moveToPositionGamma(-80.0f);
  vTaskDelay(100);
  moveToPositionZAndWait(25.0f);
  vTaskDelay(2000);

  moveToPositionAlpha(90.0f);
  moveToPositionBeta(0.0f);

  vTaskDelay(2000);

  moveToPositionGamma(0.0f);
  vTaskDelay(1000);
  moveToPreset(ArmParameters::ArmPreset::Home);
  vTaskDelay(10000);
}

void missionSequencer(void *pvParameters) {
  for (;;) {
    k230RockParameters();
    // rockHandle();
    rockSequencePt1();
    rockSequencePt2();
    solarPanel();
    // habitat();

    // code to test habitat turret align
    // vTaskDelay(3000);
    // k230_setPhase(3);
    // vTaskDelay(3000);
    // k230TurretAlignSetTargetPixel(450.0f);
    // k230TurretAlignSetDeadbandPx(10.0f);
    // k230TurretAlignSetKpX(0.008f);
    // k230TurretAlignSetKdX(0.0f);
    // k230TurretAlignToObjectAndWait(AlignTargetClass::HABITAT);

    // closeClaw();
    // vTaskDelay(1000);

    // testRawDutyCycle(1300, 1300);

    // setOscillatorSampling(true);

    vTaskDelay(10000000);
  }
}
} // namespace

void app(void) {
  // call set up function
  app_init();

  moveToPreset(ArmParameters::ArmPreset::DriveHome);
  // moveToPositionGamma(0.0f);
  // vTaskDelay(2000);

  // moveToPreset(ArmParameters::ArmPreset::Home);
  // driveFollowTape();

  // moveToPreset(ArmParameters::ArmPreset::DriveHome);

  // driveToPose(00.0f, 20.0f, 0);
  // driveToDelta(0.0f, 45.0f);

  // testChangingDutyCycle();

  // keep arm in position
  // testArmCenterServos();

  // testDashboard();
  // testDashboardAllFields();

  // 1380, 1380
  // testRawDutyCycle(1200, 1300);

  // test 1, drive blind forward, rpm in both wheels
  // testDriveBlindForward();

  // test 2 same thing but with more serial prints
  // testDriveBlindRpmAccuracy();

  // drive to a position
  // driveToPose(0.0f, 0.0f); // x y value relative to where you turned the bot
  // on

  xTaskCreatePinnedToCore(missionSequencer, "mission seq", 65536, nullptr, 1,
                          nullptr, 1);

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
