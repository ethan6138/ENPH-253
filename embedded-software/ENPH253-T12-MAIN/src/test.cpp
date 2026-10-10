#include "test.h"

#include <Arm.h>
#include <DriveTrain.h>
#include <Encoder.h>
#include <Oscillator.h>
#include <Telemetry.h>

#include <sharedState.h>

#include <Arduino.h>

void testArmCenterServos() {
  while (1) {
    moveToPosition(ArmParameters::ArmAngles{30.0f, 45.0f, 0.0f});
    closeClaw();

    vTaskDelay(1000);
  }
}

void testRawDutyCycle(int leftDuty, int rightDuty) {
  // duty, forward?, duty, forward?
  setRawDutyCycle(leftDuty, true, rightDuty, true);

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

volatile int setDuty{3150};
volatile bool lastCeiling{false};
constexpr int dutyCeiling{4000};
constexpr int dutyFloor{0};
constexpr int stepSize{50};

void testChangingDutyCycle(void) {

  while (1) {
    if ((setDuty < dutyCeiling) && (setDuty >= dutyFloor) && !lastCeiling) {
      setDuty += stepSize;
    } else if (setDuty == dutyCeiling) {
      lastCeiling = true;
      setDuty -= stepSize;
    } else if ((setDuty <= dutyCeiling) && (setDuty > dutyFloor) &&
               lastCeiling) {
      setDuty -= stepSize;
    } else if (setDuty == dutyFloor) {
      lastCeiling = false;
      setDuty += stepSize;
    } else {
      Serial.println("Sumsum you fucked up");
    }

    // Serial.print("setDuty: ");
    // Serial.println(setDuty);

    setRawDutyCycle(setDuty, true, setDuty, true);

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void testDriveBlindForward(void) {
  driveBlind(60.0f, 60.0f); // both wheels forward at 60 RPM, change as needed

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void testDriveBlindRpmAccuracy(void) {
  constexpr float targetRpm{
      0.0f}; // change this to test different target speeds
  driveBlind(targetRpm, targetRpm);

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void driveToPositionAndWait(float x, float y, float yaw) {
  driveToPose(x, y);
  while (!isAtTargetPose()) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void testDriveFollowTape(void) {
  driveFollowTape();

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void testDriveFollowTapeNoEncoder(void) {
  driveFollowTapeNoEncoder();

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void testDriveAlignToObject(AlignTargetClass targetClass) {
  driveAlignToObject(targetClass);

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void testOscillatorFrequency(void) {
  setOscillatorSampling(true);

  while (1) {
    Serial.print("OscillatorHz:");
    Serial.print(getOscillatorFrequency());
    Serial.print(" OscillatorDetected:");
    Serial.println(getOscillatorDetected() ? 1 : 0);

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// depricated
void testDashboard(void) {
  driveFollowTape(); // or driveBlind(...) if you want to test without tape
}
//   while (1) {
//     // ---- read incoming tuning commands from dashboard ----
//     while (Serial.available()) {
//       String line = Serial.readStringUntil('\n');
//       line.trim();
//       int c = line.indexOf(':');
//       if (c > 0) {
//         String key = line.substring(0, c);
//         float val = line.substring(c + 1).toFloat();
//         if (key == "WPKP")
//           setWheelPidKp(val);
//         else if (key == "WPKD")
//           setWheelPidKd(val);
//         else if (key == "TFKP")
//           setTapeFollowKp(val);
//         else if (key == "TFKD")
//           setTapeFollowKd(val);
//         else if (key == "BASE")
//           setTapeFollowBaseSpeed(val);
//       }
//     }

//     Vector3D armPos = getArmPosition();

//     // ---- print telemetry in dashboard's expected format ----
//     Serial.print("RollDeg:");
//     Serial.print(0.0f); // swap in real IMU getter once wired in
//     Serial.print(" PitchDeg:");
//     Serial.print(0.0f);
//     Serial.print(" ArmX:");
//     Serial.print(armPos.x);
//     Serial.print(" ArmY:");
//     Serial.print(armPos.y);
//     Serial.print(" ArmZ:");
//     Serial.print(armPos.z);
//     Serial.print(" TargetRpmLeft:");
//     Serial.print(getTargetRpmLeft());
//     Serial.print(" RpmLeft:");
//     Serial.print(getEncoderRpmLeft());
//     Serial.print(" TargetRpmRight:");
//     Serial.print(getTargetRpmRight());
//     Serial.print(" RpmRight:");
//     Serial.print(getEncoderRpmRight());
//     Serial.print(" TicksLeft:");
//     Serial.print(getEncoderTicksLeft());
//     Serial.print(" TicksRight:");
//     Serial.print(getEncoderTicksRight());
//     Serial.print(" Mode:");
//     Serial.print(getDriveModeString());
//     Serial.println();

//     vTaskDelay(pdMS_TO_TICKS(50));
//   }
// }

// void testDashboardAllFields(void) {
//   float t{0.0f};

//   while (1) {
//     // ---- read incoming tuning commands (same as testDashboard) ----
//     while (Serial.available()) {
//       String line = Serial.readStringUntil('\n');
//       line.trim();
//       int c = line.indexOf(':');
//       if (c > 0) {
//         String key = line.substring(0, c);
//         float val = line.substring(c + 1).toFloat();
//         if (key == "WPKP")
//           setWheelPidKp(val);
//         else if (key == "WPKD")
//           setWheelPidKd(val);
//         else if (key == "TFKP")
//           setTapeFollowKp(val);
//         else if (key == "TFKD")
//           setTapeFollowKd(val);
//         else if (key == "BASE")
//           setTapeFollowBaseSpeed(val);
//       }
//     }

//     float s1 = sinf(t);
//     float s2 = sinf(t * 0.6f + 1.0f);
//     float s3 = sinf(t * 1.3f + 2.0f);

//     Serial.print("RollDeg:");
//     Serial.print(15.0f * s1);
//     Serial.print(" PitchDeg:");
//     Serial.print(10.0f * s2);
//     Serial.print(" Alpha:");
//     Serial.print(45.0f + 20.0f * s1);
//     Serial.print(" Beta:");
//     Serial.print(60.0f + 15.0f * s2);
//     Serial.print(" Gamma:");
//     Serial.print(90.0f + 30.0f * s3);
//     Serial.print(" ArmX:");
//     Serial.print(10.0f * s1);
//     Serial.print(" ArmY:");
//     Serial.print(10.0f * s2);
//     Serial.print(" ArmZ:");
//     Serial.print(5.0f + 5.0f * s3);

//     float targetL = 60.0f + 20.0f * s1;
//     float targetR = 60.0f + 20.0f * s2;
//     Serial.print(" TargetRpmLeft:");
//     Serial.print(targetL);
//     Serial.print(" RpmLeft:");
//     Serial.print(targetL - 4.0f * s3);
//     Serial.print(" TargetRpmRight:");
//     Serial.print(targetR);
//     Serial.print(" RpmRight:");
//     Serial.print(targetR - 4.0f * s1);
//     Serial.print(" DutyLeft:");
//     Serial.print(2000.0f + 1500.0f * s1);
//     Serial.print(" DutyRight:");
//     Serial.print(2000.0f + 1500.0f * s2);
//     Serial.print(" TicksLeft:");
//     Serial.print((long)(t * 100.0f));
//     Serial.print(" TicksRight:");
//     Serial.print((long)(t * 98.0f));
//     Serial.print(" Mode:");
//     Serial.print("TAPE");

//     for (int i = 1; i <= 5; i++) {
//       Serial.print(" Tape");
//       Serial.print(i);
//       Serial.print(":");
//       Serial.print((sinf(t + i) > 0.0f) ? 1 : 0);
//     }

//     Serial.print(" Beacon:");
//     Serial.print((s1 > 0.0f) ? 1 : 0);
//     Serial.print(" Light:");
//     Serial.print((s2 > 0.0f) ? 1 : 0);
//     Serial.print(" SonarCm:");
//     Serial.print(30.0f + 20.0f * s3);

//     Serial.print(" PoseX:");
//     Serial.print(50.0f * s1);
//     Serial.print(" PoseY:");
//     Serial.print(50.0f * s2);
//     Serial.print(" OdoX:");
//     Serial.print(50.0f * s1 + 5.0f * s3);
//     Serial.print(" OdoY:");
//     Serial.print(50.0f * s2 + 5.0f * s1);

//     Serial.println();

//     t += 0.05f;
//     vTaskDelay(pdMS_TO_TICKS(50));
//   }
// }