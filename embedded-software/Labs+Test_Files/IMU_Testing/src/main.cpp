#include "Kalman.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Wire.h>

// types.h
struct IMUAngles {
  float alpha; // roll
  float beta;  // pitch
};
struct IMUAccel {
  float x;
  float y;
  float z;
};

// helpers
namespace Conversions {
constexpr float degAndRadConversions{180.0f};
}
float degToRad(float degree) {
  return degree * M_PI / Conversions::degAndRadConversions;
}
float radToDeg(float radian) {
  return radian / M_PI * Conversions::degAndRadConversions;
}

// shared state (stubbed for this standalone test — real project uses
// shared_state.h)
IMUAngles currentAngles;
IMUAccel currentAccel;
void setImuAccel(const IMUAccel &accel) { currentAccel = accel; }
IMUAccel getImuAccel() { return currentAccel; }
void setImuAngles(const IMUAngles &angles) { currentAngles = angles; }
IMUAngles getImuAngles() { return currentAngles; }

//-------------------------------------------------------
constexpr int PIN_I2C_IMU_SDA{47};
constexpr int PIN_I2C_IMU_SCL{48};
constexpr TickType_t imuTaskPeriodMs{10}; // placeholder
constexpr float rollOffsetDeg{0.47f};
constexpr float pitchOffsetDeg{-1.43f};

Adafruit_MPU6050 mpu;

Kalman kalmanRoll;
Kalman kalmanPitch;

unsigned long lastUpdateMicros = 0;
bool firstRun = true;

void imuTask(void *pvParameters) {
  lastUpdateMicros = micros();

  for (;;) {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    unsigned long now = micros();
    float dt = (now - lastUpdateMicros) / 1000000.0f;
    lastUpdateMicros = now;

    IMUAccel accelData{accel.acceleration.x, accel.acceleration.y,
                       accel.acceleration.z};
    setImuAccel(accelData);

    // // Raw accel-derived roll/pitch, in degrees (Kalman class expects deg)
    // float accelRollDeg =
    //     radToDeg(atan2(accel.acceleration.y, accel.acceleration.z));
    // float accelPitchDeg =
    //     radToDeg(atan2(-accel.acceleration.x,
    //                    sqrt(accel.acceleration.y * accel.acceleration.y +
    //                         accel.acceleration.z * accel.acceleration.z)));

    // with offset
    float accelRollDeg =
        radToDeg(atan2(accel.acceleration.y, accel.acceleration.z)) -
        rollOffsetDeg;
    float accelPitchDeg =
        radToDeg(atan2(-accel.acceleration.x,
                       sqrt(accel.acceleration.y * accel.acceleration.y +
                            accel.acceleration.z * accel.acceleration.z))) -
        pitchOffsetDeg;

    // Gyro rates in deg/s (Adafruit lib gives rad/s, convert)
    float gyroXDegPerSec = radToDeg(gyro.gyro.x);
    float gyroYDegPerSec = radToDeg(gyro.gyro.y);

    if (firstRun) {
      kalmanRoll.setAngle(accelRollDeg);
      kalmanPitch.setAngle(accelPitchDeg);
      firstRun = false;
    }

    float fusedRollDeg = kalmanRoll.getAngle(accelRollDeg, gyroXDegPerSec, dt);
    float fusedPitchDeg =
        kalmanPitch.getAngle(accelPitchDeg, gyroYDegPerSec, dt);

    IMUAngles angleData{degToRad(fusedRollDeg), degToRad(fusedPitchDeg)};
    setImuAngles(angleData);

    vTaskDelay(pdMS_TO_TICKS(imuTaskPeriodMs));
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(PIN_I2C_IMU_SDA, PIN_I2C_IMU_SCL);

  if (!mpu.begin()) {
    while (1) {
      Serial.println("MPU6050 not found");
      delay(1000);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  xTaskCreatePinnedToCore(imuTask, "imuTask", 4096, nullptr, 1, nullptr, 0);
}

void loop() {
  IMUAngles angles = getImuAngles();

  // Format matches plot_imy.py's expected "RollDeg:xx PitchDeg:xx" pattern
  Serial.print("RollDeg:");
  Serial.print(radToDeg(angles.alpha));
  Serial.print(" PitchDeg:");
  Serial.println(radToDeg(angles.beta));

  delay(50);
}