#include "Imu.h"
#include "Kalman.h"

#include <helpers.h>
#include <pins.h>
#include <types.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Wire.h>

namespace {
constexpr TickType_t imuTaskPeriodMs{10};

// TODO add in offsets
constexpr float rollOffsetDeg{1.6f};
constexpr float pitchOffsetDeg{3.6f};

Adafruit_MPU6050 mpu;
Kalman kalmanRoll;
Kalman kalmanPitch;

unsigned long lastUpdateMicros = 0;
bool firstRun = true;

Imu::IMUAngles currentAngles{};
Imu::IMUAccel currentAccel{};
float currentYawRate{0.0f};
SemaphoreHandle_t imuMutex{nullptr};

void setAngles(const Imu::IMUAngles &angles) {
  if (xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
    currentAngles = angles;
    xSemaphoreGive(imuMutex);
  }
}

void setAccel(const Imu::IMUAccel &accel) {
  if (xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
    currentAccel = accel;
    xSemaphoreGive(imuMutex);
  }
}

void setYawRate(float rate) {
  if (xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
    currentYawRate = rate;
    xSemaphoreGive(imuMutex);
  }
}

void imuTask(void *pvParameters) {
  lastUpdateMicros = micros();
  for (;;) {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    unsigned long now{micros()};
    float dt{(now - lastUpdateMicros) / Conversions::usToS};
    lastUpdateMicros = now;

    setAccel(
        {accel.acceleration.x, accel.acceleration.y, accel.acceleration.z});

    float accelRollDeg{
        radToDeg(atan2(accel.acceleration.y, accel.acceleration.z)) -
        rollOffsetDeg};
    float accelPitchDeg{
        radToDeg(atan2(-accel.acceleration.x,
                       sqrt(accel.acceleration.y * accel.acceleration.y +
                            accel.acceleration.z * accel.acceleration.z))) -
        pitchOffsetDeg};

    float gyroRollRateDegPerSec{radToDeg(gyro.gyro.x)};
    float gyroPitchRateDegPerSec{radToDeg(gyro.gyro.y)};

    if (firstRun) {
      kalmanRoll.setAngle(accelRollDeg);
      kalmanPitch.setAngle(accelPitchDeg);
      firstRun = false;
    }

    float fusedRollDeg{
        kalmanRoll.getAngle(accelRollDeg, gyroRollRateDegPerSec, dt)};
    float fusedPitchDeg{
        kalmanPitch.getAngle(accelPitchDeg, gyroPitchRateDegPerSec, dt)};

    setAngles({degToRad(fusedRollDeg), degToRad(fusedPitchDeg)});
    setYawRate(gyro.gyro.z); // rad/s

    vTaskDelay(pdMS_TO_TICKS(imuTaskPeriodMs));
  }
}
} // namespace

void imu_init() {
  imuMutex = xSemaphoreCreateMutex();
  Wire.begin(PIN_I2C_IMU_SDA, PIN_I2C_IMU_SCL);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found");
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  xTaskCreatePinnedToCore(imuTask, "imuTask", 4096, nullptr, 1, nullptr, 0);
}

Imu::IMUAngles getImuAngles() {
  Imu::IMUAngles resultRad{};
  if (xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
    resultRad = currentAngles;
    xSemaphoreGive(imuMutex);
  }
  return {radToDeg(resultRad.roll), radToDeg(resultRad.pitch)};
}

Imu::IMUAccel getImuAccel() {
  Imu::IMUAccel result{};
  if (xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
    result = currentAccel;
    xSemaphoreGive(imuMutex);
  }
  return result;
}

float getImuYawRate() {
  float resultRad{0.0f};
  if (xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
    resultRad = currentYawRate;
    xSemaphoreGive(imuMutex);
  }
  return radToDeg(resultRad);
}