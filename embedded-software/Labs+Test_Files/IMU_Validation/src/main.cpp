#include <Arduino.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// -------------------- IMU ---------------------------------
// IMU
constexpr int PIN_I2C_IMU_SDA{47};
constexpr int PIN_I2C_IMU_SCL{48};

Adafruit_MPU6050 mpu;

void setup() {

  Serial.begin(115200);

  Wire.begin(PIN_I2C_IMU_SDA, PIN_I2C_IMU_SCL);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found — check wiring/address");
    while (1) {
      delay(1000);
    }
  }

  delay(2000); 

  Serial.println("MPU6050 found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  Serial.print("Gyro Z (rad/s): ");
  Serial.print(gyro.gyro.z);
  Serial.print("   Accel X/Y/Z: ");
  Serial.print(accel.acceleration.x);
  Serial.print(", ");
  Serial.print(accel.acceleration.y);
  Serial.print(", ");
  Serial.println(accel.acceleration.z);

  delay(100);
  }