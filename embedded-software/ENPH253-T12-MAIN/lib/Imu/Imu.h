#pragma once

#include <types.h>

void imu_init(void);
Imu::IMUAngles getImuAngles(void);
Imu::IMUAccel getImuAccel(void);
float getImuYawRate(void);
