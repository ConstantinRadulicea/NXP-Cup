#ifndef __IMU_DATA_H__
#define __IMU_DATA_H__

#include <stdint.h>

#define MPU9250_ADDR 0x68
#define MAG_ADDR     0x0C

void imu_data_setup();

int8_t imu_data_is_valid();
float imu_get_yaw_rate_rad_s();
void mpu9250_data_ready_isr();

#endif