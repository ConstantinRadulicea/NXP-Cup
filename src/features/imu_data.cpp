#include <Arduino.h>
#include "Config.h"
#include <Wire.h>
#include "driver_mpu9250_basic.h"
#include <stdint.h>
#include "features/imu_data.h"

volatile static int8_t local_valid_imu_data = 0;
volatile static float local_imu_data_yaw_rate_rad_s = 0.0f;

void mpu9250_data_ready_isr() {
    float accel[3], gyro[3], mag[3];
    if (mpu9250_basic_read(accel, gyro, mag) == 0) {
        local_valid_imu_data = 1;
        local_imu_data_yaw_rate_rad_s = radians(gyro[2]);
    } else {
        local_valid_imu_data = 0;
        local_imu_data_yaw_rate_rad_s = 0.0f;
    }
  }

  void dumb_isr(){
    if(mpu9250_basic_irq_handler() != 0){
      // failed
      local_valid_imu_data = 0;
      local_imu_data_yaw_rate_rad_s = 0.0f;
    }
  }

void imu_data_setup() {
    int16_t temp;
    pinMode(IMU_DATA_INTERRUPT_PIN, INPUT);
    if (mpu9250_basic_init(MPU9250_INTERFACE_IIC, MPU9250_ADDRESS_0x68) != 0) {
      #if ENABLE_SERIAL_PRINT != 0
        SERIAL_PORT.println("% MPU9250 init failed!");
      #endif
      //noInterrupts();
        local_valid_imu_data = 0;
        local_imu_data_yaw_rate_rad_s = 0.0f;
      //interrupts();
        return;
    }
    mpu9250_gyro_offset_convert_to_register(&gs_handle, 2.27f, &temp);
    mpu9250_set_gyro_x_offset(&gs_handle, temp);

    mpu9250_gyro_offset_convert_to_register(&gs_handle, -1.9f, &temp);
    mpu9250_set_gyro_y_offset(&gs_handle, temp);

    mpu9250_gyro_offset_convert_to_register(&gs_handle, 0.4f, &temp);
    mpu9250_set_gyro_z_offset(&gs_handle, temp);

    //noInterrupts();
    local_valid_imu_data = 0;
    //interrupts();
    attachInterrupt(digitalPinToInterrupt(IMU_DATA_INTERRUPT_PIN), dumb_isr, FALLING);
    #if ENABLE_SERIAL_PRINT != 0
      SERIAL_PORT.println("% MPU9250 initialized.");
    #endif
  }

int8_t imu_data_is_valid(){
  int8_t local_value;
  //noInterrupts();
  local_value = local_valid_imu_data;
  //interrupts();
  return local_value;
}

  float imu_get_yaw_rate_rad_s(){
    
    float local_value;
    //noInterrupts();
      //mpu9250_data_ready_isr();
    local_value = local_imu_data_yaw_rate_rad_s;
    //interrupts();
    return local_value;
  }

  