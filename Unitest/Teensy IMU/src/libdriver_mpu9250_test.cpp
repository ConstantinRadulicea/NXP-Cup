#include <Arduino.h>
#include <Wire.h>
#include "driver_mpu9250_basic.h"
int decimals = 6;

  void mpu9250_data_ready_isr() {
    mpu9250_basic_irq_handler();
    float accel[3], gyro[3], mag[3], temp;
    if (mpu9250_basic_read(accel, gyro, mag) == 0) {
  
      Serial.print(accel[0], decimals); Serial.print(";");
      Serial.print(accel[1], decimals); Serial.print(";");
      Serial.print(accel[2], decimals); Serial.print(";");

      Serial.print(gyro[0], decimals); Serial.print(";");
      Serial.print(gyro[1], decimals); Serial.print(";");
      Serial.print(gyro[2], decimals); Serial.print(";");

      Serial.print(mag[0], decimals); Serial.print(";");
      Serial.print(mag[1], decimals); Serial.print(";");
      Serial.print(mag[2], decimals); Serial.print(";");
      Serial.println();
    } else {
      Serial.println("Failed to read data.");
    }
  }

#define MPU9250_ADDR 0x68
#define MAG_ADDR     0x0C

void setup() {
    int16_t temp;
    Serial.begin(115200);
    while (!Serial) {}  // Wait for Serial on Leonardo/Micro
  
    if (mpu9250_basic_init(MPU9250_INTERFACE_IIC, MPU9250_ADDRESS_0x68) != 0) {
      Serial.println("MPU9250 init failed!");
      while (1);
    }
    mpu9250_gyro_offset_convert_to_register(&gs_handle, 2.27f, &temp);
    mpu9250_set_gyro_x_offset(&gs_handle, temp);

    mpu9250_gyro_offset_convert_to_register(&gs_handle, -1.9f, &temp);
    mpu9250_set_gyro_y_offset(&gs_handle, temp);

    mpu9250_gyro_offset_convert_to_register(&gs_handle, 0.4f, &temp);
    mpu9250_set_gyro_z_offset(&gs_handle, temp);

    pinMode(2, INPUT);
    attachInterrupt(digitalPinToInterrupt(2), mpu9250_data_ready_isr, FALLING);
    Serial.println("MPU9250 initialized.");
  }

  
  
  void loop() {
//    float accel[3], gyro[3], mag[3], temp;
//    noInterrupts();
//    if (mpu9250_basic_read(accel, gyro, mag) == 0) {
//  
//      Serial.print(accel[0], decimals); Serial.print(";");
//      Serial.print(accel[1], decimals); Serial.print(";");
//      Serial.print(accel[2], decimals); Serial.print(";");
//
//      Serial.print(gyro[0], decimals); Serial.print(";");
//      Serial.print(gyro[1], decimals); Serial.print(";");
//      Serial.print(gyro[2], decimals); Serial.print(";");
//
//      Serial.print(mag[0], decimals); Serial.print(";");
//      Serial.print(mag[1], decimals); Serial.print(";");
//      Serial.print(mag[2], decimals); Serial.print(";");
//      Serial.println();
//    } else {
//      Serial.println("Failed to read data.");
//    }
//  interrupts();
    delay(1000);  // 2Hz update
  }