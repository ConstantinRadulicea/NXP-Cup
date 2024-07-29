/*
* Copyright 2023 Constantin Dumitru Petre RĂDULICEA
* 
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
* 
*   http://www.apache.org/licenses/LICENSE-2.0
* 
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/



//All defines and Includes

/*
CAR2 FULL JUICE baterie mica
g_enable_car_engine = 0.0;
g_enable_car_steering_wheel = 1.0;
g_enable_emergency_brake = 1.0;
enable_pixy_vector_approximation = 0.0;             
enable_distance_sensor1 = 1.0;
enable_distance_sensor2 = 1.0;
enable_distance_sensor3 = 1.0;

g_lane_width_vector_unit = 53.0;
g_black_color_treshold = 0.2;
g_car_length_cm = 17.5;
g_lookahead_min_distance_cm = 16.0;                       % 22
g_lookahead_max_distance_cm = 40.0;                       % 40
g_min_speed = 97.0;
g_max_speed = 115.0;                                      % 
g_emergency_break_distance_cm = 75.0;                     % 75
g_emergency_brake_min_speed = 94.0;
g_emergency_brake_distance_from_obstacle_cm = 14.0;       % 14
emergency_brake_enable_delay = 0.0;
g_steering_wheel_angle_offset = 0.0;

*/

#ifndef __CONFIG_H__
#define __CONFIG_H__

#define CAR1 1
#define CAR2 0


#define DEBUG_MODE 1
#define RACE_MODE 0
#define TEMP_MODE 0



//#define SPI_SS

#include <Arduino.h>
#include <SPI.h>
#include "SteeringWheel.h"
#include "PurePursuitGeometry.h"
#include "VectorsProcessing.h"
#include "aproximatePixyVector.h"
#include "strtod_.h"
#include "SimpleKalmanFilter.h"
#include "MovingAverage.h"
#include "ReadSerial.h"
#include <vector>
#include "PowerTrain.h"
#include <PWMServo.h>
#include "WifiConnection.h"

#ifdef I2C
  #include <pixy2_libs/host/arduino/libraries/Pixy2/Pixy2I2C.h>
#else 
  #ifdef UART
    #include <pixy2_libs/host/arduino/libraries/Pixy2/Pixy2UART.h>
  #else 
    #ifdef SPI_SS
      #include<pixy2_libs/host/arduino/libraries/Pixy2/Pixy2SPI_SS.h>
    #else
      #include <pixy2_libs/host/arduino/libraries/Pixy2/Pixy2.h>
    #endif
  #endif
#endif


#define ENABLE_SERIAL_PRINT 1
#define ENABLE_STEERING_SERVO 1
#define ENABLE_DRIVERMOTOR 1
#define ENABLE_PIXY_VECTOR_APPROXIMATION 1
#define ENABLE_WIRELESS_DEBUG 1
#define ENABLE_EMERGENCY_BREAKING 1
#define ENABLE_SETTINGS_MENU 1
#define ENABLE_DISTANCE_SENSOR1 1
#define ENABLE_DISTANCE_SENSOR2 1
#define ENABLE_EMERGENCYBRAKE_BACKWARDSBRAKE 1
#define ENABLE_REMOTE_START_STOP 1
#define ENABLE_DETATCH_MENU_AFTER_START_CAR_ENGINE 1
#define ENABLE_FINISH_LINE_DETECTION 1


#define SERIAL_PORT_BAUD_RATE 230400
#define SERIAL_PORT Serial1

#if DEBUG_MODE == 1
  #define ENABLE_SERIAL_PRINT 1
  #define ENABLE_WIRELESS_DEBUG 1
  #define ENABLE_STEERING_SERVO 1
  #define ENABLE_DRIVERMOTOR 1
  #define ENABLE_SETTINGS_MENU 1
  #define ENABLE_EMERGENCY_BREAKING 1
  #define ENABLE_PIXY_VECTOR_APPROXIMATION 0
  #define ENABLE_DISTANCE_SENSOR1 1
  #define ENABLE_DISTANCE_SENSOR2 1
  #define ENABLE_DISTANCE_SENSOR3 1
  #define ENABLE_EMERGENCYBRAKE_BACKWARDSBRAKE 0
  #define ENABLE_REMOTE_START_STOP 0
  #define ENABLE_DETATCH_MENU_AFTER_START_CAR_ENGINE 0
  #define ENABLE_FINISH_LINE_DETECTION 0
#endif

#if RACE_MODE == 1
  #define ENABLE_SERIAL_PRINT 0
  #define ENABLE_WIRELESS_DEBUG 0
  #define ENABLE_STEERING_SERVO 1
  #define ENABLE_DRIVERMOTOR 1
  #define ENABLE_SETTINGS_MENU 1
  #define ENABLE_EMERGENCY_BREAKING 1
  #define ENABLE_PIXY_VECTOR_APPROXIMATION 1
  #define ENABLE_DISTANCE_SENSOR1 1
  #define ENABLE_DISTANCE_SENSOR2 1
  #define ENABLE_DISTANCE_SENSOR3 1
  #define ENABLE_EMERGENCYBRAKE_BACKWARDSBRAKE 1
  #define ENABLE_REMOTE_START_STOP 0
  #define ENABLE_DETATCH_MENU_AFTER_START_CAR_ENGINE 0
  #define ENABLE_FINISH_LINE_DETECTION 0
#endif

#if TEMP_MODE == 1
#define ENABLE_SERIAL_PRINT 0
#define ENABLE_STEERING_SERVO 0
#define ENABLE_DRIVERMOTOR 1
#define ENABLE_PIXY_VECTOR_APPROXIMATION 0
#define ENABLE_WIRELESS_DEBUG 0
#define ENABLE_EMERGENCY_BREAKING 0
#define ENABLE_SETTINGS_MENU 1
#define ENABLE_DISTANCE_SENSOR1 0
#define ENABLE_DISTANCE_SENSOR2 0
#define ENABLE_REMOTE_START_STOP 1
#define ENABLE_DETATCH_MENU_AFTER_START_CAR_ENGINE 1
#endif

#if ENABLE_EMERGENCY_BREAKING == 1 && !(ENABLE_DISTANCE_SENSOR1 == 1 || ENABLE_DISTANCE_SENSOR2 == 1 || ENABLE_DISTANCE_SENSOR3 == 1)
  #define ENABLE_EMERGENCY_BREAKING 0
#endif


#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define abs(x) ({ \
  typeof(x) _x = (x); \
  (_x > 0) ? (_x) : (-_x); \
})

#define STEERING_SERVO_PIN  22 // 3
#define DRIVER_MOTOR_PIN  23

#define REMOTE_START_PIN 20
#define REMOTE_STOP_PIN 10

#define DISTANCE_SENSOR1_TRIG_PIN 7
#define DISTANCE_SENSOR1_ECHO_PIN 6
#define DISTANCE_SENSOR2_TRIG_PIN 5
#define DISTANCE_SENSOR2_ECHO_PIN 4
#define DISTANCE_SENSOR3_TRIG_PIN 3
#define DISTANCE_SENSOR3_ECHO_PIN 2

#define MENU_RIGHT_ARROW_BUTTON_PIN 14
#define MENU_LEFT_ARROW_BUTTON_PIN 15
#define MENU_DECREMENT_BUTTON_PIN 16
#define MENU_INCREMENT_BUTTON_PIN 17

#define EMERGENCY_BREAK_LIGHT_PIN 21

#define SPI_SS_PIXY_1_PIN 8
#define SPI_SS_PIXY_2_PIN 9



#define RPM_SENSOR_LEFT_WHEEL_PIN 2
#define RPM_SENSOR_RIGHT_WHEEL_PIN 4
#define RIGHT_WHEEL_MOTOR_PIN 23
#define LEFT_WHEEL_MOTOR_PIN 22



#include "GlobalVariables.h"
#include "log.h"
#include "LcdMenu.h"
#include "DistanceSensors.h"


/*====================================================================================================================================*/


static void HardwareReset(){
  delay(100);
  SCB_AIRCR = 0x05FA0004;
}

#endif
