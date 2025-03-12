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


#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <Arduino.h>

#define CAR_ID 2

#define DEBUG_MODE 0
#define RACE_MODE 0
#define TEMP_MODE 0
#define SINGLE_AXE_MODE 1
#define REAR_AXE_STEERING_MODE 0

#define SERIAL_PORT_BAUD_RATE 230400  //230400
#define SERIAL_PORT_TYPE_CONFIGURATION 1

#if SERIAL_PORT_TYPE_CONFIGURATION == 1
  #define SERIAL_PORT Serial1
  #if defined(TEENSYLC)
    #define SERIAL_PORT_TYPE HardwareSerial
  #elif defined(TEENSY40)
    #define SERIAL_PORT_TYPE HardwareSerialIMXRT
  #endif
#elif SERIAL_PORT_TYPE_CONFIGURATION == 2
  #define SERIAL_PORT Serial
  #define SERIAL_PORT_TYPE usb_serial_class
#endif

//#define SPI_SS


#include <SPI.h>
#include "SteeringWheel.h"
#include "PurePursuitGeometry.h"
//#include "strtod_.h"
#include "MovingAverage.h"
#include "ReadSerial.h"
#include <vector>
#include "PowerTrain.h"
#include <PWMServo.h>
#include "WifiConnection.h"
#include "CalculateCarSpeed.h"
#include "CalculateLookAheadDistance.h"

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
#define ENABLE_SERIAL_PRINT_LIMITED 0
#define ENABLE_STEERING_SERVO 1
#define ENABLE_SINGLE_AXE_STEERING 0
#define ENABLE_SINGLE_AXE_STEERING_NO_RPM 0
#define ENABLE_REAR_AXE_STEERING 0
#define ENABLE_DRIVERMOTOR 1
#define ENABLE_WIRELESS_DEBUG 1
#define ENABLE_EMERGENCY_BREAKING 1
#define ENABLE_SETTINGS_MENU 1
#define ENABLE_DISTANCE_SENSOR1 1
#define ENABLE_DISTANCE_SENSOR2 1
#define ENABLE_DISTANCE_SENSOR3 1
#define ENABLE_DETATCH_MENU_AFTER_START_CAR_ENGINE 1
#define ENABLE_FINISH_LINE_DETECTION 1
#define CAMERA_ILLUMINATION_LIGHT 1
#define ENABLE_WIRELESS_DEBUG_LIMITED 0



#if DEBUG_MODE == 1
  #define ENABLE_SERIAL_PRINT 1
  #define ENABLE_WIRELESS_DEBUG 1
  #define ENABLE_WIRELESS_DEBUG_LIMITED 0
  #define ENABLE_STEERING_SERVO 1
  #define ENABLE_SINGLE_AXE_STEERING 0
  #define ENABLE_SINGLE_AXE_STEERING_NO_RPM 0
  #define ENABLE_DRIVERMOTOR 1
  #define ENABLE_SETTINGS_MENU 1
  #define ENABLE_EMERGENCY_BREAKING 1
  #define ENABLE_DISTANCE_SENSOR1 0
  #define ENABLE_DISTANCE_SENSOR2 1
  #define ENABLE_DISTANCE_SENSOR3 0
  #define ENABLE_DETATCH_MENU_AFTER_START_CAR_ENGINE 0
  #define ENABLE_FINISH_LINE_DETECTION 1
  #define CAMERA_ILLUMINATION_LIGHT 0
#endif

#if RACE_MODE == 1
  #define ENABLE_SERIAL_PRINT 0
  #define ENABLE_WIRELESS_DEBUG 0
  #define ENABLE_STEERING_SERVO 1
  #define ENABLE_SINGLE_AXE_STEERING 0
  #define ENABLE_SINGLE_AXE_STEERING_NO_RPM 0
  #define ENABLE_DRIVERMOTOR 1
  #define ENABLE_SETTINGS_MENU 1
  #define ENABLE_EMERGENCY_BREAKING 1
  #define ENABLE_DISTANCE_SENSOR1 0 
  #define ENABLE_DISTANCE_SENSOR2 1
  #define ENABLE_DISTANCE_SENSOR3 0
  #define ENABLE_DETATCH_MENU_AFTER_START_CAR_ENGINE 0
  #define ENABLE_FINISH_LINE_DETECTION 1
  #define CAMERA_ILLUMINATION_LIGHT 1
#endif

#if TEMP_MODE == 1
  #define ENABLE_SERIAL_PRINT 1
  #define ENABLE_WIRELESS_DEBUG 0
  #define ENABLE_STEERING_SERVO 0
  #define ENABLE_SINGLE_AXE_STEERING 0
  #define ENABLE_DRIVERMOTOR 0
  #define ENABLE_SETTINGS_MENU 0
  #define ENABLE_EMERGENCY_BREAKING 1
  #define ENABLE_DISTANCE_SENSOR1 0
  #define ENABLE_DISTANCE_SENSOR2 1
  #define ENABLE_DISTANCE_SENSOR3 0
  #define ENABLE_DETATCH_MENU_AFTER_START_CAR_ENGINE 0
  #define ENABLE_FINISH_LINE_DETECTION 0
  #define CAMERA_ILLUMINATION_LIGHT 1
#endif

#if SINGLE_AXE_MODE == 1
  #define ENABLE_SERIAL_PRINT 1
  #define ENABLE_SERIAL_PRINT_LIMITED 0
  #define ENABLE_WIRELESS_DEBUG 1
  #define ENABLE_WIRELESS_DEBUG_LIMITED 0
  #define ENABLE_STEERING_SERVO 1
  #define ENABLE_SINGLE_AXE_STEERING 0
  #define ENABLE_SINGLE_AXE_STEERING_NO_RPM 1
  #define ENABLE_DRIVERMOTOR 1
  #define ENABLE_SETTINGS_MENU 1
  #define ENABLE_EMERGENCY_BREAKING 1
  #define ENABLE_DISTANCE_SENSOR1 0 
  #define ENABLE_DISTANCE_SENSOR2 1
  #define ENABLE_DISTANCE_SENSOR3 0
  #define ENABLE_DETATCH_MENU_AFTER_START_CAR_ENGINE 0
  #define ENABLE_FINISH_LINE_DETECTION 1
  #define CAMERA_ILLUMINATION_LIGHT 0
#endif

#if REAR_AXE_STEERING_MODE == 1
  #define ENABLE_SERIAL_PRINT 1
  #define ENABLE_WIRELESS_DEBUG 1
  #define ENABLE_STEERING_SERVO 1
  #define ENABLE_SINGLE_AXE_STEERING 0
  #define ENABLE_REAR_AXE_STEERING 1
  #define ENABLE_DRIVERMOTOR 1
  #define ENABLE_SETTINGS_MENU 1
  #define ENABLE_EMERGENCY_BREAKING 1
  #define ENABLE_DISTANCE_SENSOR1 0
  #define ENABLE_DISTANCE_SENSOR2 1
  #define ENABLE_DISTANCE_SENSOR3 0
  #define ENABLE_DETATCH_MENU_AFTER_START_CAR_ENGINE 0
  #define ENABLE_FINISH_LINE_DETECTION 1
  #define CAMERA_ILLUMINATION_LIGHT 0
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

#define STEERING_SERVO_PIN  7
#define RIGHT_WHEEL_MOTOR_PIN 5
#define LEFT_WHEEL_MOTOR_PIN 4
#define EDF_MOTOR_PIN 6
#define REMOTE_START_PIN -1
#define REMOTE_STOP_PIN -1

#define DISTANCE_SENSOR1_ANALOG_PIN 14
#define DISTANCE_SENSOR2_ANALOG_PIN 15
#define DISTANCE_SENSOR3_ANALOG_PIN 16

#define MENU_RIGHT_ARROW_BUTTON_PIN 22
#define MENU_LEFT_ARROW_BUTTON_PIN 17
#define MENU_DECREMENT_BUTTON_PIN 21
#define MENU_INCREMENT_BUTTON_PIN 20

#define EMERGENCY_BREAK_LIGHT_PIN 8

#define RPM_SENSOR_LEFT_WHEEL_PIN 2
#define RPM_SENSOR_RIGHT_WHEEL_PIN 3

#if CAR_ID==2
  #define STEERING_SERVO_PIN                22
  #define RIGHT_WHEEL_MOTOR_PIN             -1
  #define LEFT_WHEEL_MOTOR_PIN              23
  #define EDF_MOTOR_PIN                     7
  #define REMOTE_START_PIN                  -1
  #define REMOTE_STOP_PIN                   -1

  #define DISTANCE_SENSOR1_ANALOG_PIN       -1
  #define DISTANCE_SENSOR2_ANALOG_PIN       20
  #define DISTANCE_SENSOR3_ANALOG_PIN       -1

  #define MENU_RIGHT_ARROW_BUTTON_PIN       14
  #define MENU_LEFT_ARROW_BUTTON_PIN        15
  #define MENU_DECREMENT_BUTTON_PIN         16
  #define MENU_INCREMENT_BUTTON_PIN         17

  #define EMERGENCY_BREAK_LIGHT_PIN         21

  #define RPM_SENSOR_LEFT_WHEEL_PIN         -1
  #define RPM_SENSOR_RIGHT_WHEEL_PIN        -1
#endif


#include "log.h"
#include "LcdMenu.h"
#include "DistanceSensors.h"
#include "EnableAfterDelay.h"



/*====================================================================================================================================*/

#endif
