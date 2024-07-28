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
enable_car_engine = 0.0;
enable_car_steering_wheel = 1.0;
enable_emergency_brake = 1.0;
enable_pixy_vector_approximation = 0.0;             
enable_distance_sensor1 = 1.0;
enable_distance_sensor2 = 1.0;
enable_distance_sensor3 = 1.0;

lane_width_vector_unit_real = 53.0;
black_color_treshold = 0.2;
car_length_cm = 17.5;
lookahead_min_distance_cm = 16.0;                       % 22
lookahead_max_distance_cm = 40.0;                       % 40
min_speed = 97.0;
max_speed = 115.0;                                      % 
emergency_break_distance_cm = 75.0;                     % 75
emergency_brake_min_speed = 94.0;
emergency_brake_distance_from_obstacle_cm = 14.0;       % 14
emergency_brake_enable_delay = 0.0;
steering_wheel_angle_offset = 0.0;

*/

#ifndef __CONFIG_H__
#define __CONFIG_H__


#define CAR1 1
#define CAR2 0

#if CAR1 == 0 && CAR2 == 0
  #define CAR1 1
#endif

#define CAR2_PARAMETERS_DIFFERENCE (0.0f)
#if CAR2 == 1
  #define CAR2_PARAMETERS_DIFFERENCE (-0.0f)
#endif


#define DEBUG_MODE 1
#define RACE_MODE 0
#define TEMP_MODE 0

/*====================================================================================================================================*/
#if CAR1 == 1
static int enable_car_engine = 0;
static int enable_car_steering_wheel = 0;
static int enable_emergency_brake = 1;
static int enable_pixy_vector_approximation_soft = 0;
static int enable_distance_sensor1_soft = 1;
static int enable_distance_sensor2_soft = 1;
static int enable_distance_sensor3_soft = 1;
static int enable_remote_start_stop_soft = 0;
static int enable_finish_line_detection_soft = 1;

static float lane_width_vector_unit_real = 53.0f;
static float black_color_treshold = 0.2f; // 0=black, 1=white
static float car_length_cm = 17.5f;
static float lookahead_min_distance_cm = 22.0f;
static float lookahead_max_distance_cm = 50.0f;
static float min_speed = 0.2f + CAR2_PARAMETERS_DIFFERENCE;   // m/s
static float max_speed = 2.0f  + CAR2_PARAMETERS_DIFFERENCE;  // m/s
static float emergency_break_distance_cm = 75.0f;
static float emergency_brake_min_speed = 0.2f + CAR2_PARAMETERS_DIFFERENCE; // m/s
static float emergency_brake_distance_from_obstacle_cm = 9.0f;   // 13.5f
static float steering_wheel_angle_offset = 0.0f;
static float min_axis_angle_vector = 15.0f;
static float max_speed_after_emergency_brake_delay = 2.0f; // m/s
static float car_speed_ki = -0.02f;
static float car_speed_kd = -0.2f;
static float car_speed_ki_min_max_impact = 0.2f;
static float finish_line_angle_tolerance = 15.0f;


#if RACE_MODE == 1
  static float emergency_brake_enable_delay_s = 15.0f;
#elif DEBUG_MODE == 1
  static float emergency_brake_enable_delay_s = 0.0f;
#else
  static float emergency_brake_enable_delay_s = 15.0f;
#endif
#elif CAR2 == 1
static int enable_car_engine = 0;
static int enable_car_steering_wheel = 0;
static int enable_emergency_brake = 1;
static int enable_pixy_vector_approximation_soft = 0;
static int enable_distance_sensor1_soft = 1;
static int enable_distance_sensor2_soft = 1;
static int enable_distance_sensor3_soft = 1;
static int enable_remote_start_stop_soft = 1;

static float lane_width_vector_unit_real = 53.0f;
static float black_color_treshold = 0.2f; // 0=black, 1=white
static float car_length_cm = 17.5f;
static float lookahead_min_distance_cm = 22.0f;
static float lookahead_max_distance_cm = 50.0f;
static float min_speed = 97.0f + CAR2_PARAMETERS_DIFFERENCE;
static float max_speed = 125.0f  + CAR2_PARAMETERS_DIFFERENCE;
static float emergency_break_distance_cm = 75.0f;
static float emergency_brake_min_speed = 94.0f + CAR2_PARAMETERS_DIFFERENCE;
static float emergency_brake_distance_from_obstacle_cm = 74.0f;   // 13.5f
static float steering_wheel_angle_offset = 0.0f;
static float min_axis_angle_vector = 15.0f;
static float max_speed_after_emergency_brake_delay = 107.0f;
static float car_speed_ki = -0.01f;
static float car_speed_kd = -0.4f;
static float car_speed_ki_min_max_impact = 5.0f;


#if RACE_MODE == 1
  static float emergency_brake_enable_delay_s = 20.0f;
#elif DEBUG_MODE == 1
  static float emergency_brake_enable_delay_s = 0.0f;
#else
  static float emergency_brake_enable_delay_s = 15.0f;
#endif

#endif


/*====================================================================================================================================*/


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


#define SERIAL_PORT Serial1

#if DEBUG_MODE == 1
  #define ENABLE_SERIAL_PRINT 1
  #define ENABLE_WIRELESS_DEBUG 1
  #define ENABLE_STEERING_SERVO 1
  #define ENABLE_DRIVERMOTOR 1
  #define ENABLE_SETTINGS_MENU 1
  #define ENABLE_EMERGENCY_BREAKING 0
  #define ENABLE_PIXY_VECTOR_APPROXIMATION 0
  #define ENABLE_DISTANCE_SENSOR1 0
  #define ENABLE_DISTANCE_SENSOR2 0
  #define ENABLE_DISTANCE_SENSOR3 0
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

#if ENABLE_SETTINGS_MENU == 1
  #include <LiquidCrystal_I2C.h>
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

#define IMAGE_MAX_X 78.0f
#define IMAGE_MAX_Y 51.0f
#define SCREEN_CENTER_X ((float)IMAGE_MAX_X / 2.0f)
#define SCREEN_CENTER_Y ((float)IMAGE_MAX_Y / 2.0f)

#define LANE_WIDTH_CM 53.5f
#define LANE_WIDTH_VECTOR_UNIT_REAL lane_width_vector_unit_real

#define LOOKAHEAD_MIN_DISTANCE_CM lookahead_min_distance_cm
#define LOOKAHEAD_MAX_DISTANCE_CM lookahead_max_distance_cm
#define CAR_LENGTH_CM car_length_cm
#define BLACK_COLOR_TRESHOLD black_color_treshold // 0=black, 1=white
#define EMERGENCY_BREAK_DISTANCE_CM emergency_break_distance_cm
#define EMERGENCY_BREAK_MAX_DISTANCE_FROM_OBSTACLE_CM emergency_brake_distance_from_obstacle_cm
#define EMERGENCY_BRAKE_MIN_SPEED emergency_brake_min_speed
#define EMERGENCY_BRAKE_ENABLE_DELAY_S emergency_brake_enable_delay_s
#define STEERING_WHEEL_ANGLE_OFFSET steering_wheel_angle_offset
#define MIN_XAXIS_ANGLE_VECTOR min_axis_angle_vector
#define MAX_SPEED_AFTER_EMERGENCY_BRAKE_DELAY max_speed_after_emergency_brake_delay
#define CAR_SPEED_KI car_speed_ki
#define CAR_SPEED_KD car_speed_kd
#define CAR_SPEED_KI_MIN_MAX_IMPACT car_speed_ki_min_max_impact
#define FINISH_LINE_ANGLE_TOLERANCE finish_line_angle_tolerance


#define VECTOR_UNIT_PER_CM (float)((float)LANE_WIDTH_VECTOR_UNIT_REAL / (float)LANE_WIDTH_CM)   // CM * VECTOR_UNIT_PER_CM = VECTOR_UNIT
#define CM_PER_VECTOR_UNIT (float)((float)LANE_WIDTH_CM / (float)LANE_WIDTH_VECTOR_UNIT_REAL)   // VECTOR_UNIT_PER_CM * CM = CM
#define RADIANS_PER_DEGREE (float)((float)M_PI / 180.0f) // DEGREE * RADIAN_PER_DEGREE = RADIAN
#define DEGREES_PER_RADIAN (float)(180.0f / (float)M_PI) // RADIAN * DEGREE_PER_RADIAN = DEGREE

//#define LANE_WIDTH_VECTOR_UNIT (float)(LANE_WIDTH_VECTOR_UNIT_REAL + ((5.0f * VECTOR_UNIT_PER_CM)*2.0f))
#define LANE_WIDTH_VECTOR_UNIT (float)(LANE_WIDTH_VECTOR_UNIT_REAL)



#if CAR1 == 1
  #define STEERING_SERVO_ANGLE_MIDDLE     90    // 90 middle // 120
  #define STEERING_SERVO_ANGLE_MAX_RIGHT  35    // 0 max right // 90 - 58 = 32
  #define STEERING_SERVO_ANGLE_MAX_LEFT   145   // 180 max left // 90 + 58
#elif CAR2 == 1
  #define STEERING_SERVO_ANGLE_MIDDLE     (90)    // 90 middle // 120
  #define STEERING_SERVO_ANGLE_MAX_RIGHT  (145)    // 0 max right // 90 - 58 = 32
  #define STEERING_SERVO_ANGLE_MAX_LEFT   (35)   // 180 max left // 90 + 58
#endif

#define STEERING_SERVO_MAX_ANGLE MAX(abs(STEERING_SERVO_ANGLE_MIDDLE - STEERING_SERVO_ANGLE_MAX_RIGHT), abs(STEERING_SERVO_ANGLE_MIDDLE - STEERING_SERVO_ANGLE_MAX_LEFT))

#define MIN_SPEED min_speed
#define MAX_SPEED max_speed
#if CAR1 == 1
  #define STANDSTILL_SPEED 0.0f
#elif CAR2 == 1
  #define STANDSTILL_SPEED 0.0f
#endif

#define ENABLE_CAR_ENGINE enable_car_engine
#define ENABLE_CAR_STEERING_WHEEL enable_car_steering_wheel
#define ENABLE_EMERGENCY_BRAKE enable_emergency_brake
#define ENABLE_DISTANCE_SENSOR1_SOFT enable_distance_sensor1_soft
#define ENABLE_DISTANCE_SENSOR2_SOFT enable_distance_sensor2_soft
#define ENABLE_DISTANCE_SENSOR3_SOFT enable_distance_sensor3_soft
#define ENABLE_REMOTE_START_STOP_SOFT enable_remote_start_stop_soft
#define ENABLE_FINISH_LINE_DETECTION_SOFT enable_finish_line_detection_soft
#define ENABLE_PIXY_VECTOR_APPROXIMATION_SOFT enable_pixy_vector_approximation_soft

/*====================================================================================================================================*/
static float car_length_vector_unit = (float)car_length_cm * (float)VECTOR_UNIT_PER_CM;
static int emergency_break_active =(int) 0;
static unsigned int emergency_break_loops_count = (int)0;
static float carSpeed = (float)STANDSTILL_SPEED;
static LineABC middle_lane_line_pixy_1;
static LineABC left_lane_line_pixy_1;
static LineABC right_lane_line_pixy_1;
static float loop_time_ms = 0.0f;
static float time_passed_ms = 0.0f;
static float emergency_brake_enable_remaining_delay_s = 0.0f;
static int emergency_brake_enable_delay_started_count = 0;
static int finish_line_detected = 0;
static int finish_line_detected_now = 0;
FinishLine finish_line = {};

#include "log.h"
#include "GlobalVariables.h"
#include "LcdMenu.h"

/*====================================================================================================================================*/


static void HardwareReset(){
  delay(100);
  SCB_AIRCR = 0x05FA0004;
}

#endif
