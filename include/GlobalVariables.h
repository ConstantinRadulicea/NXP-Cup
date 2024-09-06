/*
* Copyright 2024 Constantin Dumitru Petre RĂDULICEA
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

#ifndef __GLOBALVARIABLES_H__
#define __GLOBALVARIABLES_H__

#include <HardwareSerial.h>
#include <vector>
#include <string>
#include <sstream>
#include "parseNextFloat.h"
#include "geometry2D.h"
#include "Config.h"
#include "VectorsProcessing.h"


#if CAR1 == 0 && CAR2 == 0
  #define CAR1 1
#endif


/*====================================================================================================================================*/

extern int g_enable_car_engine;
extern int g_enable_car_steering_wheel;
extern int g_enable_emergency_brake;
extern int g_enable_pixy_vector_approximation;
extern int g_enable_distance_sensor1;
extern int g_enable_distance_sensor2;
extern int g_enable_distance_sensor3;
extern int g_enable_remote_start_stop;
extern int g_enable_finish_line_detection;

extern float g_lane_width_vector_unit;
extern float g_black_color_treshold; // 0=black, 1=white
extern float g_lookahead_min_distance_cm;
extern float g_lookahead_max_distance_cm;
extern float g_min_speed;   // m/s
extern float g_max_speed;  // m/s
extern float g_emergency_brake_distance_m;
extern float g_emergency_brake_min_speed; // m/s
extern float g_emergency_brake_distance_from_obstacle_m;   // 13.5f
extern float g_steering_wheel_angle_offset;
extern float g_min_x_axis_angle_vector;
extern float g_max_speed_after_emergency_brake_delay; // m/s
extern float g_car_speed_ki;
extern float g_car_speed_kd;
extern float g_car_speed_ki_min_max_impact;
extern float g_finish_line_angle_tolerance;

extern float g_emergency_brake_enable_delay_s;



/*====================================================================================================================================*/

#define IMAGE_MAX_X 78.0f
#define IMAGE_MAX_Y 51.0f
#define SCREEN_CENTER_X ((float)IMAGE_MAX_X / 2.0f)
#define SCREEN_CENTER_Y ((float)IMAGE_MAX_Y / 2.0f)

#define LANE_WIDTH_M 0.535f

#define MeterToVectorUnit(m) ((float)(m) * ((float)g_lane_width_vector_unit / (float)LANE_WIDTH_M))
#define VectorUnitToMeter(v_unit) ((float)(v_unit) * ((float)LANE_WIDTH_M / (float)g_lane_width_vector_unit))

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

#define STANDSTILL_SPEED 0.0f

#define CAR_LENGTH_M 0.175
#define WHEEL_DIAMETER_M 0.064	//wheel diameter im meters
#define DISTANCE_BETWEEN_WHEELS_M 0.145	//distance between wheels
#define POWERTRAIN_PID_FREQUENCY_HZ 100


/*====================================================================================================================================*/
extern float g_car_length_vector_unit;
extern int g_emergency_break_active;
extern unsigned int g_emergency_break_loops_count;
extern float g_car_speed;
extern LineABC g_middle_lane_line_pixy_1;
extern LineABC g_left_lane_line_pixy_1;
extern LineABC g_right_lane_line_pixy_1;
extern float g_loop_time_ms;
extern float g_time_passed_ms;
extern float g_emergency_brake_enable_remaining_delay_s;
extern int g_emergency_brake_enable_delay_started_count;
extern int g_finish_line_detected;
extern int g_finish_line_detected_now;
extern FinishLine g_finish_line;

extern float g_powertrain_left_wheel_kp;
extern float g_powertrain_left_wheel_ki;
extern float g_powertrain_left_wheel_kd;
extern float g_powertrain_left_wheel_ki_max_sum;
extern float g_powertrain_right_wheel_kp;
extern float g_powertrain_right_wheel_ki;
extern float g_powertrain_right_wheel_kd;
extern float g_powertrain_right_wheel_ki_max_sum;


extern SteeringWheel g_steering_wheel;

extern VectorsProcessing g_pixy_1_vectors_processing;
//VectorsProcessing pixy_2_vectorsProcessing;
extern Pixy2 g_pixy_1;
//Pixy2SPI_SS pixy_2;



void parseInputGlobalVariablesRoutine(SERIAL_TYPE &serialPort);
void parseAndSetGlobalVariables_2(std::string& rawData, char variableTerminator = ';');
void parseAndSetGlobalVariables(std::vector<char>& rawData, char variableTerminator = ';');
void printGlobalVariables(SERIAL_TYPE &serialPort);


#endif