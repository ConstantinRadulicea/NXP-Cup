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
#include "TrajectoryMix.h"
#include "SteeringWheel.h"


#define TRUE 1
#define FALSE 0


#if CAR1 == 0 && CAR2 == 0
  #define CAR1 1
#endif


/*====================================================================================================================================*/

extern int8_t g_enable_car_engine;
extern int8_t g_enable_car_steering_wheel;
extern int8_t g_enable_emergency_brake;
extern int8_t g_enable_pixy_vector_approximation;
extern int8_t g_enable_distance_sensor1;
extern int8_t g_enable_distance_sensor2;
extern int8_t g_enable_distance_sensor3;
extern int8_t g_enable_remote_start_stop;
extern int8_t g_enable_finish_line_detection;

extern float g_lane_width_vector_unit;
extern float g_black_color_treshold; // 0=black, 1=white
extern float g_lookahead_min_distance_cm;
extern float g_lookahead_max_distance_cm;
extern float g_vehicle_min_speed_mps;   // m/s
extern float g_vehicle_max_speed_mps;  // m/s
extern float g_emergency_brake_activation_max_distance_m;
extern float g_emergency_brake_speed_mps; // m/s
extern float g_emergency_brake_distance_from_obstacle_m;   // 13.5f
extern float g_steering_wheel_angle_offset_deg;
extern float g_min_x_axis_angle_vector_deg;
extern float g_max_speed_after_emergency_brake_delay_mps; // m/s
extern float g_car_speed_mps_ki;
extern float g_car_speed_mps_kd;
extern float g_car_speed_mps_ki_min_max_impact;
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
  #define STEERING_SERVO_ANGLE_MIDDLE     90
  #define STEERING_SERVO_ANGLE_MAX_RIGHT  126   // +36 -> -36 going right
  #define STEERING_SERVO_ANGLE_MAX_LEFT   43     // -47 -> +47 going left
#elif CAR2 == 1
  #define STEERING_SERVO_ANGLE_MIDDLE     (90)    // 90 middle // 120
  #define STEERING_SERVO_ANGLE_MAX_RIGHT  (145)    // 0 max right // 90 - 58 = 32
  #define STEERING_SERVO_ANGLE_MAX_LEFT   (35)   // 180 max left // 90 + 58
#endif


#define STEERING_SERVO_MAX_ANGLE MAX(abs(STEERING_SERVO_ANGLE_MIDDLE - STEERING_SERVO_ANGLE_MAX_RIGHT), abs(STEERING_SERVO_ANGLE_MIDDLE - STEERING_SERVO_ANGLE_MAX_LEFT))
#define STEERING_SERVO_MIN_ANGLE MIN(abs(STEERING_SERVO_ANGLE_MIDDLE - STEERING_SERVO_ANGLE_MAX_RIGHT), abs(STEERING_SERVO_ANGLE_MIDDLE - STEERING_SERVO_ANGLE_MAX_LEFT))

#define VALID_STEERING_SERVO_ANGLE(angle) (MIN(MAX(angle, MIN(STEERING_SERVO_ANGLE_MAX_RIGHT, STEERING_SERVO_ANGLE_MAX_LEFT)), MAX(STEERING_SERVO_ANGLE_MAX_RIGHT, STEERING_SERVO_ANGLE_MAX_LEFT)))

#define STANDSTILL_SPEED 0.0f

#define WHEEL_BASE_M 0.175
#define WHEEL_DIAMETER_M 0.064	//wheel diameter im meters
#define DISTANCE_BETWEEN_WHEELS_M 0.137	//distance between wheels
#define POWERTRAIN_PID_FREQUENCY_HZ 100

#define DISTANCE_SENSOR1_OFFSET_M (0.0)
#define DISTANCE_SENSOR2_OFFSET_M (0.07)   // calibrated_data = sensor_data - offset
#define DISTANCE_SENSOR3_OFFSET_M (0.0)

#define DISTANCE_SENSOR1_CALIBRATION_FORMULA_X (-0.124)
#define DISTANCE_SENSOR1_CALIBRATION_FORMULA_C (-0.056)

#define DISTANCE_SENSOR2_CALIBRATION_FORMULA_X (-0.124)
#define DISTANCE_SENSOR2_CALIBRATION_FORMULA_C (-0.056)

#define DISTANCE_SENSOR3_CALIBRATION_FORMULA_X (-0.124)
#define DISTANCE_SENSOR3_CALIBRATION_FORMULA_C (-0.056)


/*====================================================================================================================================*/
extern float g_wheel_base_vector_unit;
extern int8_t g_emergency_break_active;
extern unsigned int g_emergency_break_loops_count;
extern float g_car_speed_mps;
extern LineABC g_middle_lane_line_pixy_1;
extern LineABC g_left_lane_line_pixy_1;
extern LineABC g_right_lane_line_pixy_1;
extern float g_loop_time_ms;
extern float g_time_passed_ms;
extern float g_emergency_brake_enable_remaining_delay_s;
extern int8_t g_emergency_brake_enable_delay_started_count;
extern int8_t g_finish_line_detected;
extern int8_t g_finish_line_detected_now;
extern float g_steering_angle_rad;
extern FinishLine g_finish_line;
extern int8_t g_start_line_calibration_acquisition;

extern LineCalibrationData g_line_calibration_data;

extern float g_powertrain_left_wheel_kp;
extern float g_powertrain_left_wheel_ki;
extern float g_powertrain_left_wheel_kd;
extern float g_powertrain_left_wheel_ki_max_sum;
extern float g_powertrain_right_wheel_kp;
extern float g_powertrain_right_wheel_ki;
extern float g_powertrain_right_wheel_kd;
extern float g_powertrain_right_wheel_ki_max_sum;

extern float g_friction_coefficient;
extern float g_downward_acceleration;

extern float g_max_acceleration;
extern float g_max_deceleration;


extern SteeringWheel g_steering_wheel;

extern VectorsProcessing g_pixy_1_vectors_processing;
//VectorsProcessing pixy_2_vectorsProcessing;
extern Pixy2 g_pixy_1;
//Pixy2SPI_SS pixy_2;



void parseInputGlobalVariablesRoutine(SERIAL_PORT_TYPE &serialPort);
void parseAndSetGlobalVariables_2(std::string& rawData, char variableTerminator = ';');
void parseAndSetGlobalVariables(std::vector<char>& rawData, char variableTerminator = ';');
void printGlobalVariables(SERIAL_PORT_TYPE &serialPort);


#endif