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
//#include "parseNextFloat.h"
#include "geometry2D.h"
#include "Config.h"
#include "TrajectoryMix.h"
#include "SteeringWheel.h"
#include "VectorsProcessing.h"
#include "BirdEyeView.h"


#define TRUE 1
#define FALSE 0


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
extern int8_t g_enable_oversteer_mitigation;

extern float g_lane_width_vector_unit;
extern float g_black_color_treshold; // 0=black, 1=white
extern float g_lookahead_min_distance_cm;
extern float g_lookahead_max_distance_cm;
extern float g_vehicle_min_speed_mps;   // m/s
extern float g_vehicle_max_speed_mps;  // m/s
extern float g_vehicle_max_speed_original_mps; // m/s
extern float g_emergency_brake_activation_max_distance_m;
extern float g_emergency_brake_speed_mps; // m/s
extern float g_emergency_brake_distance_from_obstacle_m;   // 13.5f
extern float g_steering_wheel_angle_offset_deg;
extern float g_min_x_axis_angle_vector_deg;
extern float g_max_speed_after_delay_mps; // m/s
extern float g_car_speed_mps_ki;
extern float g_car_speed_mps_kd;
extern float g_car_speed_mps_ki_min_max_impact;
extern float g_finish_line_angle_tolerance;
extern float g_oversteer_mitigation_yaw_tolerance_rad_s;
extern float g_oversteer_mitigation_yaw_delta_tolerance_rad_s;

extern float g_emergency_brake_enable_delay_s;
extern int8_t g_max_speed_delay_passed;

extern int8_t g_enable_change_aeb_max_distance_after_delay_passed;
extern float g_enable_change_aeb_max_distance_after_delay_s;



/*====================================================================================================================================*/

//#define IMAGE_MAX_X 79.0f
//#define IMAGE_MAX_Y 52.0f


extern float g_line_image_frame_width;
extern float g_line_image_frame_height;

#define SCREEN_CENTER_X ((float)g_line_image_frame_width / 2.0f)
#define SCREEN_CENTER_Y ((float)g_line_image_frame_height / 2.0f)

#define LANE_WIDTH_M 0.535f

#define MeterToVectorUnit(m) ((float)(m) * ((float)g_lane_width_vector_unit *( 1.0f / (float)LANE_WIDTH_M)))
#define VectorUnitToMeter(v_unit) ((float)(v_unit) * ((float)LANE_WIDTH_M * (1.0f / (float)g_lane_width_vector_unit)))



#if ENABLE_STEERING_SERVO == 1
  #if CAR_ID == 1
    #define STEERING_SERVO_ANGLE_MIDDLE     90
    #define STEERING_SERVO_ANGLE_MAX_RIGHT  (90+27)   // -> -27 going right 126
    #define STEERING_SERVO_ANGLE_MAX_LEFT   (90-40)     // -47 -> +45 going left 43 //49
  #elif CAR_ID == 2
    #define STEERING_SERVO_ANGLE_MIDDLE     90
    #define STEERING_SERVO_ANGLE_MAX_RIGHT  (90+43)
    #define STEERING_SERVO_ANGLE_MAX_LEFT   (90-36)
  #endif
#endif

#if ENABLE_SINGLE_AXE_STEERING == 1
  #define STEERING_ANGLE_MAX_RIGHT  (-35)
  #define STEERING_ANGLE_MAX_LEFT   (-STEERING_ANGLE_MAX_RIGHT)

  #define STEERING_ANGLE_MAX (((STEERING_ANGLE_MAX_RIGHT)>(STEERING_ANGLE_MAX_LEFT))?(STEERING_ANGLE_MAX_RIGHT):(STEERING_ANGLE_MAX_LEFT))
  #define STEERING_ANGLE_MIN (((STEERING_ANGLE_MAX_RIGHT)>(STEERING_ANGLE_MAX_LEFT))?(STEERING_ANGLE_MAX_LEFT):(STEERING_ANGLE_MAX_RIGHT))

  #define VALIDATE_REAR_STEERING_ANGLE(deg) MIN(STEERING_ANGLE_MAX, MAX(deg, STEERING_ANGLE_MIN))
#endif



#define STEERING_SERVO_MAX_ANGLE MAX(abs(STEERING_SERVO_ANGLE_MIDDLE - STEERING_SERVO_ANGLE_MAX_RIGHT), abs(STEERING_SERVO_ANGLE_MIDDLE - STEERING_SERVO_ANGLE_MAX_LEFT))
#define STEERING_SERVO_MIN_ANGLE MIN(abs(STEERING_SERVO_ANGLE_MIDDLE - STEERING_SERVO_ANGLE_MAX_RIGHT), abs(STEERING_SERVO_ANGLE_MIDDLE - STEERING_SERVO_ANGLE_MAX_LEFT))

#define VALID_STEERING_SERVO_ANGLE(angle) (MIN(MAX(angle, MIN(STEERING_SERVO_ANGLE_MAX_RIGHT, STEERING_SERVO_ANGLE_MAX_LEFT)), MAX(STEERING_SERVO_ANGLE_MAX_RIGHT, STEERING_SERVO_ANGLE_MAX_LEFT)))

#define STANDSTILL_SPEED 0.0f

#define WHEEL_BASE_M 0.175
#define WHEEL_DIAMETER_M 0.064	//wheel diameter im meters
#define TRACK_WIDTH_M 0.137	//distance between wheels
#if CAR_ID == 1
  #define TRACK_WIDTH_REAR_WHEELS_M TRACK_WIDTH_M //0.162	//distance between rear wheels
#else
  #define TRACK_WIDTH_REAR_WHEELS_M TRACK_WIDTH_M //distance between rear wheels
#endif

#define POWERTRAIN_PID_FREQUENCY_HZ 50
#ifdef TEENSYLC
#define POWERTRAIN_PID_FREQUENCY_HZ 50
#endif

#if CAR_ID == 1
  #define DISTANCE_SENSOR1_OFFSET_M (0.0694)
  #define DISTANCE_SENSOR2_OFFSET_M (0.0694)   // calibrated_data = sensor_data - offset
  #define DISTANCE_SENSOR3_OFFSET_M (0.0694)

  #define DISTANCE_SENSOR1_CALIBRATION_FORMULA_X (-0.124)
  #define DISTANCE_SENSOR1_CALIBRATION_FORMULA_C (-0.056)

  #define DISTANCE_SENSOR2_CALIBRATION_FORMULA_X (-0.124)
  #define DISTANCE_SENSOR2_CALIBRATION_FORMULA_C (-0.056)

  #define DISTANCE_SENSOR3_CALIBRATION_FORMULA_X (-0.124)
  #define DISTANCE_SENSOR3_CALIBRATION_FORMULA_C (-0.056)

  #elif CAR_ID == 2


  #define DISTANCE_SENSOR1_OFFSET_M (0.068)
  #define DISTANCE_SENSOR2_OFFSET_M (0.068)   // calibrated_data = sensor_data - offset
  #define DISTANCE_SENSOR3_OFFSET_M (0.068)

  #define DISTANCE_SENSOR1_CALIBRATION_FORMULA_X (-0.696)
  #define DISTANCE_SENSOR1_CALIBRATION_FORMULA_C (-0.201)

  #define DISTANCE_SENSOR2_CALIBRATION_FORMULA_X (-0.696)
  #define DISTANCE_SENSOR2_CALIBRATION_FORMULA_C (-0.201)

  #define DISTANCE_SENSOR3_CALIBRATION_FORMULA_X (-0.696)
  #define DISTANCE_SENSOR3_CALIBRATION_FORMULA_C (-0.201)

#endif

/*====================================================================================================================================*/
//extern float g_wheel_base_vector_unit;
extern int8_t g_emergency_break_active;
extern unsigned int g_emergency_break_loops_count;
extern float g_car_speed_mps;
extern LineABC g_middle_lane_line_pixy_1;
extern LineABC g_left_lane_line_pixy_1;
extern LineABC g_right_lane_line_pixy_1;
extern LineSegment g_left_lane_segment;
extern LineSegment g_right_lane_segment;
extern float g_loop_time_ms;
extern float g_time_passed_ms;
extern float g_emergency_brake_enable_remaining_delay_s;
extern int8_t g_emergency_brake_enable_delay_started_count;
extern int8_t g_finish_line_detected;
extern int8_t g_finish_line_detected_now;
extern int8_t g_finish_line_detected_slowdown;
extern float g_max_speed_after_finish_line_detected_mps; // m/s
extern float g_steering_angle_rad;
extern FinishLine g_finish_line;
extern int8_t g_start_line_calibration_acquisition;
extern int8_t g_start_line_calibration_acquisition_birdeye;
extern volatile int8_t g_oversteer_mitigation_active;

extern LineCalibrationData g_line_calibration_data;
extern float g_rear_axe_turn_radius_m;


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

extern float g_camera_offset_y_m; //real_value = measured_value - offset    // offset = measured_value - real_value


extern float g_enable_finish_line_detection_after_delay_s;
extern float g_max_speed_after_delay_s;


#if ENABLE_STEERING_SERVO == 1
  extern SteeringWheel g_steering_wheel;
#endif

extern VectorsProcessing g_pixy_1_vectors_processing;
//VectorsProcessing pixy_2_vectorsProcessing;
extern Pixy2 g_pixy_1;
//Pixy2SPI_SS pixy_2;

extern struct BirdEyeCalibrationData g_birdeye_calibrationdata;


void initialize_g_birdeye_calibrationdata();
void parseInputGlobalVariablesRoutine(SERIAL_PORT_TYPE &serialPort);
void parseAndSetGlobalVariables_2(std::string& rawData, char variableTerminator = ';');
void parseAndSetGlobalVariables(std::vector<char>& rawData, char variableTerminator = ';');
void printGlobalVariables(SERIAL_PORT_TYPE &serialPort);

void parseInputGlobalVariablesRoutine_optimized(SERIAL_PORT_TYPE &serialPort);

#endif