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

#include "GlobalVariables.h"
#if ENABLE_SINGLE_AXE_STEERING_NO_RPM == 0
  #include "PowerTrain.h"
#else
  #include "OneMotorPowerTrain.h"
#endif


#if CAR_ID == 1
int8_t g_enable_car_engine = 0;
int8_t g_enable_car_steering_wheel = 0;
int8_t g_enable_emergency_brake = 0;
int8_t g_enable_pixy_vector_approximation = 0;
int8_t g_enable_distance_sensor1 = 1;
int8_t g_enable_distance_sensor2 = 1;
int8_t g_enable_distance_sensor3 = 1;
int8_t g_enable_remote_start_stop = 0;
int8_t g_enable_finish_line_detection = 0;
int8_t g_enable_oversteer_mitigation = 1;

float g_lane_width_vector_unit = 63.04f;
float g_black_color_treshold = 0.2f; // 0=black, 1=white
float g_lookahead_min_distance_cm = 20.0f;
float g_lookahead_max_distance_cm = 50.0f;
float g_vehicle_min_speed_mps = 0.7f;   // m/s
float g_vehicle_max_speed_mps = 4.0f;  // m/s
float g_vehicle_max_speed_original_mps = 4.0f; // m/s
float g_emergency_brake_activation_max_distance_m = 1.5f;
float g_emergency_brake_speed_mps = 0.3f; // m/s
float g_emergency_brake_distance_from_obstacle_m = 0.1f;   // 13.5f
float g_steering_wheel_angle_offset_deg = 6.5f;      //-4.6f;
float g_min_x_axis_angle_vector_deg = 10.0f;
float g_max_speed_after_delay_mps = 1.5f; // m/s
float g_max_speed_after_finish_line_detected_mps = 0.7f; // m/s
float g_car_speed_mps_ki = -0.02f;
float g_car_speed_mps_kd = -0.2f;
float g_car_speed_mps_ki_min_max_impact = 0.3f;
float g_finish_line_angle_tolerance = 30.0f;
float g_oversteer_mitigation_yaw_tolerance_rad_s = radians(150.0f);
float g_oversteer_mitigation_yaw_delta_tolerance_rad_s = radians(360.0f);
float g_edf_raw_speed = 180.0f;

float g_powertrain_left_wheel_kp = 0.0;
float g_powertrain_left_wheel_ki = 0.1;
float g_powertrain_left_wheel_kd = 0.0;
float g_powertrain_left_wheel_ki_max_sum = 0.0;   //0.4
float g_powertrain_right_wheel_kp = 0.0;
float g_powertrain_right_wheel_ki = 0.1;
float g_powertrain_right_wheel_kd = 0.0;
float g_powertrain_right_wheel_ki_max_sum = 0.0;  // 0.4

float g_friction_coefficient = 0.8f;
float g_downward_acceleration = G_CONSTANT;

float g_max_acceleration = (g_friction_coefficient * g_downward_acceleration)/2.5f;
float g_max_deceleration = -1.0f;

float g_camera_offset_y_m = 0.0f;

float g_enable_change_aeb_max_distance_after_delay_s = -1.0f;


#if RACE_MODE == 1
  float g_emergency_brake_enable_delay_s = 0.0f;
  float g_max_speed_after_delay_s = 0.0f;
  float g_enable_finish_line_detection_after_delay_s = 5.0f;
#elif DEBUG_MODE == 1
  float g_emergency_brake_enable_delay_s = 0.0f;
  float g_max_speed_after_delay_s = 0.0f;
  float g_enable_finish_line_detection_after_delay_s = 5.0f;
#else
  float g_emergency_brake_enable_delay_s = 0.0f;
  float g_max_speed_after_delay_s = 0.0f;
  float g_enable_finish_line_detection_after_delay_s = 5.0f;
#endif

  void initialize_g_birdeye_calibrationdata() {
    struct track_widths temp_track_widths = {};

    temp_track_widths.upper_segment.A.x = 16.79;  //% FL x
    temp_track_widths.upper_segment.A.y = 50.82;  //FL y
    temp_track_widths.upper_segment.B.x = 54.63;  //FR x
    temp_track_widths.upper_segment.B.y = 53.17;  //FR y
    temp_track_widths.lower_segment.A.x = 7.47;   // RL x
    temp_track_widths.lower_segment.A.y = -1.95;  //RL y
    temp_track_widths.lower_segment.B.x = 70.39;  //RR x
    temp_track_widths.lower_segment.B.y = 1.95;   //RR y


    g_birdeye_calibrationdata = CalculateBirdEyeCalibration_TrackWidths(temp_track_widths, g_line_image_frame_width, g_line_image_frame_height, LANE_WIDTH_M);
    if (g_birdeye_calibrationdata.valid != 0) {
      g_lane_width_vector_unit = g_birdeye_calibrationdata.src_track_width;
    }
    //g_birdeye_calibrationdata.valid = 0;
}

#elif CAR_ID == 2
int8_t g_enable_car_engine = 0;
int8_t g_enable_car_steering_wheel = 0;
int8_t g_enable_emergency_brake = 0;
int8_t g_enable_pixy_vector_approximation = 0;
int8_t g_enable_distance_sensor1 = 1;
int8_t g_enable_distance_sensor2 = 1;
int8_t g_enable_distance_sensor3 = 1;
int8_t g_enable_remote_start_stop = 0;
int8_t g_enable_finish_line_detection = 0;
int8_t g_enable_oversteer_mitigation = 0;

float g_lane_width_vector_unit = 56.61f;
float g_black_color_treshold = 0.2f; // 0=black, 1=white
float g_lookahead_min_distance_cm = 20.0f;
float g_lookahead_max_distance_cm = 50.0f;
float g_vehicle_min_speed_mps = 0.35f;   // m/s
float g_vehicle_max_speed_mps = 2.0f;  // m/s
float g_vehicle_max_speed_original_mps = 2.0f; // m/s
float g_emergency_brake_activation_max_distance_m = 1.5f;
float g_emergency_brake_speed_mps = 0.35f; // m/s
float g_emergency_brake_distance_from_obstacle_m = 0.1f;   // 13.5f
float g_steering_wheel_angle_offset_deg = -7.5f;      //-4.6f;
float g_min_x_axis_angle_vector_deg = 10.0f;
float g_max_speed_after_delay_mps = 1.0f; // m/s
float g_max_speed_after_finish_line_detected_mps = 0.7f; // m/s
float g_car_speed_mps_ki = -0.02f;
float g_car_speed_mps_kd = -0.2f;
float g_car_speed_mps_ki_min_max_impact = 0.3f;
float g_finish_line_angle_tolerance = 35.0f;
float g_oversteer_mitigation_yaw_tolerance_rad_s = radians(50.0f);
float g_oversteer_mitigation_yaw_delta_tolerance_rad_s = radians(360.0f);
float g_edf_raw_speed = 90.0f;

float g_powertrain_left_wheel_kp = 0.0;
float g_powertrain_left_wheel_ki = 0.1;
float g_powertrain_left_wheel_kd = 0.0;
float g_powertrain_left_wheel_ki_max_sum = 0.4;
float g_powertrain_right_wheel_kp = 0.0;
float g_powertrain_right_wheel_ki = 0.1;
float g_powertrain_right_wheel_kd = 0.0;
float g_powertrain_right_wheel_ki_max_sum = 0.4;

float g_friction_coefficient = 0.5f;
float g_downward_acceleration = G_CONSTANT;

float g_max_acceleration = (g_friction_coefficient * g_downward_acceleration);
float g_max_deceleration = -100.f;

float g_camera_offset_y_m = 0.0f;

float g_enable_change_aeb_max_distance_after_delay_s = -1.0f;


#if RACE_MODE == 1
  float g_emergency_brake_enable_delay_s = 0.0f;
  float g_max_speed_after_delay_s = 0.0f;
  float g_enable_finish_line_detection_after_delay_s = 5.0f;
#elif DEBUG_MODE == 1
  float g_emergency_brake_enable_delay_s = 0.0f;
  float g_max_speed_after_delay_s = 0.0f;
  float g_enable_finish_line_detection_after_delay_s = 0.0f;
#else
  float g_emergency_brake_enable_delay_s = 0.0f;
  float g_max_speed_after_delay_s = 0.0f;
  float g_enable_finish_line_detection_after_delay_s = 5.0f;
#endif

void initialize_g_birdeye_calibrationdata() {
    struct track_widths temp_track_widths = {};
    temp_track_widths.upper_segment.A.x = 31.59f;   // FL x
    temp_track_widths.upper_segment.A.y = 52.15f;   // FL y
    temp_track_widths.upper_segment.B.x = 51.31f;   // FR x
    temp_track_widths.upper_segment.B.y = 51.84f;   // FR y
    temp_track_widths.lower_segment.A.x = 11.28f;   // RL x
    temp_track_widths.lower_segment.A.y = 0.44f;    // RL y
    temp_track_widths.lower_segment.B.x = 69.50f;   // RR x
    temp_track_widths.lower_segment.B.y = -0.44f;      // RR y
    g_birdeye_calibrationdata = CalculateBirdEyeCalibration_TrackWidths(temp_track_widths, g_line_image_frame_width, g_line_image_frame_height, LANE_WIDTH_M);
    if (g_birdeye_calibrationdata.valid != 0) {
      g_lane_width_vector_unit = g_birdeye_calibrationdata.src_track_width;
    }
    //g_birdeye_calibrationdata.valid = 0;
    
}


#endif




float g_line_image_frame_width = 79.0f;
float g_line_image_frame_height = 52.0f;
//float g_wheel_base_vector_unit = (float)MeterToVectorUnit(WHEEL_BASE_M);
int8_t g_emergency_break_active = 0;
unsigned int g_emergency_break_loops_count = 0;
float g_car_speed_mps = (float)STANDSTILL_SPEED;
LineABC g_middle_lane_line_pixy_1;
LineABC g_left_lane_line_pixy_1;
LineABC g_right_lane_line_pixy_1;
LineSegment g_left_lane_segment;
LineSegment g_right_lane_segment;
float g_loop_time_ms = 0.0f;
float g_time_passed_ms = 0.0f;
float g_emergency_brake_enable_remaining_delay_s = 0.0f;
int8_t g_emergency_brake_enable_delay_started_count = 0;
int8_t g_finish_line_detected = 0;
int8_t g_finish_line_detected_now = 0;
int8_t g_finish_line_detected_slowdown = 0;
float g_steering_angle_rad = 0.0f;
FinishLine g_finish_line = {};
int8_t g_start_line_calibration_acquisition = 0;
int8_t g_start_line_calibration_acquisition_birdeye = 0;
LineCalibrationData g_line_calibration_data = {};
float g_rear_axe_turn_radius_m = 0.0f;
int8_t g_max_speed_delay_passed = 0;
int8_t g_enable_change_aeb_max_distance_after_delay_passed = 0;
volatile int8_t g_oversteer_mitigation_active = 0;
int8_t g_valid_vectors_detected_flag = 0;
int8_t g_valid_track_lines_flag = 0;

struct BirdEyeCalibrationData g_birdeye_calibrationdata = {};


//SteeringWheel g_steering_wheel(STEERING_SERVO_ANGLE_MAX_LEFT, STEERING_SERVO_ANGLE_MIDDLE, STEERING_SERVO_ANGLE_MAX_RIGHT, (unsigned int)0);
#if ENABLE_STEERING_SERVO == 1
    SteeringWheel g_steering_wheel(WHEEL_BASE_M, TRACK_WIDTH_M, STEERING_SERVO_ANGLE_MAX_LEFT, STEERING_SERVO_ANGLE_MIDDLE, STEERING_SERVO_ANGLE_MAX_RIGHT);
#endif

VectorsProcessing g_pixy_1_vectors_processing;
//VectorsProcessing pixy_2_vectorsProcessing;
Pixy2 g_pixy_1;
//Pixy2SPI_SS pixy_2;


#define TOTAL_GLOBAL_PARAMETERS 61


/*
void parseAndSetGlobalVariables(std::string& rawData, char variableTerminator = ';') {
	char* pEnd;
	int resultSuccess;
  float temp_float;

	pEnd = rawData.data();
	g_lane_width_vector_unit = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  g_lookahead_min_distance_cm = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
	g_lookahead_max_distance_cm = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
	g_emergency_brake_activation_max_distance_m = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  g_vehicle_min_speed_mps = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
	g_vehicle_max_speed_original_mps = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
	g_black_color_treshold = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
	//g_car_length_cm = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  
  temp_float = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  if (temp_float >= 0.5f) {
    g_enable_car_engine = 1;
  }
  else{
    g_enable_car_engine = 0;
  }

  temp_float = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  if (temp_float >= 0.5f) {
    g_enable_car_steering_wheel = 1;
  }
  else{
    g_enable_car_steering_wheel = 0;
  }

  g_emergency_brake_speed_mps = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  g_emergency_brake_distance_from_obstacle_m = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);

  temp_float = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  if (temp_float >= 0.5f) {
    g_enable_emergency_brake = 1;
  }
  else{
    g_enable_emergency_brake = 0;
  }

  temp_float = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  if (temp_float >= 0.5f) {
    g_enable_pixy_vector_approximation = 1;
  }
  else{
    g_enable_pixy_vector_approximation = 0;
  }

  temp_float = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  if (temp_float >= 0.5f) {
    g_enable_distance_sensor1 = 1;
  }
  else{
    g_enable_distance_sensor1 = 0;
  }

  temp_float = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  if (temp_float >= 0.5f) {
    g_enable_distance_sensor2 = 1;
  }
  else{
    g_enable_distance_sensor2 = 0;
  }

  g_emergency_brake_enable_delay_s = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  g_steering_wheel_angle_offset_deg = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);

  temp_float = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  if (temp_float >= 0.5f) {
    g_enable_distance_sensor3 = 1;
  }
  else{
    g_enable_distance_sensor3 = 0;
  }

  g_min_x_axis_angle_vector_deg = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  g_max_speed_after_delay_mps = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);

  temp_float = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  if (temp_float >= 0.5f) {
    g_enable_remote_start_stop = 1;
  }
  else{
    g_enable_remote_start_stop = 0;
  }

  g_car_speed_mps_ki = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  g_car_speed_mps_kd = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  g_car_speed_mps_ki_min_max_impact = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  g_finish_line_angle_tolerance = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);


  temp_float = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  if (temp_float >= 0.5f) {
    g_enable_finish_line_detection = 1;
  }
  else{
    g_enable_finish_line_detection = 0;
  }


  g_powertrain_left_wheel_kp = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  g_powertrain_left_wheel_ki = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  g_powertrain_left_wheel_kd = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  g_powertrain_left_wheel_ki_max_sum = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  g_powertrain_right_wheel_kp = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  g_powertrain_right_wheel_ki = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  g_powertrain_right_wheel_kd = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  g_powertrain_right_wheel_ki_max_sum = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    g_powertrain.SetLeftWheelPID(g_powertrain_left_wheel_kp, g_powertrain_left_wheel_ki, g_powertrain_left_wheel_kd, g_powertrain_left_wheel_ki_max_sum);
    g_powertrain.SetRightWheelPID(g_powertrain_right_wheel_kp, g_powertrain_right_wheel_ki, g_powertrain_right_wheel_kd, g_powertrain_right_wheel_ki_max_sum);
  }
}
*/

/*==============================================================================*/

void printGlobalVariables(SERIAL_PORT_TYPE &serialPort){
  char separatorCharacter;
  int n_decimals = 3;
  separatorCharacter = ';';

  serialPort.print(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING));

  serialPort.print(FloatToString(g_lane_width_vector_unit, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_lookahead_min_distance_cm, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_lookahead_max_distance_cm, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_emergency_brake_activation_max_distance_m, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_vehicle_min_speed_mps, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_vehicle_max_speed_original_mps, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_black_color_treshold, n_decimals));
  serialPort.print(separatorCharacter);
  //serialPort.print(String(g_car_length_cm));
  //serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_enable_car_engine, 0));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_enable_car_steering_wheel, 0));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_emergency_brake_speed_mps, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_emergency_brake_distance_from_obstacle_m, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_enable_emergency_brake, 0));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_enable_pixy_vector_approximation, 0));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_enable_distance_sensor1, 0));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_enable_distance_sensor2, 0));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_emergency_brake_enable_delay_s, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_steering_wheel_angle_offset_deg, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_enable_distance_sensor3, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_min_x_axis_angle_vector_deg, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_max_speed_after_delay_mps, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_enable_remote_start_stop, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_car_speed_mps_ki, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_car_speed_mps_kd, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_car_speed_mps_ki_min_max_impact, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_finish_line_angle_tolerance, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_enable_finish_line_detection, 0));
  serialPort.print(separatorCharacter);

  serialPort.print(FloatToString(g_powertrain_left_wheel_kp, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_powertrain_left_wheel_ki, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_powertrain_left_wheel_kd, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_powertrain_left_wheel_ki_max_sum, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_powertrain_right_wheel_kp, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_powertrain_right_wheel_ki, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_powertrain_right_wheel_kd, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_powertrain_right_wheel_ki_max_sum, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_friction_coefficient, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_downward_acceleration, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_max_acceleration, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_max_deceleration, n_decimals));

  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_line_calibration_data.angle_offset, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_line_calibration_data.rotation_point.x, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_line_calibration_data.rotation_point.y, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_line_calibration_data.x_axis_offset, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_line_calibration_data.y_axis_offset, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_birdeye_calibrationdata.valid, 0));

  serialPort.print(separatorCharacter);
  serialPort.print(FloatToString(g_enable_change_aeb_max_distance_after_delay_s, n_decimals));

  serialPort.println();
}


void setGlobalVariables(std::vector<float> &fields){
  size_t total_fields = TOTAL_GLOBAL_PARAMETERS;
  float temp_float;

  if (fields.size() < total_fields) {
    SERIAL_PORT.print(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING);
    SERIAL_PORT.println(String("ERROR: missing ") + String(total_fields - fields.size()) + String(" fields"));
    return;
  }

  
  g_lane_width_vector_unit = fields[0];
  g_lookahead_min_distance_cm = fields[1];
  g_lookahead_max_distance_cm = fields[2];
  g_emergency_brake_activation_max_distance_m = fields[3];
  g_vehicle_min_speed_mps = fields[4];
  g_vehicle_max_speed_original_mps = fields[5];
  g_black_color_treshold = fields[6];
  
  temp_float = fields[7];
  if (temp_float >= 0.5f) {
    g_enable_car_engine = 1;
  }
  else{
    g_enable_car_engine = 0;
  }

  temp_float = fields[8];
  if (temp_float >= 0.5f) {
    g_enable_car_steering_wheel = 1;
  }
  else{
    g_enable_car_steering_wheel = 0;
  }
#if ENABLE_WIRELESS_DEBUG_LIMITED == 0
  g_emergency_brake_speed_mps = fields[9];
  g_emergency_brake_distance_from_obstacle_m = fields[10];
  
  temp_float = fields[11];
  if (temp_float >= 0.5f) {
    g_enable_emergency_brake = 1;
  }
  else{
    g_enable_emergency_brake = 0;
  }

  temp_float = fields[12];
  if (temp_float >= 0.5f) {
    g_enable_pixy_vector_approximation = 1;
  }
  else{
    g_enable_pixy_vector_approximation = 0;
  }

  temp_float = fields[13];
  if (temp_float >= 0.5f) {
    g_enable_distance_sensor1 = 1;
  }
  else{
    g_enable_distance_sensor1 = 0;
  }

  temp_float = fields[14];
  if (temp_float >= 0.5f) {
    g_enable_distance_sensor2 = 1;
  }
  else{
    g_enable_distance_sensor2 = 0;
  }

  g_emergency_brake_enable_delay_s = fields[15];
  g_steering_wheel_angle_offset_deg = fields[16];


  temp_float = fields[17];
  if (temp_float >= 0.5f) {
    g_enable_distance_sensor3 = 1;
  }
  else{
    g_enable_distance_sensor3 = 0;
  }

  g_min_x_axis_angle_vector_deg = fields[18];
  g_max_speed_after_delay_mps = fields[19];

  temp_float = fields[20];
  if (temp_float >= 0.5f) {
    g_enable_remote_start_stop = 1;
  }
  else{
    g_enable_remote_start_stop = 0;
  }

  g_car_speed_mps_ki = fields[21];
  g_car_speed_mps_kd = fields[22];
  g_car_speed_mps_ki_min_max_impact = fields[23];
  g_finish_line_angle_tolerance = fields[24];

  temp_float = fields[25];
  if (temp_float >= 0.5f) {
    g_enable_finish_line_detection = 1;
  }
  else{
    g_enable_finish_line_detection = 0;
  }

  g_powertrain_left_wheel_kp = fields[26];
  g_powertrain_left_wheel_ki = fields[27];
  g_powertrain_left_wheel_kd = fields[28];
  g_powertrain_left_wheel_ki_max_sum = fields[29];

  g_powertrain_right_wheel_kp = fields[30];
  g_powertrain_right_wheel_ki = fields[31];
  g_powertrain_right_wheel_kd = fields[32];
  g_powertrain_right_wheel_ki_max_sum = fields[33];

  g_friction_coefficient = fields[34];
  g_downward_acceleration = fields[35];


  g_max_acceleration = fields[36];
  g_max_deceleration = fields[37];

  g_line_calibration_data.angle_offset = fields[38];
  g_line_calibration_data.rotation_point.x = fields[39];
  g_line_calibration_data.rotation_point.y = fields[40];
  g_line_calibration_data.x_axis_offset = fields[41];
  g_line_calibration_data.y_axis_offset = fields[42];

  g_max_speed_after_delay_s = fields[43];
  g_enable_finish_line_detection_after_delay_s = fields[44];
  g_camera_offset_y_m = fields[45];

  g_max_speed_after_finish_line_detected_mps = fields[46];

  struct track_widths temp_track_widths;

  temp_track_widths.lower_segment.A.x = fields[47];
  temp_track_widths.lower_segment.A.y = fields[48];
  temp_track_widths.lower_segment.B.x = fields[49];
  temp_track_widths.lower_segment.B.y = fields[50];

  temp_track_widths.upper_segment.A.x = fields[51];
  temp_track_widths.upper_segment.A.y = fields[52];
  temp_track_widths.upper_segment.B.x = fields[53];
  temp_track_widths.upper_segment.B.y = fields[54];

  #if ENABLE_BIRDEYEVIEW != 0
    g_birdeye_calibrationdata = CalculateBirdEyeCalibration_TrackWidths(temp_track_widths, g_line_image_frame_width, g_line_image_frame_height, LANE_WIDTH_M);
    temp_float = fields[55];
    if (temp_float >= 0.5f) {
      if (g_birdeye_calibrationdata.valid != 0) {
        g_lane_width_vector_unit = g_birdeye_calibrationdata.src_track_width;
      }
    }
    else{
      g_birdeye_calibrationdata.valid = 0;
    }
  #endif

  g_enable_change_aeb_max_distance_after_delay_s = fields[56];

  temp_float = fields[57];
  if (temp_float >= 0.5f) {
    g_enable_oversteer_mitigation = 1;
  }
  else{
    g_enable_oversteer_mitigation = 0;
  }

  g_oversteer_mitigation_yaw_tolerance_rad_s = fields[58];
  g_oversteer_mitigation_yaw_delta_tolerance_rad_s = fields[59];

  g_edf_raw_speed = fields[60];


#if ENABLE_DRIVERMOTOR == 1
  #if ENABLE_SINGLE_AXE_STEERING_NO_RPM != 0
  #else
  noInterrupts();
    g_powertrain.SetLeftWheelPID(g_powertrain_left_wheel_kp, g_powertrain_left_wheel_ki, g_powertrain_left_wheel_kd, g_powertrain_left_wheel_ki_max_sum);
    g_powertrain.SetRightWheelPID(g_powertrain_right_wheel_kp, g_powertrain_right_wheel_ki, g_powertrain_right_wheel_kd, g_powertrain_right_wheel_ki_max_sum);
  interrupts();
  #endif
#endif
#endif
}


void parseAndSetGlobalVariables_2(std::string& rawData, char variableTerminator = ';') {
  float temp_float;
  int total_fields = TOTAL_GLOBAL_PARAMETERS;
  std::stringstream ss(rawData);
  std::vector<float> fields;
  SERIAL_PORT.print(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING);
    int i = 0;
    while (ss.good()) {
        std::string substr;
        getline(ss, substr, variableTerminator);
        SERIAL_PORT.print(String("[") + String(i)+ String("]") + String(substr.c_str()) + String(";"));
        if (isNumber(substr.data(), substr.length()) == 0) {
          g_enable_car_engine = 0;
          g_enable_car_steering_wheel = 0;
          SERIAL_PORT.println();
          SERIAL_PORT.print(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING);
          SERIAL_PORT.println("ERROR: invaild field");
          return;
        }
        fields.push_back(std::stof(substr));
        i++;
    }
    SERIAL_PORT.println();
    if (fields.size() < total_fields) {
      SERIAL_PORT.print(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING);
      SERIAL_PORT.println(String("ERROR: missing ") + String(total_fields - fields.size()) + String(" fields"));
      return;
    }
    
    setGlobalVariables(fields);
}


#include "strtod_.h"
void parseAndSetGlobalVariables_optimized(std::vector<char>* rawData, char variableTerminator = ';'){
  std::vector<float> fields;
  char* ptr = rawData->data();
  char* endPtr;

  while (*ptr) {
    double value = strtod_(ptr, &endPtr);
    if (ptr == endPtr) {
        SERIAL_PORT.print(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING);
        SERIAL_PORT.println("ERROR: Invalid field");
        return;
    }
    fields.push_back(value);
    
    // Move pointer to the next field
    ptr = (*endPtr == variableTerminator) ? (endPtr + 1) : endPtr;
  }
  setGlobalVariables(fields);
}

void parseInputGlobalVariablesRoutine(SERIAL_PORT_TYPE &serialPort){
  std::string serialInputBuffer;
  if(readRecordFromSerial(serialPort, String("\r\n"), serialInputBuffer)){
    serialPort.print(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("Input: "));
    serialPort.println(serialInputBuffer.c_str());
    parseAndSetGlobalVariables_2(serialInputBuffer, ';');
    //parseAndSetGlobalVariables(serialInputBuffer, ';');
    //g_enable_car_engine = 1;
    printGlobalVariables(serialPort);
  }
}


void parseInputGlobalVariablesRoutine_optimized(SERIAL_PORT_TYPE &serialPort){
  std::vector<char> *serialInputBuffer;
  if(readRecordFromSerial_vector(serialPort, String("\r\n"), &serialInputBuffer)){
    serialPort.print(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("Input["));
    serialPort.print(FloatToString((float)serialInputBuffer->size(), 0));
    serialPort.print("]: ");
    //char tempchar = 0;
    //serialInputBuffer.push_back(tempchar);
    serialPort.println(serialInputBuffer->data());
    
    
    //parseAndSetGlobalVariables_2(serialInputBuffer, ';');
    //parseAndSetGlobalVariables(serialInputBuffer, ';');
    //g_enable_car_engine = 1;
    

    #if ENABLE_WIRELESS_DEBUG_LIMITED == 0
      printGlobalVariables(serialPort);
    #endif
    parseAndSetGlobalVariables_optimized(serialInputBuffer, ';');
  }
}
