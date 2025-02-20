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


#if CAR1 == 1
int8_t g_enable_car_engine = 0;
int8_t g_enable_car_steering_wheel = 0;
int8_t g_enable_emergency_brake = 0;
int8_t g_enable_pixy_vector_approximation = 0;
int8_t g_enable_distance_sensor1 = 1;
int8_t g_enable_distance_sensor2 = 1;
int8_t g_enable_distance_sensor3 = 1;
int8_t g_enable_remote_start_stop = 0;
int8_t g_enable_finish_line_detection = 0;

float g_lane_width_vector_unit = 45.0f;
float g_black_color_treshold = 0.2f; // 0=black, 1=white
float g_lookahead_min_distance_cm = 20.0f;
float g_lookahead_max_distance_cm = 60.0f;
float g_vehicle_min_speed_mps = 0.3f;   // m/s
float g_vehicle_max_speed_mps = 2.5f;  // m/s
float g_vehicle_max_speed_original_mps = 2.5f; // m/s
float g_emergency_brake_activation_max_distance_m = 0.6f;
float g_emergency_brake_speed_mps = 0.2f; // m/s
float g_emergency_brake_distance_from_obstacle_m = 0.2f;   // 13.5f
float g_steering_wheel_angle_offset_deg = -6.29f;      //-4.6f;
float g_min_x_axis_angle_vector_deg = 15.0f;
float g_max_speed_after_delay_mps = 1.5f; // m/s
float g_max_speed_after_finish_line_detected_mps = 0.7f; // m/s
float g_car_speed_mps_ki = -0.02f;
float g_car_speed_mps_kd = -0.2f;
float g_car_speed_mps_ki_min_max_impact = 0.3f;
float g_finish_line_angle_tolerance = 15.0f;

float g_powertrain_left_wheel_kp = 0.0;
float g_powertrain_left_wheel_ki = 0.1;
float g_powertrain_left_wheel_kd = 0.0;
float g_powertrain_left_wheel_ki_max_sum = 0.4;
float g_powertrain_right_wheel_kp = 0.0;
float g_powertrain_right_wheel_ki = 0.1;
float g_powertrain_right_wheel_kd = 0.0;
float g_powertrain_right_wheel_ki_max_sum = 0.4;

float g_friction_coefficient = 0.55f;
float g_downward_acceleration = G_CONSTANT;

float g_max_acceleration = -1.0f;
float g_max_deceleration = -1.0f;

float g_camera_offset_y_m = 0.0f;


#if RACE_MODE == 1
  float g_emergency_brake_enable_delay_s = 0.0f;
  float g_max_speed_after_delay_s = 0.0f;
  float g_enable_finish_line_detection_after_delay_s = 0.0f;
#elif DEBUG_MODE == 1
  float g_emergency_brake_enable_delay_s = 0.0f;
  float g_max_speed_after_delay_s = 0.0f;
  float g_enable_finish_line_detection_after_delay_s = 0.0f;
#else
  float g_emergency_brake_enable_delay_s = 0.0f;
  float g_max_speed_after_delay_s = 0.0f;
  float g_enable_finish_line_detection_after_delay_s = 0.0f;
#endif

#endif


float g_line_image_frame_width = 78.0f;
float g_line_image_frame_height = 51.0f;
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
LineCalibrationData g_line_calibration_data = {};
float g_rear_axe_turn_radius_m = 0.0f;
int8_t g_max_speed_delay_passed = 0;

struct BirdEyeCalibrationData g_birdeye_calibrationdata = {};


//SteeringWheel g_steering_wheel(STEERING_SERVO_ANGLE_MAX_LEFT, STEERING_SERVO_ANGLE_MIDDLE, STEERING_SERVO_ANGLE_MAX_RIGHT, (unsigned int)0);
#if ENABLE_STEERING_SERVO == 1
  SteeringWheel g_steering_wheel(WHEEL_BASE_M, TRACK_WIDTH_M, STEERING_SERVO_ANGLE_MAX_LEFT, STEERING_SERVO_ANGLE_MIDDLE, STEERING_SERVO_ANGLE_MAX_RIGHT);
#endif

VectorsProcessing g_pixy_1_vectors_processing;
//VectorsProcessing pixy_2_vectorsProcessing;
Pixy2 g_pixy_1;
//Pixy2SPI_SS pixy_2;


void parseAndSetGlobalVariables_2(std::string& rawData, char variableTerminator = ';') {
  float temp_float;
  int total_fields = 47;
  std::stringstream ss(rawData);
  std::vector<std::string> fields;
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
        fields.push_back(substr);
        i++;
    }
    SERIAL_PORT.println();
    if (fields.size() < total_fields) {
      SERIAL_PORT.print(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING);
      SERIAL_PORT.println(String("ERROR: missing ") + String(total_fields - fields.size()) + String(" fields"));
      return;
    }
    

    g_lane_width_vector_unit = std::stof(fields[0]);
    g_lookahead_min_distance_cm = std::stof(fields[1]);
    g_lookahead_max_distance_cm = std::stof(fields[2]);
    g_emergency_brake_activation_max_distance_m = std::stof(fields[3]);
    g_vehicle_min_speed_mps = std::stof(fields[4]);
    g_vehicle_max_speed_original_mps = std::stof(fields[5]);
    g_black_color_treshold = std::stof(fields[6]);
    
    temp_float = std::stof(fields[7]);
    if (temp_float >= 0.5f) {
      g_enable_car_engine = 1;
    }
    else{
      g_enable_car_engine = 0;
    }

    temp_float = std::stof(fields[8]);
    if (temp_float >= 0.5f) {
      g_enable_car_steering_wheel = 1;
    }
    else{
      g_enable_car_steering_wheel = 0;
    }

    g_emergency_brake_speed_mps = std::stof(fields[9]);
    g_emergency_brake_distance_from_obstacle_m = std::stof(fields[10]);
    
    temp_float = std::stof(fields[11]);
    if (temp_float >= 0.5f) {
      g_enable_emergency_brake = 1;
    }
    else{
      g_enable_emergency_brake = 0;
    }

    temp_float = std::stof(fields[12]);
    if (temp_float >= 0.5f) {
      g_enable_pixy_vector_approximation = 1;
    }
    else{
      g_enable_pixy_vector_approximation = 0;
    }

    temp_float = std::stof(fields[13]);
    if (temp_float >= 0.5f) {
      g_enable_distance_sensor1 = 1;
    }
    else{
      g_enable_distance_sensor1 = 0;
    }

    temp_float = std::stof(fields[14]);
    if (temp_float >= 0.5f) {
      g_enable_distance_sensor2 = 1;
    }
    else{
      g_enable_distance_sensor2 = 0;
    }

    g_emergency_brake_enable_delay_s = std::stof(fields[15]);
    g_steering_wheel_angle_offset_deg = std::stof(fields[16]);


    temp_float = std::stof(fields[17]);
    if (temp_float >= 0.5f) {
      g_enable_distance_sensor3 = 1;
    }
    else{
      g_enable_distance_sensor3 = 0;
    }

    g_min_x_axis_angle_vector_deg = std::stof(fields[18]);
    g_max_speed_after_delay_mps = std::stof(fields[19]);

    temp_float = std::stof(fields[20]);
    if (temp_float >= 0.5f) {
      g_enable_remote_start_stop = 1;
    }
    else{
      g_enable_remote_start_stop = 0;
    }

    g_car_speed_mps_ki = std::stof(fields[21]);
    g_car_speed_mps_kd = std::stof(fields[22]);
    g_car_speed_mps_ki_min_max_impact = std::stof(fields[23]);
    g_finish_line_angle_tolerance = std::stof(fields[24]);

    temp_float = std::stof(fields[25]);
    if (temp_float >= 0.5f) {
      g_enable_finish_line_detection = 1;
    }
    else{
      g_enable_finish_line_detection = 0;
    }

    g_powertrain_left_wheel_kp = std::stof(fields[26]);
    g_powertrain_left_wheel_ki = std::stof(fields[27]);
    g_powertrain_left_wheel_kd = std::stof(fields[28]);
    g_powertrain_left_wheel_ki_max_sum = std::stof(fields[29]);

    g_powertrain_right_wheel_kp = std::stof(fields[30]);
    g_powertrain_right_wheel_ki = std::stof(fields[31]);
    g_powertrain_right_wheel_kd = std::stof(fields[32]);
    g_powertrain_right_wheel_ki_max_sum = std::stof(fields[33]);

    g_friction_coefficient = std::stof(fields[34]);
    g_downward_acceleration = std::stof(fields[35]);


    g_max_acceleration = std::stof(fields[36]);
    g_max_deceleration = std::stof(fields[37]);

    g_line_calibration_data.angle_offset = std::stof(fields[38]);
    g_line_calibration_data.rotation_point.x = std::stof(fields[39]);
    g_line_calibration_data.rotation_point.y = std::stof(fields[40]);
    g_line_calibration_data.x_axis_offset = std::stof(fields[41]);
    g_line_calibration_data.y_axis_offset = std::stof(fields[42]);

    g_max_speed_after_delay_s = std::stof(fields[43]);
    g_enable_finish_line_detection_after_delay_s = std::stof(fields[44]);
    g_camera_offset_y_m = std::stof(fields[45]);

    g_max_speed_after_finish_line_detected_mps = std::stof(fields[46]);


  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    g_powertrain.SetLeftWheelPID(g_powertrain_left_wheel_kp, g_powertrain_left_wheel_ki, g_powertrain_left_wheel_kd, g_powertrain_left_wheel_ki_max_sum);
    g_powertrain.SetRightWheelPID(g_powertrain_right_wheel_kp, g_powertrain_right_wheel_ki, g_powertrain_right_wheel_kd, g_powertrain_right_wheel_ki_max_sum);
  }
}

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

  serialPort.print(String(g_lane_width_vector_unit, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_lookahead_min_distance_cm, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_lookahead_max_distance_cm, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_emergency_brake_activation_max_distance_m, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_vehicle_min_speed_mps, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_vehicle_max_speed_original_mps, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_black_color_treshold, n_decimals));
  serialPort.print(separatorCharacter);
  //serialPort.print(String(g_car_length_cm));
  //serialPort.print(separatorCharacter);
  serialPort.print(String(g_enable_car_engine, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_enable_car_steering_wheel, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_emergency_brake_speed_mps, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_emergency_brake_distance_from_obstacle_m, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_enable_emergency_brake, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_enable_pixy_vector_approximation, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_enable_distance_sensor1, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_enable_distance_sensor2, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_emergency_brake_enable_delay_s, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_steering_wheel_angle_offset_deg, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_enable_distance_sensor3, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_min_x_axis_angle_vector_deg, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_max_speed_after_delay_mps, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_enable_remote_start_stop, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_car_speed_mps_ki, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_car_speed_mps_kd, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_car_speed_mps_ki_min_max_impact, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_finish_line_angle_tolerance, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_enable_finish_line_detection, n_decimals));
  serialPort.print(separatorCharacter);

  serialPort.print(String(g_powertrain_left_wheel_kp, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_powertrain_left_wheel_ki, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_powertrain_left_wheel_kd, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_powertrain_left_wheel_ki_max_sum, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_powertrain_right_wheel_kp, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_powertrain_right_wheel_ki, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_powertrain_right_wheel_kd, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_powertrain_right_wheel_ki_max_sum, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_friction_coefficient, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_downward_acceleration, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_max_acceleration, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_max_deceleration, n_decimals));

  serialPort.print(separatorCharacter);
  serialPort.print(String(g_line_calibration_data.angle_offset, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_line_calibration_data.rotation_point.x, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_line_calibration_data.rotation_point.y, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_line_calibration_data.x_axis_offset, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_line_calibration_data.y_axis_offset, n_decimals));

  serialPort.println();
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
