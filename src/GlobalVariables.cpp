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
int8_t g_enable_emergency_brake = 1;
int8_t g_enable_pixy_vector_approximation = 0;
int8_t g_enable_distance_sensor1 = 0;
int8_t g_enable_distance_sensor2 = 1;
int8_t g_enable_distance_sensor3 = 0;
int8_t g_enable_remote_start_stop = 0;
int8_t g_enable_finish_line_detection = 0;

float g_lane_width_vector_unit = 45.0f;
float g_black_color_treshold = 0.2f; // 0=black, 1=white
float g_lookahead_min_distance_cm = 20.0f;
float g_lookahead_max_distance_cm = 60.0f;
float g_min_speed = 0.3f;   // m/s
float g_max_speed = 2.5f;  // m/s
float g_emergency_brake_distance_m = 0.6f;
float g_emergency_brake_min_speed = 0.2f; // m/s
float g_emergency_brake_distance_from_obstacle_m = 0.2f;   // 13.5f
float g_steering_wheel_angle_offset_deg = -4.6f;
float g_min_x_axis_angle_vector_deg = 15.0f;
float g_max_speed_after_emergency_brake_delay = 2.0f; // m/s
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


#if RACE_MODE == 1
  float g_emergency_brake_enable_delay_s = 15.0f;
#elif DEBUG_MODE == 1
  float g_emergency_brake_enable_delay_s = 0.0f;
#else
  float g_emergency_brake_enable_delay_s = 15.0f;
#endif


#elif CAR2 == 1
int8_t g_enable_car_engine = 0;
int8_t g_enable_car_steering_wheel = 0;
int8_t g_enable_emergency_brake = 1;
int8_t g_enable_pixy_vector_approximation = 0;
int8_t g_enable_distance_sensor1 = 1;
int8_t g_enable_distance_sensor2 = 1;
int8_t g_enable_distance_sensor3 = 1;
int8_t g_enable_remote_start_stop = 1;

float g_lane_width_vector_unit = 53.0f;
float g_black_color_treshold = 0.2f; // 0=black, 1=white
float g_car_length_cm = 17.5f;
float g_lookahead_min_distance_cm = 22.0f;
float g_lookahead_max_distance_cm = 50.0f;
float g_min_speed = 97.0f + CAR2_PARAMETERS_DIFFERENCE;
float g_max_speed = 125.0f  + CAR2_PARAMETERS_DIFFERENCE;
float g_emergency_brake_distance_m = 1.0f;
float g_emergency_brake_min_speed = 94.0f + CAR2_PARAMETERS_DIFFERENCE;
float g_emergency_brake_distance_from_obstacle_m = 1.0f;   // 13.5f
float g_steering_wheel_angle_offset_deg = 0.0f;
float g_min_x_axis_angle_vector_deg = 15.0f;
float g_max_speed_after_emergency_brake_delay = 107.0f;
float g_car_speed_mps_ki = -0.01f;
float g_car_speed_mps_kd = -0.4f;
float g_car_speed_mps_ki_min_max_impact = 5.0f;


#if RACE_MODE == 1
  float g_emergency_brake_enable_delay_s = 20.0f;
#elif DEBUG_MODE == 1
  float g_emergency_brake_enable_delay_s = 0.0f;
#else
  float g_emergency_brake_enable_delay_s = 15.0f;
#endif

#endif





float g_wheel_base_vector_unit = (float)MeterToVectorUnit(WHEEL_BASE_M);
int8_t g_emergency_break_active =0;
unsigned int g_emergency_break_loops_count = 0;
float g_car_speed_mps = (float)STANDSTILL_SPEED;
LineABC g_middle_lane_line_pixy_1;
LineABC g_left_lane_line_pixy_1;
LineABC g_right_lane_line_pixy_1;
float g_loop_time_ms = 0.0f;
float g_time_passed_ms = 0.0f;
float g_emergency_brake_enable_remaining_delay_s = 0.0f;
int8_t g_emergency_brake_enable_delay_started_count = 0;
int8_t g_finish_line_detected = 0;
int8_t g_finish_line_detected_now = 0;
float g_steering_angle = 0.0f;
FinishLine g_finish_line = {};


SteeringWheel g_steering_wheel(STEERING_SERVO_ANGLE_MAX_LEFT, STEERING_SERVO_ANGLE_MIDDLE, STEERING_SERVO_ANGLE_MAX_RIGHT, (unsigned int)0);

VectorsProcessing g_pixy_1_vectors_processing;
//VectorsProcessing pixy_2_vectorsProcessing;
Pixy2 g_pixy_1;
//Pixy2SPI_SS pixy_2;


void parseAndSetGlobalVariables_2(std::string& rawData, char variableTerminator = ';') {
  float temp_float;
    std::stringstream ss(rawData);
    std::vector<std::string> fields;
 SERIAL_PORT.print("%");
    int i = 0;
    while (ss.good()) {
        std::string substr;
        getline(ss, substr, variableTerminator);
        fields.push_back(substr);
        SERIAL_PORT.print(String("[") + String(i)+ String("]") + String(substr.c_str()) + String(";"));
        i++;
    }
    SERIAL_PORT.println();

    g_lane_width_vector_unit = std::stof(fields[0]);
    g_lookahead_min_distance_cm = std::stof(fields[1]);
    g_lookahead_max_distance_cm = std::stof(fields[2]);
    g_emergency_brake_distance_m = std::stof(fields[3]);
    g_min_speed = std::stof(fields[4]);
    g_max_speed = std::stof(fields[5]);
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

    g_emergency_brake_min_speed = std::stof(fields[9]);
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
    g_max_speed_after_emergency_brake_delay = std::stof(fields[19]);

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


  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    g_powertrain.SetLeftWheelPID(g_powertrain_left_wheel_kp, g_powertrain_left_wheel_ki, g_powertrain_left_wheel_kd, g_powertrain_left_wheel_ki_max_sum);
    g_powertrain.SetRightWheelPID(g_powertrain_right_wheel_kp, g_powertrain_right_wheel_ki, g_powertrain_right_wheel_kd, g_powertrain_right_wheel_ki_max_sum);
  }
}


void parseAndSetGlobalVariables(std::string& rawData, char variableTerminator = ';') {
	char* pEnd;
	int resultSuccess;
  float temp_float;

	pEnd = rawData.data();
	g_lane_width_vector_unit = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  g_lookahead_min_distance_cm = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
	g_lookahead_max_distance_cm = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
	g_emergency_brake_distance_m = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  g_min_speed = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
	g_max_speed = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
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

  g_emergency_brake_min_speed = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
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
  g_max_speed_after_emergency_brake_delay = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);

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
  serialPort.print(String(g_emergency_brake_distance_m, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_min_speed, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_max_speed, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_black_color_treshold, n_decimals));
  serialPort.print(separatorCharacter);
  //serialPort.print(String(g_car_length_cm));
  //serialPort.print(separatorCharacter);
  serialPort.print(String(g_enable_car_engine, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_enable_car_steering_wheel, n_decimals));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_emergency_brake_min_speed, n_decimals));
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
  serialPort.print(String(g_max_speed_after_emergency_brake_delay, n_decimals));
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

  serialPort.println();
}

void parseInputGlobalVariablesRoutine(SERIAL_PORT_TYPE &serialPort){
  std::string serialInputBuffer;
  if(readRecordFromSerial(serialPort, String("\r\n"), serialInputBuffer)){
    //serialPort.print(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("Input: "));
    //serialPort.println(serialInputBuffer.c_str());
    parseAndSetGlobalVariables_2(serialInputBuffer, ';');
    //parseAndSetGlobalVariables(serialInputBuffer, ';');
    //g_enable_car_engine = 1;
    printGlobalVariables(serialPort);
  }
}
