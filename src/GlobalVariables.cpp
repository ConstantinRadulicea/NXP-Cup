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
int g_enable_car_engine = 0;
int g_enable_car_steering_wheel = 0;
int g_enable_emergency_brake = 1;
int g_enable_pixy_vector_approximation = 0;
int g_enable_distance_sensor1 = 1;
int g_enable_distance_sensor2 = 1;
int g_enable_distance_sensor3 = 1;
int g_enable_remote_start_stop = 0;
int g_enable_finish_line_detection = 1;

float g_lane_width_vector_unit = 53.0f;
float g_black_color_treshold = 0.2f; // 0=black, 1=white
float g_lookahead_min_distance_cm = 22.0f;
float g_lookahead_max_distance_cm = 50.0f;
float g_min_speed = 0.2f;   // m/s
float g_max_speed = 2.0f;  // m/s
float g_emergency_brake_distance_m = 1.0f;
float g_emergency_brake_min_speed = 0.2f; // m/s
float g_emergency_brake_distance_from_obstacle_m = 0.09f;   // 13.5f
float g_steering_wheel_angle_offset = 0.0f;
float g_min_x_axis_angle_vector = 15.0f;
float g_max_speed_after_emergency_brake_delay = 2.0f; // m/s
float g_car_speed_ki = -0.02f;
float g_car_speed_kd = -0.2f;
float g_car_speed_ki_min_max_impact = 0.2f;
float g_finish_line_angle_tolerance = 15.0f;

#if RACE_MODE == 1
  float g_emergency_brake_enable_delay_s = 15.0f;
#elif DEBUG_MODE == 1
  float g_emergency_brake_enable_delay_s = 0.0f;
#else
  float g_emergency_brake_enable_delay_s = 15.0f;
#endif


#elif CAR2 == 1
int g_enable_car_engine = 0;
int g_enable_car_steering_wheel = 0;
int g_enable_emergency_brake = 1;
int g_enable_pixy_vector_approximation = 0;
int g_enable_distance_sensor1 = 1;
int g_enable_distance_sensor2 = 1;
int g_enable_distance_sensor3 = 1;
int g_enable_remote_start_stop = 1;

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
float g_steering_wheel_angle_offset = 0.0f;
float g_min_x_axis_angle_vector = 15.0f;
float g_max_speed_after_emergency_brake_delay = 107.0f;
float g_car_speed_ki = -0.01f;
float g_car_speed_kd = -0.4f;
float g_car_speed_ki_min_max_impact = 5.0f;


#if RACE_MODE == 1
  float g_emergency_brake_enable_delay_s = 20.0f;
#elif DEBUG_MODE == 1
  float g_emergency_brake_enable_delay_s = 0.0f;
#else
  float g_emergency_brake_enable_delay_s = 15.0f;
#endif

#endif





float g_car_length_vector_unit = (float)MeterToVectorUnit(CAR_LENGTH_M);
int g_emergency_break_active =(int) 0;
unsigned int g_emergency_break_loops_count = (int)0;
float g_car_speed = (float)STANDSTILL_SPEED;
LineABC g_middle_lane_line_pixy_1;
LineABC g_left_lane_line_pixy_1;
LineABC g_right_lane_line_pixy_1;
float g_loop_time_ms = 0.0f;
float g_time_passed_ms = 0.0f;
float g_emergency_brake_enable_remaining_delay_s = 0.0f;
int g_emergency_brake_enable_delay_started_count = 0;
int g_finish_line_detected = 0;
int g_finish_line_detected_now = 0;
FinishLine g_finish_line = {};


SteeringWheel g_steering_wheel(STEERING_SERVO_ANGLE_MAX_LEFT, STEERING_SERVO_ANGLE_MIDDLE, STEERING_SERVO_ANGLE_MAX_RIGHT, (unsigned int)0);

VectorsProcessing g_pixy_1_vectors_processing;
//VectorsProcessing pixy_2_vectorsProcessing;
Pixy2 g_pixy_1;
//Pixy2SPI_SS pixy_2;




void parseAndSetGlobalVariables(std::vector<char>& rawData, char variableTerminator = ';') {
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
  g_steering_wheel_angle_offset = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);

  temp_float = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  if (temp_float >= 0.5f) {
    g_enable_distance_sensor3 = 1;
  }
  else{
    g_enable_distance_sensor3 = 0;
  }

  g_min_x_axis_angle_vector = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  g_max_speed_after_emergency_brake_delay = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);

  temp_float = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  if (temp_float >= 0.5f) {
    g_enable_remote_start_stop = 1;
  }
  else{
    g_enable_remote_start_stop = 0;
  }

  g_car_speed_ki = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  g_car_speed_kd = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  g_car_speed_ki_min_max_impact = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  g_finish_line_angle_tolerance = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);


  temp_float = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  if (temp_float >= 0.5f) {
    g_enable_finish_line_detection = 1;
  }
  else{
    g_enable_finish_line_detection = 0;
  }
}

/*==============================================================================*/

void printGlobalVariables(HardwareSerial& serialPort){
  char separatorCharacter;
  separatorCharacter = ';';

  serialPort.print(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING));

  serialPort.print(String(g_lane_width_vector_unit));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_lookahead_min_distance_cm));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_lookahead_max_distance_cm));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_emergency_brake_distance_m));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_min_speed));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_max_speed));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_black_color_treshold));
  serialPort.print(separatorCharacter);
  //serialPort.print(String(g_car_length_cm));
  //serialPort.print(separatorCharacter);
  serialPort.print(String(g_enable_car_engine));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_enable_car_steering_wheel));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_emergency_brake_min_speed));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_emergency_brake_distance_from_obstacle_m));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_enable_emergency_brake));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_enable_pixy_vector_approximation));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_enable_distance_sensor1));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_enable_distance_sensor2));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_emergency_brake_enable_delay_s));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_enable_distance_sensor3));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_min_x_axis_angle_vector));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_max_speed_after_emergency_brake_delay));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_enable_remote_start_stop));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_car_speed_ki));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_car_speed_kd));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_car_speed_ki_min_max_impact));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_finish_line_angle_tolerance));
  serialPort.print(separatorCharacter);
  serialPort.print(String(g_enable_finish_line_detection));


  serialPort.println();
}

void parseInputGlobalVariablesRoutine(HardwareSerial& serialPort){
  std::vector<char> serialInputBuffer;
  if(readRecordFromSerial(serialPort, String("\r\n"), serialInputBuffer)){
    serialPort.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("Input: ") + String(serialInputBuffer.data()));
    parseAndSetGlobalVariables(serialInputBuffer, ';');
    printGlobalVariables(serialPort);
    serialInputBuffer.clear();
  }
}
