#ifndef __GLOBALVARIABLES_H__
#define __GLOBALVARIABLES_H__

#include <HardwareSerial.h>
#include <vector>
#include "parseNextFloat.h"


void parseAndSetGlobalVariables(std::vector<char>& rawData, char variableTerminator = ';') {
	char* pEnd;
	int resultSuccess;
  float temp_float;

	pEnd = rawData.data();
	lane_width_vector_unit_real = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  lookahead_min_distance_cm = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
	lookahead_max_distance_cm = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
	emergency_break_distance_cm = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  min_speed = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
	max_speed = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
	black_color_treshold = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
	car_length_cm = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  
  temp_float = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  if (temp_float >= 0.5f) {
    enable_car_engine = 1;
  }
  else{
    enable_car_engine = 0;
  }

  temp_float = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  if (temp_float >= 0.5f) {
    enable_car_steering_wheel = 1;
  }
  else{
    enable_car_steering_wheel = 0;
  }

  emergency_brake_min_speed = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  emergency_brake_distance_from_obstacle_cm = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);

  temp_float = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  if (temp_float >= 0.5f) {
    enable_emergency_brake = 1;
  }
  else{
    enable_emergency_brake = 0;
  }

  temp_float = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  if (temp_float >= 0.5f) {
    enable_pixy_vector_approximation_soft = 1;
  }
  else{
    enable_pixy_vector_approximation_soft = 0;
  }

  temp_float = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  if (temp_float >= 0.5f) {
    enable_distance_sensor1_soft = 1;
  }
  else{
    enable_distance_sensor1_soft = 0;
  }

  temp_float = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  if (temp_float >= 0.5f) {
    enable_distance_sensor2_soft = 1;
  }
  else{
    enable_distance_sensor2_soft = 0;
  }

  emergency_brake_enable_delay_s = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  steering_wheel_angle_offset = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);

  temp_float = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  if (temp_float >= 0.5f) {
    enable_distance_sensor3_soft = 1;
  }
  else{
    enable_distance_sensor3_soft = 0;
  }

  min_axis_angle_vector = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  max_speed_after_emergency_brake_delay = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);

  temp_float = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  if (temp_float >= 0.5f) {
    enable_remote_start_stop_soft = 1;
  }
  else{
    enable_remote_start_stop_soft = 0;
  }

  car_speed_ki = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  car_speed_kd = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  car_speed_ki_min_max_impact = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  finish_line_angle_tolerance = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);


  temp_float = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
  if (temp_float >= 0.5f) {
    enable_finish_line_detection_soft = 1;
  }
  else{
    enable_finish_line_detection_soft = 0;
  }
  car_length_vector_unit = car_length_cm * VECTOR_UNIT_PER_CM;

}

/*==============================================================================*/

void printGlobalVariables(HardwareSerial& serialPort){
  char separatorCharacter;
  separatorCharacter = ';';

  serialPort.print(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING));

  serialPort.print(String(lane_width_vector_unit_real));
  serialPort.print(separatorCharacter);
  serialPort.print(String(lookahead_min_distance_cm));
  serialPort.print(separatorCharacter);
  serialPort.print(String(lookahead_max_distance_cm));
  serialPort.print(separatorCharacter);
  serialPort.print(String(emergency_break_distance_cm));
  serialPort.print(separatorCharacter);
  serialPort.print(String(min_speed));
  serialPort.print(separatorCharacter);
  serialPort.print(String(max_speed));
  serialPort.print(separatorCharacter);
  serialPort.print(String(black_color_treshold));
  serialPort.print(separatorCharacter);
  serialPort.print(String(car_length_cm));
  serialPort.print(separatorCharacter);
  serialPort.print(String(enable_car_engine));
  serialPort.print(separatorCharacter);
  serialPort.print(String(enable_car_steering_wheel));
  serialPort.print(separatorCharacter);
  serialPort.print(String(emergency_brake_min_speed));
  serialPort.print(separatorCharacter);
  serialPort.print(String(emergency_brake_distance_from_obstacle_cm));
  serialPort.print(separatorCharacter);
  serialPort.print(String(enable_emergency_brake));
  serialPort.print(separatorCharacter);
  serialPort.print(String(enable_pixy_vector_approximation_soft));
  serialPort.print(separatorCharacter);
  serialPort.print(String(enable_distance_sensor1_soft));
  serialPort.print(separatorCharacter);
  serialPort.print(String(enable_distance_sensor2_soft));
  serialPort.print(separatorCharacter);
  serialPort.print(String(emergency_brake_enable_delay_s));
  serialPort.print(separatorCharacter);
  serialPort.print(String(enable_distance_sensor3_soft));
  serialPort.print(separatorCharacter);
  serialPort.print(String(min_axis_angle_vector));
  serialPort.print(separatorCharacter);
  serialPort.print(String(max_speed_after_emergency_brake_delay));
  serialPort.print(separatorCharacter);
  serialPort.print(String(enable_remote_start_stop_soft));
  serialPort.print(separatorCharacter);
  serialPort.print(String(car_speed_ki));
  serialPort.print(separatorCharacter);
  serialPort.print(String(car_speed_kd));
  serialPort.print(separatorCharacter);
  serialPort.print(String(car_speed_ki_min_max_impact));
  serialPort.print(separatorCharacter);
  serialPort.print(String(finish_line_angle_tolerance));
  serialPort.print(separatorCharacter);
  serialPort.print(String(enable_finish_line_detection_soft));


  serialPort.println();
}


#endif