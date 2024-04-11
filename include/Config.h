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
CAR1 & CAR2 
enable_car_engine = 0.0;
enable_car_steering_wheel = 1.0;
enable_emergency_brake = 1.0;
enable_pixy_vector_approximation = 0.0;             
enable_distance_sensor1 = 1.0;
enable_distance_sensor2 = 1.0;

lane_width_vector_unit_real = 52.0;
black_color_treshold = 0.2;
car_length_cm = 17.5;
lookahead_min_distance_cm = 22.0;                       % 22
lookahead_max_distance_cm = 40.0;                       % 40
min_speed = 97.0;
max_speed = 112.0;                                      % 115 merge si 120
emergency_break_distance_cm = 75;                     % 75
emergency_brake_min_speed = 94.0;
emergency_brake_distance_from_obstacle_cm = 75.0;       % 14
emergency_brake_enable_delay = 10.0;

*/

#ifndef __CONFIG_H__
#define __CONFIG_H__


#define CAR1 0
#define CAR2 1

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
static int enable_car_engine = 0;
static int enable_car_steering_wheel = 0;
static int enable_emergency_brake = 1;
static int enable_pixy_vector_approximation_soft = 0;
static int enable_distance_sensor1_soft = 1;
static int enable_distance_sensor2_soft = 1;

static float lane_width_vector_unit_real = 53.0f;
static float black_color_treshold = 0.2f; // 0=black, 1=white
static float car_length_cm = 17.5f;
static float lookahead_min_distance_cm = 16.0f;
static float lookahead_max_distance_cm = 30.0f;
static float min_speed = 97.0f + CAR2_PARAMETERS_DIFFERENCE;
static float max_speed = 107.0f  + CAR2_PARAMETERS_DIFFERENCE;
static float emergency_break_distance_cm = 70.0f;
static float emergency_brake_min_speed = 94.0f + CAR2_PARAMETERS_DIFFERENCE;
static float emergency_brake_distance_from_obstacle_cm = 13.5f;   // 13.5f
static float steering_wheel_angle_offset = 0.0f;

#if RACE_MODE == 1
  static float emergency_brake_enable_delay_s = 15.0f;
#elif DEBUG_MODE == 1
  static float emergency_brake_enable_delay_s = 0.0f;
#else
  static float emergency_brake_enable_delay_s = 15.0f;
#endif



/*====================================================================================================================================*/



#define ENABLE_ARDUINO 0

#include <Arduino.h>
#include <SPI.h>
#include "SteeringWheel.h"
#include "pixy2_libs/host/arduino/libraries/Pixy2/Pixy2.h"
#include "PurePursuitGeometry.h"
#include "VectorsProcessing.h"
#include "aproximatePixyVector.h"
#include "strtod_.h"
#include "SimpleKalmanFilter.h"
#include "MovingAverage.h"
#include <vector>


#if ENABLE_ARDUINO == 1
  #include <Servo.h>
#else
  #include <PWMServo.h>
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

#define DEBUG_WIFI_SSID "Off Limits2"
#define DEBUG_WIFI_PASSWORD "J7s2tzvzKzva"
//#define DEBUG_WIFI_SSID "B020"
//#define DEBUG_WIFI_PASSWORD "diferential2019"

//#define DEBUG_HOST_IPADDRESS "110.100.0.88"   // Constantin B020
#define DEBUG_HOST_IPADDRESS "192.168.55.244"   // Constantin phone
//#define DEBUG_HOST_IPADDRESS "192.168.0.227"   // Constantin home
//#define DEBUG_HOST_IPADDRESS "192.168.55.122"   // Daniel phone
//#define DEBUG_HOST_IPADDRESS "192.168.79.133"   // Alex
#define DEBUG_HOST_PORT 6789
#define DEBUG_WIFI_INIT_SEQUENCE "%SERIAL2WIFI\r\n"
#define ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING '%'


#if ENABLE_WIRELESS_DEBUG == 1
  #if ENABLE_ARDUINO == 1
    #define SERIAL_PORT Serial
  #else
    #define SERIAL_PORT Serial1
  #endif
#else
  #define SERIAL_PORT Serial
#endif

#if DEBUG_MODE == 1
  #define ENABLE_SERIAL_PRINT 1
  #define ENABLE_WIRELESS_DEBUG 1
  #define ENABLE_STEERING_SERVO 1
  #define ENABLE_DRIVERMOTOR 1
  #define ENABLE_SETTINGS_MENU 1
  #define ENABLE_EMERGENCY_BREAKING 1
  #define ENABLE_PIXY_VECTOR_APPROXIMATION 1
  #define ENABLE_DISTANCE_SENSOR1 1
  #define ENABLE_DISTANCE_SENSOR2 1
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
#endif

#if ENABLE_SETTINGS_MENU == 1
  #include <LiquidCrystal_I2C.h>
#endif

#if ENABLE_DISTANCE_SENSOR1 == 0 && ENABLE_DISTANCE_SENSOR2 == 0
  #define ENABLE_EMERGENCY_BREAKING 0
#endif


#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define abs(x) ({ \
  typeof(x) _x = (x); \
  (_x > 0) ? (_x) : (-_x); \
})

#define STEERING_SERVO_PIN  3
#define DRIVER_MOTOR_PIN  9

#define DISTANCE_SENSOR1_TRIG_PIN 2
#define DISTANCE_SENSOR1_ECHO_PIN 5
#define DISTANCE_SENSOR2_TRIG_PIN 7
#define DISTANCE_SENSOR2_ECHO_PIN 8

#define MENU_LEFT_ARROW_BUTTON_PIN 17
#define MENU_RIGHT_ARROW_BUTTON_PIN 16
#define MENU_INCREMENT_BUTTON_PIN 15
#define MENU_DECREMENT_BUTTON_PIN 14

#define EMERGENCY_BREAK_LIGHT_PIN 20

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


#define VECTOR_UNIT_PER_CM (float)((float)LANE_WIDTH_VECTOR_UNIT_REAL / (float)LANE_WIDTH_CM)   // CM * VECTOR_UNIT_PER_CM = VECTOR_UNIT
#define CM_PER_VECTOR_UNIT (float)((float)LANE_WIDTH_CM / (float)LANE_WIDTH_VECTOR_UNIT_REAL)   // VECTOR_UNIT_PER_CM * CM = CM
#define RADIANS_PER_DEGREE (float)((float)M_PI / 180.0f) // DEGREE * RADIAN_PER_DEGREE = RADIAN
#define DEGREES_PER_RADIAN (float)(180.0f / (float)M_PI) // RADIAN * DEGREE_PER_RADIAN = DEGREE

//#define LANE_WIDTH_VECTOR_UNIT (float)(LANE_WIDTH_VECTOR_UNIT_REAL + ((5.0f * VECTOR_UNIT_PER_CM)*2.0f))
#define LANE_WIDTH_VECTOR_UNIT (float)(LANE_WIDTH_VECTOR_UNIT_REAL)



#if CAR1 == 1
  #define STEERING_SERVO_ANGLE_MIDDLE     90    // 90 middle // 120
  #define STEERING_SERVO_ANGLE_MAX_RIGHT  20    // 0 max right // 30
  #define STEERING_SERVO_ANGLE_MAX_LEFT   160   // 180 max left // 210
#elif CAR2 == 1
  #define STEERING_SERVO_ANGLE_MIDDLE     90    // 90 middle
  #define STEERING_SERVO_ANGLE_MAX_RIGHT  10    // 0 max right
  #define STEERING_SERVO_ANGLE_MAX_LEFT   170   // 180 max left
#endif

#define STEERING_SERVO_MAX_ANGLE MAX(abs(STEERING_SERVO_ANGLE_MIDDLE - STEERING_SERVO_ANGLE_MAX_RIGHT), abs(STEERING_SERVO_ANGLE_MIDDLE - STEERING_SERVO_ANGLE_MAX_LEFT))

#define MIN_SPEED min_speed
#define MAX_SPEED max_speed
#if CAR1 == 1
  #define STANDSTILL_SPEED 90.0f      //(car1: 90)  (car2: 83)
#elif CAR2 == 1
  #define STANDSTILL_SPEED 83.0f      //(car1: 90)  (car2: 83)
#endif

#define ENABLE_CAR_ENGINE enable_car_engine
#define ENABLE_CAR_STEERING_WHEEL enable_car_steering_wheel
#define ENABLE_EMERGENCY_BRAKE enable_emergency_brake
#define ENABLE_DISTANCE_SENSOR1_SOFT enable_distance_sensor1_soft
#define ENABLE_DISTANCE_SENSOR2_SOFT enable_distance_sensor2_soft


#define ENABLE_PIXY_VECTOR_APPROXIMATION_SOFT enable_pixy_vector_approximation_soft

/*====================================================================================================================================*/
static float car_length_vector_unit = (float)car_length_cm * (float)VECTOR_UNIT_PER_CM;
static int emergency_break_active =(int) 0;
static unsigned int emergency_break_loops_count = (int)0;
static float carSpeed = (float)STANDSTILL_SPEED;
static LineABC middle_lane_line;
static LineABC left_lane_line;
static LineABC right_lane_line;
static float loop_time_ms = 0.0f;
static float time_passed_ms = 0.0f;
static float emergency_brake_enable_remaining_delay_s = 0.0f;
static int emergency_brake_enable_delay_started_count = 0;

/*====================================================================================================================================*/

static void printDataToSerial(Vector leftVectorOld, Vector rightVectorOld, Vector leftVector, Vector rightVector, LineABC leftLine, LineABC rightLine, LineABC laneMiddleLine, PurePersuitInfo purePersuitInfo, float carAcceleration, float frontObstacleDistance, float carSpeed_){
  String commaCharStr;
  char semicolonChar;

  commaCharStr = String(',');
  semicolonChar = ';';

  SERIAL_PORT.print(String(leftVectorOld.m_x0) + commaCharStr + String(leftVectorOld.m_y0) + commaCharStr + String(leftVectorOld.m_x1) + commaCharStr + String(leftVectorOld.m_y1));
  SERIAL_PORT.print(semicolonChar);
  SERIAL_PORT.print(String(rightVectorOld.m_x0) + commaCharStr + String(rightVectorOld.m_y0) + commaCharStr + String(rightVectorOld.m_x1) + commaCharStr + String(rightVectorOld.m_y1));
  SERIAL_PORT.print(semicolonChar);
  SERIAL_PORT.print(String(leftVector.m_x0) + commaCharStr + String(leftVector.m_y0) + commaCharStr + String(leftVector.m_x1) + commaCharStr + String(leftVector.m_y1));
  SERIAL_PORT.print(semicolonChar);
  SERIAL_PORT.print(String(rightVector.m_x0) + commaCharStr + String(rightVector.m_y0) + commaCharStr + String(rightVector.m_x1) + commaCharStr + String(rightVector.m_y1));
  SERIAL_PORT.print(semicolonChar);
  SERIAL_PORT.print(String(leftLine.Ax) + commaCharStr + String(leftLine.By) + commaCharStr + String(leftLine.C));
  SERIAL_PORT.print(semicolonChar);
  SERIAL_PORT.print(String(rightLine.Ax) + commaCharStr + String(rightLine.By) + commaCharStr + String(rightLine.C));
  SERIAL_PORT.print(semicolonChar);
  SERIAL_PORT.print(String(laneMiddleLine.Ax) + commaCharStr + String(laneMiddleLine.By) + commaCharStr + String(laneMiddleLine.C));
  SERIAL_PORT.print(semicolonChar);
  SERIAL_PORT.print(String(purePersuitInfo.carPos.x) + commaCharStr + String(purePersuitInfo.carPos.y));
  SERIAL_PORT.print(semicolonChar);
  SERIAL_PORT.print(String(purePersuitInfo.nextWayPoint.x) + commaCharStr + String(purePersuitInfo.nextWayPoint.y));
  SERIAL_PORT.print(semicolonChar);
  SERIAL_PORT.print(String(purePersuitInfo.steeringAngle));
  SERIAL_PORT.print(semicolonChar);
  SERIAL_PORT.print(String(carAcceleration));
  SERIAL_PORT.print(semicolonChar);
  SERIAL_PORT.print(String(frontObstacleDistance));
  SERIAL_PORT.print(semicolonChar);
  SERIAL_PORT.print(String(purePersuitInfo.lookAheadDistance * CM_PER_VECTOR_UNIT));
  SERIAL_PORT.print(semicolonChar);
  SERIAL_PORT.print(String(carSpeed_));
  SERIAL_PORT.println();
}

/*==============================================================================*/

static void serial2WifiConnect(HardwareSerial &serialPort, String initSequence, String wifiSsid, String wifiPassword, String hostname, int port){
  String commentChar = String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING);
  serialPort.print(initSequence);
  serialPort.println(commentChar + wifiSsid);
  serialPort.println(commentChar + wifiPassword);
  serialPort.println(commentChar + hostname);
  serialPort.println(commentChar + String(port));
}

/*==============================================================================*/

static void printSerial2WifiInfo(String initSequence, String wifiSsid, String wifiPassword, String hostname, int port){
  String commentChar = String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING);
  SERIAL_PORT.print(commentChar + String("WIFI INIT SEQUENCE: ") + initSequence);
  SERIAL_PORT.println(commentChar + String("SSID: ") + wifiSsid);
  SERIAL_PORT.println(commentChar + String("PASSWORD: ") + wifiPassword);
  SERIAL_PORT.println(commentChar + String("HOSTNAME: ") + hostname);
  SERIAL_PORT.println(commentChar + String("PORT: ") + String(port));
}

/*==============================================================================*/

static void HardwareReset(){
  delay(100);
  SCB_AIRCR = 0x05FA0004;
}
/*==============================================================================*/

bool readRecordFromSerial(HardwareSerial& serialPort, String recordTermintor, std::vector<char> &record){
  static std::vector<char> inputBuffer = std::vector<char>();
  static bool terminatorFound = false;

  char tempChar, lastTerminatorCharacter;

  if (serialPort.available() <= 0) {
    //record = std::vector<char>();
    return false;
  }

  if (terminatorFound == true && inputBuffer.size() > 0) {
    terminatorFound = false;
    inputBuffer.clear();
  }
  

  lastTerminatorCharacter = recordTermintor.charAt(recordTermintor.length()-1);

  while (serialPort.available() > 0){
    tempChar = (char)serialPort.read();
    inputBuffer.push_back(tempChar);
    if (tempChar == lastTerminatorCharacter && inputBuffer.size() >= (size_t)recordTermintor.length()) {
      if (memcmp((const void*)recordTermintor.c_str(), (const void*)(inputBuffer.data() + (size_t)((int)inputBuffer.size() - (int)recordTermintor.length())), (size_t)recordTermintor.length()) == 0) {
        terminatorFound = true;
        break;
      }
    }
  }

  if (terminatorFound) {

    if (inputBuffer.size() >= (size_t)recordTermintor.length()) {
      inputBuffer.erase(inputBuffer.begin() + (inputBuffer.size() - (size_t)recordTermintor.length()), inputBuffer.end());
    }
    else {
      // If vector has less than n number of elements,
      // then delete all elements
      inputBuffer.clear();
    //record = std::vector<char>();
    return false;
    }
    // parse the inputBuffer and load the new global variables
    record = inputBuffer;
    record.push_back('\0');
    terminatorFound = false;
    inputBuffer.clear();
    return true;
  }
  record = std::vector<char>();
  return false;
}
/*==============================================================================*/
/*
* str:
* C-string beginning with the representation of a floating-point number.
* endptr:
* Reference to an already allocated object of type char*, whose value is set by the function to the next character in str after the numerical value.
* This parameter can also be a null pointer, in which case it is not used.
*/

float parseNextFloat(char* str, size_t strSize, char variableTerminator, char** endptr, int* success) {
	char* nextTerminator;
	char* pEnd;
	float result = 0;

	nextTerminator = (char*)memchr(str, (int)variableTerminator, strSize);
	if (str == nextTerminator && strSize == 1) {
		if (endptr) {
			*endptr = nextTerminator;
		}
		if (success) {
			*success = 0;
		}

		return 0.0f;
	}
	if (nextTerminator) {
		*nextTerminator = '\0';
	}
	else {
		nextTerminator = str + strSize;
	}
	
	result = (float)strtod_(str, &pEnd);  
  

	if (pEnd != nextTerminator) {
		// handle incomplete parse
		pEnd += 1;
		*success = 0;
		if (endptr) {
			*endptr = pEnd;
		}
	}
	else {
		pEnd += 1;
		*success = 1;
		if (endptr) {
			*endptr = pEnd;
		}
	}
	return result;
}

/*==============================================================================*/

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

  serialPort.println();
}

/*==============================================================================*/

#if ENABLE_SETTINGS_MENU == 1
void settingsMenuRoutine(LiquidCrystal_I2C &lcd_, int left_arrow_btn, int right_arrow_btn, int increment_btn, int decrement_btn) {
  enum LcdMenu {LCDMENU_FIRST_VALUE,
                LCDMENU_MAIN_VIEW,
                LCDMENU_ENABLE_CAR_ENGINE,
                LCDMENU_ENABLE_CAR_STEERING_WHEEL,
                LCDMENU_ENABLE_EMERGENCY_BRAKE,
                LCDMENU_ENABLE_PIXY_VECTOR_APPROXIMATION,
                LCDMENU_ENABLE_DISTANCE_SENSOR1,
                LCDMENU_ENABLE_DISTANCE_SENSOR2,
                LCDMENU_STEERING_WHEEL_ANGLE_OFFSET,
                LCDMENU_MIN_SPEED,
                LCDMENU_MAX_SPEED,
                LCDMENU_LOOKAHEAD_MIN_DISTANCE_CM,
                LCDMENU_LOOKAHEAD_MAX_DISTANCE_CM,
                LCDMENU_EMERGENCY_BREAK_DISTANCE_CM,
                LCDMENU_EMERGENCY_BRAKE_MIN_SPEED,
                LCDMENU_EMERGENCY_BRAKE_DISTANCE_FROM_OBSTACLE_CM,
                LCDMENU_EMERGENCY_BRAKE_ENABLE_DELAY_S,
                LCDMENU_LANE_WIDTH_VECTOR_UNIT_REAL,
                LCDMENU_BLACK_COLOR_TRESHOLD,
                LCDMENU_CALIBRATION_VIEW,
                LCDMENU_LAST_VALUE};
  
  static int lcdMenuIndex = ((int)LCDMENU_FIRST_VALUE) + 1;
  static int leftArrowButtonState=LOW;
  static int rightArrowButtonState=LOW;
  static float lcd_print_timeont = 0.0f;
  int incrementButton=LOW;
  int decrementButton=LOW;
  int leftArrowButtonPrevState, rightArrowButtonPrevState;

  leftArrowButtonPrevState = leftArrowButtonState;
  rightArrowButtonPrevState = rightArrowButtonState;

  leftArrowButtonState = digitalRead(left_arrow_btn);
  rightArrowButtonState = digitalRead(right_arrow_btn);
  incrementButton = digitalRead(increment_btn);
  decrementButton = digitalRead(decrement_btn);

  if (!(leftArrowButtonPrevState == HIGH && leftArrowButtonState == HIGH) && !(rightArrowButtonPrevState == HIGH && rightArrowButtonState == HIGH)) {
    if (rightArrowButtonState == HIGH && lcdMenuIndex >= ((int)LCDMENU_LAST_VALUE) - 1) {
      lcdMenuIndex = ((int)LCDMENU_FIRST_VALUE) + 1;
    } else if (rightArrowButtonState == HIGH) {
      (int)lcdMenuIndex++;
    }

    if (leftArrowButtonState == HIGH && lcdMenuIndex <= ((int)LCDMENU_FIRST_VALUE) + 1) {
      lcdMenuIndex = ((int)LCDMENU_LAST_VALUE) - 1;
    } else if (leftArrowButtonState == HIGH) {
      (int)lcdMenuIndex--;
    }
  }
  ///lcd_print_timeont -= fabsf(loop_time_ms);
  #if RACE_MODE == 1
    if (ENABLE_CAR_ENGINE == 0)
    {
  #endif
  if (leftArrowButtonState == HIGH || rightArrowButtonState == HIGH || incrementButton == HIGH || decrementButton == HIGH /*|| lcd_print_timeont <= 0.0f*/) {
    //lcd_print_timeont = 500.0f;
    lcd_.clear();
    switch (lcdMenuIndex) {
      case LCDMENU_STEERING_WHEEL_ANGLE_OFFSET:
        if (incrementButton == HIGH) {
          STEERING_WHEEL_ANGLE_OFFSET += 0.1f;
        } else if (decrementButton == HIGH) {
          STEERING_WHEEL_ANGLE_OFFSET -= 0.1f;
        }
        lcd_.setCursor(0, 0);
        lcd_.print("STR_WHEEL_OFST");
        lcd_.setCursor(0, 1);
        lcd_.print(STEERING_WHEEL_ANGLE_OFFSET);

      break;
      case LCDMENU_MAIN_VIEW:
        lcd_.setCursor(0, 0);
        lcd_.print("Loop ms: ");
        lcd_.print(loop_time_ms);

        lcd_.setCursor(0, 1);
        lcd_.print("Timer s: ");
        lcd_.print((time_passed_ms / 1000.0f));
      break;

      case LCDMENU_ENABLE_CAR_ENGINE:
        if (incrementButton == HIGH) {
          ENABLE_CAR_ENGINE = 1;
        }
        else if (decrementButton == HIGH) {
          ENABLE_CAR_ENGINE = 0;
        }
        lcd_.setCursor(0, 0);
        lcd_.print("ENABLE_ENGINE");
        lcd_.setCursor(0, 1);
        if (ENABLE_CAR_ENGINE != 0) {
          lcd_.print("Enabled");
        }
        else{
          lcd_.print("Disabled");
        }
        break;
      
      case LCDMENU_ENABLE_CAR_STEERING_WHEEL:
        if (incrementButton == HIGH) {
          ENABLE_CAR_STEERING_WHEEL = 1;
        }
        else if (decrementButton == HIGH) {
          ENABLE_CAR_STEERING_WHEEL = 0;
        }
        lcd_.setCursor(0, 0);
        lcd_.print("ENABLE_STEERING");
        lcd_.setCursor(0, 1);
        if (ENABLE_CAR_STEERING_WHEEL != 0) {
          lcd_.print("Enabled");
        }
        else{
          lcd_.print("Disabled");
        }
        break;
      
      case LCDMENU_ENABLE_EMERGENCY_BRAKE:
        if (incrementButton == HIGH) {
          ENABLE_EMERGENCY_BRAKE = 1;
        }
        else if (decrementButton == HIGH) {
          ENABLE_EMERGENCY_BRAKE = 0;
        }
        lcd_.setCursor(0, 0);
        lcd_.print("ENABLE_EMERG_BRK");
        lcd_.setCursor(0, 1);
        if (ENABLE_EMERGENCY_BRAKE != 0) {
          lcd_.print("Enabled");
        }
        else{
          lcd_.print("Disabled");
        }
      break;

      case LCDMENU_ENABLE_PIXY_VECTOR_APPROXIMATION:
        if (incrementButton == HIGH) {
          ENABLE_PIXY_VECTOR_APPROXIMATION_SOFT = 1;
        }
        else if (decrementButton == HIGH) {
          ENABLE_PIXY_VECTOR_APPROXIMATION_SOFT = 0;
        }
        lcd_.setCursor(0, 0);
        lcd_.print("ENABLE_VEC_APRX");
        lcd_.setCursor(0, 1);
        if (ENABLE_PIXY_VECTOR_APPROXIMATION_SOFT != 0) {
          lcd_.print("Enabled");
        }
        else{
          lcd_.print("Disabled");
        }
      break;

      case LCDMENU_ENABLE_DISTANCE_SENSOR1:
        if (incrementButton == HIGH) {
          ENABLE_DISTANCE_SENSOR1_SOFT = 1;
        }
        else if (decrementButton == HIGH) {
          ENABLE_DISTANCE_SENSOR1_SOFT = 0;
        }
        lcd_.setCursor(0, 0);
        lcd_.print("ENABLE_DIST_SNS1");
        lcd_.setCursor(0, 1);
        if (ENABLE_DISTANCE_SENSOR1_SOFT != 0) {
          lcd_.print("Enabled");
        }
        else{
          lcd_.print("Disabled");
        }
      break;

      case LCDMENU_ENABLE_DISTANCE_SENSOR2:
        if (incrementButton == HIGH) {
          ENABLE_DISTANCE_SENSOR2_SOFT = 1;
        }
        else if (decrementButton == HIGH) {
          ENABLE_DISTANCE_SENSOR2_SOFT = 0;
        }
        lcd_.setCursor(0, 0);
        lcd_.print("ENABLE_DIST_SNS2");
        lcd_.setCursor(0, 1);
        if (ENABLE_DISTANCE_SENSOR2_SOFT != 0) {
          lcd_.print("Enabled");
        }
        else{
          lcd_.print("Disabled");
        }
      break;

      case LCDMENU_MIN_SPEED:
        if (incrementButton == HIGH) {
          MIN_SPEED += 0.5f;
        } else if (decrementButton == HIGH) {
          MIN_SPEED -= 0.5f;
        }
        MIN_SPEED = MAX(MIN_SPEED, 0.0f);
        lcd_.setCursor(0, 0);
        lcd_.print("MIN_SPEED");
        lcd_.setCursor(0, 1);
        lcd_.print(MIN_SPEED);
        break;

      case LCDMENU_MAX_SPEED:
        if (incrementButton == HIGH) {
          MAX_SPEED += 0.5f;
        } else if (decrementButton == HIGH) {
          MAX_SPEED -= 0.5f;
        }
        MAX_SPEED = MAX(MAX_SPEED, 0.0f);
        lcd_.setCursor(0, 0);
        lcd_.print("MAX_SPEED");
        lcd_.setCursor(0, 1);
        lcd_.print(MAX_SPEED);
        break;

      case LCDMENU_LOOKAHEAD_MIN_DISTANCE_CM:
        if (incrementButton == HIGH) {
          LOOKAHEAD_MIN_DISTANCE_CM += 0.5f;
        } else if (decrementButton == HIGH) {
          LOOKAHEAD_MIN_DISTANCE_CM -= 0.5f;
        }
        LOOKAHEAD_MIN_DISTANCE_CM = MAX(LOOKAHEAD_MIN_DISTANCE_CM, 0.0f);
        lcd_.setCursor(0, 0);
        lcd_.print("LOOKAHEAD_MIN");
        lcd_.setCursor(0, 1);
        lcd_.print(LOOKAHEAD_MIN_DISTANCE_CM);
        break;

      case LCDMENU_LOOKAHEAD_MAX_DISTANCE_CM:
        if (incrementButton == HIGH) {
          LOOKAHEAD_MAX_DISTANCE_CM += 0.5f;
        } else if (decrementButton == HIGH) {
          LOOKAHEAD_MAX_DISTANCE_CM -= 0.5f;
        }
        LOOKAHEAD_MAX_DISTANCE_CM = MAX(LOOKAHEAD_MAX_DISTANCE_CM, 0.0f);
        lcd_.setCursor(0, 0);
        lcd_.print("LOOKAHEAD_MAX");
        lcd_.setCursor(0, 1);
        lcd_.print(LOOKAHEAD_MAX_DISTANCE_CM);
        break;
      
      case LCDMENU_EMERGENCY_BREAK_DISTANCE_CM:
        if (incrementButton == HIGH) {
          EMERGENCY_BREAK_DISTANCE_CM += 0.5f;
        } else if (decrementButton == HIGH) {
          EMERGENCY_BREAK_DISTANCE_CM -= 0.5f;
        }
        EMERGENCY_BREAK_DISTANCE_CM = MAX(EMERGENCY_BREAK_DISTANCE_CM, 0.0f);
        lcd_.setCursor(0, 0);
        lcd_.print("EMER_BRK_DIST_CM");
        lcd_.setCursor(0, 1);
        lcd_.print(EMERGENCY_BREAK_DISTANCE_CM);
        break;
      
      case LCDMENU_EMERGENCY_BRAKE_DISTANCE_FROM_OBSTACLE_CM:
        if (incrementButton == HIGH) {
          EMERGENCY_BREAK_MAX_DISTANCE_FROM_OBSTACLE_CM += 0.5f;
        } else if (decrementButton == HIGH) {
          EMERGENCY_BREAK_MAX_DISTANCE_FROM_OBSTACLE_CM -= 0.5f;
        }
        EMERGENCY_BREAK_MAX_DISTANCE_FROM_OBSTACLE_CM = MAX(EMERGENCY_BREAK_MAX_DISTANCE_FROM_OBSTACLE_CM, 0.0f);
        lcd_.setCursor(0, 0);
        lcd_.print("EMR_BR_DIST_OBST");
        lcd_.setCursor(0, 1);
        lcd_.print(EMERGENCY_BREAK_MAX_DISTANCE_FROM_OBSTACLE_CM);
      break;

      case LCDMENU_EMERGENCY_BRAKE_ENABLE_DELAY_S:
        if (incrementButton == HIGH) {
          EMERGENCY_BRAKE_ENABLE_DELAY_S += 0.5f;
        } else if (decrementButton == HIGH) {
          EMERGENCY_BRAKE_ENABLE_DELAY_S -= 0.5f;
        }
        EMERGENCY_BRAKE_ENABLE_DELAY_S = MAX(EMERGENCY_BRAKE_ENABLE_DELAY_S, 0.0f);
        lcd_.setCursor(0, 0);
        lcd_.print("EMER_BRK_DELY_S");
        lcd_.setCursor(0, 1);
        lcd_.print(EMERGENCY_BRAKE_ENABLE_DELAY_S);
        break;
      
      case LCDMENU_LANE_WIDTH_VECTOR_UNIT_REAL:
        if (incrementButton == HIGH) {
          LANE_WIDTH_VECTOR_UNIT_REAL += 0.5f;
        } else if (decrementButton == HIGH) {
          LANE_WIDTH_VECTOR_UNIT_REAL -= 0.5f;
        }
        LANE_WIDTH_VECTOR_UNIT_REAL = MAX(LANE_WIDTH_VECTOR_UNIT_REAL, 0.0f);
        lcd_.setCursor(0, 0);
        lcd_.print("LANE_W_VECT_UNIT");
        lcd_.setCursor(0, 1);
        lcd_.print(LANE_WIDTH_VECTOR_UNIT_REAL);
        break;

      case LCDMENU_EMERGENCY_BRAKE_MIN_SPEED:
        if (incrementButton == HIGH) {
          EMERGENCY_BRAKE_MIN_SPEED += 0.5f;
        } else if (decrementButton == HIGH) {
          EMERGENCY_BRAKE_MIN_SPEED -= 0.5f;
        }
        EMERGENCY_BRAKE_MIN_SPEED = MAX(EMERGENCY_BRAKE_MIN_SPEED, 0.0f);
        lcd_.setCursor(0, 0);
        lcd_.print("EMER_BRK_MIN_SPD");
        lcd_.setCursor(0, 1);
        lcd_.print(EMERGENCY_BRAKE_MIN_SPEED);
        break;
      
      case LCDMENU_BLACK_COLOR_TRESHOLD:
        if (incrementButton == HIGH) {
          BLACK_COLOR_TRESHOLD += 0.01f;
        } else if (decrementButton == HIGH) {
          BLACK_COLOR_TRESHOLD -= 0.01f;
        }
        BLACK_COLOR_TRESHOLD = MAX(BLACK_COLOR_TRESHOLD, 0.0f);
        lcd_.setCursor(0, 0);
        lcd_.print("BLACK_TRESHOLD");
        lcd_.setCursor(0, 1);
        lcd_.print(BLACK_COLOR_TRESHOLD);
        break;

      case LCDMENU_CALIBRATION_VIEW:
        LineABC upper_line, lower_line, middle_line;
        IntersectionLines upper_intersection, lower_intersection, left_lane_line_intersection, right_lane_line_intersection;
        float lane_width_;
        
        upper_line = xAxisABC();
        upper_line.C = -IMAGE_MAX_Y;
        lower_line = xAxisABC();
        upper_intersection = intersectionLinesABC(middle_lane_line, upper_line);
        lower_intersection = intersectionLinesABC(middle_lane_line, lower_line);

        if (upper_intersection.info != 0) {
          lcd_.setCursor(0, 0);
          lcd_.print("inf");
          lcd_.setCursor(0, 1);
          lcd_.print("inf");
        }
        else{
          lcd_.setCursor(0, 0);
          lcd_.print(upper_intersection.point.x - SCREEN_CENTER_X, 2);
          lcd_.setCursor(0, 1);
          lcd_.print(lower_intersection.point.x - SCREEN_CENTER_X, 2);
        }

        middle_line = xAxisABC();
        middle_line.C = -SCREEN_CENTER_Y;

        left_lane_line_intersection = intersectionLinesABC(left_lane_line, middle_line);
        right_lane_line_intersection = intersectionLinesABC(right_lane_line, middle_line);

        lcd_.setCursor(8, 0);
        lcd_.print("LaneWdth");
        lcd_.setCursor(8, 1);
        if (left_lane_line_intersection.info == 0 && right_lane_line_intersection.info == 0) {
          lane_width_ = euclidianDistance(left_lane_line_intersection.point, right_lane_line_intersection.point);
          lcd_.print(lane_width_);
        }
        else{
          lcd_.print("NO_LINE");
        }
        break;

      default:
        break;
    }
  }
  #if RACE_MODE == 1
  }
  #endif
}
#endif

#endif