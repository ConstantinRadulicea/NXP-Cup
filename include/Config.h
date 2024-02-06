//All defines and Includes

#ifndef __CONFIG_H__
#define __CONFIG_H__

#define ENABLE_ARDUINO 0

#include <Arduino.h>
#include <SPI.h>
#include "SteeringWheel.h"
#include "pixy2_libs/host/arduino/libraries/Pixy2/Pixy2.h"
#include "PurePursuitGeometry.h"
#include "VectorsProcessing.h"
#include "aproximatePixyVector.h"

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

#define DEBUG_MODE_STANDSTILL 0
#define DEBUG_MODE_IN_MOTION 1

#define DEBUG_WIFI_SSID "Off Limits2"
#define DEBUG_WIFI_PASSWORD "J7s2tzvzKzva"
#define DEBUG_HOST_IPADDRESS "192.168.79.243"
#define DEBUG_HOST_PORT 6789
#define DEBUG_WIFI_INIT_SEQUENCE "%SERIAL2WIFI\r\n"
#define ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING '%'



#if ENABLE_WIRELESS_DEBUG == 1
  #define SERIAL Serial1
#else
  #define SERIAL Serial
#endif

#if DEBUG_MODE_IN_MOTION == 1
  #define ENABLE_SERIAL_PRINT 1
  #define ENABLE_STEERING_SERVO 1
  #define ENABLE_DRIVERMOTOR 1
#endif

#if DEBUG_MODE_STANDSTILL == 1
  #define ENABLE_SERIAL_PRINT 1
  #define ENABLE_STEERING_SERVO 0
  #define ENABLE_DRIVERMOTOR 0
#endif




#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

#define STEERING_SERVO_PIN  3
#define DRIVER_MOTOR_PIN  9

#define IMAGE_MAX_X 78.0f
#define IMAGE_MAX_Y 51.0f
#define SCREEN_CENTER_X ((float)IMAGE_MAX_X / 2.0f)

#define LANE_WIDTH_CM 53.5f
#define LANE_WIDTH_VECTOR_UNIT_REAL 39.0f

#define LOOKAHEAD_MIN_DISTANCE_CM 15.0f
#define LOOKAHEAD_MAX_DISTANCE_CM 30.0f
#define CAR_LENGTH_CM 17.5
#define BLACK_COLOR_TRESHOLD 0.2f // 0=black, 1=white

#define VECTOR_UNIT_PER_CM (float)((float)LANE_WIDTH_VECTOR_UNIT_REAL / (float)LANE_WIDTH_CM)   // CM * VECTOR_UNIT_PER_CM = VECTOR_UNIT
#define CM_PER_VECTOR_UNIT (float)((float)LANE_WIDTH_CM / (float)LANE_WIDTH_VECTOR_UNIT_REAL)   // VECTOR_UNIT_PER_CM * CM = CM
#define RADIANS_PER_DEGREE (float)((float)M_PI / 180.0f) // DEGREE * RADIAN_PER_DEGREE = RADIAN
#define DEGREES_PER_RADIAN (float)(180.0f / (float)M_PI) // RADIAN * DEGREE_PER_RADIAN = DEGREE

#define LANE_WIDTH_VECTOR_UNIT (float)(LANE_WIDTH_VECTOR_UNIT_REAL + ((5.0f * VECTOR_UNIT_PER_CM)*2.0f))

#define STEERING_SERVO_ANGLE_MIDDLE     90    // 90 middle
#define STEERING_SERVO_ANGLE_MAX_RIGHT  0    // 38 max right
#define STEERING_SERVO_ANGLE_MAX_LEFT   180   // 135 max left
#define STEERING_SERVO_MAX_ANGLE MAX(abs(STEERING_SERVO_ANGLE_MIDDLE - STEERING_SERVO_ANGLE_MAX_RIGHT), abs(STEERING_SERVO_ANGLE_MIDDLE - STEERING_SERVO_ANGLE_MAX_LEFT))

#define MIN_SPEED (int)95
#define MAX_SPEED (int)100
#define STANDSTILL_SPEED (int)90

/*====================================================================================================================================*/

static void printDataToSerial(Vector leftVectorOld, Vector rightVectorOld, Vector leftVector, Vector rightVector, LineABC leftLine, LineABC rightLine, LineABC laneMiddleLine, PurePersuitInfo purePersuitInfo, float carAcceleration){
  SERIAL.print(String(leftVectorOld.m_x0) + String(',') + String(leftVectorOld.m_y0) + String(',') + String(leftVectorOld.m_x1) + String(',') + String(leftVectorOld.m_y1));
  SERIAL.print(';');
  SERIAL.print(String(rightVectorOld.m_x0) + String(',') + String(rightVectorOld.m_y0) + String(',') + String(rightVectorOld.m_x1) + String(',') + String(rightVectorOld.m_y1));
  SERIAL.print(';');
  SERIAL.print(String(leftVector.m_x0) + String(',') + String(leftVector.m_y0) + String(',') + String(leftVector.m_x1) + String(',') + String(leftVector.m_y1));
  SERIAL.print(';');
  SERIAL.print(String(rightVector.m_x0) + String(',') + String(rightVector.m_y0) + String(',') + String(rightVector.m_x1) + String(',') + String(rightVector.m_y1));
  SERIAL.print(';');
  SERIAL.print(String(leftLine.Ax) + String(',') + String(leftLine.By) + String(',') + String(leftLine.C));
  SERIAL.print(';');
  SERIAL.print(String(rightLine.Ax) + String(',') + String(rightLine.By) + String(',') + String(rightLine.C));
  SERIAL.print(';');
  SERIAL.print(String(laneMiddleLine.Ax) + String(',') + String(laneMiddleLine.By) + String(',') + String(laneMiddleLine.C));
  SERIAL.print(';');
  SERIAL.print(String(purePersuitInfo.carPos.x) + String(',') + String(purePersuitInfo.carPos.y));
  SERIAL.print(';');
  SERIAL.print(String(purePersuitInfo.nextWayPoint.x) + String(',') + String(purePersuitInfo.nextWayPoint.y));
  SERIAL.print(';');
  SERIAL.print(String(purePersuitInfo.steeringAngle));
  SERIAL.print(';');
  SERIAL.print(String(carAcceleration));
  SERIAL.println();
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
  SERIAL.print(commentChar + String("WIFI INIT SEQUENCE: ") + initSequence);
  SERIAL.println(commentChar + String("SSID: ") + wifiSsid);
  SERIAL.println(commentChar + String("PASSWORD: ") + wifiPassword);
  SERIAL.println(commentChar + String("HOSTNAME: ") + hostname);
  SERIAL.println(commentChar + String("PORT: ") + String(port));
}

#endif

