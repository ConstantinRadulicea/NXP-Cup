// https://colorizer.org/

#include <Arduino.h>
#include <SPI.h>
#include "SteeringWheel.h"
#include "pixy2_libs/host/arduino/libraries/Pixy2/Pixy2.h"
#include "PurePursuitGeometry.h"
#include "VectorsProcessing.h"
#include "aproximatePixyVector.h"
#include <PWMServo.h>

#define ENABLE_SERIAL_PRINT 0
#define ENABLE_STEERING_SERVO 1
#define ENABLE_DRIVERMOTOR 1
#define ENABLE_PIXY_VECTOR_APPROXIMATION 1

#define DEBUG_MODE_STANDSTILL 0
#define DEBUG_MODE_IN_MOTION 0




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
#define LANE_WIDTH_VECTOR_UNIT 53.0f
#define LOOKAHEAD_MIN_DISTANCE_CM 15.0f
#define LOOKAHEAD_MAX_DISTANCE_CM 30.0f
#define CAR_LENGTH_CM 17.5
#define BLACK_COLOR_TRESHOLD 0.2f // 0=black, 1=white

#define VECTOR_UNIT_PER_CM (float)((float)LANE_WIDTH_VECTOR_UNIT / (float)LANE_WIDTH_CM)   // CM * VECTOR_UNIT_PER_CM = VECTOR_UNIT
#define CM_PER_VECTOR_UNIT (float)((float)LANE_WIDTH_CM / (float)LANE_WIDTH_VECTOR_UNIT)   // VECTOR_UNIT_PER_CM * CM = CM

#define STEERING_SERVO_ANGLE_MIDDLE     90    // 90 middle
#define STEERING_SERVO_ANGLE_MAX_RIGHT  0    // 38 max right
#define STEERING_SERVO_ANGLE_MAX_LEFT   180   // 135 max left
#define STEERING_SERVO_MAX_ANGLE MAX(abs(STEERING_SERVO_ANGLE_MIDDLE - STEERING_SERVO_ANGLE_MAX_RIGHT), abs(STEERING_SERVO_ANGLE_MIDDLE - STEERING_SERVO_ANGLE_MAX_LEFT))

#define MIN_SPEED (int)95
#define MAX_SPEED (int)108



SteeringWheel steeringWheel(STEERING_SERVO_ANGLE_MAX_LEFT, STEERING_SERVO_ANGLE_MIDDLE, STEERING_SERVO_ANGLE_MAX_RIGHT, (unsigned int)0);
PWMServo driverMotor;
VectorsProcessing vectorsProcessing;
Pixy2 pixy;
int8_t res;
  

void setup() {
    // serial Initialization
    #if ENABLE_SERIAL_PRINT == 1
      Serial.begin(115200);
      while (!Serial){
        delay(100);
      }
    #endif

    // Initialization and attachment of the servo and motor
    
    #if ENABLE_STEERING_SERVO == 1
      steeringWheel.attach(STEERING_SERVO_PIN);
      steeringWheel.setSteeringAngleDeg(0);
    #endif

    #if ENABLE_DRIVERMOTOR == 1
      driverMotor.attach(DRIVER_MOTOR_PIN, 1000, 2000);
      driverMotor.write(90);
    #endif
    
    // we must initialize the pixy object
    res = pixy.init();
    #if ENABLE_SERIAL_PRINT == 1
      Serial.println("% pixy.init() = " + String(res));
    #endif
    
    // Getting the RGB pixel values requires the 'video' program
    res = pixy.changeProg("line");
    #if ENABLE_SERIAL_PRINT == 1
      Serial.println("% pixy.changeProg(line) = " + String(res));
    #endif
    #if ENABLE_DRIVERMOTOR == 1
      delay(10000);
    #endif
}

void pixyErrorRecovery(){
  while (pixy.init() != PIXY_RESULT_OK) {
    delay(10);
  }
}

void printDataToSerial(Vector leftVectorOld, Vector rightVectorOld, Vector leftVector, Vector rightVector, LineABC leftLine, LineABC rightLine, LineABC laneMiddleLine, PurePersuitInfo purePersuitInfo, float carAcceleration){
  Serial.print(String(leftVectorOld.m_x0) + ',' + String(leftVectorOld.m_y0) + ',' + String(leftVectorOld.m_x1) + ',' + String(leftVectorOld.m_y1));
  Serial.print(';');
  Serial.print(String(rightVectorOld.m_x0) + ',' + String(rightVectorOld.m_y0) + ',' + String(rightVectorOld.m_x1) + ',' + String(rightVectorOld.m_y1));
  Serial.print(';');
  Serial.print(String(leftVector.m_x0) + ',' + String(leftVector.m_y0) + ',' + String(leftVector.m_x1) + ',' + String(leftVector.m_y1));
  Serial.print(';');
  Serial.print(String(rightVector.m_x0) + ',' + String(rightVector.m_y0) + ',' + String(rightVector.m_x1) + ',' + String(rightVector.m_y1));
  Serial.print(';');
  Serial.print(String(leftLine.Ax) + ',' + String(leftLine.By) + ',' + String(leftLine.C));
  Serial.print(';');
  Serial.print(String(rightLine.Ax) + ',' + String(rightLine.By) + ',' + String(rightLine.C));
  Serial.print(';');
  Serial.print(String(laneMiddleLine.Ax) + ',' + String(laneMiddleLine.By) + ',' + String(laneMiddleLine.C));
  Serial.print(';');
  Serial.print(String(purePersuitInfo.carPos.x) + ',' + String(purePersuitInfo.carPos.y));
  Serial.print(';');
  Serial.print(String(purePersuitInfo.nextWayPoint.x) + ',' + String(purePersuitInfo.nextWayPoint.y));
  Serial.print(';');
  Serial.print(String(purePersuitInfo.steeringAngle));
  Serial.print(';');
  Serial.print(String(carAcceleration));
  Serial.println();
}

void loop() {
  int i, loop_iter_timeout_vector = 0;
  uint32_t timeStart, noVectorDetectedIterationCount;;
  LineABC laneMiddleLine, mirrorLine;
  Vector vec, leftVectorOld, rightVectorOld;
  PurePersuitInfo purePersuitInfo;
  Point2D carPosition;
  float carLength, laneWidth, lookAheadDistance, carSpeed;
  
  carSpeed = 0.0f;
  timeStart = 0;
  noVectorDetectedIterationCount = 0;

  mirrorLine = xAxisABC();
  mirrorLine.C = -(((float)IMAGE_MAX_Y) / 2.0f);

  carPosition.x = (float)SCREEN_CENTER_X;
  carPosition.y = 0.0f;

  laneWidth = (float)LANE_WIDTH_VECTOR_UNIT;
  carLength = (float)CAR_LENGTH_CM * (float)VECTOR_UNIT_PER_CM;
  
  lookAheadDistance = (float)LOOKAHEAD_MIN_DISTANCE_CM * (float)VECTOR_UNIT_PER_CM;
  
  vectorsProcessing.setCarPosition(carPosition);
  vectorsProcessing.setLaneWidth(laneWidth);
  vectorsProcessing.setMinXaxisAngle((1.0f / 180.0f) * M_PI);
  while (1)
  {
    timeStart = millis();
    vectorsProcessing.clear();
    if(pixy.line.getAllFeatures(LINE_VECTOR) >= (int8_t)0){
      
      if (pixy.line.numVectors > 0){
        loop_iter_timeout_vector = 0;
      }
      else{
        loop_iter_timeout_vector++;
      }
      
      for (i=0; i < pixy.line.numVectors; i++)
      {
        vec = pixy.line.vectors[i];
        vec = VectorsProcessing::mirrorVector(mirrorLine, vec);
        vectorsProcessing.addVector(vec);
      }
      leftVectorOld = vectorsProcessing.getLeftVector();
      rightVectorOld = vectorsProcessing.getRightVector();

      #if ENABLE_PIXY_VECTOR_APPROXIMATION == 1
      if (((int)vectorsProcessing.isVectorValid(rightVectorOld) + (int)vectorsProcessing.isVectorValid(leftVectorOld))==1){
        carSpeed = (float)MIN_SPEED;
        i=0;
        while (pixy.changeProg("video") != PIXY_RESULT_OK)
        {
          i++;
          #if ENABLE_SERIAL_PRINT == 1
            Serial.println("% " + String(i) + " ERROR: pixy.changeProg(\"video\")");
          #endif
          if (i >= 5)
          {
            #if ENABLE_DRIVERMOTOR == 1
              driverMotor.write(90);
            #endif
            carSpeed = 90.0f;
            pixyErrorRecovery();
          }
        }
        delay(40);

        vec = VectorsProcessing::mirrorVector(mirrorLine, leftVectorOld);
        approximatePixyVectorVector(pixy, vec, BLACK_COLOR_TRESHOLD, mirrorImage(mirrorLine, carPosition));
        vec = VectorsProcessing::mirrorVector(mirrorLine, vec);
        vectorsProcessing.setLeftVector(vec);

        vec = VectorsProcessing::mirrorVector(mirrorLine, rightVectorOld);
        approximatePixyVectorVector(pixy, vec, BLACK_COLOR_TRESHOLD, mirrorImage(mirrorLine, carPosition));
        vec = VectorsProcessing::mirrorVector(mirrorLine, vec);
        vectorsProcessing.setRightVector(vec);
        
        while (pixy.changeProg("line") != PIXY_RESULT_OK) {
          i++;
          #if ENABLE_SERIAL_PRINT == 1
            Serial.println("% " + String(i) + " ERROR: pixy.changeProg(\"line\")");
          #endif
          if (i >= 5)
          {
            #if ENABLE_DRIVERMOTOR == 1
              driverMotor.write(90);
            #endif
            carSpeed = 90.0f;
            pixyErrorRecovery();
          }
        }
        delay(40);
      }
      #endif

      laneMiddleLine = vectorsProcessing.getMiddleLine();
      purePersuitInfo = purePursuitComputeABC(carPosition, laneMiddleLine, carLength, lookAheadDistance);

      if (noVectorDetectedIterationCount > 15)
      {
        carSpeed = (float)MIN_SPEED;
        #if ENABLE_DRIVERMOTOR == 1
          driverMotor.write(carSpeed);
        #endif
      }
      else{
        carSpeed = MIN((abs((float)STEERING_SERVO_MAX_ANGLE - (float)abs(purePersuitInfo.steeringAngle * (180.0f / M_PI))) / (float)STEERING_SERVO_MAX_ANGLE) * (float)(MAX_SPEED - 90), (float)MAX_SPEED) + 90.0f;
        carSpeed = MAX((float)carSpeed, (float)MIN_SPEED);
      }
      

    }
    else{
      loop_iter_timeout_vector++;
      #if ENABLE_SERIAL_PRINT == 1
        Serial.println("% " + String(loop_iter_timeout_vector) + " ERROR: pixy.line.getAllFeatures(LINE_VECTOR)");
      #endif
      if (loop_iter_timeout_vector >= 5)
      {
        #if ENABLE_DRIVERMOTOR == 1
          driverMotor.write(90);
        #endif
        carSpeed = 90.0f;
        pixyErrorRecovery();
      }
    }
    
    #if ENABLE_SERIAL_PRINT == 1
        printDataToSerial(leftVectorOld, rightVectorOld, vectorsProcessing.getLeftVector(), vectorsProcessing.getRightVector(), VectorsProcessing::vectorToLineABC(vectorsProcessing.getLeftVector()), VectorsProcessing::vectorToLineABC(vectorsProcessing.getRightVector()), laneMiddleLine, purePersuitInfo, (carSpeed - 90.0f) / (float)(MAX_SPEED - 90));
    #endif
    
    
    #if ENABLE_STEERING_SERVO == 1
      steeringWheel.setSteeringAngleDeg(purePersuitInfo.steeringAngle * (180.0f / M_PI));
    #endif

    #if ENABLE_DRIVERMOTOR == 1
      driverMotor.write((int)carSpeed);
    #endif
    
    #if ENABLE_SERIAL_PRINT == 1
      Serial.println("% LoopTime: " + String(millis() - timeStart) + " ms");
    #endif
  }
}

