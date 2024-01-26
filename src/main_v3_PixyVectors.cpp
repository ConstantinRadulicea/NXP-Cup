// https://colorizer.org/

#include <Arduino.h>
#include <SPI.h>
#include "SteeringWheel.h"
#include "pixy2_libs/host/arduino/libraries/Pixy2/Pixy2.h"
#include "PurePursuitGeometry.h"
#include "VectorsProcessing.h"
#include "aproximatePixyVector.h"
#include <PWMServo.h>

#define ENABLE_PIXY_VECTOR_APPROXIMATION 1

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

#define MIN_SPEED (int)100
#define MAX_SPEED (int)150
#define SCREEN_CENTER_X (int)(78.0f / 2.0f)
#define IMAGE_MAX_X 78
#define IMAGE_MAX_Y 51
#define LANE_WIDTH_PIXELS 50
#define BLACK_COLOR_TRESHOLD 0.2f // 0=black, 1=white

#define STEERING_SERVO_ANGLE_MIDDLE     90    // 90 middle
#define STEERING_SERVO_ANGLE_MAX_RIGHT  0    // 38 max right
#define STEERING_SERVO_ANGLE_MAX_LEFT   180   // 135 max left
#define STEERING_SERVO_MAX_ANGLE MAX(abs(STEERING_SERVO_ANGLE_MIDDLE - STEERING_SERVO_ANGLE_MAX_RIGHT), abs(STEERING_SERVO_ANGLE_MIDDLE - STEERING_SERVO_ANGLE_MAX_LEFT))

#define STEERING_SERVO_PIN  3
#define DRIVER_MOTOR_PIN  9

SteeringWheel steeringWheel(STEERING_SERVO_ANGLE_MAX_LEFT, STEERING_SERVO_ANGLE_MIDDLE, STEERING_SERVO_ANGLE_MAX_RIGHT, (unsigned int)0);
PWMServo driverMotor;
VectorsProcessing vectorsProcessing;
Pixy2 pixy;
int8_t res;


void setup() {
    // serial Initialization
    do{
      Serial.begin(115200);
      delay(100);
    }while (!Serial);

    // Initialization and attachment of the servo and motor
    steeringWheel.attach(STEERING_SERVO_PIN);
    driverMotor.attach(DRIVER_MOTOR_PIN, 1000, 2000);
    driverMotor.write(90);
    steeringWheel.setSteeringAngleDeg(0);
    
    // we must initialize the pixy object
    res = pixy.init();
    Serial.println("% pixy.init() = " + String(res));
    // Getting the RGB pixel values requires the 'video' program
    res = pixy.changeProg("line");
    Serial.println("% pixy.changeProg(line) = " + String(res));
    delay(10000);
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
  int i;
  unsigned int timeStart;
  LineABC laneMiddleLine, mirrorLine;
  Vector vec, leftVectorOld, rightVectorOld;
  PurePersuitInfo purePersuitInfo;
  Point2D carPosition;
  float carLength, laneWidth, lookAheadDistance, carSpeed;

  mirrorLine = xAxisABC();
  mirrorLine.C = -(((float)IMAGE_MAX_Y) / 2.0f);

  carPosition.x = (float)SCREEN_CENTER_X;
  carPosition.y = 0.0f;

  laneWidth = (float)LANE_WIDTH_PIXELS;
  carLength = (17.5f / 53.5f) * laneWidth;
  
  lookAheadDistance = (20.0f / 53.5f) * laneWidth;
  
  vectorsProcessing.setCarPosition(carPosition);
  vectorsProcessing.setLaneWidth(laneWidth);
  vectorsProcessing.setMinXaxisAngle((1.0f / 180.0f) * M_PI);
  while (1)
  {
    timeStart = millis();
    vectorsProcessing.clear();
    if(pixy.line.getAllFeatures(LINE_VECTOR) >= (int8_t)0){
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
        while (pixy.changeProg("video") != PIXY_RESULT_OK);
        delay(30);

        vec = VectorsProcessing::mirrorVector(mirrorLine, leftVectorOld);
        approximatePixyVectorVector(pixy, vec, BLACK_COLOR_TRESHOLD);
        vec = VectorsProcessing::mirrorVector(mirrorLine, vec);
        vectorsProcessing.setLeftVector(vec);

        vec = VectorsProcessing::mirrorVector(mirrorLine, rightVectorOld);
        approximatePixyVectorVector(pixy, vec, BLACK_COLOR_TRESHOLD);
        vec = VectorsProcessing::mirrorVector(mirrorLine, vec);
        vectorsProcessing.setRightVector(vec);
        
        while (pixy.changeProg("line") != PIXY_RESULT_OK);
        delay(10);
      }
      #endif
    }
    laneMiddleLine = vectorsProcessing.getMiddleLine();
    purePersuitInfo = purePursuitComputeABC(carPosition, laneMiddleLine, carLength, lookAheadDistance);

	  carSpeed = MIN((abs((float)STEERING_SERVO_MAX_ANGLE - (float)abs(purePersuitInfo.steeringAngle * (180.0f / M_PI))) / (float)STEERING_SERVO_MAX_ANGLE) * (float)(MAX_SPEED - 90), (float)MAX_SPEED) + 90.0f;
	  carSpeed = MAX((float)carSpeed, (float)MIN_SPEED);
    
    printDataToSerial(leftVectorOld, rightVectorOld, vectorsProcessing.getLeftVector(), vectorsProcessing.getRightVector(), VectorsProcessing::vectorToLineABC(vectorsProcessing.getLeftVector()), VectorsProcessing::vectorToLineABC(vectorsProcessing.getRightVector()), laneMiddleLine, purePersuitInfo, (carSpeed - 90.0f) / (float)(MAX_SPEED - 90));
    
    steeringWheel.setSteeringAngleDeg(purePersuitInfo.steeringAngle * (180.0f / M_PI));
    driverMotor.write((int)carSpeed);

    Serial.println("% LoopTime: " + String(millis() - timeStart) + " ms");
  }
}

