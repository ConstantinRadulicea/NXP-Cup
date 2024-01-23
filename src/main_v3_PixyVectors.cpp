// https://colorizer.org/

#include <Arduino.h>
#include <SPI.h>
#include "SteeringWheel.h"
#include "pixy2_libs/host/arduino/libraries/Pixy2/Pixy2.h"
#include "PurePursuitGeometry.h"
#include "VectorsProcessing.h"

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

#define MIN_SPEED (int)100
#define MAX_SPEED (int)150
#define SCREEN_CENTER_X (int)(78.0f / 2.0f)
#define IMAGE_MAX_X 78
#define IMAGE_MAX_Y 51
#define LANE_WIDTH_PIXELS 50

#define STEERING_SERVO_ANGLE_MIDDLE     85    // 90 middle
#define STEERING_SERVO_ANGLE_MAX_RIGHT  0    // 38 max right
#define STEERING_SERVO_ANGLE_MAX_LEFT   180   // 135 max left
#define STEERING_SERVO_MAX_ANGLE MAX(abs(STEERING_SERVO_ANGLE_MIDDLE - STEERING_SERVO_ANGLE_MAX_RIGHT), abs(STEERING_SERVO_ANGLE_MIDDLE - STEERING_SERVO_ANGLE_MAX_LEFT))

#define STEERING_SERVO_PIN  3
#define DRIVER_MOTOR_PIN  9

SteeringWheel steeringWheel(STEERING_SERVO_ANGLE_MAX_LEFT, STEERING_SERVO_ANGLE_MIDDLE, STEERING_SERVO_ANGLE_MAX_RIGHT);
PWMServo driverMotor;
VectorsProcessing vectorsProcessing;
Pixy2 pixy;
int8_t res;


void setup() {
    // serial Initialization
    Serial.begin(115200);
    delay(100);

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
    //delay(10000);
}

void printDataToSerial(Vector leftVector, Vector rightVector, LineABC leftLine, LineABC rightLine, LineABC laneMiddleLine, PurePersuitInfo purePersuitInfo, float carAcceleration){
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
  LineABC laneMiddleLine, mirrorLine;
  Vector vec;
  PurePersuitInfo purePersuitInfo;
  Point2D carPosition;
  float carLength, laneWidth, lookAheadDistance, carSpeed;

  mirrorLine = xAxisABC();
  mirrorLine.C = -(((float)IMAGE_MAX_Y) / 2.0f);

  carPosition.x = (float)SCREEN_CENTER_X;
  carPosition.y = 0.0f;

  laneWidth = (float)LANE_WIDTH_PIXELS;
  carLength = (17.5f / 53.5f) * laneWidth;
  
  lookAheadDistance = (30.0f / 53.5f) * laneWidth;
  
  vectorsProcessing.setCarPosition(carPosition);
  vectorsProcessing.setLaneWidth(laneWidth);
  vectorsProcessing.setMinXaxisAngle((1.0f / 180.0f) * M_PI);
  while (1)
  {
    Serial.println("% Read lines");
    vectorsProcessing.clear();
    if(pixy.line.getAllFeatures(LINE_VECTOR) >= (int8_t)0){
      for (i=0; i < pixy.line.numVectors; i++)
      {
        vec = pixy.line.vectors[i];
        vec = VectorsProcessing::mirrorVector(mirrorLine, vec);
        Serial.print('%');
        vec.print();
        vectorsProcessing.addVector(vec);
      }
      Serial.println("%Iteration");
    }
    Serial.println("% Get middle line");
    laneMiddleLine = vectorsProcessing.getMiddleLine();
    Serial.println("% Get Steering angle");
    purePersuitInfo = purePursuitComputeABC(carPosition, laneMiddleLine, carLength, lookAheadDistance);

	  carSpeed = MIN((abs((float)STEERING_SERVO_MAX_ANGLE - (float)abs(purePersuitInfo.steeringAngle * (180.0f / M_PI))) / (float)STEERING_SERVO_MAX_ANGLE) * (float)(MAX_SPEED - 90), (float)MAX_SPEED) + 90.0f;
	  carSpeed = MAX((float)carSpeed, (float)MIN_SPEED);
    Serial.println("% Print data to matlab");
    printDataToSerial(vectorsProcessing.getLeftVector(), vectorsProcessing.getRightVector(), VectorsProcessing::vectorToLineABC(vectorsProcessing.getLeftVector()), VectorsProcessing::vectorToLineABC(vectorsProcessing.getRightVector()), laneMiddleLine, purePersuitInfo, (carSpeed - 90.0f) / (float)(MAX_SPEED - 90));
    Serial.println("% Steering wheel");
    steeringWheel.setSteeringAngleDeg(purePersuitInfo.steeringAngle * (180.0f / M_PI));
    Serial.println("% gas");
    driverMotor.write((int)carSpeed);
  }
}

