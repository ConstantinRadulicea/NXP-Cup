// https://colorizer.org/

#include <Arduino.h>
#include <SPI.h>
#include "SteeringWheel.h"
#include "pixy2_libs/host/arduino/libraries/Pixy2/Pixy2.h"
#include "PurePursuitGeometry.h"
#include "VectorsProcessing.h"

#define SCREEN_CENTER_X (int)(78.0f / 2.0f)
#define LINE_WIDTH_PIXELS 2
#define LANE_WIDTH_PIXELS 25
#define LANE_WIDTH_TOLERANCE_PIXELS 3

#define STEERING_SERVO_ANGLE_MIDDLE     85    // 90 middle
#define STEERING_SERVO_ANGLE_MAX_RIGHT  42    // 38 max right
#define STEERING_SERVO_ANGLE_MAX_LEFT   130   // 135 max left

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
    steeringWheel.setSteeringAngle(0);
    
    // we must initialize the pixy object
    res = pixy.init();
    Serial.println("% pixy.init() = " + String(res));
    // Getting the RGB pixel values requires the 'video' program
    res = pixy.changeProg("line");
    Serial.println("% pixy.changeProg(line) = " + String(res));
    //delay(10000);
}

void printDataToSerial(LineABC leftLine, LineABC rightLine, LineABC laneMiddleLine, PurePersuitInfo purePersuitInfo){
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
  Serial.println();
}

void loop() {
  int i;
  LineABC laneMiddleLine;
  Vector vec;
  PurePersuitInfo purePersuitInfo;
  Point2D carPosition;
  float carLength, laneWidth, lookAheadDistance;

  carPosition.x = SCREEN_CENTER_X;
  carPosition.y = 0;

  carLength = 10;
  laneWidth = LANE_WIDTH_PIXELS;
  lookAheadDistance = 30;
  
  vectorsProcessing.setCarPosition(carPosition);
  vectorsProcessing.setLaneWidth(laneWidth);
  vectorsProcessing.setMinXaxisAngle((15.0f / 180.0f) * M_PI);
  while (1)
  {
    pixy.line.getAllFeatures();
    //Serial.println("Tot lines: " + String((int)pixy.line.numVectors));

    for (i=0; i < pixy.line.numVectors; i++)
    {
      vec = pixy.line.vectors[i];
      vec = VectorsProcessing::vectorInvert(vec);
      vectorsProcessing.addVector(vec);
    }

    laneMiddleLine = vectorsProcessing.getMiddleLine();
    //Serial.println("(" + String(laneMiddleLine.Ax) + ")x + " + "(" + String(laneMiddleLine.By) + ")y + " + "(" + String(laneMiddleLine.C) + ") = 0");
    purePersuitInfo = purePursuitComputeABC(carPosition, laneMiddleLine, carLength, lookAheadDistance);

    printDataToSerial(VectorsProcessing::vectorToLineABC(vectorsProcessing.getLeftVector()), VectorsProcessing::vectorToLineABC(vectorsProcessing.getRightVector()), laneMiddleLine, purePersuitInfo);
    
    vectorsProcessing.clear();
    
  }
}

