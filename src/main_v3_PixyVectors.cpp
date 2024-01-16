// https://colorizer.org/

#include <Arduino.h>
#include <SPI.h>
#include "SteeringWheel.h"
#include "pixy2_libs/host/arduino/libraries/Pixy2/Pixy2.h"
#include "PurePursuitGeometry.h"

#define SCREEN_CENTER_X 158
#define LINE_WIDTH_PIXELS 2
#define LANE_WIDTH_PIXELS 200
#define LANE_WIDTH_TOLERANCE_PIXELS 10

#define STEERING_SERVO_ANGLE_MIDDLE     85    // 90 middle
#define STEERING_SERVO_ANGLE_MAX_RIGHT  42    // 38 max right
#define STEERING_SERVO_ANGLE_MAX_LEFT   130   // 135 max left

#define STEERING_SERVO_PIN  3
#define DRIVER_MOTOR_PIN  9

SteeringWheel steeringWheel(STEERING_SERVO_ANGLE_MAX_LEFT, STEERING_SERVO_ANGLE_MIDDLE, STEERING_SERVO_ANGLE_MAX_RIGHT);
PWMServo driverMotor;
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

void loop() {
  uint8_t i;
  while (1)
  {
    
    char buf[128];
  
    pixy.line.getAllFeatures();

    // print all vectors
    for (i=0; i<pixy.line.numVectors; i++)
    {
      sprintf(buf, "line %d: ", i);
      Serial.print(buf);
      pixy.line.vectors[i].print();
    }
    
    // print all intersections
    for (i=0; i<pixy.line.numIntersections; i++)
    {
      sprintf(buf, "intersection %d: ", i);
      Serial.print(buf);
      pixy.line.intersections[i].print();
    }
  }
}

