#include <Arduino.h>
#include <SPI.h>
#include "SteeringWheel.h"
#include "ParseNum.h"
#include "pixy2_libs/host/arduino/libraries/Pixy2/Pixy2.h"
#include "PixelGreyscaleRow.h"
// #include <stdio.h>
// #include <stdint.h>

#define MIDDLE_FRAME_LINE 158
#define LANE_WIDTH_PIXELS 1
#define BLACK_COLOR_TRESHOLD 200

#define STEERING_SERVO_ANGLE_MIDDLE     85    // 90 middle
#define STEERING_SERVO_ANGLE_MAX_RIGHT  40    // 38 max right
#define STEERING_SERVO_ANGLE_MAX_LEFT   130   // 135 max left

#define STEERING_SERVO_PIN  3
#define DRIVER_MOTOR_PIN  9

unsigned int lineMinPixelsLength = LANE_WIDTH_PIXELS;
uint8_t lineColorTreshold = BLACK_COLOR_TRESHOLD;
float servoAngle = 50;
int servoTimeout = 7;
int servoAngleIncrease = 4;
SteeringWheel steeringWheel(STEERING_SERVO_ANGLE_MAX_LEFT, STEERING_SERVO_ANGLE_MIDDLE, STEERING_SERVO_ANGLE_MAX_RIGHT);
PWMServo driverMotor;
Pixy2 pixy;
int8_t res;

PixelGreyscaleRow pixelRow;


// 90;10;2
//servoAngle;servoTimeout;servoAngleIncrease
int ParseSerialInput() {
  int i = 0;
  int values_parsed = 0;
  int char_parsed;
  int milliseconds, increase;
  float angle;
  const char* _str = input_buffer.c_str();
  Serial.print("%");
  Serial.println(_str);

  char_parsed = ReadNextFloat(&(_str[i]), &angle);
  if (char_parsed == 0) {
    return values_parsed;
  }
  values_parsed++;
  i += char_parsed + 1;

  char_parsed = ReadNextInt(&(_str[i]), (int*)&milliseconds);
  if (char_parsed == 0) {
    return values_parsed;
  }
  values_parsed++;
  i += char_parsed + 1;

    char_parsed = ReadNextInt(&(_str[i]), (int*)&increase);
  if (char_parsed == 0) {
    return values_parsed;
  }
  values_parsed++;
  i += char_parsed + 1;

  servoTimeout = milliseconds;
  servoAngle = angle;
  servoAngleIncrease = increase;
  
  return values_parsed;
}


void setup() {
    Serial.begin(115200);
    delay(2000);
    driverMotor.attach(DRIVER_MOTOR_PIN, 1000, 2000);
    delay(2000);
    //while (!Serial);
    Serial.println("Starting...");
    steeringWheel.attach(STEERING_SERVO_PIN);
    
    steeringWheel.setSteeringAngle(0);
    // we must initialize the pixy object
    res = pixy.init();
    Serial.println("pixy.init() = " + String(res));
    // Getting the RGB pixel values requires the 'video' program
    res = pixy.changeProg("video");
    Serial.println("pixy.changeProg(video) = " + String(res));
    
}

uint8_t r, g, b;
uint8_t greyscale;
PixelRowBlackLine leftLine, rightLine;
int laneCenter = 0;
int i;

void loop() {
    if (ReadSerial())
    {
      ParseSerialInput();
      input_buffer = "";
    }
    for (i = 0; i < pixy.frameWidth; i++)   // read a row of pixels from camera
    {
      if (pixy.video.getRGB(i, pixy.frameHeight/2, &r, &g, &b)==0)
      {
        greyscale = PixelGreyscaleRow::RGBtoGreyscale(r, g, b);
      }
      else{
        greyscale = 255;
      }
      pixelRow.addPixelGreyscale(greyscale);
    }
    // extract roght line and left line from the row of pixels
    leftLine = pixelRow.getFirstBlackLine(lineColorTreshold, lineMinPixelsLength);
    rightLine = pixelRow.getLastBlackLine(lineColorTreshold, lineMinPixelsLength);
    pixelRow.clear();


    Serial.println(String(leftLine.beginIndex) + ";" + String(leftLine.endIndex) + ";" + String(rightLine.beginIndex) + ";" + String(rightLine.endIndex));


    if ((leftLine.beginIndex != leftLine.endIndex) && (rightLine.beginIndex != rightLine.endIndex) && (leftLine.beginIndex != rightLine.beginIndex))     // both lines are detected
    {
        driverMotor.write(100);
        laneCenter = leftLine.endIndex + (int)((float)(rightLine.beginIndex - leftLine.endIndex) / (float)2);
        if (abs(laneCenter - MIDDLE_FRAME_LINE) > 10)   // span of pyxels where the car is allowed to stay without steering
        {
            if (laneCenter > MIDDLE_FRAME_LINE) { // steer right
                steeringWheel.setSteeringAngle(-180);
            }
            else if(laneCenter < MIDDLE_FRAME_LINE){ // steer left
                steeringWheel.setSteeringAngle(+180);
            }
        }
        else{
            steeringWheel.setSteeringAngle(0);
        }
    }
    else{
        steeringWheel.setSteeringAngle(0);
        //driverMotor.write(90);
    }
    //delay(500);
}

