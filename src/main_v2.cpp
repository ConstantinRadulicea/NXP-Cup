#include <Arduino.h>
#include <SPI.h>
#include "SteeringWheel.h"
#include "ParseNum.h"
#include "pixy2_libs/host/arduino/libraries/Pixy2/Pixy2.h"
#include "PixelGreyscaleRow.h"
#include "TrackLane.h"
#include "PID_v1.h"
// #include <stdio.h>
// #include <stdint.h>

#define SCREEN_CENTER_X 158
#define LINE_WIDTH_PIXELS 2
#define LANE_WIDTH_PIXELS 200
#define LANE_WIDTH_TOLERANCE_PIXELS 30
#define BLACK_COLOR_TRESHOLD 180

#define STEERING_SERVO_ANGLE_MIDDLE     85    // 90 middle
#define STEERING_SERVO_ANGLE_MAX_RIGHT  42    // 38 max right
#define STEERING_SERVO_ANGLE_MAX_LEFT   130   // 135 max left

#define STEERING_SERVO_PIN  3
#define DRIVER_MOTOR_PIN  9

unsigned int lineMinPixelsLength = LINE_WIDTH_PIXELS;
uint8_t lineColorTreshold = BLACK_COLOR_TRESHOLD;
SteeringWheel steeringWheel(STEERING_SERVO_ANGLE_MAX_LEFT, STEERING_SERVO_ANGLE_MIDDLE, STEERING_SERVO_ANGLE_MAX_RIGHT);
PWMServo driverMotor;
Pixy2 pixy;
int8_t res;
TrackLane trackLane(BLACK_COLOR_TRESHOLD, SCREEN_CENTER_X, LINE_WIDTH_PIXELS, LANE_WIDTH_PIXELS, LANE_WIDTH_TOLERANCE_PIXELS);

double PID_input, PID_output, PID_setpoint = 0.0;
double PID_Kp=9.0, PID_Ki=0.01, PID_Kd=0;
PID myPID(&PID_input, &PID_output, &PID_setpoint, PID_Kp, PID_Ki, PID_Kd, DIRECT);


void setup() {
    // serial init
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
    res = pixy.changeProg("video");
    Serial.println("% pixy.changeProg(video) = " + String(res));

    //Configure PID
    myPID.SetMode(AUTOMATIC);
    myPID.SetSampleTime(5);
    myPID.SetOutputLimits(-200.0, 200.0);
    delay(10000);
}

void printToSerialPixelRow()
{
    std::vector<uint8_t>& pixelRow = trackLane.getRow();
    for(int i = 0; i < pixelRow.size(); i++){
        Serial.print(String((int)pixelRow[i]) + ";");
    }
}

// used to find the lane width
// calibrate the lane width based on values read from the Pixy2 camera. 
// It adds multiple rows of pixels to the trackLane object, applies a Simple Moving Average (SMA) filter, 
// and determines the average width of the detected lane.
void autoCalibrateLaneLength(){
  int j, i;
  int laneWidth = 0;
  uint8_t greyscale, r, g, b;

  for (j = 0; j < 10; j++)
  {
      for (i = 0; i < pixy.frameWidth; i++)   // read a row of pixels from camera
      {
        if (pixy.video.getRGB(i, (int)(pixy.frameHeight-5), &r, &g, &b)==0) // pixy.frameHeight-5 = bottom up, 5 pixels above the bottom edge of the Pixy2 image.
        {
          greyscale = PixelGreyscaleRow::RGBtoGreyscale(r, g, b);
        }
        else{
          greyscale = 255;
        }
        trackLane.addPixelGreyscale(greyscale);
      }

      trackLane.applySmaFilter(4);// SMA with the last 4 pixels value (grey scale)
      laneWidth += trackLane.autocalibrateLaneWidth();
      trackLane.clear();
  }

  laneWidth = (int)((float)laneWidth / (float)(j-1));
  trackLane.setLaneWidth(laneWidth);
}

uint8_t r, g, b;
uint8_t greyscale;
PixelRowBlackLine leftLine, rightLine;
int laneCenter = 0;
int i;

void loop() {
  
  autoCalibrateLaneLength();
  while (1)
  {
    // reading a row of pixels pixel by pixel from the camera
    for (i = 0; i < pixy.frameWidth; i++)   // read a row of pixels from camera
      {
        if (pixy.video.getRGB(i, (int)(pixy.frameHeight-5), &r, &g, &b)==0)
        {
          greyscale = PixelGreyscaleRow::RGBtoGreyscale(r, g, b);
        }
        else{
          greyscale = 255;
        }
        trackLane.addPixelGreyscale(greyscale);
      }

      // do calculations to find the lane center
      trackLane.applySmaFilter(4);
      laneCenter = trackLane.getLaneCenter();
      leftLine = trackLane.getLeftEdge();
      rightLine = trackLane.getRightEdge();
      printToSerialPixelRow();
      trackLane.clear();

      Serial.println("," + String(leftLine.beginIndex) + ";" +  String(leftLine.endIndex) + ";" + String(rightLine.beginIndex) + ";" +  String(rightLine.endIndex));

      // compute PID_input using the deviation of the lane center from the camera center
      PID_input = (double)(laneCenter - ((int)SCREEN_CENTER_X)); // positive input: have to go right;    negative input: have to go left;    
      myPID.Compute();
      
      steeringWheel.setSteeringAngle((float)PID_output);
      driverMotor.write(100);

      /*
      Serial.print("laneCenter: " + String(laneCenter));
      Serial.print("\tPID_input: " + String(PID_input));
      Serial.print("\tPID_output: " + String(PID_output));
      Serial.print("\t" + String(leftLine.beginIndex) + ";" + String(leftLine.endIndex) + ";" + String(rightLine.beginIndex) + ";" + String(rightLine.endIndex));
      Serial.println();
      */
  }
}

