// https://colorizer.org/

#include "Config.h"



SteeringWheel steeringWheel(STEERING_SERVO_ANGLE_MAX_LEFT, STEERING_SERVO_ANGLE_MIDDLE, STEERING_SERVO_ANGLE_MAX_RIGHT, (unsigned int)0);

#if ENABLE_ARDUINO == 1
  Servo driverMotor;
#else
  PWMServo driverMotor;
#endif

VectorsProcessing vectorsProcessing;
Pixy2 pixy;

/*====================================================================================================================================*/

void FailureModeMessage(Pixy2 &pixy, int iteration, String errorText, float &carSpeed){
  #if ENABLE_SERIAL_PRINT == 1
    SERIAL.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("iters [") + String(iteration) + String("] " + errorText));
  #endif
  if (iteration >= 5){  
    #if ENABLE_DRIVERMOTOR == 1
      driverMotor.write((int)STANDSTILL_SPEED);
    #endif
    carSpeed = (float)STANDSTILL_SPEED;
    while (pixy.init() != PIXY_RESULT_OK) {
    delay(10);
    }
  }
}

/*==============================================================================*/

void setup() {
  int8_t pixyResult;

  // serial Initialization
  #if ENABLE_SERIAL_PRINT == 1 || ENABLE_WIRELESS_DEBUG == 1
    SERIAL.begin(115200);
    while (!SERIAL){
      delay(100);
    }
  #endif

  #if ENABLE_WIRELESS_DEBUG == 1
    serial2WifiConnect(SERIAL, String(DEBUG_WIFI_INIT_SEQUENCE), String(DEBUG_WIFI_SSID), String(DEBUG_WIFI_PASSWORD), String(DEBUG_HOST_IPADDRESS), DEBUG_HOST_PORT);
  #endif

  #if ENABLE_WIRELESS_DEBUG == 1 && ENABLE_SERIAL_PRINT == 1
    printSerial2WifiInfo(String(DEBUG_WIFI_INIT_SEQUENCE), String(DEBUG_WIFI_SSID), String(DEBUG_WIFI_PASSWORD), String(DEBUG_HOST_IPADDRESS), DEBUG_HOST_PORT);
  #endif

  // Initialization and attachment of the servo and motor
  #if ENABLE_STEERING_SERVO == 1
    steeringWheel.attach(STEERING_SERVO_PIN);
    steeringWheel.setSteeringAngleDeg(0);
  #endif

  #if ENABLE_DRIVERMOTOR == 1
    #if ENABLE_ARDUINO == 1
      driverMotor.attach(DRIVER_MOTOR_PIN);
      driverMotor.writeMicroseconds(1500);
    #else
      driverMotor.attach(DRIVER_MOTOR_PIN, 1000, 2000);
    #endif
    driverMotor.write((int)STANDSTILL_SPEED);
  #endif
    
  // we must initialize the pixy object
  pixyResult = pixy.init();
  #if ENABLE_SERIAL_PRINT == 1
    SERIAL.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("pixy.init() = ") + String(pixyResult));
  #endif
    
  // Getting the RGB pixel values requires the 'video' program
  pixyResult = pixy.changeProg("line");
  #if ENABLE_SERIAL_PRINT == 1
    SERIAL.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("pixy.changeProg(line) = ") + String(pixyResult));
  #endif
  #if ENABLE_DRIVERMOTOR == 1
    delay(3000);
  #endif
  #if ENABLE_SERIAL_PRINT == 1
    SERIAL.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("Setup completed!"));
  #endif
}

/*====================================================================================================================================*/
void loop() {
  int i;
  uint32_t timeStart, loopIterationsCountNoVectorDetected, loopIterationsCountVectorRetriveError, loopIterationsCountPixyChangeProgramError;
  LineABC laneMiddleLine, mirrorLine;
  Vector vec, leftVectorOld, rightVectorOld;
  PurePersuitInfo purePersuitInfo;
  Point2D carPosition;
  float carLength, laneWidth, lookAheadDistance, carSpeed;
  
  carSpeed = 0.0f;
  timeStart = 0;
  loopIterationsCountNoVectorDetected = 0;
  loopIterationsCountVectorRetriveError = 0;

  mirrorLine = xAxisABC();
  mirrorLine.C = -(((float)IMAGE_MAX_Y) / 2.0f);

  carPosition.x = (float)SCREEN_CENTER_X;
  carPosition.y = 0.0f;

  laneWidth = (float)LANE_WIDTH_VECTOR_UNIT;
  carLength = (float)CAR_LENGTH_CM * (float)VECTOR_UNIT_PER_CM;
  
  lookAheadDistance = (float)LOOKAHEAD_MIN_DISTANCE_CM * (float)VECTOR_UNIT_PER_CM;
  
  vectorsProcessing.setCarPosition(carPosition);
  vectorsProcessing.setLaneWidth(laneWidth);
  vectorsProcessing.setMinXaxisAngle(3.0f * RADIANS_PER_DEGREE);
  while (1)
  {
    timeStart = millis();
    vectorsProcessing.clear();
    if(pixy.line.getAllFeatures(LINE_VECTOR) >= (int8_t)0){
      loopIterationsCountVectorRetriveError = 0;
      if (pixy.line.numVectors > 0){
        loopIterationsCountNoVectorDetected = 0;
      }
      else{
        loopIterationsCountNoVectorDetected++;
      }
      
      for (i=0; i < pixy.line.numVectors; i++)
      {
        vec = pixy.line.vectors[i];
        vec = VectorsProcessing::mirrorVector(mirrorLine, vec);
        vec = VectorsProcessing::reComputeVectorStartEnd_basedOnDistanceOfPointXaxis(vec, carPosition);
        vectorsProcessing.addVector(vec);
      }
      leftVectorOld = vectorsProcessing.getLeftVector();
      rightVectorOld = vectorsProcessing.getRightVector();

      #if ENABLE_PIXY_VECTOR_APPROXIMATION == 1
      if (((int)vectorsProcessing.isVectorValid(rightVectorOld) + (int)vectorsProcessing.isVectorValid(leftVectorOld))==1){
        carSpeed = (float)MIN_SPEED;

        loopIterationsCountPixyChangeProgramError=0;
        while (pixy.changeProg("video") != PIXY_RESULT_OK)
        {
          loopIterationsCountPixyChangeProgramError++;
          FailureModeMessage(pixy, loopIterationsCountPixyChangeProgramError,"ERROR: pixy.changeProg(\"video\")",carSpeed);
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
        
        loopIterationsCountPixyChangeProgramError = 0;
        while (pixy.changeProg("line") != PIXY_RESULT_OK) {
          loopIterationsCountPixyChangeProgramError++;
          FailureModeMessage(pixy, loopIterationsCountPixyChangeProgramError,"ERROR: pixy.changeProg(\"line\")",carSpeed);
        }
        delay(40);
      }
      #endif

      laneMiddleLine = vectorsProcessing.getMiddleLine();
      purePersuitInfo = purePursuitComputeABC(carPosition, laneMiddleLine, carLength, lookAheadDistance);

      if (loopIterationsCountNoVectorDetected > 15)
      {
        carSpeed = (float)MIN_SPEED;
        #if ENABLE_DRIVERMOTOR == 1
          driverMotor.write((int)carSpeed);
        #endif
      }
      else{
        carSpeed = MIN((abs((float)STEERING_SERVO_MAX_ANGLE - (float)abs(purePersuitInfo.steeringAngle * DEGREES_PER_RADIAN)) / (float)STEERING_SERVO_MAX_ANGLE) * (float)(MAX_SPEED - STANDSTILL_SPEED), (float)MAX_SPEED) + (float)STANDSTILL_SPEED;
        carSpeed = MAX((float)carSpeed, (float)MIN_SPEED);
      }
    }
    else{
      loopIterationsCountVectorRetriveError++;
      FailureModeMessage(pixy, loopIterationsCountVectorRetriveError,"ERROR: pixy.line.getAllFeatures(LINE_VECTOR)",carSpeed);
    }
    
    #if ENABLE_SERIAL_PRINT == 1
        printDataToSerial(leftVectorOld, rightVectorOld, vectorsProcessing.getLeftVector(), vectorsProcessing.getRightVector(), VectorsProcessing::vectorToLineABC(vectorsProcessing.getLeftVector()), VectorsProcessing::vectorToLineABC(vectorsProcessing.getRightVector()), laneMiddleLine, purePersuitInfo, (carSpeed - (float)STANDSTILL_SPEED) / (float)(MAX_SPEED - STANDSTILL_SPEED));
    #endif
    
    #if ENABLE_STEERING_SERVO == 1
      steeringWheel.setSteeringAngleDeg(purePersuitInfo.steeringAngle * DEGREES_PER_RADIAN);
    #endif

    #if ENABLE_DRIVERMOTOR == 1
      driverMotor.write((int)carSpeed);
    #endif
    
    #if ENABLE_SERIAL_PRINT == 1
      SERIAL.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("LoopTime: ") + String(millis() - timeStart) + String(" ms"));
    #endif
  }
}

