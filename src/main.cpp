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

#include "Config.h"

#define TIMER1_INTERVAL_MS 10

//IntervalTimer emergencyBreakTimer;


#if ENABLE_SETTINGS_MENU == 1
  LiquidCrystal_I2C lcd(0x27, 16, 2);
#endif

SteeringWheel steeringWheel(STEERING_SERVO_ANGLE_MAX_LEFT, STEERING_SERVO_ANGLE_MIDDLE, STEERING_SERVO_ANGLE_MAX_RIGHT, (unsigned int)0);

#if ENABLE_ARDUINO == 1
  Servo driverMotor;
#else
  PWMServo driverMotor;
#endif

VectorsProcessing vectorsProcessing;
Pixy2 pixy;

/*====================================================================================================================================*/

void FailureModeMessage(Pixy2 &pixy, int iteration, String errorText){
  #if ENABLE_SERIAL_PRINT == 1
    SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("iters [") + String(iteration) + String("] ERROR: " + errorText));

  #endif
  if (iteration >= 5){  
    carSpeed = (float)STANDSTILL_SPEED;
 do{
#if ENABLE_DRIVERMOTOR == 1
   driverMotor.write((int)STANDSTILL_SPEED);
    #endif
#if ENABLE_STEERING_SERVO == 1
    steeringWheel.setSteeringAngleDeg(0.0f);
  #endif
delay(10);
    } while (pixy.init() != PIXY_RESULT_OK);
  }
}

/*==============================================================================*/

void setup() {
  int8_t pixyResult;

  // Initialization and attachment of the servo and motor
  #if ENABLE_STEERING_SERVO == 1
    pinMode(STEERING_SERVO_PIN, OUTPUT); 
    steeringWheel.attach(STEERING_SERVO_PIN);
    steeringWheel.setSteeringAngleDeg(0.0f);
  #endif

  #if ENABLE_DRIVERMOTOR == 1
    #if ENABLE_ARDUINO == 1
      driverMotor.attach(DRIVER_MOTOR_PIN);
      driverMotor.writeMicroseconds(1500);
    #else
      driverMotor.attach(DRIVER_MOTOR_PIN, 1000, 2000);
      //driverMotor.attach(DRIVER_MOTOR_PIN);
    #endif
    driverMotor.write((int)STANDSTILL_SPEED);
  #endif

  #if ENABLE_EMERGENCY_BREAKING == 1
  #if ENABLE_DISTANCE_SENSOR1 == 1
    pinMode(DISTANCE_SENSOR1_TRIG_PIN, OUTPUT); 
    pinMode(DISTANCE_SENSOR1_ECHO_PIN, INPUT); 
  #endif

  #if ENABLE_DISTANCE_SENSOR2 == 1
    pinMode(DISTANCE_SENSOR2_TRIG_PIN, OUTPUT); 
    pinMode(DISTANCE_SENSOR2_ECHO_PIN, INPUT); 
  #endif

  #if ENABLE_DISTANCE_SENSOR3 == 1
    pinMode(DISTANCE_SENSOR3_TRIG_PIN, OUTPUT); 
    pinMode(DISTANCE_SENSOR3_ECHO_PIN, INPUT); 
  #endif
   
   pinMode(EMERGENCY_BREAK_LIGHT_PIN, OUTPUT);
   digitalWrite(EMERGENCY_BREAK_LIGHT_PIN, LOW);
  #endif

  #if ENABLE_SETTINGS_MENU == 1
    lcd.init();  //display initialization
    lcd.backlight();  // activate the backlight
    pinMode(MENU_LEFT_ARROW_BUTTON_PIN, INPUT);
    pinMode(MENU_RIGHT_ARROW_BUTTON_PIN, INPUT);
    pinMode(MENU_INCREMENT_BUTTON_PIN, INPUT);
    pinMode(MENU_DECREMENT_BUTTON_PIN, INPUT);
  #endif

  // serial Initialization
  #if ENABLE_SERIAL_PRINT == 1 || ENABLE_WIRELESS_DEBUG == 1
    SERIAL_PORT.begin(115200);
    while (!SERIAL_PORT){
      delay(100);
    }
  #endif

  #if ENABLE_WIRELESS_DEBUG == 1
    serial2WifiConnect(SERIAL_PORT, String(DEBUG_WIFI_INIT_SEQUENCE), String(DEBUG_WIFI_SSID), String(DEBUG_WIFI_PASSWORD), String(DEBUG_HOST_IPADDRESS), DEBUG_HOST_PORT);
  #endif

  #if ENABLE_WIRELESS_DEBUG == 1
    printSerial2WifiInfo(String(DEBUG_WIFI_INIT_SEQUENCE), String(DEBUG_WIFI_SSID), String(DEBUG_WIFI_PASSWORD), String(DEBUG_HOST_IPADDRESS), DEBUG_HOST_PORT);
  #endif
    
  // we must initialize the pixy object
  pixyResult = pixy.init();
  
  #if ENABLE_SERIAL_PRINT == 1
    SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("pixy.init() = ") + String(pixyResult));
  #endif

  //pixy.setLamp(1,1);
    
  // Getting the RGB pixel values requires the 'video' program
  pixyResult = pixy.changeProg("line");
  #if ENABLE_SERIAL_PRINT == 1
    SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("pixy.changeProg(line) = ") + String(pixyResult));
  #endif

  #if ENABLE_DRIVERMOTOR == 1
    float startTime_ = (float)millis();
    while (((float)millis() - startTime_) < 3000.0f) {
      driverMotor.write((int)STANDSTILL_SPEED);
    }
  #endif

  #if ENABLE_SERIAL_PRINT == 1
    SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("Setup completed!"));
  #endif
}

/*==============================================================================*/

static float getFrontObstacleDistance_cm(){
  //static SimpleKalmanFilter simpleKalmanFilter(0.1f, 0.1f, 0.001f);

  #if ENABLE_DISTANCE_SENSOR1 == 1
    static MovingAverage movingAverage_sensor1(3);
  #endif
  #if ENABLE_DISTANCE_SENSOR2 == 1
    static MovingAverage movingAverage_sensor2(3);
  #endif

  #if ENABLE_DISTANCE_SENSOR3 == 1
    static MovingAverage movingAverage_sensor3(3);
  #endif
  
  // calculations were made in centimeters
  static uint32_t pulseInTimeout_us = (uint32_t)((200.0f / 34300.0f) * 1000000.0f);

  float duration;
  float measured_distance = 0.0f;
  float estimated_distance = 0.0f;
  float estimated_distance_sensor1 = 400.0f, estimated_distance_sensor2 = 400.0f, estimated_distance_sensor3 = 400.0f;

  #if ENABLE_DISTANCE_SENSOR1 == 1
    if (ENABLE_DISTANCE_SENSOR1_SOFT != 0)
    {
      digitalWrite(DISTANCE_SENSOR1_TRIG_PIN, LOW);
      delayMicroseconds(2);
      // Sets the trigPin on HIGH state for 10 micro seconds
      digitalWrite(DISTANCE_SENSOR1_TRIG_PIN, HIGH);
      delayMicroseconds(10); //This pin should be set to HIGH for 10 μs, at which point the HC­SR04 will send out an eight cycle sonic burst at 40 kHZ
      digitalWrite(DISTANCE_SENSOR1_TRIG_PIN, LOW);
      // Reads the echoPin, returns the sound wave travel time in microseconds
      duration = (float)(pulseIn(DISTANCE_SENSOR1_ECHO_PIN, HIGH, pulseInTimeout_us));
      // Calculating the distance
      measured_distance = duration * 0.034321f / 2.0f;

      if (measured_distance <= 0.0f) {
        measured_distance = 400.0f;
      }

      measured_distance = MIN(measured_distance, 400.0f);

      //estimated_distance = simpleKalmanFilter.updateEstimate(measured_distance);
      estimated_distance_sensor1 = movingAverage_sensor1.next(measured_distance);
      //estimated_distance = measured_distance;
    }
  #endif

  #if ENABLE_DISTANCE_SENSOR1 == 1 && ENABLE_DISTANCE_SENSOR2 == 1
  if ((ENABLE_DISTANCE_SENSOR1_SOFT != 0) && (ENABLE_DISTANCE_SENSOR2_SOFT != 0)) {
    delay(1);
  }
  #endif
  
  #if ENABLE_DISTANCE_SENSOR2 == 1
  if (ENABLE_DISTANCE_SENSOR2_SOFT != 0)
  {
    
    digitalWrite(DISTANCE_SENSOR2_TRIG_PIN, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(DISTANCE_SENSOR2_TRIG_PIN, HIGH);
    delayMicroseconds(10); //This pin should be set to HIGH for 10 μs, at which point the HC­SR04 will send out an eight cycle sonic burst at 40 kHZ
    digitalWrite(DISTANCE_SENSOR2_TRIG_PIN, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = (float)(pulseIn(DISTANCE_SENSOR2_ECHO_PIN, HIGH, pulseInTimeout_us));
    // Calculating the distance
    measured_distance = (duration * 0.034321f / 2.0f);

    if (measured_distance <= 0.0f) {
      measured_distance = 400.0f;
    }

    measured_distance = MIN(measured_distance, 400.0f);

    //estimated_distance = simpleKalmanFilter.updateEstimate(measured_distance);
    estimated_distance_sensor2 = movingAverage_sensor2.next(measured_distance);
    //estimated_distance = measured_distance;
  }
  #endif


  #if ENABLE_DISTANCE_SENSOR3 == 1 && (ENABLE_DISTANCE_SENSOR2 == 1 || ENABLE_DISTANCE_SENSOR1 == 1)
  if ((ENABLE_DISTANCE_SENSOR3_SOFT != 0) && ((ENABLE_DISTANCE_SENSOR1_SOFT != 0) || (ENABLE_DISTANCE_SENSOR2_SOFT != 0))) {
    delay(1);
  }
  #endif

  #if ENABLE_DISTANCE_SENSOR3 == 1
    if (ENABLE_DISTANCE_SENSOR3_SOFT != 0)
    {
      digitalWrite(DISTANCE_SENSOR3_TRIG_PIN, LOW);
      delayMicroseconds(2);
      // Sets the trigPin on HIGH state for 10 micro seconds
      digitalWrite(DISTANCE_SENSOR3_TRIG_PIN, HIGH);
      delayMicroseconds(10); //This pin should be set to HIGH for 10 μs, at which point the HC­SR04 will send out an eight cycle sonic burst at 40 kHZ
      digitalWrite(DISTANCE_SENSOR3_TRIG_PIN, LOW);
      // Reads the echoPin, returns the sound wave travel time in microseconds
      duration = (float)(pulseIn(DISTANCE_SENSOR3_ECHO_PIN, HIGH, pulseInTimeout_us));
      // Calculating the distance
      measured_distance = duration * 0.034321f / 2.0f;

      if (measured_distance <= 0.0f) {
        measured_distance = 400.0f;
      }

      measured_distance = MIN(measured_distance, 400.0f);

      //estimated_distance = simpleKalmanFilter.updateEstimate(measured_distance);
      estimated_distance_sensor3 = movingAverage_sensor3.next(measured_distance);
      //estimated_distance = measured_distance;
    }
  #endif


estimated_distance = MIN(estimated_distance_sensor1, estimated_distance_sensor2);
estimated_distance = MIN(estimated_distance, estimated_distance_sensor3);
//estimated_distance = estimated_distance_sensor3;
/*
  #if ENABLE_DISTANCE_SENSOR1 == 1 && ENABLE_DISTANCE_SENSOR2 == 1
    if ((ENABLE_DISTANCE_SENSOR1_SOFT != 0) && (ENABLE_DISTANCE_SENSOR2_SOFT != 0)) {
      estimated_distance = MIN(estimated_distance_sensor1, estimated_distance_sensor2);
    }
    else if (ENABLE_DISTANCE_SENSOR1_SOFT != 0) {
      estimated_distance = estimated_distance_sensor1;
    }
    else if (ENABLE_DISTANCE_SENSOR2_SOFT != 0) {
      estimated_distance = estimated_distance_sensor2;
    }
    
  #elif ENABLE_DISTANCE_SENSOR1 == 1
    if (ENABLE_DISTANCE_SENSOR1_SOFT != 0) {
      estimated_distance = estimated_distance_sensor1;
    }
  #elif ENABLE_DISTANCE_SENSOR2 == 1
    if (ENABLE_DISTANCE_SENSOR2_SOFT != 0) {
      estimated_distance = estimated_distance_sensor2;
    }
  #endif
*/
  return estimated_distance;
}

/*==============================================================================*/

static float calculateCarSpeed(float minSpeed, float maxSpeed, float maxSteeringWheelAngle, float steeringWheelAngle, LineABC laneMiddleLine) {
	float newCarSpeed_bySteeringAngle, speedSpan, angleCurrentTrajectoryAndMiddleLane, newCarSpeed_byTrajectoryAngle;
  LineABC currentTrajectory;

	speedSpan = maxSpeed - minSpeed;
	maxSteeringWheelAngle = fabsf(maxSteeringWheelAngle);
	steeringWheelAngle = fabsf(steeringWheelAngle);
	steeringWheelAngle = MIN(steeringWheelAngle, maxSteeringWheelAngle);

  currentTrajectory = yAxisABC();
	angleCurrentTrajectoryAndMiddleLane = fabsf(angleBetweenLinesABC(currentTrajectory, laneMiddleLine));

  newCarSpeed_byTrajectoryAngle = minSpeed + ((((float)M_PI_2 - angleCurrentTrajectoryAndMiddleLane) / (float)M_PI_2) * speedSpan);

	newCarSpeed_byTrajectoryAngle = MAX(newCarSpeed_byTrajectoryAngle, minSpeed);
	newCarSpeed_byTrajectoryAngle = MIN(newCarSpeed_byTrajectoryAngle, maxSpeed);
	
	newCarSpeed_bySteeringAngle = minSpeed + (((maxSteeringWheelAngle - steeringWheelAngle) / maxSteeringWheelAngle) * speedSpan);

	newCarSpeed_bySteeringAngle = MAX(newCarSpeed_bySteeringAngle, minSpeed);
	newCarSpeed_bySteeringAngle = MIN(newCarSpeed_bySteeringAngle, maxSpeed);

	return (float)MIN(newCarSpeed_bySteeringAngle, newCarSpeed_byTrajectoryAngle);
}

/*==============================================================================*/

static float calculateLookAheadDistance(float minDistance, float maxDistance, LineABC laneMiddleLine) {
	float angleCurrentTrajectoryAndMiddleLane, newLookAheadDistance, distanceSpan;
	LineABC currentTrajectory;
	distanceSpan = maxDistance - minDistance;

	currentTrajectory = yAxisABC();
	angleCurrentTrajectoryAndMiddleLane = fabsf(angleBetweenLinesABC(currentTrajectory, laneMiddleLine));

	newLookAheadDistance = minDistance + ((((float)M_PI_2 - angleCurrentTrajectoryAndMiddleLane) / (float)M_PI_2) * distanceSpan);

	newLookAheadDistance = MAX(newLookAheadDistance, minDistance);
	newLookAheadDistance = MIN(newLookAheadDistance, maxDistance);

	return newLookAheadDistance;
}

/*==============================================================================*/


/*====================================================================================================================================*/
void loop() {
  size_t i;
  int8_t pixyResult;
  uint32_t loopIterationsCountNoVectorDetected, loopIterationsCountVectorRetriveError, loopIterationsCountPixyChangeProgramError;
  LineABC mirrorLine;
  Vector vec, leftVectorOld, rightVectorOld;
  std::vector<Vector> vectors;
  std::vector<Intersection> intersections;
  std::vector<char> serialInputBuffer;
  PurePersuitInfo purePersuitInfo;
  Point2D carPosition;
  float laneWidth, lookAheadDistance, frontObstacleDistance;
  float timeStart;
  float max_speed_original;
  MovingAverage movingAverage_speed(10);

  serialInputBuffer.clear();

  timeStart = 0.0f;
  loopIterationsCountNoVectorDetected = 0;
  loopIterationsCountVectorRetriveError = 0;
  max_speed_original = MAX_SPEED;

  frontObstacleDistance = 0.0f;

  mirrorLine = xAxisABC();
  mirrorLine.C = -(((float)IMAGE_MAX_Y) / 2.0f);

  carPosition.x = (float)SCREEN_CENTER_X;
  carPosition.y = 0.0f;

  middle_lane_line = yAxisABC();

  laneWidth = (float)LANE_WIDTH_VECTOR_UNIT;
  
  lookAheadDistance = (float)LOOKAHEAD_MIN_DISTANCE_CM * (float)VECTOR_UNIT_PER_CM;
  
  vectorsProcessing.setCarPosition(carPosition);
  vectorsProcessing.setLaneWidth(laneWidth);
  vectorsProcessing.setMinXaxisAngle(MIN_XAXIS_ANGLE_VECTOR * RADIANS_PER_DEGREE);
  while (1)
  {
    timeStart = (float)millis();
    movingAverage_speed.next(carSpeed);
    vectorsProcessing.setMinXaxisAngle(MIN_XAXIS_ANGLE_VECTOR * RADIANS_PER_DEGREE);
    
    if (ENABLE_CAR_ENGINE == 0) {
      driverMotor.write((int)STANDSTILL_SPEED);
    }
    
    #if ENABLE_EMERGENCY_BREAKING == 1   // handling emergency braking

    if (emergency_brake_enable_delay_started_count == 0 && ENABLE_CAR_ENGINE != 0) {
      max_speed_original = MAX_SPEED;
      emergency_brake_enable_remaining_delay_s = EMERGENCY_BRAKE_ENABLE_DELAY_S;
      emergency_brake_enable_delay_started_count = 1;
    }
    else if(emergency_brake_enable_delay_started_count != 0 && ENABLE_CAR_ENGINE == 0){
      MAX_SPEED = max_speed_original;
      emergency_brake_enable_remaining_delay_s = 0;
      emergency_brake_enable_delay_started_count = 0;
    }
    
    if (ENABLE_CAR_ENGINE != 0 && emergency_brake_enable_remaining_delay_s > 0.0f) {
      emergency_brake_enable_remaining_delay_s -= (loop_time_ms / 1000.0f);
      emergency_brake_enable_remaining_delay_s = MAX(emergency_brake_enable_remaining_delay_s, 0.0f);

      if (emergency_brake_enable_remaining_delay_s <= 0.0f) {
        max_speed_original = MAX_SPEED;
        MAX_SPEED = MAX_SPEED_AFTER_EMERGENCY_BRAKE_DELAY;
      }
    }
    
    if (ENABLE_EMERGENCY_BRAKE != 0 && emergency_brake_enable_remaining_delay_s <= 0.0f && (ENABLE_DISTANCE_SENSOR1_SOFT != 0 || ENABLE_DISTANCE_SENSOR2_SOFT != 0 || ENABLE_DISTANCE_SENSOR3_SOFT != 0)) {
      frontObstacleDistance = getFrontObstacleDistance_cm();

      if (frontObstacleDistance <= EMERGENCY_BREAK_DISTANCE_CM ) {
        digitalWrite(EMERGENCY_BREAK_LIGHT_PIN, HIGH);
        emergency_break_active = 1;
        emergency_break_loops_count++;

        #if ENABLE_SERIAL_PRINT == 1
          SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("EMRG_BRK loop: ") + String(emergency_break_loops_count));
        #endif

        if (emergency_break_loops_count == 1) {
          //emergencyBreakTimer.begin(TimerHandler1, (int)TIMER1_INTERVAL_MS * (int)1000);

          #if ENABLE_DRIVERMOTOR == 1 &&  ENABLE_EMERGENCYBRAKE_BACKWARDSBRAKE == 1 // use brakes to get to a near standstill
            if (ENABLE_CAR_ENGINE != 0) {
              float tempCarSpeed = movingAverage_speed.next(carSpeed);
              float startTime_ = (float)millis();
              float brakeTime_ = (float)fabsf((tempCarSpeed - (float)STANDSTILL_SPEED)) * (40.0f / (107.0f - 90.0f));
              //float brakeTime_ = (uint32_t)fabsf((tempCarSpeed - (float)STANDSTILL_SPEED)) * expf((1.0f/5.5f)*fabsf((tempCarSpeed - (float)STANDSTILL_SPEED)));
              int brakeSpeed_ = (int)((float)STANDSTILL_SPEED - fabsf(tempCarSpeed - (float)STANDSTILL_SPEED));

              while ((((float)millis()) - startTime_) < brakeTime_) {
                driverMotor.write(brakeSpeed_);
              }
            }
          #endif
          carSpeed = (float)EMERGENCY_BRAKE_MIN_SPEED;  
        }
        
        if(frontObstacleDistance <= EMERGENCY_BREAK_MAX_DISTANCE_FROM_OBSTACLE_CM){
          carSpeed = (float)STANDSTILL_SPEED;
        }
        else{
          carSpeed = (float)EMERGENCY_BRAKE_MIN_SPEED;
        }
        #if ENABLE_DRIVERMOTOR == 1
          if (ENABLE_CAR_ENGINE != 0) {
            driverMotor.write((int)carSpeed);
          }
        #endif

      }
      else{
        emergency_break_active = 0;
        emergency_break_loops_count = 0;
        digitalWrite(EMERGENCY_BREAK_LIGHT_PIN, LOW);
      }
    }
    else{
      emergency_break_active = 0;
      emergency_break_loops_count = 0;
      digitalWrite(EMERGENCY_BREAK_LIGHT_PIN, LOW);
    }

    #endif

    #if ENABLE_SERIAL_PRINT == 1
      if(readRecordFromSerial(SERIAL_PORT, String("\r\n"), serialInputBuffer)){
        //SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("Input: ") + String(serialInputBuffer.data()));
        parseAndSetGlobalVariables(serialInputBuffer, ';');
        printGlobalVariables(SERIAL_PORT);
        serialInputBuffer.clear();
      }
    #endif

    #if ENABLE_SETTINGS_MENU == 1
      settingsMenuRoutine(lcd, MENU_LEFT_ARROW_BUTTON_PIN, MENU_RIGHT_ARROW_BUTTON_PIN, MENU_INCREMENT_BUTTON_PIN, MENU_DECREMENT_BUTTON_PIN);
    #endif

    if (ENABLE_CAR_STEERING_WHEEL == 0 && ENABLE_CAR_ENGINE != 0) {
      ENABLE_CAR_STEERING_WHEEL = 1;
    }

    vectorsProcessing.clear();
    if(pixy.line.getAllFeatures(LINE_VECTOR | LINE_INTERSECTION) >= (int8_t)0){
      vectors.resize(pixy.line.numVectors);
      memcpy(vectors.data(), pixy.line.vectors, (pixy.line.numVectors * sizeof(Vector)));
      intersections.resize(pixy.line.numIntersections);
      memcpy(intersections.data(), pixy.line.intersections, (pixy.line.numIntersections * sizeof(Intersection)));

      VectorsProcessing::filterVectorIntersections(vectors, intersections);

      loopIterationsCountVectorRetriveError = 0;
      if (vectors.size() > 0){
        loopIterationsCountNoVectorDetected = 0;
      }
      else{
        loopIterationsCountNoVectorDetected++;
      }

      for (i=0; i < vectors.size(); i++)
      {
        vec = vectors[i];
        vec = VectorsProcessing::mirrorVector(mirrorLine, vec);
        vec = VectorsProcessing::reComputeVectorStartEnd_basedOnDistanceOfPointXaxis(vec, carPosition);
        vectorsProcessing.addVector(vec);
      }
      leftVectorOld = vectorsProcessing.getLeftVector();
      rightVectorOld = vectorsProcessing.getRightVector();

      #if ENABLE_PIXY_VECTOR_APPROXIMATION == 1
      if(emergency_break_active == 0 && ENABLE_PIXY_VECTOR_APPROXIMATION_SOFT != 0){
        if (((int)vectorsProcessing.isVectorValid(rightVectorOld) + (int)vectorsProcessing.isVectorValid(leftVectorOld))==1){
          if (emergency_break_active == 0){
            carSpeed = (float)MIN_SPEED;
          }
          #if ENABLE_DRIVERMOTOR == 1
            if (ENABLE_CAR_ENGINE != 0) {
              driverMotor.write((int)carSpeed);
            }
          #endif

          loopIterationsCountPixyChangeProgramError=0;
          while ((pixyResult = pixy.changeProg("video")) != PIXY_RESULT_OK) {
            loopIterationsCountPixyChangeProgramError++;
            FailureModeMessage(pixy, loopIterationsCountPixyChangeProgramError,"pixy video");
            delay(10);
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
          while ((pixyResult = pixy.changeProg("line")) != PIXY_RESULT_OK) {
            loopIterationsCountPixyChangeProgramError++;
            FailureModeMessage(pixy, loopIterationsCountPixyChangeProgramError,"pixy line");
            delay(10);
          }
          delay(40);
        }
      }

      #endif

      middle_lane_line = vectorsProcessing.getMiddleLine();
      lookAheadDistance = calculateLookAheadDistance(LOOKAHEAD_MIN_DISTANCE_CM * VECTOR_UNIT_PER_CM, LOOKAHEAD_MAX_DISTANCE_CM * VECTOR_UNIT_PER_CM, middle_lane_line);
      purePersuitInfo = purePursuitComputeABC(carPosition, middle_lane_line, car_length_vector_unit, lookAheadDistance);
      purePersuitInfo.steeringAngle -= (STEERING_WHEEL_ANGLE_OFFSET * RADIANS_PER_DEGREE);

      if (loopIterationsCountNoVectorDetected > 15)
      {
        if (emergency_break_active == 0) {
          carSpeed = (float)MIN_SPEED;
        }
        #if ENABLE_DRIVERMOTOR == 1
          if (ENABLE_CAR_ENGINE != 0) {
            driverMotor.write((int)carSpeed);
          }
        #endif
      }
      else{
        if (emergency_break_active == 0){
          carSpeed = calculateCarSpeed((float)MIN_SPEED, MAX_SPEED, (float)STEERING_SERVO_MAX_ANGLE, purePersuitInfo.steeringAngle * DEGREES_PER_RADIAN, middle_lane_line);
        }
      }
    }
    else{
      loopIterationsCountVectorRetriveError++;
      FailureModeMessage(pixy, loopIterationsCountVectorRetriveError,"pixy getAllFeatures");
    }
    
    #if ENABLE_SERIAL_PRINT == 1
        printDataToSerial(leftVectorOld, rightVectorOld, vectorsProcessing.getLeftVector(), vectorsProcessing.getRightVector(), VectorsProcessing::vectorToLineABC(vectorsProcessing.getLeftVector()), VectorsProcessing::vectorToLineABC(vectorsProcessing.getRightVector()), middle_lane_line, purePersuitInfo, (carSpeed - (float)STANDSTILL_SPEED) / (float)(MAX_SPEED - STANDSTILL_SPEED), frontObstacleDistance, carSpeed);
    #endif
    
    #if ENABLE_STEERING_SERVO == 1
      if (ENABLE_CAR_STEERING_WHEEL != 0) {
        steeringWheel.setSteeringAngleDeg(purePersuitInfo.steeringAngle * DEGREES_PER_RADIAN);
      }
    #endif

    #if ENABLE_DRIVERMOTOR == 1
      if (ENABLE_CAR_ENGINE != 0) {
        driverMotor.write((int)carSpeed);
      }
    #endif
    
    left_lane_line = VectorsProcessing::vectorToLineABC(vectorsProcessing.getLeftVector());
    right_lane_line = VectorsProcessing::vectorToLineABC(vectorsProcessing.getRightVector());

    loop_time_ms = ((float)millis()) - timeStart;
    loop_time_ms = MAX(loop_time_ms, 0.0f);
    time_passed_ms += loop_time_ms;
    
    #if ENABLE_SERIAL_PRINT == 1
      SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("LoopTime: ") + String(loop_time_ms) + String(" ms"));
    #endif
  }
}



