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

SteeringWheel g_steering_wheel(STEERING_SERVO_ANGLE_MAX_LEFT, STEERING_SERVO_ANGLE_MIDDLE, STEERING_SERVO_ANGLE_MAX_RIGHT, (unsigned int)0);

#if ENABLE_ARDUINO == 1
  Servo driverMotor;
#else
  PWMServo driverMotor;
#endif

VectorsProcessing g_pixy_1_vectors_processing;
//VectorsProcessing pixy_2_vectorsProcessing;
Pixy2SPI_SS g_pixy_1;
//Pixy2SPI_SS pixy_2;

/*====================================================================================================================================*/

void FailureModeMessage(Pixy2SPI_SS &pixy, int iteration, String errorText){
  #if ENABLE_SERIAL_PRINT == 1
    SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("iters [") + String(iteration) + String("] ERROR: " + errorText));

  #endif
  if (iteration >= 5){  
    g_car_speed = (float)STANDSTILL_SPEED;
 do{
#if ENABLE_DRIVERMOTOR == 1
   driverMotor.write((int)STANDSTILL_SPEED);
    #endif
#if ENABLE_STEERING_SERVO == 1
    g_steering_wheel.setSteeringAngleDeg(0.0f);
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
    g_steering_wheel.attach(STEERING_SERVO_PIN);
    g_steering_wheel.setSteeringAngleDeg(0.0f);
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

  #if ENABLE_REMOTE_START_STOP == 1
    pinMode(REMOTE_STOP_PIN, INPUT);
    pinMode(REMOTE_START_PIN, INPUT);
  #endif

  // serial Initialization
  #if ENABLE_SERIAL_PRINT == 1 || ENABLE_WIRELESS_DEBUG == 1
    SERIAL_PORT.begin(230400);
    #if RACE_MODE == 1
    #else
      while (!SERIAL_PORT){
      delay(100);
    }
    #endif

  #endif

  #if ENABLE_WIRELESS_DEBUG == 1
    serial2WifiConnect(SERIAL_PORT, String(DEBUG_WIFI_INIT_SEQUENCE), String(DEBUG_WIFI_SSID), String(DEBUG_WIFI_PASSWORD), String(DEBUG_HOST_IPADDRESS), DEBUG_HOST_PORT);
  #endif

  #if ENABLE_WIRELESS_DEBUG == 1
    printSerial2WifiInfo(SERIAL_PORT, String(DEBUG_WIFI_INIT_SEQUENCE), String(DEBUG_WIFI_SSID), String(DEBUG_WIFI_PASSWORD), String(DEBUG_HOST_IPADDRESS), DEBUG_HOST_PORT);
  #endif
    
  pinMode(SPI_SS_PIXY_1_PIN, OUTPUT);
  pinMode(SPI_SS_PIXY_2_PIN, OUTPUT);

  pixyResult = g_pixy_1.init(SPI_SS_PIXY_1_PIN);
  #if ENABLE_SERIAL_PRINT == 1
    SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("g_pixy_1.init() = ") + String(pixyResult));
  #endif
  /*
  pixyResult = pixy_2.init(SPI_SS_PIXY_2_PIN);
  #if ENABLE_SERIAL_PRINT == 1
    SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("pixy_2.init() = ") + String(pixyResult));
  #endif
  */
  


  g_pixy_1.setLamp(1,1);
    
  // Getting the RGB pixel values requires the 'video' program
  pixyResult = g_pixy_1.changeProg("line");
  #if ENABLE_SERIAL_PRINT == 1
    SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("g_pixy_1.changeProg(line) = ") + String(pixyResult));
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

void remote_control_routine(){
    // Remote State Reading
    #if ENABLE_REMOTE_START_STOP == 1
      // stop car
      if (g_enable_remote_start_stop != 0)
      {
        if (digitalRead(REMOTE_STOP_PIN) == HIGH && g_enable_car_engine != 0) {
          g_enable_car_engine = 0;
        }
        // start car
        else if(digitalRead(REMOTE_START_PIN) == HIGH && g_enable_car_engine == 0){
          g_emergency_brake_enable_delay_started_count = 0;
          g_emergency_brake_enable_remaining_delay_s = 0.0f;
          g_enable_car_engine = 1;
        }
      }
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
    if (g_enable_distance_sensor1 != 0)
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
  if ((g_enable_distance_sensor1 != 0) && (g_enable_distance_sensor2 != 0)) {
    delay(1);
  }
  #endif
  
  #if ENABLE_DISTANCE_SENSOR2 == 1
  if (g_enable_distance_sensor2 != 0)
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
  if ((g_enable_distance_sensor3 != 0) && ((g_enable_distance_sensor1 != 0) || (g_enable_distance_sensor2 != 0))) {
    delay(1);
  }
  #endif

  #if ENABLE_DISTANCE_SENSOR3 == 1
    if (g_enable_distance_sensor3 != 0)
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

  return estimated_distance;
}

/*==============================================================================*/

static float calculateCarSpeed(float minSpeed, float maxSpeed, float maxSteeringWheelAngle, float steeringWheelAngle, LineABC laneMiddleLine, float ki, float kd, float kiMinMaxImpact) {
	float newCarSpeed_bySteeringAngle, speedSpan, angleCurrentTrajectoryAndMiddleLane, newCarSpeed_byTrajectoryAngle;
  static float sumSteeringWheelAngle = 0.0f;
  static float prevSteeringWheelAngleError = 0.0f;
  float derivativeSteeringError;
  LineABC currentTrajectory;

  kiMinMaxImpact = fabs(kiMinMaxImpact);

	speedSpan = maxSpeed - minSpeed;
	maxSteeringWheelAngle = fabsf(maxSteeringWheelAngle);
	steeringWheelAngle = fabsf(steeringWheelAngle);
	steeringWheelAngle = MIN(steeringWheelAngle, maxSteeringWheelAngle);

  derivativeSteeringError = fabs(steeringWheelAngle - prevSteeringWheelAngleError);

  currentTrajectory = yAxisABC();
	angleCurrentTrajectoryAndMiddleLane = fabsf(angleBetweenLinesABC(currentTrajectory, laneMiddleLine));

  newCarSpeed_byTrajectoryAngle = minSpeed + ((((float)M_PI_2 - angleCurrentTrajectoryAndMiddleLane) / (float)M_PI_2) * speedSpan) + (ki * sumSteeringWheelAngle) + (kd * derivativeSteeringError);

	newCarSpeed_byTrajectoryAngle = MAX(newCarSpeed_byTrajectoryAngle, minSpeed);
	newCarSpeed_byTrajectoryAngle = MIN(newCarSpeed_byTrajectoryAngle, maxSpeed);
	
	newCarSpeed_bySteeringAngle = minSpeed + (((maxSteeringWheelAngle - steeringWheelAngle) / maxSteeringWheelAngle) * speedSpan) + (ki * sumSteeringWheelAngle) + (kd * derivativeSteeringError);

	newCarSpeed_bySteeringAngle = MAX(newCarSpeed_bySteeringAngle, minSpeed);
	newCarSpeed_bySteeringAngle = MIN(newCarSpeed_bySteeringAngle, maxSpeed);

  sumSteeringWheelAngle += steeringWheelAngle;

  sumSteeringWheelAngle = MAX(sumSteeringWheelAngle, -kiMinMaxImpact / ki);
	sumSteeringWheelAngle = MIN(sumSteeringWheelAngle, kiMinMaxImpact / ki);

  prevSteeringWheelAngleError = steeringWheelAngle;

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
  int8_t pixyResult, pixy_1_result, pixy_2_result;
  uint32_t pixy_1_loopIterationsCountNoVectorDetected, pixy_2_loopIterationsCountNoVectorDetected, loopIterationsCountPixyChangeProgramError;
  LineABC mirrorLine;
  Vector vec, pixy_1_leftVectorOld, pixy_1_rightVectorOld;
  Vector pixy_2_leftVectorOld, pixy_2_rightVectorOld;
  std::vector<Vector> vectors;
  std::vector<Intersection> intersections;
  std::vector<char> serialInputBuffer;
  PurePursuitInfo purePersuitInfo;
  Point2D carPosition;
  float laneWidth, lookAheadDistance, frontObstacleDistance;
  float timeStart;
  float max_speed_original;
  MovingAverage movingAverage_speed(10);
  
  int consecutiveValidFinishLines = 0;

  pixy_1_result = PIXY_RESULT_ERROR;
  pixy_2_result = PIXY_RESULT_ERROR;
  serialInputBuffer.clear();

  timeStart = 0.0f;
  pixy_1_loopIterationsCountNoVectorDetected = 0;
  pixy_1_loopIterationsCountNoVectorDetected = 0;
  max_speed_original = g_max_speed;

  frontObstacleDistance = 0.0f;

  mirrorLine = xAxisABC();
  mirrorLine.C = -(((float)IMAGE_MAX_Y) / 2.0f);

  carPosition.x = (float)SCREEN_CENTER_X;
  carPosition.y = 0.0f;

  g_middle_lane_line_pixy_1 = yAxisABC();

  laneWidth = (float)g_lane_width_vector_unit;
  
  lookAheadDistance = (float)g_lookahead_min_distance_cm * (float)VECTOR_UNIT_PER_CM;
  
  g_pixy_1_vectors_processing.setCarPosition(carPosition);
  g_pixy_1_vectors_processing.setLaneWidth(laneWidth);
  g_pixy_1_vectors_processing.setMinXaxisAngle(g_min_x_axis_angle_vector * RADIANS_PER_DEGREE);

/*
  pixy_2_vectorsProcessing.setCarPosition(carPosition);
  pixy_2_vectorsProcessing.setLaneWidth(laneWidth);
  pixy_2_vectorsProcessing.setMinXaxisAngle(g_min_x_axis_angle_vector * RADIANS_PER_DEGREE);
*/
  for (;;)
  {
    timeStart = (float)millis();
    movingAverage_speed.next(g_car_speed);
    g_pixy_1_vectors_processing.setMinXaxisAngle(g_min_x_axis_angle_vector * RADIANS_PER_DEGREE);
    //pixy_2_vectorsProcessing.setMinXaxisAngle(g_min_x_axis_angle_vector * RADIANS_PER_DEGREE);

    remote_control_routine();


    if (g_enable_car_engine == 0) {
      consecutiveValidFinishLines = 0;
      g_finish_line_detected = 0;
      g_finish_line_detected_now = 0;
      driverMotor.write((int)STANDSTILL_SPEED);
    }
    
    #if ENABLE_EMERGENCY_BREAKING == 1   // handling emergency braking

    if (g_enable_emergency_brake != 0 && (g_enable_distance_sensor1 != 0 || g_enable_distance_sensor2 != 0 || g_enable_distance_sensor3 != 0)) {
      
    if (g_emergency_brake_enable_delay_started_count == 0 && g_enable_car_engine != 0) {
      max_speed_original = g_max_speed;
      g_emergency_brake_enable_remaining_delay_s = g_emergency_brake_enable_delay_s;
      g_emergency_brake_enable_delay_started_count = 1;
    }
    else if(g_emergency_brake_enable_delay_started_count != 0 && g_enable_car_engine == 0){
      g_max_speed = max_speed_original;
      g_emergency_brake_enable_remaining_delay_s = 0.0f;
      g_emergency_brake_enable_delay_started_count = 0.0f;
    }
    
    if (g_enable_car_engine != 0 && g_emergency_brake_enable_remaining_delay_s > 0.0f) {
      g_emergency_brake_enable_remaining_delay_s -= (g_loop_time_ms / 1000.0f);
      g_emergency_brake_enable_remaining_delay_s = MAX(g_emergency_brake_enable_remaining_delay_s, 0.0f);

      if (g_emergency_brake_enable_remaining_delay_s <= 0.0f) {
        max_speed_original = g_max_speed;
        g_max_speed = g_max_speed_after_emergency_brake_delay;
      }
    }
    if (g_emergency_brake_enable_remaining_delay_s <= 0.0f)
    {
      
      frontObstacleDistance = getFrontObstacleDistance_cm();

      if (frontObstacleDistance <= g_emergency_break_distance_cm ) {
        digitalWrite(EMERGENCY_BREAK_LIGHT_PIN, HIGH);
        g_emergency_break_active = 1;
        g_emergency_break_loops_count++;

        #if ENABLE_SERIAL_PRINT == 1
          SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("EMRG_BRK loop: ") + String(g_emergency_break_loops_count));
        #endif

        if (g_emergency_break_loops_count == 1) {

          #if ENABLE_DRIVERMOTOR == 1 &&  ENABLE_EMERGENCYBRAKE_BACKWARDSBRAKE == 1 // use brakes to get to a near standstill
            if (g_enable_car_engine != 0) {
              float tempCarSpeed = movingAverage_speed.next(g_car_speed);
              float startTime_ = (float)millis();
              float brakeTime_ = (float)fabsf((tempCarSpeed - (float)STANDSTILL_SPEED)) * (40.0f / (107.0f - 90.0f));
              int brakeSpeed_ = (int)((float)STANDSTILL_SPEED - fabsf(tempCarSpeed - (float)STANDSTILL_SPEED));

              while ((((float)millis()) - startTime_) < brakeTime_) {
                driverMotor.write(brakeSpeed_);
              }
            }
          #endif
          g_car_speed = (float)g_emergency_brake_min_speed;  
        }
        
        if(frontObstacleDistance <= g_emergency_brake_distance_from_obstacle_cm){
          g_car_speed = (float)STANDSTILL_SPEED;
        }
        else{
          g_car_speed = (float)g_emergency_brake_min_speed;
        }
        #if ENABLE_DRIVERMOTOR == 1
          if (g_enable_car_engine != 0) {
            driverMotor.write((int)g_car_speed);
          }
        #endif

      }
      else{
        g_emergency_break_active = 0;
        g_emergency_break_loops_count = 0;
        digitalWrite(EMERGENCY_BREAK_LIGHT_PIN, LOW);
      }
      }
      else{
      g_emergency_break_active = 0;
      g_emergency_break_loops_count = 0;
      digitalWrite(EMERGENCY_BREAK_LIGHT_PIN, LOW);
    }
    }
    else{
      g_emergency_break_active = 0;
      g_emergency_break_loops_count = 0;
      digitalWrite(EMERGENCY_BREAK_LIGHT_PIN, LOW);
    }

    #endif

    #if ENABLE_SERIAL_PRINT == 1
      if(readRecordFromSerial(SERIAL_PORT, String("\r\n"), serialInputBuffer)){
        SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("Input: ") + String(serialInputBuffer.data()));
        parseAndSetGlobalVariables(serialInputBuffer, ';');
        printGlobalVariables(SERIAL_PORT);
        serialInputBuffer.clear();
      }
    #endif

    #if ENABLE_SETTINGS_MENU == 1
      settingsMenuRoutine();
    #endif

    if (g_enable_car_steering_wheel == 0 && g_enable_car_engine != 0) {
      g_enable_car_steering_wheel = 1;
    }

    g_pixy_1_vectors_processing.clear();

    pixy_1_result = PIXY_RESULT_ERROR;

    pixy_1_result = g_pixy_1.line.getAllFeatures(LINE_VECTOR /*| LINE_INTERSECTION*/, true);
    
/*===================================================START first camera============================================================================*/
    if(pixy_1_result >= (int8_t)0){
      vectors.resize(g_pixy_1.line.numVectors);
      memcpy(vectors.data(), g_pixy_1.line.vectors, (g_pixy_1.line.numVectors * sizeof(Vector)));
      //intersections.resize(g_pixy_1.line.numIntersections);
      //memcpy(intersections.data(), g_pixy_1.line.intersections, (g_pixy_1.line.numIntersections * sizeof(Intersection)));
      intersections.clear();
      VectorsProcessing::findIntersections(vectors, intersections);
      VectorsProcessing::filterVectorIntersections(vectors, intersections);

      pixy_1_loopIterationsCountNoVectorDetected = 0;
      if (vectors.size() > 0){
        pixy_1_loopIterationsCountNoVectorDetected = 0;
      }
      else{
        pixy_1_loopIterationsCountNoVectorDetected++;
      }

      for (i=0; i < vectors.size(); i++)
      {
        vec = vectors[i];
        vec = VectorsProcessing::mirrorVector(mirrorLine, vec);
        vec = VectorsProcessing::reComputeVectorStartEnd_basedOnDistanceOfPointXaxis(vec, carPosition);
        g_pixy_1_vectors_processing.addVector(vec);
      }
      pixy_1_leftVectorOld = g_pixy_1_vectors_processing.getLeftVector();
      pixy_1_rightVectorOld = g_pixy_1_vectors_processing.getRightVector();

      vectors.resize(g_pixy_1.line.numVectors);
      for (size_t i = 0; i < g_pixy_1.line.numVectors; i++) {
        vectors[i] = VectorsProcessing::mirrorVector(mirrorLine, g_pixy_1.line.vectors[i]);
      }

      #if ENABLE_FINISH_LINE_DETECTION == 1
        if (g_enable_finish_line_detection != 0) {
          g_finish_line = VectorsProcessing::findStartFinishLine(vectors, g_pixy_1_vectors_processing.getLeftVector(), g_pixy_1_vectors_processing.getRightVector(), g_pixy_1_vectors_processing.getMiddleLine(), g_finish_line_angle_tolerance);
          if (VectorsProcessing::isFinishLineValid(g_finish_line)) {
            consecutiveValidFinishLines += 1;
            g_finish_line_detected_now = 1;
            if (consecutiveValidFinishLines >= 5) {
              g_finish_line_detected = 1;
            }
          }
          else{
            consecutiveValidFinishLines = 0;
            g_finish_line_detected_now = 0;
            memset(&g_finish_line, 0, sizeof(g_finish_line));
          }
        }
      #endif
      
      #if ENABLE_PIXY_VECTOR_APPROXIMATION == 1
      if(g_emergency_break_active == 0 && g_enable_pixy_vector_approximation != 0){
        if (((int)g_pixy_1_vectors_processing.isVectorValid(pixy_1_rightVectorOld) + (int)g_pixy_1_vectors_processing.isVectorValid(pixy_1_leftVectorOld))==1){
          if (g_emergency_break_active == 0){
            g_car_speed = (float)g_min_speed;
          }
          #if ENABLE_DRIVERMOTOR == 1
            if (g_enable_car_engine != 0) {
              driverMotor.write((int)g_car_speed);
            }
          #endif

          loopIterationsCountPixyChangeProgramError=0;
          while ((pixyResult = g_pixy_1.changeProg("video")) != PIXY_RESULT_OK) {
            loopIterationsCountPixyChangeProgramError++;
            FailureModeMessage(g_pixy_1, loopIterationsCountPixyChangeProgramError,"pixy video");
            delay(10);
          }
          delay(40);

          vec = VectorsProcessing::mirrorVector(mirrorLine, pixy_1_leftVectorOld);
          approximatePixyVectorVector(g_pixy_1, vec, g_black_color_treshold, mirrorImageABC(mirrorLine, carPosition));
          vec = VectorsProcessing::mirrorVector(mirrorLine, vec);
          g_pixy_1_vectors_processing.setLeftVector(vec);

          vec = VectorsProcessing::mirrorVector(mirrorLine, pixy_1_rightVectorOld);
          approximatePixyVectorVector(g_pixy_1, vec, g_black_color_treshold, mirrorImageABC(mirrorLine, carPosition));
          vec = VectorsProcessing::mirrorVector(mirrorLine, vec);
          g_pixy_1_vectors_processing.setRightVector(vec);
          
          loopIterationsCountPixyChangeProgramError = 0;
          while ((pixyResult = g_pixy_1.changeProg("line")) != PIXY_RESULT_OK) {
            loopIterationsCountPixyChangeProgramError++;
            FailureModeMessage(g_pixy_1, loopIterationsCountPixyChangeProgramError,"pixy line");
            delay(10);
          }
          delay(40);
        }
      }
      #endif
    }
    else{
      pixy_1_loopIterationsCountNoVectorDetected++;
      FailureModeMessage(g_pixy_1, pixy_1_loopIterationsCountNoVectorDetected,"pixy getAllFeatures");
    }
/*===================================================END first camera============================================================================*/

/*===================================================START second camera============================================================================*/
    /*
    if(pixy_2_result >= (int8_t)0){
      vectors.resize(pixy_2.line.numVectors);
      memcpy(vectors.data(), pixy_2.line.vectors, (pixy_2.line.numVectors * sizeof(Vector)));
      intersections.resize(pixy_2.line.numIntersections);
      memcpy(intersections.data(), pixy_2.line.intersections, (pixy_2.line.numIntersections * sizeof(Intersection)));

      VectorsProcessing::filterVectorIntersections(vectors, intersections);

      pixy_2_loopIterationsCountNoVectorDetected = 0;
      if (vectors.size() > 0){
        pixy_2_loopIterationsCountNoVectorDetected = 0;
      }
      else{
        pixy_2_loopIterationsCountNoVectorDetected++;
      }

      for (i=0; i < vectors.size(); i++)
      {
        vec = vectors[i];
        vec = VectorsProcessing::mirrorVector(mirrorLine, vec);
        vec = VectorsProcessing::reComputeVectorStartEnd_basedOnDistanceOfPointXaxis(vec, carPosition);
        pixy_2_vectorsProcessing.addVector(vec);
      }
      pixy_2_leftVectorOld = pixy_2_vectorsProcessing.getLeftVector();
      pixy_2_rightVectorOld = pixy_2_vectorsProcessing.getRightVector();

      #if ENABLE_PIXY_VECTOR_APPROXIMATION == 1
      if(g_emergency_break_active == 0 && g_enable_pixy_vector_approximation != 0){
        if (((int)pixy_2_vectorsProcessing.isVectorValid(pixy_2_rightVectorOld) + (int)pixy_2_vectorsProcessing.isVectorValid(pixy_2_leftVectorOld))==1){
          if (g_emergency_break_active == 0){
            g_car_speed = (float)g_min_speed;
          }
          #if ENABLE_DRIVERMOTOR == 1
            if (g_enable_car_engine != 0) {
              driverMotor.write((int)g_car_speed);
            }
          #endif

          loopIterationsCountPixyChangeProgramError=0;
          while ((pixyResult = pixy_2.changeProg("video")) != PIXY_RESULT_OK) {
            loopIterationsCountPixyChangeProgramError++;
            FailureModeMessage(pixy_2, loopIterationsCountPixyChangeProgramError,"pixy video");
            delay(10);
          }
          delay(40);

          vec = VectorsProcessing::mirrorVector(mirrorLine, pixy_2_leftVectorOld);
          approximatePixyVectorVector(pixy_2, vec, g_black_color_treshold, mirrorImageABC(mirrorLine, carPosition));
          vec = VectorsProcessing::mirrorVector(mirrorLine, vec);
          pixy_2_vectorsProcessing.setLeftVector(vec);

          vec = VectorsProcessing::mirrorVector(mirrorLine, pixy_2_rightVectorOld);
          approximatePixyVectorVector(pixy_2, vec, g_black_color_treshold, mirrorImageABC(mirrorLine, carPosition));
          vec = VectorsProcessing::mirrorVector(mirrorLine, vec);
          pixy_2_vectorsProcessing.setRightVector(vec);
          
          loopIterationsCountPixyChangeProgramError = 0;
          while ((pixyResult = pixy_2.changeProg("line")) != PIXY_RESULT_OK) {
            loopIterationsCountPixyChangeProgramError++;
            FailureModeMessage(pixy_2, loopIterationsCountPixyChangeProgramError,"pixy line");
            delay(10);
          }
          delay(40);
        }
      }
      #endif
    }
    else{
      pixy_2_loopIterationsCountNoVectorDetected++;
      FailureModeMessage(pixy_2, pixy_2_loopIterationsCountNoVectorDetected,"pixy getAllFeatures");
    }
    */
    /*===================================================END second camera============================================================================*/


    g_middle_lane_line_pixy_1 = g_pixy_1_vectors_processing.getMiddleLine();
    lookAheadDistance = calculateLookAheadDistance(g_lookahead_min_distance_cm * VECTOR_UNIT_PER_CM, g_lookahead_max_distance_cm * VECTOR_UNIT_PER_CM, g_middle_lane_line_pixy_1);
    purePersuitInfo = purePursuitComputeABC(carPosition, g_middle_lane_line_pixy_1, g_car_length_vector_unit, lookAheadDistance);
    purePersuitInfo.steeringAngle -= (g_steering_wheel_angle_offset * RADIANS_PER_DEGREE);


    if (pixy_1_loopIterationsCountNoVectorDetected > 15)
    {
      if (g_emergency_break_active == 0) {
        g_car_speed = (float)g_min_speed;
      }
      #if ENABLE_DRIVERMOTOR == 1
        if (g_enable_car_engine != 0) {
          driverMotor.write((int)g_car_speed);
        }
      #endif
    }
    else{
      if (g_emergency_break_active == 0){
        g_car_speed = calculateCarSpeed((float)g_min_speed, g_max_speed, (float)STEERING_SERVO_MAX_ANGLE, degrees(purePersuitInfo.steeringAngle), g_middle_lane_line_pixy_1, g_car_speed_ki, g_car_speed_kd, g_car_speed_ki_min_max_impact);
      }
    }
        
    #if ENABLE_STEERING_SERVO == 1
      if (g_enable_car_steering_wheel != 0) {
        g_steering_wheel.setSteeringAngleDeg(degrees(purePersuitInfo.steeringAngle));
      }
    #endif

    #if ENABLE_DRIVERMOTOR == 1
      if (g_enable_car_engine != 0) {
        driverMotor.write((int)g_car_speed);
      }
    #endif
    
    g_left_lane_line_pixy_1 = VectorsProcessing::vectorToLineABC(g_pixy_1_vectors_processing.getLeftVector());
    g_right_lane_line_pixy_1 = VectorsProcessing::vectorToLineABC(g_pixy_1_vectors_processing.getRightVector());

    g_loop_time_ms = ((float)millis()) - timeStart;
    g_loop_time_ms = MAX(g_loop_time_ms, 0.0f);
    g_time_passed_ms += g_loop_time_ms;

    #if ENABLE_SERIAL_PRINT == 1
        printDataToSerial(SERIAL_PORT, pixy_1_leftVectorOld, pixy_1_rightVectorOld, g_pixy_1_vectors_processing.getLeftVector(), g_pixy_1_vectors_processing.getRightVector(), VectorsProcessing::vectorToLineABC(g_pixy_1_vectors_processing.getLeftVector()), VectorsProcessing::vectorToLineABC(g_pixy_1_vectors_processing.getRightVector()), g_middle_lane_line_pixy_1, purePersuitInfo, (g_car_speed - (float)STANDSTILL_SPEED) / (float)(g_max_speed - STANDSTILL_SPEED), frontObstacleDistance, g_car_speed);
    #endif
    
    /*
    #if ENABLE_SERIAL_PRINT == 1
      SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("LoopTime: ") + String(g_loop_time_ms) + String(" ms"));
    #endif
    */
  }
}



