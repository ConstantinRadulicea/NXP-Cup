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
#define RX_BUFFER_SIZE 4092
#define TX_BUFFER_SIZE 16384
static char RX_BUFFER[RX_BUFFER_SIZE];
static char TX_BUFFER[TX_BUFFER_SIZE];

/*====================================================================================================================================*/

void FailureModeMessage(Pixy2 &pixy, int iteration, String errorText){
  #if ENABLE_SERIAL_PRINT == 1
    SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("iters [") + String(iteration) + String("] ERROR: " + errorText));
  #endif
  if (iteration >= 10){  
    g_car_speed = (float)STANDSTILL_SPEED;
  do{
      #if ENABLE_DRIVERMOTOR == 1
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          g_powertrain.SetSpeedRequest((int)STANDSTILL_SPEED, 0.0, 0);
        }
      #endif
      #if ENABLE_STEERING_SERVO == 1
        g_steering_angle = 0.0f;
        g_steering_wheel.setSteeringAngleDeg(-g_steering_wheel_angle_offset_deg);
      #endif
      delay(10);
    } while (pixy.init() < ((int8_t)0));
  }
}

/*==============================================================================*/

void setup() {
  int8_t pixyResult;
  //Serial.begin(SERIAL_PORT_BAUD_RATE);
  // Initialization and attachment of the servo and motor
  #if ENABLE_STEERING_SERVO == 1
    pinMode(STEERING_SERVO_PIN, OUTPUT);
    g_steering_wheel.attach(STEERING_SERVO_PIN);
    g_steering_angle = 0.0f;
    g_steering_wheel.setSteeringAngleDeg(-g_steering_wheel_angle_offset_deg);
  #endif

  #if ENABLE_DRIVERMOTOR == 1
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      PowerTrainSetup(WHEEL_DIAMETER_M, DISTANCE_BETWEEN_WHEELS_M, POWERTRAIN_PID_FREQUENCY_HZ, LEFT_WHEEL_MOTOR_PIN, RIGHT_WHEEL_MOTOR_PIN, RPM_SENSOR_LEFT_WHEEL_PIN, RPM_SENSOR_RIGHT_WHEEL_PIN);
      g_powertrain.SetLeftWheelPID(g_powertrain_left_wheel_kp, g_powertrain_left_wheel_ki, g_powertrain_left_wheel_kd, g_powertrain_left_wheel_ki_max_sum);
      g_powertrain.SetRightWheelPID(g_powertrain_right_wheel_kp, g_powertrain_right_wheel_ki, g_powertrain_right_wheel_kd, g_powertrain_right_wheel_ki_max_sum);
    }
  #endif


  #if ENABLE_EMERGENCY_BREAKING == 1

   DistanceSensorsSetupAnalog(
    DISTANCE_SENSOR1_ANALOG_PIN,
    DISTANCE_SENSOR2_ANALOG_PIN,
    DISTANCE_SENSOR3_ANALOG_PIN
    );
   
   pinMode(EMERGENCY_BREAK_LIGHT_PIN, OUTPUT);
   digitalWrite(EMERGENCY_BREAK_LIGHT_PIN, LOW);
  #endif

  #if ENABLE_SETTINGS_MENU != 0
    LcdMenuSetup(MENU_LEFT_ARROW_BUTTON_PIN, MENU_RIGHT_ARROW_BUTTON_PIN, MENU_INCREMENT_BUTTON_PIN, MENU_DECREMENT_BUTTON_PIN);
  #endif

  #if ENABLE_REMOTE_START_STOP == 1
    pinMode(REMOTE_STOP_PIN, INPUT);
    pinMode(REMOTE_START_PIN, INPUT);
  #endif

  // serial Initialization
  #if ENABLE_SERIAL_PRINT == 1 || ENABLE_WIRELESS_DEBUG == 1
  #if SERIAL_PORT_TYPE_CONFIGURATION == 1
    SERIAL_PORT.addMemoryForRead(RX_BUFFER, RX_BUFFER_SIZE);
    SERIAL_PORT.addMemoryForWrite(TX_BUFFER, TX_BUFFER_SIZE);
  #endif
    SERIAL_PORT.begin(SERIAL_PORT_BAUD_RATE);
    while (!SERIAL_PORT){
      delay(50);
    }
  #endif

  #if ENABLE_WIRELESS_DEBUG == 1
    serial2WifiConnect(SERIAL_PORT, String(DEBUG_WIFI_INIT_SEQUENCE), String(DEBUG_WIFI_SSID), String(DEBUG_WIFI_PASSWORD), String(DEBUG_HOST_IPADDRESS), DEBUG_HOST_PORT);
    //printSerial2WifiInfo(Serial, String(DEBUG_WIFI_INIT_SEQUENCE), String(DEBUG_WIFI_SSID), String(DEBUG_WIFI_PASSWORD), String(DEBUG_HOST_IPADDRESS), DEBUG_HOST_PORT);
  #endif

  pixyResult = g_pixy_1.init();
  #if ENABLE_SERIAL_PRINT == 1
    SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("g_pixy_1.init() = ") + String(pixyResult));
  #endif

  #if CAMERA_ILLUMINATION_LIGHT != 0
    g_pixy_1.setLamp(1,1);
  #else
    g_pixy_1.setLamp(0,0);
  #endif
    
  // Getting the RGB pixel values requires the 'video' program
  pixyResult = g_pixy_1.changeProg("line");
  #if ENABLE_SERIAL_PRINT == 1
    SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("g_pixy_1.changeProg(line) = ") + String(pixyResult));
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
  
  PurePursuitInfo purePersuitInfo;
  Point2D carPosition;
  float laneWidth, lookAheadDistance, frontObstacleDistance_m;
  float timeStart;
  float max_speed_original;
  MovingAverage movingAverage_speed(10);
  
  int consecutiveValidFinishLines = 0;

  pixy_1_result = PIXY_RESULT_ERROR;
  pixy_2_result = PIXY_RESULT_ERROR;

  timeStart = 0.0f;
  pixy_1_loopIterationsCountNoVectorDetected = 0;
  pixy_1_loopIterationsCountNoVectorDetected = 0;
  max_speed_original = g_max_speed;

  frontObstacleDistance_m = 0.0f;

  mirrorLine = xAxisABC();
  mirrorLine.C = -(((float)IMAGE_MAX_Y) / 2.0f);

  carPosition.x = (float)SCREEN_CENTER_X;
  carPosition.y = 0.0f;

  g_middle_lane_line_pixy_1 = yAxisABC();

  laneWidth = (float)g_lane_width_vector_unit;
  
  lookAheadDistance = (float)MeterToVectorUnit(g_lookahead_min_distance_cm/100.0f);
  
  g_pixy_1_vectors_processing.setCarPosition(carPosition);
  g_pixy_1_vectors_processing.setLaneWidth(laneWidth);
  
  g_pixy_1_vectors_processing.setMinXaxisAngle(radians(g_min_x_axis_angle_vector));

/*
  pixy_2_vectorsProcessing.setCarPosition(carPosition);
  pixy_2_vectorsProcessing.setLaneWidth(laneWidth);
  pixy_2_vectorsProcessing.setMinXaxisAngle(radians(g_min_x_axis_angle_vector));
*/
  for (;;)
  {
    timeStart = (float)millis();

    #if ENABLE_SERIAL_PRINT == 1
    parseInputGlobalVariablesRoutine(SERIAL_PORT);
    #endif

    #if ENABLE_SETTINGS_MENU == 1
      settingsMenuRoutine();
    #endif


    movingAverage_speed.next(g_car_speed);
    g_pixy_1_vectors_processing.setMinXaxisAngle(radians(g_min_x_axis_angle_vector));
    //pixy_2_vectorsProcessing.setMinXaxisAngle(radians(g_min_x_axis_angle_vector));

    remote_control_routine();


    if (g_enable_car_engine == 0) {
      consecutiveValidFinishLines = 0;
      g_finish_line_detected = 0;
      g_finish_line_detected_now = 0;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        g_powertrain.SetSpeedRequest(STANDSTILL_SPEED, 0.0, 0);
      }
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
      g_emergency_brake_enable_delay_started_count = 0;
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
      
      frontObstacleDistance_m = getFrontObstacleDistanceAnalog_m();

      if (frontObstacleDistance_m <= g_emergency_brake_distance_m ) {
        digitalWrite(EMERGENCY_BREAK_LIGHT_PIN, HIGH);
        g_emergency_break_active = 1;
        g_emergency_break_loops_count++;

        #if ENABLE_SERIAL_PRINT == 1
          SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("EMRG_BRK loop: ") + String(g_emergency_break_loops_count));
        #endif
        
        if(frontObstacleDistance_m <= g_emergency_brake_distance_from_obstacle_m){
          g_car_speed = (float)STANDSTILL_SPEED;
        }
        else{
          g_car_speed = (float)g_emergency_brake_min_speed;
        }
        #if ENABLE_DRIVERMOTOR == 1
          if (g_enable_car_engine != 0) {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
              g_powertrain.SetSpeedRequest(g_car_speed, VectorUnitToMeter(purePersuitInfo.turnRadius), SteeringWheel::AngleToDirectionDeg(g_steering_angle));
            }
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


    if (g_enable_car_steering_wheel == 0 && g_enable_car_engine != 0) {
      g_enable_car_steering_wheel = 1;
    }

    g_pixy_1_vectors_processing.clear();

    pixy_1_result = PIXY_RESULT_ERROR;

    pixy_1_result = g_pixy_1.line.getAllFeatures(LINE_VECTOR /*| LINE_INTERSECTION*/, true);
    SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("pixy_1_result: ") + String(pixy_1_result));

    
/*===================================================START first camera============================================================================*/
    if(pixy_1_result >= ((int8_t)0)){
      SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("vector: ") + String(g_pixy_1.line.numVectors));
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
              ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                g_powertrain.SetSpeedRequest(g_car_speed, VectorUnitToMeter(purePersuitInfo.turnRadius), SteeringWheel::AngleToDirectionDeg(g_steering_angle));
              }            
            }
          #endif

          loopIterationsCountPixyChangeProgramError=0;
          while ((pixyResult = g_pixy_1.changeProg("video")) < ((int8_t)0)) {
            loopIterationsCountPixyChangeProgramError++;
            FailureModeMessage(g_pixy_1, loopIterationsCountPixyChangeProgramError,"pixy video");
            delay(10);
          }
          //delay(40);

          vec = VectorsProcessing::mirrorVector(mirrorLine, pixy_1_leftVectorOld);
          approximatePixyVectorVector(g_pixy_1, vec, g_black_color_treshold, mirrorImageABC(mirrorLine, carPosition));
          vec = VectorsProcessing::mirrorVector(mirrorLine, vec);
          g_pixy_1_vectors_processing.setLeftVector(vec);

          vec = VectorsProcessing::mirrorVector(mirrorLine, pixy_1_rightVectorOld);
          approximatePixyVectorVector(g_pixy_1, vec, g_black_color_treshold, mirrorImageABC(mirrorLine, carPosition));
          vec = VectorsProcessing::mirrorVector(mirrorLine, vec);
          g_pixy_1_vectors_processing.setRightVector(vec);
          
          loopIterationsCountPixyChangeProgramError = 0;
          while ((pixyResult = g_pixy_1.changeProg("line")) < ((int8_t)0)) {
            loopIterationsCountPixyChangeProgramError++;
            FailureModeMessage(g_pixy_1, loopIterationsCountPixyChangeProgramError,"pixy line");
            delay(10);
          }
          //delay(40);
        }
      }
      #endif
    }
    else{
      pixy_1_loopIterationsCountNoVectorDetected++;
      FailureModeMessage(g_pixy_1, pixy_1_loopIterationsCountNoVectorDetected,"pixy getAllFeatures");
    }
/*===================================================END first camera============================================================================*/


    g_middle_lane_line_pixy_1 = g_pixy_1_vectors_processing.getMiddleLine();
    lookAheadDistance = calculateLookAheadDistance(MeterToVectorUnit(g_lookahead_min_distance_cm/100.0f), MeterToVectorUnit(g_lookahead_max_distance_cm/100.0f), g_middle_lane_line_pixy_1);
    purePersuitInfo = purePursuitComputeABC(carPosition, g_middle_lane_line_pixy_1, g_car_length_vector_unit, lookAheadDistance);
    g_steering_angle = g_steering_wheel.vaildSteeringAngleDeg(purePersuitInfo.steeringAngle);
    //SERIAL_PORT.println(String("% steeringAngle: ") + String(purePersuitInfo.steeringAngle, 5));

    if (pixy_1_loopIterationsCountNoVectorDetected > 15)
    {
      #if ENABLE_SERIAL_PRINT != 0
        SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("No vector detected: ") + String(pixy_1_loopIterationsCountNoVectorDetected));
      #endif
      if (g_emergency_break_active == 0) {
        g_car_speed = (float)g_min_speed;
      }
    }
    else{
      if (g_emergency_break_active == 0){
        g_car_speed = calculateCarSpeed((float)g_min_speed, g_max_speed, (float)STEERING_SERVO_MAX_ANGLE, degrees(g_steering_angle), g_middle_lane_line_pixy_1, g_car_speed_ki, g_car_speed_kd, g_car_speed_ki_min_max_impact);
      }
    }
    
    #if ENABLE_STEERING_SERVO == 1
      if (g_enable_car_steering_wheel != 0) {
        g_steering_wheel.setSteeringAngleDeg(degrees(g_steering_angle) - g_steering_wheel_angle_offset_deg);
      }
    #endif

    #if ENABLE_DRIVERMOTOR == 1
      if (g_enable_car_engine != 0) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          g_powertrain.SetSpeedRequest(g_car_speed, VectorUnitToMeter(purePersuitInfo.turnRadius), SteeringWheel::AngleToDirectionDeg(degrees(g_steering_angle)));
        }
      }
    #endif
    
    g_left_lane_line_pixy_1 = VectorsProcessing::vectorToLineABC(g_pixy_1_vectors_processing.getLeftVector());
    g_right_lane_line_pixy_1 = VectorsProcessing::vectorToLineABC(g_pixy_1_vectors_processing.getRightVector());

    g_loop_time_ms = ((float)millis()) - timeStart;
    g_loop_time_ms = MAX(g_loop_time_ms, 0.0f);
    g_time_passed_ms += g_loop_time_ms;

    #if ENABLE_SERIAL_PRINT == 1
        //SERIAL_PORT.println(String("%g_car_speed: ") + String(g_car_speed, 5));
        //SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("Speed: ") + String(g_car_speed));
        printDataToSerial(SERIAL_PORT, pixy_1_leftVectorOld, pixy_1_rightVectorOld, g_pixy_1_vectors_processing.getLeftVector(), g_pixy_1_vectors_processing.getRightVector(), VectorsProcessing::vectorToLineABC(g_pixy_1_vectors_processing.getLeftVector()), VectorsProcessing::vectorToLineABC(g_pixy_1_vectors_processing.getRightVector()), g_middle_lane_line_pixy_1, purePersuitInfo, (g_car_speed - (float)STANDSTILL_SPEED) / (float)(g_max_speed - STANDSTILL_SPEED), frontObstacleDistance_m, g_car_speed);
    #endif
  }
}



