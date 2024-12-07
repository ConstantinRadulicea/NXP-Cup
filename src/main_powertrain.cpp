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

#include "setup.h"
#include "features/automatic_emergency_braking.h"
#include "features/finish_line_detection.h"

/*====================================================================================================================================*/

void loop() {
  size_t i;
  int8_t pixy_1_result;
  uint32_t pixy_1_loopIterationsCountNoVectorDetected;
  LineABC mirrorLine;
  Vector vec, pixy_1_leftVectorOld, pixy_1_rightVectorOld;
  std::vector<Vector> vectors;
  std::vector<Intersection> intersections;
  PurePursuitInfo purePersuitInfo;
  Point2D carPosition;
  Point2D centerRearAxeCarPosition_vectorUnit;
  float laneWidth, lookAheadDistance;
  float timeStart;
  float speed_request_mps;

  AEB_out_t AEB_out;
  FLD_out_t FLD_out;

  memset(&AEB_out, 0, sizeof(AEB_out_t));
  memset(&FLD_out, 0, sizeof(FLD_out_t));

  pixy_1_result = PIXY_RESULT_ERROR;

  timeStart = 0.0f;
  pixy_1_loopIterationsCountNoVectorDetected = 0;
  pixy_1_loopIterationsCountNoVectorDetected = 0;

  mirrorLine = xAxisABC();
  mirrorLine.C = -(((float)IMAGE_MAX_Y) / 2.0f);

  carPosition.x = (float)SCREEN_CENTER_X;
  carPosition.y = 0.0f;

  g_middle_lane_line_pixy_1 = yAxisABC();

  laneWidth = (float)g_lane_width_vector_unit;
  
  lookAheadDistance = (float)MeterToVectorUnit(g_lookahead_min_distance_cm/100.0f);
  
  g_pixy_1_vectors_processing.setCarPosition(carPosition);
  g_pixy_1_vectors_processing.setLaneWidth(laneWidth);
  
  g_pixy_1_vectors_processing.setMinXaxisAngle(radians(g_min_x_axis_angle_vector_deg));

/*
  pixy_2_vectorsProcessing.setCarPosition(carPosition);
  pixy_2_vectorsProcessing.setLaneWidth(laneWidth);
  pixy_2_vectorsProcessing.setMinXaxisAngle(radians(g_min_x_axis_angle_vector_deg));
*/
  for (;;)
  {
    timeStart = (float)millis();
    EnableSlowSpeedAfterDelay(&g_max_speed_delay_passed, g_max_speed_after_delay_s);
    #if ENABLE_STEERING_SERVO == 1
      g_steering_wheel.SetRawAngleOffset(g_steering_wheel_angle_offset_deg);
    #endif
    #if ENABLE_WIRELESS_DEBUG == 1
    parseInputGlobalVariablesRoutine(SERIAL_PORT);
    #endif

    #if ENABLE_SETTINGS_MENU == 1
      settingsMenuRoutine();
    #endif

    g_pixy_1_vectors_processing.setMinXaxisAngle(radians(g_min_x_axis_angle_vector_deg));
    //pixy_2_vectorsProcessing.setMinXaxisAngle(radians(g_min_x_axis_angle_vector_deg));

    if (g_enable_car_engine == 0) {
      FLD_deactivate();
      #if ENABLE_DRIVERMOTOR == 1
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          g_powertrain.SetSpeedRequest_slow(STANDSTILL_SPEED, 0.0, 0, g_max_acceleration, g_max_deceleration);
        }
      #endif
    }

    #if ENABLE_STEERING_SERVO == 1
      if (g_enable_car_steering_wheel == 0) {
        g_steering_wheel.setSteeringWheelAngleDeg(0.0f);
      }
    #endif

    if (g_enable_car_steering_wheel == 0 && g_enable_car_engine != 0) {
      g_enable_car_steering_wheel = 1;
    }

    g_pixy_1_vectors_processing.clear();

    pixy_1_result = PIXY_RESULT_ERROR;

    pixy_1_result = g_pixy_1.line.getAllFeatures(LINE_VECTOR /*| LINE_INTERSECTION*/, true);
    //SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("pixy_1_result: ") + String(pixy_1_result));

    
/*===================================================START first camera============================================================================*/
    if(pixy_1_result >= ((int8_t)0)){
      //SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("vector: ") + String(g_pixy_1.line.numVectors));
      vectors.resize(g_pixy_1.line.numVectors);
      memcpy(vectors.data(), g_pixy_1.line.vectors, (g_pixy_1.line.numVectors * sizeof(Vector)));
      //intersections.resize(g_pixy_1.line.numIntersections);
      //memcpy(intersections.data(), g_pixy_1.line.intersections, (g_pixy_1.line.numIntersections * sizeof(Intersection)));
      intersections.clear();
      VectorsProcessing::findIntersections(vectors, intersections);
      VectorsProcessing::filterVectorIntersections(vectors, intersections);

      //pixy_1_loopIterationsCountNoVectorDetected = 0;
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
        if (g_start_line_calibration_acquisition == 0) {
          vec = calibrateVector(vec, g_line_calibration_data);
        }
        vec = VectorsProcessing::reComputeVectorStartEnd_basedOnDistanceOfPointXaxis(vec, carPosition);
        g_pixy_1_vectors_processing.addVector(vec);
      }
      pixy_1_leftVectorOld = g_pixy_1_vectors_processing.getLeftVector();
      pixy_1_rightVectorOld = g_pixy_1_vectors_processing.getRightVector();

      vectors.resize(g_pixy_1.line.numVectors);
      for (size_t i = 0; i < g_pixy_1.line.numVectors; i++) {
        vectors[i] = VectorsProcessing::mirrorVector(mirrorLine, g_pixy_1.line.vectors[i]);
      }
      
      #if ENABLE_PIXY_VECTOR_APPROXIMATION == 1
      if(g_emergency_break_active == 0 && g_enable_pixy_vector_approximation != 0){
        if (((int)g_pixy_1_vectors_processing.isVectorValid(pixy_1_rightVectorOld) + (int)g_pixy_1_vectors_processing.isVectorValid(pixy_1_leftVectorOld))==1){
          if (g_emergency_break_active == 0){
            g_car_speed_mps = (float)g_vehicle_min_speed_mps;
          }
          #if ENABLE_DRIVERMOTOR == 1
            if (g_enable_car_engine != 0) {
              ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                g_powertrain.SetSpeedRequest_slow(g_car_speed_mps, VectorUnitToMeter(purePersuitInfo.turnRadius), SteeringWheel::AngleToDirectionDeg(g_steering_angle_rad), g_max_acceleration, g_max_deceleration);
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
      FailureModeMessage(&g_pixy_1, pixy_1_loopIterationsCountNoVectorDetected,"pixy getAllFeatures");
    }
/*===================================================END first camera============================================================================*/

    #if ENABLE_EMERGENCY_BREAKING == 1   // handling emergency braking
      AEB_out = automatic_emergency_braking();
    #endif
    #if ENABLE_FINISH_LINE_DETECTION == 1
      FLD_out = finish_line_detection(&vectors);       
    #endif


    g_middle_lane_line_pixy_1 = g_pixy_1_vectors_processing.getMiddleLine();
    lookAheadDistance = CalculateLookAheadDistance(MeterToVectorUnit(g_lookahead_min_distance_cm/100.0f), MeterToVectorUnit(g_lookahead_max_distance_cm/100.0f), g_middle_lane_line_pixy_1);
    if (!isValidFloatNumber(&lookAheadDistance, __LINE__)) {
      continue;
    }
    
    centerRearAxeCarPosition_vectorUnit.x = carPosition.x;
    centerRearAxeCarPosition_vectorUnit.y = carPosition.y - MeterToVectorUnit(WHEEL_BASE_M);
    purePersuitInfo = purePursuitComputeABC(centerRearAxeCarPosition_vectorUnit, g_middle_lane_line_pixy_1, (float)MeterToVectorUnit(WHEEL_BASE_M), lookAheadDistance);
    if (!isValidFloatNumber(&(purePersuitInfo.steeringAngle), __LINE__)) {
      continue;
    }

    #if ENABLE_STEERING_SERVO == 1
      g_steering_angle_rad = radians(g_steering_wheel.vaildAngleDeg(degrees(purePersuitInfo.steeringAngle)));
    #elif ENABLE_REAR_STEERING_ONLY == 1
      g_steering_angle_rad =  radians(VALIDATE_REAR_STEERING_ANGLE(degrees(purePersuitInfo.steeringAngle)));
    #else
      g_steering_angle_rad = purePersuitInfo.steeringAngle;
    #endif

    g_rear_axe_turn_radius_m = RearWheelTurnRadius(WHEEL_BASE_M, g_steering_angle_rad);
    if (!isValidFloatNumber(&(g_rear_axe_turn_radius_m), __LINE__)) {
      continue;
    }

    if (pixy_1_loopIterationsCountNoVectorDetected >= MAX_ITERATION_PIXY_ERROR)
    {
      #if ENABLE_SERIAL_PRINT != 0
        SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("No vector detected: ") + String(pixy_1_loopIterationsCountNoVectorDetected));
      #endif
      if (g_emergency_break_active == 0) {
        g_car_speed_mps = (float)g_vehicle_min_speed_mps;
      }
    }
    else{
      if (g_emergency_break_active == 0){        
        g_car_speed_mps = CalculateCarSpeed(g_vehicle_min_speed_mps, g_vehicle_max_speed_mps, WHEEL_BASE_M, g_friction_coefficient, g_downward_acceleration, g_steering_angle_rad);
      }
    }
    
    if (!isValidFloatNumber(&(g_car_speed_mps), __LINE__)) {
      continue;
    }

    #if ENABLE_STEERING_SERVO == 1
      if (g_enable_car_steering_wheel != 0) {
        g_steering_wheel.setSteeringWheelAngleDeg(degrees(g_steering_angle_rad));
      }
    #endif

    // max_speed Arbitrator
    speed_request_mps = g_vehicle_max_speed_original_mps;
    if (g_enable_emergency_brake != 0 && g_emergency_break_active != 0) {
      speed_request_mps = MIN(AEB_out.speed_request_mps, speed_request_mps);
    }
    if (g_enable_finish_line_detection != 0 && g_finish_line_detected_slowdown != 0) {
        speed_request_mps = MIN(FLD_out.speed_request_mps, speed_request_mps);
    }
    if(g_max_speed_delay_passed != 0){
      speed_request_mps = MIN(g_max_speed_after_delay_mps, speed_request_mps);
    }
    g_vehicle_max_speed_mps = speed_request_mps;
    

    #if ENABLE_DRIVERMOTOR == 1
      if (g_enable_car_engine != 0) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          g_powertrain.SetSpeedRequest_slow(g_car_speed_mps, g_rear_axe_turn_radius_m, SteeringWheel::AngleToDirectionDeg(degrees(g_steering_angle_rad)), g_max_acceleration, g_max_deceleration);
        }
      }
    #endif
    
    g_left_lane_line_pixy_1 = VectorsProcessing::vectorToLineABC(g_pixy_1_vectors_processing.getLeftVector());
    g_right_lane_line_pixy_1 = VectorsProcessing::vectorToLineABC(g_pixy_1_vectors_processing.getRightVector());

    g_loop_time_ms = ((float)millis()) - timeStart;
    g_loop_time_ms = MAX(g_loop_time_ms, 0.0f);
    g_time_passed_ms += g_loop_time_ms;

    #if ENABLE_SERIAL_PRINT == 1
        printDataToSerial(SERIAL_PORT, pixy_1_leftVectorOld, pixy_1_rightVectorOld, g_pixy_1_vectors_processing.getLeftVector(), g_pixy_1_vectors_processing.getRightVector(), VectorsProcessing::vectorToLineABC(g_pixy_1_vectors_processing.getLeftVector()), VectorsProcessing::vectorToLineABC(g_pixy_1_vectors_processing.getRightVector()), g_middle_lane_line_pixy_1, purePersuitInfo, AEB_out.obstacle_distance_m, g_car_speed_mps);
    #endif
  }
}



