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
#include "WheelRpm.h"

/*====================================================================================================================================*/

void loop() {
  size_t i;
  int8_t pixy_1_result;
  LineABC mirrorLine;
  Vector vec, pixy_1_leftVector, pixy_1_rightVector, calibrated_vector;
  Vector pixy_1_leftVectorOld, pixy_1_rightVectorOld;
  std::vector<Vector> uncalibrated_vectors;
  std::vector<Vector> calibrated_vectors;
  std::vector<Intersection> intersections;
  PurePursuitInfo purePersuitInfo;
  Point2D carPosition;
  Point2D centerRearAxeCarPosition_vectorUnit;
  float lookAheadDistance;
  float temp_time;
  float timeStart;
  float speed_request_mps;
  float p_camera_no_vector_detected_stopwatch_s = 0.0f;
  float p_camera_error_stopwatch_s = 0.0f;
  float local_unvalidated_steering_angle_rad;
  g_loop_time_ms = 0.0f;

  AEB_out_t AEB_out;
  FLD_out_t FLD_out;

  memset(&AEB_out, 0, sizeof(AEB_out_t));
  memset(&FLD_out, 0, sizeof(FLD_out_t));

  pixy_1_result = PIXY_RESULT_ERROR;

  timeStart = 0.0f;

  mirrorLine = xAxisABC();
  mirrorLine.C = -(SCREEN_CENTER_Y);

  carPosition.x = (float)SCREEN_CENTER_X;
  carPosition.y = 0.0f;

  g_middle_lane_line_pixy_1 = yAxisABC();
  
  lookAheadDistance = (float)MeterToVectorUnit(g_lookahead_min_distance_cm/100.0f);
  
  g_pixy_1_vectors_processing.setCarPosition(carPosition);
  g_pixy_1_vectors_processing.setLaneWidth((float)g_lane_width_vector_unit);
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
      parseInputGlobalVariablesRoutine_optimized(SERIAL_PORT);
      //parseInputGlobalVariablesRoutine(SERIAL_PORT);
    #endif

    #if ENABLE_SETTINGS_MENU == 1
      settingsMenuRoutine();
    #endif
    
    carPosition.y = - MeterToVectorUnit(g_camera_offset_y_m);
    g_pixy_1_vectors_processing.setCarPosition(carPosition);
    g_pixy_1_vectors_processing.setLaneWidth((float)g_lane_width_vector_unit);
    g_pixy_1_vectors_processing.setMinXaxisAngle(radians(g_min_x_axis_angle_vector_deg));

    if (g_enable_car_engine == 0) {
      FLD_deactivate();
      #if ENABLE_DRIVERMOTOR == 1
        #if ENABLE_SINGLE_AXE_STEERING_NO_RPM != 0
          g_onemotorpowertrain.SetSpeedRequest_slow(STANDSTILL_SPEED);
        #else
          ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            g_powertrain.SetSpeedRequest_slow(STANDSTILL_SPEED, 0.0, 0, g_max_acceleration, g_max_deceleration);
          }
        #endif
      #endif
    }

    #if ENABLE_STEERING_SERVO == 1
      if (g_enable_car_steering_wheel == 0) {
        g_steering_wheel.setSteeringWheelAngleDeg(0.0f);
      }

      if (g_enable_car_steering_wheel == 0 && g_enable_car_engine != 0) {
        g_enable_car_steering_wheel = 1;
      }
    #endif

    g_pixy_1_vectors_processing.clear();
    pixy_1_result = g_pixy_1.line.getAllFeatures(LINE_VECTOR /*| LINE_INTERSECTION*/, true);
    
/*===================================================START first camera============================================================================*/
    if(pixy_1_result >= ((int8_t)0)){
      p_camera_error_stopwatch_s = 0.0f;
      uncalibrated_vectors.resize(g_pixy_1.line.numVectors);
      memcpy(uncalibrated_vectors.data(), g_pixy_1.line.vectors, (g_pixy_1.line.numVectors * sizeof(Vector)));

      intersections.clear();
      VectorsProcessing::findIntersections(uncalibrated_vectors, intersections);
      VectorsProcessing::filterVectorIntersections(uncalibrated_vectors, intersections);

      if (uncalibrated_vectors.size() > 0){
        p_camera_no_vector_detected_stopwatch_s = 0.0f;
      }
      else{
        p_camera_no_vector_detected_stopwatch_s += MillisToSec(g_loop_time_ms);
      }

      calibrated_vectors.resize(uncalibrated_vectors.size());
      for (i=0; i < uncalibrated_vectors.size(); i++)
      {
        vec = uncalibrated_vectors[i];
        vec = VectorsProcessing::mirrorVector(mirrorLine, vec);
        vec = VectorsProcessing::reComputeVectorStartEnd_basedOnDistanceOfPointXaxis(vec, carPosition);
        calibrated_vector = vec;

        if (g_birdeye_calibrationdata.valid && g_start_line_calibration_acquisition == 0){
          calibrated_vector = BirdEye_CalibrateVector(g_birdeye_calibrationdata, calibrated_vector);
        }
        if (g_start_line_calibration_acquisition == 0) {
          calibrated_vector = calibrateVector(calibrated_vector, g_line_calibration_data);
        }
        
        calibrated_vectors[i] = calibrated_vector;
        g_pixy_1_vectors_processing.addVector(calibrated_vector);
      }
    }
    else{
      p_camera_no_vector_detected_stopwatch_s += MillisToSec(g_loop_time_ms);
      p_camera_error_stopwatch_s += MillisToSec(g_loop_time_ms);
      FailureModeMessage(&g_pixy_1, p_camera_error_stopwatch_s,"pixy getAllFeatures");
    }
/*===================================================END first camera============================================================================*/

pixy_1_leftVectorOld = pixy_1_leftVector;
pixy_1_rightVectorOld = pixy_1_rightVector;

pixy_1_leftVector = g_pixy_1_vectors_processing.getLeftVector();
pixy_1_rightVector = g_pixy_1_vectors_processing.getRightVector();



    //if (g_birdeye_calibrationdata.valid && g_start_line_calibration_acquisition_birdeye == 0){
    //  pixy_1_leftVectorOld = BirdEye_CalibrateVector(g_birdeye_calibrationdata, pixy_1_leftVectorOld);
    //  pixy_1_rightVectorOld = BirdEye_CalibrateVector(g_birdeye_calibrationdata, pixy_1_rightVectorOld);
    //}
    //if (g_start_line_calibration_acquisition == 0) {
    //  pixy_1_leftVectorOld = calibrateVector(pixy_1_leftVectorOld, g_line_calibration_data);
    //  pixy_1_rightVectorOld = calibrateVector(pixy_1_rightVectorOld, g_line_calibration_data);
    //}



    g_left_lane_segment = VectorsProcessing::vectorToLineSegment(pixy_1_leftVector);
    g_right_lane_segment = VectorsProcessing::vectorToLineSegment(pixy_1_rightVector);
    g_middle_lane_line_pixy_1 = g_pixy_1_vectors_processing.getMiddleLine();
    


    #if ENABLE_FINISH_LINE_DETECTION == 1
      FLD_out = finish_line_detection(&calibrated_vectors, pixy_1_leftVector, pixy_1_rightVector);
      if (FLD_out.active != 0) {
        g_enable_emergency_brake = 1;
        g_enable_change_aeb_max_distance_after_delay_passed = 1;
      }

    #endif
    #if ENABLE_EMERGENCY_BREAKING == 1   // handling emergency braking
      AEB_out = automatic_emergency_braking();
    #endif


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
      #if ENABLE_SINGLE_AXE_STEERING == 1
        g_steering_angle_rad =  radians(VALIDATE_REAR_STEERING_ANGLE(degrees(purePersuitInfo.steeringAngle)));
        local_unvalidated_steering_angle_rad = purePersuitInfo.steeringAngle;
      #elif ENABLE_REAR_AXE_STEERING == 1
      g_steering_angle_rad = radians(g_steering_wheel.vaildAngleDeg(degrees(-(purePersuitInfo.steeringAngle))));
      local_unvalidated_steering_angle_rad = -(purePersuitInfo.steeringAngle);
      #else
        g_steering_angle_rad = radians(g_steering_wheel.vaildAngleDeg(degrees(purePersuitInfo.steeringAngle)));
        local_unvalidated_steering_angle_rad = purePersuitInfo.steeringAngle;
      #endif
    #else
        g_steering_angle_rad = purePersuitInfo.steeringAngle;
        local_unvalidated_steering_angle_rad = purePersuitInfo.steeringAngle;
    #endif


    //SERIAL_PORT.println("% " + String(g_steering_wheel.vaildAngleDeg(degrees(purePersuitInfo.steeringAngle))));

    g_rear_axe_turn_radius_m = RearWheelTurnRadius(WHEEL_BASE_M, g_steering_angle_rad);
    if (!isValidFloatNumber(&(g_rear_axe_turn_radius_m), __LINE__)) {
      continue;
    }


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


    if (p_camera_no_vector_detected_stopwatch_s >= CAMERA_NO_VECTOR_DETECTED_TIMEOUT_S)
    {
      #if ENABLE_SERIAL_PRINT != 0 || ENABLE_SERIAL_PRINT_LIMITED != 0
        SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("No vector detected: ") + FloatToString(p_camera_no_vector_detected_stopwatch_s, 2) + String(" s"));
      #endif
      g_vehicle_max_speed_mps = MIN(g_vehicle_min_speed_mps, g_vehicle_max_speed_mps);
    }
    
    g_car_speed_mps = CalculateCarSpeed(g_vehicle_min_speed_mps, g_vehicle_max_speed_mps, WHEEL_BASE_M, g_friction_coefficient, g_downward_acceleration, local_unvalidated_steering_angle_rad);
    
    if (!isValidFloatNumber(&(g_car_speed_mps), __LINE__)) {
      continue;
    }

    #if ENABLE_STEERING_SERVO == 1
      if (g_enable_car_steering_wheel != 0) {
        g_steering_wheel.setSteeringWheelAngleDeg(degrees(g_steering_angle_rad));
      }
    #endif


    #if ENABLE_DRIVERMOTOR == 1
      if (g_enable_car_engine != 0) {
        #if ENABLE_SINGLE_AXE_STEERING_NO_RPM != 0
          g_onemotorpowertrain.SetSpeedRequest_slow(g_car_speed_mps);
        #else
          ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            g_powertrain.SetSpeedRequest_slow(g_car_speed_mps, g_rear_axe_turn_radius_m, SteeringWheel::AngleToDirectionDeg(degrees(g_steering_angle_rad)), g_max_acceleration, g_max_deceleration);
          }
        #endif
      }
    #endif
    
    g_left_lane_line_pixy_1 = lineSegmentToLineABC(g_left_lane_segment);
    g_right_lane_line_pixy_1 = lineSegmentToLineABC(g_right_lane_segment);

    #if (ENABLE_SERIAL_PRINT != 0 || ENABLE_SERIAL_PRINT_LIMITED != 0) && defined(TEENSYLC)
      //SERIAL_PORT.println(String(ESCAPED_CHARACTER_AT_BEGINNING_OF_STRING) + String("ms:") + FloatToString(g_loop_time_ms, 0));
    #endif

    #if ENABLE_SERIAL_PRINT == 1
        printDataToSerial(SERIAL_PORT, pixy_1_leftVectorOld, pixy_1_rightVectorOld, pixy_1_leftVector, pixy_1_rightVector, VectorsProcessing::vectorToLineABC(g_pixy_1_vectors_processing.getLeftVector()), VectorsProcessing::vectorToLineABC(g_pixy_1_vectors_processing.getRightVector()), g_middle_lane_line_pixy_1, purePersuitInfo, AEB_out.obstacle_distance_m, g_car_speed_mps);
    #endif

    temp_time = (float)millis();
    if (temp_time < timeStart){
      timeStart = temp_time;
      temp_time = 0.0f;
    }
    
    g_loop_time_ms = temp_time - timeStart;
    g_loop_time_ms = MAX(g_loop_time_ms, 0.0f);
    g_time_passed_ms += g_loop_time_ms;
  }
}



