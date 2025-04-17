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


#include "log.h"
#include "GlobalVariables.h"
#include "WheelRpm.h"
#include "FloatToString.h"
#include "features/imu_data.h"

#if ENABLE_SINGLE_AXE_STEERING_NO_RPM == 0
  #include "PowerTrain.h"
#else
  #include "OneMotorPowerTrain.h"
#endif

void serialprint_linesegment(SERIAL_PORT_TYPE &serialPort, LineSegment seg, String separator){
  serialPort.print(FloatToString(seg.A.x, 1));
  serialPort.print(separator);
  serialPort.print(FloatToString(seg.A.y, 1));
  serialPort.print(separator);
  serialPort.print(FloatToString(seg.B.x, 1));
  serialPort.print(separator);
  serialPort.print(FloatToString(seg.B.y, 1));
}

void printDataToSerial(SERIAL_PORT_TYPE &serialPort, LineSegment leftVectorOld, LineSegment rightVectorOld, LineSegment leftVector, LineSegment rightVector, LineABC leftLine, LineABC rightLine, LineABC laneMiddleLine, PurePursuitInfo purePersuitInfo, float frontObstacleDistance, float carSpeed_){
  String commaCharStr;
  char semicolonChar;
  RpmSensorData temp_rpmData;
  float adjusted_rpm, raw_rpm;
  int n_decimals = 3;

  float carAcceleration;
  if (floatCmp((float)(g_vehicle_max_speed_mps - STANDSTILL_SPEED), 0.0f) != 0) {
      carAcceleration  = (g_car_speed_mps - (float)STANDSTILL_SPEED) / (float)(g_vehicle_max_speed_mps - STANDSTILL_SPEED);
  }
  else{
    carAcceleration = 0.0f;
  }


  commaCharStr = String(',');
  semicolonChar = ';';
  serialprint_linesegment(serialPort, leftVectorOld, commaCharStr);
  serialPort.print(semicolonChar);
  serialprint_linesegment(serialPort, rightVectorOld, commaCharStr);
  serialPort.print(semicolonChar);
  serialprint_linesegment(serialPort, leftVector, commaCharStr);
  serialPort.print(semicolonChar);
  serialprint_linesegment(serialPort, rightVector, commaCharStr);
  serialPort.print(semicolonChar);
  serialPort.print(FloatToString(leftLine.Ax, n_decimals) + commaCharStr + FloatToString(leftLine.By, n_decimals) + commaCharStr + FloatToString(leftLine.C, n_decimals));
  serialPort.print(semicolonChar);
  serialPort.print(FloatToString(rightLine.Ax, n_decimals) + commaCharStr + FloatToString(rightLine.By, n_decimals) + commaCharStr + FloatToString(rightLine.C, n_decimals));
  serialPort.print(semicolonChar);
  serialPort.print(FloatToString(laneMiddleLine.Ax, n_decimals) + commaCharStr + FloatToString(laneMiddleLine.By, n_decimals) + commaCharStr + FloatToString(laneMiddleLine.C, n_decimals));
  serialPort.print(semicolonChar);
  serialPort.print(FloatToString(purePersuitInfo.frontAxePosition.x, n_decimals) + commaCharStr + FloatToString(purePersuitInfo.frontAxePosition.y, n_decimals));
  serialPort.print(semicolonChar);
  serialPort.print(FloatToString(purePersuitInfo.nextWayPoint.x, n_decimals) + commaCharStr + FloatToString(purePersuitInfo.nextWayPoint.y, n_decimals));
  serialPort.print(semicolonChar);
  serialPort.print(FloatToString(g_steering_angle_rad, n_decimals));
  serialPort.print(semicolonChar);
  serialPort.print(FloatToString(carAcceleration, n_decimals));
  serialPort.print(semicolonChar);
  serialPort.print(FloatToString(frontObstacleDistance, 3));
  serialPort.print(semicolonChar);
  serialPort.print(FloatToString(VectorUnitToMeter(purePersuitInfo.lookAheadDistance) * 100.0f, n_decimals));
  serialPort.print(semicolonChar);
  serialPort.print(FloatToString(carSpeed_, n_decimals));
  serialPort.print(semicolonChar);
  serialPort.print(String(g_finish_line_detected));
  serialPort.print(semicolonChar);



  serialprint_linesegment(serialPort, g_finish_line.leftSegment, commaCharStr);
  serialPort.print(semicolonChar);
  serialprint_linesegment(serialPort, g_finish_line.rightSegment, commaCharStr);
  serialPort.print(semicolonChar);
  serialPort.print(String(g_finish_line_detected_now));
  serialPort.print(semicolonChar);
  serialPort.print(FloatToString(g_loop_time_ms, n_decimals));


  #if ENABLE_SINGLE_AXE_STEERING_NO_RPM == 0

  temp_rpmData = getLeftWheelRpmData();
  adjusted_rpm = getCurrentRpm_adjusted(&temp_rpmData);
  serialPort.print(semicolonChar);
  serialPort.print(FloatToString(temp_rpmData.Rpm, n_decimals));
  serialPort.print(semicolonChar);
  serialPort.print(FloatToString(adjusted_rpm, n_decimals));

  temp_rpmData = getRightWheelRpmData();
  adjusted_rpm = getCurrentRpm_adjusted(&temp_rpmData);
  serialPort.print(semicolonChar);
  serialPort.print(FloatToString(temp_rpmData.Rpm, n_decimals));
  serialPort.print(semicolonChar);
  serialPort.print(FloatToString(adjusted_rpm, n_decimals));


ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
  serialPort.print(semicolonChar);
  serialPort.print(FloatToString(g_powertrain.GetLeftWheelSpeedRequest_raw(), n_decimals));
  serialPort.print(semicolonChar);
  serialPort.print(FloatToString(g_powertrain.GetRightWheelSpeedRequest_raw(), n_decimals));
  serialPort.print(semicolonChar);
  serialPort.print(FloatToString(g_powertrain.GetLeftWheelSpeedRequest(), n_decimals));
  serialPort.print(semicolonChar);
  serialPort.print(FloatToString(g_powertrain.GetRightWheelSpeedRequest(), n_decimals));

  serialPort.print(semicolonChar);
  serialPort.print(FloatToString(g_powertrain.GetLeftWheelSpeed(), n_decimals));
  serialPort.print(semicolonChar);
  serialPort.print(FloatToString(g_powertrain.GetRightWheelSpeed(), n_decimals));
}
#else
serialPort.print(semicolonChar);
serialPort.print("0.0");
serialPort.print(semicolonChar);
serialPort.print("0.0");

serialPort.print(semicolonChar);
serialPort.print("0.0");
serialPort.print(semicolonChar);
serialPort.print("0.0");

serialPort.print(semicolonChar);
serialPort.print(FloatToString(g_onemotorpowertrain.GetSpeedRequest_raw(), 2));
serialPort.print(semicolonChar);
serialPort.print("0.0");

serialPort.print(semicolonChar);
serialPort.print("0.0");
serialPort.print(semicolonChar);
serialPort.print(FloatToString(g_onemotorpowertrain.GetSpeedRequest(), 2));

serialPort.print(semicolonChar);
serialPort.print("0.0");
serialPort.print(semicolonChar);
serialPort.print("0.0");
#endif

  serialPort.print(semicolonChar);
  serialPort.print(FloatToString(g_line_calibration_data.angle_offset, n_decimals));
  serialPort.print(semicolonChar);
  serialPort.print(FloatToString(g_line_calibration_data.rotation_point.x, n_decimals));
  serialPort.print(semicolonChar);
  serialPort.print(FloatToString(g_line_calibration_data.rotation_point.y, n_decimals));
  serialPort.print(semicolonChar);
  serialPort.print(FloatToString(g_line_calibration_data.x_axis_offset, n_decimals));
  serialPort.print(semicolonChar);
  serialPort.print(FloatToString(g_line_calibration_data.y_axis_offset, n_decimals));

  #ifdef ENABLE_IMU != 0
  serialPort.print(semicolonChar);
  serialPort.print(FloatToString(imu_get_yaw_rate_rad_s(), n_decimals));
  #else
  serialPort.print(semicolonChar);
  serialPort.print("0.0");
  #endif

  serialPort.println();
}