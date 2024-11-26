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

void printDataToSerial(SERIAL_PORT_TYPE &serialPort, Vector leftVectorOld, Vector rightVectorOld, Vector leftVector, Vector rightVector, LineABC leftLine, LineABC rightLine, LineABC laneMiddleLine, PurePursuitInfo purePersuitInfo, float carAcceleration, float frontObstacleDistance, float carSpeed_){
  String commaCharStr;
  char semicolonChar;
  RpmSensorData temp_rpmData;
  float adjusted_rpm, raw_rpm;
  int n_decimals = 3;

  commaCharStr = String(',');
  semicolonChar = ';';

  serialPort.print(String(leftVectorOld.m_x0) + commaCharStr + String(leftVectorOld.m_y0) + commaCharStr + String(leftVectorOld.m_x1) + commaCharStr + String(leftVectorOld.m_y1));
  serialPort.print(semicolonChar);
  serialPort.print(String(rightVectorOld.m_x0) + commaCharStr + String(rightVectorOld.m_y0) + commaCharStr + String(rightVectorOld.m_x1) + commaCharStr + String(rightVectorOld.m_y1));
  serialPort.print(semicolonChar);
  serialPort.print(String(leftVector.m_x0) + commaCharStr + String(leftVector.m_y0) + commaCharStr + String(leftVector.m_x1) + commaCharStr + String(leftVector.m_y1));
  serialPort.print(semicolonChar);
  serialPort.print(String(rightVector.m_x0) + commaCharStr + String(rightVector.m_y0) + commaCharStr + String(rightVector.m_x1) + commaCharStr + String(rightVector.m_y1));
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
  serialPort.print(String(g_finish_line.leftSegment.m_x0) + commaCharStr + String(g_finish_line.leftSegment.m_y0) + commaCharStr + String(g_finish_line.leftSegment.m_x1) + commaCharStr + String(g_finish_line.leftSegment.m_y1));
  serialPort.print(semicolonChar);
  serialPort.print(String(g_finish_line.rightSegment.m_x0) + commaCharStr + String(g_finish_line.rightSegment.m_y0) + commaCharStr + String(g_finish_line.rightSegment.m_x1) + commaCharStr + String(g_finish_line.rightSegment.m_y1));
  serialPort.print(semicolonChar);
  serialPort.print(String(g_finish_line_detected_now));
  serialPort.print(semicolonChar);
  serialPort.print(FloatToString(g_loop_time_ms, n_decimals));

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
}

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

  serialPort.println();
}