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

void printDataToSerial(HardwareSerial &serialPort, Vector leftVectorOld, Vector rightVectorOld, Vector leftVector, Vector rightVector, LineABC leftLine, LineABC rightLine, LineABC laneMiddleLine, PurePursuitInfo purePersuitInfo, float carAcceleration, float frontObstacleDistance, float carSpeed_){
  String commaCharStr;
  char semicolonChar;
  RpmSensorData temp_rpmData;
  float adjusted_rpm, raw_rpm;

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
  serialPort.print(String(leftLine.Ax) + commaCharStr + String(leftLine.By) + commaCharStr + String(leftLine.C));
  serialPort.print(semicolonChar);
  serialPort.print(String(rightLine.Ax) + commaCharStr + String(rightLine.By) + commaCharStr + String(rightLine.C));
  serialPort.print(semicolonChar);
  serialPort.print(String(laneMiddleLine.Ax) + commaCharStr + String(laneMiddleLine.By) + commaCharStr + String(laneMiddleLine.C));
  serialPort.print(semicolonChar);
  serialPort.print(String(purePersuitInfo.carPos.x) + commaCharStr + String(purePersuitInfo.carPos.y));
  serialPort.print(semicolonChar);
  serialPort.print(String(purePersuitInfo.nextWayPoint.x) + commaCharStr + String(purePersuitInfo.nextWayPoint.y));
  serialPort.print(semicolonChar);
  serialPort.print(String(purePersuitInfo.steeringAngle));
  serialPort.print(semicolonChar);
  serialPort.print(String(carAcceleration));
  serialPort.print(semicolonChar);
  serialPort.print(String(frontObstacleDistance, 3));
  serialPort.print(semicolonChar);
  serialPort.print(String(VectorUnitToMeter(purePersuitInfo.lookAheadDistance) * 100.0f));
  serialPort.print(semicolonChar);
  serialPort.print(String(carSpeed_));
  serialPort.print(semicolonChar);
  serialPort.print(String(g_finish_line_detected));
  serialPort.print(semicolonChar);
  serialPort.print(String(g_finish_line.leftSegment.m_x0) + commaCharStr + String(g_finish_line.leftSegment.m_y0) + commaCharStr + String(g_finish_line.leftSegment.m_x1) + commaCharStr + String(g_finish_line.leftSegment.m_y1));
  serialPort.print(semicolonChar);
  serialPort.print(String(g_finish_line.rightSegment.m_x0) + commaCharStr + String(g_finish_line.rightSegment.m_y0) + commaCharStr + String(g_finish_line.rightSegment.m_x1) + commaCharStr + String(g_finish_line.rightSegment.m_y1));
  serialPort.print(semicolonChar);
  serialPort.print(String(g_finish_line_detected_now));
  serialPort.print(semicolonChar);
  serialPort.print(String(g_loop_time_ms));

  temp_rpmData = getLeftWheelRpmData();
  adjusted_rpm = getCurrentRpm_adjusted(&temp_rpmData);
  serialPort.print(semicolonChar);
  serialPort.print(String(temp_rpmData.Rpm));
  serialPort.print(semicolonChar);
  serialPort.print(String(adjusted_rpm));

  temp_rpmData = getRightWheelRpmData();
  adjusted_rpm = getCurrentRpm_adjusted(&temp_rpmData);
  serialPort.print(semicolonChar);
  serialPort.print(String(temp_rpmData.Rpm));
  serialPort.print(semicolonChar);
  serialPort.print(String(adjusted_rpm));


  serialPort.println();
}