#ifndef __LOG_H__
#define __LOG_H__

#include <HardwareSerial.h>
#include <vector>
#include "geometry2D.h"
#include "PurePursuitGeometry.h"

static void printDataToSerial(HardwareSerial &serialPort, Vector leftVectorOld, Vector rightVectorOld, Vector leftVector, Vector rightVector, LineABC leftLine, LineABC rightLine, LineABC laneMiddleLine, PurePursuitInfo purePersuitInfo, float carAcceleration, float frontObstacleDistance, float carSpeed_){
  String commaCharStr;
  char semicolonChar;

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
  serialPort.print(String(frontObstacleDistance));
  serialPort.print(semicolonChar);
  serialPort.print(String(purePersuitInfo.lookAheadDistance * CM_PER_VECTOR_UNIT));
  serialPort.print(semicolonChar);
  serialPort.print(String(carSpeed_));
  serialPort.print(semicolonChar);
  serialPort.print(String(finish_line_detected));
  serialPort.print(semicolonChar);
  serialPort.print(String(finish_line.leftSegment.m_x0) + commaCharStr + String(finish_line.leftSegment.m_y0) + commaCharStr + String(finish_line.leftSegment.m_x1) + commaCharStr + String(finish_line.leftSegment.m_y1));
  serialPort.print(semicolonChar);
  serialPort.print(String(finish_line.rightSegment.m_x0) + commaCharStr + String(finish_line.rightSegment.m_y0) + commaCharStr + String(finish_line.rightSegment.m_x1) + commaCharStr + String(finish_line.rightSegment.m_y1));
  serialPort.print(semicolonChar);
  serialPort.print(String(finish_line_detected_now));
  serialPort.print(semicolonChar);
  serialPort.print(String(loop_time_ms));
  serialPort.println();
}



#endif