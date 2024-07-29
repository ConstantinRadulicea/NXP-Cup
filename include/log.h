#ifndef __LOG_H__
#define __LOG_H__

#include <HardwareSerial.h>
#include <vector>
#include "geometry2D.h"
#include "PurePursuitGeometry.h"
#include "GlobalVariables.h"

void printDataToSerial(HardwareSerial &serialPort, Vector leftVectorOld, Vector rightVectorOld, Vector leftVector, Vector rightVector, LineABC leftLine, LineABC rightLine, LineABC laneMiddleLine, PurePursuitInfo purePersuitInfo, float carAcceleration, float frontObstacleDistance, float carSpeed_);


#endif