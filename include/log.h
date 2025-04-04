/*
* Copyright 2024 Constantin Dumitru Petre RĂDULICEA
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

#ifndef __LOG_H__
#define __LOG_H__

#include <HardwareSerial.h>
#include <vector>
#include "geometry2D.h"
#include "PurePursuitGeometry.h"
#include "GlobalVariables.h"

void printDataToSerial(SERIAL_PORT_TYPE &serialPort, LineSegment leftVectorOld, LineSegment rightVectorOld, LineSegment leftVector, LineSegment rightVector, LineABC leftLine, LineABC rightLine, LineABC laneMiddleLine, PurePursuitInfo purePersuitInfo, float frontObstacleDistance, float carSpeed_);


#endif