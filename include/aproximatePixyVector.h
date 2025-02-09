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

#ifndef __APPROXIMATEPIXYVECTOT_H__
#define __APPROXIMATEPIXYVECTOT_H__

#define IMAGE_MAX_X_VECTOR_UNIT 78.0f
#define IMAGE_MAX_Y_VECTOR_UNIT 51.0f

#include "rgb2hsv.h"
#include "VectorsProcessing.h"
#include "PurePursuitGeometry.h"
#include "pixy2_libs/host/arduino/libraries/Pixy2/Pixy2.h"
#include<pixy2_libs/host/arduino/libraries/Pixy2/Pixy2SPI_SS.h>

Vector vectorApproximationSelectionLogic(Vector vec, Point2D approximatedPointFound, Point2D carPosition);
// return 0 on success
int approximatePixyVectorVector(Pixy2& pixy, Vector& vec, float blackTreshold, Point2D carPosition);

#endif
