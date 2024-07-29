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

#ifndef __DISTANCESENSORS_H__
#define __DISTANCESENSORS_H__

#include <stdint.h>
#include "Config.h"

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

#ifndef ENABLE_DISTANCE_SENSOR1
  #define ENABLE_DISTANCE_SENSOR1 0
#endif

#ifndef ENABLE_DISTANCE_SENSOR2
  #define ENABLE_DISTANCE_SENSOR2 0
#endif

#ifndef ENABLE_DISTANCE_SENSOR3
  #define ENABLE_DISTANCE_SENSOR3 0
#endif

void DistanceSensorsSetup(
int _distance_sensor1_trig_pin, int _distance_sensor1_echo_pin,
int _distance_sensor2_trig_pin, int _distance_sensor2_echo_pin,
int _distance_sensor3_trig_pin, int _distance_sensor3_echo_pin);


float getFrontObstacleDistance_cm();



#endif