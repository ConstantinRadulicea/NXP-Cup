#ifndef __DISTANCESENSORS_H__
#define __DISTANCESENSORS_H__

#include <stdint.h>

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