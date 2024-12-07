
#ifndef __AUTOMATIC_EMERGENCY_BRAKING_H__
#define __AUTOMATIC_EMERGENCY_BRAKING_H__

#include "GlobalVariables.h"
#include "EnableAfterDelay.h"

typedef struct AEB_out_s{
int8_t enabled;
int8_t active;
float obstacle_distance_m;
float speed_request_mps;
unsigned int active_loops_count;
}AEB_out_t;

void AEB_setup();

AEB_out_t automatic_emergency_braking();


#endif