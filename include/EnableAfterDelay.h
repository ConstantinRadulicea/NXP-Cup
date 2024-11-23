#ifndef __ENABLEAFTERDELAY_H__
#define __ENABLEAFTERDELAY_H__

#include "geometry2D.h"
#include "GlobalVariables.h"

void EnableLineDetectionAfterDelay(int8_t* g_enable_finish_line_detection_ptr, float seconds);
void EnableEmergencyBrakeAfterDelay(int8_t* g_enable_finish_line_detection_ptr, float seconds);
void EnableSlowSpeedAfterDelay(int8_t* g_enable_finish_line_detection_ptr, float seconds);

#endif