#ifndef __SETUP_H__
#define __SETUP_H__

#include "Config.h"
#include "features/automatic_emergency_braking.h"
#include "features/finish_line_detection.h"

#define CAMERA_NO_VECTOR_DETECTED_TIMEOUT_S 0.2f
#define CAMERA_ERROR_TIMEOUT_S 0.2f

void FailureModeMessage(Pixy2 *pixy, float time_passed, String errorText);

int isValidFloatNumber(float *num, int line);

void setup();

#endif