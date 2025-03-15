#ifndef __SETUP_H__
#define __SETUP_H__

#include "GlobalVariables.h"
#include "features/automatic_emergency_braking.h"
#include "features/finish_line_detection.h"

#if ENABLE_SINGLE_AXE_STEERING_NO_RPM == 0
  #include "PowerTrain.h"
#else
  #include "OneMotorPowerTrain.h"
#endif

#define CAMERA_NO_VECTOR_DETECTED_TIMEOUT_S 0.1f
#define CAMERA_ERROR_TIMEOUT_S 0.1f

void FailureModeMessage(Pixy2 *pixy, float time_passed, String errorText);

int isValidFloatNumber(float *num, int line);

void setup();

#endif