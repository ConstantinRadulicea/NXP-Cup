#ifndef __SETUP_H__
#define __SETUP_H__

#include "Config.h"

#define MAX_ITERATION_PIXY_ERROR 50

void FailureModeMessage(Pixy2 *pixy, int iteration, String errorText);

int isValidFloatNumber(float *num, int line);

void setup();

#endif