#ifndef __FINISH_LINE_DETECTION_H__
#define __FINISH_LINE_DETECTION_H__

#include "GlobalVariables.h"
#include "EnableAfterDelay.h"

#define CONSECUTUVE_FINISH_LINE_DETECTED_ACTIVATION 30

typedef struct FLD_out_s{
int8_t enabled;
int8_t active;
int8_t detected_this_loop;
float speed_request_mps;
FinishLine g_finish_line;
}FLD_out_t;


void FLD_setup();

void FLD_deactivate();

FLD_out_t finish_line_detection(std::vector<Vector> *vectors);

#endif