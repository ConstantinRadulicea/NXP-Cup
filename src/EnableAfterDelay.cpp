
#include "EnableAfterDelay.h"

void EnableAfterDelay(int8_t *enable_ptr, int* started_count, float* remaining_delay_s, float seconds){
    if (enable_ptr == NULL) {
      return;
    }
        if ((*enable_ptr) == 0 && floatCmp(seconds, 0.0f) > 0)
        {
          if ((*started_count) == 0 && g_enable_car_engine != 0) {
            (*remaining_delay_s) = seconds;
            (*started_count) = 1;
          }
          else if((*started_count) != 0 && g_enable_car_engine == 0){
            (*remaining_delay_s) = 0.0f;
            (*started_count) = 0;
          }
          
          if (g_enable_car_engine != 0 && floatCmp((*remaining_delay_s), 0.0f) > 0) {
            (*remaining_delay_s) -= (g_loop_time_ms / 1000.0f);
            (*remaining_delay_s) = MAX((*remaining_delay_s), 0.0f);

            if (floatCmp((*remaining_delay_s), 0.0f) <= 0) {
              (*enable_ptr) = 1;
            }
          }
        }
        else if (floatCmp(seconds, 0.0f) > 0 && g_enable_car_engine == 0)
        {
          (*enable_ptr) = 0;
          (*remaining_delay_s) = 0.0f;
          (*started_count) = 0;
        }
}


void EnableLineDetectionAfterDelay(int8_t* g_enable_finish_line_detection_ptr, float seconds){
    static int started_count = 0;
    static float remaining_delay_s = 0.0f;
    EnableAfterDelay(g_enable_finish_line_detection_ptr, &started_count, &remaining_delay_s, seconds);
}


void EnableEmergencyBrakeAfterDelay(int8_t* g_enable_finish_line_detection_ptr, float seconds){
    static int started_count = 0;
    static float remaining_delay_s = 0.0f;
    EnableAfterDelay(g_enable_finish_line_detection_ptr, &started_count, &remaining_delay_s, seconds);
}


void EnableSlowSpeedAfterDelay(int8_t* g_enable_finish_line_detection_ptr, float seconds){
    static int started_count = 0;
    static float remaining_delay_s = 0.0f;
 EnableAfterDelay(g_enable_finish_line_detection_ptr, &started_count, &remaining_delay_s, seconds);
}
