
#include "features/finish_line_detection.h"



static unsigned int consecutiveValidFinishLines = 0;
static int local_detected_line_before = 0;
static float timer_remaining_s = 0.5f;

void FLD_setup(){

}

void FLD_deactivate(){
    consecutiveValidFinishLines = 0;
    g_finish_line_detected_now = 0;
    g_finish_line_detected = 0;
    g_finish_line_detected_slowdown = 0;
    local_detected_line_before = 0;
    timer_remaining_s = 0.5f;
    memset(&g_finish_line, 0, sizeof(g_finish_line));
}

FLD_out_t finish_line_detection(std::vector<LineSegment> *vectors, LineSegment left_line, LineSegment right_line){
    FLD_out_t out;
    

    out.active = (int8_t)0;
    out.speed_request_mps = 0.0f;
    out.detected_this_loop = (int8_t)0;

    EnableLineDetectionAfterDelay(&g_enable_finish_line_detection, g_enable_finish_line_detection_after_delay_s);
    out.enabled = g_enable_finish_line_detection;

    if (g_enable_finish_line_detection != 0 && g_finish_line_detected_slowdown == 0) {
      g_finish_line = VectorsProcessing::findStartFinishLine((*vectors), left_line, right_line, g_pixy_1_vectors_processing.getMiddleLine(), g_finish_line_angle_tolerance);
      if (VectorsProcessing::isFinishLineValid(g_finish_line)) {
        consecutiveValidFinishLines += 1;
        g_finish_line_detected_now = 1;
        if (consecutiveValidFinishLines >= CONSECUTUVE_FINISH_LINE_DETECTED_ACTIVATION) {
          g_finish_line_detected = 1;
          local_detected_line_before = 1;
          //g_finish_line_detected_slowdown = 1;
          out.speed_request_mps = g_max_speed_after_finish_line_detected_mps;
        }
      }
      else if (g_finish_line_detected == 0) {
        FLD_deactivate();
      }
      else{
        consecutiveValidFinishLines = 0;
        g_finish_line_detected_now = 0;
        memset(&g_finish_line, 0, sizeof(g_finish_line));
      }
    }
    if (local_detected_line_before != 0 && g_finish_line_detected_now != 1)
    {
      timer_remaining_s -= (g_loop_time_ms / 1000.0f);;
      if (timer_remaining_s <= 0.0f)
      {
        g_finish_line_detected_slowdown = 1;
      }
    }
    
    out.active = g_finish_line_detected_slowdown;
    out.detected_this_loop = g_finish_line_detected;
    if (out.active) {
      out.speed_request_mps = g_max_speed_after_finish_line_detected_mps;
    }
    return out;
}