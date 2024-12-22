
#include "features/finish_line_detection.h"

static unsigned int consecutiveValidFinishLines = 0;

void FLD_setup(){

}

void FLD_deactivate(){
    consecutiveValidFinishLines = 0;
    g_finish_line_detected_now = 0;
    g_finish_line_detected = 0;
    memset(&g_finish_line, 0, sizeof(g_finish_line));
}

FLD_out_t finish_line_detection(std::vector<Vector> *vectors){
    FLD_out_t out;

    out.active = (int8_t)0;
    out.speed_request_mps = 0.0f;
    out.detected_this_loop = (int8_t)0;

    EnableLineDetectionAfterDelay(&g_enable_finish_line_detection, g_enable_finish_line_detection_after_delay_s);
    out.enabled = g_enable_finish_line_detection;

    if (g_enable_finish_line_detection != 0) {
      g_finish_line = VectorsProcessing::findStartFinishLine((*vectors), g_pixy_1_vectors_processing.getLeftVector(), g_pixy_1_vectors_processing.getRightVector(), g_pixy_1_vectors_processing.getMiddleLine(), g_finish_line_angle_tolerance);
      if (VectorsProcessing::isFinishLineValid(g_finish_line)) {
        consecutiveValidFinishLines += 1;
        g_finish_line_detected_now = 1;
        if (consecutiveValidFinishLines >= CONSECUTUVE_FINISH_LINE_DETECTED_ACTIVATION) {
          g_finish_line_detected = 1;
          g_finish_line_detected_slowdown = 1;
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
    out.active = g_finish_line_detected_slowdown;
    out.detected_this_loop = g_finish_line_detected;
    }
    return out;
}