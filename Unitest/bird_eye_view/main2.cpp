
#include "BirdEyeView.h"
#include <stdio.h>

#define LANE_WIDTH_M 0.535f

#define IMAGE_MAX_X 79.0f
#define IMAGE_MAX_Y 52.0f

static float g_line_image_frame_width = IMAGE_MAX_X;
static float g_line_image_frame_height = IMAGE_MAX_Y;
static float g_lane_width_vector_unit = 56.61f;

#define MeterToVectorUnit(m) ((float)(m) * ((float)g_lane_width_vector_unit *( 1.0f / (float)LANE_WIDTH_M)))
#define VectorUnitToMeter(v_unit) ((float)(v_unit) * ((float)LANE_WIDTH_M * (1.0f / (float)g_lane_width_vector_unit)))

static struct BirdEyeCalibrationData g_birdeye_calibrationdata = {};

void initialize_g_birdeye_calibrationdata() {
    struct track_widths temp_track_widths = {};
    temp_track_widths.upper_segment.A.x = 28.92f;   // FL x
    temp_track_widths.upper_segment.A.y = 51.78f;   // FL y
    temp_track_widths.upper_segment.B.x = 49.91f;   // FR x
    temp_track_widths.upper_segment.B.y = 52.21f;   // FR y
    temp_track_widths.lower_segment.A.x = 11.46f;   // RL x
    temp_track_widths.lower_segment.A.y = -0.6f;    // RL y
    temp_track_widths.lower_segment.B.x = 69.53f;   // RR x
    temp_track_widths.lower_segment.B.y = 0.6f;      // RR y


    temp_track_widths.lower_segment.A.x = 8.44f;   // RL x
    temp_track_widths.lower_segment.A.y = -0.78f;  // RL y
    temp_track_widths.lower_segment.B.x = 68.08f;  // RR x
    temp_track_widths.lower_segment.B.y = 0.70f;    // RR y
    temp_track_widths.upper_segment.A.x = 24.9f;   // FL x
    temp_track_widths.upper_segment.A.y = 51.68f;  // FL y
    temp_track_widths.upper_segment.B.x = 48.88f;  // FR x
    temp_track_widths.upper_segment.B.y = 52.32f;  // FR y

    temp_track_widths.lower_segment.A.x = 4.3;   // RL x
    temp_track_widths.lower_segment.A.y = 3.12;  // RL y
    temp_track_widths.lower_segment.B.x = 66.05;  // RR x
    temp_track_widths.lower_segment.B.y = -3.20;    // RR y
    temp_track_widths.upper_segment.A.x = 28.58;   // FL x
    temp_track_widths.upper_segment.A.y = 53.28;  // FL y
    temp_track_widths.upper_segment.B.x = 52.3;  // FR x
    temp_track_widths.upper_segment.B.y = 50.79;  // FR y

    g_birdeye_calibrationdata = CalculateBirdEyeCalibration_TrackWidths(temp_track_widths, g_line_image_frame_width, g_line_image_frame_height, LANE_WIDTH_M);
}



LineSegment BirdEye_CalibrateLineSegmentScaledToVector(struct BirdEyeCalibrationData calib_data, LineSegment seg) {
    LineSegment calibrated_seg;

    calibrated_seg = BirdEye_CalibrateLineSegment(calib_data, seg);

    calibrated_seg.A.x = MeterToVectorUnit(calibrated_seg.A.x);
    calibrated_seg.A.y = MeterToVectorUnit(calibrated_seg.A.y);
    calibrated_seg.B.x = MeterToVectorUnit(calibrated_seg.B.x);
    calibrated_seg.B.y = MeterToVectorUnit(calibrated_seg.B.y);

    return calibrated_seg;
}


/*

0.0093961423    -0.0046280171   -0.0036999662
0.0000005197    0.0093246028    -0.0000215750
0.0002529219    -0.0124073541   1.0000000000

*/


void main() {
    LineSegment calibrated_vector, uncalibrated_vector;
    initialize_g_birdeye_calibrationdata();

    uncalibrated_vector.A.x = 10;
    uncalibrated_vector.A.y = 2;

    uncalibrated_vector.B.x = 15;
    uncalibrated_vector.B.y = 30;


    calibrated_vector = BirdEye_CalibrateLineSegmentScaledToVector(g_birdeye_calibrationdata, uncalibrated_vector);

}