#include "BirdEyeView.h"
#include <stdio.h>


/*
 left_line:
    P1(4; 1)
    P2(17; 44)

 right_line:
    P1(63; 1)
    P2(48; 44)


 lower_size: 59
 upper_size: 31

*/

#define LANE_WIDTH_M 0.53

static LineSegment left_line_segment = LineSegment{ Point2D{4.0, 1.0}, Point2D{17.0, 44.0} };
static LineSegment right_line_segment = LineSegment{ Point2D{63.0, 1.0}, Point2D{48.0, 44.0} };

static float g_line_image_frame_width = 78.0f;
static float g_line_image_frame_height = 51.0f;

BirdEyeCalibrationData g_birdeye_calibrationdata;
int main2() {

    float *H[3][3];
    g_birdeye_calibrationdata = CalculateBirdEyeCalibration_lines(left_line_segment, right_line_segment, g_line_image_frame_width, g_line_image_frame_height, LANE_WIDTH_M);

    printf("Perspective Transformation Matrix:\n");
    for (int i = 0; i < 3; i++) {
        printf("[%f %f %f]\n", g_birdeye_calibrationdata.birdeye_transform_matrix[i][0], g_birdeye_calibrationdata.birdeye_transform_matrix[i][1], g_birdeye_calibrationdata.birdeye_transform_matrix[i][2]);
    }

    Point2D testPoint = left_line_segment.A;
    Point2D transformed = applyPerspectiveTransform(g_birdeye_calibrationdata.birdeye_transform_matrix, testPoint);
    printf("Transformed (%.2f, %.2f) -> (%.2f, %.2f)\n", testPoint.x, testPoint.y, transformed.x, transformed.y);

    testPoint = left_line_segment.B;
    transformed = applyPerspectiveTransform(g_birdeye_calibrationdata.birdeye_transform_matrix, testPoint);
    printf("Transformed (%.2f, %.2f) -> (%.2f, %.2f)\n", testPoint.x, testPoint.y, transformed.x, transformed.y);

    testPoint = right_line_segment.A;
    transformed = applyPerspectiveTransform(g_birdeye_calibrationdata.birdeye_transform_matrix, testPoint);
    printf("Transformed (%.2f, %.2f) -> (%.2f, %.2f)\n", testPoint.x, testPoint.y, transformed.x, transformed.y);

    testPoint = right_line_segment.B;
    transformed = applyPerspectiveTransform(g_birdeye_calibrationdata.birdeye_transform_matrix, testPoint);
    printf("Transformed (%.2f, %.2f) -> (%.2f, %.2f)\n", testPoint.x, testPoint.y, transformed.x, transformed.y);

	return 0;
}

