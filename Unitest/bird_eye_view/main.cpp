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

LineSegment left_segment = LineSegment{ Point2D{4.0, 1.0}, Point2D{17.0, 44.0} };
LineSegment right_segment = LineSegment{ Point2D{63.0, 1.0}, Point2D{48.0, 44.0} };



int main() {
    struct track_widths src_track_widths, dst_track_widths;
    Point2D src[4] = { {4, 1}, {17, 44}, {50, 400}, {550, 400} };
    Point2D dst[4] = { {150, 0}, {450, 0}, {150, 400}, {450, 400} };

    
    src_track_widths = getTrackWidths(left_segment, right_segment, 51.0f);
    dst_track_widths = srcViewToRealView(src_track_widths, 0.52f, 78.0f);

    src[0] = src_track_widths.lower_segment.A;
    src[1] = src_track_widths.lower_segment.B;
    src[2] = src_track_widths.upper_segment.A;
    src[3] = src_track_widths.upper_segment.B;

    dst[0] = dst_track_widths.lower_segment.A;
    dst[1] = dst_track_widths.lower_segment.B;
    dst[2] = dst_track_widths.upper_segment.A;
    dst[3] = dst_track_widths.upper_segment.B;

    float H[3][3];
    getPerspectiveTransform(src, dst, H);

    printf("Perspective Transformation Matrix:\n");
    for (int i = 0; i < 3; i++) {
        printf("[%f %f %f]\n", H[i][0], H[i][1], H[i][2]);
    }

    Point2D testPoint = left_segment.A;
    Point2D transformed = applyPerspectiveTransform(H, testPoint);
    printf("Transformed (%.2f, %.2f) -> (%.2f, %.2f)\n", testPoint.x, testPoint.y, transformed.x, transformed.y);

    testPoint = left_segment.B;
    transformed = applyPerspectiveTransform(H, testPoint);
    printf("Transformed (%.2f, %.2f) -> (%.2f, %.2f)\n", testPoint.x, testPoint.y, transformed.x, transformed.y);

    testPoint = right_segment.A;
    transformed = applyPerspectiveTransform(H, testPoint);
    printf("Transformed (%.2f, %.2f) -> (%.2f, %.2f)\n", testPoint.x, testPoint.y, transformed.x, transformed.y);

    testPoint = right_segment.B;
    transformed = applyPerspectiveTransform(H, testPoint);
    printf("Transformed (%.2f, %.2f) -> (%.2f, %.2f)\n", testPoint.x, testPoint.y, transformed.x, transformed.y);

	return 0;
}

